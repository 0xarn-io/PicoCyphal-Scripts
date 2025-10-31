#include <SPI.h>
#include <107-Arduino-Cyphal.h>
#include <107-Arduino-MCP2515.h>
#include <107-Arduino-CriticalSection.h>
#include <107-Arduino-Cyphal-Support.h>

using namespace uavcan::node;
using uavcan::si::unit::temperature::Scalar_1_0;

/*** HW pins (Waveshare RP2350 + XL2515 on SPI1) ***/
static constexpr uint8_t MCP2515_CS  = 9;   // GP9  -> CS
static constexpr uint8_t MCP2515_INT = 8;   // GP8  -> INT
static constexpr uint8_t LED_PIN     = 25;  // GP25 -> onboard LED

#define SPI_CAN SPI1

/********** Forward **********/
void onReceiveBufferFull(CanardFrame const & frame);

/********** MCP2515 **********/
ArduinoMCP2515 mcp2515(
  [](){ SPI_CAN.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0)); digitalWrite(MCP2515_CS, LOW); },
  [](){ digitalWrite(MCP2515_CS, HIGH); SPI_CAN.endTransaction(); },
  [](uint8_t d){ return SPI_CAN.transfer(d); },
  micros,
  millis,
  onReceiveBufferFull,
  nullptr
);

/********** Cyphal node **********/
cyphal::Node::Heap<cyphal::Node::DEFAULT_O1HEAP_SIZE> node_heap;
cyphal::Node node_hdl(
  node_heap.data(), node_heap.size(), micros,
  [](CanardFrame const & f){ return mcp2515.transmit(f); }
);

/********** Publishers **********/
auto heartbeat_pub = node_hdl.create_publisher<Heartbeat_1_0>(1 * 1000 * 1000ULL);   // fixed port (7509)
static constexpr CanardPortID TEMP_PORT_ID = 4901;
auto temp_pub      = node_hdl.create_publisher<Scalar_1_0>(TEMP_PORT_ID, 1 * 1000 * 1000ULL);  // subject-id overload

void setup() {
  Serial.begin(115200);

  // SPI1
  SPI_CAN.setSCK(10);
  SPI_CAN.setTX(11);
  SPI_CAN.setRX(12);
  SPI_CAN.begin();

  pinMode(MCP2515_CS, OUTPUT);
  digitalWrite(MCP2515_CS, HIGH);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(MCP2515_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MCP2515_INT),
                  [](){ mcp2515.onExternalEventHandler(); }, FALLING);

  // CAN @ 250 kbit/s, 16 MHz CAN clock
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);
  mcp2515.setNormalMode();

  node_hdl.setNodeId(42);

  // Node info (GetInfo.1.0)
  static const auto node_info = node_hdl.create_node_info(
    1, 0,   // protocol
    0, 1,   // hardware
    0, 2,   // software
    0,      // vcs rev
    cyphal::support::UniqueId::instance().value(),
    "Temperature_Node"
  );
}

void loop() {
  { CriticalSection g; node_hdl.spinSome(); }

  static unsigned long last_hb = 0;
  static unsigned long last_temp = 0;
  const unsigned long now = millis();

  // Heartbeat every 1 s
  if (now - last_hb > 1000) {
    last_hb = now;
    Heartbeat_1_0 hb;
    hb.uptime = now / 1000;
    hb.health.value = Health_1_0::NOMINAL;
    hb.mode.value   = Mode_1_0::OPERATIONAL;
    hb.vendor_specific_status_code = 0;
    heartbeat_pub->publish(hb);
  }

  // Temperature every 1 s (fake steady 25 °C)
  if (now - last_temp >= 1000) {
    last_temp = now;
    float t_c = readPT1000();
    Scalar_1_0 msg;
    msg.kelvin = t_c + 273.15f;
    Serial.print(F("Temp K: ")); Serial.println(msg.kelvin);
    temp_pub->publish(msg);
  }
}

void onReceiveBufferFull(CanardFrame const & frame) {
  node_hdl.onCanFrameReceived(frame);
  digitalWrite(LED_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(LED_PIN, LOW);
}

/**************************************************************************************
 * PT1000 READER (separate, simple, clean)
 **************************************************************************************/
float readPT1000()
{
  // ---- Config (adjust to your actual hardware) ----
  constexpr uint8_t  ADC_PIN = 27;     // GP27 (ADC1)
  constexpr float    VREF_V  = 3.3f;   // ADC reference/bus voltage
  constexpr uint16_t ADC_MAX = 4095;   // 12-bit ADC
  constexpr float    R_REF   = 10000.0f; // <-- likely 10k to 3V3 (change if needed)
  // IEC 60751 coefficients (PT100/1000 share A,B)
  constexpr float    A = 3.9083e-3f;
  constexpr float    B = -5.775e-7f;

  uint16_t raw = analogRead(ADC_PIN);

  // Convert ADC -> voltage with clamping away from 0 and Vref to avoid division by zero
  float v = (float)raw * VREF_V / ADC_MAX;
  if (v < 0.001f)         v = 0.001f;
  if (v > VREF_V - 0.001f) v = VREF_V - 0.001f;

  // LOW-SIDE divider: PT1000 to GND, R_REF to 3V3, sense at midpoint
  // V = Vref * (R_PT / (R_PT + R_REF))  ->  R_PT = (R_REF * V) / (Vref - V)
  float rpt = (R_REF * v) / (VREF_V - v);

  // Sanity range for PT1000 around room temp (optional, to catch wiring errors)
  if (rpt < 500.0f || rpt > 4000.0f) {
    return NAN;  // out of plausible range -> caller should ignore this sample
  }

  // For T >= 0°C: R = R0*(1 + A*T + B*T^2), R0=1000Ω -> quadratic in T
  float Rt_R0 = rpt / 1000.0f;
  if (Rt_R0 < 1.0f) {
    // likely <0°C; keep it simple: clamp to 0°C for now
    return 0.0f;
  }

  float disc = A*A - 4.0f*B*(1.0f - Rt_R0);
  if (disc < 0.0f) disc = 0.0f;              // avoid NaN
  float t_c = (-A + sqrtf(disc)) / (2.0f*B); // valid for T >= 0°C
  return t_c;
}
