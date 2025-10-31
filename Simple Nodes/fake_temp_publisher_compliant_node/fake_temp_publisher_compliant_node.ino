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
  if (now - last_temp > 1000) {
    last_temp = now;
    Scalar_1_0 t{};
    t.kelvin = 298.15F;   // 25 °C
    temp_pub->publish(t);
  }
}

void onReceiveBufferFull(CanardFrame const & frame) {
  node_hdl.onCanFrameReceived(frame);
  digitalWrite(LED_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(LED_PIN, LOW);
}
