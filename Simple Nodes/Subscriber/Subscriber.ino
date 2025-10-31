#include <SPI.h>
#include <107-Arduino-Cyphal.h>
#include <107-Arduino-MCP2515.h>
#include <107-Arduino-CriticalSection.h>
#include <107-Arduino-Cyphal-Support.h>  // For UniqueId support

using namespace uavcan::node;
using uavcan::si::unit::temperature::Scalar_1_0;

// Waveshare RP2350 CAN (XL2515 on SPI1)
static constexpr uint8_t MCP2515_CS  = 9;   // GP9  -> CS
static constexpr uint8_t MCP2515_INT = 8;   // GP8  -> INT
static constexpr uint8_t LED_PIN     = 25;  // GP25 -> onboard LED

#define SPI_CAN SPI1

/********** Forward declarations **********/
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
  [](const CanardFrame& f){ return mcp2515.transmit(f); }
);

/********** Subscribers **********/
constexpr CanardPortID TEMP_PORT_ID = 4901;
void onTemperature(const Scalar_1_0& msg /*, const CanardTransfer* meta */);
volatile float g_latest_temp_kelvin = NAN;
auto temp_sub = 
  node_hdl.create_subscription<Scalar_1_0>(TEMP_PORT_ID, onTemperature);


/********** Heartbeat publisher **********/
auto heartbeat_pub =
  node_hdl.create_publisher<Heartbeat_1_0>(1 * 1000 * 1000UL);

void setup() {
  Serial.begin(115200);

  // --- SPI1 setup ---
  SPI_CAN.setSCK(10);   // GP10 SCK
  SPI_CAN.setTX(11);    // GP11 MOSI
  SPI_CAN.setRX(12);    // GP12 MISO
  SPI_CAN.begin();

  pinMode(MCP2515_CS, OUTPUT);
  digitalWrite(MCP2515_CS, HIGH);

  // --- LED setup ---
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // --- INT setup ---
  pinMode(MCP2515_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MCP2515_INT),
                  [](){ mcp2515.onExternalEventHandler(); }, FALLING);

  // --- CAN config ---
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);
  mcp2515.setNormalMode();

  node_hdl.setNodeId(43);  // optional

  // --- Node Info using library (for GetInfo_1_0 service) ---
  static const auto node_info = node_hdl.create_node_info(
    /* uavcan.node.Version.1.0 protocol_version */
    1, 0,
    /* uavcan.node.Version.1.0 hardware_version */
    0, 1,
    /* uavcan.node.Version.1.0 software_version */
    0, 15,
    /* saturated uint64 software_vcs_revision_id */
    0,
    /* saturated uint8[16] unique_id */
    cyphal::support::UniqueId::instance().value(),
    /* saturated uint8[<=50] name */
    "Servo_Node"
  );
}

void loop() {
  { CriticalSection g; node_hdl.spinSome(); }

  // Heartbeat every 1 s
  static unsigned long prev = 0;
  unsigned long now = millis();
  if (now - prev > 1000) {
    prev = now;
    Heartbeat_1_0 hb;
    hb.uptime = now / 1000;
    hb.health.value = Health_1_0::NOMINAL;
    hb.mode.value = Mode_1_0::OPERATIONAL;
    hb.vendor_specific_status_code = 0;
    heartbeat_pub->publish(hb);
  }
}

void onTemperature(const Scalar_1_0& msg /*, const CanardTransfer* meta */)
{
  // If your 107-Arduino-Cyphal version provides transfer metadata, you can gate by source node like:
  // if (meta && meta->metadata.remote_node_id != 42) return;  // only accept from node 42

  g_latest_temp_kelvin = msg.kelvin;
  Serial.print(F("Temp K: ")); Serial.println(g_latest_temp_kelvin);

  // Brief blink to signal reception (optional)
  digitalWrite(LED_PIN, HIGH);
  delayMicroseconds(50);
  digitalWrite(LED_PIN, LOW);
}

/********** RX callback **********/
void onReceiveBufferFull(CanardFrame const & frame) {
  // Feed frame into Cyphal stack
  node_hdl.onCanFrameReceived(frame);

  // Blink LED briefly to indicate activity
  digitalWrite(LED_PIN, HIGH);
  delayMicroseconds(100);  // very short blink (non-blocking enough)
  digitalWrite(LED_PIN, LOW);
}