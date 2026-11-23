#include <Arduino.h>
#include <BLE.h>
#include <cmath>
#include <cstdlib>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace {
const BLEUUID serviceUuid("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
const BLEUUID txUuid("6e400003-b5a3-f393-e0a9-e50e24dcca9e");
constexpr char deviceName[] = "TEG_XIAO";
constexpr int fanPin = 4;  // D2 on XIAO ESP32C3.
constexpr bool enableFanOutput = false;
constexpr double fanOnC = 28.0;
constexpr double fanOffC = 26.0;
constexpr uint32_t staleAfterMs = 15000;

struct CsvRow {
  char text[256];
  uint32_t receivedAt;
};

BLEClient client;
QueueHandle_t rows = nullptr;
bool fanRequested = false;
bool haveFreshTemperature = false;
uint32_t lastTemperatureAt = 0;

void setFan(bool requested) {
  if (requested != fanRequested) {
    Serial.println(requested ? "FAN ON" : "FAN OFF");
  }
  fanRequested = requested;
  digitalWrite(fanPin, enableFanOutput && requested ? HIGH : LOW);
}

// Fragment state is used only from BLE callbacks.
CsvRow pending{};
size_t fragmentLength = 0;
bool discardRow = false;

void resetFragment() {
  fragmentLength = 0;
  discardRow = false;
}

void onNotification(BLERemoteCharacteristic, const uint8_t* data, size_t size, BLESubscription) {
  for (size_t i = 0; i < size; ++i) {
    const char next = static_cast<char>(data[i]);
    if (next == '\n') {
      if (!discardRow && fragmentLength > 0) {
        pending.text[fragmentLength] = '\0';
        pending.receivedAt = millis();
        if (xQueueSend(rows, &pending, 0) != pdPASS) {
          // Keep the newest rows if serial output falls behind.
          CsvRow oldest;
          xQueueReceive(rows, &oldest, 0);
          xQueueSend(rows, &pending, 0);
        }
      }
      resetFragment();
    } else if (fragmentLength == sizeof(pending.text) - 1) {
      discardRow = true;
    } else if (!discardRow) {
      pending.text[fragmentLength++] = next;
    }
  }
}

bool parseRow(const char* text, double (&fields)[15]) {
  const char* field = text;
  for (size_t i = 0; i < 15; ++i) {
    char* end = nullptr;
    fields[i] = std::strtod(field, &end);
    const char delimiter = (i == 14) ? '\0' : ',';
    if (end == field || *end != delimiter) {
      return false;
    }
    field = end + 1;
  }
  return std::isfinite(fields[0]) && fields[0] >= 0;
}

bool subscribeToSensor() {
  BLEScan scan = BLE.getScan();
  const BLEScan::Results results = scan.startBlocking(2500);
  for (const BLEAdvertisedDevice& device : results) {
    if (device.isAdvertisingService(serviceUuid) && device.getName() == deviceName) {
      if (!client.connect(device)) {
        return false;
      }
      BLERemoteService service = client.getService(serviceUuid);
      BLERemoteCharacteristic tx = service ? service.getCharacteristic(txUuid)
                                           : BLERemoteCharacteristic{};
      if (!tx || !tx.canNotify() || !tx.subscribe(BLESubscription::Notify, onNotification)) {
        Serial.println("# NUS TX subscription failed");
        client.disconnect();
        return false;
      }
      Serial.printf("# Subscribed to TEG_XIAO NUS TX, MTU=%u\n",
                    static_cast<unsigned>(client.getMTU()));
      return true;
    }
  }
  return false;
}
}  // namespace

void setup() {
  pinMode(fanPin, OUTPUT);
  digitalWrite(fanPin, LOW);
  Serial.begin(115200);
  Serial.printf("# ESP32C3 receiver. Fan GPIO %s.\n",
                enableFanOutput ? "enabled" : "held low");

  rows = xQueueCreate(6, sizeof(CsvRow));
  if (!rows || !BLE.begin("TEG_Fan")) {
    Serial.println("# Receiver initialization failed");
    return;
  }

  BLE.getScan().setActiveScan(true);
  client = BLE.createClient();
  if (client) {
    client.setMTU(247);
    client.onDisconnect([](BLEClient&, const BLEConnInfo&, uint8_t) { resetFragment(); });
  } else {
    Serial.println("# BLE client creation failed");
  }
}

void loop() {
  static bool subscribed = false;
  if (!client) {
    delay(1000);
    return;
  }

  if (!client.isConnected() || !subscribed) {
    if (subscribed) {
      Serial.println("# Sensor disconnected");
    }
    subscribed = false;
    haveFreshTemperature = false;
    setFan(false);
    xQueueReset(rows);
    if (client.isConnected()) {
      delay(50);  // Wait for a failed subscription's disconnect to complete.
      return;
    }
    subscribed = subscribeToSensor();
    if (!subscribed) {
      delay(500);
    }
    return;
  }

  CsvRow row;
  if (xQueueReceive(rows, &row, pdMS_TO_TICKS(50)) == pdPASS) {
    double fields[15];
    if (parseRow(row.text, fields)) {
      Serial.printf("RX,%lu,%s\n", static_cast<unsigned long>(row.receivedAt), row.text);
      const double temperatureC = fields[9];
      const bool fresh = millis() - row.receivedAt < staleAfterMs;
      if (client.isConnected() && fresh && fields[14] == 0 && std::isfinite(temperatureC)) {
        haveFreshTemperature = true;
        lastTemperatureAt = row.receivedAt;
        setFan(temperatureC >= fanOnC || (temperatureC > fanOffC && fanRequested));
      } else {
        haveFreshTemperature = false;
        setFan(false);
      }
    } else {
      Serial.println("# Dropped malformed CSV row");
    }
  }

  if (!client.isConnected() ||
      (haveFreshTemperature && millis() - lastTemperatureAt >= staleAfterMs)) {
    haveFreshTemperature = false;
    setFan(false);
  }
}
