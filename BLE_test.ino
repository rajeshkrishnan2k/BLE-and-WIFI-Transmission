
//#include <BLEDevice.h>
//#include <BLEUtils.h>
//#include <BLEServer.h>
//#include <BLE2902.h>
//#include <SPIFFS.h>
//#include <FS.h>
// 
//
//// UUIDs
//#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
//#define CHARACTERISTIC_UUID "abcd1234-5678-1234-5678-abcdef123456"
//#define CONTROL_UUID        "beefcafe-dead-beef-dead-beefcafe0001"
//
//#define BUFFER_SIZE         509  // Reduced to match BLE MTU limits
//#define BUFFER_POOL_SIZE    15   // Increased buffer pool size
//#define DEBUG               1
//
//#if DEBUG
//#define LOG(x) Serial.println(x)
//#else
//#define LOG(x)
//#endif
//
//BLECharacteristic *pCharacteristic;
//bool deviceConnected = false;
//bool clientReady = false;
//QueueHandle_t dataQueue;
//SemaphoreHandle_t bufferSemaphore;
//uint8_t bufferPool[BUFFER_POOL_SIZE][BUFFER_SIZE];
//
//// Callback for client writes to the control characteristic
//class ControlCallback : public BLECharacteristicCallbacks {
//    void onWrite(BLECharacteristic *pCharacteristic) {
//      uint8_t* data = pCharacteristic->getData();
//      size_t length = pCharacteristic->getLength();
//
//      if (length >= 5 && memcmp(data, "READY", 5) == 0) {
//        clientReady = true;
//        LOG("Client ready. Starting transfer.");
//      }
//    }
//};
//
//class MyServerCallbacks : public BLEServerCallbacks {
//    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
//      deviceConnected = true;
//      LOG("Device connected. Waiting for client signal...");
//  pServer->getAdvertising()->stop(); 
//  
//      // Set faster connection parameters (7.5ms interval)
//      esp_ble_conn_update_params_t conn_params = {0};
//      memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
//      conn_params.latency = 0;
//      conn_params.min_int = 6;   // 7.5ms
//      conn_params.max_int = 6;   // 7.5ms
//      conn_params.timeout = 400; // 4s timeout
//      esp_ble_gap_update_conn_params(&conn_params);
//    }
//
//    void onDisconnect(BLEServer* pServer) {
//      deviceConnected = false;
//      clientReady = false;
//      LOG("Disconnected. Restarting advertising.");
//      pServer->startAdvertising();
//    }
//};
//
//void readFileTask(void *params) {
//  LOG("File Reader Task started.");
//
//  // Wait for client readiness
//  while (!clientReady) vTaskDelay(10 / portTICK_PERIOD_MS);
//
//  File videoFile = SPIFFS.open("/video.mp4", "r");
//  if (!videoFile) {
//    LOG("Failed to open video file.");
//    vTaskDelete(NULL);
//    return;
//  }
//
//  LOG("Transfer started.");
//  int bufferIndex = 0;
//  while (videoFile.available() > 0) {
//    if (xSemaphoreTake(bufferSemaphore, portMAX_DELAY) == pdTRUE) {
//      uint8_t *buf = bufferPool[bufferIndex];
//      size_t bytesRead = videoFile.readBytes((char *)buf, BUFFER_SIZE);
//
//      if (bytesRead > 0) {
//        xQueueSend(dataQueue, &buf, portMAX_DELAY);
//        bufferIndex = (bufferIndex + 1) % BUFFER_POOL_SIZE;
//        vTaskDelay(15 / portTICK_PERIOD_MS);  // Increased delay
//      } else {
//        xSemaphoreGive(bufferSemaphore);
//      }
//    }
//  }
//
//  videoFile.close();
//  LOG("Transfer complete.");
//  vTaskDelete(NULL);
//}
//
//void bleNotifyTask(void *params) {
//  LOG("BLE Notify Task started.");
//  uint8_t *buf;
//  while (1) {
//    if (xQueueReceive(dataQueue, &buf, portMAX_DELAY) == pdPASS) {
//      if (deviceConnected && clientReady) {
//        pCharacteristic->setValue(buf, BUFFER_SIZE);
//        pCharacteristic->notify();
//        LOG("Packet sent.");
//      }
//      xSemaphoreGive(bufferSemaphore);
//    }
//  }
//}
//
//void setup() {
//
//  Serial.begin(115200);
//  if (!SPIFFS.begin(true)) {
//    LOG("SPIFFS mount failed.");
//    return;
//  }
//  setCpuFrequencyMhz(80);
//  // Initialize buffers and semaphores
//  bufferSemaphore = xSemaphoreCreateCounting(BUFFER_POOL_SIZE, BUFFER_POOL_SIZE);
//  dataQueue = xQueueCreate(20, sizeof(uint8_t*));  // Larger queue
//
//  // BLE setup
//  BLEDevice::init("ESP32_BLE");
//  BLEDevice::setMTU(512);  
//  BLEDevice::setPower(ESP_PWR_LVL_P9);  
//
//  BLEServer *pServer = BLEDevice::createServer();
//  pServer->setCallbacks(new MyServerCallbacks());
//
//// Configure advertising parameters
//  BLEAdvertising *pAdvertising = pServer->getAdvertising();
//  pAdvertising->setMinInterval(800);  // 1000ms (1600 * 0.625ms)
//  pAdvertising->setMaxInterval(1600);  // 1500ms (2400 * 0.625ms)
//
//
//  BLEService *pService = pServer->createService(SERVICE_UUID);
//  pCharacteristic = pService->createCharacteristic(
//                      CHARACTERISTIC_UUID,
//                      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
//                    );
//  pCharacteristic->addDescriptor(new BLE2902());
//
//  // Control characteristic
//  BLECharacteristic *pControl = pService->createCharacteristic(
//                                  CONTROL_UUID,
//                                  BLECharacteristic::PROPERTY_WRITE
//                                );
//  pControl->setCallbacks(new ControlCallback());
//
//  pService->start();
//  pServer->getAdvertising()->start();
//
//  xTaskCreate(readFileTask, "ReadFileTask", 4096, NULL, 1, NULL);
//  xTaskCreate(bleNotifyTask, "BleNotifyTask", 4096, NULL, 2, NULL);
//}
//
//void loop() {}


#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <SPIFFS.h>
#include <FS.h>

// UUIDs
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "abcd1234-5678-1234-5678-abcdef123456"
#define CONTROL_UUID        "beefcafe-dead-beef-dead-beefcafe0001"

#define BUFFER_SIZE         509
#define BUFFER_POOL_SIZE    15
#define DEBUG               1

#if DEBUG
#define LOG(x) Serial.println(x)
#else
#define LOG(x)
#endif

// Global BLE objects
BLEServer *pServer;
BLECharacteristic *pCharacteristic;
uint16_t clientConnId = 0;
bool deviceConnected = false;
bool clientReady = false;

// Buffer management
QueueHandle_t dataQueue;
SemaphoreHandle_t bufferSemaphore;
uint8_t bufferPool[BUFFER_POOL_SIZE][BUFFER_SIZE];

class ControlCallback : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      uint8_t* data = pCharacteristic->getData();
      size_t length = pCharacteristic->getLength();

      if (length >= 5 && memcmp(data, "READY", 5) == 0) {
        clientReady = true;
        LOG("Client ready. Starting transfer.");
      }
    }
};

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
      deviceConnected = true;
      clientConnId = param->connect.conn_id;
      LOG("Device connected. Waiting for client signal...");
      pServer->getAdvertising()->stop();

      // Set fast connection parameters
      esp_ble_conn_update_params_t conn_params = {0};
      memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
      conn_params.latency = 0;
      conn_params.min_int = 6;   // 7.5ms
      conn_params.max_int = 6;   // 7.5ms
      conn_params.timeout = 400; // 4s
      esp_ble_gap_update_conn_params(&conn_params);
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      clientReady = false;
      LOG("Disconnected. Restarting advertising.");
      pServer->startAdvertising();
    }
};

void readFileTask(void *params) {
  LOG("File Reader Task started.");

  while (!clientReady) vTaskDelay(10 / portTICK_PERIOD_MS);

  File videoFile = SPIFFS.open("/video.mp4", "r");
  if (!videoFile) {
    LOG("Failed to open video file.");
    vTaskDelete(NULL);
    return;
  }

  LOG("Transfer started.");
  int bufferIndex = 0;
  while (videoFile.available() > 0) {
    if (xSemaphoreTake(bufferSemaphore, portMAX_DELAY) == pdTRUE) {
      uint8_t *buf = bufferPool[bufferIndex];
      size_t bytesRead = videoFile.readBytes((char *)buf, BUFFER_SIZE);

      if (bytesRead > 0) {
        xQueueSend(dataQueue, &buf, portMAX_DELAY);
        bufferIndex = (bufferIndex + 1) % BUFFER_POOL_SIZE;
        vTaskDelay(15 / portTICK_PERIOD_MS);
      } else {
        xSemaphoreGive(bufferSemaphore);
      }
    }
  }

  videoFile.close();
  LOG("Transfer complete.");

  if (deviceConnected) {
    LOG("Disconnecting client...");
    pServer->disconnect(clientConnId);
  }

  vTaskDelete(NULL);
}

void bleNotifyTask(void *params) {
  LOG("BLE Notify Task started.");
  uint8_t *buf;
  
  while (1) {
    if (xQueueReceive(dataQueue, &buf, portMAX_DELAY) == pdPASS) {
      if (deviceConnected && clientReady) {
        pCharacteristic->setValue(buf, BUFFER_SIZE);
        pCharacteristic->notify();
        LOG("Packet sent.");
      }
      xSemaphoreGive(bufferSemaphore);
    }
  }
}

void setup() {
  Serial.begin(115200);
  if (!SPIFFS.begin(true)) {
    LOG("SPIFFS mount failed.");
    return;
  }

  // Set CPU frequency
  setCpuFrequencyMhz(80);

  // Initialize buffers
  bufferSemaphore = xSemaphoreCreateCounting(BUFFER_POOL_SIZE, BUFFER_POOL_SIZE);
  dataQueue = xQueueCreate(20, sizeof(uint8_t*));

  // BLE initialization
  BLEDevice::init("ESP32_BLE");
  BLEDevice::setMTU(512);
  BLEDevice::setPower(ESP_PWR_LVL_N12);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Advertising configuration
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->setMinInterval(800);
  pAdvertising->setMaxInterval(1600);

  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Data characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());

  // Control characteristic
  BLECharacteristic *pControl = pService->createCharacteristic(
                                  CONTROL_UUID,
                                  BLECharacteristic::PROPERTY_WRITE
                                );
  pControl->setCallbacks(new ControlCallback());

  pService->start();
  pAdvertising->start();

  // Create tasks
  xTaskCreate(readFileTask, "ReadFileTask", 4096, NULL, 1, NULL);
  xTaskCreate(bleNotifyTask, "BleNotifyTask", 4096, NULL, 2, NULL);
}

void loop() {
  // Empty
}
