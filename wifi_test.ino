#include <WiFi.h>
#include <SPIFFS.h>
#include <FS.h>
#include <ESPmDNS.h>
//#include <MD5Builder.h>  /* !MD5 */
#include "esp_bt.h" 

// WiFi Configuration
const char* ssid = "DOISHA";
const char* password = "hello1234";
const char* hostname = "esp32video";

#define PORT 1234
#define BUFFER_SIZE /*1460*/ 4096
#define DEBUG 1


#if DEBUG
  #define LOG(x) Serial.println(x)
#else
  #define LOG(x)
#endif

WiFiServer server(PORT);
WiFiClient client;
SemaphoreHandle_t clientMutex;
//String fileHash;  /* !MD5 */

/* @brief Sends the video file over the connected client */
void sendFile() 
{
  File file = SPIFFS.open("/video.mp4", "r");
  if (!file) 
  {
    LOG("File open failed");
    return;
  }

  LOG("Sending " + String(file.size()) + " bytes");
  
//  MD5Builder md5;
//  md5.begin();
//  md5.addStream(file, file.size());
//  md5.calculate();
//  fileHash = md5.toString();
//  file.seek(0);

  size_t totalSent = 0;
  uint8_t buffer[BUFFER_SIZE];
  
  while (file.available()) {
    size_t bytesRead = file.readBytes((char*)buffer, BUFFER_SIZE);
    if (bytesRead == 0) break;

    size_t bytesSent = 0;
    while (bytesSent < bytesRead) 
    {
      if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(100)) == pdTRUE) 
      {
        if (client.connected()) 
        {
          size_t chunk = client.write(buffer + bytesSent, bytesRead - bytesSent);
          if (chunk > 0) {
            bytesSent += chunk;
            totalSent += chunk;
          }
        }
        else
        {
          LOG("Client disconnected during transfer.");
          file.close();
          return;
        }
        xSemaphoreGive(clientMutex);
      }
      
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
  }

  file.close();
  LOG("Transfer complete. Sent " + String(totalSent) + " bytes");
  WiFi.setSleep(WIFI_PS_MAX_MODEM);
}

/* @brief Handles a new client connection by waiting for a "READY" command
 and then initiating the file transfer */
void handleClient() 
{
  client.setTimeout(5000);
  LOG("Client connected: " + client.remoteIP().toString());
  client.setNoDelay(true);  // Disable Nagle's algorithm
  unsigned long startTime = millis();
  bool transferInitiated = false;
   
  while (millis() - startTime < 10000) 
  {
    if (client.available() >= 5) 
    {
      char cmd[6] = {0};
      client.readBytes(cmd, 5);
      if (memcmp(cmd, "READY", 5) == 0) 
      {
        LOG("Transfer initiated");
        transferInitiated = true;
        //client.print("MD5:" + fileHash);
        break;
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  
  if (transferInitiated) 
  {
    WiFi.setSleep(WIFI_PS_NONE); 
    sendFile();
  }
  
  client.stop();
  LOG("Client disconnected");
}

/* @brief Setup function for initializing serial communication, SPIFFS, WiFi,
 mDNS, and other system settings */
void setup() 
{
  Serial.begin(115200);
   /* Initialize SPIFFS and check for successful mounting.*/
  
  if (!SPIFFS.begin(true)) 
  {
    LOG("SPIFFS Mount Failed");
    /*  Halt further execution if SPIFFS cannot be mounted. */
    while (1) 
    {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }

  /* Disable Bluetooth if unused */
   if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_IDLE) 
   {
    esp_bt_controller_disable();
  }
  
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(hostname);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) 
  {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    LOG("Connecting to WiFi...");
  }

  /*  Set WiFi transmit power. */
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  /*  Set WiFi sleep mode to reduce power consumption. */
  WiFi.setSleep(WIFI_PS_MAX_MODEM); //WIFI_PS_MIN_MODEM
  
  setCpuFrequencyMhz(160);

  if (!MDNS.begin(hostname)) 
  {
    LOG("mDNS failed!");
  }
  MDNS.addService("_esp32video", "tcp", PORT);

  LOG("WiFi connected\nIP: " + WiFi.localIP().toString());
  LOG("mDNS: " + String(hostname) + ".local");

  clientMutex = xSemaphoreCreateMutex();
  server.begin();
}


void loop() 
{
  WiFiClient newClient = server.accept();
  if (newClient) 
  {
    if (xSemaphoreTake(clientMutex, portMAX_DELAY) == pdTRUE) 
    {
      if (client) client.stop();
      client = newClient;
      xSemaphoreGive(clientMutex);
    }
    handleClient();
  }
  //delay(100);
  vTaskDelay(100 / portTICK_PERIOD_MS);
}
