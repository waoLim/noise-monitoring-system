#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Arduino.h>
#include <esp_task_wdt.h>
#include <SPI.h>
#include <SD.h>
#include "FS.h"
#include "driver/i2s.h"

// Structure for storing network credentials
struct NetworkCredentials {
    const char* ssid;
    const char* password;
    int32_t rssi;  // To store signal strength
};

// Array of network credentials
NetworkCredentials networks[] = {
    {"Ernest_Angelo", "wowowiwow", -100},       // Network 1
    {"Weslim", "g3Y4pAWbEX", -100},             // Network 2
    {"Waypay", "g3Y4pAWbEX", -100},               // Network 3
};

const int numNetworks = sizeof(networks) / sizeof(networks[0]);
const char* mqtt_broker = "ed7632329e6e4fbcbe77b1fa917585a1.s1.eu.hivemq.cloud";
const char* mqtt_topic = "UPCARE/v2/CARE_2425_F7";
const char* broker_user = "ernest.angelo.valencia";
const char* broker_pass = "tavzob-Faxna3-quwjiq";
const int mqtt_port = 8883;

const char* device_id = "ESP32_Node4"; // Unique identifier for this ESP32
const int LED_PIN = 2;  // Built-in LED

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "time.nist.gov", 8 * 3600, 60000); // 8 hours offset for the Philippines (UTC+8)

const char* root_ca = \
"-----BEGIN CERTIFICATE-----\n" \
"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\n" \
"-----END CERTIFICATE-----\n";



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// RECORDING TASK/////////////////////////////////////////////////////////////////////////////////

// SD card settings
const int SD_CS_PIN = 5; 

File file;

// WAV header parameters
const unsigned long Subchunk1Size = 16;
const unsigned int AudioFormat = 1;
const unsigned int numChannels = 1;
const unsigned long sampleRate = 44100;
const unsigned int bitsPerSample = 16;
const unsigned long byteRate = sampleRate * numChannels * (bitsPerSample / 8);
const unsigned int blockAlign = numChannels * bitsPerSample / 8;
unsigned long ChunkSize = 0;
unsigned long Subchunk2Size = 0;
unsigned long Bytecounter = 0;
int fileCounter = 1;

// Helper functions for LED indicators
void indicateSuccess() {
  digitalWrite(LED_PIN, HIGH);
  vTaskDelay(100 / portTICK_PERIOD_MS);  // Quick flash
  digitalWrite(LED_PIN, LOW);
}

void indicateFailure() {
  // Triple flash for failure
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Function to scan networks and update RSSI values
void scanNetworks() {
    Serial.println("Scanning available networks...");
    
    int n = WiFi.scanNetworks();
    if (n == 0) {
        Serial.println("No networks found");
    } else {
        for (int i = 0; i < n; ++i) {
            String scannedSSID = WiFi.SSID(i);
            int32_t scannedRSSI = WiFi.RSSI(i);
            
            // Update RSSI for known networks
            for (int j = 0; j < numNetworks; j++) {
                if (scannedSSID == networks[j].ssid) {
                    networks[j].rssi = scannedRSSI;
                    break;
                }
            }
            // Short delay to prevent watchdog timer issues
            delay(10);
        }
    }
    // Clean up after scan
    WiFi.scanDelete();
}

// Function to find strongest available network
int findStrongestNetwork() {
    int strongestIndex = -1;
    int32_t strongestRSSI = -100;
    
    for (int i = 0; i < numNetworks; i++) {
        if (networks[i].rssi > strongestRSSI) {
            strongestRSSI = networks[i].rssi;
            strongestIndex = i;
        }
    }
    
    return strongestIndex;
}

void writeWavHeader(File file, int sampleRate, int bitsPerSample, int channels, unsigned long dataSize) {
    byte wavHeader[44];

    // ChunkID "RIFF"
    wavHeader[0] = 'R'; wavHeader[1] = 'I'; wavHeader[2] = 'F'; wavHeader[3] = 'F';
    // ChunkSize (file size - 8 bytes)
    unsigned long fileSize = dataSize + 36;
    wavHeader[4] = (byte)(fileSize & 0xFF);
    wavHeader[5] = (byte)((fileSize >> 8) & 0xFF);
    wavHeader[6] = (byte)((fileSize >> 16) & 0xFF);
    wavHeader[7] = (byte)((fileSize >> 24) & 0xFF);
    // Format "WAVE"
    wavHeader[8] = 'W'; wavHeader[9] = 'A'; wavHeader[10] = 'V'; wavHeader[11] = 'E';
    
    // Subchunk1ID "fmt "
    wavHeader[12] = 'f'; wavHeader[13] = 'm'; wavHeader[14] = 't'; wavHeader[15] = ' ';
    // Subchunk1Size (16 for PCM)
    wavHeader[16] = 16; wavHeader[17] = 0; wavHeader[18] = 0; wavHeader[19] = 0;
    // AudioFormat (1 for PCM)
    wavHeader[20] = 1; wavHeader[21] = 0;
    // NumChannels
    wavHeader[22] = (byte)channels; wavHeader[23] = 0;
    // SampleRate
    wavHeader[24] = (byte)(sampleRate & 0xFF);
    wavHeader[25] = (byte)((sampleRate >> 8) & 0xFF);
    wavHeader[26] = (byte)((sampleRate >> 16) & 0xFF);
    wavHeader[27] = (byte)((sampleRate >> 24) & 0xFF);
    // ByteRate = SampleRate * NumChannels * BitsPerSample/8
    unsigned long byteRate = sampleRate * channels * (bitsPerSample / 8);
    wavHeader[28] = (byte)(byteRate & 0xFF);
    wavHeader[29] = (byte)((byteRate >> 8) & 0xFF);
    wavHeader[30] = (byte)((byteRate >> 16) & 0xFF);
    wavHeader[31] = (byte)((byteRate >> 24) & 0xFF);
    // BlockAlign = NumChannels * BitsPerSample/8
    wavHeader[32] = (byte)(channels * (bitsPerSample / 8));
    wavHeader[33] = 0;
    // BitsPerSample
    wavHeader[34] = (byte)bitsPerSample;
    wavHeader[35] = 0;

    // Subchunk2ID "data"
    wavHeader[36] = 'd'; wavHeader[37] = 'a'; wavHeader[38] = 't'; wavHeader[39] = 'a';
    // Subchunk2Size (data size)
    wavHeader[40] = (byte)(dataSize & 0xFF);
    wavHeader[41] = (byte)((dataSize >> 8) & 0xFF);
    wavHeader[42] = (byte)((dataSize >> 16) & 0xFF);
    wavHeader[43] = (byte)((dataSize >> 24) & 0xFF);

    // Write header to file
    file.seek(0);
    file.write(wavHeader, 44);
}


void startRecording();
bool initializeSDCard();

void AudioRecordingTask(void *pvParameters) {
    while (true) {
        startRecording();
    }
}

bool initializeSDCard() {
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD card initialization failed!");
        return false;
    }
    return true;
}

// I2S Configuration for ESP32
void i2sInit() {
    const i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = sampleRate,
        .bits_per_sample = (i2s_bits_per_sample_t)bitsPerSample,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };
    const i2s_pin_config_t pin_config = {
        .bck_io_num = 26,    // Set BCK pin
        .ws_io_num = 22,     // Set WS pin
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = 25    // Set SD pin for data input
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    i2s_zero_dma_buffer(I2S_NUM_0);
}   

void startRecording() {


    if (!initializeSDCard()) {
        Serial.println("SD card reinitialization failed!");
        return;
    }

    Bytecounter = 0;

    timeClient.begin();
    timeClient.update();

    unsigned long currtime = timeClient.getEpochTime();
    struct tm *ptm = localtime((time_t *)&currtime);
    char time[25];
    sprintf(time, "%04d-%02d-%02dT%02d-%02d-%02d", //ISO String Format
            ptm->tm_year + 1900,  // tm_year is years since 1900
            ptm->tm_mon + 1,      // tm_mon is months since January (0-11)
            ptm->tm_mday,         // Day of the month
            ptm->tm_hour,         // Hours
            ptm->tm_min,          // Minutes
            ptm->tm_sec);         // Seconds

    // Generate new filename
    String FILENAME = "/" + String(time)+ String(device_id) + ".wav";


    // Open the file in write mode and reserve space for WAV header
    file = SD.open(FILENAME.c_str(), FILE_WRITE);
    if (!file) {
        Serial.println("Failed to create file!");
        return;
    }
    file.seek(44); // Reserve space for the header

    // Start recording
    Serial.println("Recording started...");
    i2sInit();
    size_t bytesRead;
    byte i2sData[512];

    unsigned long startTime = millis();
    
    // Record for a specified duration
    while (millis() - startTime < 60000) { //1 min recording
        i2s_read(I2S_NUM_0, (char*)i2sData, 512, &bytesRead, portMAX_DELAY);
        if (bytesRead > 0) {
            file.write(i2sData, bytesRead);
            Bytecounter += bytesRead;
        }
    }

    // Stop recording
    i2s_driver_uninstall(I2S_NUM_0);
    Serial.println("Recording stopped.");
    
    // Write WAV header with correct file size
    writeWavHeader(file, sampleRate, bitsPerSample, numChannels, Bytecounter);

    // Close the file
    file.close();
    Serial.println("Recording saved to SD card.");
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// TRANSMISSION TASK//////////////////////////////////////////////////////////////////////////////
unsigned long nextTransmissionTime = 0;

void setup_wifi() {
    Serial.println("Starting WiFi connection process...");
    
    int attempts = 0;
    bool connected = false;
    
    while (!connected && attempts < 3) {  // Try up to 3 times
        // Scan for networks and update RSSI values
        scanNetworks();
        
        // Find strongest network
        int strongestIndex = findStrongestNetwork();
        
        if (strongestIndex >= 0) {
            Serial.print("Attempting to connect to strongest network: ");
            Serial.println(networks[strongestIndex].ssid);
            Serial.print("Signal strength (RSSI): ");
            Serial.println(networks[strongestIndex].rssi);
            
            WiFi.begin(networks[strongestIndex].ssid, networks[strongestIndex].password);
            
            // Try to connect for 10 seconds
            int connectionAttempts = 0;
            while (WiFi.status() != WL_CONNECTED && connectionAttempts < 20) {
                delay(500);
                Serial.print(".");
                connectionAttempts++;
            }
            
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("\nConnected successfully!");
                Serial.print("IP address: ");
                Serial.println(WiFi.localIP());
                connected = true;
                break;
            } else {
                Serial.println("\nFailed to connect to this network");
                networks[strongestIndex].rssi = -100;  // Mark this network as unavailable
            }
        }
        
        attempts++;
        if (!connected) {
            Serial.println("Trying again...");
            delay(1000);
        }
    }
    
    if (!connected) {
        Serial.println("Failed to connect to any network");
    }
}

void reconnect() {
  int attempts = 0;
  while (!mqttClient.connected()&& attempts < 5) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect(device_id, broker_user, broker_pass)) {
      Serial.println("Connected to MQTT broker");
      Serial2.write('U'); // Send "Restart" signal to Teensy
      vTaskDelay(300 /portTICK_PERIOD_MS);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 1 seconds");
      attempts++;
      vTaskDelay(5000 /portTICK_PERIOD_MS);
    }
  }
}

void check_wifi() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected. Reconnecting...");
        WiFi.disconnect();
        setup_wifi();
    }
}

bool receiveFloat(float &value) {
  static bool receiving = false;
  static byte buffer[sizeof(float)];
  static int index = 0;
  unsigned long startrxlaeq = 0;
  while (Serial2.available()) {
    byte b = Serial2.read();

    if (b == 0x02) { // Start marker
      //startrxlaeq = micros();
      receiving = true;
      index = 0;
    } else if (b == 0x03) { // End marker
      //Serial2.write('D'); // Send "Done" signal to Teensy
      //Serial.print("Receive Time: ");
      //Serial.println(micros()-startrxlaeq); // LAeq receiving time
      

      if (receiving && index == sizeof(float)) {
        memcpy(&value, buffer, sizeof(float));
        return true;
      }
      receiving = false;
    } else if (receiving) {
      if (index < sizeof(float)) {
        buffer[index++] = b;
      } else {
        receiving = false;
      }
    }
  }
  return false;
}

const int uartBaudRate = 115200; // UART baud rate
const int totalFrames = 225;    // Total number of frames
const uint8_t mfccSize = 13;         // Number of MFCC coefficients per frame
const unsigned long timeout = 15000; // Timeout for UART read operations (in milliseconds)

float mfcc_array[totalFrames][mfccSize]; // Array to store received MFCC coefficients

uint8_t calculateChecksum(float *data, size_t length) {
  uint8_t checksum = 0;
  byte *byteData = (byte *)data;
  for (size_t i = 0; i < length * sizeof(float); i++) {
    checksum ^= byteData[i];
  }
  return checksum;
}

bool waitForByte(uint8_t expectedByte, unsigned long timeout) {
  unsigned long startMillis = millis();
  while (millis() - startMillis < timeout) {
    if (Serial2.available() && Serial2.read() == expectedByte) {
      return true;
    }
  }
  return false;
}

bool readBytes(byte *buffer, size_t length, unsigned long timeout) {
  unsigned long startMillis = millis();
  size_t bytesRead = 0;
  while (millis() - startMillis < timeout && bytesRead < length) {
    if (Serial2.available()) {
      buffer[bytesRead++] = Serial2.read();
    }
  }
  return bytesRead == length;
}

bool readUInt16(uint16_t &value, unsigned long timeout) {
  unsigned long startMillis = millis();
  byte buffer[2];
  size_t bytesRead = 0;
  
  while (millis() - startMillis < timeout && bytesRead < 2) {
    if (Serial2.available()) {
      buffer[bytesRead++] = Serial2.read();
    }
  }
  
  if (bytesRead == 2) {
    value = (buffer[0] << 8) | buffer[1]; // Combine the two bytes into a uint16_t value
    return true;
  }
  
  return false;
}


void Send2MQTT(void * parameter){
  Serial2.begin(115200);
  Serial.begin(115200);
  setup_wifi();
  espClient.setCACert(root_ca);
  mqttClient.setServer(mqtt_broker, mqtt_port);
  

  if (!mqttClient.setBufferSize(20480)) { //20kB buffer size 
    Serial.println("Failed to set buffer size");
  }
  timeClient.begin();
  timeClient.update();

  unsigned long currentTime = timeClient.getEpochTime();
  nextTransmissionTime = (currentTime / 5) * 5 + 5; // Align to the next 5-second mark

  reconnect();




while(1){
  esp_task_wdt_reset();  // Reset the watchdog to prevent it from triggering
  check_wifi();
  if (!mqttClient.connected()) {
    reconnect();
  }

  mqttClient.loop();
  timeClient.update();
  unsigned long currentTime = timeClient.getEpochTime();


    if (currentTime >= nextTransmissionTime) {
    nextTransmissionTime += 5; // Schedule next transmission at the next 5-second mark
    Serial2.write('R'); // Send "Ready" signal to Teensy
    vTaskDelay(600 /portTICK_PERIOD_MS);
    float receivedLAeq;
    if (receiveFloat(receivedLAeq)) {
      struct tm *ptm = localtime((time_t *)&currentTime);
      char timeString[25];
      sprintf(timeString, "%04d-%02d-%02dT%02d:%02d:%02d", //ISO String Format
              ptm->tm_year + 1900,  // tm_year is years since 1900
              ptm->tm_mon + 1,      // tm_mon is months since January (0-11)
              ptm->tm_mday,         // Day of the month
              ptm->tm_hour,         // Hours
              ptm->tm_min,          // Minutes
              ptm->tm_sec);         // Seconds

      String payload = "{";
      payload += "\"INMP441_Geohash\":\"" + String("wdw56up3vd64") + "\",";
      payload += "\"INMP441_LAeq\":\"" + String(receivedLAeq) + "\",";
      payload += "\"local_time\":\"" + String(timeString) + "\",";
      payload += "\"source\":\"" + String("ESP32_Node4_LAeq") + "\",";
      payload += "\"type\":\"" + String("data") + "\"";
      payload += "}";

      //for debugging
      //Serial.print("JSON payload");
      Serial.println(payload);

      if (mqttClient.publish(mqtt_topic, payload.c_str())) {
        Serial.println("LAeq Published!");
        indicateSuccess();  // LED indicator for success
      } else {
        Serial.println("LAeq Not Published!");
        indicateFailure();  // LED indicator for failure
      }
    } else {
      Serial.println("No data received from Teensy");

      struct tm *ptm = localtime((time_t *)&currentTime);
      char timeString[25];
      sprintf(timeString, "%04d-%02d-%02dT%02d:%02d:%02d", //ISO String Format
              ptm->tm_year + 1900,  // tm_year is years since 1900
              ptm->tm_mon + 1,      // tm_mon is months since January (0-11)
              ptm->tm_mday,         // Day of the month
              ptm->tm_hour,         // Hours
              ptm->tm_min,          // Minutes
              ptm->tm_sec);         // Seconds

      String payload = "{";
      payload += "\"INMP441_LAeqError\":\"" + String("1") + "\",";
      payload += "\"local_time\":\"" + String(timeString) + "\",";
      payload += "\"source\":\"" + String("ESP32_Node4_LaeqError") + "\",";
      payload += "\"type\":\"" + String("data") + "\"";
      payload += "}";
      mqttClient.publish(mqtt_topic, payload.c_str());
    }
    }
      
        bool StartByteReceived = true;
      // Wait for the start byte
        if (!waitForByte(0x01, timeout)) {
        Serial.println("Error: Start byte not received!");
          StartByteReceived = false;
        }Serial.println("Start byte received!");
        //unsigned long starttimeMFCC = millis();

      if (StartByteReceived){

        bool MFCCValid = true;

        // Read the number of frames and MFCC size
        uint16_t receivedFrames;
        uint8_t receivedMfccSize;

        if (!readUInt16(receivedFrames, timeout) ||
            !readBytes(&receivedMfccSize, sizeof(receivedMfccSize), timeout)) {
          Serial.println("Error: Frame count or MFCC size not received!");
          MFCCValid = false;
        }

        if (receivedFrames != totalFrames || receivedMfccSize != mfccSize) {
          Serial.println(receivedFrames);
          Serial.println(receivedMfccSize);
          Serial.println("Error: Frame or MFCC size mismatch!");
          MFCCValid = false;
        }

        if(MFCCValid){
        // Read the MFCC coefficients
        bool dataValid = true;
        for (int i = 0; i < totalFrames; i++) {
          uint8_t receivedChecksum;
          if (!readBytes((byte *)&receivedChecksum, sizeof(receivedChecksum), timeout)) {
            Serial.println("Error: Checksum not received!");
            dataValid = false;
            return;
          }

          byte mfccBytes[mfccSize * sizeof(float)];
          if (!readBytes(mfccBytes, mfccSize * sizeof(float), timeout)) {
            Serial.println("Error: MFCC data not received!");
            dataValid = false;
            return;
          }

          for (int j = 0; j < mfccSize; j++) {
            mfcc_array[i][j] = ((float *)mfccBytes)[j]; // Convert bytes back to float
          }

          uint8_t calculatedChecksum = calculateChecksum(mfcc_array[i], mfccSize);
          if (receivedChecksum != calculatedChecksum) {
            Serial.println("Error: Checksum mismatch!");
            dataValid = false;
            break;
          }Serial.println("Checksum matched!");
        }

        // Wait for the end byte
        if (!waitForByte(0x04, timeout)) {
          Serial.println("Error: End byte not received!");
          dataValid = false;
        }
        //Serial2.write('F'); //send "Finished" signal to Teensy
        //Serial.println("End byte received");
        //Serial.print("Receive Time: ");
        //Serial.println(millis()-starttimeMFCC); // MFCC receiving time
        
        if (dataValid) {
          // // Debug: Print the received MFCC array
          // for (int i = 0; i < totalFrames; i++) {
          //   for (int j = 0; j < mfccSize; j++) {
          //     Serial.print(mfcc_array[i][j]);
          //     Serial.print(" ");
          //   }
          //   Serial.println();
          // }
          // Serial.println("Data received correctly!");
          // memset(mfcc_array, 0, sizeof(mfcc_array));

          // Publish MFCC array to MQTT topic
          String mfccData;


          mfccData =  "{";
          mfccData += "\"INMP441_MFCC\":\"";
          for (int i = 0; i < totalFrames; i++) {
            for (int j = 0; j < mfccSize; j++) {
              mfccData += String(mfcc_array[i][j]);
              if (j < mfccSize - 1) {
                mfccData += ",";
              }
            }
            if (i < totalFrames - 1) {
              mfccData += "|";
            }
          }
          mfccData += "\",";
          struct tm *ptm = localtime((time_t *)&currentTime);
          char timeStringmfcc[25];
          sprintf(timeStringmfcc, "%04d-%02d-%02dT%02d:%02d:%02d", //ISO String Format
                  ptm->tm_year + 1900,  // tm_year is years since 1900
                  ptm->tm_mon + 1,      // tm_mon is months since January (0-11)
                  ptm->tm_mday,         // Day of the month
                  ptm->tm_hour,         // Hours
                  ptm->tm_min,          // Minutes
                  ptm->tm_sec);         // Seconds
          mfccData += "\"local_time\":\"" + String(timeStringmfcc) + "\",";
          mfccData += "\"source\":\"" + String("ESP32_Node4_MFCC") + "\",";
          mfccData += "\"type\":\"" + String("data") + "\"";
          mfccData +=  "}";
          int mfccDataSize = mfccData.length();
          Serial.print("Data size: ");
          Serial.println(mfccDataSize);
          if (mqttClient.publish(mqtt_topic, mfccData.c_str())) {
            Serial.println("MFCC array published!");
            indicateSuccess();  // LED indicator for success
            memset(mfcc_array, 0, sizeof(mfcc_array));

          } else {

            Serial.println("MFCC array not published!");
            indicateFailure();  // LED indicator for failure
            
            struct tm *ptm = localtime((time_t *)&currentTime);
            char timeString[25];
            sprintf(timeString, "%04d-%02d-%02dT%02d:%02d:%02d", //ISO String Format
                    ptm->tm_year + 1900,  // tm_year is years since 1900
                    ptm->tm_mon + 1,      // tm_mon is months since January (0-11)
                    ptm->tm_mday,         // Day of the month
                    ptm->tm_hour,         // Hours
                    ptm->tm_min,          // Minutes
                    ptm->tm_sec);         // Seconds

            String payload = "{";
            payload += "\"INMP441_MFCCError\":\"" + String("1") + "\",";
            payload += "\"local_time\":\"" + String(timeString) + "\",";
            payload += "\"source\":\"" + String("ESP32_Node4_MFCCError") + "\",";
            payload += "\"type\":\"" + String("data") + "\"";
            payload += "}";
            mqttClient.publish(mqtt_topic, payload.c_str());   
          }
        } else {
          Serial.println("Data corrupted!");
          memset(mfcc_array, 0, sizeof(mfcc_array));
            struct tm *ptm = localtime((time_t *)&currentTime);
            char timeString[25];
            sprintf(timeString, "%04d-%02d-%02dT%02d:%02d:%02d", //ISO String Format
                    ptm->tm_year + 1900,  // tm_year is years since 1900
                    ptm->tm_mon + 1,      // tm_mon is months since January (0-11)
                    ptm->tm_mday,         // Day of the month
                    ptm->tm_hour,         // Hours
                    ptm->tm_min,          // Minutes
                    ptm->tm_sec);         // Seconds

            String payload = "{";
            payload += "\"INMP441_MFCCError\":\"" + String("1") + "\",";
            payload += "\"local_time\":\"" + String(timeString) + "\",";
            payload += "\"source\":\"" + String("ESP32_Node4_MFCCError") + "\",";
            payload += "\"type\":\"" + String("data") + "\"";
            payload += "}";
            mqttClient.publish(mqtt_topic, payload.c_str()); 
            vTaskDelay(5000 /portTICK_PERIOD_MS);//wait for next mfcc Transmission

        }
        }else{
          Serial.println("Data corrupted!");
          memset(mfcc_array, 0, sizeof(mfcc_array));
            struct tm *ptm = localtime((time_t *)&currentTime);
            char timeString[25];
            sprintf(timeString, "%04d-%02d-%02dT%02d:%02d:%02d", //ISO String Format
                    ptm->tm_year + 1900,  // tm_year is years since 1900
                    ptm->tm_mon + 1,      // tm_mon is months since January (0-11)
                    ptm->tm_mday,         // Day of the month
                    ptm->tm_hour,         // Hours
                    ptm->tm_min,          // Minutes
                    ptm->tm_sec);         // Seconds

            String payload = "{";
            payload += "\"INMP441_MFCCError\":\"" + String("1") + "\",";
            payload += "\"local_time\":\"" + String(timeString) + "\",";
            payload += "\"source\":\"" + String("ESP32_Node4_MFCCError") + "\",";
            payload += "\"type\":\"" + String("data") + "\"";
            payload += "}";
            mqttClient.publish(mqtt_topic, payload.c_str()); 
            vTaskDelay(5000 /portTICK_PERIOD_MS);//wait for next mfcc Transmission

        }
        }else{
          Serial.println("Data corrupted!");
          memset(mfcc_array, 0, sizeof(mfcc_array));
            struct tm *ptm = localtime((time_t *)&currentTime);
            char timeString[25];
            sprintf(timeString, "%04d-%02d-%02dT%02d:%02d:%02d", //ISO String Format
                    ptm->tm_year + 1900,  // tm_year is years since 1900
                    ptm->tm_mon + 1,      // tm_mon is months since January (0-11)
                    ptm->tm_mday,         // Day of the month
                    ptm->tm_hour,         // Hours
                    ptm->tm_min,          // Minutes
                    ptm->tm_sec);         // Seconds

            String payload = "{";
            payload += "\"INMP441_MFCCError\":\"" + String("1") + "\",";
            payload += "\"local_time\":\"" + String(timeString) + "\",";
            payload += "\"source\":\"" + String("ESP32_Node4_MFCCError") + "\",";
            payload += "\"type\":\"" + String("data") + "\"";
            payload += "}";
            mqttClient.publish(mqtt_topic, payload.c_str()); 
            vTaskDelay(5000 /portTICK_PERIOD_MS);//wait for next mfcc Transmission

        }
 
}

}


void setup() {
Serial.begin(115200);
pinMode(LED_PIN, OUTPUT);  // Initialize LED pin
digitalWrite(LED_PIN, LOW);  // Start with LED off
esp_task_wdt_init(30, true);  // Set WDT timeout to 20 seconds 

xTaskCreatePinnedToCore(
    Send2MQTT,      // Function that should be called
    "Transmit to MQTT",    // Name of the task (for debugging)
    16384,               // Stack size (bytes)
    NULL,               // Parameter to pass
    1,                  // Task priority
    NULL,               // Task handle
    1         // Core 1
);

xTaskCreatePinnedToCore(
    AudioRecordingTask,      // Function that should be called
    "Record Audio to SD Card",    // Name of the task (for debugging)D
   4096,               // Stack size (bytes)
    NULL,               // Parameter to pass
    1,                  // Task priority
    NULL,               // Task handle
    0        // Core 0
    
);

}

void loop() {

}