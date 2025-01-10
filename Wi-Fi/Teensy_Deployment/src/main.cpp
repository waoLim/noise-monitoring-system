/*
 * This file is part of the FreeRTOS port to Teensy boards.
 * Copyright (c) 2020-2024 Timo Sandmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library. If not, see <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include "arduino_freertos.h"
#include "avr/pgmspace.h"
#include <wiring.h>
#include "arduinoMFCC.h"
#include "arduinoMFCC.cpp"

// Task handles
TaskHandle_t xLAeqComputationTask;
TaskHandle_t xAudioRecordingTask;
TaskHandle_t xMFCCComputationTask;
TaskHandle_t xRestartTask;

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// TEENSY AUDIO LIBRARY CONNECTIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////

// GUItool: begin automatically generated code
AudioSynthNoisePink      pink1;          //xy=117,268
AudioInputI2S            i2s1;           //xy=118,193
AudioInputI2S2           i2s2;
AudioRecordQueue         queue;          // For MFCC Task
AudioRecordQueue         queue1;         // For Audio Record Task
AudioMixer4              mixer1;         //xy=260,240
AudioFilterStateVariable filter31;       //xy=456,195
AudioFilterStateVariable filter63;       //xy=456,235
AudioFilterStateVariable filter125;      //xy=456,275
AudioFilterStateVariable filter250;      //xy=456,315
AudioFilterStateVariable filter500;      //xy=456,355
AudioFilterStateVariable filter1000;     //xy=456,395
AudioFilterStateVariable filter2000;     //xy=456,435
AudioFilterStateVariable filter4000;     //xy=456,475
AudioFilterStateVariable filter8000;     //xy=459,528
AudioAnalyzeRMS          rms31;          //xy=630,195
AudioAnalyzeRMS          rms63;          //xy=630,235
AudioAnalyzeRMS          rms125;         //xy=630,275
AudioAnalyzeRMS          rms250;         //xy=630,315
AudioAnalyzeRMS          rms500;         //xy=630,355
AudioAnalyzeRMS          rms1000;        //xy=630,395
AudioAnalyzeRMS          rms2000;        //xy=630,435
AudioAnalyzeRMS          rms4000;        //xy=630,475
AudioAnalyzeRMS          rms8000;        //xy=630,515
AudioConnection          patchCord1(pink1, 0, mixer1, 1);
AudioConnection          patchCord2(i2s1, 0, mixer1, 0);
AudioConnection          patchCord3(mixer1, 0, filter31, 0);
AudioConnection          patchCord4(mixer1, 0, filter63, 0);
AudioConnection          patchCord5(mixer1, 0, filter125, 0);
AudioConnection          patchCord6(mixer1, 0, filter250, 0);
AudioConnection          patchCord7(mixer1, 0, filter500, 0);
AudioConnection          patchCord8(mixer1, 0, filter1000, 0);
AudioConnection          patchCord9(mixer1, 0, filter2000, 0);
AudioConnection          patchCord10(mixer1, 0, filter4000, 0);
AudioConnection          patchCord11(mixer1, 0, filter8000, 0);
AudioConnection          patchCord12(filter31, 1, rms31, 0);
AudioConnection          patchCord13(filter63, 1, rms63, 0);
AudioConnection          patchCord14(filter125, 1, rms125, 0);
AudioConnection          patchCord15(filter250, 1, rms250, 0);
AudioConnection          patchCord16(filter500, 1, rms500, 0);
AudioConnection          patchCord17(filter1000, 1, rms1000, 0);
AudioConnection          patchCord18(filter2000, 1, rms2000, 0);
AudioConnection          patchCord19(filter4000, 1, rms4000, 0);
AudioConnection          patchCord20(filter8000, 1, rms8000, 0);

AudioConnection          patchCord21(i2s1, 0, queue, 0);
AudioConnection          patchCord22(i2s2,0, queue1,0);
// GUItool: end automatically generated code


/*
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUDIO RECORDING
/////////////////////////////////////////////////////////////////////////////////////////////////////////

// SD card settings
const int chipSelect = BUILTIN_SDCARD;

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

void writeOutHeader(unsigned long recByteSaved);
void startRecording();
bool initializeSDCard();

void AudioRecordingTask(void *pvParameters) {
    while (true) {
        startRecording();
    }
}

bool initializeSDCard() {
    int initTries = 0;
    while (!SD.begin(chipSelect) && initTries < 5) {
        Serial.println("SD card initialization failed! Retrying...");
        delay(500);
        initTries++;
    }
    return initTries < 5;
}

void startRecording() {
    if (!initializeSDCard()) {
        Serial.println("SD card reinitialization failed!");
        return;
    }

    Bytecounter = 0;

    // Generate new filename
    String FILENAME = "recording" + String(fileCounter++) + ".wav";

    if (SD.exists(FILENAME.c_str())) {
        SD.remove(FILENAME.c_str());
    }

    // Open the file in write mode and reserve space for WAV header
    file = SD.open(FILENAME.c_str(), FILE_WRITE);
    if (!file) {
        Serial.println("Failed to create file!");
        return;
    }
    file.seek(44); // Reserve space for the header

    // Start recording
    Serial.println("Recording started...");
    queue1.begin();
    unsigned long startTime = millis();

    // Record for a specified duration
    while (millis() - startTime < 30000) {
        if (queue1.available() >= 2) {
            byte* buffer = (byte*)queue1.readBuffer();
            file.write(buffer, 256); // Write 512 bytes (2 blocks of 256 bytes each)
            queue1.freeBuffer();
            buffer = (byte*)queue1.readBuffer();
            file.write(buffer, 256);
            Bytecounter += 512;
            queue1.freeBuffer();
        }
    }

    // Stop recording
    queue1.end();
    Serial.println("Recording stopped.");
    Serial.print("Packets remaining in Queue: ");
    Serial.println(queue1.available());

    // Write WAV header with correct file size
    writeOutHeader(Bytecounter);

    // Close the file
    file.close();
    Serial.println("Recording saved to SD card.");
}

void writeOutHeader(unsigned long recByteSaved) {
    Subchunk2Size = recByteSaved;
    ChunkSize = Subchunk2Size + 36;
    file.seek(0);
    file.write("RIFF");
    file.write(ChunkSize & 0xff);
    file.write((ChunkSize >> 8) & 0xff);
    file.write((ChunkSize >> 16) & 0xff);
    file.write((ChunkSize >> 24) & 0xff);
    file.write("WAVE");
    file.write("fmt ");
    file.write(Subchunk1Size & 0xff);
    file.write((Subchunk1Size >> 8) & 0xff);
    file.write((Subchunk1Size >> 16) & 0xff);
    file.write((Subchunk1Size >> 24) & 0xff);
    file.write(AudioFormat & 0xff);
    file.write((AudioFormat >> 8) & 0xff);
    file.write(numChannels & 0xff);
    file.write((numChannels >> 8) & 0xff);
    file.write(sampleRate & 0xff);
    file.write((sampleRate >> 8) & 0xff);
    file.write((sampleRate >> 16) & 0xff);
    file.write((sampleRate >> 24) & 0xff);
    file.write(byteRate & 0xff);
    file.write((byteRate >> 8) & 0xff);
    file.write((byteRate >> 16) & 0xff);
    file.write((byteRate >> 24) & 0xff);
    file.write(blockAlign & 0xff);
    file.write((blockAlign >> 8) & 0xff);
    file.write(bitsPerSample & 0xff);
    file.write((bitsPerSample >> 8) & 0xff);
    file.write("data");
    file.write(Subchunk2Size & 0xff);
    file.write((Subchunk2Size >> 8) & 0xff);
    file.write((Subchunk2Size >> 16) & 0xff);
    file.write((Subchunk2Size >> 24) & 0xff);
    file.close();

    Serial.println("Header written");
    Serial.print("Subchunk2: ");
    Serial.println(Subchunk2Size);
}

*/


///////////////////////////////////////////////////////////////////////////////
// LAEQ COMPUTATION
///////////////////////////////////////////////////////////////////////////////

#define sq(x) ({ typeof(x) _x = (x); _x * _x; })


void sendFloat(float value) {
  Serial1.write(0x02); // Start marker
  Serial1.write((byte*)&value, sizeof(value));
  Serial1.write(0x03); // End marker
}

void LAeqComputationTask(void *pvParameters) {
  AudioMemory(60);
  Serial.begin(115200);
  Serial1.begin(115200);

  pink1.amplitude(0.01);  // for testing filters
  mixer1.gain(0,1);  // microphone level
  mixer1.gain(1,0);  // pink noise level

  // Setup filter center frequencies and Q (1 octave)
  filter31.frequency(31.5);
  filter31.resonance(1.414);
  filter63.frequency(63);
  filter63.resonance(1.414);
  filter125.frequency(125);
  filter125.resonance(1.414);
  filter250.frequency(250);
  filter250.resonance(1.414);
  filter500.frequency(500);
  filter500.resonance(1.414);
  filter1000.frequency(1000);
  filter1000.resonance(1.414);
  filter2000.frequency(2000);
  filter2000.resonance(1.414);
  filter4000.frequency(4000);
  filter4000.resonance(1.414);
  filter8000.frequency(8000);
  filter8000.resonance(1.414);

  uint8_t prescale = 20;  
  float offset = 88.61;    
  float Awt = 0;
  float AwtAccum = 0;
  float LAeq = 0;
  uint8_t n = 0; 
  unsigned long sampletime = 100;  
  unsigned long starttime;
  //bool shouldContinue = true;

  while(1) {
    // Wait for "Ready" signal from ESP32
    if (Serial1.read() == 'R') {
      n = 0;
      AwtAccum = 0;
      starttime = millis();  


      while((millis() - starttime) < sampletime) {
        if (rms31.available() && rms63.available() && rms125.available() &&
            rms250.available() && rms500.available() && rms1000.available() &&
            rms2000.available() && rms4000.available() && rms8000.available()) {

          Awt = 0;
          Awt += sq(rms31.read()   * prescale * 0.01071519 );
          Awt += sq(rms63.read()   * prescale * 0.04897788 );
          Awt += sq(rms125.read()  * prescale * 0.15667510 );
          Awt += sq(rms250.read()  * prescale * 0.37153523 );
          Awt += sq(rms500.read()  * prescale * 0.69183097 );
          Awt += sq(rms1000.read() * prescale              );
          Awt += sq(rms2000.read() * prescale * 1.1481536  );
          Awt += sq(rms4000.read() * prescale * 1.1220185  );
          Awt += sq(rms8000.read() * prescale * 0.8810489  );

          AwtAccum += Awt;  
          n++;  
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);  // FreeRTOS delay
      }

      LAeq = 10 * log10f(AwtAccum / n) + offset;
      Serial.print("Sent LAeq: ");
      Serial.println(LAeq, 1);  // Report 1 decimal place
      //unsigned long Laeqtimetx = micros(); //start time for transmission
      //unsigned long LaeqStartRTT = millis(); //start time for transmission
      sendFloat(LAeq);
      //unsigned long latencytest = millis();
      //Serial.print("LAeq Sending Time: ");
      //Serial.print(micros()-Laeqtimetx);  
      //Serial.println("us");

      //while(Serial1.read()!='D'); // wait for acknowledgement byte
      //Serial.print("LAeq Tx Time: ");
      //Serial.print(millis()-LaeqStartRTT-0.325);//325us one way transmission of one byte
      //Serial.println("ms");

      //Serial.print("LAeq Tx Latency: ");
      //Serial.print(millis()-latencytest-0.325);//325us one way transmission of one byte
      //Serial.println("ms");      
       // LAeq sending Time
      //shouldContinue = false;
      //vTaskDelay(5000 / portTICK_PERIOD_MS); 
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////
// MFCC COMPUTATION
//////////////////////////////////////////////////////////////////////////////////

// MFCC related declarations
const uint8_t numFilters = 26;  // Number of Mel filter banks
const uint16_t frameSize = 1024;  // Frame size for FFT (23.2ms per frame)
const uint8_t hopSize = 10;      // Hop size for overlapping windows
const uint8_t mfccSize = 13;     // Number of MFCC coefficients
const float FS = 44100;  // Sampling rate of INMP441

arduinoMFCC mfcc(numFilters, frameSize, hopSize, mfccSize, FS);

// Memory Allocation

float *mfcc_coeffs   =(float *)malloc(numFilters * sizeof(float ));
float *rmfcc_coeffs  =(float *)malloc(mfccSize * sizeof(float ));
float **mfcc_array; // 2D array to store MFCC coefficients for each frame
float **mel_filter   = (float **)malloc(numFilters * sizeof(float *));
float **dct_matrix   = (float **)malloc(mfccSize* sizeof(float *));
float *hwindows      = (float *)malloc(frameSize* sizeof(float ));


bool firstperioddone = false; //check if the first MFCC_array is done
bool startReceived = false; // Check if the start signal has been received

int frameCount = 0;
const int totalFrames = 225; // Stop after 5.22 secs

// Clear the mfcc_array by overwriting its contents with zeros
void clearMfccArray() {
  for (int i = 0; i < totalFrames; i++) {
    for (int j = 0; j < mfccSize; j++) {
      mfcc_array[i][j] = 0.0; // Set all elements to zero
    }
  }
}

// Clear the mfcc_coeffs array by overwriting its contents with zeros
void clearMfccCoeffs() {
  for (int i = 0; i < numFilters; i++) {
    mfcc_coeffs[i] = 0.0; // Set all elements to zero
  }
}

// Clear the rmfcc_coeffs array by overwriting its contents with zeros
void clearRmfccCoeffs() {
  for (int i = 0; i < mfccSize; i++) {
    rmfcc_coeffs[i] = 0.0; // Set all elements to zero
  }
}

void clearBuffer(float* buffer, int size) {
  for (int i = 0; i < size; i++) {
    buffer[i] = 0.0;
  }
}

uint8_t calculateChecksum(float *data, size_t length) {
  uint8_t checksum = 0;
  byte *byteData = (byte *)data;
  for (size_t i = 0; i < length * sizeof(float); i++) {
    checksum ^= byteData[i];
  }
  return checksum;
}


void MFCCComputationTask(void *pvParameters) {
  Serial.begin(115200);
  Serial1.begin(115200); // Initialize UART for communication with ESP32

  // Memory Initialization
  for (int i = 0; i < numFilters; i++) {
    mel_filter[i] = (float *)malloc((frameSize / 2) * sizeof(float));
  }
  for (int i = 0; i < mfccSize; i++) {
    dct_matrix[i] = (float *)malloc(numFilters * sizeof(float));
  }    

  mfcc.create_hamming_window(frameSize, hwindows);
  mfcc.create_mel_filter_bank(FS, numFilters, frameSize, mel_filter); 
  mfcc.create_dct_matrix(dct_matrix);

  // Allocate memory for the MFCC array
  mfcc_array = (float **)malloc(totalFrames * sizeof(float *));
  for (int i = 0; i < totalFrames; i++) {
    mfcc_array[i] = (float *)malloc(mfccSize * sizeof(float));
  }

  // Initialize audio system
  AudioMemory(60);

  // Begin audio input
  queue.begin();

  while (1) {



        /*if (!startReceived) {
          if (Serial1.available()) {
            char startChar = Serial1.read();
            if (startChar == 'R') {
              startReceived = true;
              Serial.println("Start signal received. Beginning processing.");
            }
          }
          return; // Exit the loop if the start signal has not been received
        }*/
        if (queue.available() >=8  && frameCount < totalFrames) {

            //Memory allocation for the sampled audio from mic
            const int packet_size = 128;
            const int num_packets = 8;
            int16_t* combinedSamples = (int16_t*)malloc(packet_size * num_packets * sizeof(int16_t));

            for (int i = 0; i < num_packets - 1; ++i) {
                int16_t *samples = queue.readBuffer();
                memcpy(combinedSamples + (i * packet_size), samples, packet_size * sizeof(int16_t));
                queue.freeBuffer();
            }
            float buffer[frameSize];

            // Convert audio samples to float and normalize
            for (int i = 0; i < frameSize; i++) {
                buffer[i] = (double)combinedSamples[i] / 32768.0;
            }

            // Compute MFCC
            mfcc.pre_emphasis(frameSize, buffer);    
            mfcc.apply_hamming_window(buffer, hwindows);
            mfcc.apply_mel_filter_bank_power(frameSize, buffer);
            mfcc.apply_mel_filter_bank(numFilters, frameSize, buffer, mel_filter, mfcc_coeffs);
            mfcc.apply_dct(mfccSize, numFilters, frameSize, mel_filter, mfcc_coeffs, rmfcc_coeffs, dct_matrix);

            // Store the coefficients in the array
            for (int i = 0; i < mfccSize; i++) {
                mfcc_array[frameCount][i] = rmfcc_coeffs[i];
            }

            //disregards the first 5 seconds computed MFCC
            frameCount++;
            
            if (frameCount >= totalFrames){
                //unsigned long STARTtimetx = millis();

                // Transmit the MFCC array via UART to ESP32
                Serial1.write(0x01); // Start byte
                Serial1.write((uint8_t)(totalFrames >> 8)); // Send the high byte first
                Serial1.write((uint8_t)totalFrames); // Send the low byte
                Serial1.write((uint8_t)mfccSize); // Number of coefficients per frame


                for (int i = 0; i < totalFrames; i++) {
                    uint8_t checksum = calculateChecksum(mfcc_array[i], mfccSize);
                    Serial1.write((byte *)&checksum, sizeof(checksum)); // Send checksum
                    for (int j = 0; j < mfccSize; j++) {
                        Serial1.write((byte *)&mfcc_array[i][j], sizeof(mfcc_array[i][j])); // Send each MFCC coefficient as bytes
                        vTaskDelay(1 /portTICK_PERIOD_MS);
                    }
                }
                Serial1.write(0x04); // End byte 
                //unsigned long mfcclatency = micros();
                //Serial.println("Sent MFCC array");
                //Serial.print("MFCC Sending Time: ");
                //Serial.print(millis()-STARTtimetx); //sending time for a 5 second MFCC
                //Serial.println("ms");

               /// while(Serial1.read() != 'F'); //wait for acknowledgement byte
                //Serial.print("MFCC Tx Time: ");
                //Serial.print(millis()-STARTtimetx-0.325); //325us one way transmission of one byte
                //Serial.println("ms");

                //Serial.print("MFCC Latency: ");
                //Serial.print(micros()-mfcclatency-325); //325us one way transmission of one byte
                //Serial.println("us");     

                frameCount = 0;
                clearMfccArray();
               Serial.println("Next 5 secs"); 
            } 
            clearMfccCoeffs();
            clearRmfccCoeffs();
            clearBuffer(buffer, frameSize);    
            free(combinedSamples);
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////
// Check for Restart Request
/////////////////////////////////////////////////////////////////////////////////

void CheckForRestartTask(void *pvParameters) {
while(1){
  if (Serial1.available()) {
    char receivedChar = Serial1.read();
    if (receivedChar == 'U') {
      Serial.println("restart signal received");
        // Restart the mfcc and laeq tasks
        vTaskSuspend(xMFCCComputationTask); // Suspend the tasks
        vTaskDelete(xMFCCComputationTask);  // Delete the tasks

        vTaskSuspend(xLAeqComputationTask); // Suspend the tasks
        vTaskDelete(xLAeqComputationTask);  // Delete the tasks


        // Recreate the tasks
        xTaskCreate(MFCCComputationTask, "MFCCComTask", 4096, NULL, 2, &xMFCCComputationTask);
        xTaskCreate(LAeqComputationTask, "LAeqTask", 4096, NULL, 2, &xLAeqComputationTask);
        Serial.println("Tasks restarted.");
    }
  }
  vTaskDelay(500 / portTICK_PERIOD_MS); // Delay to prevent busy-waiting
}
}
///////////////////////////////////////////////////////////////////////////////////////
 


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println("Booted");
    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    Serial.println(PSTR("\r\nBooting FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));

    xTaskCreate(CheckForRestartTask, "RestartTask", 2048, NULL, 2, &xRestartTask);
    xTaskCreate(LAeqComputationTask, "LAeqTask", 4096, NULL, 2, &xLAeqComputationTask);
    //xTaskCreate(AudioRecordingTask, "AudioRecTask", 8192, NULL, 1, &xAudioRecordingTask);
    xTaskCreate(MFCCComputationTask, "MFCCComTask", 4096, NULL, 2, &xMFCCComputationTask);

    Serial.println("setup(): starting scheduler...");
    Serial.flush();

    vTaskStartScheduler();
}

void loop() {}
