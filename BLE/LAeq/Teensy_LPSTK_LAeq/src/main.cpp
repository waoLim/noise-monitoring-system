#include <Arduino.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <TimeLib.h>

// GUItool: begin automatically generated code
AudioSynthNoisePink pink1; 
AudioInputI2S i2s1; 
AudioMixer4 mixer1; 
AudioFilterStateVariable filter31, filter63, filter125, filter250, filter500, filter1000, filter2000, filter4000, filter8000; 
AudioAnalyzeRMS rms31, rms63, rms125, rms250, rms500, rms1000, rms2000, rms4000, rms8000; 
AudioConnection patchCord1(pink1, 0, mixer1, 1);
AudioConnection patchCord2(i2s1, 0, mixer1, 0);
AudioConnection patchCord3(mixer1, 0, filter31, 0);
AudioConnection patchCord4(mixer1, 0, filter63, 0);
AudioConnection patchCord5(mixer1, 0, filter125, 0);
AudioConnection patchCord6(mixer1, 0, filter250, 0);
AudioConnection patchCord7(mixer1, 0, filter500, 0);
AudioConnection patchCord8(mixer1, 0, filter1000, 0);
AudioConnection patchCord9(mixer1, 0, filter2000, 0);
AudioConnection patchCord10(mixer1, 0, filter4000, 0);
AudioConnection patchCord11(mixer1, 0, filter8000, 0);
AudioConnection patchCord12(filter31, 1, rms31, 0);
AudioConnection patchCord13(filter63, 1, rms63, 0);
AudioConnection patchCord14(filter125, 1, rms125, 0);
AudioConnection patchCord15(filter250, 1, rms250, 0);
AudioConnection patchCord16(filter500, 1, rms500, 0);
AudioConnection patchCord17(filter1000, 1, rms1000, 0);
AudioConnection patchCord18(filter2000, 1, rms2000, 0);
AudioConnection patchCord19(filter4000, 1, rms4000, 0);
AudioConnection patchCord20(filter8000, 1, rms8000, 0);
// GUItool: end automatically generated code

const byte START_MARKER = 0xA5;
const byte END_MARKER = 0x5A;

uint8_t prescale = 20;
float offset = 93.56;
float Awt = 0;
float AwtAccum = 0;
float LAeq = 0;
uint8_t n = 0;
unsigned long sampletime = 100; // Reduced sample time for faster processing
unsigned long lastMillis = 0;

void setup() {
  AudioMemory(20);
  Serial.begin(115200);
  Serial3.begin(115200); // Using Serial3 for CC1352R communication
  setTime(8, 0, 0, 1, 1, 2024); // Set the initial time to 08:00:00 on January 1, 2024

  pink1.amplitude(0.01);
  mixer1.gain(0, 1);
  mixer1.gain(1, 0);

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
}

byte calculateChecksum(const byte* data, size_t length) {
  byte checksum = 0;
  for (size_t i = 0; i < length; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

void sendLAeq(float value) {
  byte* valueBytes = (byte*)&value;
  byte packet[7]; // START(1) + float(4) + checksum(1) + END(1)
  
  packet[0] = START_MARKER;
  memcpy(&packet[1], valueBytes, 4);
  packet[5] = calculateChecksum(valueBytes, 4);
  packet[6] = END_MARKER;
  
 Serial3.write(packet, 7);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastMillis >= 1000) { // Send LAeq every seconds
    lastMillis = currentMillis;
    n = 0;
    AwtAccum = 0;
    unsigned long starttime = millis();

    while ((millis() - starttime) < sampletime) {
      if (rms31.available() && rms63.available() && rms125.available() && rms250.available() &&
          rms500.available() && rms1000.available() && rms2000.available() && rms4000.available() &&
          rms8000.available()) {

        Awt = 0;
        Awt += sq(rms31.read() * prescale * 0.01071519);
        Awt += sq(rms63.read() * prescale * 0.04897788);
        Awt += sq(rms125.read() * prescale * 0.15667510);
        Awt += sq(rms250.read() * prescale * 0.37153523);
        Awt += sq(rms500.read() * prescale * 0.69183097);
        Awt += sq(rms1000.read() * prescale);
        Awt += sq(rms2000.read() * prescale * 1.1481536);
        Awt += sq(rms4000.read() * prescale * 1.1220185);
        Awt += sq(rms8000.read() * prescale * 0.8810489);

        AwtAccum += Awt;
        n++;
      }
      delay(1);
    }

    LAeq = 10 * log10f(AwtAccum / n) + offset;
    sendLAeq(LAeq); // Send LAeq with START_MARKER, END_MARKER, and Checksum to CC1352R
    Serial.print("Sent LAeq: ");
    Serial.println(LAeq, 2);
  }
}