/*Used Libraries*/
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

/*Audio connectiosn via Teensy Audio Library*/

// GUItool: begin automatically generated code
AudioPlaySdWav           playSdWav1;     //xy=250,91
AudioOutputI2S           i2s1;           //xy=364,168
AudioConnection          patchCord1(playSdWav1, 0, i2s1, 0);
// GUItool: end automatically generated code

/*Read test WAV file from SD card*/
void startWavSD(const char *FILE){
  /*Begin reading the Wav file*/
  Serial.println("Playing");
  playSdWav1.play(FILE);

  /*Wait for WAV file to be read*/
  delay(5);

  /*Wait for Playing to finish*/
  while(playSdWav1.isPlaying()){
    //Idle
  }
}

void setup() {
  /*Serial Setup (for computer)*/
  Serial.begin(9600); //9600 baud rate for USB
  delay(1000);
  Serial.println("Serial Setup Complete");

  /*Audio Functions Setup*/
  AudioMemory(250); //Allocate Memory for audio samples
                    //250 is the number of audio blocks with each block holding 128 samples
                    //Each block is ~2.9ms of play time
  /*Initialize SPI SD reader with the built in SD card reader*/
  if(!SD.begin(BUILTIN_SDCARD)){
    Serial.println("Unable to access SD Card");
    while(1);
  }
  else{
    Serial.println("SD Card connected");
  }

  /*start reading Wav file from SD card*/
  startWavSD("TEST.WAV");
}

void loop() {
  // put your main code here, to run repeatedly:

}
