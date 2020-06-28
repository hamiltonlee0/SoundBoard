/**
 * @file buttonpad_test.ino
 * @author Hamilton Lee
 *
 * This file tests i2s input for teensy 3.6 board and tests
 * hardware for SPH0645 Mic from adafruit
 * 
 * Product: https://learn.adafruit.com/adafruit-i2s-mems-microphone-breakout?view=all
 * 
 */
 
/*Libraries Used*/
#include <Wire.h>

/*Audio library for Teensy*/
#include <Audio.h>

/*Note that sampling frequency for our data is at 44.1kHz */
AudioInputI2S i2s1;                               //Input channel for our I2S
AudioRecordQueue queue1;                          //Queue for audio data from I2S
AudioConnection patchCord1(i2s1, 0, queue1, 0);   //Connection between i2s and the queue

uint16_t tempBuff[256]; //Audio Buffer for recording audio

const int numSamples = 4*44100;                    //Number of samples for 4 seconds of audio (44.1kHz)
int count = 0;                                   //Counter variable for each sample

/*Send the audio data through serial.
 *Written for Serial plotter to view waveform.
 *For Debugging.*/
void toSerialPlotter(){
  for(int i = 0; i < 256; i++){
    if(tempBuff[i]!=0){
      Serial.print(tempBuff[i]);
      Serial.print(" ");
      //These values set the bounds for the Serial Plotter
      Serial.print(66000);
      Serial.print(" ");
      Serial.println(62000);
    }
  }
}

/*Send the audio data through serial.
 *Written for csv data - 5 seconds of audio
 *As ordered pairs.*/
void toSerial(){
  for(int i = 0; i < 256; i++){
    if(tempBuff[i]!=0 && count < numSamples){
      Serial.print(count);
      Serial.print(", ");
      Serial.println(tempBuff[i]);
      count++;
    }
    else if(count == numSamples){
      queue1.end(); //Stop reading values
      while(1); //Program stops running when 5 seconds of audio is sent
    }
  }
}

void setup() {
  Serial.begin(9600); // Set up serial for Teensy; We will read values to USB port
  while(!Serial){
    ; // Wait for USB to connect
  }
  delay(100);
  //Serial.println("Serial Setup Complete!");

  /*We need to allocate memory for audio samples.
   *The argument is the number of blocks with each block holding 128 samples.
   *Each block is about 2.9ms of play time.*/ 
  AudioMemory(250);
  //Serial.println("Beginning Recording");
  queue1.begin(); //Beginning Recording
}

void loop() {
  /*Check if there are one blocks of audio available or not.
   *Note that the I2S driver used records at 16 bits per sample
   *while the board reads 32 bit samples. LSBs are truncated.*/
  if(queue1.available()>=1){
    //Copy blocks to buffer
    memcpy(tempBuff, queue1.readBuffer(), 256);
    queue1.freeBuffer();
    toSerialPlotter(); //Print to Serial for Serial plotter
    //toSerial(); //Send to Serial (for csv file)
  }
}
