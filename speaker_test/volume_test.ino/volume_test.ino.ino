/*Used Libraries*/
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

/*Audio connections via Teensy Audio Library*/
AudioPlaySdWav           playSdWav1;     //Wav file we are playing
AudioRecordQueue         queue1;         //Play Wav file to record queue
AudioPlayQueue           queue2;         //Transfer record queue to play queue
AudioOutputI2S           i2s1;           //Play queue to i2s
AudioConnection          patchCord1(playSdWav1, 0, queue1, 0);
AudioConnection          patchCord2(queue2, 0, i2s1, 0);

/*Potentiometer PIN*/
const int POT_PIN = 20;

/*Volume variable*/
volatile uint16_t volume = 8;

/*Setup for the processor*/
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

  /*Potentiometer input to record volume*/
  pinMode(POT_PIN, INPUT);

  /*Start reading from the SD card*/
  Serial.println("Playing");
  queue1.begin();
  playSdWav1.play("TEST.WAV");
  //delay(5); //Wait for file to start playing
}

/*Volume adjusted copy to play queue*/
void copyToBuff(int16_t outBuff[], int16_t inBuff[]){
  for(int i = 0; i < 128; i++){
    outBuff[i] = inBuff[i] >> (volume);
  }
}


/*while(1) loop for the processor*/
void loop() {
  /*If there is an available audio block*/
  if(playSdWav1.isPlaying() && (queue1.available() >= 1)){
    copyToBuff(queue2.getBuffer(), queue1.readBuffer());
    queue2.playBuffer();
    queue1.freeBuffer();
  }
  
  /*Update the Volume Variable*/
  volume = (analogRead(POT_PIN) + 1) / 126;
  Serial.println(volume);
}
