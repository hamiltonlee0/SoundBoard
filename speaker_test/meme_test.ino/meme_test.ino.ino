/*Used Libraries*/
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include "lpf.h" //Low pass Filter designed using TFilter

/*Audio connections via Teensy Audio Library*/
AudioPlaySdWav           playSdWav1;     //xy=69,226
AudioFilterFIR           fir1;           //xy=259,152
AudioEffectBitcrusher    bitcrusher2;    //xy=351,277
AudioEffectBitcrusher    bitcrusher1;    //xy=433,142
AudioMixer4              mixer1;         //xy=613,224
AudioOutputI2S           i2s1;           //xy=808,231
AudioConnection          patchCord1(playSdWav1, 0, fir1, 0);
AudioConnection          patchCord2(playSdWav1, 1, bitcrusher2, 0);
AudioConnection          patchCord3(fir1, bitcrusher1);
AudioConnection          patchCord4(bitcrusher2, 0, mixer1, 1);
AudioConnection          patchCord5(bitcrusher1, 0, mixer1, 0);
AudioConnection          patchCord6(mixer1, 0, i2s1, 0);

/*Potentiometer PIN*/
const int POT_PIN = 20;

/*Volume variable*/
volatile uint16_t volume = 8;

/*Gain for mixer*/
float gain = 0;

/*Setup for processor*/
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
  
  /*LPF bit depth - 3 bit audio*/
  bitcrusher1.bits(2);

  /*HPF bit depth - 2 bit audio*/
  bitcrusher2.bits(3);

  /*Initialize the Filter*/
  fir1.begin(LPFilter, 200);
  
  /*Start reading from the SD card*/
  Serial.println("Playing");
  playSdWav1.play("TEST.WAV");
  //delay(5); //Wait for file to start playing

}

void loop() {

  /*Volume Update*/
  volume = (analogRead(POT_PIN) + 1) / 126;

  /*Mixer Gain*/
  gain = 1 - (float)volume/(float)8;
  mixer1.gain(0, 0.75*gain); //Bass has worse bit crushing and is louder
  mixer1.gain(1, 0.5*gain); //Slightly higher resolution upper register
  //Serial.println(0.75*(float)volume/(float)8); //HPF gain
}
