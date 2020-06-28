/*Libraries Used*/
#include <Wire.h> //Standard Arduino Wire library
#include <Audio.h> //Audio library for Teensy
#include <SPI.h> //SPI for SD Card
#include <SD.h> //SD functions for Teensy
#include <SerialFlash.h> //SD flash for Teensy
#include "lpf.h" //Low pass filter for the filter button

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=AUDIO VARIABLES & Connections=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

/*I2S Setup for Teensy (for MIC and AMPLIFIER)
 *Note: sampling frequency for our data is at 44.1kHz */
//Recording Connections
AudioInputI2S            i2s1;                               //Input channel for I2S MIC
AudioRecordQueue         queue1;                             //Queue for audio data from I2S MIC
AudioConnection          patchCord1(i2s1, 0, queue1, 0);     //Connection between I2S MIC and the queue

//General Play Queue
AudioPlayQueue           playQueue;
AudioOutputI2S           outputI2S;
AudioConnection          playQueueToI2S(playQueue, 0, outputI2S, 0);

//Play Audio - from SD - WAV files
AudioPlaySdWav           wavFilter;
AudioFilterFIR           firLPFSD;
AudioEffectBitcrusher    bitcrusher2SD;
AudioEffectBitcrusher    bitcrusher1SD;
AudioRecordQueue         volumeRecordSD;
AudioMixer4              mixerSD;
AudioConnection          wavToFilt(wavFilter, 0, firLPFSD, 0);
AudioConnection          wavToBC(wavFilter, 0, bitcrusher2SD, 0);
AudioConnection          firSDToBC(firLPFSD, bitcrusher1SD);
AudioConnection          BC2ToMixerSD(bitcrusher2SD, 0, mixerSD, 1);
AudioConnection          BC1ToMixerSD(bitcrusher1SD, 0, mixerSD, 0);
AudioConnection          mixerToRecordQueueSD(mixerSD, 0, volumeRecordSD, 0);

//Play Audio - from SD - RAW files
AudioPlaySdRaw           rawFilter;
AudioFilterFIR           firLPFRaw;
AudioEffectBitcrusher    bitcrusher2Raw;
AudioEffectBitcrusher    bitcrusher1Raw;
AudioRecordQueue         volumeRecordRaw;
AudioMixer4              mixerRaw;
AudioConnection          rawToFilt(rawFilter, firLPFRaw);
AudioConnection          rawToBC(rawFilter, bitcrusher2Raw);
AudioConnection          firRawToBC(firLPFRaw, bitcrusher1Raw);
AudioConnection          BC2ToMixerRaw(bitcrusher2Raw, 0, mixerRaw, 1);
AudioConnection          BC1ToMixerRaw(bitcrusher1Raw, 0, mixerRaw, 0);
AudioConnection          mixerToRecordQueueRaw(mixerRaw, volumeRecordRaw);

/*Names of the audio flies in SD card*/
const char *AudioFiles[4][4] = { {"N/A", "R11.WAV", "R7.WAV", "R3.WAV"}, {"N/A", "R10.WAV", "R6.WAV", "R2.WAV"}, {"R2.RAW", "R9.WAV", "R5.WAV", "R1.WAV"}, {"R1.RAW", "R8.WAV", "R4.WAV", "N/A"} };

/*Audio Buffer for recording audio before sending to SD Card*/
byte tempBuff[512]; 

/*Number of samples for our recorded sound*/
const int numSamples = 5*44100; //Number of samples for 5 seconds of audio

/*Counting variable for samples*/
volatile int countSamp = 0; 

/*File for recording audio*/
File rec;

/*Start recording flag*/
volatile uint8_t RECFLG = 0; //0x00: System is idle, not recording
                             //0x01: Request to record on file 1
                             //0x10: Request to record on file 2
                             //0x11: Recording

/*If system is playing a noise flag*/
volatile uint8_t ISPLAYING = 0; //0x000: No sound is playing
                                //0x001: volumeRecordSD is playing
                                //0x010: volumeRecordRaw is playing

/*Volume variable*/
volatile uint16_t volume = 8; //Default to 0 volume

/*Use Filter Variable*/
volatile bool USEFILT = 0; //Default is off

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=BUTTONPAD VARIABLES=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/
/*For Switch Debouncing Consecutive Presses*/
#define CONSEC_PRESS 5 //~5ms of being pressed

/*Potentiometer PIN*/
const int POT_PIN = 20;

/*LED Button Pressed PIN*/
//GREEN Led through 330 Ohm Reisistor
const int LED_PIN = 33;

/*LED Column pins*/
static uint8_t LED_COL[4] = {6, 4, 2, 0};

/*RED Row pins*/
static uint8_t RED_ROW[4] = {17, 16, 15, 14};

/*GREEN Row pins*/
static uint8_t GREEN_ROW[4] = {8, 10, 11, 12};

/*BLUE Row pins*/
static uint8_t BLUE_ROW[4] = {29, 30, 31, 32};

/*SWITCH Column pins*/
static uint8_t SWITCH_COL[4] = {7, 5, 3, 1};

/*SWITCH Row pins*/
static uint8_t SWITCH_ROW[4] = {25, 26, 27, 28};

/*Matrix representing number of cycles a switch has been pressed in a row*/
volatile uint16_t SWITCH_COUNT[4][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};

/*Matrix containing flags for a switch being pressed*/
volatile bool SWITCH_PRESS[4][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};

/*Matrices representing if a given LED should be on or off*/
byte RED_ON[4][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
byte GREEN_ON[4][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
byte BLUE_ON[4][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};

/*Variables to store which sound effect is currently playing*/
volatile byte onI = 3; //LED turns blue for the sound effect that is playing
volatile byte onJ = 3; //This is used to reset that

/*Zero matrix to reset the above matirces*/
byte ZEROS[4][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=HELPER FUNCTIONS=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/

/*Starts Recording:
 *Board supports two recorded 5 second sound bites.
 *swtnum = 1 records to the first audio file
 *swtnum = 2 records to the second audio file*/
void startRecording(uint8_t swtnum){
  /*Recording 1*/
  if(swtnum == 1){
    //remove existing recording
    if(SD.exists("R2.RAW")){
      SD.remove("R2.RAW");
    }
    rec = SD.open("R2.RAW", FILE_WRITE);
  }
  /*Recording 2*/
  else{
    if(SD.exists("R1.RAW")){
      SD.remove("R1.RAW");
    }
    rec = SD.open("R1.RAW", FILE_WRITE);
  }
  /*If successful, begin recording*/
  if(rec){
    Serial.println("Recording Started");
    RECFLG = 3; //Set the recording flag to "Recording"
    queue1.begin();
  }
  else{
    Serial.println("Recording Failed");
    while(1);
  }
}

/*Write recorded mic data to the SD card*/
void record(void){
  /*If we have the sufficient amount of samples, end recording and reset variables*/
  if(countSamp >= numSamples){
    queue1.end(); //End the buffer
    while (queue1.available() > 0) { //Flush the rest of the buffer
      rec.write((byte*)queue1.readBuffer(), 256);
      queue1.freeBuffer();
    }
    queue1.clear(); //Clear buffer
    rec.close(); //Close the file
    Serial.println("Recording Finished");
    countSamp = 0;
    RECFLG = 0;
    PIT_TCTRL1 |= PIT_TCTRL_TIE; //Re-enable interrupts for the pit1 timer
    PIT_TCTRL0 |= PIT_TCTRL_TIE; //Re-enable interrupts for the pit0 timer
    
    digitalWrite(LED_COL[0], HIGH);
    digitalWrite(LED_COL[1], HIGH);
    defaultLED(); //Set the LED scheme the the original colors
    digitalWrite(LED_PIN, LOW);
    return;
  }
  
  /*Check if there are one blocks of audio available or not.
   *Note that the I2S driver used records at 16 bits per sample
   *while the board reads 32 bit samples. LSBs are truncated.
   *256 samples are recorded every single call of record*/
  if(queue1.available() >= 2){
    memcpy(tempBuff, queue1.readBuffer(), 256);
    queue1.freeBuffer();
    memcpy(tempBuff + 256, queue1.readBuffer(), 256);
    queue1.freeBuffer();
    
    /*Write the bytes to the SD card*/
    rec.write(tempBuff, 512);

    /*Increment number of samples read*/
    countSamp += 256;
  }
}

/*Clears LEDs on button pad*/
void zeroLED(){
  /*Zero all the LED matrices*/
  memcpy(&RED_ON[0][0], &ZEROS[0][0], 4*4*sizeof(ZEROS[0][0]));
  memcpy(&GREEN_ON[0][0], &ZEROS[0][0], 4*4*sizeof(ZEROS[0][0]));
  memcpy(&BLUE_ON[0][0], &ZEROS[0][0], 4*4*sizeof(ZEROS[0][0]));
}

/*Default LED configuration*/
void defaultLED(){
  /*Zero the LEDs*/
  zeroLED();

  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      GREEN_ON[i][j] = 1;
    }
  }
  
  GREEN_ON[3][3] = 0;
  RED_ON[3][3] = 1;
  if(USEFILT){ //Change a button color if filter is in use
    BLUE_ON[3][3] = 1;
  }
  
  GREEN_ON[0][0] = 0;
  BLUE_ON[0][0] = 1;
  
  GREEN_ON[1][0] = 0;
  BLUE_ON[1][0] = 1;
}

/*Set the row values to low*/
void rowLow(){
  for(int i = 0; i< 4; i++){
    digitalWrite(RED_ROW[i], LOW);
    digitalWrite(GREEN_ROW[i], LOW);
    digitalWrite(BLUE_ROW[i], LOW);
  }
}

/*set the column values to High*/
void columnHigh(){
  for(int i = 0; i< 4; i++){
    digitalWrite(LED_COL[i], HIGH);
  }
}

/*Volume adjusted - copy a record queue to play queue*/
void copyToBuff(int16_t outBuff[], int16_t inBuff[]){
  for(int i = 0; i < 128; i++){
    if(ISPLAYING == 2){
      outBuff[i] = inBuff[i] << 2; //adjust for mic softness
      outBuff[i] = outBuff[i] >> (volume);
    }
    else{
      outBuff[i] = inBuff[i] >> (volume); //each >> is around 6dB less
    }
  }
}

/*Performs the switch action for a given switch*/
void switchAction(int j, int i){
  /*If a recording button is pressed*/
  if(((j == 0) && (i == 0))||((j == 1) && (i == 0))){
    /*Cannot record if a sound effect is playing*/
    if(ISPLAYING > 0){
      return;
    }
    PIT_TCTRL1 &= ~(PIT_TCTRL_TIE); //Disable interrupts for the PIT1 timer, don't check buttons                               
    PIT_TCTRL0 &= ~(PIT_TCTRL_TIE); //Disable LED updates
    RECFLG = 1 + j; //Set the record flag accordingly
    if(RECFLG == 1){ //Clear other flags so no other recording starts right after
      SWITCH_COUNT[1][0] = 0;
      SWITCH_PRESS[1][0] = 0;
    }
    else{
      SWITCH_COUNT[0][0] = 0;
      SWITCH_PRESS[0][0] = 0;
    }
  }
  /*If filter button is pressed*/
  else if((j == 3) && (i == 3)){
    USEFILT = !USEFILT;
    defaultLED(); //Reset the LED colors
    /*Filter is on*/
    if(USEFILT){
      /*Filter On color scheme*/
      if(ISPLAYING > 0){
        digitalWrite(BLUE_ROW[onJ], LOW);
        digitalWrite(RED_ROW[onJ], HIGH);
      }
      /*Wav Files*/
      firLPFSD.begin(LPFilter, 200); //Start the filter
      bitcrusher1SD.bits(2);    //Low frequency portion
      bitcrusher2SD.bits(3);    //High frequency portion
      mixerSD.gain(0, 0.5); //Set gain for both channels
      mixerSD.gain(1, 0.5); 

      /*Raw Files*/
      firLPFRaw.begin(LPFilter, 200); //Start the filter
      bitcrusher1Raw.bits(7);    //Low frequency portion
      bitcrusher2Raw.bits(8);    //High frequency portion
      mixerRaw.gain(0, 0.5); //Set gain for both channels
      mixerRaw.gain(1, 0.5);
    }
    /*Filter is off*/
    else{
      /*Filter On color scheme*/
      if(ISPLAYING > 0){
        digitalWrite(RED_ROW[onJ], LOW);
        digitalWrite(BLUE_ROW[onJ], HIGH);
      }
      /*Wav Files*/
      firLPFSD.end(); //Filter Off
      bitcrusher1SD.bits(0); //0 volume from first channel of audio
      bitcrusher2SD.bits(16); //passthrough from second channel of audio
      mixerSD.gain(1, 1.0); //This channel is on
      mixerSD.gain(0, 0.0); //This channel is Off

      /*Raw Files*/
      firLPFRaw.end(); //Filter Off
      bitcrusher1Raw.bits(0); //0 volume from first channel of audio
      bitcrusher2Raw.bits(16); //passthrough from second channel of audio
      mixerRaw.gain(1, 1.0); //This channel is on
      mixerRaw.gain(0, 0.0); //This channel is Off
    }
  }
  /*Otherwise start a sound effect*/
  else{
    /*Disable matrix scan interrupt for LEDs*/
    NVIC_DISABLE_IRQ(IRQ_PIT_CH1);
    if(ISPLAYING > 0){
      /*End every possible audio audio file*/
      wavFilter.stop();
      rawFilter.stop();
      /*Clear all the audio record queue and flush the buffers*/
      volumeRecordSD.end(); //end record queue
      volumeRecordRaw.end();
      if(ISPLAYING == 1){
        while(volumeRecordSD.available() > 0){
          copyToBuff(playQueue.getBuffer(), volumeRecordSD.readBuffer()); //Switch between buffer (volume adjust)
          playQueue.playBuffer();
          volumeRecordSD.freeBuffer();
        }
      }
      else{
        while(volumeRecordRaw.available() > 0){
          copyToBuff(playQueue.getBuffer(), volumeRecordRaw.readBuffer()); //Switch between buffer (volume adjust)
          playQueue.playBuffer();
          volumeRecordRaw.freeBuffer();
        }
      }
      volumeRecordSD.clear(); //clear the record queues
      volumeRecordRaw.clear();
      /*Turn off previous LED*/
      digitalWrite(LED_COL[onI], HIGH);
      digitalWrite(BLUE_ROW[onJ], LOW);
      digitalWrite(RED_ROW[onJ], LOW);
    }
    /*set LED matrix accordingly*/
    rowLow();
    columnHigh();
    digitalWrite(LED_COL[i], LOW);
    if(!USEFILT){
      digitalWrite(BLUE_ROW[j], HIGH);
    }
    else{
      digitalWrite(RED_ROW[j], HIGH);
    }
    /*Keep track of which LED is on for the specific sound effect*/
    onI = i;
    onJ = j;
    /*If the played file is a RAW file; not WAV*/
    if((i == 0) && (j == 2 || j == 3)){
      volumeRecordRaw.begin();
      Serial.println(AudioFiles[j][i]);
      rawFilter.play(AudioFiles[j][i]);
      delay(5); //Give some time for the wav file to load
      ISPLAYING = 2; //Set the playing mode flag
    }
    /*If is a WAV file*/
    else{
      volumeRecordSD.begin();
      wavFilter.play(AudioFiles[j][i]);
      delay(5); //Give some time for the wav file to load
      ISPLAYING = 1; //Set the playing mode flag
    }
  } 
}

void setup() {
  /*Serial Setup (for computer/debugging)*/
  Serial.begin(9600); //9600 baud rate for USB
  delay(1000);
  Serial.println("Serial Setup Complete");

  /*LED and button Setup*/
  for(int i = 0; i < 4; i++){
    //LED driving pins are outputs
    pinMode(LED_COL[i], OUTPUT);
    pinMode(RED_ROW[i], OUTPUT);
    pinMode(GREEN_ROW[i], OUTPUT);
    pinMode(BLUE_ROW[i], OUTPUT);

    //Switch columns are outputs, rows are inputs active low
    pinMode(SWITCH_COL[i], OUTPUT);
    pinMode(SWITCH_ROW[i], INPUT_PULLUP);

    //LED connections are default HIGH
    digitalWrite(LED_COL[i], HIGH);
    digitalWrite(RED_ROW[i], HIGH);
    digitalWrite(GREEN_ROW[i], HIGH);
    digitalWrite(BLUE_ROW[i], HIGH);

    //Switch columns are default HIGH
    digitalWrite(SWITCH_COL[i], HIGH);
  }

  /*Default LED settings*/
  defaultLED();
  
  /*Setup potentiometer as an input*/
  pinMode(POT_PIN, INPUT);

  /*LED_PIN (test LED) is an output*/
  pinMode(LED_PIN, OUTPUT);
  
  /*Filter, bitcrusher, and mixer settings*/
  firLPFSD.end(); //Filter Off
  bitcrusher1SD.bits(0); //0 volume from first channel of audio
  bitcrusher2SD.bits(16); //passthrough from second channel of audio
  mixerSD.gain(1, 1.0); //This channel is on
  mixerSD.gain(0, 0.0); //This channel is Off

  firLPFRaw.end(); //Filter Off
  bitcrusher1Raw.bits(0); //0 volume from first channel of audio
  bitcrusher2Raw.bits(16); //passthrough from second channel of audio
  mixerRaw.gain(1, 1.0); //This channel is on
  mixerRaw.gain(0, 0.0); //This channel is Off
  
  /*Audio Functions Setup*/
  AudioMemory(500); //Allocate Memory for audio samples
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

  /*Set up Interrupts for Debouncing Timer and Mode Update Timer*/
        /*Looking at the preprocessor macros, pit1 number is
         *larger than all the I2S numbers so no need to change priority*/
  /*Note: 60Mhz Counter*/

  //Enable Interrupts for PIT (Periodic Interrupt Timer)
  NVIC_ENABLE_IRQ(IRQ_PIT_CH1);
  NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
  //Clock Gating to the PIT module
  SIM_SCGC6 |= SIM_SCGC6_PIT; 

  //Enable Clock for PIT timers
  PIT_MCR &= ~(1 << 1);
  //1 ms timer 
  PIT_LDVAL1 = 0xEA60;
  //1 ms timer
  PIT_LDVAL0 = 0xEA60; 
  //Enable Interrupts and start timer
  PIT_TCTRL1 |= PIT_TCTRL_TIE | PIT_TCTRL_TEN;
  PIT_TCTRL0 |= PIT_TCTRL_TIE | PIT_TCTRL_TEN;

}

void loop() {
  /*Recording request generated*/
  if((RECFLG == 1 || RECFLG == 2) && ISPLAYING == 0){      
    /*Set LED Matrix Colors accordingly*/
    rowLow();
    if(RECFLG == 1){
      digitalWrite(LED_COL[0], LOW);
      digitalWrite(RED_ROW[0], HIGH);
    }
    else{
      digitalWrite(LED_COL[0], LOW);
      digitalWrite(RED_ROW[1], HIGH);
    }
    startRecording(RECFLG);
  }
  /*System is currently recording*/
  else if(RECFLG == 3){
    record();
  }
  /*Play Audio routine*/
  else if(ISPLAYING == 1){
    if(wavFilter.isPlaying() && (volumeRecordSD.available() >= 1)){ //If the wav is still playing
      copyToBuff(playQueue.getBuffer(), volumeRecordSD.readBuffer()); //put the recorded samples into the play queue (adjust for volume)
      playQueue.playBuffer();
      volumeRecordSD.freeBuffer();
    } 
      else if (!wavFilter.isPlaying()){ //If the audio clip is finished, clear the flag and buffers
        volumeRecordSD.end(); //end record queue
        volumeRecordSD.clear(); //clear the record queue
        ISPLAYING = 0; //clear the flag
        digitalWrite(LED_COL[onI], HIGH); //reset LED matrix
        digitalWrite(BLUE_ROW[onJ], HIGH);
        digitalWrite(RED_ROW[onJ], HIGH);
        NVIC_ENABLE_IRQ(IRQ_PIT_CH1); //renable interrupts for matrix scan LED
      }
  }
  else if(ISPLAYING == 2){
    if(rawFilter.isPlaying() && (volumeRecordRaw.available() >= 1)){ //If the raw is still playing
      copyToBuff(playQueue.getBuffer(), volumeRecordRaw.readBuffer()); //put the recorded samples into the play queue (adjust for volume)
      playQueue.playBuffer();
      volumeRecordRaw.freeBuffer();
    } 
    else if (!rawFilter.isPlaying()){ //If the audio clip is finished, clear the flag and buffers
      volumeRecordRaw.end(); //end record queue
      volumeRecordRaw.clear(); //clear the record queue
      ISPLAYING = 0; //clear the flag
      digitalWrite(LED_COL[onI], HIGH); //reset LED matrix
      digitalWrite(BLUE_ROW[onJ], HIGH);
      digitalWrite(RED_ROW[onJ], HIGH);
      NVIC_ENABLE_IRQ(IRQ_PIT_CH1); //renable interrupts for matrix scan LED
    }
  }
  /*Switch flag routines*/
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      /*If user has released the switch, clear the flag and perform the button action*/
      if(SWITCH_PRESS[j][i] == 1 && SWITCH_COUNT[j][i] == 0){
        SWITCH_PRESS[j][i] = 0; //Clear the button
        switchAction(j, i);
        goto endLoop; //A button Detected
      }
    }
  }
  endLoop:
  /*Update volume variable*/
  volume = (analogRead(POT_PIN) + 1) / 126;
  if(volume == 8){
    volume = 16; //True 0 sound
  }
}

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=INTERRUPT SERVICE ROUTINES=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/
/*Interrupt Service Routine for PIT1 Timer for Debouncing (Every ~1ms)*/
void pit0_isr(void){
  /*Matrix Scan on buttonpad switches*/
  for(int i = 0; i < 4; i++){ //For each column
    digitalWrite(SWITCH_COL[i], LOW); //Set the column to LOW
    for(int j = 0; j < 4; j++){ //For each row
      if(!digitalRead(SWITCH_ROW[j])){ //If we detect a press
        SWITCH_COUNT[j][i]++;
        /*If we read correct amount of presses*/
        if(SWITCH_COUNT[j][i] > CONSEC_PRESS){
          SWITCH_COUNT[j][i] = CONSEC_PRESS + 1; //Don't increment anymore
          digitalWrite(SWITCH_COL[i], HIGH); //Reset column to HIGH
          SWITCH_PRESS[j][i] = 1; //Set the button trigger flag for the specific button
        }
      }
      else{
        if(SWITCH_COUNT[j][i] != 0){
          SWITCH_COUNT[j][i]--; //Decrement the number of read presses
        }
      }
    }
    digitalWrite(SWITCH_COL[i], HIGH); //Reset the column to high
  }
  PIT_TFLG0 = 0x01; //Clear interrupt flag
}

/*Interrupt Service Routine for PIT0 Timer LED Updates (Every ~1ms)*/
void pit1_isr(void){
  if(ISPLAYING == 0){
    /*Update the LED coloring on the pad*/
    rowLow();
    columnHigh();
    for(int i = 0; i < 4; i++){
      digitalWrite(LED_COL[i], LOW);
      for(int j = 0; j < 4; j++){
        digitalWrite(RED_ROW[j], RED_ON[j][i]);
        digitalWrite(GREEN_ROW[j], GREEN_ON[j][i]);
        digitalWrite(BLUE_ROW[j], BLUE_ON[j][i]);
      }
      delay(1);
      digitalWrite(LED_COL[i], HIGH);
    }
  }
  PIT_TFLG1 = 0x01; //Clear interrupt flag
}
