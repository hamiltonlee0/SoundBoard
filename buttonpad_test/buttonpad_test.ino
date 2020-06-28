/**
 * @file buttonpad_test.ino
 * @author Hamilton Lee
 *
 * This file tests hardware functionality for matrix scan LED button pad.
 * For teensy 3.6 board
 * 
 * Product: https://learn.sparkfun.com/tutorials/button-pad-hookup-guide
 */

/*For Switch Debouncing*/
#define CONSEC_PRESS 3 

/*If LED should be on or off*/
volatile bool isOn = 0;

/*If the press reading is the first press of many consecutive*/
volatile bool firstOn = 1;

/*Number of buttons that are not pressed*/
volatile uint8_t notPressed = 0;

/*Potentiometer PIN*/
const int POT_PIN = 20;

/*LED Button Pressed PIN*/
//GREEN Led through 330 Ohm Reisistor
const int LED_PIN = 33;

/*LED Column Pin Connections*/
/*
      LED_COL_1 = 6;
      LED_COL_2 = 4;
      LED_COL_3 = 2;
      LED_COL_4 = 0;
*/

static uint8_t LED_COL[4] = {6, 4, 2, 0};

/*SWITCH Column Pin Connections*/
/*
      SWITCH_COL_1 = 6;
      SWITCH_COL_2 = 4;
      SWITCH_COL_3 = 2;
      SWITCH_COL_4 = 0;
*/

static uint8_t SWITCH_COL[4] = {7, 5, 3, 1};

/*SWITCH Row Pin Connections*/
/*
      SWITCH_ROW_1 = 6;
      SWITCH_ROW_2 = 4;
      SWITCH_ROW_3 = 2;
      SWITCH_ROW_4 = 0;
*/

static uint8_t SWITCH_ROW[4] = {25, 26, 27, 28}; 

/*Red LED Row Pin Connections*/
/*
      RED_ROW_1 = 17;
      RED_ROW_2 = 16;
      RED_ROW_3 = 15;
      RED_ROW_4 = 14;
*/

static uint8_t RED_ROW[4] = {17, 16, 15, 14};

/*Green LED Row Pin Connections*/
/*
      GREEN_ROW_1 = 8;
      GREEN_ROW_2 = 10;
      GREEN_ROW_3 = 11;
      GREEN_ROW_4 = 12;
*/

static uint8_t GREEN_ROW[4] = {8, 10, 11, 12};

/*Blue LED Row Pin Connections*/
/*
      BLUE_ROW_1 = 29;
      BLUE_ROW_2 = 30;
      BLUE_ROW_3 = 31;
      BLUE_ROW_4 = 32;
*/

static uint8_t BLUE_ROW[4] = {29, 30, 31, 32};

/*Matrix representing number of cycles a switch has been pressed in a row*/
volatile uint16_t SWITCH_PRESS[4][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};

void setup() {
  //Serial Setup
  Serial.begin(9600);
  delay(100);
  Serial.println("Serial Setup Complete");
  
  /*Initial pin setups for button pads*/
  for(int i = 0; i < 4; i++){
    //Set pins as outputs
    pinMode(LED_COL[i], OUTPUT);
    pinMode(RED_ROW[i], OUTPUT);
    //pinMode(GREEN_ROW[i], OUTPUT);
    //pinMode(BLUE_ROW[i], OUTPUT);

    /*Switch Setup*/
    pinMode(SWITCH_COL[i], OUTPUT);
    pinMode(SWITCH_ROW[i], INPUT_PULLUP); //Active Low configuration
    
    /*Default HIGH value for LED Connections*/
    digitalWrite(LED_COL[i], HIGH);
    digitalWrite(RED_ROW[i], HIGH);
    //digitalWrite(GREEN_ROW[i], HIGH);
    //digitalWrite(BLUE_ROW[i], HIGH);

    /*Default HIGH value for Switch Columns*/
    digitalWrite(SWITCH_COL[i], HIGH);
  }
  
  /*Potentiometer is an input*/
  pinMode(POT_PIN, INPUT);

  /*LED_PIN is an output */
  pinMode(LED_PIN, OUTPUT);

//  /********** Test - Turn ALL Red/Green/Blue LEDs **********/
//  for(int i = 0; i < 4; i++){
//    digitalWrite(LED_COL[i], LOW);
//  }

  /*Set up Interrupts for Debouncing timer*/
    /*Looking at the preprocessor macros, pit1 number is
     *larger than all the I2S numbers so no need to change priority*/
  //Note: 60Mhz Counter
  NVIC_ENABLE_IRQ(IRQ_PIT_CH1); //Enable Interrupts for PIT1
  SIM_SCGC6 |= SIM_SCGC6_PIT; //Clock Gating to the PIT module
  
  PIT_MCR &= ~(1 << 1); //Enable Clock
  PIT_LDVAL1 = 0xEA60; //1ms timer
  PIT_TCTRL1 |= PIT_TCTRL_TIE | PIT_TCTRL_TEN; // Enable Interrupts and start timer
}

/*Set each ROW to LOW*/
void rowLow(){
  for(int i = 0; i < 4; i++){
    digitalWrite(RED_ROW[i], LOW);
    //digitalWrite(GREEN_ROW[i], LOW);
    //digitalWrite(BLUE_ROW[i], LOW);
    
  }
}

/*Set each COL to HIGH*/
void columnHigh(){
  for(int i = 0; i < 4; i++){
    digitalWrite(LED_COL[i], HIGH);
  }
}

void loop() {
  /* Reading potentiometer voltage values*/
  Serial.println(analogRead(POT_PIN));

/********** Test - Step through each RED LED **********/ 
  /*Set all rows to LOW*/
  rowLow();
  /*Set all columns to HIGH*/
  columnHigh();
     
  /*Step through each LED*/
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      //All diodes in Reverse Bias as this point
      
      digitalWrite(LED_COL[j], LOW);
      digitalWrite(RED_ROW[i], HIGH);
      //Now target LED is in forward bias
        
      delay(500);

      digitalWrite(LED_COL[j], HIGH);
      digitalWrite(RED_ROW[i], LOW);
      //All diodes in Reverse Bias again
    }
  }
}

/*Interrupt Service Routine for PIT channel 1*/
void pit1_isr(void){
  /*Matrix Scan on buttonpad switches*/
  for(int i = 0; i < 4; i++){ //For each column
    digitalWrite(SWITCH_COL[i], LOW); //Set the column to LOW
    for(int j = 0; j < 4; j++){ //For each row
      if(!digitalRead(SWITCH_ROW[j])){ //If we detect a press
        SWITCH_PRESS[i][j]++;
        /*If we read correct amount of presses*/
        if(SWITCH_PRESS[i][j] > CONSEC_PRESS){
          SWITCH_PRESS[i][j] = CONSEC_PRESS + 1; //Don't increment anymore
          digitalWrite(SWITCH_COL[i], HIGH); //Reset column to HIGH
          if(firstOn){ //If this is the first consecutive detection, toggle LED
            isOn = !isOn;
          }
          firstOn = 0;
          goto matrixScanEnd; //Branch out of loops, we detected a press
        }
      }
      else{
        if(SWITCH_PRESS[i][j] != 0){
          SWITCH_PRESS[i][j]--; //Decrement the number of read presses
        }
        /*Count the number of buttons that are fully not pressed*/
        if(SWITCH_PRESS[i][j] == 0){
          notPressed++;
        }
      }
    }
    digitalWrite(SWITCH_COL[i], HIGH); //Reset the column to high
  }
  matrixScanEnd:
  /*If all buttons are fully not pressed then LED can toggle again*/
  if(notPressed == 16){
    firstOn = 1;
  }
  notPressed = 0; //Reset counter variable
  digitalWrite(LED_PIN, isOn); //Toggle LED if necessary
  PIT_TFLG1 = 0x01; //Clear interrupt flag
}
