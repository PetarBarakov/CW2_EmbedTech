#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>

//Constants
  const uint32_t interval = 100; //Display update interval

//Global Variables
  volatile uint32_t currentStepSize[12];
  volatile uint8_t keyArray[7];

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

  //rotation variable
  volatile int32_t knob3Rotation;
  volatile int32_t knob2Rotation;
  volatile int32_t knob1Rotation;
  volatile int32_t knob0Rotation;


  //sender receiver setting
  volatile bool ifSender;

  //global handle, mutex
  SemaphoreHandle_t keyArrayMutex;
  SemaphoreHandle_t rxMessageMutex;
  SemaphoreHandle_t CAN_TX_Semaphore;
  SemaphoreHandle_t senderBoolMutex;
  SemaphoreHandle_t sampleBufferSemaphore;
  SemaphoreHandle_t stepSizeSemaphore;
  SemaphoreHandle_t waveformSemaphore;

  //global queue handler
  QueueHandle_t msgInQ;
  QueueHandle_t msgOutQ;

  //received message
  uint8_t RX_Message[8]={0};

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

#define SAMPLE_BUFFER_SIZE 256

uint8_t sampleBuffer0[SAMPLE_BUFFER_SIZE];
uint8_t sampleBuffer1[SAMPLE_BUFFER_SIZE];
volatile bool writeBuffer1 = false;

//testing execution time
//#define TEST_SCANKEYS 1
//#define DISABLE_THREADS 1


//class 
class knob {        // The class
  public:              // Access specifier
    int32_t rotationVariable;
    uint8_t currentState;
    uint8_t previousState;
    int32_t rotationDirection;
    void updateRotation() {  // Method/function defined inside the class
      if (bitRead(currentState,0)!=bitRead(previousState,0)){
          if(bitRead(currentState,1)==0b0){
              if(bitRead(currentState,0)==0b1){
                  rotationDirection = 1;
              }
              else if(bitRead(currentState,0)==0b0){
                  rotationDirection = -1;
              }
          }
          else if(bitRead(currentState,1)==0b1){
              if(bitRead(currentState,0)==0b1){
                  rotationDirection = -1;
              }
              else if(bitRead(currentState,0)==0b0){
                  rotationDirection = 1;
              }
          }
              //missed state, assuming same sign as last legal rotation
          rotationVariable = rotationVariable + rotationDirection;
          previousState = currentState;
      }
    }
    void setLimit(int32_t max, int32_t min){
        //limiting to between max and min
        //set to 8 and 0
        if (rotationVariable > max){
            rotationVariable = max;
        }
        if (rotationVariable < min){
            rotationVariable = min;
        }
    }
    int32_t readRotation(){
        return rotationVariable;
    }
};

//Function to set outputs using key matrix

void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

uint8_t readCols(){
    //digitalWrite(RA0_PIN,LOW);
    //digitalWrite(RA1_PIN,LOW);
    //digitalWrite(RA2_PIN,LOW);
    //digitalWrite(REN_PIN,HIGH);
    uint8_t tmp = digitalRead(C3_PIN);
    tmp = (tmp << 1) | digitalRead(C2_PIN);
    tmp = (tmp << 1) | digitalRead(C1_PIN);
    tmp = (tmp << 1) | digitalRead(C0_PIN);
    return tmp;
}

void setRow(uint8_t rowIdx){
    digitalWrite(REN_PIN,LOW);
    digitalWrite(RA0_PIN,bitRead(rowIdx,0));
    digitalWrite(RA1_PIN,bitRead(rowIdx,1));
    digitalWrite(RA2_PIN,bitRead(rowIdx,2));
    digitalWrite(REN_PIN,HIGH);
}

/*
void sampleISR() {
    //update the phase accumulator and set the analogue output voltage at each sample interval
    //sawtooth wave here
    static int32_t phaseAcc = 0;
    phaseAcc += currentStepSize;
    int32_t Vout = phaseAcc >> 24;
    //volume control, log taper
    Vout = Vout >> (8 - knob3Rotation);
    analogWrite(OUTR_PIN, Vout + 128);
}
*/

//double ISR
void doubleISR(){
    static uint32_t readCtr = 0;

    if (readCtr == SAMPLE_BUFFER_SIZE) {
	    readCtr = 0;
	    writeBuffer1 = !writeBuffer1;
	    xSemaphoreGiveFromISR(sampleBufferSemaphore, NULL);
	  }
	
    if (writeBuffer1)
	    analogWrite(OUTR_PIN, sampleBuffer0[readCtr++]);
    else
	    analogWrite(OUTR_PIN, sampleBuffer1[readCtr++]);
}

//ISR for Receiving CAN
void CAN_RX_ISR (void) {
	  uint8_t RX_Message_ISR[8];
	  uint32_t ID = 0x123;
	  CAN_RX(ID, RX_Message_ISR);
	  xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

//create the ISR which will give the semaphore each time a mailbox becomes available
void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}


//double ISR sample thread
void sampleTask(void * pvParameters){

  /*
  double table[1000] = {0};
  for (int i = 1; i <= 1000; i++)
  {
    sine_table[i-1] = std::sin(PI * i/ 1000.0);
  }*/


  //static int32_t phaseAcc[12] = {0};
  uint32_t phaseAcc[12] = {0};
  float phaseAccf[12] = {0};
  int32_t VoutIndividual[12] = {0};
  int32_t Vout = 0;
  int32_t sineValue[12] = {0};
  //volatile int32_t localStepSize[12] = {0};
  //int32_t testStepSizes [] = {262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494};
  //const int32_t testStepSizes [] = {276, 292, 309, 327, 346, 366, 387, 410, 434, 459, 486, 514};
      //sawtooth wave
  /*for(int i = 0; i < 12; i++)
        {
        realStepSize[i]=  testStepSizes[i] << 16 << 1;
        //2.97890873344
        //(1/22000) * 2^16
        Serial.print(realStepSize[i]);
        Serial.print(" ");
        }*/

        //step size is FCW, frequenc control word
  while(1){
	  xSemaphoreTake(sampleBufferSemaphore, portMAX_DELAY);
	  for (uint32_t writeCtr = 0; writeCtr < SAMPLE_BUFFER_SIZE; writeCtr++) {
      xSemaphoreTake(stepSizeSemaphore, portMAX_DELAY);
      //xSemaphoreTake(stepSizeSemaphore, portMAX_DELAY);
      //try computing phase accumulator step size in decode key task
      //convert phase to sine here
      //test = sinf(1);

      //different waveforms
      /*
      if(waveform == 0){
      //sawtooth wave
        //currentStepSize[1] = 262;
        currentStepSize[1]=  currentStepSize[1] << 17;
        //2.97890873344
        //(1/22000) * 2^16
        Serial.print(currentStepSize[1]);
        Serial.print(" ");
      }*/
      /*
      if(waveform == 0){
      //sawtooth wave
        for(int i = 0; i < 12; i++)
        {
        realStepSize[i]=  currentStepSize[i] << 17;
        //2.97890873344
        //(1/22000) * 2^16
        Serial.print(realStepSize[i]);
        Serial.print(" ");
        }
      }*/


      //xSemaphoreGive(stepSizeSemaphore);
      //multiple key
      /*for (int i = 0; i<12; i++){
        phaseAcc[i] += currentStepSize[i];
        VoutIndividual[i] = phaseAcc[i] >> 24;
        //(2^32)*(2^-24)=256
        Vout = Vout + VoutIndividual[i];
        //Serial.print(Vout);
      }
      */
      for (int i = 0; i<12; i++){
        phaseAcc[i] += currentStepSize[i];
        phaseAccf[i] += currentStepSize[i];
        /*
        switch(currentStepSize[i]){
        case 0:
        phaseAccf[i] = 0;
        break;
        default:
        phaseAccf[i] += currentStepSize[i];
        break;
        }*/
        //knob1Rotation for waveform
        switch(knob1Rotation){
          case 0:
          //sawtooth
          if (currentStepSize[i] == 0){
              VoutIndividual[i] = 0;
          }
          else{
              VoutIndividual[i] = (phaseAcc[i] >> 24) - 128;
          }
          //VoutIndividual[i] = (phaseAcc[i] >> 24)-128;
          break;
          case 1:
          //sinewave
          /*
          switch(currentStepSize[i]){
          case 0:
          VoutIndividual[i] = 0;
          break;
          default:
          VoutIndividual[i] = 128*sinf(phaseAccf[i]);
          break;
          }*/
          if (currentStepSize[i] == 0){
              VoutIndividual[i] = 0;
          }
          else{
              VoutIndividual[i] = 127*sinf(phaseAccf[i]);
          }
          //Serial.print(VoutIndividual[i]);
          //Serial.print(" ");
          break;
          case 2:
          //square wave
          //VoutIndividual[i] = 128*sinf(phaseAccf[i]);
          //successfully implemented
          if (currentStepSize[i] == 0){
              VoutIndividual[i] = 0;
          }
          else if (phaseAcc[i] >> 31 > 0){
              VoutIndividual[i] = 127;
          }
          else{
              VoutIndividual[i] = -128;
          }
          break;
          case 3:
          //triangular wave
          if (currentStepSize[i] == 0){
              VoutIndividual[i] = 0;
          }
          if (phaseAcc[i] >> 31 > 0){
              VoutIndividual[i] = min(VoutIndividual[i]+1, (int32_t) 127);
          }
          else{
              VoutIndividual[i] = max(VoutIndividual[i]-1, (int32_t) -128);
          }
          /*
          if (sineValue[i] > 0) {
            VoutIndividual[i] = VoutIndividual[i] += 1;
          }
          else{
            VoutIndividual[i] = VoutIndividual[i] -= 1;
          }*/
          break;
        }
        //(2^32)*(2^-24)=256
        Vout = Vout + VoutIndividual[i];
        //Serial.print(Vout);
      }
      //xSemaphoreGive(stepSizeSemaphore);
      xSemaphoreGive(stepSizeSemaphore);

        //Serial.print(Vout);
        //Serial.print(" ");

      //int32_t Vout = currentStepSize*writeCtr >> 24;

      //Vout = Vout >> (int) log2(round(count / 2) * 2) >> (8 - knob3Rotation);
      

      //clipping algorithm to be implemented

      //somehow full magnitude doesn't work
      Vout = Vout >> (8 - knob3Rotation +1);

      while (Vout > 128){
        Vout = Vout >> 2;
      }
      //volume control, log taper

      //for generating sine wave

      //uint32_t Vout = (int) 128*sin(frequency[]);
      //Calculate one sample
		  if (writeBuffer1)
			  sampleBuffer1[writeCtr] = Vout + 128;
		  else
			  sampleBuffer0[writeCtr] = Vout + 128;
	  }
  }
}


void scanKeysTask(void * pvParameters) {
  //set the initiation interval to 50ms
  //changed to 20 ms
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  //uint8_t previousState = 0b00;
  volatile int32_t rotationDirection;
  volatile int32_t rotationVariable;
  knob knob3;
  knob3.rotationVariable=0;
  knob3.previousState=0b00;
  knob knob2;
  knob2.rotationVariable=0;
  knob2.previousState=0b00;
  knob knob1;
  knob1.rotationVariable=0;
  knob1.previousState=0b00;
  knob knob0;
  knob0.rotationVariable=0;
  knob0.previousState=0b00;
  volatile bool keyState[12] = {0};
  //sender boolean initialized to 0
  bool localIfSender=1;
  //waveform
  int32_t waveform = 0;
  
  //previous array
  volatile uint8_t previousKeyArray[7];

  //array storing an outgoing message
  uint8_t TX_Message[8] = {0};
  /*
  uint32_t step_sizes[12];
  uint32_t current_freq = 440 * pow( pow(2.0, 1.0/12.0), -9); //key C frequency

  for(int i = 0; i < 12; i++)
  {
    step_sizes[i]= pow(2, 32) * current_freq / 22000;
    
    current_freq *= pow(2.0, 1.0/12.0);
    Serial.print(current_freq);
    Serial.print(" ");
    
  }*/

  while (1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    //const int32_t stepSizes [] = {276, 292, 309, 327, 346, 366, 387, 410, 434, 459, 486, 514};
    //frequency
    //const int32_t stepSizes [] = {262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494};
    const uint32_t stepSizes [] = {51076056,54113197,57330935,60740010,64351799,68178356,72232452,76527617,81078186,85899346,91007187,96418756};
    //const float sineStepSizes [] = {1253616,340000017,360215041,381644438,64351799,68178356,72232452,76527617,81078186,85899346,91007187,96418756};
    //sine phase accumulator step size
    //const int32_t stepSizes [] = {262,54113197,57330935,60740010,64351799,68178356,72232452,76527617,81078186,85899346,91007187,96418756};
    /*for(int i = 0; i < 12; i++)
    {
    Serial.print(step_sizes[i]);
    Serial.print(" ");
    }*/
    for (int i = 0; i<= 5; i ++) {
      //key scanning loop
      //row number is the array index
        uint8_t index = i;
        setRow(index);
        delayMicroseconds(3);
        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
        keyArray[i] = readCols();
        xSemaphoreGive(keyArrayMutex);
    }

    bool flag = false;
    int32_t localStepSize[12] = {0};
    for (int i = 0; i<= 5; i ++) {
        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
        //u8g2.print(keyArray[i],HEX); 
        uint8_t keyPressed = keyArray[i];
        uint8_t previousKeyPressed = previousKeyArray[i];

        xSemaphoreGive(keyArrayMutex);
        if (i==0){
              if (!bitRead(keyPressed,0)){
                //C
                localStepSize[0] = stepSizes[0];
                //u8g2.drawStr(2,30,"C");
                if (bitRead(previousKeyPressed,0)){
                  TX_Message[0] = 'P';
                  TX_Message[1] = 4;
                  TX_Message[2] = 0;
                }
                //Str = "C";
              }
              if(!bitRead(keyPressed,1)){
                //C#
                localStepSize[1] = stepSizes[1];
                //Str = "C#";
                //u8g2.drawStr(2,30,"C#");
                if (bitRead(previousKeyPressed,1)){
                  TX_Message[0] = 'P';
                  TX_Message[1] = 4;
                  TX_Message[2] = 1;
                }
              }
              if(!bitRead(keyPressed,2)){
                //D
                localStepSize[2] = stepSizes[2];
                //Str = "D";
                //u8g2.drawStr(2,30,"D");
                if (bitRead(previousKeyPressed,2)){
                  TX_Message[0] = 'P';
                  TX_Message[1] = 4;
                  TX_Message[2] = 2;
                }
              }
              if(!bitRead(keyPressed,3)){
                //D#
                localStepSize[3] = stepSizes[3];
                //Str = "D#";
                //u8g2.drawStr(2,30,"D#");
                if (bitRead(previousKeyPressed,3)){
                  TX_Message[0] = 'P';
                  TX_Message[1] = 4;
                  TX_Message[2] = 3;
                }
              }
              if(keyPressed == 0b1111){
                //Str = "";
                if (!bitRead(previousKeyPressed,0)){
                  TX_Message[0] = 'R';
                  TX_Message[1] = 4;
                  TX_Message[2] = 0;
                }
                if (!bitRead(previousKeyPressed,1)){
                  TX_Message[0] = 'R';
                  TX_Message[1] = 4;
                  TX_Message[2] = 1;
                }
                if (!bitRead(previousKeyPressed,2)){
                  TX_Message[0] = 'R';
                  TX_Message[1] = 4;
                  TX_Message[2] = 2;
                }
                if (!bitRead(previousKeyPressed,3)){
                  TX_Message[0] = 'R';
                  TX_Message[1] = 4;
                  TX_Message[2] = 3;
                }
              }
        }
        else if (i==1){
              if(!bitRead(keyPressed,0)){
                //E
                localStepSize[4] = stepSizes[4];
                //Str = "E";
                //u8g2.drawStr(2,30,"E");
                if (bitRead(previousKeyPressed,0)!= 0){
                  TX_Message[0] = 'P';
                  TX_Message[1] = 4;
                  TX_Message[2] = 4;
                }
              }
              if(!bitRead(keyPressed,1)){
                //F
                localStepSize[5] = stepSizes[5];
                //Str = "F";
                //u8g2.drawStr(2,30,"F");
                if (bitRead(previousKeyPressed,1)!= 0){
                  TX_Message[0] = 'P';
                  TX_Message[1] = 4;
                  TX_Message[2] = 5;
                }
              }
              if(!bitRead(keyPressed,2)){
                //F#
                localStepSize[6] = stepSizes[6];
                //Str = "F#";
                //u8g2.drawStr(2,30,"F#");
                if (bitRead(previousKeyPressed,2)!= 0){
                  TX_Message[0] = 'P';
                  TX_Message[1] = 4;
                  TX_Message[2] = 6;
                }
              }
              if(!bitRead(keyPressed,3)){
                //G
                localStepSize[7] = stepSizes[7];
                //Str = "G";
                //u8g2.drawStr(2,30,"G");
                if (bitRead(previousKeyPressed,3)!= 0){
                  TX_Message[0] = 'P';
                  TX_Message[1] = 4;
                  TX_Message[2] = 7;
                }
              }
              if(keyPressed == 0b1111){
                //Str = "";
                if (bitRead(previousKeyPressed,0)!= 1){
                  TX_Message[0] = 'R';
                  TX_Message[1] = 4;
                  TX_Message[2] = 4;
                }
                if (bitRead(previousKeyPressed,1)!= 1){
                  TX_Message[0] = 'R';
                  TX_Message[1] = 4;
                  TX_Message[2] = 5;
                }
                if (bitRead(previousKeyPressed,2)!= 1){
                  TX_Message[0] = 'R';
                  TX_Message[1] = 4;
                  TX_Message[2] = 6;
                }
                if (bitRead(previousKeyPressed,3)!= 1){
                  TX_Message[0] = 'R';
                  TX_Message[1] = 4;
                  TX_Message[2] = 7;
                }
              }
        }
        else if (i==2 && flag == false){
              if(!bitRead(keyPressed,0)){
                //G#
                localStepSize[8] = stepSizes[8];
                //Str = "G#";
                //u8g2.drawStr(2,30,"G#");
                if (bitRead(previousKeyPressed,0)!= 0){
                  TX_Message[0] = 'P';
                  TX_Message[1] = 4;
                  TX_Message[2] = 8;
                }
              }
              if(!bitRead(keyPressed,1)){
                //A
                localStepSize[9] = stepSizes[9];
                //Str = "A";
                //u8g2.drawStr(2,30,"A");
                if (bitRead(previousKeyPressed,1)!= 0){
                  TX_Message[0] = 'P';
                  TX_Message[1] = 4;
                  TX_Message[2] = 9;
                }
              }
              if(!bitRead(keyPressed,2)){
                //A#
                localStepSize[10] = stepSizes[10];
                //Str = "A#";
                //u8g2.drawStr(2,30,"A#");
                if (bitRead(previousKeyPressed,2)!= 0){
                  TX_Message[0] = 'P';
                  TX_Message[1] = 4;
                  TX_Message[2] = 10;
                }
              }
              if(!bitRead(keyPressed,3)){
                //B
                localStepSize[11] = stepSizes[11];
                //Str = "B";
                //u8g2.drawStr(2,30,"B");
                if (bitRead(previousKeyPressed,3)!= 0){
                  TX_Message[0] = 'P';
                  TX_Message[1] = 4;
                  TX_Message[2] = 11;
                }
              }
              if(keyPressed == 0b1111){
                //Str = "";
                if (bitRead(previousKeyPressed,0)!= 1){
                  TX_Message[0] = 'R';
                  TX_Message[1] = 4;
                  TX_Message[2] = 8;
                }
                if (bitRead(previousKeyPressed,1)!= 1){
                  TX_Message[0] = 'R';
                  TX_Message[1] = 4;
                  TX_Message[2] = 9;
                }
                if (bitRead(previousKeyPressed,2)!= 1){
                  TX_Message[0] = 'R';
                  TX_Message[1] = 4;
                  TX_Message[2] = 10;
                }
                if (bitRead(previousKeyPressed,3)!= 1){
                  TX_Message[0] = 'R';
                  TX_Message[1] = 4;
                  TX_Message[2] = 11;
                }
              }
        }
        else if (i == 3){
          //decoding knob 3
          //xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
          uint8_t currentState3 = (bitRead(keyPressed,1) << 1) | bitRead(keyPressed,0);
          uint8_t currentState2 = (bitRead(keyPressed,3) << 1) | bitRead(keyPressed,2);
          
          knob3.currentState = currentState3;
          knob2.currentState = currentState2;
          knob3.updateRotation();
          knob2.updateRotation();
          knob3.setLimit(8,0);
          knob2.setLimit(8,0);

          //xSemaphoreGive(keyArrayMutex);
          __atomic_store_n(&knob3Rotation, knob3.readRotation(), __ATOMIC_RELAXED);
          __atomic_store_n(&knob2Rotation, knob2.readRotation(), __ATOMIC_RELAXED);
        }
        else if (i == 4){
          uint8_t currentState1 = (bitRead(keyPressed,1) << 1) | bitRead(keyPressed,0);
          uint8_t currentState0 = (bitRead(keyPressed,3) << 1) | bitRead(keyPressed,2);

          knob1.currentState = currentState1;
          knob0.currentState = currentState0;
          knob1.updateRotation();
          knob0.updateRotation();
          //knob1 for waveform
          knob1.setLimit(3,0);
          knob0.setLimit(8,0);

          //xSemaphoreGive(keyArrayMutex);
          __atomic_store_n(&knob1Rotation, knob1.readRotation(), __ATOMIC_RELAXED);
          __atomic_store_n(&knob0Rotation, knob0.readRotation(), __ATOMIC_RELAXED);
        }
        else if (i == 5){
          if (bitRead(keyPressed,0) == 0)
          {
            //press knob 2 to set status to sender
            localIfSender = true;
          }
          else if (bitRead(keyPressed,1) == 0)
          {
            //press knob 3 to set status to receiver
            localIfSender = false;
          }
          __atomic_store_n(&ifSender, localIfSender, __ATOMIC_RELAXED);
        }
    }
    //currentStepSize=localStepSize;
    //adjusting octave in the message transmitted and in the stepsize sent to global step size
    TX_Message[1] = knob2.readRotation();
    if(TX_Message[1]>=4)
    {   
        for (int i = 0; i <12; i++){
          if (localStepSize[i]!=0){
            localStepSize[i] = localStepSize[i] << ((int32_t) TX_Message[1]-4);
          }
        }
    }
    else if(TX_Message[1]<4){
        for (int i = 0; i <12; i++){
          if(localStepSize[i]!=0){
            localStepSize[i] = localStepSize[i] >> (4- (int32_t) TX_Message[1]);
          }
        }
    }
    //__atomic_store_n(&currentStepSize, localStepSize, __ATOMIC_RELAXED);
    //__atomic_store_n(&ifSender, localIfSender, __ATOMIC_RELAXED);

    for(int i = 0; i<7; i++){
        previousKeyArray[i] = keyArray[i];
    }
    //place an item on the queue
    //if is set to a sender
    xSemaphoreTake(senderBoolMutex, portMAX_DELAY);
    if(ifSender){
        xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
        for (int i = 0; i <12; i++){
           localStepSize[i] = 0;
        }
    }
    xSemaphoreGive(senderBoolMutex);


    xSemaphoreTake(stepSizeSemaphore, portMAX_DELAY);
    for (int i = 0; i <12; i++){
        currentStepSize[i] = localStepSize[i];
    }
    xSemaphoreGive(stepSizeSemaphore);
    //__atomic_store_n(&currentStepSize, localStepSize, __ATOMIC_RELAXED);

    //xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
    //CAN_TX(0x123, TX_Message);
  }
}

void displayUpdateTask(void * pvParameters) {
  //set the initiation interval to 100ms
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  //uint32_t ID=0x123;
  //uint8_t RX_Message[8]={0};

  while (1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
      static uint32_t count = 0;

      //while (CAN_CheckRXLevel())
      //ID
	    //CAN_RX(ID, RX_Message);
    //Update display
      u8g2.clearBuffer();         // clear the internal memory
      u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
      u8g2.drawStr(2,10,"Helllo World!");  // write something to the internal memory
    
    //uint8_t keys = readCols();
      u8g2.setCursor(66,30);
      xSemaphoreTake(rxMessageMutex, portMAX_DELAY);
      u8g2.print((char) RX_Message[0]);
      u8g2.print(RX_Message[1]);
      u8g2.print(RX_Message[2]);
      xSemaphoreGive(rxMessageMutex);

    //print sender/receiver status
      xSemaphoreTake(senderBoolMutex, portMAX_DELAY);
      if(ifSender==1){
        u8g2.print((char) 'S');
      }
      else{
        u8g2.print((char) 'R');
      }
      xSemaphoreGive(senderBoolMutex);

      u8g2.setCursor(2,20);

      //for (int i = 0; i<= 2; i ++) {
        //u8g2.print(keyArray[i],HEX); 
      //}

      bool flag = false;
      for (int i = 0; i<= 4; i ++) {
        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
        //u8g2.print(keyArray[i],HEX); 
        if (i==0){
          u8g2.print(knob3Rotation,DEC); 
          u8g2.print(knob2Rotation,DEC);
          u8g2.print(knob1Rotation,DEC);
        }
        //printing key array
        //u8g2.print(keyArray[i],HEX); 
        uint8_t keyPressed = keyArray[i];
        xSemaphoreGive(keyArrayMutex);
        
        if (i==0 && flag==false){
              switch (keyPressed){
                case 0b1110:
                //C
                u8g2.drawStr(2,30,"C");
                flag = true;
                //Str = "C";
                break;
                case 0b1101:
                //C#
                flag = true;
                //Str = "C#";
                u8g2.drawStr(2,30,"C#");
                break;
                case 0b1011:
                //D
                flag = true;
                //Str = "D";
                u8g2.drawStr(2,30,"D");
                break;
                case 0b0111:
                //D#
                flag = true;
                //Str = "D#";
                u8g2.drawStr(2,30,"D#");
                break;
                case 0b1111:
                //Str = "";
                break;
                default:
                break;
              }
        }
        else if (i==1 && flag == false){
              switch (keyPressed){
                case 0b1110:
                //E
                flag = true;
                //Str = "E";
                u8g2.drawStr(2,30,"E");
                break;
                case 0b1101:
                //F
                flag = true;
                //Str = "F";
                u8g2.drawStr(2,30,"F");
                break;
                case 0b1011:
                //F#
                flag = true;
                //Str = "F#";
                u8g2.drawStr(2,30,"F#");
                break;
                case 0b0111:
                //G
                flag = true;
                //Str = "G";
                u8g2.drawStr(2,30,"G");
                break;
                case 0b1111:
                //Str = "";
                break;
                default:
                break;
              }
        }
        else if (i==2 && flag == false){
              switch (keyPressed){
                case 0b1110:
                //G#
                flag = true;
                //Str = "G#";
                u8g2.drawStr(2,30,"G#");
                break;
                case 0b1101:
                //A
                flag = true;
                //Str = "A";
                u8g2.drawStr(2,30,"A");
                break;
                case 0b1011:
                //A#
                flag = true;
                //Str = "A#";
                u8g2.drawStr(2,30,"A#");
                break;
                case 0b0111:
                //B
                flag = true;
                //Str = "B";
                u8g2.drawStr(2,30,"B");
                break;
                case 0b1111:
                //Str = "";
                break;
                default:
                break;
              }
        }
      }
      //u8g2.print(count++);
    //u8g2.print(keys,HEX); 
      u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
      digitalToggle(LED_BUILTIN);
  }
}

void decodeTask(void * pvParameters){
  const int32_t stepSizes [] = {51076056,54113197,57330935,60740010,64351799,68178356,72232452,76527617,81078186,85899346,91007187,96418756};
  uint8_t RX_Message_local[8]={0};
  uint8_t counter = 0;
  while(1){
      xQueueReceive(msgInQ, RX_Message_local, portMAX_DELAY);
      xSemaphoreTake(rxMessageMutex, portMAX_DELAY);
      for(int i = 0; i<8; i++){
        RX_Message[i] = RX_Message_local[i];
      }
      int32_t stepSize;
      if(RX_Message[0]==0x52){
          //R
          stepSize = 0;
          xSemaphoreTake(stepSizeSemaphore, portMAX_DELAY);
          currentStepSize[(int) RX_Message[2]] = stepSize;
          xSemaphoreGive(stepSizeSemaphore);
      }
      else if(RX_Message[0]==0x50){
          //P
          xSemaphoreTake(senderBoolMutex, portMAX_DELAY);
          if(!ifSender){
            stepSize = stepSizes[(int) RX_Message[2]];
          //stepSize = stepSize << ((int32_t) RX_Message[1] -4);
          //xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
          //adjusting octave sent
            if(RX_Message[1]>=4){
              stepSize = stepSize << ((int32_t) RX_Message[1]-4);
            }
            else if(RX_Message[1]<4){
              stepSize = stepSize >> (4-(int32_t) RX_Message[1]);
            }
          //xSemaphoreGive(keyArrayMutex);
          //__atomic_store_n(&currentStepSize, stepSize, __ATOMIC_RELAXED);
            xSemaphoreTake(stepSizeSemaphore, portMAX_DELAY);
              currentStepSize[(int) RX_Message[2]] = stepSize;
            xSemaphoreGive(stepSizeSemaphore);
          }
          xSemaphoreGive(senderBoolMutex);
      }
      xSemaphoreGive(rxMessageMutex);
  }
}

//transmit thread
void CAN_TX_Task (void * pvParameters) {
	uint8_t msgOut[8];
	while (1) {
	xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
	}
}

void setup() {
  // put your setup code here, to run once:

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  //Create Timer
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  #ifndef DISABLE_SAMPLE_ISR
  //sampleTimer->attachInterrupt(sampleISR);
  //double ISR
  sampleTimer->attachInterrupt(doubleISR);
  #endif
  sampleTimer->resume();

  //preposessor directives for measuring execution time
  //#ifndef DISABLE_THREADS
  //initialize and run the thread
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
  scanKeysTask,		/* Function that implements the task */
  "scanKeys",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  //64 byte stack not large enough
  NULL,			/* Parameter passed into the task */
  5,			/* Task priority */
  &scanKeysHandle );  /* Pointer to store the task handle */

  //#ifndef DISABLE_THREADS
    //initialize and run the thread
  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
  displayUpdateTask,		/* Function that implements the task */
  "displayUpdate",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  //64 byte stack not large enough
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &displayUpdateHandle );  /* Pointer to store the task handle */
  

   //initialize and run the thread
  TaskHandle_t decodeHandle = NULL;
  xTaskCreate(
  decodeTask,		/* Function that implements the task */
  "decode",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  //64 byte stack not large enough
  NULL,			/* Parameter passed into the task */
  2,			/* Task priority */
  &decodeHandle );  /* Pointer to store the task handle */

  //#endif
  //initialize and run the thread
  TaskHandle_t transmitHandle = NULL;
  xTaskCreate(
  CAN_TX_Task,		/* Function that implements the task */
  "transmit",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  //64 byte stack not large enough
  NULL,			/* Parameter passed into the task */
  3,			/* Task priority */
  &transmitHandle );  /* Pointer to store the task handle */

  //sample task
  TaskHandle_t sampleHandle = NULL;
  xTaskCreate(
  sampleTask,		/* Function that implements the task */
  "sample",		/* Text name for the task */
  1024,      		/* Stack size in words, not bytes */
  //64 byte stack not large enough
  NULL,			/* Parameter passed into the task */
  4,			/* Task priority */
  &sampleHandle );  /* Pointer to store the task handle */

  //#endif

  //#endif
  
  //assign handle to mutex
  keyArrayMutex = xSemaphoreCreateMutex();
  rxMessageMutex = xSemaphoreCreateMutex();
  senderBoolMutex = xSemaphoreCreateMutex();

  stepSizeSemaphore = xSemaphoreCreateMutex();

  //double buffer
  sampleBufferSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(sampleBufferSemaphore);

  //initialize semaphore for transmit thread
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

  //initialize queue handler
  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(384,8);

  //initialize CAN
  CAN_Init(true);
  setCANFilter(0x123,0x7ff);
  #ifndef DISABLE_CAN_ISR
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  #endif
  CAN_Start();
  
  #ifdef TEST_SCANKEYS
	uint32_t startTime = micros();
  Serial.println(startTime);
	for (int iter = 0; iter < 2; iter++) {
    //32
    scanKeysTask(NULL);
	}
	Serial.println(micros()-startTime);
	while(1);
  #endif

  //start the RTOS scheduler
  vTaskStartScheduler();


}

void loop() {
  // put your main code here, to run repeatedly:

}