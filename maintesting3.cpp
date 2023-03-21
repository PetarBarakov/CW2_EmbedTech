#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>

//Constants
  const uint32_t interval = 100; //Display update interval

//Global Variables
  volatile uint32_t currentStepSize[12] = {0};
  volatile uint8_t keyArray[7] = {0};

//Timing
  volatile bool loopCondition;

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
  volatile bool ifSender = 0;

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

#define SAMPLE_BUFFER_SIZE 110
//256

uint8_t sampleBuffer0[SAMPLE_BUFFER_SIZE];
uint8_t sampleBuffer1[SAMPLE_BUFFER_SIZE];
volatile bool writeBuffer1 = false;

//testing execution time

//#define TEST_SCANKEYS 1
//#define TEST_DISPLAY  1
//#define TEST_DECODE  1
//#define TEST_TX  1
#define TEST_SAMPLE  1
//#define DISABLE_SAMPLE_ISR 1
#define DISABLE_THREADS 1
#define DISABLE_CAN_ISR 1



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
    //uint32_t ID = 0x120;
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
  //uint32_t phaseAccf[12] = {0};
  int32_t VoutIndividual[12] = {0};
  int32_t Vout = 0;
  //int32_t sineValue[12] = {0};
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

  #ifdef TEST_SAMPLE 
  loopCondition = true;
  while(loopCondition){
  #endif 
  #ifndef TEST_SAMPLE 
  while(1){
  xSemaphoreTake(sampleBufferSemaphore, portMAX_DELAY);
  #endif 
	  //xSemaphoreTake(sampleBufferSemaphore, portMAX_DELAY);
	  for (uint32_t writeCtr = 0; writeCtr < SAMPLE_BUFFER_SIZE; writeCtr++) {
      #ifndef TEST_SAMPLE 
      xSemaphoreTake(stepSizeSemaphore, portMAX_DELAY);
      #endif 
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
        //phaseAccf[i] += currentStepSize[i];
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
              VoutIndividual[i] = 127*sinf(phaseAcc[i]);
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
              // VoutIndividual[i] = min(VoutIndividual[i]+1, (int32_t) 127);
              // 3.18 better sound
              VoutIndividual[i] = VoutIndividual[i]+1;
              
          }
          else{
              // VoutIndividual[i] = max(VoutIndividual[i]-1, (int32_t) -128);
              // 3.18 better sound
              VoutIndividual[i] = VoutIndividual[i]-1;
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
      #ifndef TEST_SAMPLE 
      xSemaphoreGive(stepSizeSemaphore);
      #endif

        //Serial.print(Vout);
        //Serial.print(" ");

      //int32_t Vout = currentStepSize*writeCtr >> 24;

      //Vout = Vout >> (int) log2(round(count / 2) * 2) >> (8 - knob3Rotation);
      

      //clipping algorithm to be implemented

      //somehow full magnitude doesn't work
      Vout = Vout >> (8 - knob3Rotation +1);
      //clip
      if (Vout > 127){
        Vout = 127;
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

    //Serial.println(micros());
    #ifdef TEST_SAMPLE 
    loopCondition = true;
    #endif 
    //Serial.printf("\n\r[2] Min available stack size %d * %d bytes\n\r", uxTaskGetStackHighWaterMark(NULL), sizeof(portBASE_TYPE));
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
  bool localIfSender=0;
  #ifdef TEST_SCANKEYS
  localIfSender=1;
  #endif
  //waveform
  int32_t waveform = 0;
  
  //previous array
  volatile uint8_t previousKeyArray[7];

  //array storing an outgoing message
  uint8_t TX_Message[12][8] = {0};

  const uint32_t stepSizes [] = {51076056,54113197,57330935,60740010,64351799,68178356,72232452,76527617,81078186,85899346,91007187,96418756};
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
  //UBaseType_t uxHighWaterMark;
  //uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  // switches to make contents of the while loop run only once
  #ifdef TEST_SCANKEYS
  loopCondition = true;
  while(loopCondition){
  #endif

  #ifndef TEST_SCANKEYS
  while (1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  #endif
    //const int32_t stepSizes [] = {276, 292, 309, 327, 346, 366, 387, 410, 434, 459, 486, 514};
    //frequency
    //const int32_t stepSizes [] = {262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494};
    //const uint32_t stepSizes [] = {51076056,54113197,57330935,60740010,64351799,68178356,72232452,76527617,81078186,85899346,91007187,96418756};
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

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    uint64_t keyPressedNote = (keyArray[2] << 8) | (keyArray[1] << 4) | keyArray[0];
    xSemaphoreGive(keyArrayMutex);

    uint64_t previousKeyPressedNote = (previousKeyArray[2] << 8) | (previousKeyArray[1] << 4) | previousKeyArray[0];

    for (uint8_t i = 0; i< 12; i ++) {
    //reading keys
        if(!bitRead(keyPressedNote,i)){
            localStepSize[i] = stepSizes[i];
            if (bitRead(previousKeyPressedNote, i)){
            TX_Message[i][0] = 'P';
            //TX_Message[1] = 4;
            TX_Message[i][2] = i;
            }
        }
        else{
            if (!bitRead(previousKeyPressedNote, i)){
            TX_Message[i][0] = 'R';
            //TX_Message[1] = 4;
            TX_Message[i][2] = i;
            }
            else{
            TX_Message[i][0] = 0;
            //TX_Message[1] = 4;
            TX_Message[i][2] = 0;
            }
        }
    }

    //for (int i = 3; i<= 5; i ++) {
    //reading knobs and other inputs
    //remove the loop
    //concatenate keyArray bits
        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
        //u8g2.print(keyArray[i],HEX); 
        uint64_t keyPressed = (keyArray[6] << 12) | (keyArray[5] << 8) | (keyArray[4] << 4) | keyArray[3];
        //uint8_t previousKeyPressed = previousKeyArray[i];

        xSemaphoreGive(keyArrayMutex);

        //if (i == 3){
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
        //}
        //else if (i == 4){
          uint8_t currentState1 = (bitRead(keyPressed,5) << 1) | bitRead(keyPressed,4);
          uint8_t currentState0 = (bitRead(keyPressed,7) << 1) | bitRead(keyPressed,6);

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
        //}
        //else if (i == 5){
          if (bitRead(keyPressed,8) == 0)
          {
            //press knob 2 to set status to sender
            localIfSender = true;
          }
          else if (bitRead(keyPressed,9) == 0)
          {
            //press knob 3 to set status to receiver
            localIfSender = false;
          }
          __atomic_store_n(&ifSender, localIfSender, __ATOMIC_RELAXED);
        //}
    //}
    //currentStepSize=localStepSize;
    //adjusting octave in the message transmitted and in the stepsize sent to global step size
    for(int i = 0; i<12; i++){
        TX_Message[i][1] = knob2.readRotation();
    }
    
    /*
    for (int i = 0; i < 12; i++){
        if(localStepSize[i] != 0){
          if (TX_Message[1] >= 4){
            localStepSize[i] = localStepSize[i] << ((int32_t) TX_Message[1]-4);
          }
          else if(TX_Message[1] < 4){
            localStepSize[i] = localStepSize[i] >> (4- (int32_t) TX_Message[1]);
          }
        }
    }*/
    //another implementation
    
    if(TX_Message[1][1]>=4)
    {   
        for (int i = 0; i <12; i++){
          //if (localStepSize[i]!=0){
            localStepSize[i] = localStepSize[i] << ((int32_t) TX_Message[i][1]-4);
          //}
        }
    }
    else if(TX_Message[1][1]<4){
        for (int i = 0; i <12; i++){
          //if(localStepSize[i]!=0){
            localStepSize[i] = localStepSize[i] >> (4- (int32_t) TX_Message[i][1]);
          //}
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
        uint8_t local_TX_Message[8] = {0};
        for(int i = 0; i<12; i++){
          for(int j = 0; j<8; j++){
              local_TX_Message[j] = TX_Message[i][j];
          }
          #ifndef TEST_SCANKEYS
          if(local_TX_Message[0]=='R' || local_TX_Message[0]=='P')
          xQueueSend( msgOutQ, local_TX_Message, portMAX_DELAY);
          #endif
          
        }
        
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


    #ifdef TEST_SCANKEYS
    loopCondition = false;
    #endif

    //stack size
    //Serial.printf("\n\r[2] Start Min available stack size %d * %d bytes\n\r", uxHighWaterMark, sizeof(portBASE_TYPE));
    //Serial.printf("\n\r[2] Min available stack size %d * %d bytes\n\r", uxTaskGetStackHighWaterMark(NULL), sizeof(portBASE_TYPE));

    
  }
}

void displayUpdateTask(void * pvParameters) {
  //set the initiation interval to 100ms
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  //uint32_t ID=0x123;
  //uint8_t RX_Message[8]={0};

  //const char notes[] = {"C ","C#","D ","D#","E ","F ","F#","G ","G#","A ","A#","B "};
  //const char *notes = 'C\0C#D\0D#E\0F\0F#G\0G#A\0A#B\0';
  //const char *notes[] = {'C','\0','C','#','D','\0','D','#','E','\0', 'F','\0','F','#','G','\0','G','#','A','\0','A','#','B','\0'};
  //const char notes[] = {'C ','C#','D ','D#','E','F ','F#','G ','G#','A ','A#','B '};
  #ifdef TEST_DISPLAY
  loopCondition = true;
  while(loopCondition){
  #endif

  #ifndef TEST_DISPLAY
  while (1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  #endif
      static uint32_t count = 0;

      //while (CAN_CheckRXLevel())
      //ID
	    //CAN_RX(ID, RX_Message);
    //Update display
      u8g2.clearBuffer();         // clear the internal memory
      u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
      //u8g2.drawStr(2,10,"Helllo World!");  // write something to the internal memory
    
    //uint8_t keys = readCols();
      //u8g2.setCursor(66,30);
      u8g2.setCursor(2,10);
      
      xSemaphoreTake(rxMessageMutex, portMAX_DELAY);
      u8g2.print((char) RX_Message[0]);
      u8g2.print(RX_Message[1]);
      u8g2.print(RX_Message[2]);
      xSemaphoreGive(rxMessageMutex);
      

      u8g2.setCursor(32,10);
    //print sender/receiver status
      xSemaphoreTake(senderBoolMutex, portMAX_DELAY);
      if(ifSender==1){
        u8g2.print((char) 'S');
      }
      else{
        u8g2.print((char) 'R');
      }
      #ifdef TEST_DISPLAY
        u8g2.print((char) 'S');
      #endif
      xSemaphoreGive(senderBoolMutex);

      u8g2.setCursor(2,20);

      //for (int i = 0; i<= 2; i ++) {
        //u8g2.print(keyArray[i],HEX); 
      //}

      u8g2.print(knob3Rotation,DEC); 
      u8g2.print(knob2Rotation,DEC);
      u8g2.print(knob1Rotation,DEC);

      //const char *note;
      //note = notes;

      xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      uint64_t keyPressedNote = (keyArray[2] << 8) | (keyArray[1] << 4) | keyArray[0];
      #ifdef TEST_DISPLAY
      keyPressedNote = 000000000000;
      #endif
      xSemaphoreGive(keyArrayMutex);
/*
      for (uint8_t i = 0; i< 12; i ++) {
        //reading keys
        if(!bitRead(keyPressedNote,i)){
            u8g2.drawStr(2,30,notes[(2*i)]);
            u8g2.drawStr(4,30,notes[(2*i)+1]);
        }
      }
      */
     for (int i = 0; i< 12; i ++) {
        //reading keys
        if(!bitRead(keyPressedNote,i)){
            switch(i){
              //8 space width for every letter
              //14 for sharp note
              case 0:
              u8g2.drawStr(2,30,"C");
              break;
              case 1:
              u8g2.drawStr(10,30,"C#");
              break;
              case 2:
              u8g2.drawStr(24,30,"D");
              break;
              case 3:
              u8g2.drawStr(32,30,"D#");
              break;
              case 4:
              u8g2.drawStr(46,30,"E");
              break;
              case 5:
              u8g2.drawStr(54,30,"F");
              break;
              case 6:
              u8g2.drawStr(62,30,"F#");
              break;
              case 7:
              u8g2.drawStr(76,30,"G");
              break;
              case 8:
              u8g2.drawStr(84,30,"G#");
              break;
              case 9:
              u8g2.drawStr(98,30,"A");
              break;
              case 10:
              u8g2.drawStr(106,30,"A#");
              break;
              case 11:
              u8g2.drawStr(120,30,"B");
              break;
            }
        }
      }

      u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
      digitalToggle(LED_BUILTIN);

      //Serial.printf("\n\r[2] Min available stack size %d * %d bytes\n\r", uxTaskGetStackHighWaterMark(NULL), sizeof(portBASE_TYPE));

      #ifdef TEST_DISPLAY
      loopCondition = false;
      #endif
  }
  
}

void decodeTask(void * pvParameters){
  const int32_t stepSizes [] = {51076056,54113197,57330935,60740010,64351799,68178356,72232452,76527617,81078186,85899346,91007187,96418756};
  uint8_t RX_Message_local[8]={0};
  uint8_t counter = 0;
  //bool localIfSender = true;
  #ifdef TEST_DECODE
  //uint8_t RX_Message_test[8]={0};
  //uint32_t ID = 0x123;
  //CAN_TX(ID, TX_Message);
  //CAN_RX(ID, RX_Message_local);
	//xQueueSendFromISR(msgInQ, RX_Message_test, NULL);
  loopCondition = true;
  while(loopCondition){
  uint32_t ID = 0x123;
  //CAN_TX(ID, TX_Message);
  CAN_RX(ID, RX_Message_local);
  #endif

  #ifndef TEST_DECODE
  while(1){
      xQueueReceive(msgInQ, RX_Message_local, portMAX_DELAY);
  #endif
    //if(!ifSender){
      //if(!ifSender){
      //xQueueReceive(msgInQ, RX_Message_local, portMAX_DELAY);
      //}

      /*#ifdef TEST_DECODE
        //uint8_t RX_Message_test[8]={0};
      uint32_t ID = 0x123;
      CAN_RX(ID, RX_Message_local);
      #endif*/
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
      //Serial.printf("\n\r[2] Min available stack size %d * %d bytes\n\r", uxTaskGetStackHighWaterMark(NULL), sizeof(portBASE_TYPE));
    //}

      #ifdef TEST_DECODE
      loopCondition = false;
      #endif

      //Serial.println(micros());
  }
}

//transmit thread
void CAN_TX_Task (void * pvParameters) {
	//uint8_t msgOut[8];
  //bool localIfSender = false;

  #ifdef TEST_TX
  //uint8_t RX_Message_test[8]={0};
  //uint32_t ID = 0x123;
  //CAN_TX(ID, TX_Message);
  //CAN_RX(ID, RX_Message_local);
	//xQueueSendFromISR(msgInQ, RX_Message_test, NULL);
  loopCondition = true;
  uint8_t msgOut[8] = {0};
  while(loopCondition){
  //uint32_t ID = 0x123;
  //CAN_TX(ID, TX_Message);
  //CAN_RX(ID, RX_Message_local);
  #endif

  #ifndef TEST_TX
  uint8_t msgOut[8];
	while (1) {
  xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
  xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
  #endif
    //xSemaphoreTake(senderBoolMutex, portMAX_DELAY);
    //localIfSender = ifSender;
    //xSemaphoreGive(senderBoolMutex);
    //if(ifSender){
	  //xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
    //xSemaphoreTake(senderBoolMutex, portMAX_DELAY);
    //localIfSender = ifSender;
    //xSemaphoreGive(senderBoolMutex);
		//xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
    //if(localIfSender){
		CAN_TX(0x123, msgOut);

    #ifdef TEST_TX
    loopCondition = false;
    #endif

    //}
    //}
    //xSemaphoreGive(senderBoolMutex);
    //Serial.printf("\n\r[2] Min available stack size %d * %d bytes\n\r", uxTaskGetStackHighWaterMark(NULL), sizeof(portBASE_TYPE));
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
  
  //initialize and run the thread
  #ifndef DISABLE_THREADS
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
  scanKeysTask,		/* Function that implements the task */
  "scanKeys",		/* Text name for the task */
  340,      		/* Stack size in words, not bytes */
  //64 byte stack not large enough
  NULL,			/* Parameter passed into the task */
  4,			/* Task priority */
  &scanKeysHandle );  /* Pointer to store the task handle */
  
    //initialize and run the thread
  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
  displayUpdateTask,		/* Function that implements the task */
  "displayUpdate",		/* Text name for the task */
  309,      		/* Stack size in words, not bytes */
  //64 byte stack not large enough
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &displayUpdateHandle );  /* Pointer to store the task handle */
  

   //initialize and run the thread
  TaskHandle_t decodeHandle = NULL;
  xTaskCreate(
  decodeTask,		/* Function that implements the task */
  "decode",		/* Text name for the task */
  283,      		/* Stack size in words, not bytes */
  //64 byte stack not large enough
  NULL,			/* Parameter passed into the task */
  3,			/* Task priority */
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
  2,			/* Task priority */
  &transmitHandle );  /* Pointer to store the task handle */
  #endif

  
  //sample task
  TaskHandle_t sampleHandle = NULL;
  xTaskCreate(
  sampleTask,		/* Function that implements the task */
  "sample",		/* Text name for the task */
  321,      		/* Stack size in words, not bytes */
  //64 byte stack not large enough
  NULL,			/* Parameter passed into the task */
  5,			/* Task priority */
  &sampleHandle );  /* Pointer to store the task handle */
  #ifndef DISABLE_THREADS

  #endif

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
  //36
  //384
  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(36,8);
  #ifdef TEST_SCANKEYS
  msgOutQ = xQueueCreate(384,8);
  #endif
  #ifdef TEST_TX
  msgOutQ = xQueueCreate(384,8);
  #endif
  //#ifdef TEST_DECODE
  //msgInQ = xQueueCreate(36,8);
  //#endif

  //initialize CAN
  #ifdef TEST_SCANKEYS
  CAN_Init(true);
  #endif
  #ifndef TEST_SCANKEYS
  //CAN_Init(false);
  #endif

  CAN_Init(true);

  //CAN_Init(true);
  setCANFilter(0x123,0x7ff);
  #ifndef DISABLE_CAN_ISR
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  #endif
  CAN_Start();
  
  #ifdef TEST_SCANKEYS
	uint32_t startTime = micros();
  Serial.println(startTime);
	for (int iter = 0; iter < 32; iter++) {
    //32
    scanKeysTask(NULL);
	}
	Serial.println(micros()-startTime);
	while(1);
  #endif

  #ifdef TEST_DISPLAY
	uint32_t startTime = micros();
  //Serial.println(startTime);
	for (int iter = 0; iter < 32; iter++) {
    //32
    displayUpdateTask(NULL);
	}
	Serial.println(micros()-startTime);
	while(1);
  #endif

  #ifdef TEST_DECODE
	uint32_t startTime = micros();
  //Serial.println(startTime);
	for (int iter = 0; iter < 32; iter++) {
    //32
    decodeTask(NULL);
	}
	Serial.println(micros()-startTime);
	while(1);
  #endif

  #ifdef TEST_TX
	uint32_t startTime = micros();
  //Serial.println(startTime);
	for (int iter = 0; iter < 32; iter++) {
    //32
    CAN_TX_Task(NULL);
	}
	Serial.println(micros()-startTime);
	while(1);
  #endif

  #ifdef TEST_SAMPLE
	uint32_t startTime = micros();
  Serial.println(startTime);
	for (int iter = 0; iter < 32; iter++) {
    //32
    sampleTask(NULL);
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