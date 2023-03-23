#include <Arduino.h>
#include <STM32FreeRTOS.h>


#include <KnobControl.h>
#include <AutoDetection.h>


//Constants
extern const uint32_t interval; //Display update interval

//Global Variables
extern volatile uint32_t currentStepSize[12];
extern volatile uint8_t keyArray[7];

//Timing
extern volatile bool loopCondition;

//Pin definitions
//Row select and enable
extern const int RA0_PIN;
extern const int RA1_PIN;
extern const int RA2_PIN;
extern const int REN_PIN;

//Matrix input and output
extern const int C0_PIN;
extern const int C1_PIN;
extern const int C2_PIN;
extern const int C3_PIN;
extern const int OUT_PIN;

//Audio analogue out
extern const int OUTL_PIN;
extern const int OUTR_PIN;

//Joystick analogue in
extern const int JOYY_PIN;
extern const int JOYX_PIN;

//Output multiplexer bits
extern const int DEN_BIT ;
extern const int DRST_BIT;
extern const int HKOW_BIT;
extern const int HKOE_BIT;

//rotation variable
extern volatile int32_t knob3Rotation;
extern volatile int32_t knob2Rotation;
extern volatile int32_t knob1Rotation;
extern volatile int32_t knob0Rotation;


//sender receiver setting
extern volatile bool ifSender;

//global handle, mutex
extern SemaphoreHandle_t keyArrayMutex;
extern SemaphoreHandle_t rxMessageMutex;
extern SemaphoreHandle_t CAN_TX_Semaphore;

//SemaphoreHandle_t senderBoolMutex;
extern SemaphoreHandle_t sampleBufferSemaphore;
extern SemaphoreHandle_t stepSizeSemaphore;
//SemaphoreHandle_t waveformSemaphore;

//global queue handler
extern QueueHandle_t msgInQ;
extern QueueHandle_t msgOutQ;

//received message
extern uint8_t RX_Message[8];



extern volatile uint8_t possition;
volatile uint8_t octave;


uint8_t readCols(){
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
  #ifdef TEST_SCANKEYS
  loopCondition = true;
  while(loopCondition){
  #endif

  #ifndef TEST_SCANKEYS
  while (1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  #endif

    uint32_t local_key_array[7];

    bool OutBits[7];
    GenerateHandshake(OutBits);
        

    for (int i = 0; i<= 6; i ++) {
      //key scanning loop
      //row number is the array index
        uint8_t index = i;
        setRow(index);
        digitalWrite(OUT_PIN, OutBits[i]);
        delayMicroseconds(3);
        local_key_array[i] = readCols();
    }
    
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);

    for(int i = 0; i < 7; i++)
    {
      keyArray[i] = local_key_array[i];
    }

    xSemaphoreGive(keyArrayMutex);

    //decode east and west signals
    bool west_detect = ((local_key_array[5] & 0b1000) >> 3);
    bool east_detect = ((local_key_array[6] & 0b1000) >> 3);

    //read the octave
    uint32_t octave_reading = knob2.readRotation();

    


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

    for(int i = 0; i<7; i++){
        previousKeyArray[i] = keyArray[i];
    }

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


    xSemaphoreTake(stepSizeSemaphore, portMAX_DELAY);
    for (int i = 0; i <12; i++){
        currentStepSize[i] = localStepSize[i];
    }
    xSemaphoreGive(stepSizeSemaphore);


    #ifdef TEST_SCANKEYS
    loopCondition = false;
    #endif

    
  }
}