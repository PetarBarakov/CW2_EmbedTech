#include <Arduino.h>
#include <STM32FreeRTOS.h>


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

#define SAMPLE_BUFFER_SIZE 110
  //256

  extern uint8_t sampleBuffer0[SAMPLE_BUFFER_SIZE];
  extern uint8_t sampleBuffer1[SAMPLE_BUFFER_SIZE];
  extern volatile bool writeBuffer1;

  //Display driver object
    // extern U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2;


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
  //uint8_t RX_Message_ISR[8] = {'P', 5, 3};
  //RX_Message_ISR[8];
  //uint32_t ID = 0x123;
  
  while(loopCondition){
  //uint32_t startTime = micros();
  //uint32_t ID = 0x123;
  //CAN_TX(ID, TX_Message);
  //CAN_RX(ID, RX_Message_local);
    //uint32_t ID = 0x120;
	//xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
	//xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
	//xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
  xQueueReceive(msgInQ, RX_Message_local, portMAX_DELAY);
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
      xSemaphoreGive(rxMessageMutex);
      int32_t stepSize;
      if(RX_Message_local[0]==0x52){
          //R
          stepSize = 0;
          xSemaphoreTake(stepSizeSemaphore, portMAX_DELAY);
          currentStepSize[(int) RX_Message_local[2]] = stepSize;
          xSemaphoreGive(stepSizeSemaphore);
      }
      else if(RX_Message_local[0]==0x50){
          //P
          //xSemaphoreTake(senderBoolMutex, portMAX_DELAY);
          if(!ifSender){
            stepSize = stepSizes[(int) RX_Message_local[2]];
          //stepSize = stepSize << ((int32_t) RX_Message[1] -4);
          //xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
          //adjusting octave sent
            if(RX_Message_local[1]>=4){
              stepSize = stepSize << ((int32_t) RX_Message_local[1]-4);
            }
            else if(RX_Message_local[1]<4){
              stepSize = stepSize >> (4-(int32_t) RX_Message_local[1]);
            }
          //xSemaphoreGive(keyArrayMutex);
          //__atomic_store_n(&currentStepSize, stepSize, __ATOMIC_RELAXED);
            xSemaphoreTake(stepSizeSemaphore, portMAX_DELAY);
              currentStepSize[(int) RX_Message_local[2]] = stepSize;
            xSemaphoreGive(stepSizeSemaphore);
          }
          //xSemaphoreGive(senderBoolMutex);
      }
      //Serial.printf("\n\r[2] Min available stack size %d * %d bytes\n\r", uxTaskGetStackHighWaterMark(NULL), sizeof(portBASE_TYPE));
    //}

      #ifdef TEST_DECODE
      loopCondition = false;
      //Serial.println(micros()-startTime);
      #endif

      //Serial.println(micros());
  }
}