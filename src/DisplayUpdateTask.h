#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <U8g2lib.h>


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
    extern U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2;

void displayUpdateTask(void * pvParameters) {
  //set the initiation interval to 100ms
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const String notes[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
  const String octaves[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8"};
  //uint32_t ID=0x123;
  //uint8_t RX_Message[8]={0};

  //const char notes[] = {"C ","C#","D ","D#","E ","F ","F#","G ","G#","A ","A#","B "};
  //const char *notes = 'C\0C#D\0D#E\0F\0F#G\0G#A\0A#B\0';
  //const char *notes[] = {'C','\0','C','#','D','\0','D','#','E','\0', 'F','\0','F','#','G','\0','G','#','A','\0','A','#','B','\0'};
  //const char notes[] = {'C ','C#','D ','D#','E','F ','F#','G ','G#','A ','A#','B '};

  uint8_t RX_MessageLocal[3];
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
      u8g2.setFont(u8g2_font_pxplusibmcgathin_8u); // choose a suitable font ncenB08
      //u8g2.drawStr(2,10,"Helllo World!");  // write something to the internal memory
    
    //uint8_t keys = readCols();
      //u8g2.setCursor(66,30);
      u8g2.setCursor(2,10);
      
      xSemaphoreTake(rxMessageMutex, portMAX_DELAY);
      for(int i = 0; i< 3 ; i++){
        RX_MessageLocal[i] = RX_Message[i];
      }
      xSemaphoreGive(rxMessageMutex);

      u8g2.setCursor(2,10);
    //print sender/receiver status
      //xSemaphoreTake(senderBoolMutex, portMAX_DELAY);
      if(ifSender==1){
        u8g2.print((char) 'S');
      }
      else{
        u8g2.print((char) 'R');
      }
      #ifdef TEST_DISPLAY
        u8g2.print((char) 'S');
      #endif
      //xSemaphoreGive(senderBoolMutex);

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
     u8g2.setCursor(2, 30);
     for (int i = 0; i< 12; i ++) {
        //reading keys
        if(!bitRead(keyPressedNote,i)){
            u8g2.print(notes[i]);
            u8g2.print(knob2Rotation);
     
        }
      }

      if(RX_MessageLocal[0] == 'P'){
        u8g2.print(notes[RX_MessageLocal[2]]);
        u8g2.print(RX_MessageLocal[1]);
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
