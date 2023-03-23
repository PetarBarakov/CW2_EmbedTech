#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>

// #include <AutoDetection.h>
// #include <KnobControl.h>
#include <ScanKeysTask.h>
#include <SampleTask.h>
#include <DisplayUpdateTask.h>
#include <DecodeTask.h>



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
  volatile int32_t knob1Rotation = 2;
  volatile int32_t knob0Rotation;


  //sender receiver setting
  volatile bool ifSender = 0;

  //global handle, mutex
  SemaphoreHandle_t keyArrayMutex;
  SemaphoreHandle_t rxMessageMutex;
  SemaphoreHandle_t CAN_TX_Semaphore;
  //SemaphoreHandle_t senderBoolMutex;
  SemaphoreHandle_t sampleBufferSemaphore;
  SemaphoreHandle_t stepSizeSemaphore;
  //SemaphoreHandle_t waveformSemaphore;

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
//#define TEST_SAMPLE  1
//#define TEST_DOUBLE_ISR 1
//#define TEST_CAN_RX_ISR 1
//#define TEST_CAN_TX_ISR 1
//#define DISABLE_SAMPLE_ISR 1
//#define DISABLE_THREADS 1
//#define DISABLE_CAN_ISR 1


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
   /*#ifdef TEST_CAN_RX_ISR
	  uint32_t startTime = micros();
    #endif*/
	  uint8_t RX_Message_ISR[8];
	  uint32_t ID = 0x123;
    //uint32_t ID = 0x120;
	  CAN_RX(ID, RX_Message_ISR);
	  xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
/*
    #ifdef TEST_CAN_RX_ISR
	  Serial.println(micros()-startTime);
    #endif*/
}

//create the ISR which will give the semaphore each time a mailbox becomes available
void CAN_TX_ISR (void) {
	  xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}


//double ISR sample thread

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
  uint8_t msgOut[8];
  
  while(loopCondition){
  xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
  xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
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
    xSemaphoreGive(CAN_TX_Semaphore);
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
  sampleTimer->attachInterrupt(doubleISR);
  #ifndef DISABLE_SAMPLE_ISR
  //sampleTimer->attachInterrupt(sampleISR);
  //double ISR
  sampleTimer->attachInterrupt(doubleISR);
  #endif
  #ifdef DISABLE_SAMPLE_ISR
  //sampleTimer->attachInterrupt(sampleISR);
  //double ISR
  //sampleTimer->attachInterrupt(CAN_RX_ISR);
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
  362,      		/* Stack size in words, not bytes */
  //64 byte stack not large enough
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &displayUpdateHandle );  /* Pointer to store the task handle */
  
  
   //initialize and run the thread
  TaskHandle_t decodeHandle = NULL;
  xTaskCreate(
  decodeTask,		/* Function that implements the task */
  "decode",		/* Text name for the task */
  1024,      		/* Stack size in words, not bytes */
  //64 byte stack not large enough
  NULL,			/* Parameter passed into the task */
  3,			/* Task priority */
  &decodeHandle );  /* Pointer to store the task handle */
  
  #endif

  
  
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

  #ifndef DISABLE_THREADS

  
  
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
  

  #endif

  //#endif
  
  //assign handle to mutex
  keyArrayMutex = xSemaphoreCreateMutex();
  rxMessageMutex = xSemaphoreCreateMutex();
  //senderBoolMutex = xSemaphoreCreateMutex();

  stepSizeSemaphore = xSemaphoreCreateMutex();

  //double buffer
  sampleBufferSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(sampleBufferSemaphore);

  //initialize semaphore for transmit thread
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

  //initialize queue handler
  //36
  //384
  //msgInQ = xQueueCreate(36,8);
  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(36,8);
  #ifdef TEST_SCANKEYS
  msgOutQ = xQueueCreate(384,8);
  #endif
  #ifdef TEST_TX
  //msgOutQ = xQueueCreate(384,8);
  #endif
  #ifdef TEST_CAN_RX_ISR
  msgInQ = xQueueCreate(384,8);
  #endif
  //#ifdef TEST_DECODE
  //msgInQ = xQueueCreate(36,8);
  //#endif

  //initialize CAN
  #ifdef TEST_SCANKEYS
  //CAN_Init(true);
  #endif
  #ifndef TEST_SCANKEYS
  //CAN_Init(false);
  #endif

  //CAN_Init(false);

  //CAN_Init(true);
  //setCANFilter(0x123,0x7ff);
  #ifndef DISABLE_CAN_ISR
  CAN_Init(false);
  setCANFilter(0x123,0x7ff);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_Start();
  #endif


  #ifdef TEST_TX
  CAN_Init(true);
  setCANFilter(0x123,0x7ff);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_Start();
  #endif
  //CAN_Start();
  
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
  //uint8_t RX_Message_Test[8] = {'P', 5, 3,0,0,0,0,0};
  uint8_t RX_Message_Test[8] = {'P', 3, 5,0,0,0,0,0};
  uint8_t RX_Message_Test1[8] = {'R', 6, 7,0,0,0,0,0};
  uint8_t RX_Message_Test2[8] = {'P', 2, 11,0,0,0,0,0};
    //RX_Message_ISR[8];
  //uint32_t ID = 0x123;
  //for (int iter = 0; iter < 32; iter++) {
  xQueueSend(msgInQ, RX_Message_Test,portTICK_PERIOD_MS);
  xQueueSend(msgInQ, RX_Message_Test1,portTICK_PERIOD_MS);
  xQueueSend(msgInQ, RX_Message_Test2,portTICK_PERIOD_MS);
  //}
	//for (int iter = 0; iter < 1; iter++) {
    //32
  uint32_t startTime = micros();
  for (int iter = 0; iter < 3; iter++) {
    //32
    decodeTask(NULL);
    
	}
	Serial.println(micros()-startTime);
	while(1);
  #endif

  #ifdef TEST_TX
  uint8_t TX_Message_Test[8] = {'P', 3, 5,0,0,0,0,0};
  uint8_t TX_Message_Test1[8] = {'R', 6, 7,0,0,0,0,0};
  uint8_t TX_Message_Test2[8] = {'P', 2, 11,0,0,0,0,0};

  xQueueSend(msgOutQ, TX_Message_Test,portTICK_PERIOD_MS);
  xQueueSend(msgOutQ, TX_Message_Test1,portTICK_PERIOD_MS);
  xQueueSend(msgOutQ, TX_Message_Test2,portTICK_PERIOD_MS);

	uint32_t startTime = micros();
  Serial.println(startTime);
	for (int iter = 0; iter < 3; iter++) {
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
  
  #ifdef TEST_CAN_RX_ISR
	uint32_t startTime = micros();
  //Serial.println(startTime);
	for (int iter = 0; iter < 32; iter++) {
    //32
    CAN_RX_ISR();
	}
    //CAN_RX_ISR();
    /*Serial.println("World");
  	uint8_t RX_Message_ISR[8] = {0};
	  uint32_t ID = 0x123;
    //uint32_t ID = 0x120;
	  CAN_RX(ID, RX_Message_ISR);*/
	  //xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);

	Serial.println(micros()-startTime);
	while(1);
  #endif

  #ifdef TEST_CAN_TX_ISR
	uint32_t startTime = micros();
  //Serial.println(startTime);
	for (int iter = 0; iter < 3; iter++) {
    //32
    CAN_TX_Task(NULL);
	}
    //CAN_RX_ISR();
    /*Serial.println("World");
  	uint8_t RX_Message_ISR[8] = {0};
	  uint32_t ID = 0x123;
    //uint32_t ID = 0x120;
	  CAN_RX(ID, RX_Message_ISR);*/
	  //xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);

	Serial.println(micros()-startTime);
	while(1);
  #endif

  //Serial.begin(9600);
  //Serial.println("World");
  #ifdef TEST_DOUBLE_ISR
	uint32_t startTime = micros();
  //Serial.println(startTime);
	for (int iter = 0; iter < 32; iter++) {
    //32
    doubleISR();
	}
    //CAN_RX_ISR();

	  //xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);

	Serial.println(micros()-startTime);
	while(1);
  #endif

  //start the RTOS scheduler
  vTaskStartScheduler();


}

void loop() {
  // put your main code here, to run repeatedly:

}