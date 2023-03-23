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

const uint32_t sinLUT[1024] = {
128,128,129,130,131,131,132,133,
134,135,135,136,137,138,138,139,
140,141,142,142,143,144,145,145,
146,147,148,149,149,150,151,152,
152,153,154,155,155,156,157,158,
158,159,160,161,162,162,163,164,
165,165,166,167,167,168,169,170,
170,171,172,173,173,174,175,176,
176,177,178,178,179,180,181,181,
182,183,183,184,185,186,186,187,
188,188,189,190,190,191,192,192,
193,194,194,195,196,196,197,198,
198,199,200,200,201,202,202,203,
203,204,205,205,206,207,207,208,
208,209,210,210,211,211,212,213,
213,214,214,215,215,216,217,217,
218,218,219,219,220,220,221,221,
222,222,223,224,224,225,225,226,
226,227,227,228,228,228,229,229,
230,230,231,231,232,232,233,233,
234,234,234,235,235,236,236,236,
237,237,238,238,238,239,239,240,
240,240,241,241,241,242,242,242,
243,243,243,244,244,244,245,245,
245,246,246,246,246,247,247,247,
248,248,248,248,249,249,249,249,
250,250,250,250,250,251,251,251,
251,251,252,252,252,252,252,252,
253,253,253,253,253,253,253,254,
254,254,254,254,254,254,254,254,
254,254,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,254,
254,254,254,254,254,254,254,254,
254,254,253,253,253,253,253,253,
253,252,252,252,252,252,252,251,
251,251,251,251,250,250,250,250,
250,249,249,249,249,248,248,248,
248,247,247,247,246,246,246,246,
245,245,245,244,244,244,243,243,
243,242,242,242,241,241,241,240,
240,240,239,239,238,238,238,237,
237,236,236,236,235,235,234,234,
234,233,233,232,232,231,231,230,
230,229,229,228,228,228,227,227,
226,226,225,225,224,224,223,222,
222,221,221,220,220,219,219,218,
218,217,217,216,215,215,214,214,
213,213,212,211,211,210,210,209,
208,208,207,207,206,205,205,204,
203,203,202,202,201,200,200,199,
198,198,197,196,196,195,194,194,
193,192,192,191,190,190,189,188,
188,187,186,186,185,184,183,183,
182,181,181,180,179,178,178,177,
176,176,175,174,173,173,172,171,
170,170,169,168,167,167,166,165,
165,164,163,162,162,161,160,159,
158,158,157,156,155,155,154,153,
152,152,151,150,149,149,148,147,
146,145,145,144,143,142,142,141,
140,139,138,138,137,136,135,135,
134,133,132,131,131,130,129,128,
128,127,126,125,124,124,123,122,
121,120,120,119,118,117,117,116,
115,114,113,113,112,111,110,110,
109,108,107,106,106,105,104,103,
103,102,101,100,100,99,98,97,
97,96,95,94,93,93,92,91,
90,90,89,88,88,87,86,85,
85,84,83,82,82,81,80,79,
79,78,77,77,76,75,74,74,
73,72,72,71,70,69,69,68,
67,67,66,65,65,64,63,63,
62,61,61,60,59,59,58,57,
57,56,55,55,54,53,53,52,
52,51,50,50,49,48,48,47,
47,46,45,45,44,44,43,42,
42,41,41,40,40,39,38,38,
37,37,36,36,35,35,34,34,
33,33,32,31,31,30,30,29,
29,28,28,27,27,27,26,26,
25,25,24,24,23,23,22,22,
21,21,21,20,20,19,19,19,
18,18,17,17,17,16,16,15,
15,15,14,14,14,13,13,13,
12,12,12,11,11,11,10,10,
10,9,9,9,9,8,8,8,
7,7,7,7,6,6,6,6,
5,5,5,5,5,4,4,4,
4,4,3,3,3,3,3,3,
2,2,2,2,2,2,2,1,
1,1,1,1,1,1,1,1,
1,1,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,1,
1,1,1,1,1,1,1,1,
1,1,2,2,2,2,2,2,
2,3,3,3,3,3,3,4,
4,4,4,4,5,5,5,5,
5,6,6,6,6,7,7,7,
7,8,8,8,9,9,9,9,
10,10,10,11,11,11,12,12,
12,13,13,13,14,14,14,15,
15,15,16,16,17,17,17,18,
18,19,19,19,20,20,21,21,
21,22,22,23,23,24,24,25,
25,26,26,27,27,27,28,28,
29,29,30,30,31,31,32,33,
33,34,34,35,35,36,36,37,
37,38,38,39,40,40,41,41,
42,42,43,44,44,45,45,46,
47,47,48,48,49,50,50,51,
52,52,53,53,54,55,55,56,
57,57,58,59,59,60,61,61,
62,63,63,64,65,65,66,67,
67,68,69,69,70,71,72,72,  
73,74,74,75,76,77,77,78,
79,79,80,81,82,82,83,84,
85,85,86,87,88,88,89,90,
90,91,92,93,93,94,95,96,
97,97,98,99,100,100,101,102,
103,103,104,105,106,106,107,108,
109,110,110,111,112,113,113,114,
115,116,117,117,118,119,120,120,
121,122,123,124,124,125,126,127};

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
              VoutIndividual[i] = sinLUT[phaseAcc[i] >> 24] - 128;
              //phaseAcc need to be in rad, from pi to -pi
              //LUT
              //power of 2 long
              //table itself from -pi to pi
              //1024 long
              //get top ten bits of phase accumulator, >>24
              //use that as index of table
              //const expression function or just calculate outside
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
    loopCondition = false;
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
    //xSemaphoreTake(senderBoolMutex, portMAX_DELAY);
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
    //xSemaphoreGive(senderBoolMutex);


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
  2048,      		/* Stack size in words, not bytes */
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
  CAN_Init(true);
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