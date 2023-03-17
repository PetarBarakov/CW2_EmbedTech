#include <Arduino.h>
#include <U8g2lib.h>
#include <math.h>
#include <STM32FreeRTOS.h>

// Constants
const uint32_t interval = 100; // Display update interval
const uint32_t stepSizes[] = {51071066, 54116588, 57337813, 60734742, 64346419, 68172844, 72233541, 76528508, 81077269, 85899346, 91014262, 96422016};
const uint32_t stepSizesHalfUp[] = {54116588, 57337813, 60734742, 64346419, 68172844, 72233541, 76528508, 81077269, 85899346, 91014262, 96422016, 102155567};
const uint32_t stepSizesHalfDown[] = {48204667, 51071066, 54116588, 57337813, 60734742, 64346419, 68172844, 72233541, 76528508, 81077269, 85899346, 91014262};
const uint32_t stepSizesWholeUp[] = {57337813, 60734742, 64346419, 68172844, 72233541, 76528508, 81077269, 85899346, 91014262, 96422016, 102155567, 108230053};
const uint32_t stepSizesWholeDown[] = {45499147, 48204667, 51071066, 54116588, 57337813, 60734742, 64346419, 68172844, 72233541, 76528508, 81077269, 85899346};
volatile uint32_t currentStepSize;
volatile uint8_t keyArray[7];
const String notes[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", ""};
volatile uint8_t note_index_1;
volatile uint8_t note_index_2;
volatile uint8_t note_index_3;
volatile uint8_t note_index_4;
volatile uint8_t note_index_5;
volatile uint8_t note_index_6;
volatile uint8_t note_index_7;
volatile uint8_t note_index_8;
volatile uint8_t note_index_9;
volatile uint8_t note_index_10;
volatile uint8_t note_index_11;
volatile uint8_t note_index_12;
volatile uint32_t rotationVariableK3;
volatile uint32_t rotationVariableK2;
SemaphoreHandle_t keyArrayMutex;

// Pin definitions
// Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

// Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

// Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

// Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

// Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

// Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

// Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value)
{
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN, value);
  digitalWrite(REN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN, LOW);
}

// Function to read the collumns
uint8_t readCols()
{

  uint8_t C0 = digitalRead(C0_PIN);
  uint8_t C1 = digitalRead(C1_PIN);
  uint8_t C2 = digitalRead(C2_PIN);
  uint8_t C3 = digitalRead(C3_PIN);

  uint8_t full_row = C0 + (C1 << 1) + (C2 << 2) + (C3 << 3);
  return full_row;
}

//knob class
class knobClass {
  public:
    knobClass(uint8_t UL, uint8_t LL){
      upperLimit = UL;
      lowerLimit = LL;
      direction = 0;
      rotationVariable = 0;
      keyArrayTemp = 0b00;
      previousStateTemp = 0b00;
      click = 1;
     }

    void updateRotationVariable(uint8_t keyArrayIn, uint8_t previousStateIn, uint8_t keyArrayClick){
      keyArrayTemp = keyArrayIn;
      previousStateTemp = previousStateIn;
      click = keyArrayClick;

      if (previousStateTemp == 0b00 && keyArrayTemp == 0b01){
        rotationVariable = (rotationVariable + 1);
        direction = 1;
      }
      else if (previousStateTemp == 0b01 && keyArrayTemp == 0b00){
        rotationVariable = (rotationVariable - 1);
        direction = -1;
      }
      else if (previousStateTemp == 0b10 && keyArrayTemp == 0b11){
        rotationVariable = (rotationVariable - 1);
        direction = -1;
      }
      else if (previousStateTemp == 0b11 && keyArrayTemp == 0b10){
        rotationVariable = (rotationVariable + 1);
        direction = 1;
      }
      else if ((previousStateTemp ^ keyArrayTemp) == 0b11){
        rotationVariable = (rotationVariable + direction);
        direction = direction;
      }
      else{
        rotationVariable = rotationVariable;
      }
      if (click == 0b0000){
        rotationVariable = 0;
        direction = -1;
      }

    }

    uint32_t readCurrentValue(){
      return rotationVariable;
    } 

    void setLimits(){
      if(rotationVariable > upperLimit){
        rotationVariable = upperLimit;
      }
      if (rotationVariable < lowerLimit){
        rotationVariable = lowerLimit;
      }
    }
  
  private:
    uint8_t lowerLimit;
    uint8_t upperLimit;

    uint8_t direction;
    uint32_t rotationVariable;
    uint8_t previousStateTemp;
    uint8_t keyArrayTemp;
    uint8_t click;
};

uint32_t find_note(volatile uint8_t keyArray[7], uint32_t knob_value)
{
  uint32_t note_array[] = {};
  int len;
  int key_pressed = 0;


  if ((keyArray[0]&0b0001) == 0b0000)
  {
  __atomic_store_n(&note_index_1, ((0+knob_value-2) + 12)%12 , __ATOMIC_RELAXED);
  key_pressed = 1;
  
    if (knob_value == 0){
      return stepSizesWholeDown[0];
    }
    else if (knob_value == 1){
      return stepSizesHalfDown[0];
    }
    else if (knob_value == 2){
      return stepSizes[0];
    }
    else if (knob_value == 3){
      return stepSizesHalfUp[0];
    }
    else if (knob_value == 4){
      return stepSizesWholeUp[0];
    }
    
  }
  else{
     __atomic_store_n(&note_index_1, 12 , __ATOMIC_RELAXED);
  }
  if ((keyArray[0]&0b0010) == 0b0000)
  {
    __atomic_store_n(&note_index_2, ((1+knob_value-2) + 12)%12, __ATOMIC_RELAXED);
    key_pressed = 1;
    if (knob_value == 0){
      return stepSizesWholeDown[1];
    }
    else if (knob_value == 1){
      return stepSizesHalfDown[1];
    }
    else if (knob_value == 2){
      return stepSizes[1];
    }
    else if (knob_value == 3){
      return stepSizesHalfUp[1];
    }
    else if (knob_value == 4){
      return stepSizesWholeUp[1];
    }
  }
  else{
     __atomic_store_n(&note_index_2, 12 , __ATOMIC_RELAXED);
  }
  if ((keyArray[0]&0b0100) == 0b0000)
  {
    __atomic_store_n(&note_index_3, ((2+knob_value-2) + 12)%12, __ATOMIC_RELAXED);
    key_pressed = 1;
    if (knob_value == 0){
      return stepSizesWholeDown[2];
    }
    else if (knob_value == 1){
      return stepSizesHalfDown[2];
    }
    else if (knob_value == 2){
      return stepSizes[2];
    }
    else if (knob_value == 3){
      return stepSizesHalfUp[2];
    }
    else if (knob_value == 4){
      return stepSizesWholeUp[2];
    }
  }
  else{
     __atomic_store_n(&note_index_3, 12 , __ATOMIC_RELAXED);
  }
  if ((keyArray[0]&0b1000) == 0b0000)
  {
    __atomic_store_n(&note_index_4, ((3+knob_value-2) + 12)%12, __ATOMIC_RELAXED);
    key_pressed = 1;
    if (knob_value == 0){
      return stepSizesWholeDown[3];
    }
    else if (knob_value == 1){
      return stepSizesHalfDown[3];
    }
    else if (knob_value == 2){
      return stepSizes[3];
    }
    else if (knob_value == 3){
      return stepSizesHalfUp[3];
    }
    else if (knob_value == 4){
      return stepSizesWholeUp[3];
    }
  }
  else{
     __atomic_store_n(&note_index_4, 12 , __ATOMIC_RELAXED);
  }
  if ((keyArray[1]&0b0001) == 0b0000)
  {
    __atomic_store_n(&note_index_5, ((4+knob_value-2) + 12)%12, __ATOMIC_RELAXED);
    key_pressed = 1;
    if (knob_value == 0){
      return stepSizesWholeDown[4];
    }
    else if (knob_value == 1){
      return stepSizesHalfDown[4];
    }
    else if (knob_value == 2){
      return stepSizes[4];
    }
    else if (knob_value == 3){
      return stepSizesHalfUp[4];
    }
    else if (knob_value == 4){
      return stepSizesWholeUp[4];
    }
  }
  else{
     __atomic_store_n(&note_index_5, 12 , __ATOMIC_RELAXED);
  }
  if ((keyArray[1]&0b0010) == 0b0000)
  {
    __atomic_store_n(&note_index_6, ((5+knob_value-2) + 12)%12, __ATOMIC_RELAXED);
    key_pressed = 1;
    if (knob_value == 0){
      return stepSizesWholeDown[5];
    }
    else if (knob_value == 1){
      return stepSizesHalfDown[5];
    }
    else if (knob_value == 2){
      return stepSizes[5];
    }
    else if (knob_value == 3){
      return stepSizesHalfUp[5];
    }
    else if (knob_value == 4){
      return stepSizesWholeUp[5];
    }
  }
  else{
     __atomic_store_n(&note_index_6, 12 , __ATOMIC_RELAXED);
  }
  if ((keyArray[1]&0b0100) == 0b0000)
  {
    __atomic_store_n(&note_index_7, ((6+knob_value-2) + 12)%12, __ATOMIC_RELAXED);
    key_pressed = 1;
    if (knob_value == 0){
      return stepSizesWholeDown[6];
    }
    else if (knob_value == 1){
      return stepSizesHalfDown[6];
    }
    else if (knob_value == 2){
      return stepSizes[6];
    }
    else if (knob_value == 3){
      return stepSizesHalfUp[6];
    }
    else if (knob_value == 4){
      return stepSizesWholeUp[6];
    }
  }
  else{
     __atomic_store_n(&note_index_7, 12 , __ATOMIC_RELAXED);
  }
  if ((keyArray[1]&0b1000) == 0b0000)
  {
    __atomic_store_n(&note_index_8, ((7+knob_value-2) + 12)%12, __ATOMIC_RELAXED);
    key_pressed = 1;
    if (knob_value == 0){
      return stepSizesWholeDown[7];
    }
    else if (knob_value == 1){
      return stepSizesHalfDown[7];
    }
    else if (knob_value == 2){
      return stepSizes[7];
    }
    else if (knob_value == 3){
      return stepSizesHalfUp[7];
    }
    else if (knob_value == 4){
      return stepSizesWholeUp[7];
    }
  }
  else{
     __atomic_store_n(&note_index_8, 12 , __ATOMIC_RELAXED);
  }
  if ((keyArray[2]&0b0001) == 0b0000)
  {
    __atomic_store_n(&note_index_9, ((8+knob_value-2) + 12)%12, __ATOMIC_RELAXED);
    key_pressed = 1;
    if (knob_value == 0){
      return stepSizesWholeDown[8];
    }
    else if (knob_value == 1){
      return stepSizesHalfDown[8];
    }
    else if (knob_value == 2){
      return stepSizes[8];
    }
    else if (knob_value == 3){
      return stepSizesHalfUp[8];
    }
    else if (knob_value == 4){
      return stepSizesWholeUp[8];
    }
  }
  else{
     __atomic_store_n(&note_index_9, 12 , __ATOMIC_RELAXED);
  }
  if ((keyArray[2]&0b0010) == 0b0000)
  {
    __atomic_store_n(&note_index_10, ((9+knob_value-2) + 12)%12, __ATOMIC_RELAXED);
    key_pressed = 1;
    if (knob_value == 0){
      return stepSizesWholeDown[9];
    }
    else if (knob_value == 1){
      return stepSizesHalfDown[9];
    }
    else if (knob_value == 2){
      return stepSizes[9];
    }
    else if (knob_value == 3){
      return stepSizesHalfUp[9];
    }
    else if (knob_value == 4){
      return stepSizesWholeUp[9];
    }
  }
  else{
     __atomic_store_n(&note_index_10, 12 , __ATOMIC_RELAXED);
  }
  if ((keyArray[2]&0b0100) == 0b0000)
  {
    __atomic_store_n(&note_index_11, ((10+knob_value-2) + 12)%12, __ATOMIC_RELAXED);
    key_pressed = 1;
    if (knob_value == 0){
      return stepSizesWholeDown[10];
    }
    else if (knob_value == 1){
      return stepSizesHalfDown[10];
    }
    else if (knob_value == 2){
      return stepSizes[10];
    }
    else if (knob_value == 3){
      return stepSizesHalfUp[10];
    }
    else if (knob_value == 4){
      return stepSizesWholeUp[10];
    }
  }
  else{
     __atomic_store_n(&note_index_11, 12 , __ATOMIC_RELAXED);
  }
  if ((keyArray[2]&0b1000) == 0b0000)
  {
    __atomic_store_n(&note_index_12, ((11+knob_value-2) + 12)%12, __ATOMIC_RELAXED);
    key_pressed = 1;
    if (knob_value == 0){
      return stepSizesWholeDown[11];
    }
    else if (knob_value == 1){
      return stepSizesHalfDown[11];
    }
    else if (knob_value == 2){
      return stepSizes[11];
    }
    else if (knob_value == 3){
      return stepSizesHalfUp[11];
    }
    else if (knob_value == 4){
      return stepSizesWholeUp[11];
    }
  }
  else{
     __atomic_store_n(&note_index_12, 12 , __ATOMIC_RELAXED);
  }
  if (key_pressed == 0)
  {
    return 0;
  }
  
}

void setRow(uint8_t rowIdx)
{
  digitalWrite(REN_PIN, LOW);

  if (rowIdx % 2 == 1)
  {
    digitalWrite(RA0_PIN, HIGH);
  }
  else
  {
    digitalWrite(RA0_PIN, LOW);
  }
  if (rowIdx == 2 || rowIdx == 3 || rowIdx == 6 || rowIdx == 7)
  {
    digitalWrite(RA1_PIN, HIGH);
  }
  else
  {
    digitalWrite(RA1_PIN, LOW);
  }
  if (rowIdx >= 4)
  {
    digitalWrite(RA2_PIN, HIGH);
  }
  else
  {
    digitalWrite(RA2_PIN, LOW);
  }

  digitalWrite(REN_PIN, HIGH);
}


void scanKeysTask(void *pvParameters)
{
  uint8_t keyArrayTemporary[7];
  uint32_t variable_pitch;
  knobClass volumeKnob(8,0);
  knobClass testKnob(4,0);
  uint8_t previousState;
  uint32_t localCurrentStepSize;
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS; // divide any time in ms by portTICK_PERIOD_MS to convert to scheduler ticks
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    for (int i = 0; i <= 6; i++)
    {
      setRow(i);
      delayMicroseconds(3);
      uint8_t keys = readCols();
      keyArrayTemporary[i] = keys;
    }
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    keyArray[0] = keyArrayTemporary[0];
    keyArray[1] = keyArrayTemporary[1];
    keyArray[2] = keyArrayTemporary[2];
    keyArray[3] = keyArrayTemporary[3];
    keyArray[4] = keyArrayTemporary[4];
    keyArray[5] = keyArrayTemporary[5];
    keyArray[6] = keyArrayTemporary[6];
    xSemaphoreGive(keyArrayMutex);

  
    volumeKnob.updateRotationVariable(keyArrayTemporary[3] & 0b0011, previousState & 0b0011, (keyArrayTemporary[5]>>1) & 0b0001);
    testKnob.updateRotationVariable((keyArrayTemporary[3] & 0b1100)>>2, (previousState & 0b1100)>>2, (keyArrayTemporary[5]) & 0b0001);
    previousState = keyArrayTemporary[3];
    volumeKnob.setLimits();
    testKnob.setLimits();
    uint32_t volume = volumeKnob.readCurrentValue();
    uint32_t test = testKnob.readCurrentValue();
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    __atomic_store_n(&rotationVariableK3, volume, __ATOMIC_RELAXED);
    __atomic_store_n(&rotationVariableK2, test, __ATOMIC_RELAXED);
    xSemaphoreGive(keyArrayMutex);

    Serial.begin(9600);
    Serial.println(analogRead(JOYX_PIN));
    

    localCurrentStepSize = find_note(keyArray, test);
    __atomic_store_n(&currentStepSize,localCurrentStepSize, __ATOMIC_RELAXED);
  }
}

void sampleISR()
{
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  Vout = Vout >> (8 - rotationVariableK3);
  analogWrite(OUTR_PIN, Vout + 128);
}

void displayUpdateTask(void *pvParameters){
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS; // divide any time in ms by portTICK_PERIOD_MS to convert to scheduler ticks
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    u8g2.clearBuffer();                 // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2, 10, "Murilo!");     // write something to the internal memory
    u8g2.setCursor(2, 20);
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    u8g2.print(keyArray[0], HEX);
    u8g2.print(keyArray[1], HEX);
    u8g2.print(keyArray[2], HEX);
    u8g2.print(keyArray[3], HEX);
    u8g2.print(keyArray[5], HEX);
    u8g2.print(keyArray[6], HEX);
    xSemaphoreGive(keyArrayMutex);
    u8g2.setCursor(2, 30);
   
    u8g2.print(notes[note_index_1]);
    u8g2.print(notes[note_index_2]);
    u8g2.print(notes[note_index_3]);
    u8g2.print(notes[note_index_4]);
    u8g2.print(notes[note_index_5]);
    u8g2.print(notes[note_index_6]);
    u8g2.print(notes[note_index_7]);
    u8g2.print(notes[note_index_8]);
    u8g2.print(notes[note_index_9]);
    u8g2.print(notes[note_index_10]);
    u8g2.print(notes[note_index_11]);
    u8g2.print(notes[note_index_12]);
    
    
    u8g2.setCursor(60, 10);
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    u8g2.print(rotationVariableK3, DEC);
    u8g2.print(rotationVariableK2, DEC);
    xSemaphoreGive(keyArrayMutex);
    

    u8g2.sendBuffer(); // transfer internal memory to the display

    digitalToggle(LED_BUILTIN);
  }
}

void setup(){
  // put your setup code here, to run once:

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);

  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  // Set pin directions
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

  // Initialise display
  setOutMuxBit(DRST_BIT, LOW); // Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH); // Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH); // Enable display power supply

  // Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
      scanKeysTask,     // Function that implements the task //
      "scanKeys",       // Text name for the task
      128,               // Stack size in words, not bytes
      NULL,             // Parameter passed into task
      2,                // Task priority
      &scanKeysHandle); // Pointer to store the task handle

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
      displayUpdateTask,     // Function that implements the task //
      "displayUpdate",       // Text name for the task
      256,                   // Stack size in words, not bytes
      NULL,                  // Parameter passed into task
      1,                     // Task priority
      &displayUpdateHandle); // Pointer to store the task handle

  keyArrayMutex = xSemaphoreCreateMutex();
  vTaskStartScheduler(); // This has to be at the end of setup
}

void loop()
{
  // put your main code here, to run repeatedly:
}