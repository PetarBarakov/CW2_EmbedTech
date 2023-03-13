 #include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <math.h>
#include <SynthSetup.h>
#include <KnobControl.h>


//Constants

  const int NUMBER_OF_KEYS = 12;
  const uint32_t sample_frequency = 22000; //22kHz
  const uint32_t key_A_freq = 440;

  volatile uint32_t currentStepSize;
  uint32_t step_sizes[NUMBER_OF_KEYS];

  volatile uint32_t key_array[3];
  volatile uint32_t knob_array;
  volatile char key_name[2];
  volatile uint8_t volume_level;

  //CAN variable setup
  volatile uint8_t TX_Message[8] = {0};


  //mutex variable for the key_array variable
  SemaphoreHandle_t keyArrayMutex;
  SemaphoreHandle_t knobArrayMutex;


  

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);


uint8_t readCols()
{

  bool C0_bit = digitalRead(C0_PIN);
  bool C1_bit = digitalRead(C1_PIN);
  bool C2_bit = digitalRead(C2_PIN);
  bool C3_bit = digitalRead(C3_PIN);

  uint8_t column_data = C0_bit + (C1_bit << 1) + (C2_bit << 2) + (C3_bit << 3);

  return column_data;
  
}

void setRow(uint8_t rowIdx)
{

  digitalWrite(REN_PIN, LOW);

  if(rowIdx & 0b001)
  {
    digitalWrite(RA0_PIN, HIGH);
  }else
  {
    digitalWrite(RA0_PIN, LOW);
  }
  
  if(rowIdx & 0b010)
  {
    digitalWrite(RA1_PIN, HIGH);
  }else
  {
    digitalWrite(RA1_PIN, LOW);
  }
  
  if(rowIdx & 0b100)
  {
    digitalWrite(RA2_PIN, HIGH);
  }else
  {
    digitalWrite(RA2_PIN, LOW);
  }

  digitalWrite(REN_PIN, HIGH);

}

void gen_step_array ()
{
  uint32_t current_freq = key_A_freq * pow( pow(2.0, 1.0/12.0), -9); //key C frequency

  for(int i = 0; i < NUMBER_OF_KEYS; i++)
  {
    step_sizes[i]= pow(2, 32) * current_freq / sample_frequency;
    
    current_freq *= pow(2.0, 1.0/12.0);
    
  }

}

uint32_t decode_keys(const uint32_t* step_sizes)
{
  uint32_t step = 0;
  *key_name = ' ';
  *(key_name + 1) = ' ';

  //there are 3 key readings of interest

  for(int i = 0; i < 3; i++)
  {
    uint32_t negated_key = (~(*(key_array + i))) & 0b1111;
      
    if(negated_key & 0b0001)
    {
      step = *(step_sizes + 4*i + 0);
      
      if(i == 0)
      {
        *key_name = 'C';
      }
      
      if(i == 1)
      {
        *key_name = 'E';
      }
      
      if(i == 2)
      {
        *key_name = 'G';
        *(key_name + 1) = '#';
      }

      break;      
    }
    
    if (negated_key & 0b10)
    {
      step = *(step_sizes + 4*i + 1);

      if(i == 0)
      {
        *key_name = 'C';
        *(key_name + 1) = '#';
      }else if(i == 1)
      {
        *key_name = 'F';
      }else if(i == 2)
      {
        *key_name = 'A';
      }

      break;
    }
    
    if (negated_key & 0b0100)
    {
      step = *(step_sizes + 4*i + 2);

      if(i == 0)
      {
        *key_name = 'D';
      }else if(i == 1)
      {
        *key_name = 'F';
        *(key_name + 1) = '#';
      }else if(i == 2)
      {
        *key_name = 'A';
        *(key_name + 1) = '#';
      }
      
      break;
    }
    
    if (negated_key & 0b1000)
    {
      step = *(step_sizes + 4*i + 3);
      
      if(i == 0)
      {
        *key_name = 'D';
        *(key_name + 1) = '#';
      }else if(i == 1)
      {
        *key_name = 'G';
      }else if(i == 2)
      {
        *key_name = 'B';
      }


      break;
    } 
  }

  return step;
}

void sampleISR()
{
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  Vout = Vout >> (8 - volume_level);

  analogWrite(OUTR_PIN, Vout + 128);

}

void volume_control(uint8_t knobAB_reading, uint8_t knobS_reading)
{
  static Knob volume_knob(0, 8); //volume knob ID 3
  volume_knob.update_knob(knobAB_reading, knobS_reading);

  static uint8_t local_volume_level;
  local_volume_level = volume_knob.knob_data();

  __atomic_store_n(&volume_level, local_volume_level, __ATOMIC_RELAXED);
}

// Functions for the different tasks


void scanKeysTask(void * pvParameters) {

  
  const TickType_t xFrequency = 30/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  gen_step_array(); 

  while (true)
  {

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    

    //read key presses
    uint32_t local_key_array[3];
    uint8_t knobAB_reading[4];
    bool knobS_reading[4];

    for(int i = 0; i <= 6; i++)
    {
      setRow(i);
      delayMicroseconds(3);

      if(i <= 2) local_key_array[i] = readCols();
      else
      {
        // read knobs from. 
        // The knob data is stored into the arrays in descending order.
        // knob_data[0] corresponds to knob3, knob_data[1] corresponds to knob 2 and so on.

        uint32_t row_data = readCols();

        if(i == 3){
          knobAB_reading[0] = row_data & 0b11;
          knobAB_reading[1] = (row_data & 0b1100) >> 2;
        }

        if(i == 4)
        {
          knobAB_reading[2] = row_data & 0b11;
          knobAB_reading[3] = (row_data & 0b1100) >> 2;
        }

        if(i == 5)
        {
          knobS_reading[0] = (row_data & 0b10) >> 1;
          knobS_reading[1] = (row_data & 0b1);
        }

        if(i == 6)
        {
          knobS_reading[2] = (row_data & 0b10) >> 1;
          knobS_reading[3] = (row_data & 0b1);
        }
      }
    }
    

    //Mutex protection while transfering the key detection data
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    
    for(int i = 0; i<= 2; i++)
    {
      key_array[i] = local_key_array[i];
    }

    xSemaphoreGive(keyArrayMutex);



    uint32_t localCurrentStepSize;
    localCurrentStepSize = decode_keys(step_sizes);

    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);

    volume_control(knobAB_reading[0], knobS_reading[0]);
    

    //CAN
    static uint32_t prev_key_array[3] = {0xF, 0xF, 0xF};

    if(prev_key_array[0] != key_array[0] || prev_key_array[1] != key_array[1] || prev_key_array[2] != key_array[2])
    {
      TX_Message[0] = 'P';
      TX_Message[1] = 4;
      TX_Message[2] = 9;
    }
    else if (prev_key_array[0] == key_array[0] || prev_key_array[1] == key_array[1] || prev_key_array[2] == key_array[2])
    {
      TX_Message[0] = 0;
      TX_Message[1] = 0;
      TX_Message[2] = 0;
    }
    
    for(int i = 0; i < 3; i++)
    {
      prev_key_array[i] = key_array[i];
    }
  }

}

void displayUpdateTask(void* pvParameters)
{
  const TickType_t xFrequency = interval/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(true)
  {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);

    //print the key_matrix reading of the 12 keys
    u8g2.setCursor(2,10);
    u8g2.print(key_array[0],HEX);
    u8g2.print(" ");
    u8g2.print(key_array[1],HEX);
    u8g2.print(" ");
    u8g2.print(key_array[2],HEX);
    u8g2.print(" ");

    
    xSemaphoreGive(keyArrayMutex);  
  
    //print step size
    u8g2.setCursor(2, 20);
    u8g2.print(key_name[0]);
    u8g2.print(key_name[1]);

    //print volume
    u8g2.setCursor(80, 10);
    u8g2.print("Vol: ");
    u8g2.print(volume_level);

    u8g2.setCursor(66,30);
    u8g2.print((char) TX_Message[0]);
    u8g2.print(TX_Message[1]);
    u8g2.print(TX_Message[2]);
    
    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);    
  }
}


void setup() {
   //Initialise UART
  Serial.begin(9600);

  PinModeSynthSetUp();

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise timer
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);

  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  //RTOS 
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(scanKeysTask, "scanKeys", 64, NULL, 1, &scanKeysHandle ); 
  //(#task function name, #text name, #word stack size, #parameter passed, #priority, #handle)

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(displayUpdateTask, "Display Task", 256, NULL, 2, &displayUpdateHandle);

  //Create the mutex
  keyArrayMutex = xSemaphoreCreateMutex();
  knobArrayMutex = xSemaphoreCreateMutex();

  vTaskStartScheduler();
}

void loop() {
  
}

