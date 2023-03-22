 #include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <math.h>

#include <SynthSetup.h>
#include <KnobControl.h>

// #include <global_variables.h>

void sampleISR()
{
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  Vout = Vout >> (8 - volume_level);

  analogWrite(OUTR_PIN, Vout + 128);

}

// Functions for the different tasks


uint32_t decode_keys(const uint32_t* step_sizes)
{
  uint32_t step = 0;

  //there are 3 key readings of interest

  for(int i = 0; i < 3; i++)
  {
    uint32_t negated_key = (~(*(key_array + i))) & 0b1111;
      
    if(negated_key & 0b0001)
    {
      step = *(step_sizes + 4*i + 0);
    }
    
    if (negated_key & 0b10)
    {
      step = *(step_sizes + 4*i + 1);
    }
    
    if (negated_key & 0b0100)
    {
      step = *(step_sizes + 4*i + 2);
    }
    
    if (negated_key & 0b1000)
    {
      step = *(step_sizes + 4*i + 3);
    } 

  }

  return step;
}

void decode_key_names(char* key_name_in, uint8_t number_of_keys_pressed, uint32_t* pressed_steps)
{
  char key_name_string [3*number_of_keys_pressed] = {}; // two characters per key + spaces in between

  for(int i = 0; i < number_of_keys_pressed; i++)
  {
    if(*(pressed_steps + i) == step_sizes[0])
    {
      key_name_string[3*i] = 'C';
      key_name_string[3*i+1] = ' ';
      key_name_string[3*i+2] = ' ';
    }
    else if(*(pressed_steps + i) == step_sizes[1])
    {
      key_name_string[3*i] = 'C';
      key_name_string[3*i+1] = '#';
      key_name_string[3*i+2] = ' ';
    }
    else if(*(pressed_steps + i) == step_sizes[2])
    {
      key_name_string[3*i] = 'D';
      key_name_string[3*i+1] = ' ';
      key_name_string[3*i+2] = ' ';
    }
    else if(*(pressed_steps + i) == step_sizes[3])
    {
      key_name_string[3*i] = 'D';
      key_name_string[3*i+1] = '#';
      key_name_string[3*i+2] = ' ';
    }
    else if(*(pressed_steps + i) == step_sizes[4])
    {
      key_name_string[3*i] = 'E';
      key_name_string[3*i+1] = ' ';
      key_name_string[3*i+2] = ' ';
    }
    else if(*(pressed_steps + i) == step_sizes[5])
    {
      key_name_string[3*i] = 'F';
      key_name_string[3*i+1] = ' ';
      key_name_string[3*i+2] = ' ';
    }
    else if(*(pressed_steps + i) == step_sizes[6])
    {
      key_name_string[3*i] = 'F';
      key_name_string[3*i+1] = '#';
      key_name_string[3*i+2] = ' ';
    }
    else if(*(pressed_steps + i) == step_sizes[7])
    {
      key_name_string[3*i] = 'G';
      key_name_string[3*i+1] = ' ';
      key_name_string[3*i+2] = ' ';
    }
    else if(*(pressed_steps + i) == step_sizes[8])
    {
      key_name_string[3*i] = 'G';
      key_name_string[3*i+1] = '#';
      key_name_string[3*i+2] = ' ';
    }
    else if(*(pressed_steps + i) == step_sizes[9])
    {
      key_name_string[3*i] = 'A';
      key_name_string[3*i+1] = ' ';
      key_name_string[3*i+2] = ' ';
    }
    else if(*(pressed_steps + i) == step_sizes[10])
    {
      key_name_string[3*i] = 'A';
      key_name_string[3*i+1] = '#';
      key_name_string[3*i+2] = ' ';
    }
    else if(*(pressed_steps + i) == step_sizes[11])
    {
      key_name_string[3*i] = 'B';
      key_name_string[3*i+1] = ' ';
      key_name_string[3*i+2] = ' ';
    }
    else
    {
      key_name_string[3*i] = ' ';
      key_name_string[3*i+1] = ' ';
      key_name_string[3*i+2] = ' ';
    }
  }

  Serial.println(key_name_string);
  strcpy(key_name_in, key_name_string);
}

void volume_control(uint8_t knobAB_reading, uint8_t knobS_reading)
{
  static Knob volume_knob(0, 8); //volume knob ID 3
  volume_knob.update_knob(knobAB_reading, knobS_reading);

  static uint8_t local_volume_level;
  local_volume_level = volume_knob.knob_data();

  __atomic_store_n(&volume_level, local_volume_level, __ATOMIC_RELAXED);
}


void scanKeysTask(void * pvParameters) {

  
  const TickType_t xFrequency = 30/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

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

    uint32_t step[1] = {currentStepSize};
  
    //print step size
    u8g2.setCursor(2, 20);

    // Serial.println(step[1]);
    char name_string[3];
    decode_key_names(name_string, 3, step);
    Serial.print("from main: ");
    Serial.println(name_string);
    u8g2.print(name_string);

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
  gen_step_array(); 


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

