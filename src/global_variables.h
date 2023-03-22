#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>


//Constants

  const int NUMBER_OF_KEYS = 12;
  const uint32_t sample_frequency = 22000; //22kHz
  const uint32_t key_A_freq = 440;

  volatile uint32_t currentStepSize;
  uint32_t step_sizes[NUMBER_OF_KEYS];

  volatile uint32_t key_array[3];
  volatile uint32_t knob_array;
//   volatile char key_name[2];
  volatile uint8_t volume_level;

  //CAN variable setup
  volatile uint8_t TX_Message[8] = {0};


  //mutex variable for the key_array variable
  SemaphoreHandle_t keyArrayMutex;
  SemaphoreHandle_t knobArrayMutex;


  

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);