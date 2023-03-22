#include <Arduino.h>
#include <global_variables.h>

 
 const uint32_t interval = 100; //Display update interval

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

 void PinModeSynthSetUp()
 {
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
 }

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



// Step Sizes for a the initial A frequency
 // {
//     51076057,   //step_C
//     54113197,   //step_C#
//     57330935,   //step_D
//     60740010,   //step_D# ////
//     64351799,   //step_E
//     68178356,   //step_F
//     72232452,   //step_F#
//     76527617,   //step_G
//     81078186,   //step_G#
//     85899346,   //step_A
//     91007187,   //step_A#
//     96418756,   //step_B
//   };