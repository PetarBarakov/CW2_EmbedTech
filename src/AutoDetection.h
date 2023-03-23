#include <Arduino.h>
#include <STM32FreeRTOS.h>

#define NUM_OUTPUT_BITS 7

extern QueueHandle_t msgInQ;
extern QueueHandle_t msgOutQ;

extern volatile uint32_t octave;

//function to detect the position of the keyboard
void GenerateHandshake(bool outBits[])
{

    outBits[0] = 0;
    outBits[1] = 0;
    outBits[2] = 0;
    outBits[3] = 1;
    outBits[4] = 1;
    outBits[5] = 1;
    outBits[6] = 1;

}

uint32_t* PositionTX(uint8_t current_position, uint8_t keyTXmsg[])
{
    uint8_t AutoDetectionTX[8];

    AutoDetectionTX[0] = keyTXmsg[0];
    AutoDetectionTX[1] = keyTXmsg[1];
    AutoDetectionTX[2] = keyTXmsg[2];
    AutoDetectionTX[3] = 0;
    AutoDetectionTX[4] = 0;
    AutoDetectionTX[5] = 0;
    AutoDetectionTX[6] = GenerateHashID();
    AutoDetectionTX[7] = current_position + 1;


    return AutoDetectionTX;
    
}

uint8_t PositionRX()
{
    uint32_t msgReceived [8];
    xQueueReceive(msgInQ, msgReceived, portMAX_DELAY);

    return msgReceived[7];
    
}



void assign_possition (bool west_detect, bool east_detect, uint8_t current_position, uint8_t TXposition, )
{

    //positions are assigned east to west
    //the detect singnals are active low
    if(west_detect == 1)
    {
        if(east_detect == 0)
        {   
            //receive position from east
        }
        else
        {
                       
        }
    }
    else // there is a western keyboard
    {
        if(east_detect == 1)
        {
            //this is the most eastern keyboard
            PositionTX(0);
            return 0;
        }
        else
        {
            uint8_t current_position = PositionRX();
            PositionTX(current_position);
            return current_position;
        }
    }
}


