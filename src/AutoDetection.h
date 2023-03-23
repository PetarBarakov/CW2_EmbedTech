#include <Arduino.h>
#include <STM32FreeRTOS.h>

#define NUM_OUTPUT_BITS 7

extern QueueHandle_t msgInQ;
extern QueueHandle_t msgOutQ;

//function to detect the position of the keyboard
bool* GenerateHandshake()
{
    bool outBits[NUM_OUTPUT_BITS];

    outBits[0] = 0;
    outBits[1] = 0;
    outBits[2] = 0;
    outBits[3] = 1;
    outBits[4] = 1;
    outBits[5] = 1;
    outBits[6] = 1;

    return outBits;
}

uint8_t GenerateHashID()
{   
    uint32_t ID[3];

    uint32_t ID[0] = IDHAL_GetUIDw0();
    uint32_t ID[1] = IDHAL_GetUIDw1();
    uint32_t ID[2] = IDHAL_GetUIDw2();

    uint8_t HashId;

    HashId = (ID[0] >> 26) + (ID[1] >> 26) + (ID[2] >> 26);

    return HashId;
}

void PositionTX(uint8_t current_position)
{
    uint8_t AutoDetectionTX[8];

    AutoDetectionTX[0] = 0; //keyTXmsg[0];
    AutoDetectionTX[1] = 0; //keyTXmsg[1];
    AutoDetectionTX[2] = 0; //keyTXmsg[2];
    AutoDetectionTX[3] = 0;
    AutoDetectionTX[4] = 0;
    AutoDetectionTX[5] = 0;
    AutoDetectionTX[6] = GenerateHashID();
    AutoDetectionTX[7] = current_position + 1;


    xQueueSend(msgOutQ, AutoDetectionTX, portMAX_DELAY);
    
}

uint8_t PositionRX()
{
    uint32_t msgReceived [8];
    xQueueReceive(msgInQ, msgReceived, portMAX_DELAY);

    return msgReceived[7];
    
}

uint8_t assign_possition (bool west_detect, bool east_detect)
{
    uint8_t position = 0;

    //positions are assigned east to west
    //the detect singnals are active low
    if(west_detect == 1)
    {
        if(east_detect == 0)
        {
            return PositionRX();
        }
        else
        {
            return 0;            
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

