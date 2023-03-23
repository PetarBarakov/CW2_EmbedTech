#include <Arduino.h>


//class for controlling knob
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