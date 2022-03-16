#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <math.h>

using namespace std;

// Reading from the keyboard
extern volatile uint8_t keyArray[7];
extern SemaphoreHandle_t keyArrayMutex;

class Knob {
  private:
    int upperLimit;
    int lowerLimit;
    int row;
    int firstColumn;
    int prevState;
    int rotation;

    int decodeRotationStateChange(uint8_t prevState, uint8_t currentState) volatile {
        int rotationChange = 0;
        if(prevState == 0b00){
        // Regular state changes
        if (currentState == 0b01) {
            rotationChange = -1;
        } else if (currentState == 0b10) {
            rotationChange = +1;
        }
        // Skip state changes
        else if (currentState == 0b11) {
            rotationChange = -2;
        }   
        } else if (prevState == 0b01) {
        // Regular state changes
        if(currentState == 0b00){
            rotationChange = +1;
        } else if (currentState == 0b11) {
            rotationChange = -1; 
        }
        // Skip state changes
        else if (currentState == 0b10) {
            rotationChange = -2;
        }   
        } else if (prevState == 0b10) {
        // Regular state changes
        if(currentState == 0b00){
            rotationChange = -1;
        } else if (currentState == 0b11) {
            rotationChange = +1; 
        }
        // Skip state changes
        else if (currentState == 0b01) {
            rotationChange = -2;
        }   
        } else if (prevState == 0b11) {
        // Regular state changes
        if (currentState == 0b01) {
            rotationChange = +1;
        } else if (currentState == 0b10) {
            rotationChange = -1;
        }   
        // Skip state changes
        else if (currentState == 0b00) {
            rotationChange = -2;
        }   
        }
        return rotationChange;
    }

  public:
    Knob(int _row, int _firstColumn, int _lowerLimit, int _upperLimit) {
        lowerLimit = _lowerLimit;
        upperLimit = _upperLimit;
        row = _row;
        firstColumn = _firstColumn;
        prevState = 0b11;
        rotation = 8;
    }

    /// Analyse the output of the keymatrix read and compute the rotation of the knob
    void setCurrentRotation() volatile {
        // Define local variables
        uint8_t currentRotationState = 0b00;
        int localRotation = rotation;

        // Get the 4rth byte of keyArray (where the data about the piano key presses is)
        uint8_t keyArrayR;
        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
        memcpy(&keyArrayR, (void*) &keyArray[row], sizeof(keyArray[row]));
        xSemaphoreGive(keyArrayMutex);

        // Iterate through the last 2 bits of the row's value to get the currentRotationState
        // Stored as {A,B}, so reverse from lab notes
        int keyArrayValue0 = ((keyArrayR >> firstColumn) & 0x01) ? 0b1 : 0b0;
        int keyArrayValue1 = ((keyArrayR >> (firstColumn + 1)) & 0x01) ? 0b1 : 0b0;
        currentRotationState = (keyArrayValue0 << 1) + keyArrayValue1;

        if(currentRotationState != prevState) {
        // Compute the new roation based on previous rotation + rotation change
        localRotation = localRotation + decodeRotationStateChange(prevState, currentRotationState);
        if(localRotation < lowerLimit) {
            localRotation = lowerLimit;
        } else if(localRotation > upperLimit){
            localRotation = upperLimit;
        }
        Serial.println("setCurrentKnobRotation --");
        Serial.print("\tprevRotationState: (int) ");
        Serial.println(prevState);
        Serial.print("\tcurrentRotationState: ");
        Serial.print((keyArrayR & 0x01) ? 0b1 : 0b0);
        Serial.println(((keyArrayR >> 1) & 0x01) ? 0b1 : 0b0);
        Serial.print("\tlocalRotation: ");
        Serial.println(localRotation);

        // Set the new rotation value using atomic store
        __atomic_store_n(&rotation, localRotation, __ATOMIC_RELAXED);
        }
        // Return the current rotation state for next iteration
        prevState = currentRotationState;
    }

    int getRotation() volatile {
        return rotation;
    }
};