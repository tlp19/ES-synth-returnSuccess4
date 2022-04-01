#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <math.h>

using namespace std;

// Reading from the keyboard
extern volatile uint8_t keyArray[7];
extern SemaphoreHandle_t keyArrayMutex;

extern volatile bool isReceiverBoard;
extern volatile uint8_t boardIndex;

extern QueueHandle_t msgOutQ;

/// Takes in as argument:
///  - int index: Index of the knob on the physical board (used to identify the knob in CAN messages)
///  - int row: The row index of the knob in the key matrix
///  - int firstColumn: The index of the first column of the 2 that are read from the key matrix
///  - int lowerLimit: The lower bound that the value of the rotation of the knob can have
///  - int upperLimit: The upper bound that the value of the rotation of the knob can have
///  - bool loops: If true, the value of the rotation of the knob will wrap-around to the lower bound when it exceeds the upper bound (and vice-versa)
class CAN_Knob {
  private:
    int index;
    int upperLimit;
    int lowerLimit;
    bool loops;
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
    CAN_Knob(int _index, int _row, int _firstColumn, int _lowerLimit, int _upperLimit, bool _loops) {
        index = _index;
        lowerLimit = _lowerLimit;
        upperLimit = _upperLimit;
        loops = _loops;
        row = _row;
        firstColumn = _firstColumn;
        prevState = 0b11;
        rotation = floor((_lowerLimit + _upperLimit) / 2);
    }

    /// Send the current rotation of the Knob as a CAN Message
    void sendRotationCANMsg() volatile {
        uint8_t TX_Message[8] = {0};
        TX_Message[0] = 'K';
        TX_Message[1] = boardIndex;
        TX_Message[2] = index;
        TX_Message[3] = rotation;
        xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
    }

    /// Analyse the output of the keymatrix read and compute the rotation of the knob
    void updateCurrentRotation() volatile {
        if(isReceiverBoard) {
            // Define local variables
            uint8_t currentRotationState = 0b00;
            int localRotation = rotation;

            // Get the row of keyArray where the data about the knob state is
            uint8_t keyArrayR;
            xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
            memcpy(&keyArrayR, (void*) &keyArray[row], sizeof(keyArray[row]));
            xSemaphoreGive(keyArrayMutex);

            // Iterate through the last 2 bits of the row's value to get the currentRotationState
            // Stored as {A,B}, so reverse from lab notes
            int keyArrayValue0 = ((keyArrayR >> firstColumn) & 0x01) ? 0b1 : 0b0;
            int keyArrayValue1 = ((keyArrayR >> (firstColumn + 1)) & 0x01) ? 0b1 : 0b0;
            currentRotationState = (keyArrayValue0 << 1) + keyArrayValue1;

            // If there has been a change of state
            if(currentRotationState != prevState) {
                // Compute the new rotation based on previous rotation + rotation change
                localRotation = localRotation + decodeRotationStateChange(prevState, currentRotationState);

                // If the 'loops' boolean is set to true, wrap-around
                if(loops){
                    if(localRotation < lowerLimit) {
                        localRotation = upperLimit;
                    } else if(localRotation > upperLimit){
                        localRotation = lowerLimit;
                    }
                // If not, block at the limit
                } else {
                    if(localRotation < lowerLimit) {
                        localRotation = lowerLimit;
                    } else if(localRotation > upperLimit){
                        localRotation = upperLimit;
                    }
                }

                // Set the new rotation value using atomic store
                __atomic_store_n(&rotation, localRotation, __ATOMIC_RELAXED);

                sendRotationCANMsg();
            }

            // Store the current rotation state for next iteration
            prevState = currentRotationState;
        }
    }

    /// Manually set the rotation value of a CAN_KNOB
    void setRotation(int value) volatile {
        int localRotation = value;
        // If the 'loops' boolean is set to true, wrap-around
        if(loops){
            if(localRotation < lowerLimit) {
                localRotation = upperLimit - (lowerLimit - localRotation - 1);
            } else if(localRotation > upperLimit){
                localRotation = lowerLimit - (upperLimit - localRotation - 1);
            }
        // If not, block at the limit
        } else {
            if(localRotation < lowerLimit) {
                localRotation = lowerLimit;
            } else if(localRotation > upperLimit){
                localRotation = upperLimit;
            }
        }
        __atomic_store_n(&rotation, localRotation, __ATOMIC_RELAXED);
    }

    /// Returns the current rotation value of the knob
    int getRotation() volatile {
        return rotation;
    }
};