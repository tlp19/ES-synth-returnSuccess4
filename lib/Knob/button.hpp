#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <math.h>

using namespace std;

// Reading from the keyboard
extern volatile uint8_t keyArray[7];
extern SemaphoreHandle_t keyArrayMutex;

class Button {
  private:
    int row;
    int column;
    bool pressed;

  public:
    Button(int _row, int _column) {
        row = _row;
        column = _column;
        pressed = false;
    }

    /// Analyse the output of the keymatrix read and compute the rotation of the knob
    void setCurrentState() volatile {
        // Get the 4rth byte of keyArray (where the data about the piano key presses is)
        uint8_t keyArrayR;
        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
        memcpy(&keyArrayR, (void*) &keyArray[row], sizeof(keyArray[row]));
        xSemaphoreGive(keyArrayMutex);

        // Iterate through the last 2 bits of the row's value to get the currentRotationState
        // Stored as {A,B}, so reverse from lab notes
        int currentPressedState = !((keyArrayR >> column) & 0x01);

        // Set the new rotation value using atomic store
        __atomic_store_n(&pressed, currentPressedState, __ATOMIC_RELAXED);
    }

    bool isPressed() volatile {
        return pressed;
    }
};