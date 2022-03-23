#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <math.h>

using namespace std;

// Reading from the keyboard
extern volatile uint8_t keyArray[7];
extern SemaphoreHandle_t keyArrayMutex;

/// Takes in as argument:
///  - int row: The row index of the button in the key matrix
///  - int firstColumn: The column index of the button in the key matrix
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

    /// Analyse the output of the keymatrix read and compute the state of the button
    void setCurrentState() volatile {
        // Get the row of the keyArray where the data about the button press is
        uint8_t keyArrayR;
        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
        memcpy(&keyArrayR, (void*) &keyArray[row], sizeof(keyArray[row]));
        xSemaphoreGive(keyArrayMutex);

        // Take the binary number at the right column index
        int currentPressedState = !((keyArrayR >> column) & 0x01);

        // Set the new state value using atomic store
        __atomic_store_n(&pressed, currentPressedState, __ATOMIC_RELAXED);
    }

    /// Returns if the Button is being pressed or not
    bool isPressed() volatile {
        return pressed;
    }
};