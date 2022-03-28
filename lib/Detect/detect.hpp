#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <math.h>

using namespace std;


extern const int REN_PIN;
extern const int OUT_PIN;
// Reading from the keyboard
extern volatile uint8_t keyArray[7];
extern SemaphoreHandle_t keyArrayMutex;
extern void setRow(uint8_t rowIdx);

/// Takes in as argument:
///  - int row: The row index of the button in the key matrix
///  - int firstColumn: The column index of the button in the key matrix
class Detect {
  private:
    int row;
    int column;
    bool state;

  public:
    Detect(int _row, int _column) {
        row = _row;
        column = _column;
    }

    /// Analyse the output of the keymatrix read and update the state
    void writeToPin(bool value) volatile {
        setRow(row);
        digitalWrite(REN_PIN,LOW);
        digitalWrite(OUT_PIN,value);
        digitalWrite(REN_PIN,HIGH);
        delayMicroseconds(2);
        digitalWrite(REN_PIN,LOW);
    }

    /// Analyse the output of the keymatrix read and update the state
    void readFromPin() volatile {
        // Get the row of the keyArray where the data about the button press is
        uint8_t keyArrayR;
        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
        memcpy(&keyArrayR, (void*) &keyArray[row], sizeof(keyArray[row]));
        xSemaphoreGive(keyArrayMutex);

        // Take the binary number at the right column index
        int currentPressedState = !((keyArrayR >> column) & 0x01);

        // Set the new state value using atomic store
        __atomic_store_n(&state, currentPressedState, __ATOMIC_RELAXED);
    }

    /// Returns if the Button is being pressed or not
    bool getState() volatile {
        return state;
    }
};