#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <math.h>
#include "knob.hpp"
#include<ES_CAN.h>

//Constants
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

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

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

/// Select the row that you then want to read from in the key matrix
void setRow(uint8_t rowIdx){
  // Set Row-Select-Enable to LOW at first
  digitalWrite(REN_PIN, LOW);
  // Encode the index to drive the 3 Row selects
  switch (rowIdx)
  {
  case 0:
    digitalWrite(RA0_PIN, LOW);
    digitalWrite(RA1_PIN, LOW);
    digitalWrite(RA2_PIN, LOW);
    break;

  case 1:
    digitalWrite(RA0_PIN, HIGH);
    digitalWrite(RA1_PIN, LOW);
    digitalWrite(RA2_PIN, LOW);
    break;

  case 2:
    digitalWrite(RA0_PIN, LOW);
    digitalWrite(RA1_PIN, HIGH);
    digitalWrite(RA2_PIN, LOW);
    break;

  case 3:
    digitalWrite(RA0_PIN, HIGH);
    digitalWrite(RA1_PIN, HIGH);
    digitalWrite(RA2_PIN, LOW);
    break;

  case 4:
    digitalWrite(RA0_PIN, LOW);
    digitalWrite(RA1_PIN, LOW);
    digitalWrite(RA2_PIN, HIGH);
    break;

  case 5:
    digitalWrite(RA0_PIN, HIGH);
    digitalWrite(RA1_PIN, LOW);
    digitalWrite(RA2_PIN, HIGH);
    break;

  case 6:
    digitalWrite(RA0_PIN, LOW);
    digitalWrite(RA1_PIN, HIGH);
    digitalWrite(RA2_PIN, HIGH);
    break;

  case 7:
    digitalWrite(RA0_PIN, HIGH);
    digitalWrite(RA1_PIN, HIGH);
    digitalWrite(RA2_PIN, HIGH);
    break;
  
  default:
    digitalWrite(RA0_PIN, LOW);
    digitalWrite(RA1_PIN, LOW);
    digitalWrite(RA2_PIN, LOW);
    break;
  } 
  // Set Row-Select-Enable to HIGH to read the corresponding row
  digitalWrite(REN_PIN, HIGH);
}

/// Read the columns of the key matrix
uint8_t readCols() {
  // Read from the input columns
  int inputC0 = digitalRead(C0_PIN)?0x01:0x00;
  int inputC1 = digitalRead(C1_PIN)?0x01:0x00;
  int inputC2 = digitalRead(C2_PIN)?0x01:0x00;
  int inputC3 = digitalRead(C3_PIN)?0x01:0x00;

  return inputC0 + (inputC1<<1) + (inputC2<<2)+ (inputC3<<3);
}

/// Compute the stepsizes, based on a base frequency and a key index
int32_t computeStepSize(double freq, int offset) {
  int32_t sampl_freq = 22000;
  int32_t stepSize = (pow(2, 32) * freq) / sampl_freq;
  float offset_factor = pow(2.0, (1.0/12.0));
  return stepSize * pow(offset_factor, offset);
}
const int32_t stepSizes [] = {computeStepSize(440, -9), computeStepSize(440, -8), computeStepSize(440, -7), computeStepSize(440, -6), computeStepSize(440, -5),computeStepSize(440, -4), computeStepSize(440, -3), computeStepSize(440, -2), computeStepSize(440, -1), computeStepSize(440, 0), computeStepSize(440, 1), computeStepSize(440, 2)};
const char* keysList [] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
// Store the current stepSize in a volatile variable
volatile int32_t currentStepSize = 0;
// Store the currentKey being played
volatile char* currentKey;

// Reading from the keyboard
volatile uint8_t keyArray[7];
SemaphoreHandle_t keyArrayMutex;

// Initialise the array with all unpressed
volatile uint8_t keyArray_prev[7] = {1,1,1,1,1,1,1};

/// Analyse the output of the keymatrix read, and get which key is being pressed (also setting the right currentStepSize)
void setCurrentStepSize() {
  int32_t localCurrentStepSize = 0;
  // Iterate through the first 3 rows/3 first bytes of keyArray (where the data about the piano key presses is)
  for (int i=0 ; i <= 2 ; i++) {
    uint8_t keyArrayI;
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      memcpy(&keyArrayI, (void*) &keyArray[i], sizeof(keyArray[i]));
    xSemaphoreGive(keyArrayMutex);
    // Iterate through the last 4 bits of the row's value, checking each time if it is zero
    for (int j=0 ; j <= 3 ; j++) {
        bool isSelected = !(((keyArrayI >> j)) & 0x01);
        // If it is zero, then the key is being pressed
        if (isSelected) {
          localCurrentStepSize = stepSizes[i*4+j];
        }
    }
  }
  __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  // equivalent to currentStepSize = localCurrentStepSize;
}

/// Analyse the output of the keymatrix read, and get which key is being pressed (also setting the right currentStepSize)
const char* getCurrentKey() {
  const char* currentKey = "-";
  // Iterate through the first 3 rows/3 first bytes of keyArray (where the data about the piano key presses is)
  for (int i=0 ; i <= 2 ; i++) {
    uint8_t keyArrayI;
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      memcpy(&keyArrayI, (void*) &keyArray[i], sizeof(keyArray[i]));
    xSemaphoreGive(keyArrayMutex);
    // Iterate through the last 4 bits of the row's value, checking each time if it is zero
    for (int j=0 ; j <= 3 ; j++) {
        bool isSelected = !(((keyArrayI >> j)) & 0x01);
        // If it is zero, then the key is being pressed
        if (isSelected) {
          currentKey = keysList[i*4+j];
        }
    }
  }
  return currentKey;
}



// Global variable for rotation of Knob 3
volatile Knob knob3 = Knob(3, 0, 0, 16);

// Global variable for rotation of Knob 3
volatile Knob knob2 = Knob(3, 2, 0, 16);

// ========================  INTERRUPTS & THREADS  ===========================


// ------------------------------ INTERRUPTS ----------------------------

/// Output a sawtooth waveform to the speakers
void sampleISR() {
  // Build a sawtooth waveform
  static int32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = phaseAcc >> 24;
  // Adjust the volume based on the volume controller
  Vout = Vout >> (8 - knob3.getRotation()/2);
  analogWrite(OUTR_PIN, Vout + 128);
}

// -------------------------------- THREADS -----------------------------

// THREAD: Scan the keys and set the currentStepSize
void scanKeysTask(void * pvParameters) {

  // CAN Bus transmissable message
  uint8_t TX_Message[8]= {0};
  TX_Message[1] = 4;

  // Define parameters for how to run the thread
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;  //Initiation interval -> 50ms (have to div. by const. to get time in ms)
  TickType_t xLastWakeTime = xTaskGetTickCount();       //Store last initiation time

  uint8_t prevRotationState = 0b00;
  // Body of the thread (i.e. what it does)
  while(1){
    // Perform reading of the key matrix
    for (int i=0 ; i<=3 ; i++) {
      // Select the row in the matrix we want to read from
      setRow(i);
      // Small delay for parasitic capacitance between setRow and readCols
      delayMicroseconds(3);

      // Compute function in a local variable to minimize Mutex locking time
      uint8_t keyArrayI = readCols();
      // Store it in the mutex with a mememory copy
      xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
        // "assignment" of keyArray[i] using memcpy
        memcpy((void*) &keyArray[i], &keyArrayI, sizeof(keyArrayI));
      xSemaphoreGive(keyArrayMutex);
    }
    // Set the current stepSize, according to the key matrix
    setCurrentStepSize();
    // Set the current rotation of knob 3, according to the key matrix
    knob3.setCurrentRotation();
    knob2.setCurrentRotation();

    // Check for unsent updates and write to the CAN bus
    for (int i=0; i<=2; i++) {
      // Check for any changes that need to be sent on CAN
      if(keyArray[i] != keyArray_prev[i]) {
        for (int j=0 ; j <= 3 ; j++) {
          bool key_now = (((keyArray[i] >> j)) & 0x01);
          bool key_prev = (((keyArray_prev[i] >> j)) & 0x01);
          if(key_now != key_prev) {
            // This is the bit that is different
            if(key_now) {
              // Key is released
              TX_Message[0] = 'R';
            } else {
              // Key is pressed
              TX_Message[0] = 'P';
            }
            // Update the key index
            TX_Message[2] = i*4+j;
            // Register that the change has been sent
            keyArray_prev[i] ^= 1 << j;
          }
        }
      }
    } 

    // send the message over the bus using the CAN
    CAN_TX(0x123, TX_Message);

    // Delay the next execution until new initiation according to xFrequency
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// THREAD: Update the display on the device
void displayUpdateTask(void * pvParameters) {
  // Define parameters for how to run the thread
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;  //Initiation interval -> 100ms
  TickType_t xLastWakeTime = xTaskGetTickCount();       //Store last initiation time

  // Body of the thread (i.e. what it does)
  while(1){
    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_profont12_tf); // choose a suitable font

    // Use a Mutex to safely access keyArray: make local copies of relevant data to minimize locking time
    // copy keyArray[i] into a local variable to only lock mutex during copy operation
    uint8_t keyArray0, keyArray1, keyArray2;
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      memcpy(&keyArray0, (void*) &keyArray[0], sizeof(keyArray[0]));
    xSemaphoreGive(keyArrayMutex);
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      memcpy(&keyArray1, (void*) &keyArray[1], sizeof(keyArray[1]));
    xSemaphoreGive(keyArrayMutex);
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      memcpy(&keyArray2, (void*) &keyArray[2], sizeof(keyArray[0]));
    xSemaphoreGive(keyArrayMutex);

    uint32_t value = keyArray0 + (keyArray1 << 4) + (keyArray2 << 8);
    
    // b. Print the keyArray as a hexadecimal number
    u8g2.drawStr(2,10, "KeyArray:"); 
    u8g2.setCursor(60,10);
    u8g2.print(value,HEX);

    // c. Print the key to the screen
    u8g2.drawStr(2,20, "Key:"); 
    const char* key = getCurrentKey();
    u8g2.drawStr(30,20, key); 

    // d. Print the knob rotation to the screen
    u8g2.drawStr(90,30, "Vol:"); 
    u8g2.setCursor(116,30);
    u8g2.print(knob3.getRotation()); 

    u8g2.setCursor(70,30);
    u8g2.print(knob2.getRotation()); 

    uint32_t ID;
    uint8_t RX_Message[8]={0};

    // Poll for received messages
    while (CAN_CheckRXLevel())
          CAN_RX(ID, RX_Message);

    // Debug code for CAN Bus
    u8g2.drawStr(80,20, "CAN:"); 
    u8g2.setCursor(106,20);
    u8g2.print((char) RX_Message[0]);
    u8g2.print(RX_Message[1]);
    u8g2.print(RX_Message[2]);

    //Send to the display
    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);

    // Delay the next execution until new initiation according to xFrequency
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


/// =========================== ARDUINO SETUP & LOOP ===================================

void setup() {
  // put your setup code here, to run once:

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

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  for (int i=0; i<12 ; i++) {
    Serial.print("stepSizes[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(stepSizes[i]);
  }

  // ------ INITIALIZE COMMON RESSOURCES -----

  // Mutex to access safely the global keyArray variable
  keyArrayMutex = xSemaphoreCreateMutex();

  // ---- INITIALIZE INTERRUPTS AND THREADS: ----

  // Initialize timer for interrupt
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  // Initialize the thread to scan keys and set the currentStepSize
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,
    "scanKeys",
    64,
    NULL,
    2,
    &scanKeysHandle
  );

    // Initialize the thread to scan keys and set the currentStepSize
  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
    displayUpdateTask,
    "displayUpdate",
    256,
    NULL,
    1,
    &scanKeysHandle
  );

  // Initialise CAN bus
  CAN_Init(true);
  setCANFilter(0x123,0x7ff);
  CAN_Start();

  // Start the RTOS scheduler to run the threads
  vTaskStartScheduler();

  
}

void loop() {
  // put your main code here, to run repeatedly:
  // EMPTY as we are using threads
}