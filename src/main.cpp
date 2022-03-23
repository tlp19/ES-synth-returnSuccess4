#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <math.h>
#include "knob.hpp"
#include "button.hpp"
#include <ES_CAN.h>

//Key Matrix knobs locations
  const int knob3Row = 3;
  const int knob2Row = 3;
  const int knob1Row = 4;
  const int knob0Row = 4;
  const int knob3FCol = 0;
  const int knob2FCol = 2;
  const int knob1FCol = 0;
  const int knob0FCol = 2;

//Key Matrix knob buttons location
 const int knob3ButtonRow = 5;
 const int knob3ButtonCol = 1;

//Key Matrix joystick button location
 const int joystickButtonRow = 5;
 const int joystickButtonCol = 2;

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

int32_t shiftByOctave(int32_t stepSize, int octave) {
  if ((octave-4) >= 0) {
    return stepSize = (stepSize << (octave-4));
  } else {
    return stepSize = (stepSize >> -(octave-4));
  }
}

const int32_t stepSizes [] = {computeStepSize(440, -9), computeStepSize(440, -8), computeStepSize(440, -7), computeStepSize(440, -6), computeStepSize(440, -5),computeStepSize(440, -4), computeStepSize(440, -3), computeStepSize(440, -2), computeStepSize(440, -1), computeStepSize(440, 0), computeStepSize(440, 1), computeStepSize(440, 2)};
const char* keysList [] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
// Store the current stepSize in a volatile variable
volatile int32_t currentStepSize = 0;
// Store the currentKey being played
volatile char* currentKey;

// Mutex to protect shared ressources
SemaphoreHandle_t keyArrayMutex;
// Reading from the keyboard
volatile uint8_t keyArray[7];
// Initialise the array with all unpressed
volatile uint8_t keyArray_prev[3] = {1,1,1};

// Global object for Knob 3
volatile Knob knob3 = Knob(knob3Row, knob3FCol, 1, 16, false);
volatile Button knob3Button = Button(knob3ButtonRow, knob3ButtonCol);
// Global object for Knob 2
volatile Knob knob2 = Knob(knob2Row, knob2FCol, 0, 9, false);
// Global object for Knob 1
volatile Knob knob1 = Knob(knob1Row, knob1FCol, 0, 5, true);

// Global object for Joystick button
volatile Button joystickButton = Button(joystickButtonRow, joystickButtonCol);

// Board state variables:
  // Global boolean to know if the board is muted or not
  volatile bool isMuted = false;
  // Global boolean to know if the board is a sender or receiver
  volatile bool isReceiverBoard = true;

// CAN Bus Message Receive Queue
QueueHandle_t msgInQ;
// Mutex to protect shared ressources
SemaphoreHandle_t RX_MessageMutex;
// Global Message variable
volatile uint8_t RX_Message[8] = {0};

// CAN Bus Message Receive Queue
QueueHandle_t msgOutQ;
// Mutex to protect shared ressources
SemaphoreHandle_t CAN_TX_Semaphore;

/// Analyse the output of the keymatrix read, and get which key is being pressed (also setting the right currentStepSize)
void setCurrentStepSize() {
  // Local variable for currentStepSize
  int32_t localCurrentStepSize;
  // CAN Bus transmissable message
  uint8_t TX_Message[8]= {0};

  // Iterate through the first 3 rows/3 first bytes of keyArray (where the data about the piano key presses is)
  for (int i=0 ; i <= 2 ; i++) {
    uint8_t keyArrayI;
    uint8_t keyArray_prevI;
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      memcpy(&keyArrayI, (void*) &keyArray[i], sizeof(keyArray[i]));
      memcpy(&keyArray_prevI, (void*) &keyArray_prev[i], sizeof(keyArray_prev[i]));
    xSemaphoreGive(keyArrayMutex);

    // Check if the keyArray has changed
    if(keyArrayI != keyArray_prevI) {
      // Iterate through the last 4 bits of the row's value, checking each time if it is zero
      for (int j=0 ; j <= 3 ; j++) {
        // If it is one, then the key is being pressed
        bool keyNowSelected = !(((keyArrayI >> j)) & 0x01);
        bool keyPrevSelected = !(((keyArray_prevI >> j)) & 0x01);
        // If the key state changed
        if(keyNowSelected != keyPrevSelected) {
          if(keyNowSelected) {
            // Key is pressed
            TX_Message[0] = 'P';
            localCurrentStepSize = stepSizes[i*4+j];
            localCurrentStepSize = shiftByOctave(localCurrentStepSize, knob2.getRotation());
          } else {
            // Key is released
            TX_Message[0] = 'R';
            localCurrentStepSize = 0;
          }
          // Update the octave
          TX_Message[1] = knob2.getRotation();
          // Update the key index
          TX_Message[2] = i*4+j;
          // Register that the change has been sent (relative toggle one/zero)
          keyArray_prevI ^= 1 << j;
          // Update the global keyArray_prev
          xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
            // "assignment" of keyArray_prev[i] using memcpy
            memcpy((void*) &keyArray_prev[i], &keyArray_prevI, sizeof(keyArray_prevI));
          xSemaphoreGive(keyArrayMutex);
        }
      }
      
      if(isReceiverBoard){
        // If Receiver board, set the currentStepSize to be played by the board
        __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
      } else {
        // Send the message over the bus using the CAN if the board is a sender
        xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
      }
    }
  }

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


// ========================  INTERRUPTS & THREADS  ===========================


// ------------------------------ INTERRUPTS ----------------------------

/// Output a sawtooth waveform to the speakers
void sampleISR() {
  if(!isMuted){
    // Build a sawtooth waveform
    static int32_t phaseAcc = 0;
    phaseAcc += currentStepSize;
    int32_t Vout = phaseAcc >> 24;
    // Adjust the volume based on the volume controller
    Vout = Vout >> (8 - knob3.getRotation()/2);
    analogWrite(OUTR_PIN, Vout + 128);
  }
}

// CAN Bus Message Queue ISR Writer
void CAN_RX_ISR (void) {
  uint8_t RX_Message_ISR[8];
  uint32_t ID;
  CAN_RX(ID, RX_Message_ISR);
  xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

// CAN Bus Message Queue ISR Writer
void CAN_TX_ISR (void) {
  xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

// -------------------------------- THREADS -----------------------------

// THREAD: Scan the keys and set the currentStepSize
void scanKeysTask(void * pvParameters) {
  // Define parameters for how to run the thread
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;  //Initiation interval -> 50ms (have to div. by const. to get time in ms)
  TickType_t xLastWakeTime = xTaskGetTickCount();       //Store last initiation time

  // Body of the thread (i.e. what it does)
  while(1){
    // Perform reading of the key matrix
    for (int i=0 ; i<=6 ; i++) {
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
    knob1.setCurrentRotation();
    // Set the state of the knob button objects
    knob3Button.setCurrentState();
    // Set the state of the joystick button object
    joystickButton.setCurrentState();

    if(joystickButton.isPressed()) {
      //set this board to be the sender
      __atomic_store_n(&isReceiverBoard, true, __ATOMIC_RELAXED);
      //send message to tell other boards to be receivers
      uint8_t TX_Message[8] = {0};
      TX_Message[0] = 'S';
      xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
    }

    static uint32_t lastSwitched = 0;
    if(knob3Button.isPressed() && ((micros()-lastSwitched) > 500000)) { //can't switch more than once every 0.5s
      // Change the mute state of the board
      __atomic_store_n(&isMuted, !isMuted, __ATOMIC_RELAXED);
      lastSwitched = micros();
    }

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

    // Print the current local key to the screen
    u8g2.drawStr(2,10, "Key:"); 
    const char* key = getCurrentKey();
    u8g2.drawStr(30,10, key); 

    // Print the knobs rotation to the screen
    u8g2.drawStr(90,30, "Vol:"); 
    if(!isMuted){
      u8g2.setCursor(116,30);
      u8g2.print(knob3.getRotation()); 
    } else {
      u8g2.drawStr(116,30, "X"); 
    }


    u8g2.drawStr(50,30, "Oct:"); 
    u8g2.setCursor(76,30);
    u8g2.print(knob2.getRotation());

    u8g2.setCursor(30,30);
    u8g2.print(knob1.getRotation()); 

    char* senderOrReceiver;
    if(isReceiverBoard) {
      senderOrReceiver = (char*)"R";
    } else {
      senderOrReceiver = (char*)"S";
    }
    u8g2.setCursor(2,30);
    u8g2.print(senderOrReceiver);
    
    // Print CAN bus messages
    u8g2.drawStr(75,10, "CAN:"); 
    u8g2.setCursor(100,10);
    // Make a local copy of last received message
    uint8_t localRX_Message[8];
    xSemaphoreTake(RX_MessageMutex, portMAX_DELAY);
      memcpy(&localRX_Message, (void*) &RX_Message, sizeof(RX_Message));
    xSemaphoreGive(RX_MessageMutex);

    u8g2.print((char) localRX_Message[0]);
    u8g2.print(localRX_Message[1]);
    u8g2.print(localRX_Message[2]);

    //Send to the display
    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);

    // Delay the next execution until new initiation according to xFrequency
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/// THREAD: Decode Thread for CAN Bus Communications
void decodeTask(void * pvParameters) {
  while(1) {
    uint8_t localRX_Message[8];
    xQueueReceive(msgInQ, localRX_Message, portMAX_DELAY);
    xSemaphoreTake(RX_MessageMutex, portMAX_DELAY);
      memcpy((void*) &RX_Message, &localRX_Message, sizeof(localRX_Message));
    xSemaphoreGive(RX_MessageMutex);

    if(RX_Message[0]=='S') {
      // Set the current board as a sender
      __atomic_store_n(&isReceiverBoard, false, __ATOMIC_RELAXED);
    }

    if(isReceiverBoard) {
      if(RX_Message[0]=='P') {
        int32_t localCurrentStepSize = 0;
        // Set the note accordingly
        localCurrentStepSize = stepSizes[RX_Message[2]];
        localCurrentStepSize = shiftByOctave(localCurrentStepSize, RX_Message[1]);
        __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
      } else if (RX_Message[0]=='R') {
        __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
      }
    }
  }
}

void CAN_TX_Task (void * pvParameters) {
  uint8_t msgOut[8];
  while(1) {
    xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
    xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
    CAN_TX(0x123, msgOut);
  }
}

/// =========================== ARDUINO SETUP & LOOP ===================================

void setup() {
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


  // ------ INITIALIZE COMMON RESSOURCES -----

  // Mutex to access safely the global keyArray variable
  keyArrayMutex = xSemaphoreCreateMutex();
  // Mutex to access safely the last RX_Message
  RX_MessageMutex = xSemaphoreCreateMutex();
  // Semaphore to handle safe sending of messages
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

  // ------ INITIALIZE QUEUES ------

  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(36,8);

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
    4,
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
    &displayUpdateHandle
  );

  // Initialize the thread to decode messages from the CAN Bus
  TaskHandle_t decodeHandle = NULL;
  xTaskCreate(
    decodeTask,
    "decode",
    256,
    NULL,
    2,
    &decodeHandle
  );

  // Initialize the thread to send messages to the CAN Bus
  TaskHandle_t CAN_TX_Handle = NULL;
  xTaskCreate(
    CAN_TX_Task,
    "CAN_TX",
    256,
    NULL,
    3,
    &CAN_TX_Handle
  );

    // Initialise CAN bus mode: true for single-board (loopback), false for multi-board
  CAN_Init(false);

  // Interrupt when receiving a CAN message
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  // Interrupt when sending a CAN message
  CAN_RegisterTX_ISR(CAN_TX_ISR);

  setCANFilter(0x123,0x7ff);
  CAN_Start();

  // Start the RTOS scheduler to run the threads
  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
  // EMPTY as we are using threads
}