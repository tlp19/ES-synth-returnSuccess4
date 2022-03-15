#include <Arduino.h>
#include <U8g2lib.h>
#include <math.h>

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
volatile int32_t currentStepSize;

/// Analyse the output of the keymatrix read, and get which key is being pressed (also setting the right currentStepSize)
const char* setCurrentStepSizeAndGetKey(uint8_t keyArray[7]) {
  const char* currentKey = "";
  // Iterate through the first 3 rows/3 first bytes of keyArray (where the data about the piano key presses is)
  for (int i=0 ; i <= 2 ; i++) {
    // Iterate through the last 4 bits of the row's value, checking each time if it is zero
    for (int j=0 ; j <= 3 ; j++) {
        bool isSelected = !(((keyArray[i] >> j)) & 0x01);
        // If it is zero, then the key is being pressed
        if (isSelected) {
          currentStepSize = stepSizes[i*4+j];
          currentKey = keysList[i*4+j];
        }
    }
  }
  return currentKey;
}

/// Output a sawtooth waveform to the speakers
void sampleISR() {
  static int32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = phaseAcc >> 24;
  analogWrite(OUTR_PIN, Vout + 128);
}



/// ----------------------------------------------------------------------------------------------------------------


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

  // Initialize timer
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();
}



void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t next = millis();
  static uint32_t count = 0;

  if (millis() > next) {
    next += interval;

    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    // a. Text
    u8g2.drawStr(2,10,"Hello World!");  // write something to the internal memory
    // b. Number
    uint8_t keyArray[7];
    for (int i=0 ; i<=2 ; i++) {
      // Select the row in the matrix we want to read from
      setRow(i);
      // Small delay for parasitic capacitance
      delayMicroseconds(3);
      keyArray[i] = readCols();
    }
    
    uint32_t value = keyArray[0] + (keyArray[1] << 4) + (keyArray[2] << 8);
    u8g2.setCursor(2,20);
    u8g2.print(value,HEX);
    const char* key = setCurrentStepSizeAndGetKey(keyArray);
    u8g2.drawStr(2,30, key); 
    //Send to the display
    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}