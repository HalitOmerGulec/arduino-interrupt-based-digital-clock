// https://github.com/HalitOmerGulec
//Digital Clock With Using Interrupt Hardware via Arduino
//Before use this code don't forget the update date on RTC module. Code dont contain the date set commands.

#include <virtuabotixRTC.h>                     //Input Lib for RTC reads. I using the DS1302.
#include <avr/interrupt.h>                      //Input Lib for use Timer and Interrupts.
#include <Wire.h>                               //Input Lib for Disable I2C because i use A4 and A5 as segment control pin (i think this is better for less lag). If your rtc use I2C you dont disable I2C.
const uint8_t CLK = 7, DAT = 8, RST = 4;        // Settings RTC Pins.
virtuabotixRTC RTC(CLK, DAT, RST);              // Create the RTC function.
                                                
#define SegmentA 10                             // Set the pins. In this project they are output and HIGH=Active .
#define SegmentB 12
#define SegmentC 2
#define SegmentD A5
#define SegmentE 11
#define SegmentF A4
#define SegmentG A2
#define SegmentDot 13
#define DigitOneLS 3
#define DigitTwoLS 6
#define DigitThreeLS 5
#define DigitFourLS 9

#define LDRSense A0                             //I use an LDR because at nights we dont want the Flashlight on wall. HIGH Value=Dedected Lights.  |5V->LDR->LDRSensePin->R10k->GND|
                                                //Set some system values.
                                               
const uint8_t digitPins[4] = {DigitOneLS, DigitTwoLS, DigitThreeLS, DigitFourLS};

const uint8_t segmentPins[8] = {
  SegmentA, SegmentB, SegmentC, SegmentD,
  SegmentE, SegmentF, SegmentG, SegmentDot
};

const boolean numbers[10][7] = {
  {1,1,1,1,1,1,0}, {0,1,1,0,0,0,0}, {1,1,0,1,1,0,1}, {1,1,1,1,0,0,1}, {0,1,1,0,0,1,1},
  {1,0,1,1,0,1,1}, {1,0,1,1,1,1,1}, {1,1,1,0,0,0,0}, {1,1,1,1,1,1,1}, {1,1,1,1,0,1,1}
};

volatile uint8_t currentDigit = 0;
volatile uint8_t brightness = 0;
volatile uint8_t brightnessMAX = 32;           //Duty range of brightness. If you can see flicker, use smaller value.
volatile uint8_t timevalue[4] = {0, 0, 0, 0}; 
volatile uint8_t subStep = 0;
volatile uint16_t blinkValue=19500;             //This value control the dot blinking time.
uint8_t ldrCounter = 255;
uint8_t ldrCheck = 25;                          //This value set the time of the ldr check loop.
int16_t rawLdr=0;
uint16_t ldreadMAX=550;                         //This value is Maximum readed value by ldr; DONT FORGET the test and set.

                                                //This is Interrupt based clock print function. Used ISR because this way dont effected by cpu overloads and no lag while arduino working well.
ISR(TIMER2_COMPA_vect) {
  for (uint8_t i = 0; i < 4; i++) digitalWrite(digitPins[i], 0);

  subStep++;
  if (subStep >= brightnessMAX) {
    subStep = 0;
    currentDigit++;
    if (currentDigit >= 4) currentDigit = 0;
  }

  static uint32_t blinkCounter = 0; 
  static bool dotState = 0;
  blinkCounter++;
  if (blinkCounter >= blinkValue) { 
    dotState = !dotState; 
    blinkCounter = 0;
  }

  if (subStep < brightness) {
    for (uint8_t s = 0; s < 7; s++) {
      digitalWrite(segmentPins[s], numbers[timevalue[currentDigit]][s]);
    }

    if (currentDigit == 1 && dotState) {
      digitalWrite(SegmentDot, 1);
    } else {
      digitalWrite(SegmentDot, 0);
    }

    digitalWrite(digitPins[currentDigit], 1);
  } 
  else {
    digitalWrite(SegmentDot, 0);
    for (uint8_t i = 0; i < 4; i++) digitalWrite(digitPins[i], 0);
  }
}
                                                //Set the pins.
void setup() {
  for (uint8_t i = 0; i < 8; i++) pinMode(segmentPins[i], OUTPUT); 
  for (uint8_t i = 0; i < 4; i++) pinMode(digitPins[i], OUTPUT);

  Wire.end();                                   // Disable I2C for better A4,A5 pin performance.
  pinMode(LDRSense, INPUT);
  rawLdr = analogRead(LDRSense);                //Check the outside light.
  brightness = map(rawLdr, 0, ldreadMAX, 1, brightnessMAX); 
  if (brightness > brightnessMAX) brightness = brightnessMAX;
  if (brightness < 1) brightness = 1;
  ldrCounter = 0;
  delay(45);
  playStartupAnimation();                       //Some luxury open animations. If you dont like you can disable.
                                                //Settings of the Interrupt hardware.
  cli();
  TCCR2A = (1 << WGM21);
  TCCR2B = (1 << CS21);
  OCR2A  = 31;
  TIMSK2 = (1 << OCIE2A);
  sei();

}
                                                //Operating System is here.
void loop() {
  RTC.updateTime();
  uint8_t h = RTC.hours;
  uint8_t m = RTC.minutes;

  uint8_t v0 = h / 10;
  uint8_t v1 = h % 10;
  uint8_t v2 = m / 10;
  uint8_t v3 = m % 10;

if (ldrCounter >= ldrCheck) {  
    rawLdr = analogRead(LDRSense);
    brightness = map(rawLdr, 0, ldreadMAX, 1, brightnessMAX); 
    if (brightness > brightnessMAX) brightness = brightnessMAX;
    if (brightness < 1) brightness = 1;
    ldrCounter = 0;
}
  ldrCounter++;
  cli(); 
  timevalue[0] = v0;
  timevalue[1] = v1;
  timevalue[2] = v2;
  timevalue[3] = v3;
  sei();
  delay(200);                                   //I dont recommend under the 200ms for refresh.
}
                                                //And functions for start animation.
const uint8_t animDigits[] = {0, 0, 1, 2, 3, 3, 3, 3, 2, 1, 0, 0};
const uint8_t animSegs[]   = {SegmentF, SegmentA, SegmentA, SegmentA, SegmentA, SegmentB, SegmentC, SegmentD, SegmentD, SegmentD, SegmentD, SegmentE};

void playStartupAnimation() {
  for (uint16_t limit = 1; limit <= 12; limit++) {
    unsigned long startTime = millis();
    while (millis() - startTime < 150) { 
      for (uint16_t i = 0; i < limit; i++) {
        allDigitsOff();
        allSegmentsOff();
        
        digitalWrite(animSegs[i], 1);
        digitalWrite(digitPins[animDigits[i]], 1);
        
        delayMicroseconds(80); 
        
        allDigitsOff(); 
        delayMicroseconds(420); 
      }
    }
  }

  
  for (uint16_t f = 0; f < 3; f++) {
    unsigned long t1 = millis();
    while (millis() - t1 < 450) { 
      allDigitsOff(); allSegmentsOff();
      digitalWrite(SegmentDot, 1);
      for(uint16_t d=0; d<4; d++) digitalWrite(digitPins[d], 1);
      delayMicroseconds(50); 
      allDigitsOff();
      delayMicroseconds(450);
    }

    unsigned long t2 = millis();
    while (millis() - t2 < 420) { 
      for (uint16_t i = 0; i < 12; i++) {
        allDigitsOff(); allSegmentsOff();
        digitalWrite(animSegs[i], 1);
        digitalWrite(digitPins[animDigits[i]], 1);
        delayMicroseconds(80); 
        allDigitsOff();
        delayMicroseconds(420);
      }
    }
  }
  allDigitsOff();
  allSegmentsOff();
}

void allDigitsOff() { for (uint16_t i = 0; i < 4; i++) digitalWrite(digitPins[i], 0); }
void allSegmentsOff() { for (uint16_t i = 0; i < 8; i++) digitalWrite(segmentPins[i], 0); }
