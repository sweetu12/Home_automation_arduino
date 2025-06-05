/*
 *  Home‑Automation Demo  (Uno R3)
 *  Ultrasonic door + lights
 *  SG90 servo (pin 8)
 *  Fan on L293D (EN=9, IN1=11, IN2=5)
 *  DHT11 on pin 7
 *  1602 LCD (RS=3  E=6  D4=A0  D5=A1  D6=A2  D7=A3)
 */

#include <Servo.h>
#include <DHT11.h>
#include <LiquidCrystal.h>

// Pin map 
const byte TRIG_PIN   = 12;
const byte ECHO_PIN   = 2;
const byte SERVO_PIN  = 8;
const byte LED_PIN    = 4;
const byte FAN_EN     = 9;
const byte FAN_IN1    = 11;
const byte FAN_IN2    = 5;
const byte DHT_PIN    = 7;

// 1602 in 4‑bit mode: RS, E, D4–D7 
LiquidCrystal lcd(3, 6, A0, A1, A2, A3);

// Control constants 
const float  DIST_OPEN      = 25.0;   // cm (open ≤ 25 cm for 2 s)
const float  DIST_CLOSE     = 30.0;   // cm (close ≥ 30 cm)
const unsigned long DOOR_DELAY  = 2000;   // ms to keep user present
const float  LIGHT_DIST     = 40.0;   // cm (turn lights on)
const unsigned long LIGHT_HOLD = 3000;   // ms after person leaves
const int    TEMP_ON_C      = 21;     // °C (fan on)
const int    TEMP_OFF_C     = 19;     // °C (fan off)
const unsigned long DHT_INT = 1000;   // ms sample rate
const unsigned long US_INT  = 20;    // ms between pings

// Globals 
Servo doorServo;
DHT11 dht(DHT_PIN);

unsigned long tDoor  = 0;
unsigned long tLight = 0;
unsigned long tDHT   = 0;
unsigned long tUS    = 0;

bool doorOpen  = false;
bool lightOn   = false;
bool fanOn     = false;

int  curTemp   = 0;
int  curHum    = 0;

// Ultrasonic filter
float distFilt = 0.0;


void setup() {
  
  // Basic IO 
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(FAN_EN, OUTPUT);
  pinMode(FAN_IN1, OUTPUT);
  pinMode(FAN_IN2, OUTPUT);

  // Servo 
  doorServo.attach(SERVO_PIN, 500, 2500);
  doorServo.write(0);               // locked

  // LCD 
  lcd.begin(16, 2);
  lcd.print("Home Automation");
  lcd.setCursor(0, 1);
  lcd.print("Initializing…");

  // Servo sweep test (shows power/gnd OK) 
  for (int a = 0; a <= 90; a += 10) { doorServo.write(a); delay(20); }
  for (int a = 90; a >= 0; a -= 10) { doorServo.write(a); delay(20); }
  doorServo.write(0);

  Serial.begin(115200);
}




void loop() {
  unsigned long now = millis();

  // 1 ) Ultrasonic every  ms 
  
  if (now - tUS >= US_INT) {
    tUS = now;
    float d = pingDistance();               // cm (0 if no echo)

    /* simple first‑order low‑pass */
    const float alpha = 0.2;                // 0<α≤1
    distFilt = (1 - alpha) * distFilt + alpha * d;
  }

  // 2 ) Servo
  
  if (!doorOpen) {
    if (distFilt <= DIST_OPEN) {
      if (tDoor == 0) tDoor = now;
      else if (now - tDoor >= DOOR_DELAY) {
        doorServo.write(90);          // unlock
        doorOpen = true;
        tDoor = 0;
      }
    } else tDoor = 0;
  } else {                            // already open
    if (distFilt >= DIST_CLOSE) {
      doorServo.write(0);             // lock
      doorOpen = false;
    }
  }

  // 3 ) LED
  
  if (distFilt <= LIGHT_DIST) {
    digitalWrite(LED_PIN, HIGH);
    lightOn = true;
    tLight = now;
  } else if (lightOn && now - tLight >= LIGHT_HOLD) {
    digitalWrite(LED_PIN, LOW);
    lightOn = false;
  }

  // 4 ) DHT & LCD
  
  if (now - tDHT >= DHT_INT) {
    tDHT = now;
    if (dht.readTemperatureHumidity(curTemp, curHum) == 0) {
      lcd.setCursor(0, 0);
lcd.print("T:");
lcd.print(curTemp);
lcd.print("C  H:");
lcd.print(curHum);
lcd.print("%  ");           // extra spaces clear leftovers

lcd.setCursor(0, 1);
lcd.print("Dist:");
lcd.print(distFilt, 1);     // one decimal place
lcd.print(" cm   ");        // pad to clear
    }
  }

  // 5 ) Fan
  if (!fanOn && curTemp >= TEMP_ON_C) {
    digitalWrite(FAN_IN1, HIGH);
    digitalWrite(FAN_IN2, LOW);
    analogWrite(FAN_EN, 255);
    fanOn = true;
  } else if (fanOn && curTemp <= TEMP_OFF_C) {
    analogWrite(FAN_EN, 0);
    fanOn = false;
  }
}

float pingDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long dur = pulseIn(ECHO_PIN, HIGH, 11500);
  
  if (dur == 0) return 0.0;
  
  return (34321.0 * dur * 1e-6) * 0.5;         
}
