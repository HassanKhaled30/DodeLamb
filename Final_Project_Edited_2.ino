#include <AccelStepper.h> // Library for stepper motor

#define motorPin1  5      // IN1 on the ULN2003 driver
#define motorPin2  6      // IN2 on the ULN2003 driver
#define motorPin3  7     // IN3 on the ULN2003 driver
#define motorPin4  8     // IN4 on the ULN2003 driver
#define STEP_PER_REVOLUTION 2048 // this value is from datasheet
#define MotorInterfaceType 8

#include <Wire.h> // For LCD 
#include <LiquidCrystal_I2C.h> // For LCD I2C Module

LiquidCrystal_I2C lcd(0x27, 16, 2); 
AccelStepper stepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);
int pos = 0;  // stepper motor pos
char dir = NULL;

int enA = 11; // LED analog
int in1 = 12; //LED polarity1
int in2 = 13; //LED polarity2

boolean isFirstTime = true; // for soft start opening
boolean timerDone = false; // For timer Value
boolean Pause = false; // for pause the led & motor

unsigned long previousTime = 0;
unsigned long interval = 15 * 60 * 1000; // 15 * 60 *1000 // 60000 = 1 Min edited from 10000
//////////////
// Rotary Encoder Inputs
#define CLK 2
#define DT 3
#define SW 4

int counter = 0; 
int currentStateCLK;
int lastStateCLK;
String currentDir = "";
unsigned long lastButtonPress = 0;

void setup() {
  stepper.setMaxSpeed(1000); // For Stepper Motor Speed
  stepper.setAcceleration(3000); // For Stepper Motor Acceleration 
  pinMode (10, INPUT); // LDR Sensor
  pinMode (enA, OUTPUT); // Analog for LED
  pinMode (in1, OUTPUT); // LED
  pinMode (in2, OUTPUT); // LED
  pinMode (A2, INPUT); // POT for intensity of light
  pinMode (9, INPUT_PULLUP); // push bottun
  Serial.begin(9600);
  //////////////////////////////
  //encoder pins as inputs
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(SW, INPUT_PULLUP);
  // Read the initial state of CLK
  lastStateCLK = digitalRead(CLK);
  ///////////////
  //LCD I2C welcome screen
  lcd.init();
  lcd.backlight();
  lcd.setCursor(1, 0);
  lcd.print("Hello, Friend");
  delay (3000);
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("Wish you");
  lcd.setCursor(2, 1);
  lcd.print("Sweet dreams");
  delay (3000);
  lcd.clear();
  previousTime = millis(); // for counting time
}

void loop() {
  unsigned long currentTime = millis(); 
  if (currentTime - previousTime >= interval) {  //timer
    timerDone = true;
    Serial.println("Sweet Dreams"); // Time OUT
    interval = 15 * 60 * 1000; // 15 min * 60 sec * 1000 milliesecond
    // LCD ending message
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print("Sweet Dreams");
    delay(1000);
    lcd.noBacklight();
    previousTime = currentTime;
  }
  if (digitalRead(10) == HIGH && !timerDone  && !Pause) { //Main Condition for LED & Motor
    pos += 1000; 
    stepper.moveTo(pos);
    stepper.run();
    pos = stepper.currentPosition();
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    if (isFirstTime) {
      for ( int i = 0 ; i < 50 ; i ++) {
        analogWrite(enA, i); //Soft Start for LED
        delay(15);
      }
      isFirstTime = false;
    }
    ///////////////
    analogWrite(enA, map(analogRead(A2), 0, 1023, 50, 255)); // for changing the intensity of light

  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    stepper.stop();
  }
  ////////////
  //pause condition
  if (digitalRead(9) == LOW) {
    Pause = !Pause;
    delay (500);
    Serial.println(Pause);
    if (Pause) {
      lcd.clear();
      lcd.setCursor(5, 0);
      lcd.print("Pause");
    }
    else {
      lcd.clear();
    }
  }
  ///////////////////////////
  //Rotary encoder for changing time or restart 
  // Read the current state of CLK
  currentStateCLK = digitalRead(CLK);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1) {

    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(DT) != currentStateCLK) {
      counter --;
      currentDir = "CCW";
    } else {
      // Encoder is rotating CW so increment
      counter ++;
      currentDir = "CW";
    }
    counter = constrain(counter, 0, 2.5 * 60 * 100); // rotary encoder values

    Serial.print("Direction: ");
    Serial.print(currentDir);
    Serial.print(" | Counter: ");
    Serial.println(counter);
    lcd.setCursor(13, 1); // preview the value on LCD Screen
    lcd.print(counter);
  }

  // Remember last CLK state
  lastStateCLK = currentStateCLK;

  // Read the button state
  int btnState = digitalRead(SW);

  //If we detect LOW signal, button is pressed
  if (btnState == LOW) {
    //if 50ms have passed since last LOW pulse, it means that the
    //button has been pressed, released and pressed again
    if (millis() - lastButtonPress > 100) {
      lcd.backlight();
      Serial.println("Time is Set"); //Button Pressed!
      lcd.clear();
      lcd.setCursor(2, 0);
      lcd.print("Time is set");
      lcd.setCursor(2, 1);
      lcd.print(interval);
      interval += constrain ( (counter * 1000 * 60 ), 0, 60 * 60 * 1000); 
      Serial.println(interval);
      counter = 0;
      timerDone = false;
    }

    // Remember last button press event
    lastButtonPress = millis();
  }


}
