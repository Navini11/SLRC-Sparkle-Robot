#include <L298N.h>
#include <Arduino.h>
#include "Adafruit_VL53L0X.h"
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//////////////
// Have to redefine those below pins as we plugged
#define AIN1 26  //IN3
#define BIN1 22  //IN1
#define AIN2 28  //IN4
#define BIN2 24  //IN2
#define PWMA 10
#define PWMB 11
//motor1=right
////////

#define NUM_SENSORS 8
#define NUM_READINGS 250  // Number of readings to average for calibration

int sensorValueArray[NUM_SENSORS * NUM_READINGS];  // Array to store sensor readings



// Define switch pin numbers
int switch_1_calib = 5;  // Auto calibration
int switch_2_main = 4;   // Main code


LiquidCrystal_I2C lcd(0x27, 16, 2);

String wall_colour = "GREEN";
String path_colour;
String metal;
String metal_box_position;

//////ultrasonic pins
#define ftrig 53
#define fecho 51
long fdistance;

Servo boxArm;
Servo upArm;
Servo rotateArm;
Servo arm;

// Assign pin numbers to each servo
const int boxArmPin = 36;  //black
const int upArmPin = 38;
const int rotateArmPin = 40;  //red
const int armPin = 45;
#define metal_detector_pin 43
// Assigning pins to the colour sensor
#define S0 13
#define S1 12
#define S2 7
#define S3 9
#define sensorOut 8

int greenMin = 26;
int greenMax = 148;
int blueMin = 18;
int blueMax = 112;

// Address assignment for the two sensors
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x33
#define LOX4_ADDRESS 0x35

// Pins to control shutdown for the two sensors
#define SHT_LOX1 35
#define SHT_LOX2 50
#define SHT_LOX3 48
#define SHT_LOX4 41

// Objects for the VL53L0X sensors
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();

// Variables to store sensor measurements
int sensor1, sensor2, sensor3, sensor4;

// Measurement data structures for the two sensors
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
VL53L0X_RangingMeasurementData_t measure4;

bool tof = true;

//new one motor 1 26 28 //8 in2 in1  //out1
//motor 2  22 24//9 //in4 in3    //out 3


// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

// Encoder pins (must be interrupt-capable pins)
const int encoderAPin = 2;  // Encoder A pin
const int encoderBPin = 3;  // Encoder B pin
//encoderA_remaining pin=44
//encoderA_remaining pin=46

// Volatile variables used by the encoder
volatile long encoderACount = 0;
volatile long encoderBCount = 0;

// Function prototypes
void encoderAHandler();
void encoderBHandler();

// Constants
const int SensorCount = 8;                                                           // Total number of sensors
const int analogSensorCount = 8;                                                     // Number of analog sensors
const int analogSensorPins[analogSensorCount] = { A0, A1, A2, A3, A4, A5, A6, A7 };  // Analog pins for the remaining sensors
int IR_weight[8] = { -50, -30, -15, -5, 5, 15, 30, 50 };
float errorArray[50] = { 0 };
int sensorValues[SensorCount];
int Front_IR_Pin = A9;

// PID control parameters
float Kp = 5;  // Proportional term
float Ki = 0;  // Integral term
float Kd = 5;  // Derivative term
float sum;
int increment_count = -1;

float error;

// PID variables
float P, I, D, previousError = 0;

float lsp, rsp;
int speed = 130;
int turns_speed = 160;
int lfspeed = speed;
int Front_IR;
int globel_threshold = 175;

// Function prototypes
void readSensors(int *values);
float calculatePID(int *sensorValues);
void motor_drive(int left, int right);
void turnLeft(int *sensorValues);
void turnRight(int *sensorValues);
void noLine();

#define recieved_led_1 23
#define recieved_led_2 25

#define send_colour_request 27
#define send_shape_request 29

#define led_blue 31
#define led_green 33

void setup() {
  digitalWrite(send_colour_request, HIGH);
  delay(500);
  // Corrected to send_shape_request

  digitalWrite(send_colour_request, LOW);
  digitalWrite(send_shape_request, LOW);

  pinMode(switch_1_calib, INPUT_PULLUP);
  pinMode(switch_2_main, INPUT_PULLUP);

  if (digitalRead(switch_1_calib) == LOW) {
    increment_count = 2000;
  } else if (digitalRead(switch_2_main) == LOW) {
    increment_count = -1;
    lcd.clear();
    lcd.print("MAIN");
  }


  // Initialize analog sensor pins as input
  for (int i = 0; i < analogSensorCount; i++) {
    pinMode(analogSensorPins[i], INPUT);
  }
  pinMode(Front_IR_Pin, INPUT);

  // Attach servos to their pins
  boxArm.attach(boxArmPin);
  upArm.attach(upArmPin);
  rotateArm.attach(rotateArmPin);
  arm.attach(armPin);

  // Initialize motor driver pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(ftrig, OUTPUT);
  pinMode(fecho, INPUT);

  pinMode(metal_detector_pin, INPUT);

  // Set up sensor shutdown pins as outputs
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  pinMode(SHT_LOX4, OUTPUT);


  // Initialize sensor setup
  //setupSensors();

  lcd.init();

  lcd.backlight();

  //Colour sensor setup
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  // Set Frequency scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  delay(1000);
  zip_mode();
  lcd.clear();

  // Set up encoder interrupts
  pinMode(encoderAPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderAPin), encoderAHandler, RISING);

  pinMode(encoderBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderBPin), encoderBHandler, RISING);

  pinMode(send_colour_request, OUTPUT);
  pinMode(send_shape_request, OUTPUT);
  pinMode(recieved_led_1, INPUT);  // Corrected syntax
  pinMode(recieved_led_2, INPUT);
  pinMode(led_blue, OUTPUT);
  pinMode(led_green, OUTPUT);

  digitalWrite(send_colour_request, LOW);
  digitalWrite(send_shape_request, LOW);

  // Start with motors stopped
  motor1.stop();
  motor2.stop();
  Serial.begin(9600);
}

void loop() {
  int Front_IR_Analog = analogRead(Front_IR_Pin);
  if (Front_IR_Analog < 100) {
    Front_IR = 0;
  } else {
    Front_IR = 1;
  }
  readSensors(sensorValues);
  fUltrasonic_read();
  if (increment_count == -1) {
    while (true) {
      digitalWrite(send_shape_request, LOW);
      digitalWrite(send_colour_request, HIGH);
      if ((digitalRead(recieved_led_1) == HIGH) || (digitalRead(recieved_led_2) == HIGH)) {
        break;
      }
    }
    digitalWrite(send_shape_request, LOW);
    digitalWrite(send_colour_request, HIGH);  // Corrected to send_shape_request
    increment_count += 1;
  } else if (increment_count == 0) {
    lcd.clear();
    lcd.print("increment 0");
    if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 1) {
      increment(sensorValues);
      digitalWrite(send_shape_request, LOW);
      digitalWrite(send_colour_request, HIGH);  // Corrected to send_shape_request
    }
  } else if (increment_count == 1) {
    lcd.clear();
    lcd.print("increment 1");
    if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && Front_IR == 1) {
      colour_detection();
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 2) {
    lcd.clear();
    lcd.print("increment 2");
    encoder_refresh();
    encoder_back();
    motor1.stop();
    motor2.stop();
    delay(300);
    increment_count += 1;
  } else if (increment_count == 3) {
    lcd.clear();
    lcd.print("increment 3");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1) || (sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(300);
      turnLeft(sensorValues);
      increment_count += 1;
    } else {
      go_straight_backward();
    }
  } else if (increment_count == 4) {
    lcd.clear();
    lcd.print("increment 4");
    if (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 0) {
      motor1.stop();
      motor2.stop();
      delay(300);
      turnRight(sensorValues);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 5) {
    lcd.clear();
    lcd.print("increment 5");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(1500);
      lcd.clear();
      lcd.print("");
      turnLeft(sensorValues);
      increment_count += 1;
      lcd.clear();
      lcd.print("Turn OK");
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 6) {
    lcd.clear();
    lcd.print("increment 6");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 1)) {
      motor1.stop();
      motor2.stop();
      delay(2000);
      if (detectColor() == 1) {
        path_colour = "BLUE";
        lcd.clear();
        lcd.print("BLUE");
      } else {
        path_colour = "GREEN";
        lcd.clear();
        lcd.print("GREEN");
      }
      delay(1000);
      line_follow_encoders(11);
      if (path_colour == "BLUE" && wall_colour == "GREEN") {
        turnLeft_t(sensorValues);
        increment_count += 1;
      } else if (path_colour == "BLUE" && wall_colour == "BLUE") {
        turnRight_t(sensorValues);
        increment_count += 6;
      } else if (path_colour == "GREEN" && wall_colour == "BLUE") {
        turnLeft_t(sensorValues);
        increment_count += 1;
      } else if (path_colour == "GREEN" && wall_colour == "GREEN") {
        turnRight_t(sensorValues);
        increment_count += 6;
      }
      digitalWrite(led_green, LOW);
      digitalWrite(led_blue, LOW);
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 7) {
    lcd.clear();
    lcd.print("increment 7");
    if (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 0) {
      motor1.stop();
      motor2.stop();
      delay(300);
      turnRight(sensorValues);
      increment_count += 1;
    } else {
      line_follow_circle(sensorValues);
    }
  } else if (increment_count == 8) {
    lcd.clear();
    lcd.print("increment 8");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && Front_IR == 0) || (sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 0)) {
      motor1.stop();
      motor2.stop();
      delay(4000);
      shape_detector();
      turnLeft(sensorValues);
      increment_count += 1;
    } else {
      line_follow_circle(sensorValues);
    }
  } else if (increment_count == 9) {
    lcd.clear();
    lcd.print("increment 9");
    if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1) {
      motor1.stop();
      motor2.stop();
      delay(300);
      turnLeft(sensorValues);
      increment_count += 1;
    } else {
      line_follow_circle(sensorValues);
    }
  } else if (increment_count == 10) {
    lcd.clear();
    lcd.print("increment 10");
    if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && Front_IR == 0) {
      motor1.stop();
      motor2.stop();
      delay(300);
      turnLeft(sensorValues);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 11) {
    lcd.clear();
    lcd.print("increment 11");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 1)) {
      motor1.stop();
      motor2.stop();
      delay(1000);
      fUltrasonic_read();
      line_follow_encoders(11);
      turnLeft_t(sensorValues);
      increment_count += 6;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 12) {
    lcd.clear();
    lcd.print("increment 12");
    if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && Front_IR == 0) {
      motor1.stop();
      motor2.stop();
      delay(300);
      turnLeft(sensorValues);
      increment_count += 1;
    } else {
      line_follow_circle(sensorValues);
    }
  } else if (increment_count == 13) {
    lcd.clear();
    lcd.print("increment 13");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && Front_IR == 0) || (sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 0)) {
      motor1.stop();
      motor2.stop();
      delay(4000);
      shape_detector();
      turnLeft(sensorValues);
      increment_count += 1;
    } else {
      line_follow_circle(sensorValues);
    }
  } else if (increment_count == 14) {
    lcd.clear();
    lcd.print("increment 14");
    if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1) {
      motor1.stop();
      motor2.stop();
      delay(300);
      turnLeft(sensorValues);
      increment_count += 1;
    } else {
      line_follow_circle(sensorValues);
    }
  } else if (increment_count == 15) {
    lcd.clear();
    lcd.print("increment 15");
    if (sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && sensorValues[8] == 1 && Front_IR == 0) {
      motor1.stop();
      motor2.stop();
      delay(300);
      turnRight(sensorValues);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 16) {
    lcd.clear();
    lcd.print("increment 16");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 1)) {
      motor1.stop();
      motor2.stop();
      delay(1000);
      fUltrasonic_read();
      line_follow_encoders(11);
      turnRight_t(sensorValues);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 17) {
    lcd.clear();
    lcd.print("increment 17");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && Front_IR == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 1)) {
      motor1.stop();
      motor2.stop();
      delay(300);
      lcd.clear();
      lcd.print("Before Turn");
      turnRight(sensorValues);
      lcd.clear();
      lcd.print("After Turn");
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 18) {
    fUltrasonic_read();  //Towards BOX 1
    lcd.clear();
    lcd.print("increment 18");
    if ((fdistance <= 3)) {
      ultrasonic_detection();
      increment_count += 1;
    } else if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && Front_IR == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 1)) {
      motor1.stop();
      motor2.stop();
      lcd.clear();
      lcd.print("Front Line");
      delay(300);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 19) {
    lcd.clear();
    lcd.print("increment 19");
    white_back(sensorValues);
    motor1.stop();
    motor2.stop();
    delay(300);
  } else if (increment_count == 20) {
    lcd.clear();
    lcd.print("increment 20");
    encoder_refresh();
    encoder_back();
    increment_count += 1;
  } else if (increment_count == 21) {
    lcd.clear();
    lcd.print("increment 21");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(300);
      increment_count += 1;
    } else {
      go_straight_backward();
    }
  } else if (increment_count == 22) {
    if (metal == "METAL") {
      metal_box_position = "POSITION 1";
      increment(sensorValues);
      increment_count += 10;
    } else {
      iterative_PID(300);
      turnLeft(sensorValues);
      increment_count += 1;
    }
  } else if (increment_count == 23) {
    fUltrasonic_read();  //Towards BOX 2
    lcd.clear();
    lcd.print("increment 23");
    if ((fdistance <= 3)) {
      ultrasonic_detection();
      increment_count += 1;
    } else if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && Front_IR == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 1)) {
      motor1.stop();
      motor2.stop();
      lcd.clear();
      lcd.print("Front Line");
      delay(300);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 24) {
    lcd.clear();
    lcd.print("increment 24");
    white_back(sensorValues);
    motor1.stop();
    motor2.stop();
    delay(300);
  } else if (increment_count == 25) {
    lcd.clear();
    lcd.print("increment 25");
    encoder_refresh();
    encoder_back();
    increment_count += 1;
  } else if (increment_count == 26) {
    lcd.clear();
    lcd.print("increment 26");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(300);
      increment_count += 1;
    } else {
      go_straight_backward();
    }
  } else if (increment_count == 27) {
    if (metal == "METAL") {
      metal_box_position = "POSITION 2";
      increment(sensorValues);
      increment_count += 5;
    } else {
      iterative_PID(300);
      turnLeft(sensorValues);
      increment_count += 1;
    }
  } else if (increment_count == 28) {
    fUltrasonic_read();
    lcd.clear();
    lcd.print("increment 28");
    if ((fdistance <= 3)) {
      ultrasonic_detection();  //Towards BOX 3
      increment_count += 1;
    } else if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && Front_IR == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 1)) {
      motor1.stop();
      motor2.stop();
      lcd.clear();
      lcd.print("Front Line");
      delay(300);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 29) {
    lcd.clear();
    lcd.print("increment 29");
    white_back(sensorValues);
    motor1.stop();
    motor2.stop();
    delay(300);
  } else if (increment_count == 30) {
    lcd.clear();
    lcd.print("increment 30");
    encoder_refresh();
    encoder_back();
    increment_count += 1;
  } else if (increment_count == 31) {
    lcd.clear();
    lcd.print("increment 31");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(300);
      increment_count += 1;
    } else {
      go_straight_backward();
    }
  } else if (increment_count == 32) {
    if (metal == "METAL") {
      metal_box_position = "POSITION 3";
      increment(sensorValues);
    }
  } else if (increment_count == 33) {
    fUltrasonic_read();
    lcd.clear();
    lcd.print("increment 33");
    if ((fdistance < 20)) {
      motor1.stop();
      motor2.stop();
      String distance = String(fdistance);
      lcd.clear();
      lcd.print(distance);
      delay(1000);
      moveServo(arm, 40);
      delay(1000);
      pick_the_box();
      iterative_PID(100);
      line_follow_encoders(12);
      motor1.stop();
      motor2.stop();
      delay(1000);
      move_box_up();
      line_follow_encoders(5);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 34) {
    lcd.clear();
    lcd.print("increment 34");
    encoder_refresh();
    encoder_back();
    increment_count += 1;
  } else if (increment_count == 35) {
    lcd.clear();
    lcd.print("increment 35");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(300);
      if (metal_box_position == "POSITION 1") {
        increment_count += 1;
      } else if (metal_box_position == "POSITION 2") {
        increment_count += 2;
      } else if (metal_box_position == "POSITION 3") {
        increment_count += 6;
      }
    } else {
      go_straight_backward();
    }
  } else if (increment_count == 36) {
    lcd.clear();
    lcd.print("increment 36");
    increment(sensorValues);
    turnRight(sensorValues);
    increment_count += 5;
  } else if (increment_count == 37) {
    lcd.clear();
    lcd.print("increment 37");
    increment(sensorValues);
    turnRight(sensorValues);
    line_follow_encoders(5);
  } else if (increment_count == 38) {
    lcd.clear();
    lcd.print("increment 38");
    encoder_refresh();
    encoder_back();
    increment_count += 1;
  } else if (increment_count == 39) {
    lcd.clear();
    lcd.print("increment 39");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(300);
      increment_count += 1;
    } else {
      go_straight_backward();
    }
  } else if (increment_count == 40) {
    lcd.clear();
    lcd.print("increment 40");
    increment(sensorValues);
    turnRight(sensorValues);
    increment_count += 1;
  } else if (increment_count == 41) {
    lcd.clear();
    lcd.print("increment 41");
    increment(sensorValues);
    turnLeft(sensorValues);
  } else if (increment_count == 42) {
    lcd.clear();
    lcd.print("increment 42");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 1)) {
      motor1.stop();
      motor2.stop();
      delay(1000);
      fUltrasonic_read();
      line_follow_encoders(15);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 43) {
    lcd.clear();
    lcd.print("increment 43");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(1500);
      lcd.clear();
      lcd.print("");
      turnLeft(sensorValues);
      line_follow_encoders(7);
      increment_count += 1;
      lcd.clear();
      lcd.print("Turn OK");
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 44) {
    lcd.clear();
    lcd.print("increment 38");
    encoder_refresh();
    encoder_back();
    increment_count += 1;
  } else if (increment_count == 45) {
    lcd.clear();
    lcd.print("increment 39");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(300);
      increment_count += 1;
    } else {
      go_straight_backward();
    }
  } else if (increment_count == 46) {
    fUltrasonic_read();
    lcd.clear();
    lcd.print("increment 44");
    if ((fdistance < 28)) {
      motor1.stop();
      motor2.stop();
      String distance = String(fdistance);
      lcd.clear();
      lcd.print(distance);
      delay(1000);
      moveServo(arm, 40);
      delay(1000);
      line_follow_encoders(7);
      place_the_box_first();
      encoder_back_distance(3);
      place_the_box_second();
      motor1.stop();
      motor2.stop();
      delay(1000);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 47) {
    lcd.clear();
    lcd.print("increment 45");
    encoder_back_distance(10);
    arm_down();
    line_follow_encoders(17);
    encoder_back_distance(10);
    zip_mode();
    increment_count += 1;
  } else if (increment_count == 48) {
    lcd.clear();
    lcd.print("increment 38");
    encoder_refresh();
    encoder_back();
    increment_count += 1;
  } else if (increment_count == 49) {
    lcd.clear();
    lcd.print("increment 39");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(300);
      increment_count += 1;
    } else {
      go_straight_backward();
    }
  } else if (increment_count == 50) {
    lcd.clear();
    lcd.print("increment 37");
    increment(sensorValues);
    turnRight(sensorValues);
    line_follow_encoders(5);
  } else if (increment_count == 51) {
    fUltrasonic_read();
    lcd.clear();
    lcd.print("increment 28");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && Front_IR == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 1)) {
      motor1.stop();
      motor2.stop();
      lcd.clear();
      lcd.print("Front Line");
      delay(50000);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 1000) {
    moveServo(boxArm, 130);
    delay(2000);
    moveServo(boxArm, 90);
    delay(2000);
  } else if (increment_count == 2000) {
    calibrateSensors();
     motor1.stop();
      motor2.stop();
      delay(3000);
    increment_count += 1;
  } else if (increment_count == 2001) {
    if (digitalRead(switch_1_calib) == HIGH && digitalRead(switch_2_main) == LOW) {
    increment_count = -1;
  }
  }
}

void increment(int *sensorValues) {
  motor1.stop();
  motor2.stop();
  delay(500);
  white(sensorValues);
}

void line_follow(int *sensorValues) {
  if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0) {
    motor1.stop();
    motor2.stop();
    noLine(sensorValues);
  } else {

    float pidValue = calculatePID(sensorValues);
    PID_Linefollow(pidValue);
  }
}

void line_follow_circle(int *sensorValues) {
  if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0) {
    motor1.stop();
    motor2.stop();
    noLine(sensorValues);
  } else {

    float pidValue = calculatePID(sensorValues);
    PID_Linefollow(pidValue);
  }
}
void readSensors(int *values) {
  // Read analog sensors
  for (int i = 0; i < analogSensorCount; i++) {
    values[i] = analogRead(analogSensorPins[i]) > globel_threshold ? 0 : 1;  // Assuming higher values indicate no line
  }
}

float calculatePID(int *sensorValues) {
  float position = 0;
  int onLine = 0;

  // Loop through all sensors
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] == 1) {  // Assuming 1 indicates line detected
      position += IR_weight[(i)];
      onLine++;
    }
  }

  // If no line is detected by any sensor, use the last known error value
  if (onLine == 0) {
    // If previous error is not available, assume the line is straight ahead
    error = -previousError;
  } else {
    // Calculate the average position of the line
    position /= onLine;
    // Calculate error based on sensor position
    error = 0 - position;
  }
  for (int i = 49; i > 0; i--) {
    errorArray[i] = errorArray[i - 1];
  }
  errorArray[0] = error;
  // PID terms
  P = error;
  I += error;
  D = error - previousError;
  previousError = error;

  // Calculate PID value
  float pidValue = (Kp * P) + (Ki * I) + (Kd * D);


  return pidValue;
}

void PID_Linefollow(float pidValue) {
  lsp = lfspeed - pidValue;
  rsp = lfspeed + pidValue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < -255) {
    lsp = -255;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < -255) {
    rsp = -255;
  }
  motor_drive(lsp, rsp);
}

void motor_drive(float left, float right) {
  int absRight = abs(right);  // Absolute value for right speed
  int absLeft = abs(left);    // Absolute value for left speed

  if (right > 0) {
    motor1.setSpeed(absRight);
    motor1.forward();
  } else {
    motor1.setSpeed(absRight);
    motor1.backward();
  }

  if (left > 0) {
    motor2.setSpeed(absLeft);
    motor2.forward();
  } else {
    motor2.setSpeed(absLeft);
    motor2.backward();
  }
}

bool isImmediateTurn(int *sensorValues) {
  bool turnLeft = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1;
  bool turnRight = sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  return (turnLeft || turnRight);
}
void turn(int *sensorValues) {
  // Check if A0, A1, A2, A3 are detecting the line (indicating a left turn)
  if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) {
    turnLeft(sensorValues);

  }
  // Check if A4, A5, A6, A7 are detecting the line (indicating a right turn)
  else if (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) {
    turnRight(sensorValues);
  } else {
    // If no sharp turn is detected, stop the motors
    motor1.stop();
    motor2.stop();
  }
}

void iterative_PID(int number) {
  for (int i = 0; i < number; i++) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    float pidValue = calculatePID(sensorValues);
    PID_Linefollow(pidValue);
  }
}

void line_follow_encoders(int distance) {
  lcd.clear();
  lcd.print("Encoders");
  encoderACount = 0;
  encoderBCount = 0;
  int dis_rotation = 21.0486;
  int encoder_count = distance * (374 / dis_rotation);
  while (encoderACount < encoder_count || encoderBCount < encoder_count) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    float pidValue = calculatePID(sensorValues);
    PID_Linefollow(pidValue);
  }
  motor1.stop();
  motor2.stop();
  delay(2000);
  lcd.clear();
}

void encoder_back_distance(int distance) {
  lcd.clear();
  lcd.print("Encoders");
  encoderACount = 0;
  encoderBCount = 0;
  int dis_rotation = 21.0486;
  int encoder_count = distance * (374 / dis_rotation);
  while (encoderACount < encoder_count || encoderBCount < encoder_count) {
    go_straight_backward();
  }
  motor1.stop();
  motor2.stop();
  delay(2000);
  lcd.clear();
}

void turnLeft(int *sensorValues) {
  // Perform a sharp left turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool enoughturn = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0 && Front_IR == 0;
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  bool forward = sensorValues[0] == 1 && sensorValues[1] == 1;
  int Front_IR_Analog = analogRead(Front_IR_Pin);
  if (Front_IR_Analog < 100) {
    Front_IR = 0;
  } else {
    Front_IR = 1;
  }
  if (Front_IR == 1) {
    lcd.clear();
    lcd.print("Front IR");
    while (forward) {
      sensorValues[SensorCount];
      readSensors(sensorValues);
      forward = sensorValues[0] == 1 && sensorValues[1] == 1;
      if (!forward) {
        break;
      }
      line_follow(sensorValues);
    }
  } else {
    lcd.clear();
    lcd.print("NO IR");
    while (forward) {
      sensorValues[SensorCount];
      readSensors(sensorValues);
      forward = sensorValues[0] == 1 && sensorValues[1] == 1;
      if (!forward) {
        break;
      }
      motor1.setSpeed(130);  // Set right motor speed
      motor1.forward();      // Move right motor forward
      motor2.setSpeed(130);  // Set left motor speed
      motor2.forward();      // Move left motor backward for a sharper turn
    }
  }
  motor1.stop();
  motor2.stop();
  delay(200);
  if (sensorValues[3] == 1 && sensorValues[4] == 1 && Front_IR == 1) {
    lcd.clear();
    lcd.print("PID");
    iterative_PID(300);
    lcd.clear();
  } else {
    lcd.clear();
    lcd.print("NO PID");
    motor1.setSpeed(130);  // Set right motor speed
    motor1.forward();      // Move right motor forward
    motor2.setSpeed(130);  // Set left motor speed
    motor2.forward();      // Move left motor backward for a sharper turn
    delay(250);
  }
  motor1.stop();
  motor2.stop();
  delay(200);
  sensorValues[SensorCount];
  readSensors(sensorValues);
  encoderACount = 0;
  encoderBCount = 0;
  lfspeed = turns_speed;
  while (!enoughturn) {
    lcd.clear();
    lcd.print("TURN");
    int Front_IR_Analog = analogRead(Front_IR_Pin);
    if (Front_IR_Analog < 100) {
      Front_IR = 0;
    } else {
      Front_IR = 1;
    }
    sensorValues[SensorCount];
    readSensors(sensorValues);
    bool enoughturn = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0 && Front_IR == 0;
    whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (enoughturn) {
      break;
    }
    left_encoder();
  }
  lfspeed = speed;
  iterative_PID(500);
  motor1.stop();
  motor2.stop();
}

void turnRight(int *sensorValues) {
  // Perform a sharp right turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 0;
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  bool forward = sensorValues[6] == 1 && sensorValues[6] == 1;
  int Front_IR_Analog = analogRead(Front_IR_Pin);
  if (Front_IR_Analog < 100) {
    Front_IR = 0;
  } else {
    Front_IR = 1;
  }
  if (Front_IR == 1) {
    while (forward) {
      sensorValues[SensorCount];
      readSensors(sensorValues);
      forward = sensorValues[6] == 1 && sensorValues[6] == 1;
      if (!forward) {
        break;
      }
      line_follow(sensorValues);
    }
  } else {
    while (forward) {
      sensorValues[SensorCount];
      readSensors(sensorValues);
      forward = sensorValues[6] == 1 && sensorValues[6] == 1;
      if (!forward) {
        break;
      }
      motor1.setSpeed(130);  // Set right motor speed
      motor1.forward();      // Move right motor forward
      motor2.setSpeed(130);  // Set left motor speed
      motor2.forward();      // Move left motor backward for a sharper turn
    }
  }
  motor1.stop();
  motor2.stop();
  delay(200);
  if (sensorValues[3] == 1 && sensorValues[4] == 1 && Front_IR == 1) {
    iterative_PID(300);
  } else {
    motor1.setSpeed(130);  // Set right motor speed
    motor1.forward();      // Move right motor forward
    motor2.setSpeed(130);  // Set left motor speed
    motor2.forward();      // Move left motor backward for a sharper turn
    delay(250);
  }
  motor1.stop();
  motor2.stop();
  delay(200);
  encoderACount = 0;
  encoderBCount = 0;
  lfspeed = turns_speed;
  while (!enoughturn) {
    int Front_IR_Analog = analogRead(Front_IR_Pin);
    if (Front_IR_Analog < 100) {
      Front_IR = 0;
    } else {
      Front_IR = 1;
    }
    sensorValues[SensorCount];
    readSensors(sensorValues);
    enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 0;
    if (enoughturn) {
      break;
    }
    right_encoder();
  }
  lfspeed = speed;
  iterative_PID(500);
  motor1.stop();
  motor2.stop();
}

void turnLeft_t(int *sensorValues) {
  // Perform a sharp left turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool enoughturn = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0 && Front_IR == 0;
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  bool forward = sensorValues[0] == 1 && sensorValues[1] == 1;
  int Front_IR_Analog = analogRead(Front_IR_Pin);
  if (Front_IR_Analog < 100) {
    Front_IR = 0;
  } else {
    Front_IR = 1;
  }
  motor1.stop();
  motor2.stop();
  delay(200);
  // if (sensorValues[3] == 1 && sensorValues[4] == 1 && Front_IR == 1) {
  //   lcd.clear();
  //   lcd.print("PID");
  //   iterative_PID(300);
  //   lcd.clear();
  // } else {
  //   lcd.clear();
  //   lcd.print("NO PID");
  //   motor1.setSpeed(130);  // Set right motor speed
  //   motor1.forward();      // Move right motor forward
  //   motor2.setSpeed(130);  // Set left motor speed
  //   motor2.forward();      // Move left motor backward for a sharper turn
  //   delay(250);
  // }
  // motor1.stop();
  // motor2.stop();
  // delay(200);
  // sensorValues[SensorCount];
  readSensors(sensorValues);
  encoderACount = 0;
  encoderBCount = 0;
  lfspeed = turns_speed;
  left_encoder_ninty();
  lfspeed = speed;
  iterative_PID(500);
  motor1.stop();
  motor2.stop();
}

void turnRight_t(int *sensorValues) {
  // Perform a sharp right turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 0;
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  bool forward = sensorValues[6] == 1 && sensorValues[6] == 1;
  motor1.stop();
  motor2.stop();
  delay(200);
  // if (sensorValues[3] == 1 && sensorValues[4] == 1 && Front_IR == 1) {
  //   iterative_PID(300);
  // } else {
  //   motor1.setSpeed(130);  // Set right motor speed
  //   motor1.forward();      // Move right motor forward
  //   motor2.setSpeed(130);  // Set left motor speed
  //   motor2.forward();      // Move left motor backward for a sharper turn
  //   delay(250);
  // }
  // motor1.stop();
  // motor2.stop();
  // delay(200);
  // encoderACount = 0;
  encoderBCount = 0;
  lfspeed = turns_speed;
  right_encoder_ninty();
  lfspeed = speed;
  iterative_PID(500);
  motor1.stop();
  motor2.stop();
}

void noLine(int *sensorValues) {
  float firstNonZeroValue;
  for (int i = 0; i < 50; i++) {
    if (errorArray[i] != 0) {
      firstNonZeroValue = errorArray[i];
      break;  //
    }
  }

  if (firstNonZeroValue < 0) {
    turnRight(sensorValues);

  } else if (firstNonZeroValue > 0) {
    turnLeft(sensorValues);
  }
}

void white(int *sensorValues) {
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  encoder_refresh();
  lfspeed = 185;
  while (whitedetect) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (!whitedetect) {
      break;
    }
    go_straight_forward();
  }
  motor1.stop();
  motor2.stop();
  delay(200);
  lfspeed = speed;
  increment_count += 1;
}

void white_back(int *sensorValues) {
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  while (whitedetect) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (!whitedetect) {
      break;
    }
    motor1.setSpeed(180);  // Set right motor speed
    motor1.backward();     // Move right motor forward
    motor2.setSpeed(180);  // Set left motor speed
    motor2.backward();     // Move left motor backward for a sharper turn
  }
  motor1.stop();
  motor2.stop();
  delay(200);
  increment_count += 1;
}

void t_forward(int *sensorValues) {
  bool whitedetect = sensorValues[0] == 1 && sensorValues[7] == 1;
  while (whitedetect) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    whitedetect = sensorValues[0] == 1 && sensorValues[7] == 1;
    if (!whitedetect) {
      break;
    }
    line_follow(sensorValues);
  }
  motor1.stop();
  motor2.stop();
  delay(3000);
}

void enough_forward(int *sensorValues) {
  bool forward = (sensorValues[0] == 1 && sensorValues[71] == 1) || (sensorValues[6] == 1 && sensorValues[7] == 1);
  while (forward) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    forward = (sensorValues[0] == 1 && sensorValues[71] == 1) || (sensorValues[6] == 1 && sensorValues[7] == 1);
    if (!forward) {
      break;
    }
    motor1.setSpeed(100);  // Set right motor speed
    motor1.forward();      // Move right motor forward
    motor2.setSpeed(100);  // Set left motor speed
    motor2.forward();      // Move left motor backward for a sharper turn
  }
  motor1.stop();
  motor2.stop();
  delay(200);
}

void enough_back(int *sensorValues) {
  bool backLeft = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1;
  bool backRight = sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  while (!backLeft && !backRight) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    backLeft = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1;
    backRight = sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (backLeft || backRight) {
      break;
    }
    motor1.setSpeed(100);  // Set right motor speed
    motor1.backward();     // Move right motor forward
    motor2.setSpeed(100);  // Set left motor speed
    motor2.backward();     // Move left motor backward for a sharper turn
  }
  motor1.stop();
  motor2.stop();
  delay(200);
}

void colour_detection() {
  motor1.stop();
  motor2.stop();
  delay(1000);
  bool forward = sensorValues[0] == 1 && sensorValues[1] == 1;
  while (forward) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    forward = sensorValues[0] == 1 && sensorValues[1] == 1;
    if (!forward) {
      break;
    }
    motor1.setSpeed(150);  // Set right motor speed
    motor1.forward();      // Move right motor forward
    motor2.setSpeed(150);  // Set left motor speed
    motor2.forward();      // Move left motor backward for a sharper turn
  }
  motor1.stop();
  motor2.stop();
  delay(200);
  fUltrasonic_read();
  while (fdistance > 24) {
    fUltrasonic_read();
    sensorValues[SensorCount];
    readSensors(sensorValues);
    line_follow(sensorValues);
  }
  motor1.stop();
  motor2.stop();
  delay(1000);
  digitalWrite(send_shape_request, LOW);
  digitalWrite(send_colour_request, HIGH);  // Corrected to send_shape_request

  // Wait for a response
  delay(7000);  // Delay to allow time for response

  // Check for response
  if (digitalRead(recieved_led_1) == HIGH) {  // Corrected syntax
    digitalWrite(led_green, HIGH);
    digitalWrite(led_blue, LOW);
    wall_colour = "GREEN";
    // Ensure other LED is turned off
  } else if (digitalRead(recieved_led_2) == HIGH) {  // Corrected syntax
    digitalWrite(led_blue, HIGH);
    digitalWrite(led_green, LOW);  // E
    wall_colour = "BLUE";
  } else {
    digitalWrite(led_green, HIGH);
    digitalWrite(led_blue, LOW);
    wall_colour = "GREEN";
  }
  delay(1000);
}

void moveServo(Servo &servo, int targetPosition) {
  int currentPosition = servo.read();
  if (currentPosition < targetPosition) {
    for (int pos = currentPosition; pos <= targetPosition; pos += 1) {
      servo.write(pos);
      delay(10);  // adjust delay for desired speed
    }
  } else {
    for (int pos = currentPosition; pos >= targetPosition; pos -= 1) {
      servo.write(pos);
      delay(10);  // adjust delay for desired speed
    }
  }
}

void moveServo_slow(Servo &servo, int targetPosition) {
  int currentPosition = servo.read();
  if (currentPosition < targetPosition) {
    for (int pos = currentPosition; pos <= targetPosition; pos += 1) {
      servo.write(pos);
      delay(20);  // adjust delay for desired speed
    }
  } else {
    for (int pos = currentPosition; pos >= targetPosition; pos -= 1) {
      servo.write(pos);
      delay(20);  // adjust delay for desired speed
    }
  }
}

void pick_the_box() {
  moveServo(rotateArm, 94);
  moveServo(boxArm, 90);
  moveServo(upArm, 20);
}

void move_box_up() {
  moveServo(boxArm, 140);
  delay(500);
  moveServo_slow(upArm, 140);
  moveServo(arm, 180);
}

void place_the_box() {
  moveServo(arm, 40);
  moveServo(rotateArm, 94);
  moveServo(upArm, 20);
  moveServo(boxArm, 90);
  delay(2000);
  moveServo(upArm, 140);
  delay(2000);
  moveServo(boxArm, 120);
}

void place_the_box_first() {
  moveServo(arm, 40);
  moveServo(rotateArm, 94);
  moveServo(upArm, 20);
  moveServo(boxArm, 90);
  delay(2000);
}

void place_the_box_second() {
  moveServo(upArm, 140);
  moveServo(boxArm, 120);
  delay(2000);
}

void zip_mode() {
  moveServo(upArm, 135);
  moveServo(boxArm, 130);
  moveServo(rotateArm, 94);
  moveServo(arm, 165);
}

void initial() {
  moveServo(rotateArm, 90);
  moveServo(upArm, 90);
  moveServo(boxArm, 90);
}

void arm_down() {
  moveServo(rotateArm, 94);
  moveServo(boxArm, 180);
  moveServo(upArm, 20);
}

long fUltrasonic_read() {
  digitalWrite(ftrig, LOW);
  delayMicroseconds(2);
  digitalWrite(ftrig, HIGH);
  delayMicroseconds(10);
  long time = pulseIn(fecho, HIGH);
  fdistance = time / 29 / 2;
  return fdistance;
}

// Function to set up the sensors
void setupSensors() {
  // Reset both sensors
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);

  delay(10);
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  digitalWrite(SHT_LOX4, HIGH);

  delay(10);

  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);


  // Initialize sensor 1
  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println("Failed to initialize sensor 1");
    tof = false;
  }
  delay(10);

  digitalWrite(SHT_LOX2, HIGH);

  // Initialize sensor 2
  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println("Failed to initialize sensor 2");
    tof = false;
  }
  delay(10);

  digitalWrite(SHT_LOX3, HIGH);

  //Initialize sensor 3
  if (!lox3.begin(LOX3_ADDRESS)) {
    Serial.println("Failed to initialize sensor 3");
    tof = false;
  }
  delay(10);

  digitalWrite(SHT_LOX4, HIGH);
  //Initialize sensor 4
  if (!lox4.begin(LOX4_ADDRESS)) {
    Serial.println("Failed to initialize sensor 4");
    tof = false;
  }
  delay(10);
}

// Function to read sensor measurements
void readSensors() {
  // Perform ranging tests on both sensors
  lox1.rangingTest(&measure1, false);
  lox2.rangingTest(&measure2, false);
  lox3.rangingTest(&measure3, false);
  lox4.rangingTest(&measure4, false);


  if (measure1.RangeStatus != 4) {  // Check if measurement is valid
    sensor1 = measure1.RangeMilliMeter;
  }

  if (measure2.RangeStatus != 4) {  // Check if measurement is valid
    sensor2 = measure2.RangeMilliMeter;
  }

  if (measure3.RangeStatus != 4) {  // Check if measurement is valid
    sensor3 = measure3.RangeMilliMeter;
  }

  if (measure4.RangeStatus != 4) {  // Check if measurement is valid
    sensor4 = measure4.RangeMilliMeter;
  }
}

int detectColor() {
  int greenPW;
  greenPW = getGreenPW();
  int greenValue = map(greenPW, greenMin, greenMax, 255, 0);
  delay(200);

  int bluePW;
  bluePW = getBluePW();
  int blueValue = map(bluePW, blueMin, blueMax, 255, 0);
  delay(200);

  // Determine the detected color based on the values of green and blue
  if (greenValue > blueValue) {
    return 0;  // Green
  } else {
    return 1;  // Blue
  }
}
//Function to read Green Pulse Widths
int getGreenPW() {

  // Set sensor to read Green only
  digitalWrite(S3, HIGH);
  digitalWrite(S2, HIGH);
  // Define integer to represent Pulse Width
  int PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut, LOW);
  // Return the value
  return PW;
}

// Function to read Blue Pulse Widths
int getBluePW() {

  // Set sensor to read Blue only
  digitalWrite(S3, HIGH);
  digitalWrite(S2, LOW);
  // Define integer to represent Pulse Width
  int PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut, LOW);
  // Return the value
  return PW;
}

void metal_detector() {
  int sum = 0;
  for (int i = 0; i < 10; i++) {
    if (digitalRead(metal_detector_pin) == HIGH) {
      Serial.println(sum);
      sum++;
    } else {
      Serial.println(sum);
    }
  }
  int average = sum / 10;
  if (average > 0.5) {
    metal = "METAL";
    lcd.clear();
    lcd.print("METAL");
  } else {
    metal = "NON METAL";
    lcd.clear();
    lcd.print("NON METAL");
  }
}

// Interrupt service routine for encoder A
void encoderAHandler() {
  encoderACount++;
}

// Interrupt service routine for encoder B
void encoderBHandler() {
  encoderBCount++;
}

void go_straight_forward() {
  // Simple strategy to move straight: Adjust speeds to match encoder counts
  long diff = encoderACount - encoderBCount;

  int adjustment = diff * 2;  // Adjust based on the difference

  int speedA = lfspeed - adjustment;
  int speedB = lfspeed + adjustment;

  int absRight = abs(speedA);  // Absolute value for right speed
  int absLeft = abs(speedB);   // Absolute value for left speed

  if (speedA > 0) {
    motor1.setSpeed(absRight);
    motor1.forward();
  } else {
    motor1.setSpeed(absRight);
    motor1.backward();
  }

  if (speedB > 0) {
    motor2.setSpeed(absLeft);
    motor2.forward();
  } else {
    motor2.setSpeed(absLeft);
    motor2.backward();
  }
}

void go_straight_backward() {
  // Simple strategy to move straight: Adjust speeds to match encoder counts
  long diff = encoderACount - encoderBCount;

  int adjustment = diff * 2;  // Adjust based on the difference

  int speedA = lfspeed - adjustment;
  int speedB = lfspeed + adjustment;

  int absRight = abs(speedA);  // Absolute value for right speed
  int absLeft = abs(speedB);   // Absolute value for left speed

  if (speedA > 0) {
    motor1.setSpeed(absRight);
    motor1.backward();
  } else {
    motor1.setSpeed(absRight);
    motor1.forward();
  }

  if (speedB > 0) {
    motor2.setSpeed(absLeft);
    motor2.backward();
  } else {
    motor2.setSpeed(absLeft);
    motor2.forward();
  }

  // For debugging, print the speeds
  Serial.print("Motor A Speed: ");
  Serial.print(speedA);
  Serial.print(", Motor B Speed: ");
  Serial.println(speedB);
}

void left_encoder() {
  // Simple strategy to move straight: Adjust speeds to match encoder counts
  long diff = encoderACount - encoderBCount;

  int adjustment = diff * 2;  // Adjust based on the difference

  int speedA = lfspeed - adjustment;
  int speedB = lfspeed + adjustment;

  int absRight = abs(speedA);  // Absolute value for right speed
  int absLeft = abs(speedB);   // Absolute value for left speed

  if (speedA > 0) {
    motor1.setSpeed(absRight);
    motor1.forward();
  } else {
    motor1.setSpeed(absRight);
    motor1.backward();
  }

  if (speedB > 0) {
    motor2.setSpeed(absLeft);
    motor2.backward();
  } else {
    motor2.setSpeed(absLeft);
    motor2.forward();
  }
}

void right_encoder() {
  // Simple strategy to move straight: Adjust speeds to match encoder counts
  long diff = encoderACount - encoderBCount;

  int adjustment = diff * 2;  // Adjust based on the difference

  int speedA = lfspeed - adjustment;
  int speedB = lfspeed + adjustment;

  int absRight = abs(speedA);  // Absolute value for right speed
  int absLeft = abs(speedB);   // Absolute value for left speed

  if (speedA > 0) {
    motor1.setSpeed(absRight);
    motor1.backward();
  } else {
    motor1.setSpeed(absRight);
    motor1.forward();
  }

  if (speedB > 0) {
    motor2.setSpeed(absLeft);
    motor2.forward();
  } else {
    motor2.setSpeed(absLeft);
    motor2.backward();
  }
}

void left_encoder_ninty() {
  encoderACount = 0;
  encoderBCount = 0;
  while (encoderACount < 281 || encoderBCount < 281) {
    left_encoder();
  }
  motor1.stop();
  motor2.stop();
  delay(2000);
}
void right_encoder_ninty() {
  encoderACount = 0;
  encoderBCount = 0;
  while (encoderACount < 281 || encoderBCount < 281) {
    right_encoder();
  }
  motor1.stop();
  motor2.stop();
  delay(2000);
}

void encoder_refresh() {
  motor1.stop();
  motor2.stop();
  delay(300);
  encoderACount = 0;
  encoderBCount = 0;
}

void encoder_back() {
  motor1.setSpeed(120);
  motor1.backward();
  motor2.setSpeed(120);
  motor2.backward();
  delay(100);
}

void encoder_forward() {
  motor1.setSpeed(120);
  motor1.forward();
  motor2.setSpeed(120);
  motor2.forward();
  delay(100);
}

void ultrasonic_detection() {
  motor1.stop();
  motor2.stop();
  String distance = String(fdistance);
  lcd.clear();
  lcd.print(distance);
  delay(5000);
  moveServo(arm, 75);
  delay(2000);
  metal_detector();
  delay(1000);
  moveServo(arm, 165);
}

void shape_detector() {
  digitalWrite(send_shape_request, HIGH);
  digitalWrite(send_colour_request, LOW);  // Corrected to send_shape_request
  digitalWrite(send_shape_request, HIGH);
  digitalWrite(send_colour_request, LOW);  // Corrected to send_shape_request

  // Wait for a response
  delay(10000);  // Delay to allow time for response

  // int sum_shape;
  // for (int i = 0; i < 25; i++) {
  //   if (digitalRead(recieved_led_1) == HIGH) {
  //     sum_shape++;
  //   }
  // }

  // Check for response
  if (digitalRead(recieved_led_1) == HIGH) {  // Corrected syntax
    digitalWrite(led_blue, HIGH);
    digitalWrite(led_green, LOW);  // E
  } else {                         // Corrected syntax
    digitalWrite(led_green, HIGH);
    digitalWrite(led_blue, LOW);
    // Ensure other LED is turned off
  }
}

void bubbleSort(int arr[], int size) {
  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        // Swap elements if they are in the wrong order
        int temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

int get_threshold_index(int arr[], int size) {
  int maxDiff = 0;
  int break_index = 0;

  for (int i = 0; i < size - 1; i++) {
    int currentDiff = arr[i + 1] - arr[i];
    if (currentDiff > maxDiff) {
      maxDiff = currentDiff;
      break_index = i;
    }
  }

  break_index;
}

void calibrateSensors() {
  // Read sensor values and store them in the sensor value array
  int index = 0;
  for (int j = 0; j < NUM_READINGS; j++) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorValueArray[index] = analogRead(A0 + i);
      go_straight_forward();
      index++;
    }
  }
  // Sort the sensor value array
  //sort(sensorValueArray, sensorValueArray + (NUM_SENSORS * NUM_READINGS));
  bubbleSort(sensorValueArray, NUM_SENSORS * NUM_READINGS);

  int thresholdIndex = get_threshold_index(sensorValueArray, NUM_SENSORS * NUM_READINGS);
  int threshold = sensorValueArray[thresholdIndex];
  globel_threshold = threshold + 100;

  lcd.clear();
  String threshold_str = String(globel_threshold);
    lcd.print(threshold_str);

  // Print calibrated threshold
  Serial.print("Calibrated Threshold: ");
  Serial.println(threshold);
}

