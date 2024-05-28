#include <L298N.h>
#include <Arduino.h>
#include "Adafruit_VL53L0X.h"
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <EEPROM.h>

int address = 0;

int shooting_total;
int max_height;
int shooting_angle;

//////////////
const int ena_shoot = 5;   // PWM pin for motor A speed control
const int in1_shoot = 39;  // Motor A input 1
const int in2_shoot = 37;  // Motor A input 2

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

int sensorValueArray[NUM_READINGS * NUM_SENSORS];  // Array to store sensor readings

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

struct Segment {
  int start;
  int end;
  int length() const {
    return end - start + 1;
  }
} segments[181 / 2];  // Assuming a maximum possible number of segments

int segmentCount = 0;

// Define switch pin numbers
int switch_1_calib = 41;     // Auto calibration
int switch_2_main_slow = 4;  // Main code
int switch_3_increment = 50;
int switch_4_main_medium = 44;
int switch_5_main_fast = 48;

LiquidCrystal_I2C lcd(0x27, 16, 2);

String wall_colour = "GREEN";
String path_colour;
String metal;
String metal_box_position;

//////ultrasonic pins
#define ftrig 53
#define fecho 51
long fdistance;
int distance_to_the_hole;

Servo boxArm;
Servo upArm;
Servo rotateArm;
Servo arm;
Servo radar;
Servo shooter;

// Assign pin numbers to each servo
const int boxArmPin = 36;  //black
const int upArmPin = 38;
const int rotateArmPin = 40;  //red
const int armPin = 45;
const int radarPin = 52;
const int shooterPin = 6;
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
int speed = 155;
int turns_speed = 190;
int lfspeed = speed;
int Front_IR;
int globel_threshold = 210;

// Function prototypes
void readSensors(int *values);
float calculatePID(int *sensorValues);
void motor_drive(int left, int right);
void turnLeft(int *sensorValues);
void turnRight(int *sensorValues);
void noLine();
void trash_yard_object();

#define recieved_led_1 23
#define recieved_led_2 25

#define send_colour_request 27
#define send_shape_request 29

#define led_blue 31
#define led_green 33

#define led_blue_trashyard 30
#define led_green_trashyard 32
#define led_red_trashyard 34

void setup() {
  Serial.begin(9600);
  digitalWrite(send_colour_request, HIGH);
  delay(500);
  // Corrected to send_shape_request

  digitalWrite(send_colour_request, LOW);
  digitalWrite(send_shape_request, LOW);

  pinMode(ena_shoot, OUTPUT);
  pinMode(in1_shoot, OUTPUT);
  pinMode(in2_shoot, OUTPUT);

  pinMode(switch_1_calib, INPUT_PULLUP);
  pinMode(switch_2_main_slow, INPUT_PULLUP);
  pinMode(switch_3_increment, INPUT_PULLUP);
  pinMode(switch_4_main_medium, INPUT_PULLUP);
  pinMode(switch_5_main_fast, INPUT_PULLUP);

  if (digitalRead(switch_1_calib) == LOW) {
    increment_count = 2000;
  } else if (digitalRead(switch_2_main_slow) == LOW) {
    increment_count = -2;
    lcd.clear();
    lcd.print("MAIN");
  } else if (digitalRead(switch_4_main_medium) == LOW) {
    increment_count = -3;
    lcd.clear();
    lcd.print("MAIN");
  } else if (digitalRead(switch_5_main_fast) == LOW) {
    increment_count = -4;
    lcd.clear();
    lcd.print("MAIN");
  }


  if (digitalRead(switch_3_increment) == LOW) {
    max_height = 15;
  } else {
    max_height = 10;
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
  radar.attach(radarPin);
  shooter.attach(shooterPin);

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

  delay(100);
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

  pinMode(led_blue_trashyard, OUTPUT);
  pinMode(led_green_trashyard, OUTPUT);
  pinMode(led_red_trashyard, OUTPUT);

  digitalWrite(led_blue_trashyard, HIGH);
  digitalWrite(led_green_trashyard, HIGH);
  digitalWrite(led_red_trashyard, HIGH);

  digitalWrite(send_colour_request, LOW);
  digitalWrite(send_shape_request, LOW);

  Wire.begin();  // Initialize I2C communications

  // Initialize all ToF sensors
  for (int i = 0; i < 4; i++) {
    selectMuxChannel(i);
    if (!lox.begin()) {
      Serial.println(F("Failed to boot VL53L0X"));
    } else {
      Serial.print(F("VL53L0X "));
      Serial.print(i);
      Serial.println(F(" initialized"));
    }
  }
  sensorValues[SensorCount];
  readSensors(sensorValues);

  // Start with motors stopped
  motor1.stop();
  motor2.stop();
  delay(1000);
}

void loop() {
  int Front_IR_Analog = analogRead(Front_IR_Pin);
  if (Front_IR_Analog < 200) {
    Front_IR = 0;
  } else {
    Front_IR = 1;
  }
  fUltrasonic_read();
  sensorValues[SensorCount];
  readSensors(sensorValues);
  if (increment_count == -4) {
    speed = 180;
    turns_speed = 210;
    lfspeed = speed;
    increment_count = -1;
  } else if (increment_count == -3) {
    speed = 165;
    turns_speed = 200;
    lfspeed = speed;
    increment_count = -1;
  } else if (increment_count == -2) {
    speed = 150;
    turns_speed = 190;
    lfspeed = speed;
    increment_count = -1;
  } else if (increment_count == -1) {
    while (true) {
      digitalWrite(send_shape_request, LOW);
      digitalWrite(send_colour_request, HIGH);
      if ((digitalRead(recieved_led_1) == HIGH) || (digitalRead(recieved_led_2) == HIGH)) {
        break;
      }
    }
    digitalWrite(send_shape_request, LOW);
    digitalWrite(send_colour_request, HIGH);  // Corrected to send_shape_request
    if (readFromEEPROM(0) == 0) {
      globel_threshold = 210;
    } else {
      globel_threshold = readFromEEPROM(0);
    }
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
      encoder_refresh();
      increment_count += 2;
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
      turnLeft(sensorValues);
      increment_count += 1;
    } else {
      go_straight_backward();
    }
  } else if (increment_count == 4) {
    lcd.clear();
    lcd.print("increment 4");
    if (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 0) {
      turnRight(sensorValues);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 5) {
    lcd.clear();
    lcd.print("increment 5");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      turnLeft(sensorValues);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 6) {
    lcd.clear();
    lcd.print("increment 6");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 1)) {
      motor1.stop();
      motor2.stop();
      delay(1500);
      if (detectColor() == 1) {
        path_colour = "BLUE";
        lcd.clear();
        lcd.print("BLUE");
      } else {
        path_colour = "GREEN";
        lcd.clear();
        lcd.print("GREEN");
      }
      delay(100);
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
      delay(1000);
      shape_detector();
      encoder_forward_distance(8);
      encoder_hundredeighty();
      line_follow_encoders(7);
      increment_count += 2;
    } else {
      line_follow_circle(sensorValues);
    }
  } else if (increment_count == 9) {
    lcd.clear();
    lcd.print("increment 9");
    if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1) {
      turnLeft(sensorValues);
      increment_count += 1;
    } else {
      line_follow_circle(sensorValues);
    }
  } else if (increment_count == 10) {
    lcd.clear();
    lcd.print("increment 10");
    if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && Front_IR == 0) {
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
      delay(100);
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
      delay(1000);
      shape_detector();
      encoder_forward_distance(8);
      encoder_hundredeighty();
      line_follow_encoders(7);
      increment_count += 2;
    } else {
      line_follow_circle(sensorValues);
    }
  } else if (increment_count == 14) {
    lcd.clear();
    lcd.print("increment 14");
    if (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1) {
      turnLeft(sensorValues);
      increment_count += 1;
    } else {
      line_follow_circle(sensorValues);
    }
  } else if (increment_count == 15) {
    lcd.clear();
    lcd.print("increment 15");
    if (sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 0) {
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
      delay(100);
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
      lcd.clear();
      lcd.print("Before Turn");
      turnRight(sensorValues);
      line_follow_encoders(3);
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
      delay(100);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 19) {
    lcd.clear();
    lcd.print("increment 19");
    encoder_refresh();
    white_back(sensorValues);
    motor1.stop();
    motor2.stop();
    delay(50);
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
      delay(100);
      increment_count += 1;
    } else {
      go_straight_backward();
    }
  } else if (increment_count == 22) {
    if (metal == "METAL") {
      metal_box_position = "POSITION 1";
      increment_count += 11;
    } else {
      iterative_PID(300);
      turnLeft(sensorValues);
      line_follow_encoders(3);
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
      delay(100);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 24) {
    lcd.clear();
    lcd.print("increment 24");
    encoder_refresh();
    white_back(sensorValues);
    motor1.stop();
    motor2.stop();
    delay(50);
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
      delay(100);
      increment_count += 1;
    } else {
      go_straight_backward();
    }
  } else if (increment_count == 27) {
    if (metal == "METAL") {
      metal_box_position = "POSITION 2";
      increment_count += 6;
    } else {
      iterative_PID(300);
      turnLeft(sensorValues);
      line_follow_encoders(3);
      metal_box_position = "POSITION 3";
      increment_count += 2;
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
      delay(100);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 29) {
    lcd.clear();
    lcd.print("increment 29");
    encoder_refresh();
    white_back(sensorValues);
    motor1.stop();
    motor2.stop();
    delay(50);
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
      delay(100);
      increment_count += 1;
    } else {
      go_straight_backward();
    }
  } else if (increment_count == 32) {
    metal_box_position = "POSITION 3";
    increment_count += 1;
  } else if (increment_count == 33) {
    lcd.clear();
    lcd.print("increment 33");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      encoder_refresh();
      white_back(sensorValues);
      motor1.stop();
      motor2.stop();
      delay(50);
      encoder_refresh();
      encoder_back_distance(10);
    } else {
      encoder_refresh();
      encoder_back_distance(10);
    }

  } else if (increment_count == 34) {
    lcd.clear();
    lcd.print("increment 31");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(50);
      increment(sensorValues);
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 35) {
    fUltrasonic_read();
    lcd.clear();
    lcd.print("increment 33");
    if ((fdistance < 20)) {
      motor1.stop();
      motor2.stop();
      String distance = String(fdistance);
      lcd.clear();
      lcd.print(distance);
      delay(50);
      moveServo(arm, 30);
      pick_the_box();
      iterative_PID(200);
      line_follow_encoders(12);
      motor1.stop();
      motor2.stop();
      delay(100);
      move_box_up();
      line_follow_encoders(5);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 36) {
    lcd.clear();
    lcd.print("increment 34");
    encoder_refresh();
    encoder_back();
    increment_count += 1;
  } else if (increment_count == 37) {
    lcd.clear();
    lcd.print("increment 37");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(50);
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
  } else if (increment_count == 38) {
    lcd.clear();
    lcd.print("increment 38");
    increment(sensorValues);
    turnRight(sensorValues);
    increment_count += 5;
  } else if (increment_count == 39) {
    lcd.clear();
    lcd.print("increment 39");
    increment(sensorValues);
    encoder_hundredeighty();
    line_follow_encoders(5);
    increment_count += 4;
  } else if (increment_count == 40) {
    lcd.clear();
    lcd.print("increment 38");
    encoder_refresh();
    encoder_back();
    increment_count += 1;
  } else if (increment_count == 41) {
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
  } else if (increment_count == 42) {
    lcd.clear();
    lcd.print("increment 40");
    increment(sensorValues);
    turnRight(sensorValues);
    increment_count += 1;
  } else if (increment_count == 43) {
    lcd.clear();
    lcd.print("increment 43");
    increment(sensorValues);
    turnLeft(sensorValues);
  } else if (increment_count == 44) {
    lcd.clear();
    lcd.print("increment 44");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 1)) {
      motor1.stop();
      motor2.stop();
      delay(50);
      fUltrasonic_read();
      line_follow_encoders(15);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 45) {
    lcd.clear();
    lcd.print("increment 43");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(50);
      lcd.clear();
      lcd.print("");
      turnLeft(sensorValues);
      line_follow_encoders(5);
      increment_count += 1;
      lcd.clear();
      lcd.print("Turn OK");
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 46) {
    lcd.clear();
    lcd.print("increment 38");
    encoder_refresh();
    encoder_back();
    increment_count += 1;
  } else if (increment_count == 47) {
    lcd.clear();
    lcd.print("increment 39");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(50);
      increment_count += 1;
    } else {
      go_straight_backward();
    }
  } else if (increment_count == 48) {
    fUltrasonic_read();
    lcd.clear();
    lcd.print("increment 44");
    if ((fdistance < 25)) {
      motor1.stop();
      motor2.stop();
      String distance = String(fdistance);
      lcd.clear();
      lcd.print(distance);
      delay(500);
      iterative_ultrasonic();
      place_the_box_first();
      //moveServo(arm, 40);
      delay(100);
      line_follow_encoders(distance_to_the_hole - 20);
      motor1.stop();
      motor2.stop();
      delay(100);
      place_the_box_second();
      //encoder_back_distance(3);
      // motor1.stop();
      // motor2.stop();
      // delay(1000);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 49) {
    lcd.clear();
    lcd.print("increment 45");
    //encoder_back_distance(10);
    zip_mode();
    //arm_down();
    line_follow_encoders(18);
    encoder_refresh();
    encoder_back_distance(10);
    increment_count += 1;
  } else if (increment_count == 50) {
    lcd.clear();
    lcd.print("increment 38");
    encoder_refresh();
    encoder_back();
    increment_count += 1;
  } else if (increment_count == 51) {
    lcd.clear();
    lcd.print("increment 39");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(50);
      increment_count += 1;
    } else {
      go_straight_backward();
    }
  } else if (increment_count == 52) {
    lcd.clear();
    lcd.print("increment 37");
    increment(sensorValues);
    turnRight(sensorValues);
    line_follow_encoders(5);
  } else if (increment_count == 53) {
    fUltrasonic_read();
    lcd.clear();
    lcd.print("increment 28");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && Front_IR == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 1)) {
      motor1.stop();
      motor2.stop();
      lcd.clear();
      lcd.print("Front Line");
      delay(50);
      increment(sensorValues);
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 54) {
    fUltrasonic_read();
    lcd.clear();
    lcd.print("increment 54");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && Front_IR == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 1)) {
      motor1.stop();
      motor2.stop();
      lcd.clear();
      lcd.print("Front Line");
      delay(50);
      increment(sensorValues);
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 55) {
    fUltrasonic_read();
    lcd.clear();
    lcd.print("increment 55");
    if ((sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0 && Front_IR == 0)) {
      motor1.stop();
      motor2.stop();
      delay(100);
      encoder_forward_distance(16);
      encoder_hundredeighty();
      motor1.stop();
      motor2.stop();
      delay(100);
      encoder_refresh();
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 56) {
    fUltrasonic_read();
    lcd.clear();
    lcd.print("increment 56");
    if ((sensorValues[0] == 1 || sensorValues[1] == 1 || sensorValues[2] == 1 || sensorValues[3] == 1 || sensorValues[4] == 1 || sensorValues[5] == 1 || sensorValues[6] == 1 || sensorValues[7] == 1)) {
      line_follow_encoders(17);
      motor1.stop();
      motor2.stop();
      delay(1000);
      increment_count += 1;
    } else {
      go_straight_forward();
    }
  } else if (increment_count == 57) {
    distance_measure_tof_with_five();
    delay(4000);
    max_height_function();
    increment_count += 1;
  } else if (increment_count == 58) {
    lcd.clear();
    lcd.print("increment 58");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && Front_IR == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 1)) {
      lcd.clear();
      lcd.print("Before Turn");
      turnLeft(sensorValues);
      line_follow_encoders(3);
      lcd.clear();
      lcd.print("After Turn");
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 59) {
    lcd.clear();
    lcd.print("increment 59");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && Front_IR == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 1)) {
      lcd.clear();
      lcd.print("Before Turn");
      turnRight(sensorValues);
      line_follow_encoders(3);
      lcd.clear();
      lcd.print("After Turn");
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 60) {
    fUltrasonic_read();  //Towards BOX 1
    lcd.clear();
    lcd.print("increment 60");
    if ((fdistance <= 3)) {
      ultrasonic_detection();
      increment_count += 1;
    } else if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && Front_IR == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 1)) {
      motor1.stop();
      motor2.stop();
      lcd.clear();
      lcd.print("Front Line");
      delay(100);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 61) {
    lcd.clear();
    lcd.print("increment 61");
    encoder_refresh();
    white_back(sensorValues);
    motor1.stop();
    motor2.stop();
    delay(50);
  } else if (increment_count == 62) {
    lcd.clear();
    lcd.print("increment 62");
    encoder_refresh();
    encoder_back();
    increment_count += 1;
  } else if (increment_count == 63) {
    lcd.clear();
    lcd.print("increment 63");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(100);
      increment_count += 1;
    } else {
      go_straight_backward();
    }
  } else if (increment_count == 64) {
    if (metal == "METAL") {
      metal_box_position = "POSITION 1";
      increment_count += 11;
    } else {
      iterative_PID(300);
      turnLeft(sensorValues);
      line_follow_encoders(3);
      increment_count += 1;
    }
  } else if (increment_count == 65) {
    fUltrasonic_read();  //Towards BOX 2
    lcd.clear();
    lcd.print("increment 65");
    if ((fdistance <= 3)) {
      ultrasonic_detection();
      increment_count += 1;
    } else if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && Front_IR == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 1)) {
      motor1.stop();
      motor2.stop();
      lcd.clear();
      lcd.print("Front Line");
      delay(100);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 66) {
    lcd.clear();
    lcd.print("increment 66");
    encoder_refresh();
    white_back(sensorValues);
    motor1.stop();
    motor2.stop();
    delay(50);
  } else if (increment_count == 67) {
    lcd.clear();
    lcd.print("increment 67");
    encoder_refresh();
    encoder_back();
    increment_count += 1;
  } else if (increment_count == 68) {
    lcd.clear();
    lcd.print("increment 68");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(100);
      increment_count += 1;
    } else {
      go_straight_backward();
    }
  } else if (increment_count == 69) {
    if (metal == "METAL") {
      metal_box_position = "POSITION 2";
      increment_count += 6;
    } else {
      iterative_PID(300);
      turnLeft(sensorValues);
      line_follow_encoders(3);
      metal_box_position = "POSITION 3";
      increment_count += 2;
    }
  } else if (increment_count == 70) {
    fUltrasonic_read();
    lcd.clear();
    lcd.print("increment 70");
    if ((fdistance <= 3)) {
      ultrasonic_detection();  //Towards BOX 3
      increment_count += 1;
    } else if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && Front_IR == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 1)) {
      motor1.stop();
      motor2.stop();
      lcd.clear();
      lcd.print("Front Line");
      delay(100);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 71) {
    lcd.clear();
    lcd.print("increment 71");
    encoder_refresh();
    white_back(sensorValues);
    motor1.stop();
    motor2.stop();
    delay(50);
  } else if (increment_count == 72) {
    lcd.clear();
    lcd.print("increment 72");
    encoder_refresh();
    encoder_back();
    increment_count += 1;
  } else if (increment_count == 73) {
    lcd.clear();
    lcd.print("increment 73");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(100);
      increment_count += 1;
    } else {
      go_straight_backward();
    }
  } else if (increment_count == 74) {
    metal_box_position = "POSITION 3";
    increment_count += 1;
  } else if (increment_count == 75) {
    lcd.clear();
    lcd.print("increment 75");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      encoder_refresh();
      white_back(sensorValues);
      motor1.stop();
      motor2.stop();
      delay(50);
      encoder_refresh();
      encoder_back_distance(10);
    } else {
      encoder_refresh();
      encoder_back_distance(10);
    }

  } else if (increment_count == 76) {
    lcd.clear();
    lcd.print("increment 76");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(50);
      increment(sensorValues);
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 77) {
    fUltrasonic_read();
    lcd.clear();
    lcd.print("increment 77");
    if ((fdistance < 20)) {
      motor1.stop();
      motor2.stop();
      String distance = String(fdistance);
      lcd.clear();
      lcd.print(distance);
      delay(50);
      moveServo(arm, 30);
      pick_the_box();
      iterative_PID(200);
      line_follow_encoders(12);
      motor1.stop();
      motor2.stop();
      delay(100);
      move_box_up();
      line_follow_encoders(5);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 78) {
    lcd.clear();
    lcd.print("increment 78");
    encoder_refresh();
    encoder_back();
    increment_count += 1;
  } else if (increment_count == 79) {
    lcd.clear();
    lcd.print("increment 79");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(50);
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
  } else if (increment_count == 80) {
    lcd.clear();
    lcd.print("increment 80");
    increment(sensorValues);
    turnRight(sensorValues);
    increment_count += 5;
  } else if (increment_count == 81) {
    lcd.clear();
    lcd.print("increment 81");
    increment(sensorValues);
    encoder_hundredeighty();
    line_follow_encoders(5);
    increment_count += 4;
  } else if (increment_count == 82) {
    lcd.clear();
    lcd.print("increment 82");
    encoder_refresh();
    encoder_back();
    increment_count += 1;
  } else if (increment_count == 83) {
    lcd.clear();
    lcd.print("increment 83");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(300);
      increment_count += 1;
    } else {
      go_straight_backward();
    }
  } else if (increment_count == 84) {
    lcd.clear();
    lcd.print("increment 84");
    increment(sensorValues);
    turnRight(sensorValues);
    increment_count += 1;
  } else if (increment_count == 85) {
    lcd.clear();
    lcd.print("increment 85");
    increment(sensorValues);
    turnLeft(sensorValues);
  } else if (increment_count == 86) {
    lcd.clear();
    lcd.print("increment 86");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 1)) {
      motor1.stop();
      motor2.stop();
      delay(50);
      fUltrasonic_read();
      increment(sensorValues);
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 87) {
    fUltrasonic_read();
    lcd.clear();
    lcd.print("increment 87");
    if ((fdistance < 25)) {
      motor1.stop();
      motor2.stop();
      String distance = String(fdistance);
      lcd.clear();
      lcd.print(distance);
      delay(500);
      iterative_ultrasonic();
      place_the_box_first();
      //moveServo(arm, 40);
      delay(100);
      line_follow_encoders(distance_to_the_hole - 20);
      motor1.stop();
      motor2.stop();
      delay(100);
      place_the_box_second();
      //encoder_back_distance(3);
      // motor1.stop();
      // motor2.stop();
      // delay(1000);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 88) {
    lcd.clear();
    lcd.print("increment 88");
    //encoder_back_distance(10);
    zip_mode();
    //arm_down();
    line_follow_encoders(18);
    encoder_refresh();
    encoder_back_distance(10);
    increment_count += 1;
  } else if (increment_count == 89) {
    lcd.clear();
    lcd.print("increment 89");
    encoder_refresh();
    encoder_back();
    increment_count += 1;
  } else if (increment_count == 90) {
    lcd.clear();
    lcd.print("increment 90");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(50);
      increment_count += 1;
    } else {
      go_straight_backward();
    }
  } else if (increment_count == 91) {
    lcd.clear();
    lcd.print("increment 91");
    increment(sensorValues);
    turnLeft(sensorValues);
    line_follow_encoders(5);
  } else if (increment_count == 92) {
    lcd.clear();
    lcd.print("increment 90");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(50);
      encoder_forward_distance(2);
      turnLeft(sensorValues);
      encoder_forward_distance(12);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 93) {
    lcd.clear();
    lcd.print("increment 93");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(50);
      increment_count += 1;
    } else {
      go_straight_backward();
    }
  } else if (increment_count == 94) {
    fUltrasonic_read();
    lcd.clear();
    lcd.print("increment 94");
    if ((fdistance < 25)) {
      motor1.stop();
      motor2.stop();
      String distance = String(fdistance);
      lcd.clear();
      lcd.print(distance);
      delay(500);
      iterative_ultrasonic();
      delay(100);
      line_follow_encoders(distance_to_the_hole - 10);
      motor1.stop();
      motor2.stop();
      delay(100);
      ball_picking();
      zip_mode_shooter();
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 95) {
    lcd.clear();
    lcd.print("increment 95");
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1) || (sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)) {
      motor1.stop();
      motor2.stop();
      delay(1000);
      encoder_refresh();
      encoder_hundredeighty();
      motor1.stop();
      motor2.stop();
      //delay(15000);
      increment_count += 1;
    } else {
      go_straight_backward();
    }
  } else if (increment_count == 96) {
    if ((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 1)) {
      motor1.stop();
      motor2.stop();
      delay(1000);
      lfspeed = 140;
      encoder_refresh();
      encoder_hundredeighty();
      delay(100);
      encoder_refresh();
      line_follow_encoders(20);
      encoder_refresh();
      encoder_back_distance(36);
      motor1.stop();
      motor2.stop();
      delay(1000);
      increment_count += 1;
    } else {
      line_follow(sensorValues);
    }
  } else if (increment_count == 97) {
    shooting_angle_calculation();
    moveServo(shooter, shooting_angle);
    delay(2000);
    shoot();
    delay(3000);
    shoot();
    delay(3000);
    shoot();
    delay(3000);
  } else if (increment_count == 1000) {
    distance_measure_tof();
    delay(1000);

  } else if (increment_count == 1001) {
    // distance_measure_tof_specific_angle(52);
    // delay(2000);
    moveServo_slow(radar, 90);
    delay(2000);
    // moveServo_slow(boxArm, 95);
    // delay(2000);

  } else if (increment_count == 2000) {
    calibrateSensors();
    motor1.stop();
    motor2.stop();
    delay(3000);
    increment_count += 1;
  } else if (increment_count == 2001) {
    if (digitalRead(switch_1_calib) == HIGH && digitalRead(switch_2_main_slow) == LOW) {
      speed = 150;
    turns_speed = 190;
    lfspeed = speed;
      increment_count = -1;
    } else if (digitalRead(switch_1_calib) == HIGH && digitalRead(switch_4_main_medium) == LOW) {
      speed = 165;
    turns_speed = 200;
    lfspeed = speed;
      increment_count = -1;
    } else if (digitalRead(switch_1_calib) == HIGH && digitalRead(switch_5_main_fast) == LOW) {
      speed = 185;
    turns_speed = 210;
    lfspeed = speed;
      increment_count = -1;
    }
  } else if (increment_count == 3000) {
    line_follow_encoders_without_distance();
  }
}

void increment(int *sensorValues) {
  motor1.stop();
  motor2.stop();
  delay(50);
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

void turnLeft(int *sensorValues) {
  // Perform a sharp left turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool enoughturn = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0;
  encoderACount = 0;
  encoderBCount = 0;
  encoder_forward_distance(9);
  encoderACount = 0;
  encoderBCount = 0;
  lfspeed = turns_speed;
  left_encoder_fouryfive();
  sensorValues[SensorCount];
  readSensors(sensorValues);
  while (!enoughturn) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    enoughturn = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0;
    if (enoughturn) {
      break;
    }
    left_encoder();
  }
  lfspeed = speed;
  line_follow_encoders(3);
  motor1.stop();
  motor2.stop();
  delay(50);
}

void turnRight(int *sensorValues) {
  // Perform a sharp right turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  encoderACount = 0;
  encoderBCount = 0;
  encoder_forward_distance(9);
  encoderACount = 0;
  encoderBCount = 0;
  lfspeed = turns_speed;
  right_encoder_fouryfive();
  sensorValues[SensorCount];
  readSensors(sensorValues);
  while (!enoughturn) {
    sensorValues[SensorCount];
    readSensors(sensorValues);
    enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
    if (enoughturn) {
      break;
    }
    right_encoder();
  }
  lfspeed = speed;
  line_follow_encoders(3);
  motor1.stop();
  motor2.stop();
  delay(50);
}

void turnLeft_t(int *sensorValues) {
  // Perform a sharp left turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool enoughturn = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0 && Front_IR == 0;
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  bool forward = sensorValues[0] == 1 && sensorValues[1] == 1;
  int Front_IR_Analog = analogRead(Front_IR_Pin);
  readSensors(sensorValues);
  encoderACount = 0;
  encoderBCount = 0;
  lfspeed = turns_speed;
  left_encoder_ninty();
  lfspeed = speed;
  line_follow_encoders(7);
  motor1.stop();
  motor2.stop();
  delay(50);
}

void turnRight_t(int *sensorValues) {
  // Perform a sharp right turn
  // Assuming motor1 is the right motor and motor2 is the left motor
  bool enoughturn = sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1 && Front_IR == 0;
  bool whitedetect = sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1;
  bool forward = sensorValues[6] == 1 && sensorValues[7] == 1;
  readSensors(sensorValues);
  encoderACount = 0;
  encoderBCount = 0;
  lfspeed = turns_speed;
  right_encoder_ninty();
  lfspeed = speed;
  line_follow_encoders(7);
  motor1.stop();
  motor2.stop();
  delay(50);
}

void noLine_turnLeft(int *sensorValues) {
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

void noLine_turnRight(int *sensorValues) {
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
