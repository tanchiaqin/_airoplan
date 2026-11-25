// Controller = Raspberry Pi Pico W
// Motor driver = Cytron Maker Pi Pico
// Communication = Built-in Pico W Classic Bluetooth (SerialBT.h) -> Android Serial Bluetooth Terminal
// Color sensor = TCS34725 (I2C0: SDA=GP4, SCL=GP5)
// Line sensor = Two LM358 IR sensors (GP3, GP17)
// Servo 1 = GP12

#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <Servo.h>
#include <SerialBT.h>

// Motor Driver Pins
const int M1A = 8;
const int M1B = 9;
const int M2A = 10;
const int M2B = 11;

// IR Line Sensor Pins
const int LEFT_IR_PIN = 3;
const int RIGHT_IR_PIN = 17;

// Scoop Servo Pin
const int SERVO_PIN = 12;

// PWM Speed Configuration (50% of 255 = 127)
const int MOTOR_SPEED_PERCENT = 50;
const int MOTOR_PWM = (MOTOR_SPEED_PERCENT * 255L) / 100;
const int MAX_PWM = 255;

const bool INVERT_LEFT_MOTOR = false;
const bool INVERT_RIGHT_MOTOR = false;

const int BLACK = HIGH;
const int WHITE = LOW;

// Color Sensor Initialization
Adafruit_TCS34725 colorSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
bool colorSensorReady = false;

enum Color {
  UNKNOWN,
  RED,
  BLUE,
  GREEN
};

Color targetColor = UNKNOWN;
Color detectedColor = UNKNOWN;

// Scoop Servo Angles
Servo scoopServo;
const int SERVO_REST_ANGLE = 90;
const int SERVO_SCOOP_ANGLE = 30;

// Robot State Machines
enum AutoState {
  AUTO_START,
  READ_FIRST_MARKER,
  FOLLOW_LINE,
  SEARCH_TARGET_MARKER,
  TARGET_FOUND,
  AUTO_FINISHED,
  AUTO_ERROR
};

enum Mode {
  MANUAL,
  AUTO
};

Mode currentMode = MANUAL;
AutoState autoState = AUTO_START;

// Bluetooth Commands:
// 'F'=Forward, 'B'=Backward, 'L'=Left, 'R'=Right, 'S'=Stop
// 'A'=Auto Mode, 'M'=Manual Mode
// 'U'=Scoop Up (Release), 'D'=Scoop Down (Scoop)
char bluetoothCommand = 'S';
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 400;

// Color Calibration Thresholds
const uint16_t MIN_CLEAR_VALUE = 100;
const float RED_THRESHOLD = 0.45;
const float BLUE_THRESHOLD = 0.40;
const float GREEN_THRESHOLD = 0.40;

int leftIRValue = LOW;
int rightIRValue = LOW;

void setup() {
  Serial.begin(115200);

  // Configure I2C0 for TCS34725 Color Sensor (GP4 = SDA, GP5 = SCL)
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();

  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
  stopMotors();

  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);

  scoopServo.attach(SERVO_PIN, 500, 2500);
  releaseBall();

  colorSensorReady = colorSensor.begin();
  if (colorSensorReady) {
    Serial.println("Color sensor is ready");
  } else {
    Serial.println("Color sensor error - check GP4/GP5 wiring");
  }

  // Initialize Pico W Built-in Bluetooth (setName must come BEFORE begin)
  SerialBT.setName("AIRO-PLAN AIROST");
  SerialBT.begin(115200);

  Serial.println("AIRO-PLAN system is ready | Default mode: Manual");
}

void loop() {
  readBluetooth();

  if (currentMode == MANUAL) {
    manualMode();
  } else if (currentMode == AUTO) {
    autoMode();
  } else {
    stopMotors();
  }
}

// ================= MOTOR CONTROL =================
void setLeftMotor(int speed) {
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  if (INVERT_LEFT_MOTOR) speed = -speed;

  if (speed > 0) {
    analogWrite(M1A, speed);
    analogWrite(M1B, 0);
  } else if (speed < 0) {
    analogWrite(M1A, 0);
    analogWrite(M1B, -speed);
  } else {
    analogWrite(M1A, 0);
    analogWrite(M1B, 0);
  }
}

void setRightMotor(int speed) {
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  if (INVERT_RIGHT_MOTOR) speed = -speed;

  if (speed > 0) {
    analogWrite(M2A, speed);
    analogWrite(M2B, 0);
  } else if (speed < 0) {
    analogWrite(M2A, 0);
    analogWrite(M2B, -speed);
  } else {
    analogWrite(M2A, 0);
    analogWrite(M2B, 0);
  }
}

void moveForward()  { setLeftMotor(MOTOR_PWM);  setRightMotor(MOTOR_PWM); }
void moveBackward() { setLeftMotor(-MOTOR_PWM); setRightMotor(-MOTOR_PWM); }
void turnLeft()     { setLeftMotor(-MOTOR_PWM); setRightMotor(MOTOR_PWM); }
void turnRight()    { setLeftMotor(MOTOR_PWM);  setRightMotor(-MOTOR_PWM); }
void stopMotors()   { setLeftMotor(0);          setRightMotor(0); }

// ================= SERVO CONTROL =================
void scoopBall()   { scoopServo.write(SERVO_SCOOP_ANGLE); }
void releaseBall() { scoopServo.write(SERVO_REST_ANGLE); }

// ================= BUILT-IN BLUETOOTH RECEIVING =================
void readBluetooth() {
  while (SerialBT.available() > 0) {
    char command = (char)SerialBT.read();
    if (command == '\n' || command == '\r' || command == ' ') {
      continue;
    }

    command = toupper(command);  // Accepts both 'f' and 'F'
    bluetoothCommand = command;
    lastCommandTime = millis();

    Serial.print("BT Received: ");
    Serial.println(command);

    if (command == 'A') {
      stopMotors();
      autoState = AUTO_START;
      targetColor = UNKNOWN;
      currentMode = AUTO;
      bluetoothCommand = 'S';
    } 
    else if (command == 'M') {
      stopMotors();
      currentMode = MANUAL;
      bluetoothCommand = 'S';
    }
    else if (command == 'U' && currentMode == MANUAL) {
      releaseBall();
    }
    else if (command == 'D' && currentMode == MANUAL) {
      scoopBall();
    }
  }
}

// ================= MANUAL MODE =================
void manualMode() {
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    stopMotors();
    bluetoothCommand = 'S';
    return;
  }

  switch (bluetoothCommand) {
    case 'F': moveForward();  break;
    case 'B': moveBackward(); break;
    case 'L': turnLeft();     break;
    case 'R': turnRight();    break;
    default:  stopMotors();   break;
  }
}

// ================= SENSORS & COLOR =================
void readLineSensors() {
  leftIRValue = digitalRead(LEFT_IR_PIN);
  rightIRValue = digitalRead(RIGHT_IR_PIN);
}

Color readDetectedColor() {
  if (!colorSensorReady) {
    return UNKNOWN;
  }

  uint16_t r, g, b, c;
  colorSensor.getRawData(&r, &g, &b, &c);

  if (c < MIN_CLEAR_VALUE) {
    return UNKNOWN;
  }

  float total = (float)r + g + b;
  if (total <= 0) {
    return UNKNOWN;
  }

  float redRatio = r / total;
  float greenRatio = g / total;
  float blueRatio = b / total;

  if (redRatio > RED_THRESHOLD && redRatio > greenRatio && redRatio > blueRatio) return RED;
  if (blueRatio > BLUE_THRESHOLD && blueRatio > redRatio && blueRatio > greenRatio) return BLUE;
  if (greenRatio > GREEN_THRESHOLD && greenRatio > redRatio && greenRatio > blueRatio) return GREEN;

  return UNKNOWN;
}

void storeTargetColor() {
  Color newColor = readDetectedColor();
  if (newColor != UNKNOWN) {
    targetColor = newColor;
    Serial.print("Target Color Stored: ");
    if (targetColor == RED) Serial.println("RED");
    else if (targetColor == BLUE) Serial.println("BLUE");
    else if (targetColor == GREEN) Serial.println("GREEN");
    autoState = FOLLOW_LINE;
  }
}

bool checkTargetColor() {
  detectedColor = readDetectedColor();
  return (detectedColor != UNKNOWN && detectedColor == targetColor);
}

// ================= AUTO MODE =================
void autoMode() {
  readLineSensors();

  switch (autoState) {
    case AUTO_START:
      releaseBall();
      autoState = READ_FIRST_MARKER;
      break;

    case READ_FIRST_MARKER:
      stopMotors();
      storeTargetColor();
      break;

    case FOLLOW_LINE:
      if (leftIRValue == WHITE && rightIRValue == WHITE) {
        moveForward();
      } else if (leftIRValue == BLACK && rightIRValue == WHITE) {
        turnLeft();
      } else if (leftIRValue == WHITE && rightIRValue == BLACK) {
        turnRight();
      } else {
        stopMotors();
        autoState = SEARCH_TARGET_MARKER;
      }
      break;

    case SEARCH_TARGET_MARKER:
      if (checkTargetColor()) {
        autoState = TARGET_FOUND;
      } else {
        moveForward();
        delay(150);
        autoState = FOLLOW_LINE;
      }
      break;

    case TARGET_FOUND:
      stopMotors();
      scoopBall();
      delay(500);
      autoState = AUTO_FINISHED;
      break;

    case AUTO_FINISHED:
    case AUTO_ERROR:
    default:
      stopMotors();
      break;
  }
}
