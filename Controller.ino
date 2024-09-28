#include <Bluepad32.h>
#include <math.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
LiquidCrystal_I2C lcd(0x27, 20, 4);

int L1andR1pressTime = 0;
String entermode = "";
bool isMobileRobot = false;
bool isRobotArm = false;
int angleServo1;
int angleServo2;
int angleServo3;

Servo servo1;
Servo servo2;
Servo servo3;

const int servoPin1 = 12;
const int servoPin2 = 13;
const int servoPin3 = 23;

#define ENA 14  // Motor A speed control
#define IN1 27  // Motor A direction
#define IN2 26  // Motor A direction
#define ENB 32  // Motor B speed control
#define IN3 25  // Motor B direction
#define IN4 33  // Motor B direction

#define M1inA 36
#define M1inB 39
#define M2inA 34
#define M2inB 35

#define pulPin1 18
#define enaPin1 19
#define pulPin2 17
#define enaPin2 5
#define pulPin3 4
#define enaPin3 16
#define dirPin 2

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      clearLine(1);
      lcd.print("Controller connected");
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      //Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
      //              properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    clearLine(1);
    lcd.print("Controller connected");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      clearLine(1);
      lcd.print("Controller is discon");
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    clearLine(1);
    lcd.print("Controller is discon");
  }
}

void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
    "idx=%d, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, L2: %4d, L1: %4d, R2: %4d, R1: %4d, "
    "misc: %4d, Button a:%6d b:%6d x:%6d y:%6d, D-pad %8d \n",
    ctl->index(),    // Controller Index
    ctl->buttons(),  // bitmask of pressed buttons
    ctl->axisX(),    // (-511 - 512) left X Axis
    ctl->axisY(),    // (-511 - 512) left Y axis
    ctl->axisRX(),   // (-511 - 512) right X axis
    ctl->axisRY(),   // (-511 - 512) right Y axis
    ctl->brake(),    // (0 - 1023): brake button
    ctl->l1(),
    ctl->throttle(),
    ctl->r1(),
    ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    ctl->a(),
    ctl->b(),
    ctl->x(),
    ctl->y(),
    ctl->dpad());
}

void processGamepad(ControllerPtr ctl) {
  // Print the controller's index
  //Serial.printf("Controller index: %d\n", ctl->index());

  // Check each button and print if pressed
  /*if (ctl->a()) {
    Serial.println("Button A pressed");
  }
  if (ctl->b()) {
    Serial.println("Button B pressed");
  }
  if (ctl->x()) {
    Serial.println("Button X pressed");
  }
  if (ctl->y()) {
    Serial.println("Button Y pressed");
  }
  if (ctl->l1()) {
    Serial.println("Button L1 pressed");
  }
  if (ctl->r1()) {
    Serial.println("Button R1 pressed");
  }
  if (ctl->l2()) {
    Serial.println("Button L2 pressed");
  }
  if (ctl->r2()) {
    Serial.println("Button R2 pressed");
  }
  if (ctl->miscSelect()) {  // Corrected from 'select' to 'miscSelect'
    Serial.println("Select button pressed");
  }
  if (ctl->miscStart()) {  // Corrected from 'start' to 'miscStart'
    Serial.println("Start button pressed");
  }
  if (ctl->dpad() & DPAD_UP) {  // Corrected from 'up' to 'dpad' with mask DPAD_UP
    Serial.println("D-pad Up pressed");
  }
  if (ctl->dpad() & DPAD_DOWN) {  // Corrected from 'down' to 'dpad' with mask DPAD_DOWN
    Serial.println("D-pad Down pressed");
  }
  if (ctl->dpad() & DPAD_LEFT) {  // Corrected from 'left' to 'dpad' with mask DPAD_LEFT
    Serial.println("D-pad Left pressed");
  }
  if (ctl->dpad() & DPAD_RIGHT) {  // Corrected from 'right' to 'dpad' with mask DPAD_RIGHT
    Serial.println("D-pad Right pressed");
  }*/

  // Call the dump function to print all the axis and button info
  //dumpGamepad(ctl);
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        //processGamepad(myController);
        selection(myController);
        if(isRobotArm == true){
          processRobotArm(myController);
        }else if(isMobileRobot == true){
          processRobotWheels(myController);
        }
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

void forward(int steps, int ENAPIN, int PULPIN) {
  digitalWrite(ENAPIN, LOW);  // Enable motor driver
  digitalWrite(dirPin, LOW);  // Set direction forward

  for (int i = 0; i < steps; i++) {  // Fix initialization of 'i'
    digitalWrite(PULPIN, HIGH);      // Pulse to move stepper motor
    delayMicroseconds(500); 
    digitalWrite(PULPIN, LOW);
    delayMicroseconds(500); 
  }
}

void backward(int steps, int ENAPIN, int PULPIN) {
  digitalWrite(ENAPIN, LOW);   // Enable motor driver
  digitalWrite(dirPin, HIGH);  // Set direction forward

  for (int i = 0; i < steps; i++) {  // Fix initialization of 'i'
    digitalWrite(PULPIN, HIGH);      // Pulse to move stepper motor
    delayMicroseconds(1000); 
    digitalWrite(PULPIN, LOW);
    delayMicroseconds(1000); 
  }
}

void processRobotWheels(ControllerPtr ctl) {
  int axisY = ctl->axisY();
  int axisX = ctl->axisX();
  int axisRX = ctl->axisRX();  // (-511 - 512) right X axis
  int axisRY = ctl->axisRY();  // (-511 - 512) right Y axis
  int L2 = ctl->brake();
  int R2 = ctl->throttle();
  int speed_motor_a;
  int speed_motor_b;

  if (L2 == 1020 && R2 == 1020) {
    Serial.print("|||BEAST MODE!!!|||");
    if (axisY < 0) {
      axisY = map(axisY, -512, -1, 255, 0);
      //Serial.print("speed forward : ");
      //Serial.println(axisY);
      speed_motor_a = axisY;
      speed_motor_b = axisY;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENA, speed_motor_a);
      analogWrite(ENB, speed_motor_b);
    } else if (axisY > 0) {
      axisY = map(axisY, 1, 508, 0, 255);
      //Serial.print("speed backward : ");
      //Serial.println(axisY);
      speed_motor_a = axisY;
      speed_motor_b = axisY;
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENA, speed_motor_a);
      analogWrite(ENB, speed_motor_b);
    } else {
      axisY = 0;
    }

    if (axisRX < 0) {
      axisRX = map(axisRX, -512, -1, 255, 0);
      //Serial.print("speed Left : ");
      //Serial.println(axisX);
      speed_motor_a = axisRX;
      speed_motor_b = axisRX;
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENA, speed_motor_a);
      analogWrite(ENB, speed_motor_b);
    } else if (axisRX > 0) {
      axisRX = map(axisRX, 1, 508, 0, 255);
      //Serial.print("speed Right : ");
      //Serial.println(axisX);
      speed_motor_a = axisRX;
      speed_motor_b = axisRX;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENA, speed_motor_a);
      analogWrite(ENB, speed_motor_b);
    } else {
      axisX = 0;
    }
  } else {
    if (axisY < 0) {
      axisY = map(axisY, -512, -1, 127, 0);
      //Serial.print("speed forward : ");
      //Serial.println(axisY);
      speed_motor_a = axisY;
      speed_motor_b = axisY;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENA, speed_motor_a);
      analogWrite(ENB, speed_motor_b);
    } else if (axisY > 0) {
      axisY = map(axisY, 1, 508, 0, 127);
      //Serial.print("speed backward : ");
      //Serial.println(axisY);
      speed_motor_a = axisY;
      speed_motor_b = axisY;
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENA, speed_motor_a);
      analogWrite(ENB, speed_motor_b);
    } else {
      axisY = 0;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }

    if (axisRX < 0) {
      axisRX = map(axisRX, -512, -1, 127, 0);
      //Serial.print("speed Left : ");
      //Serial.println(axisX);
      speed_motor_a = axisRX;
      speed_motor_b = axisRX;
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENA, speed_motor_a);
      analogWrite(ENB, speed_motor_b);
    } else if (axisRX > 0) {
      axisRX = map(axisRX, 1, 508, 0, 127);
      //Serial.print("speed Right : ");
      //Serial.println(axisX);
      speed_motor_a = axisRX;
      speed_motor_b = axisRX;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENA, speed_motor_a);
      analogWrite(ENB, speed_motor_b);
    } else {
      axisX = 0;
    }
  }

  if (axisRX == 0 && axisY == 0) {
    speed_motor_a = 0;
    speed_motor_b = 0;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, speed_motor_a);
    analogWrite(ENB, speed_motor_b);
  }

  Serial.print("Speed Motor Left : ");
  Serial.print(speed_motor_a);
  Serial.print(" Speed Motor Right : ");
  Serial.println(speed_motor_b);

  //Serial.println(joint_left_angle);
}

void processRobotArm(ControllerPtr ctl) {
  int axisRY = ctl->axisRY();
  int axisRX = ctl->axisRX();
  int axisY = ctl->axisY();
  int axisX = ctl->axisX();
  int L1 = ctl->l1();
  int R1 = ctl->r1();
  int dpad = ctl->dpad();

  if (dpad == 4) {
    forward(49, enaPin1, pulPin1);
  } else if (dpad == 8) {
    backward(49, enaPin1, pulPin1);
  }

  if (dpad == 1) {
    forward(49, enaPin2, pulPin2);
  } else if (dpad == 2) {
    backward(49, enaPin2, pulPin2);
  }

  if (L1 == 1) {
    forward(16, enaPin3, pulPin3);
  } else if (R1 == 1) {
    backward(16, enaPin3, pulPin3);
  }

  if (axisY == -512){
    angleServo1 = angleServo1 + 1;
    if (angleServo1 > 180){
      angleServo1 = 180;
    }
  }else if (axisY == 508){
    angleServo1 = angleServo1 - 1;
    if (angleServo1 < 0){
      angleServo1 = 0;
    }
  }else if (axisX == -512){
    angleServo2 = angleServo2 + 1;
    if (angleServo2 > 180){
      angleServo2 = 180;
    }
  }else if (axisX == 508){
    angleServo2 = angleServo2 - 1;
    if (angleServo2 < 0){
      angleServo2 = 0;
    }
  }

  if (axisRY == -512){
    angleServo3 = angleServo3 + 1;
    if (angleServo3 > 100){
      angleServo3 = 100;
    }
  }else if (axisRY == 508){
    angleServo3 = angleServo3 - 1;
    if (angleServo3 < 80){
      angleServo3 = 80;
    }
  }

  Serial.print(angleServo1);
  Serial.print(" ");
  Serial.print(angleServo2);
  Serial.print(" ");
  Serial.println(angleServo3);

}

void selection(ControllerPtr ctl) {
  int L2 = ctl->l2();
  int R2 = ctl->r2();
  int X = ctl->x();
  int Y = ctl->y();
  int A = ctl->a();
  int B = ctl->b();
  int miscbuttons = ctl->miscButtons();

  if (L2 == 1 && R2 == 1) {
    L1andR1pressTime = L1andR1pressTime + 1;
    Serial.println(L1andR1pressTime);

    if (L1andR1pressTime >= 60) {
      // Action to perform after holding for 2 seconds
      Serial.println("L1 and R1 held for 2 seconds");
      if (miscbuttons == 4) {
        entermode = "ModeSet";
        clearLine(2);
        lcd.setCursor(0, 2);
        lcd.println("Enter RobotArm");
        isRobotArm = true;
        isMobileRobot = false;
      } else if (miscbuttons == 2) {
        entermode = "manual";
        clearLine(2);
        lcd.setCursor(0, 2);
        lcd.print("Enter MobileRobot");
        isMobileRobot = true;
        isRobotArm = false;
      }

      if (entermode == "ModeSet") {
        if (Y == 1) {
          clearLine(3);
          lcd.setCursor(0, 3);
          lcd.print("mode 1");
        } else if (X == 1) {
          clearLine(3);
          lcd.setCursor(0, 3);
          lcd.print("mode 2");
        } else if (A == 1) {
          clearLine(3);
          lcd.setCursor(0, 3);
          lcd.print("mode 3");
        } else if (B == 1) {
          clearLine(3);
          lcd.setCursor(0, 3);
          lcd.print("exis mode");
        }
      }
    }
  } else {
    L1andR1pressTime = 0;
  }
}

void clearLine(int line) {
  lcd.setCursor(0, line);             // กำหนดตำแหน่งเคอร์เซอร์ที่บรรทัดที่ต้องการลบ
  lcd.print("                    ");  // เขียนช่องว่าง 20 ตัว (จำนวนคอลัมน์ของจอ)
  lcd.setCursor(0, line);             // นำเคอร์เซอร์กลับมาตำแหน่งเริ่มต้นของบรรทัดที่ถูกลบ
}

void setup() {
  Serial.begin(115200);

  // Set motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(pulPin1, OUTPUT);
  pinMode(enaPin1, OUTPUT);
  pinMode(pulPin2, OUTPUT);
  pinMode(enaPin2, OUTPUT);
  pinMode(pulPin3, OUTPUT);
  pinMode(enaPin3, OUTPUT);
  pinMode(dirPin, OUTPUT);

  digitalWrite(enaPin1, HIGH);
  digitalWrite(enaPin2, HIGH);
  digitalWrite(enaPin3, HIGH);
  digitalWrite(dirPin, LOW);

  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);

  servo1.write(90);
  servo2.write(0);
  servo3.write(90);

  lcd.init();       // Initialize the LCD
  lcd.backlight();  // Turn on the backlight

  lcd.setCursor(2, 1);  // Set cursor to column 0, row 1
  lcd.print("Welcome to Our's");

  lcd.setCursor(3, 2);  // Set cursor to column 0, row 2
  lcd.print("Robot Project");

  delay(1500);

  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Bluepad32 for Arduino");
  const uint8_t* addr = BP32.localBdAddress();
  lcd.setCursor(0, 2);
  lcd.print("BD Addr: FD:03:DB:62:C5:37");

  delay(1500);
  lcd.clear();
  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
}

void loop() {
  bool dataUpdated = BP32.update();
  if (dataUpdated){
    processControllers();
  }
  servo1.write(angleServo1);
  servo2.write(angleServo2);
  servo3.write(angleServo3);
}
