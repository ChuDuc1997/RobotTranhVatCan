#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h>

#define MIDDLE_TRIGGER_PIN A0
#define MIDDLE_ECHO_PIN A1
#define LEFT_TRIGGER_PIN A2
#define LEFT_ECHO_PIN A3
#define RIGHT_TRIGGER_PIN A4
#define RIGHT_ECHO_PIN A5
#define SERVO_PIN 10

#define MAX_DISTANCE 300
#define MOTOR_SPEED 150

#define MIDDLE_DISTANCE 20
#define LEFT_DISTANCE 15
#define RIGHT_DISTANCE 15

enum Angle{
  TURN_LEFT = 0,
  MIDDLE = 90,
  TURN_RIGHT = 180
};

enum Direction{
  STOP,
  UP,
  DOWN,
  LEFT,
  RIGHT
};

class SERVO{
protected:
  Servo servo;
  unsigned int pin;
  unsigned int angle;

public:
  SERVO(){
    pin = SERVO_PIN;
    angle = MIDDLE;
  }

  void attach(){
    servo.attach(pin);
  }

  void write(){
    servo.write(angle);
  }

  unsigned int setAngle(unsigned int a){
    angle = a;
  }

  unsigned int getAngle(){
    return angle;
  }
};

class MOTOR{
private:
  AF_DCMotor leftMotor = AF_DCMotor(1, MOTOR12_1KHZ);
  AF_DCMotor rightMotor = AF_DCMotor(4, MOTOR34_1KHZ);
  unsigned int direction = STOP;

public:
  void setSpeed(uint8_t speed){
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(speed);
  }

  void stop(){
    leftMotor.run(RELEASE);
    rightMotor.run(RELEASE);
  }

  void forward(){
    leftMotor.run(FORWARD);
    rightMotor.run(FORWARD);
  }

  void backward(){
    leftMotor.run(BACKWARD);
    rightMotor.run(BACKWARD);
  }

  void turnLeft(){
    leftMotor.run(FORWARD);
    rightMotor.run(BACKWARD);
  }

  void turnRight(){
    leftMotor.run(BACKWARD);
    rightMotor.run(FORWARD);
  }

  void setDirection(unsigned int d){
    direction = d;
  }

  unsigned int getDirection(){
    return direction;
  }
};

class US{
protected:
  NewPing middleSonar = NewPing(MIDDLE_TRIGGER_PIN, MIDDLE_ECHO_PIN, MAX_DISTANCE);
  NewPing leftSonar = NewPing(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);
  NewPing rightSonar = NewPing(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);

public:
  unsigned int readDistanceMiddle(){
    return middleSonar.ping_cm();
  }

  unsigned int readDistanceLeft(){
    return leftSonar.ping_cm();
  }

  unsigned int readDistanceRight(){
    return rightSonar.ping_cm();
  }

  bool alertMiddle(){
    unsigned int middleDistance = readDistanceMiddle();
    if(middleDistance > 0 && middleDistance <= MIDDLE_DISTANCE)
      return true;
    return false;
  }

  bool alertRight(){
    unsigned int rightDistance = readDistanceRight();
    if(rightDistance > 0 && rightDistance <= RIGHT_DISTANCE)
      return true;
    return false;
  }

  bool alertLeft(){
    unsigned int leftDistance = readDistanceLeft();
    if(leftDistance > 0 && leftDistance <= LEFT_DISTANCE)
      return true;
    return false;
  }

  unsigned int alert(){
    bool flagMiddle = alertMiddle();
    bool flagLeft = alertLeft();
    bool flagRight = alertRight();

    if (flagMiddle || (flagLeft && flagRight)){
      return 1;
    }else if (flagLeft){
      return 2;
    }else if (flagRight){
      return 3;
    }else{
      return 0;
    }
  }
};

SemaphoreHandle_t motorSemaphore;
SemaphoreHandle_t servoSemaphore;
SemaphoreHandle_t usSemaphore;

MOTOR motor;
US us;
SERVO servoUS = SERVO();

void setup(){
  motor.setSpeed(MOTOR_SPEED);
  servoUS.attach();
  servoUS.write();

  motorSemaphore = xSemaphoreCreateBinary();
  servoSemaphore = xSemaphoreCreateBinary();
  usSemaphore = xSemaphoreCreateBinary();

  xTaskCreate(readDistance, "Read Distance", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(controlMotor, "Control Motor", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(controlServo, "Control Servo", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}

void loop() {}

void readDistance(void *p){
  unsigned int flag = 0;
  unsigned int leftDistance = 0;
  unsigned int rightDistance = 0;

  while (1){
    flag = us.alert();
    if (flag == 1){
      motor.setDirection(STOP);
      xSemaphoreGive(motorSemaphore);
      vTaskDelay(pdMS_TO_TICKS(100));

      motor.setDirection(DOWN);
      xSemaphoreGive(motorSemaphore);
      vTaskDelay(pdMS_TO_TICKS(500));

      motor.setDirection(STOP);
      xSemaphoreGive(motorSemaphore);
      vTaskDelay(pdMS_TO_TICKS(100));

      servoUS.setAngle(TURN_LEFT);
      xSemaphoreGive(servoSemaphore);
      vTaskDelay(pdMS_TO_TICKS(500));
      leftDistance = us.readDistanceMiddle();

      servoUS.setAngle(TURN_RIGHT);
      xSemaphoreGive(servoSemaphore);
      vTaskDelay(pdMS_TO_TICKS(500));
      rightDistance = us.readDistanceMiddle();

      servoUS.setAngle(MIDDLE);
      xSemaphoreGive(servoSemaphore);
      vTaskDelay(pdMS_TO_TICKS(100));

      if((leftDistance > 0 && leftDistance <= LEFT_DISTANCE) && (rightDistance > 0 && rightDistance <= RIGHT_DISTANCE)){
        continue;
      }
      else{
        if(leftDistance > rightDistance){
          motor.setDirection(RIGHT);
          xSemaphoreGive(motorSemaphore);
          vTaskDelay(pdMS_TO_TICKS(180));
        }else{
          motor.setDirection(LEFT);
          xSemaphoreGive(motorSemaphore);
          vTaskDelay(pdMS_TO_TICKS(180));
        }
      }
    }else if(flag == 2){
      motor.setDirection(RIGHT);
      xSemaphoreGive(motorSemaphore);
      vTaskDelay(pdMS_TO_TICKS(20));
    }else if(flag == 3){
      motor.setDirection(LEFT);
      xSemaphoreGive(motorSemaphore);
      vTaskDelay(pdMS_TO_TICKS(20));
    }

    motor.setDirection(UP);
    xSemaphoreGive(motorSemaphore);
    vTaskDelay(pdMS_TO_TICKS(50));
    
  }
}

void controlServo(void *p){
  while (1){
    xSemaphoreTake(servoSemaphore, portMAX_DELAY);
    servoUS.write();
  }
}

void controlMotor(void *p){
  while (1){
    xSemaphoreTake(motorSemaphore, portMAX_DELAY);
    switch (motor.getDirection()){
    case STOP:
      motor.stop();
      break;
    case UP:
      motor.forward();
      break;
    case DOWN:
      motor.backward();
      break;
    case LEFT:
      motor.turnLeft();
      break;
    case RIGHT:
      motor.turnRight();
      break;
    }
  }
}