#include "PinChangeInterrupt.h"

#define max_speed 200

int outputA[2] = {2, 3};
int outputB[2] = {4, 5};

volatile int lastEncoded[2] = {0, 0}, sum[2] = {0, 0}, encoded[2] = {0, 0}, MSB[2] = {0, 0}, LSB[2] = {0, 0};
float encoderValue[2] = {0, 0};
float correction[2] = {130, -404};
/*
   ACW ----> Increment
   CW  ----> Decrement
*/
float PID[2] = {0, 0};
float error[2] = {0, 0};
float P[2] = {0, 0};
float I[2] = {0, 0};
float D[2] = {0, 0};
float last_error[2] = {0, 0};

int theta_2[50] = {465, 463, 462, 461, 459, 458, 456, 455, 466, 526, 640, 580, 517, 503, 502, 501, 501, 500, 499, 498, 498, 497, 496, 495, 494, 493, 492, 492, 491, 490, 489, 488, 487, 485, 484, 483, 482, 481, 480, 479, 478, 476, 475, 474, 473, 471, 470, 469, 467, 466};
//theta_1 - right
//theta_2 - left
int theta_1[50] = { -228, -228, -229, -230, -231, -231, -232, -233, -246, -310, -370, -256, -196, -185, -186, -188, -189, -190, -192, -193, -195, -196, -197, -199, -200, -201, -202, -204, -205, -206, -207, -209, -210, -211, -212, -213, -214, -215, -216, -217, -218, -219, -220, -221, -222, -223, -224, -225, -226, -227};
int set_position[2] = { -1080, 1080};

float Kp[2] = {3,3}, Kd[2] = {0,0}, Ki[2] = {0, 0};


uint8_t motor_pin[2] = {9, 11};
uint8_t dir_pin[2] = {10, 12};

long last_ms = 0, PID_dt = 100;
boolean flag = 0;

void setup() {
  for (int i = 0; i < 2; i++) {
    pinMode (outputA[i], INPUT_PULLUP);
    pinMode (outputB[i], INPUT_PULLUP);
  }
  attachPCINT(digitalPinToPCINT(outputA[0]), encode_0_0, CHANGE);
  attachPCINT(digitalPinToPCINT(outputB[0]), encode_0_0, CHANGE);
  attachPCINT(digitalPinToPCINT(outputA[1]), encode_0_1, CHANGE);
  attachPCINT(digitalPinToPCINT(outputB[1]), encode_0_1, CHANGE);

  Serial.begin (115200);
  init_motors();

//  theta_1[0] = 0;
//  theta_2[0] = 0;
//  theta_1[49] = 0;
//  theta_2[49] = 0;
//  for (int i = 1; i < 25; i++)
//  {
//    theta_1[i] = 0;
//    theta_1[i] = theta_1[i - 1] - 8;
//    theta_1[49 - i] = theta_1[i];
//    theta_2[i] = 0;
//    theta_2[i] = theta_2[i - 1] + 8;
//    theta_2[49 - i] = theta_2[i];
//  }
  last_ms = millis();
  set_position[0] = theta_1[0];
  set_position[1] = theta_2[0];

  while (!flag)
  {
    if (Serial.available() > 0)
    {
      if (Serial.read() == 's')
      {
        flag = !flag;
      }
    }
    print_position();
    cal_PID();
    motors(PID);
  }
  float wo[2] = {0, 0};
  motors(wo);
  last_ms = millis();
}


//140, -136

void loop() {
  if (flag == 1)
  {
    for (int i = 1; i < 50; i++)
    {
      last_ms = millis();
      set_position[0] = theta_1[i];
      set_position[1] = theta_2[i];
      while (millis() - last_ms < PID_dt)
      {
        print_position();
        cal_PID();
        motors(PID);
      }
    }
  }
}

void init_motors() { /*Intializes Motor Pins to OUTPUT*/
  for (int i = 0; i < 2; i++) { //Only for 1 Motor (i = 0)
    pinMode(motor_pin[i], OUTPUT);
    pinMode(dir_pin[i], OUTPUT);
  }
}

void print_position()
{
  for (int i  = 0 ; i < 2; i++)
  {
    Serial.print(encoderValue[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
}

void set_motor(uint8_t index, int motor_speed) {
  //if (index % 2 == 0)
  //motor_speed = -1 * motor_speed;
  if (abs(motor_speed) < 5)
    motor_speed = 0;
  if (motor_speed >= 0) {
    //Clockwise direction
    digitalWrite(dir_pin[index], HIGH);
    analogWrite(motor_pin[index], motor_speed);
  }
  else if (motor_speed < 0) {
    //Anti Clockwise direction
    digitalWrite(dir_pin[index], LOW);
    analogWrite(motor_pin[index], -1 * motor_speed);
  }
}

void motors(float motor_speed[2]) {
  for (int i = 0; i < 2; i++) {
    set_motor(i, motor_speed[i]);
  }
}

void encode_0_0() {
  MSB[0] = digitalRead(outputA[0]); //MSB = most significant bit
  LSB[0] = digitalRead(outputB[0]); //LSB = least significant bit

  encoded[0] = (MSB[0] << 1) | LSB[0]; //converting the 2 pin value to single number
  sum[0]  = (lastEncoded[0] << 2) | encoded[0]; //adding it to the previous encoded value

  if (sum[0] == 0b1101 || sum[0] == 0b0100 || sum[0] == 0b0010 || sum[0] == 0b1011) encoderValue[0] ++;
  if (sum[0] == 0b1110 || sum[0] == 0b0111 || sum[0] == 0b0001 || sum[0] == 0b1000) encoderValue[0] --;

  lastEncoded[0] = encoded[0]; //store this value for next time
}

void encode_0_1() {
  MSB[1] = digitalRead(outputA[1]); //MSB = most significant bit
  LSB[1] = digitalRead(outputB[1]); //LSB = least significant bit

  encoded[1] = (MSB[1] << 1) | LSB[1]; //converting the 2 pin value to single number
  sum[1]  = (lastEncoded[1] << 2) | encoded[1]; //adding it to the previous encoded value

  if (sum[1] == 0b1101 || sum[1] == 0b0100 || sum[1] == 0b0010 || sum[1] == 0b1011) encoderValue[1] ++;
  if (sum[1] == 0b1110 || sum[1] == 0b0111 || sum[1] == 0b0001 || sum[1] == 0b1000) encoderValue[1] --;

  lastEncoded[1] = encoded[1]; //store this value for next time
}

int sign(int i) {
  if (i >= 0)
    return 1;
  else
    return -1;
}

void cal_PID() {
  for (int i = 0; i < 2; i++) {
    error[i] = set_position[i] - encoderValue[i];
    error[i] = error[i] + correction[i];
    P[i] = error[i];
    I[i] += error[i];
    D[i] = error[i] - last_error[i];
    PID[i] = -1 * ((Kp[i] * P[i]) + (Ki[i] * I[i]) + (Kd[i] * D[i]));
    last_error[i] = error[i];
    if (abs(PID[i]) > max_speed)
      PID[i] = sign(PID[i]) * max_speed;
  }

}
