#include "PinChangeInterrupt.h"

#define max_speed 200

int outputA[2] = {A14, A10};
int outputB[2] = {A15, A11};

volatile int lastEncoded[2] = {0, 0}, sum[2] = {0, 0}, encoded[2] = {0, 0}, MSB[2] = {0, 0}, LSB[2] = {0, 0};
float encoderValue[2] = {0, 0};
float PID[2] = {0, 0};
float error[2] = {0, 0};
float P[2] = {0, 0};
float I[2] = {0, 0};
float D[2] = {0, 0};
float last_error[2] = {0, 0};

int set_position[2] = {6600, 6600};

float Kp[2] = {1, 1}, Kd[2] = {0, 0}, Ki[2] = {0, 0};


uint8_t motor_pin[2] = {9, 13};
uint8_t dir_pin[2] = {26, 30};

long last_ms = 0, PID_dt = 5000;
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
}

void loop() {
  set_position[0] = 6600;
  set_position[1] = 6600;
  last_ms = millis();
  Serial.println("DOWN");
  while (1)
  {
    print_position();
    cal_error();
    if (abs(error[0]) < 100)
    {
      break;
    }
    cal_PID();
    float w[2] = {PID[0] - PID[1], PID[0] + PID[1]};

    print_position();
    motors(w);
  }
  Serial.println("UP");
  set_position[0] = 0;
  set_position[1] = 0;
  last_ms = millis();
  while (1)
  {
    //print_position();
    cal_error();
    if (abs(error[0]) < 100)
    {
      break;
    }
    cal_PID();
    float w[2] = {PID[0] - PID[1], PID[0] + PID[1]};
    print_position();
    motors(w);
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
    Serial.print(error[i]);
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

void cal_error()
{
  error[0] = ((set_position[0] - encoderValue[0]) + (set_position[1] - encoderValue[1])) / 2;
  error[1] = (encoderValue[0] - encoderValue[1]);
}

void cal_PID() {
  for (int i = 0; i < 2; i++) {
    P[i] = error[i];
    I[i] += error[i];
    D[i] = error[i] - last_error[i];
    PID[i] = ((Kp[i] * P[i]) + (Ki[i] * I[i]) + (Kd[i] * D[i]));
    last_error[i] = error[i];

    if (abs(error[i]) < 10)
    {
      PID[i] = 0;
    }
    if (abs(PID[i]) > max_speed)
      PID[i] = sign(PID[i]) * max_speed;
  }
}
