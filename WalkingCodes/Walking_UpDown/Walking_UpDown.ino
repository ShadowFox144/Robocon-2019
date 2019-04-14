#include "PinChangeInterrupt.h"

#define max_speed 200

/*
   [x][y]
   x --> index of leg
   y --> index of motor of leg x
*/

int outputA[4][2] = {{A13, A10}, {51, A15}, {11, 12}, {50, A8}};
int outputB[4][2] = {{A12, A11}, {53, A14}, {13, 10}, {52, A9}};

volatile int lastEncoded[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}}, sum[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}}, encoded[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}}, MSB[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}}, LSB[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
float encoderValue[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
float correction[4][2] = {{ -138, 138}, { -138, 138}, { -138, 138}, { -138, 138}};
/*
   ACW ----> Increment
   CW  ----> Decrement
*/
float PID[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
float error[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
float P[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
float I[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
float D[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
float last_error[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};

//int theta_2[48] = { -2675, -2436, -2356, -2277, -2197, -2118, -2039, -1959, -1880, -1801, -1722, -1644, -1565, -1487, -1409, -1332, -1255, -1178, -1102, -1027, -952, -878, -804, -732, -660, -589, -520, -451, -383, -317, -251, -187, -125, -63, -3, 54, 111, 167, 221, 274, 325, 374, 422, 468, 845, 836, -495, -2049};
////theta_1 - right
////theta_2 - left
//int theta_1[48] = {468, 422, 374, 325, 274, 221, 167, 111, 54, -3, -63, -125, -187, -251, -317, -383, -451, -520, -589, -660, -732, -804, -878, -952, -1027, -1102, -1178, -1255, -1332, -1409, -1487, -1565, -1644, -1722, -1801, -1880, -1959, -2039, -2118, -2197, -2277, -2356, -2595, -2675, -2049, -495, 836, 845};


int theta_2[48] = { -2675, -2595, -2356, -2277, -2197, -2118, -2039, -1959, -1880, -1801, -1722, -1644, -1565, -1487, -1409, -1332, -1255, -1178, -1102, -1027, -952, -878, -804, -732, -660, -589, -520, -451, -383, -317, -251, -187, -125, -63, -3, 54, 111, 167, 221, 274, 325, 374, 422, 468, 1259, 1891, 288, -1852};
//theta_1 - right
//theta_2 - left
int theta_1[48] = {468, 422, 374, 325, 274, 221, 167, 111, 54, -3, -63, -125, -187, -251, -317, -383, -451, -520, -589, -660, -732, -804, -878, -952, -1027, -1102, -1178, -1255, -1332, -1409, -1487, -1565, -1644, -1722, -1801, -1880, -1959, -2039, -2118, -2197, -2277, -2516, -2595, -2675, -1852, 288, 1891, 1259};

int set_position[4][2] = {{ 0, 0}, { 0, 0}, { 0, 0}, { 0, 0}};

float Kp[4][2] = {{1, 1}, {1, 1}, {1, 1}, {1, 1}}, Kd[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}}, Ki[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};


uint8_t motor_pin[4][2] = {{6, 4}, {7, 8}, {3, 5}, {2, 9}};
uint8_t dir_pin[4][2] = {{32, 26}, {40, 48}, {A2, A4}, {A5, A0}};

long last_ms = 0, PID_dt = 100;
boolean flag = 0;

void setup() {
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 2; j++) {
      pinMode (outputA[i][j], INPUT_PULLUP);
      pinMode (outputB[i][j], INPUT_PULLUP);
      //Kp[i][j] = 0;
    }
  }
  attachPCINT(digitalPinToPCINT(outputA[0][0]), encode_0_0, CHANGE);
  attachPCINT(digitalPinToPCINT(outputB[0][0]), encode_0_0, CHANGE);
  attachPCINT(digitalPinToPCINT(outputA[0][1]), encode_0_1, CHANGE);
  attachPCINT(digitalPinToPCINT(outputB[0][1]), encode_0_1, CHANGE);
  attachPCINT(digitalPinToPCINT(outputA[1][0]), encode_1_0, CHANGE);
  attachPCINT(digitalPinToPCINT(outputB[1][0]), encode_1_0, CHANGE);
  attachPCINT(digitalPinToPCINT(outputA[1][1]), encode_1_1, CHANGE);
  attachPCINT(digitalPinToPCINT(outputB[1][1]), encode_1_1, CHANGE);

  attachPCINT(digitalPinToPCINT(outputA[2][0]), encode_2_0, CHANGE);
  attachPCINT(digitalPinToPCINT(outputB[2][0]), encode_2_0, CHANGE);
  attachPCINT(digitalPinToPCINT(outputA[2][1]), encode_2_1, CHANGE);
  attachPCINT(digitalPinToPCINT(outputB[2][1]), encode_2_1, CHANGE);
  attachPCINT(digitalPinToPCINT(outputA[3][0]), encode_3_0, CHANGE);
  attachPCINT(digitalPinToPCINT(outputB[3][0]), encode_3_0, CHANGE);
  attachPCINT(digitalPinToPCINT(outputA[3][1]), encode_3_1, CHANGE);
  attachPCINT(digitalPinToPCINT(outputB[3][1]), encode_3_1, CHANGE);

  Serial.begin (115200);
  init_motors();
  set_position[0][0] = theta_1[0];
  set_position[0][1] = theta_2[0];
  set_position[1][0] = theta_1[23];
  set_position[1][1] = theta_2[23];
  set_position[2][0] = theta_1[35];
  set_position[2][1] = theta_2[35];
  set_position[3][0] = theta_1[47];
  set_position[3][1] = theta_2[47];

  last_ms = millis();
  long del = 5000;
  while (millis() - last_ms < del)
  {
    cal_PID();
    motors(PID);
  }


}

void loop()
{
  for (int i = 0; i < 48; i++)
  {

    last_ms = millis();
    set_position[0][0] = theta_1[i];
    set_position[0][1] = theta_2[i];
    if (i < 12)
    {
      set_position[0][0] = theta_1[i];
      set_position[0][1] = theta_2[i];
      set_position[1][0] = theta_1[i + 12];
      set_position[1][1] = theta_2[i + 12];
      set_position[2][0] = theta_1[i + 24];
      set_position[2][1] = theta_2[i + 24];
      set_position[3][0] = theta_1[i + 36];
      set_position[3][1] = theta_2[i + 36];
    }
    else if (i < 24)
    {
      set_position[0][0] = theta_1[i];
      set_position[0][1] = theta_2[i];
      set_position[1][0] = theta_1[i + 12];
      set_position[1][1] = theta_2[i + 12];
      set_position[2][0] = theta_1[i + 24];
      set_position[2][1] = theta_2[i + 24];
      set_position[3][0] = theta_1[i - 12];
      set_position[3][1] = theta_2[i - 12];
    }
    else if (i < 36)
    {
      set_position[0][0] = theta_1[i];
      set_position[0][1] = theta_2[i];
      set_position[1][0] = theta_1[i + 12];
      set_position[1][1] = theta_2[i + 12];
      set_position[2][0] = theta_1[i - 24];
      set_position[2][1] = theta_2[i - 24];
      set_position[3][0] = theta_1[i - 12];
      set_position[3][1] = theta_2[i - 12];
    }
    else
    {
      set_position[0][0] = theta_1[i];
      set_position[0][1] = theta_2[i];
      set_position[1][0] = theta_1[i - 36];
      set_position[1][1] = theta_2[i - 36];
      set_position[2][0] = theta_1[i - 24];
      set_position[2][1] = theta_2[i - 24];
      set_position[3][0] = theta_1[i - 12];
      set_position[3][1] = theta_2[i - 12];
    }
    while (millis() - last_ms < PID_dt)
    {
      print_position();
      cal_PID();
      motors(PID);
    }
  }
}

void init_motors() { /*Intializes Motor Pins to OUTPUT*/
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      pinMode(motor_pin[i][j], OUTPUT);
      pinMode(dir_pin[i][j], OUTPUT);
    }
  }
}

void print_position()
{
  for (int i  = 0 ; i < 4; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      Serial.print(encoderValue[i][j]);
      Serial.print(" ");
    }
  }
  Serial.println(" ");
}

void set_motor(uint8_t index_1, uint8_t index_2, int motor_speed) {
  //if (index % 2 == 0)
  //motor_speed = -1 * motor_speed;
  if (abs(motor_speed) < 5)
    motor_speed = 0;
  if (motor_speed >= 0) {
    //Clockwise direction
    digitalWrite(dir_pin[index_1][index_2], HIGH);
    analogWrite(motor_pin[index_1][index_2], motor_speed);
  }
  else if (motor_speed < 0) {
    //Anti Clockwise direction
    digitalWrite(dir_pin[index_1][index_2], LOW);
    analogWrite(motor_pin[index_1][index_2], -1 * motor_speed);
  }
}

void motors(float motor_speed[4][2]) {
  for (int j = 0; j < 2; j++)
  {
    set_motor(0, j, motor_speed[0][j]);
    set_motor(1, j, -1 * motor_speed[1][j]);
    set_motor(2, j, motor_speed[2][j]);
    set_motor(3, j, -1 * motor_speed[3][j]);
  }
}

void encode_0_0() {
  noInterrupts();
  MSB[0][0] = digitalRead(outputA[0][0]); //MSB = most significant bit
  LSB[0][0] = digitalRead(outputB[0][0]); //LSB = least significant bit

  // Serial.println("00");


  encoded[0][0] = (MSB[0][0] << 1) | LSB[0][0]; //converting the 2 pin value to single number
  sum[0][0]  = (lastEncoded[0][0] << 2) | encoded[0][0]; //adding it to the previous encoded value

  if (sum[0][0] == 0b1101 || sum[0][0] == 0b0100 || sum[0][0] == 0b0010 || sum[0][0] == 0b1011) encoderValue[0][0] ++;
  if (sum[0][0] == 0b1110 || sum[0][0] == 0b0111 || sum[0][0] == 0b0001 || sum[0][0] == 0b1000) encoderValue[0][0] --;

  lastEncoded[0][0] = encoded[0][0]; //store this value for next tim
  interrupts();
}

void encode_0_1() {
  MSB[0][1] = digitalRead(outputA[0][1]); //MSB = most significant bit
  LSB[0][1] = digitalRead(outputB[0][1]); //LSB = least significant bit

  encoded[0][1] = (MSB[0][1] << 1) | LSB[0][1]; //converting the 2 pin value to single number
  sum[0][1]  = (lastEncoded[0][1] << 2) | encoded[0][1]; //adding it to the previous encoded value

  if (sum[0][1] == 0b1101 || sum[0][1] == 0b0100 || sum[0][1] == 0b0010 || sum[0][1] == 0b1011) encoderValue[0][1] ++;
  if (sum[0][1] == 0b1110 || sum[0][1] == 0b0111 || sum[0][1] == 0b0001 || sum[0][1] == 0b1000) encoderValue[0][1] --;

  lastEncoded[0][1] = encoded[0][1]; //store this value for next time
}
void encode_1_0() {
  noInterrupts();
  MSB[1][0] = digitalRead(outputA[1][0]); //MSB = most significant bit
  LSB[1][0] = digitalRead(outputB[1][0]); //LSB = least significant bit

  encoded[1][0] = (MSB[1][0] << 1) | LSB[1][0]; //converting the 2 pin value to single number
  sum[1][0]  = (lastEncoded[1][0] << 2) | encoded[1][0]; //adding it to the previous encoded value

  if (sum[1][0] == 0b1101 || sum[1][0] == 0b0100 || sum[1][0] == 0b0010 || sum[1][0] == 0b1011) encoderValue[1][0] ++;
  if (sum[1][0] == 0b1110 || sum[1][0] == 0b0111 || sum[1][0] == 0b0001 || sum[1][0] == 0b1000) encoderValue[1][0] --;

  lastEncoded[1][0] = encoded[1][0]; //store this value for next time
  interrupts();
}

void encode_1_1() {
  MSB[1][1] = digitalRead(outputA[1][1]); //MSB = most significant bit
  LSB[1][1] = digitalRead(outputB[1][1]); //LSB = least significant bit

  encoded[1][1] = (MSB[1][1] << 1) | LSB[1][1]; //converting the 2 pin value to single number
  sum[1][1]  = (lastEncoded[1][1] << 2) | encoded[1][1]; //adding it to the previous encoded value

  if (sum[1][1] == 0b1101 || sum[1][1] == 0b0100 || sum[1][1] == 0b0010 || sum[1][1] == 0b1011) encoderValue[1][1] ++;
  if (sum[1][1] == 0b1110 || sum[1][1] == 0b0111 || sum[1][1] == 0b0001 || sum[1][1] == 0b1000) encoderValue[1][1] --;

  lastEncoded[1][1] = encoded[1][1]; //store this value for next time
}

void encode_2_0() {
  MSB[2][0] = digitalRead(outputA[2][0]); //MSB = most significant bit
  LSB[2][0] = digitalRead(outputB[2][0]); //LSB = least significant bit

  encoded[2][0] = (MSB[2][0] << 1) | LSB[2][0]; //converting the 2 pin value to single number
  sum[2][0]  = (lastEncoded[2][0] << 2) | encoded[2][0]; //adding it to the previous encoded value

  if (sum[2][0] == 0b1101 || sum[2][0] == 0b0100 || sum[2][0] == 0b0010 || sum[2][0] == 0b1011) encoderValue[2][0] ++;
  if (sum[2][0] == 0b1110 || sum[2][0] == 0b0111 || sum[2][0] == 0b0001 || sum[2][0] == 0b1000) encoderValue[2][0] --;

  lastEncoded[2][0] = encoded[2][0]; //store this value for next time
}

void encode_2_1() {
  MSB[2][1] = digitalRead(outputA[2][1]); //MSB = most significant bit
  LSB[2][1] = digitalRead(outputB[2][1]); //LSB = least significant bit

  encoded[2][1] = (MSB[2][1] << 1) | LSB[2][1]; //converting the 2 pin value to single number
  sum[2][1]  = (lastEncoded[2][1] << 2) | encoded[2][1]; //adding it to the previous encoded value

  if (sum[2][1] == 0b1101 || sum[2][1] == 0b0100 || sum[2][1] == 0b0010 || sum[2][1] == 0b1011) encoderValue[2][1] ++;
  if (sum[2][1] == 0b1110 || sum[2][1] == 0b0111 || sum[2][1] == 0b0001 || sum[2][1] == 0b1000) encoderValue[2][1] --;

  lastEncoded[2][1] = encoded[2][1]; //store this value for next time
}

void encode_3_0() {
  MSB[3][0] = digitalRead(outputA[3][0]); //MSB = most significant bit
  LSB[3][0] = digitalRead(outputB[3][0]); //LSB = least significant bit

  encoded[3][0] = (MSB[3][0] << 1) | LSB[3][0]; //converting the 2 pin value to single number
  sum[3][0]  = (lastEncoded[3][0] << 2) | encoded[3][0]; //adding it to the previous encoded value

  if (sum[3][0] == 0b1101 || sum[3][0] == 0b0100 || sum[3][0] == 0b0010 || sum[3][0] == 0b1011) encoderValue[3][0] ++;
  if (sum[3][0] == 0b1110 || sum[3][0] == 0b0111 || sum[3][0] == 0b0001 || sum[3][0] == 0b1000) encoderValue[3][0] --;

  lastEncoded[3][0] = encoded[3][0]; //store this value for next time
}

void encode_3_1() {
  MSB[3][1] = digitalRead(outputA[3][1]); //MSB = most significant bit
  LSB[3][1] = digitalRead(outputB[3][1]); //LSB = least significant bit

  encoded[3][1] = (MSB[3][1] << 1) | LSB[3][1]; //converting the 2 pin value to single number
  sum[3][1]  = (lastEncoded[3][1] << 2) | encoded[3][1]; //adding it to the previous encoded value

  if (sum[3][1] == 0b1101 || sum[3][1] == 0b0100 || sum[3][1] == 0b0010 || sum[3][1] == 0b1011) encoderValue[3][1] ++;
  if (sum[3][1] == 0b1110 || sum[3][1] == 0b0111 || sum[3][1] == 0b0001 || sum[3][1] == 0b1000) encoderValue[3][1] --;

  lastEncoded[3][1] = encoded[3][1]; //store this value for next time
}


int sign(int i) {
  if (i >= 0)
    return 1;
  else
    return -1;
}

void cal_PID()
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      error[i][j] = set_position[i][j] - encoderValue[i][j];
      //error[i][j] = error[i][j] - correction[i][j];
      P[i][j] = error[i][j];
      I[i][j] += error[i][j];
      D[i][j] = error[i][j] - last_error[i][j];
      PID[i][j] = ((Kp[i][j] * P[i][j]) + (Ki[i][j] * I[i][j]) + (Kd[i][j] * D[i][j]));
      if (i == 3)
      {
        PID[i][j] = -1 * PID[i][j];
      }
      last_error[i][j] = error[i][j];
      if (abs(PID[i][j]) > max_speed)
        PID[i][j] = sign(PID[i][j]) * max_speed;
    }
  }
}
