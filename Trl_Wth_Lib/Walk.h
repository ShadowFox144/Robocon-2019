/*
  Walk.h - Library for storing walk funtions
*/
#ifndef Walk_h
#define Walk_h
#include "PinChangeInterrupt.h"

#include "Arduino.h"

// class DC_Motor{
// public:
// };

// class Encoder{
//  public:
// };

class Motor{//} : public Encoder, public DC_Motor{
public:
  //DC_Motor
  uint8_t Motor_Pin;
  uint8_t Dir_Pin;
  void DC_Motor(uint8_t, uint8_t);
  void motor_init();
  void set_motor_speed(int);
  //Encoder
  uint8_t Pin_A;
  uint8_t Pin_B;
  volatile int encoded;
  volatile int last_encoded;
  volatile int MSB;
  volatile int LSB;
  volatile int sum;
  int encoderValue;
  //Functions:
  void Encoder(uint8_t , uint8_t );
  void Encoder_init();
  void encode_A();
  void encode_B();
  void print_position();
  //Motor
  float P;
  float I;
  float D;
  float Kp;
  float Kd;
  float Ki;
  float error;
  float last_error;
  //Variables
  float set_position;
  float PID;
  //Functions
  void Motor_1(uint8_t, uint8_t, uint8_t, uint8_t );
  void set_Kpid(float , float , float );
  int sign(int i);
  void calc_PID();
  void motors();
};

class Leg{
public:
  int theta_1_V[50];
  int theta_2_V[50];
  long last_ms;
  long PID_dt;
  Motor Mtr[2];
  Leg(uint8_t [], uint8_t [],uint8_t [], uint8_t []);
  void set_points(int [], int []);
  void move_leg();
  void print_position_encoders();
  void cal_PID();
  void setAllMotors();
};
#endif
