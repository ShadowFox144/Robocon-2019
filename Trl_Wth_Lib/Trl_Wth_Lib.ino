#include "Walk.h"
#include "PinChangeInterrupt.h"

#define max_speed 0.6
#define max_speed 0.6
#define follow_speed 0.6
#define DELAY 50

uint8_t motor_pin[2] = {9, 11};
uint8_t dir_pin[2] = {10, 12};
uint8_t outputA[2] = {2, 3};
uint8_t outputB[2] = {4, 5};

int theta_1[50] = {503,502,502,501,500,500,499,498,497,496,496,495,494,493,492,491,490,489,488,487,486,485,484,483,482,480,479,478,477,476,474,473,472,471,469,468,467,465,464,463,461,460,459,457,456,454,467,492,509,509};
int theta_2[50] = { -184,-186,-187,-188,-190,-191,-193,-194,-195,-197,-198,-199,-201,-202,-203,-204,-206,-207,-208,-209,-210,-211,-213,-214,-215,-216,-217,-218,-219,-220,-221,-222,-223,-224,-225,-225,-226,-227,-228,-229,-230,-230,-231,-232,-232,-233,-239,-239,-222,-197};

float Kp[2] = {1, 1}, Kd[2] = {0, 0}, Ki[2] = {0, 0};

Leg L1(motor_pin,dir_pin,outputA,outputB);

void setup() {
  L1.set_points(theta_1,theta_2);
  for (int i = 0; i < 2; i++)
    L1.Mtr[i].set_Kpid(Kp[i],Ki[i],Kd[i]);
}

void loop() {
  L1.move_leg();
}
