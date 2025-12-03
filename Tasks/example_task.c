#include "example_task.h"

int motor_id = 3;

PID speed;

// float initial_offset = 0;
int target_speed = 60;
//hahaha
// int actual_speed;
void example_task(void *arg)
{
  speed.kp = 7.5; // ??????
  speed.ki = 0.00855555;
  speed.kd = 6;
  speed.integral = 0;
  speed.last_err = 0;
  speed.integral_limit = 9000;
  speed.max_out = 1000;
  // actual_speed = CAN1_Motors[2].speed;
  while (1)
  {
    if (control.online)
    {
      float out = PID_calc(&speed, CAN1_Motors[3].speed, target_speed);
      // actual_speed = CAN1_Motors[2].speed;
      CAN1_MotorCurrents[motor_id] = (int16_t)out;
      CanMotor_SendCurrent(CAN1_MOTOR_1_TO_4);

      delay(1);
    }
    else
    {
      CAN1_MotorCurrents[motor_id] = 0;
      CanMotor_SendCurrent(CAN1_MOTOR_1_TO_4);
    }
  }
}