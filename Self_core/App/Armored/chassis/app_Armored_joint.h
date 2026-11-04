//
// Created by 33140 on 2026/10/31.
//

#ifndef BNGU_CONTROL_SYSTEM_APP_ARMORED_JOINT_H
#define BNGU_CONTROL_SYSTEM_APP_ARMORED_JOINT_H

#include "drv_motor.h"
#include "lib_pid.h"

typedef struct {
    drv_motor_dm_t dm_motor;
    lib_pid_t Height_ctrl;
    lib_pid_t pitch_ctrl;
    lib_pid_t roll_ctrl;
}dm_joint_motor;



#endif // BNGU_CONTROL_SYSTEM_APP_ARMORED_JOINT_H
