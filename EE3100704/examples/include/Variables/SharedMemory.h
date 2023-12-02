//
// Created by percy on 23. 11. 12.
//

#ifndef EE3100704_PROJECTS_SHAREDMEMORY_H
#define EE3100704_PROJECTS_SHAREDMEMORY_H

#include "Variable.h"

typedef struct _SHM_
{
    // Initial Position
    double init_x=0.0;
    double init_y=0.0;
    double init_z=0.07;
    double init_w=1.0;
    double init_i=0.0;
    double init_j=0.0;
    double init_k=0.0;
    double init_angle_1=0.0872664; // FL_HIP_ROLL
    double init_angle_2=2.17643; // FL_HIP_PITCH
    double init_angle_3=-2.76635; // FL_KNEE_PITCH
    double init_angle_4=-0.0872664; // FR_HIP_ROLL
    double init_angle_5=2.1869; // FR_HIP_PITCH
    double init_angle_6=-2.75587; // FR_KNEE_PITCH
    double init_angle_7=0.0837758; // HL_HIP_ROLL
    double init_angle_8=2.17992; // HL_HIP_PITCH
    double init_angle_9=-2.73; // HL_KNEE_PITCH
    double init_angle_10=-0.0837758; // HR_HIP_ROLL
    double init_angle_11=2.18166; // HR_HIP_PITCH
    double init_angle_12=-2.73; // HR_KNEE_PITCH

    // Stand Position
    double test_x=0.0;
    double test_y=0.0;
    double test_z=0.32;
    double test_w=1.0;
    double test_i=0.0;
    double test_j=0.0;
    double test_k=0.0;
    double test_angle_1=-0.00523599;
    double test_angle_2=1.309;
    double test_angle_3=-2.35619;
    double test_angle_4=-0.00523599;
    double test_angle_5=1.32645;
    double test_angle_6=-2.35619;
    double test_angle_7=0.00698132;
    double test_angle_8=1.24791;
    double test_angle_9=-1.91986;
    double test_angle_10=0.0174533;
    double test_angle_11=1.24791;
    double test_angle_12=-1.91986;

    double test2_x=0.0;
    double test2_y=0.0;
    double test2_z=0.2;
    double test2_w=1.0;
    double test2_i=0.0;
    double test2_j=0.0;
    double test2_k=0.0;
    double test2_angle_1=-0.00523599;
    double test2_angle_2=1.309;
    double test2_angle_3=0.0;
    double test2_angle_4=-0.00523599;
    double test2_angle_5=1.32645;
    double test2_angle_6=0.0;
    double test2_angle_7=0.00698132;
    double test2_angle_8=1.24791;
    double test2_angle_9=0.0;
    double test2_angle_10=0.0174533;
    double test2_angle_11=1.24791;
    double test2_angle_12=0.0;

    double damping_x=0.0;
    double damping_y=0.0;
    double damping_z=0.2;
    double damping_w=1.0;
    double damping_i=0.0;
    double damping_j=0.0;
    double damping_k=0.0;
    double damping_angle_1=-0.00523599;
    double damping_angle_2=1.309;
    double damping_angle_3=-2.05619;
    double damping_angle_4=-0.00523599;
    double damping_angle_5=1.32645;
    double damping_angle_6=-2.05619;
    double damping_angle_7=0.00698132;
    double damping_angle_8=1.24791;
    double damping_angle_9=-1.91986;
    double damping_angle_10=0.0174533;
    double damping_angle_11=1.24791;
    double damping_angle_12=-1.91986;

    double duration;
} SHM, *pSHM;

#endif //EE3100704_PROJECTS_SHAREDMEMORY_H
