//
// Created by percy on 23. 11. 12.
//

#ifndef EE3100704_PROJECTS_SHAREDMEMORY_H
#define EE3100704_PROJECTS_SHAREDMEMORY_H

#include "Variable.h"
#include <Eigen/Eigen>

typedef struct _SHM_
{
    // Initial Position
    double init_x=0.0;
    double init_y=0.0;
    double init_z=0.0655;
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

    double duration;
    double Force;
    Eigen::VectorXd* currentPosition = nullptr;
} SHM, *pSHM;

void initializeSHM(SHM& SharedMemory, int vectorSize);

void releaseSHM(SHM& SharedMemory);

#endif //EE3100704_PROJECTS_SHAREDMEMORY_H
