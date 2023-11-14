//
// Created by percy on 23. 11. 12.
//

#ifndef EE3100704_PROJECTS_SHAREDMEMORY_H
#define EE3100704_PROJECTS_SHAREDMEMORY_H

#include "Variable.h"

typedef struct _SHM_
{
    double init_x=0.0;
    double init_y=0.0;
    double init_z=0.075;
    double init_w=1.0;
    double init_i=0.0;
    double init_j=0.0;
    double init_k=0.0;
    double init_angle_1=0.0872664;
    double init_angle_2=2.17643;
    double init_angle_3=-2.76635;
    double init_angle_4=-0.0872664;
    double init_angle_5=2.1869;
    double init_angle_6=-2.75587;
    double init_angle_7=0.0837758;
    double init_angle_8=2.17992;
    double init_angle_9=-2.73;
    double init_angle_10=-0.0837758;
    double init_angle_11=2.18166;
    double init_angle_12=-2.73;

} SHM, *pSHM;

#endif //EE3100704_PROJECTS_SHAREDMEMORY_H
