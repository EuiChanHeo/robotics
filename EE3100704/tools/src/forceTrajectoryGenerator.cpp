//
// Created by percy on 23. 12. 7.
//

#include "forceTrajectoryGenerator.h"

double forceTrajectoryGenerator::computeForce_jump(double currentTime)
{
    Coefficient(0) = 45.08;
    Coefficient(1) = -11.27;

    F_z = 45.08*currentTime + Coefficient(1)*pow(currentTime,2);

    return F_z;
}

double forceTrajectoryGenerator::computeForce_jump_third(double currentTime)
{
    Coefficient(1) = -90;

    F_z = 45.08 + 200.0 + Coefficient(1)*pow(currentTime,1) ;

    return F_z;

}
double forceTrajectoryGenerator::computeForce_z(double currentTime)
{
    Coefficient(0) = 0;
    Coefficient(1) = 101.43;
    Coefficient(2) = -78.89;
    Coefficient(3) = 16.905;
//    Coefficient(0) = 0;
//    Coefficient(1) = 10.125;
//    Coefficient(2) = -47.25;
//    Coefficient(3) = 10.125;
    F_z = 0.0 + Coefficient(0)*currentTime + Coefficient(1)*pow(currentTime,2) + Coefficient(2)*pow(currentTime,3) + Coefficient(3)*pow(currentTime,4);

    return F_z;
}

