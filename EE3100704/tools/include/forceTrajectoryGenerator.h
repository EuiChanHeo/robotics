//
// Created by percy on 23. 12. 7.
//

#ifndef EE3100704_PROJECTS_FORCETRAJECTORYGENERATOR_H
#define EE3100704_PROJECTS_FORCETRAJECTORYGENERATOR_H

#include <iostream>
#include <Eigen/Eigen>
#include <cmath>

class forceTrajectoryGenerator {
public:
    double computeForce_jump(double currentTime);
    double computeForce_z(double currentTime);
    double computeForce_jump_third(double currentTime);

    double F_z,F_x;
    Eigen::Matrix <double, 4,1> Coefficient;

private:
    Eigen::MatrixXd mMatrixA = Eigen::MatrixXd(4, 4);
    Eigen::MatrixXd mCoefficient = Eigen::MatrixXd(4, 1);
    Eigen::MatrixXd mFunctionValue = Eigen::MatrixXd(4, 1);
    double mReferenceTime;
    double mTimeDuration;
};


#endif //EE3100704_PROJECTS_FORCETRAJECTORYGENERATOR_H
