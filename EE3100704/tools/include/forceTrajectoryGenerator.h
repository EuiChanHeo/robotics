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
    double calcCoefficient(double T_start, double T_end, double T_constraint, double start_y, double currentTime, double end_y);
    double F_z,F_x;
    Eigen::Matrix <double, 4,1> Coefficient;
    Eigen::VectorXd coefficient = Eigen::VectorXd(5);

private:
    Eigen::MatrixXd mMatrixA = Eigen::MatrixXd(4, 4);
    Eigen::MatrixXd mCoefficient = Eigen::MatrixXd(4, 1);
    Eigen::MatrixXd mFunctionValue = Eigen::MatrixXd(4, 1);
    double mReferenceTime;
    double mTimeDuration;
};


#endif //EE3100704_PROJECTS_FORCETRAJECTORYGENERATOR_H
