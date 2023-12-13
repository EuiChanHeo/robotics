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

double forceTrajectoryGenerator::calcCoefficient(double T_start, double T_end, double T_constraint, double start_y, double currentTime, double end_y)
{

    Eigen::MatrixXd A(5, 5);
    Eigen::VectorXd B(5);

    A <<    pow(T_start,4), pow(T_start,3), pow(T_start,2), pow(T_start,1), 1,
            pow(T_end,4), pow(T_end,3), pow(T_end,2), pow(T_end,1), 1,
            4* pow(T_start,3), 3*pow(T_start,2), 2*pow(T_start,1), 1, 0,
            4* pow(T_end,3), 3*pow(T_end,2), 2*pow(T_end,1), 1, 0,
            4* pow(T_constraint,3), 3*pow(T_constraint,2), 2*pow(T_constraint,1), 1, 0;

    B << start_y, end_y, 0, 0, 0;

    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(A);
    Eigen::VectorXd X = lu_decomp.solve(B);

    coefficient(0) = X(0);
    coefficient(1) = X(1);
    coefficient(2) = X(2);
    coefficient(3) = X(3);
    coefficient(4) = X(4);

    F_z = coefficient(4) + coefficient(3)*currentTime + coefficient(2)*pow(currentTime,2) + coefficient(1)*pow(currentTime,3) + coefficient(0)*pow(currentTime,4);

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
    F_z = 0.0 + Coefficient(0)*currentTime + Coefficient(1)*pow(currentTime,2) + Coefficient(2)*pow(currentTime,3) + Coefficient(3)*pow(currentTime,4);

    return F_z;
}

