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

    // Create matrices and vectors for the system of equations
    Eigen::MatrixXd A(5, 5);
    Eigen::VectorXd B(5);

    std::cout << "T_start : " << T_start << std::endl;
    std::cout << "T_end : " << T_end << std::endl;
    std::cout << "T_constraint : " << T_constraint << std::endl;
    std::cout << "start_y : " << start_y << std::endl;

    // Fill the matrices and vectors with coefficients from the equations
    A <<    pow(T_start,4), pow(T_start,3), pow(T_start,2), pow(T_start,1), 1,
            pow(T_end,4), pow(T_end,3), pow(T_end,2), pow(T_end,1), 1,
            4* pow(T_start,3), 3*pow(T_start,2), 2*pow(T_start,1), 1, 0,
            4* pow(T_end,3), 3*pow(T_end,2), 2*pow(T_end,1), 1, 0,
            4* pow(T_constraint,3), 3*pow(T_constraint,2), 2*pow(T_constraint,1), 1, 0;

    B << start_y, end_y, 0, 0, 0;

    std::cout << A << std::endl;
    std::cout << B << std::endl;

    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(A);

    if (!lu_decomp.isInvertible()) {
        std::cout << "Matrix A doesn't have a inverse Matrix" << std::endl;
    }
    else
    {
        std::cout << "Matrix A has a inverse Matrix" << std::endl;
    }

    Eigen::VectorXd X = lu_decomp.solve(B);

//    Eigen::VectorXd X = A.fullPivLu().solve(B);

    std::cout << "X : " << X << std::endl;
    coefficient(0) = X(0);
    coefficient(1) = X(1);
    coefficient(2) = X(2);
    coefficient(3) = X(3);
    coefficient(4) = X(4);

    // Output the coefficients
    std::cout << "coefficient_0 : " << coefficient(0) << std::endl;
    std::cout << "coefficient_1 : " << coefficient(1) << std::endl;
    std::cout << "coefficient_2 : " << coefficient(2) << std::endl;
    std::cout << "coefficient_3 : " << coefficient(3) << std::endl;
    std::cout << "coefficient_4 : " << coefficient(4) << std::endl;

    F_z = coefficient(4) + coefficient(3)*currentTime + coefficient(2)*pow(currentTime,2) + coefficient(1)*pow(currentTime,3) + coefficient(0)*pow(currentTime,4);

    std::cout << F_z << std::endl;

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

