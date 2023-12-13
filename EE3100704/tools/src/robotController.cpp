//
// Created by tina on 23. 11. 7.
//

#include "robotController.hpp"

extern SHM sharedMemory;

void robotController::setInitialState(raisim::ArticulatedSystem *robot, Eigen::VectorXd initialPosition)
{
    robot->setGeneralizedCoordinate(initialPosition);
}

void robotController::setPDgain(Eigen::VectorXd Pgain, Eigen::VectorXd Dgain)
{
    mPgain = Pgain;
    mDgain = Dgain;
}

void robotController::Dynamics_Stand(raisim::World *world, raisim::ArticulatedSystem *robot)
{

}
void robotController::setFold_2(raisim::World *world, raisim::ArticulatedSystem *robot)
{
    setTime setTime;
    float timeDuration = 1.0;
    cubicTrajectoryGenerator trajectoryGenerator[robot->getGeneralizedCoordinateDim()];

    setTime.setTimeInitiallize();
    setTime.timedT = 0.001;

    Eigen::VectorXd goalPosition(robot->getGeneralizedCoordinateDim());
    goalPosition << 0, 0, 0.07, 1, 0, 0, 0, 0.0872664, 1.5708, -2.76635, -0.0872664, 1.5708, -2.75587, 0.0837758, 1.5708, -2.73, -0.0837758, 1.5708, -2.73;

    Eigen::VectorXd goalVelocity(robot -> getDOF());
    goalVelocity.setZero();
    Eigen::VectorXd jointPositionTarget(robot->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointVelocityTarget(robot->getDOF());
    jointVelocityTarget.setZero();
    Eigen::VectorXd currentPosition(robot->getGeneralizedCoordinateDim()); // 19
    Eigen::VectorXd currentVelocity;
    Eigen::VectorXd error(12);
    error.setZero();
    Eigen::VectorXd velocity_error(12);
    velocity_error.setZero();
    Eigen::VectorXd torque(12);
    Eigen::VectorXd test_torque(18);
    robot -> setPdGains(mPgain, mDgain);

    robot -> getState(currentPosition, currentVelocity);

    for (int i = 0; i < robot->getGeneralizedCoordinateDim(); i++)
    {
        trajectoryGenerator[i].updateTrajectory(currentPosition[i],goalPosition[i],setTime.localtime,timeDuration) ;
    }

    while(true)
    {
        setTime.setLocaltime(); //get in while loop.
        robot -> getState(currentPosition, currentVelocity);

        for (int jointNum = 0; jointNum < robot->getGeneralizedCoordinateDim(); jointNum++)
        {
            jointPositionTarget[jointNum] = trajectoryGenerator[jointNum].getPositionTrajectory(setTime.localtime);
        }

        for(int jointNum = 0; jointNum < robot->getDOF(); jointNum++)
        {
            jointVelocityTarget[jointNum] = trajectoryGenerator[jointNum].getVelocityTrajectory(setTime.localtime);
        }
        error = jointPositionTarget.tail(12) - currentPosition.tail(12);
        velocity_error = jointVelocityTarget.tail(12) - currentVelocity.tail(12);

        torque.tail(12) = mPgain.tail(12).cwiseProduct(error) + mDgain.tail(12).cwiseProduct(velocity_error);
        test_torque.tail(12) = torque;
        test_torque.head(6)  << 0, 0, 0, 0, 0, 0;

        robot -> setGeneralizedForce(test_torque);
        world -> integrate();
        usleep(1000);

        if (setTime.localtime == timeDuration)
        {
            std::cout << "end!" << std::endl;
            break;
        }

    }
}

void robotController::setFold(raisim::World *world, raisim::ArticulatedSystem *robot)
{
    world->setGravity({0,0,-9.8});
    cubicTrajectoryGenerator trajectoryGenerator[robot->getGeneralizedCoordinateDim()];

    setTime setTime;
    float timeDuration = 1.0;

    /// set time
    setTime.setTimeInitiallize();
    setTime.timedT = 0.001;

    Eigen::VectorXd goalPosition(robot->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointPositionTarget(robot->getGeneralizedCoordinateDim()), jointVelocityTarget(robot->getDOF());
    Eigen::VectorXd currentPosition(robot->getGeneralizedCoordinateDim());
    jointVelocityTarget.setZero();

    for (int i = 0; i < robot->getGeneralizedCoordinateDim(); i++)
    {
        currentPosition(i) = robot->getGeneralizedCoordinate()[i] ;
    }

    // set joint fold position
    goalPosition << 0, 0, 0.07, 1, 0, 0, 0, 0.0872664, 1.5708, -2.76635, -0.0872664, 1.5708, -2.75587, 0.0837758, 1.5708, -2.73, -0.0837758, 1.5708, -2.73;

    // create trajectory
    for (int i = 0; i < robot->getGeneralizedCoordinateDim(); i++)
    {
        trajectoryGenerator[i].updateTrajectory(currentPosition[i],goalPosition[i],setTime.localtime,timeDuration) ;
    }

    while (1)
    {
        setTime.setLocaltime(); //get in while loop.
        for (int jointNum = 0; jointNum < robot->getGeneralizedCoordinateDim() ; jointNum++)
        {
            jointPositionTarget[jointNum] = trajectoryGenerator[jointNum].getPositionTrajectory(setTime.localtime);
        }

        /// robot set stand position
//        robot->setGeneralizedCoordinate(jointPositionTarget);
        robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
        robot->setPdGains(mPgain, mDgain);
        robot->setPdTarget(jointPositionTarget, jointVelocityTarget);
        world->integrate();
        usleep(1000);
        if (setTime.localtime == timeDuration)
            break;
    }
    robot->setPdTarget(jointPositionTarget, jointVelocityTarget);
}

void robotController::Force_stand(raisim::World *world, raisim::ArticulatedSystem *robot)
{
    setTime setTime;
    setTime.setTimeInitiallize();
    setTime.timedT = 0.001;

    forceTrajectoryGenerator ForceTrajectory;

    float timeDuration = 4.0;
    int iteration;
    int check_iteration = 0;
    double L_1 = 0.2305, L_2 = 0.228;


    Eigen::VectorXd desiredJointPosition(12);
    Eigen::VectorXd currentPosition(19);
    
    Eigen::VectorXd currentVelocity(robot->getDOF());

    Eigen::VectorXd Jacobian(12);
    Eigen::VectorXd torque(12);
    Eigen::VectorXd test_torque(18);
    Eigen::VectorXd getForce(18);
    Eigen::VectorXd zeroTorque(18);
    zeroTorque.setZero();
    getForce.setZero();

    robot -> setPdGains(mPgain, mDgain);
    robot -> getState(currentPosition, currentVelocity);

    while(true)
    {
        check_iteration ++;
        setTime.setLocaltime();
        robot -> getState(currentPosition, currentVelocity);

        sharedMemory.Force = ForceTrajectory.calcCoefficient(0.0, 4.0, 3.5, 0.0,setTime.localtime, 45.08);

        for(iteration = 0; iteration < 12;)
        {
            Jacobian(iteration) = 0.0;
            Jacobian(iteration + 1) = (- L_1 * sin(currentPosition[iteration+8]) + L_2 * cos(currentPosition[iteration+8] - currentPosition[iteration+9] - 0.785398));
            Jacobian(iteration + 2) = 4*(- L_2 * cos(currentPosition[iteration+8] - currentPosition[iteration+9] - 0.785398));
            iteration += 3;
        }

        torque = Jacobian * sharedMemory.Force * 2.0 ;

        test_torque.tail(12) = torque;
        test_torque.head(6)  << 0, 0, 0, 0, 0, 0;

        robot -> setGeneralizedForce(test_torque);
        world -> integrate();

        usleep(1000);

        if (setTime.localtime == timeDuration)
        {
            std::cout << "end!" << std::endl;
            std::cout << torque << std::endl;
            break;
        }
    }

    timeDuration += 2.0;

    while(true)
    {
        setTime.setLocaltime();
        robot -> getState(currentPosition, currentVelocity);

        sharedMemory.Force = ForceTrajectory.calcCoefficient(4.0, 6.0, 5.5, 45.08,setTime.localtime, 5.0);

        for(iteration = 0; iteration < 12;)
        {
            Jacobian(iteration) = 0.0;
            Jacobian(iteration + 1) = (- L_1 * sin(currentPosition[iteration+8]) + L_2 * cos(currentPosition[iteration+8] - currentPosition[iteration+9] - 0.785398));
            Jacobian(iteration + 2) = 4.0*(- L_2 * cos(currentPosition[iteration+8] - currentPosition[iteration+9] - 0.785398));
            iteration += 3;
        }

        torque = Jacobian * sharedMemory.Force * 2.0;

        test_torque.tail(12) = torque;
        test_torque.head(6)  << 0, 0, 0, 0, 0, 0;

        robot -> setGeneralizedForce(test_torque);
        world -> integrate();

        usleep(1000);

        if (setTime.localtime == timeDuration)
        {
            std::cout << "end!" << std::endl;
            break;
        }
    }

    timeDuration += 0.5;

    while(true)
    {
        setTime.setLocaltime();
        robot -> getState(currentPosition, currentVelocity);

        sharedMemory.Force = ForceTrajectory.calcCoefficient(6.0, 6.5, 6.375, 5.0,setTime.localtime, 750.0);

        for(iteration = 0; iteration < 12;)
        {
            Jacobian(iteration) = 0.0;
            Jacobian(iteration + 1) = 0.3*(- L_1 * sin(currentPosition[iteration+8]) + L_2 * cos(currentPosition[iteration+8] - currentPosition[iteration+9] - 0.785398));
            Jacobian(iteration + 2) = 6.0*(- L_2 * cos(currentPosition[iteration+8] - currentPosition[iteration+9] - 0.785398));
            iteration += 3;
        }

        torque = Jacobian * sharedMemory.Force * 2.0;

        test_torque.tail(12) = torque;
        test_torque.head(6)  << 0, 0, 0, 0, 0, 0;

        robot -> setGeneralizedForce(test_torque);
        world -> integrate();

        usleep(1000);

        if (setTime.localtime == timeDuration)
        {
            std::cout << "end!" << std::endl;
            for(int i = 0; i < 12 ; i++)
            {
                std::cout << i+1 <<"th joint torque : " << torque(i) << std::endl;
            }
            break;
        }
    }

    timeDuration += 0.5;

    while(true)
    {
        setTime.setLocaltime();
        robot -> getState(currentPosition, currentVelocity);

        sharedMemory.Force = ForceTrajectory.calcCoefficient(6.5, 7.0, 6.875, 750.0,setTime.localtime, 5.0);

        for(iteration = 0; iteration < 12;)
        {
            Jacobian(iteration) = 0.0;
            Jacobian(iteration + 1) = 0.3*(- L_1 * sin(currentPosition[iteration+8]) + L_2 * cos(currentPosition[iteration+8] - currentPosition[iteration+9] - 0.785398));
            Jacobian(iteration + 2) = 6.0*(- L_2 * cos(currentPosition[iteration+8] - currentPosition[iteration+9] - 0.785398));
            iteration += 3;
        }

        torque = Jacobian * sharedMemory.Force * 2.0;

        test_torque.tail(12) = torque;
        test_torque.head(6)  << 0, 0, 0, 0, 0, 0;

        robot -> setGeneralizedForce(test_torque);
        world -> integrate();

        usleep(1000);

        if (setTime.localtime == timeDuration)
        {
            std::cout << "end!" << std::endl;
            break;
        }
    }

    timeDuration += 4.0;

    while(true)
    {
        setTime.setLocaltime();
        robot -> getState(currentPosition, currentVelocity);

        sharedMemory.Force = ForceTrajectory.calcCoefficient(7.0, 11.0, 10.0, 5.0,setTime.localtime, 45.08);

        for(iteration = 0; iteration < 12;)
        {
            Jacobian(iteration) = 0.0;
            Jacobian(iteration + 1) = (- L_1 * sin(currentPosition[iteration+8]) + L_2 * cos(currentPosition[iteration+8] - currentPosition[iteration+9] - 0.785398));
            Jacobian(iteration + 2) = 4.0*(- L_2 * cos(currentPosition[iteration+8] - currentPosition[iteration+9] - 0.785398));
            iteration += 3;
        }

        torque = Jacobian * sharedMemory.Force * 2.0;

        test_torque.tail(12) = torque;
        test_torque.head(6)  << 0, 0, 0, 0, 0, 0;

        robot -> setGeneralizedForce(test_torque);
        world -> integrate();

        usleep(1000);

        if (setTime.localtime == timeDuration)
        {
            std::cout << "end!" << std::endl;
            break;
        }
    }

}

