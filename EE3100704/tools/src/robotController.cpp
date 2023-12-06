//
// Created by tina on 23. 11. 7.
//

#include "robotController.hpp"

SHM SharedMemory;

void robotController::setInitialState(raisim::ArticulatedSystem *robot, Eigen::VectorXd initialPosition)
{
    robot->setGeneralizedCoordinate(initialPosition);
}

void robotController::setPDgain(Eigen::VectorXd Pgain, Eigen::VectorXd Dgain)
{
    mPgain = Pgain;
    mDgain = Dgain;
}

void robotController::setBasePose()
{
    basePose = Eigen::VectorXd (baseQuaternion);
    /// set base goal position
    std::cout << "\ninput base position x value ";
    std::cin >> basePose[0];
    std::cout << "input base position y value ";
    std::cin >> basePose[1];
    std::cout << "input base position z value ";
    std::cin >> basePose[2];
    std::cout << "input base orientation w value ";
    std::cin >> basePose[3];
    std::cout << "input base orientation i value ";
    std::cin >> basePose[4];
    std::cout << "input base orientation j value ";
    std::cin >> basePose[5];
    std::cout << "input base orientation k value ";
    std::cin >> basePose[6];

    std::cout << "base position : ";
    for (int position = 0; position < 3; position++)
    {
        std::cout << basePose[position] << "  ";
    }
    std::cout << "\nbase orientation : ";
    for (int orientation = 0; orientation < 4; orientation++)
    {
        std::cout << basePose[orientation + 3] << "  " ;
    }
}

void robotController::setFixedBasePosition(raisim::World* world, raisim::ArticulatedSystem *robot, float timeDuration)
{
    cubicTrajectoryGenerator trajectoryGenerator[robot->getDOF()];
    setTime setTime;

    Eigen::VectorXd goalPosition(robot->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointPositionTarget(robot->getGeneralizedCoordinateDim()), jointVelocityTarget(robot->getDOF());
    Eigen::VectorXd currentPosition(robot->getGeneralizedCoordinateDim());

    /// get joint current state
    for (int i = 0; i < robot->getDOF(); i++)
    {
        currentPosition(i) = robot->getGeneralizedCoordinate()[i] ;
    }

    /// set time
    setTime.setTimeInitiallize();
    setTime.timedT = 0.02;

    /// set joint goal position
    std::cout << " " << std::endl;
    for (int i = 0; i < robot->getDOF(); i++)
    {
        std::cout << "input joint " << i+1 << " value (degree) : ";
        std::cin >> goalPosition[i];
    }
    goalPosition = goalPosition*d2r;

    /// check goal position
    for (int i = 0; i < robot->getGeneralizedCoordinateDim(); i++)
    {
        std::cout << goalPosition[i];
    }

    /// create trajectory
    for (int i = 0; i < robot->getGeneralizedCoordinateDim(); i++)
    {
        trajectoryGenerator[i].updateTrajectory(currentPosition[i],goalPosition[i],setTime.localtime,timeDuration)  ;
    }

    while (1)
    {
        setTime.setLocaltime(); //get in while loop.
        for (int jointNum = 0; jointNum < robot->getGeneralizedCoordinateDim() ; jointNum++)
        {
            jointPositionTarget[jointNum] = trajectoryGenerator[jointNum].getPositionTrajectory(setTime.localtime);
            jointVelocityTarget[jointNum] = trajectoryGenerator[jointNum].getVelocityTrajectory(setTime.localtime);
        }

        /// robot set position
        robot->setGeneralizedCoordinate(jointPositionTarget);
        robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
        world->integrate();
        usleep(10000);
        if (setTime.localtime == timeDuration)
            break;
    }
    robot->setPdGains(mPgain, mDgain);
    robot->setPdTarget(jointPositionTarget, jointVelocityTarget);
    std::cout << "\n" <<  "robot current position  :  " << robot->getGeneralizedCoordinate() << std::endl;

}

void robotController::setFloatingBasePosition(raisim::World* world, raisim::ArticulatedSystem *robot, float timeDuration)
{
    cubicTrajectoryGenerator trajectoryGenerator[robot->getGeneralizedCoordinateDim()];
    setTime setTime;
    Eigen::VectorXd goalPosition(robot->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointPositionTarget(robot->getGeneralizedCoordinateDim()), jointVelocityTarget(robot->getDOF());
    Eigen::VectorXd currentPosition(robot->getGeneralizedCoordinateDim());
    jointVelocityTarget.setZero();

    /// get joint current state
    for (int i = 0; i < robot->getGeneralizedCoordinateDim(); i++)
    {
        currentPosition(i) = robot->getGeneralizedCoordinate()[i] ;
    }

    /// set time
    setTime.setTimeInitiallize();
    setTime.timedT = 0.02;

    /// set joint goal position
    std::cout << " " << std::endl;
    for (int i = 0; i < robot->getGeneralizedCoordinateDim()-baseQuaternion; i++)
    {
        std::cout << "input joint " << i+1 << " value (degree) : ";
        std::cin >> goalPosition[i+baseQuaternion];
    }
    goalPosition = d2r*goalPosition;

    /// set base goal position
    setBasePose();
    for (int i = 0; i < baseQuaternion; i++)
    {
        goalPosition[i] = basePose[i];
    }

    /// check goal position
    std::cout << "\ngoalPosition  :  " ;
    for (int i = 0; i < robot->getGeneralizedCoordinateDim()-baseQuaternion; i++)
    {
        std::cout << goalPosition[i];
    }
    /// create trajectory
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
        /// robot set position
        robot->setGeneralizedCoordinate(jointPositionTarget);
        robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
        robot->setPdGains(mPgain, mDgain);
        robot->setPdTarget(jointPositionTarget, jointVelocityTarget);
        world->integrate();
        usleep(10000);
        if (setTime.localtime == timeDuration)
            break;
    }
    std::cout << "\n" <<  "robot current position  :  " << robot->getGeneralizedCoordinate() << std::endl;
    robot->setPdTarget(jointPositionTarget, jointVelocityTarget);

}

void robotController::torque_Stand_1(raisim::World *world, raisim::ArticulatedSystem *robot)
{
    setTime setTime;
    float timeDuration = 1.5;
    cubicTrajectoryGenerator trajectoryGenerator[robot->getGeneralizedCoordinateDim()];

    setTime.setTimeInitiallize();
    setTime.timedT = 0.001;

    Eigen::VectorXd goalPosition(robot->getGeneralizedCoordinateDim());
//    goalPosition << 0, 0, 0.35, 1, 0, 0, 0, -0.01, 0.0 , -(1.59523 - 0.55), -0.01, 0.0, -(1.59872 - 0.55), 0.01, 0.0, -(1.5 - 0.55), 0.01, 0.0, -(1.5 - 0.55);
    goalPosition << 0, 0, 0.35, 1, 0, 0, 0, -0.01, 0.794125 + 0.523599  ,-(1.59523 + 0.523599),-0.01, 0.802851 + 0.523599, -(1.59872 + 0.523599), 0.01, 0.724311 + 0.523599, -(1.5 + 0.523599), 0.01, 0.731293 + 0.523599, -(1.5 + 0.523599);

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

void robotController::torque_Stand_2(raisim::World *world, raisim::ArticulatedSystem *robot)
{
    setTime setTime;
    float timeDuration = 1.5;
    cubicTrajectoryGenerator trajectoryGenerator[robot->getGeneralizedCoordinateDim()];

    setTime.setTimeInitiallize();
    setTime.timedT = 0.001;

    Eigen::VectorXd goalPosition(robot->getGeneralizedCoordinateDim());
    goalPosition << 0, 0, 0.35, 1, 0, 0, 0, -0.01, 0.0 , -(1.59523 - 0.55), -0.01, 0.0, -(1.59872 - 0.55), 0.01, 0.0, -(1.5 - 0.55), 0.01, 0.0, -(1.5 - 0.55);

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
    robot -> setGeneralizedForce(test_torque);
}

void robotController::setStand_1(raisim::World *world, raisim::ArticulatedSystem *robot)
{
    world->setGravity({0,0,-9.8});
    cubicTrajectoryGenerator trajectoryGenerator[robot->getGeneralizedCoordinateDim()];

    setTime setTime;
    float timeDuration = 5.0;

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

    // set joint stand position
    goalPosition << 0, 0, 0.35, 1, 0, 0, 0, -0.00523599, 0.794125 + 0.523599  ,-(1.59523 + 0.523599),-0.00523599, 0.802851 + 0.523599, -(1.59872 + 0.523599), 0.00698132, 0.724311 + 0.523599, -(1.5 + 0.523599), 0.0174533, 0.731293 + 0.523599, -(1.5 + 0.523599);

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

void robotController::setStand_2(raisim::World *world, raisim::ArticulatedSystem *robot)
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

    goalPosition << 0, 0, 0.35, 1, 0, 0, 0, -0.00523599, 0.794125 ,-1.59523,-0.00523599, 0.802851, -1.59872, 0.00698132, 0.724311, -1.5, 0.0174533, 0.731293, -1.5;

    /// create trajectory
    for (int i = 0; i < robot->getGeneralizedCoordinateDim(); i++)
    {
        trajectoryGenerator[i].updateTrajectory(currentPosition[i],goalPosition[i],setTime.localtime,timeDuration) ;
    }

    std::cout << " stand up ! " << std::endl;

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

void robotController::setStand(raisim::World *world, raisim::ArticulatedSystem *robot)
{
    setStand_1(world,robot);
    setStand_2(world,robot);
}

void robotController::setSit(raisim::World *world, raisim::ArticulatedSystem *robot)
{
    cubicTrajectoryGenerator trajectoryGenerator[robot->getGeneralizedCoordinateDim()];
    setTime setTime;
    float timeDuration = 10.0;

    /// set time
    setTime.setTimeInitiallize();
    setTime.timedT = 0.02;

    Eigen::VectorXd goalPosition(robot->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointPositionTarget(robot->getGeneralizedCoordinateDim()), jointVelocityTarget(robot->getDOF());
    Eigen::VectorXd currentPosition(robot->getGeneralizedCoordinateDim());
    jointVelocityTarget.setZero();

    for (int i = 0; i < robot->getGeneralizedCoordinateDim(); i++)
    {
        currentPosition(i) = robot->getGeneralizedCoordinate()[i] ;
    }

    /// set joint sit position
    goalPosition << 0, 0, 0.07, 1, 0, 0, 0, 0.0872664, 2.17643, -2.76635, -0.0872664, 2.1869, -2.75587, 0.0837758, 2.17992, -2.73, -0.0837758, 2.18166, -2.73;

    /// create trajectory
    for (int i = 0; i < robot->getGeneralizedCoordinateDim(); i++)
    {
        trajectoryGenerator[i].updateTrajectory(currentPosition[i],goalPosition[i],setTime.localtime,timeDuration) ;
    }

    std::cout << " sit down ! " << std::endl;

    while (1)
    {
        setTime.setLocaltime();
        for (int jointNum = 0; jointNum < robot->getGeneralizedCoordinateDim() ; jointNum++)
        {
            jointPositionTarget[jointNum] = trajectoryGenerator[jointNum].getPositionTrajectory(setTime.localtime);
        }

        /// robot set sit position
//        robot->setGeneralizedCoordinate(jointPositionTarget);
        robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
        robot->setPdGains(mPgain, mDgain);
        robot->setPdTarget(jointPositionTarget, jointVelocityTarget);
        world->integrate();
        usleep(10000);
        if (setTime.localtime == timeDuration)
            break;
    }
    std::cout << robot->getGeneralizedCoordinate() << "\n" <<std::endl;
    robot->setPdTarget(jointPositionTarget, jointVelocityTarget);
}