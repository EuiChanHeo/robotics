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
void robotController::setTrot(raisim::World* world, raisim::ArticulatedSystem* robot)
{
    
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

void robotController::setPDControl(raisim::World *world, raisim::ArticulatedSystem *robot)
{
    cubicTrajectoryGenerator trajectoryGenerator[robot->getGeneralizedCoordinateDim()];
    setTime setTime;
    float timeDuration = 10.0;

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


}

void robotController::setTestMotion(raisim::World *world, raisim::ArticulatedSystem *robot)
{
    world->setGravity({0,0,-9.0});
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

    /// set joint stand position
    goalPosition << SharedMemory.test_x, SharedMemory.test_y, SharedMemory.test_z, SharedMemory.test_w, SharedMemory.test_i, SharedMemory.test_j, SharedMemory.test_k, SharedMemory.test_angle_1, SharedMemory.test_angle_2, SharedMemory.test_angle_3, SharedMemory.test_angle_4, SharedMemory.test_angle_5, SharedMemory.test_angle_6, SharedMemory.test_angle_7, SharedMemory.test_angle_8, SharedMemory.test_angle_9, SharedMemory.test_angle_10, SharedMemory.test_angle_11, SharedMemory.test_angle_12;
    /// create trajectory
    for (int i = 0; i < robot->getGeneralizedCoordinateDim(); i++)
    {
        trajectoryGenerator[i].updateTrajectory(currentPosition[i],goalPosition[i],setTime.localtime,timeDuration) ;
    }

    std::cout << " change posture ! " << std::endl;

    while (1)
    {
        setTime.setLocaltime(); //get in while loop.
        for (int jointNum = 0; jointNum < robot->getGeneralizedCoordinateDim() ; jointNum++)
        {
            jointPositionTarget[jointNum] = trajectoryGenerator[jointNum].getPositionTrajectory(setTime.localtime);
        }

        /// robot set stand position
//        robot->setGeneralizedCoordinate(jointPositionTarget);
//        robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
//        Eigen::VectorXd tau_test(18);
//        tau_test << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
//        robot->setGeneralizedForce(tau_test);

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

void robotController::setTestMotion_2(raisim::World *world, raisim::ArticulatedSystem *robot)
{
    world->setGravity({0,0,-9.0});
    cubicTrajectoryGenerator trajectoryGenerator[robot->getGeneralizedCoordinateDim()];
    setTime setTime;
    float timeDuration = 10.0;

    /// set time
    setTime.setTimeInitiallize();
//    setTime.timedT = 0.02;
    setTime.timedT = 0.08;
    Eigen::VectorXd goalPosition(robot->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointPositionTarget(robot->getGeneralizedCoordinateDim()), jointVelocityTarget(robot->getDOF());
    Eigen::VectorXd currentPosition(robot->getGeneralizedCoordinateDim());
    jointVelocityTarget.setZero();

    for (int i = 0; i < robot->getGeneralizedCoordinateDim(); i++)
    {
        currentPosition(i) = robot->getGeneralizedCoordinate()[i] ;
    }

    /// set joint stand position
    goalPosition << SharedMemory.test2_x, SharedMemory.test2_y, SharedMemory.test2_z, SharedMemory.test2_w, SharedMemory.test2_i, SharedMemory.test2_j, SharedMemory.test2_k, SharedMemory.test2_angle_1, SharedMemory.test2_angle_2, SharedMemory.test2_angle_3, SharedMemory.test2_angle_4, SharedMemory.test2_angle_5, SharedMemory.test2_angle_6, SharedMemory.test2_angle_7, SharedMemory.test2_angle_8, SharedMemory.test2_angle_9, SharedMemory.test2_angle_10, SharedMemory.test2_angle_11, SharedMemory.test2_angle_12;
    /// create trajectory
    for (int i = 0; i < robot->getGeneralizedCoordinateDim(); i++)
    {
        trajectoryGenerator[i].updateTrajectory(currentPosition[i],goalPosition[i],setTime.localtime,timeDuration) ;
    }

    std::cout << " change posture ! " << std::endl;

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
        usleep(10000);
        if (setTime.localtime == timeDuration)
            break;
    }
    std::cout << robot->getGeneralizedCoordinate() << "\n" <<std::endl;
    robot->setPdTarget(jointPositionTarget, jointVelocityTarget);
}
void robotController::setDampingMotion(raisim::World* world, raisim::ArticulatedSystem* robot)
{
    world->setGravity({0,0,-9.0});
    cubicTrajectoryGenerator trajectoryGenerator[robot->getGeneralizedCoordinateDim()];
    setTime setTime;
    float timeDuration = 10.0;

    /// set time
    setTime.setTimeInitiallize();
//    setTime.timedT = 0.02;
    setTime.timedT = 0.08;
    Eigen::VectorXd goalPosition(robot->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointPositionTarget(robot->getGeneralizedCoordinateDim()), jointVelocityTarget(robot->getDOF());
    Eigen::VectorXd currentPosition(robot->getGeneralizedCoordinateDim());
    jointVelocityTarget.setZero();

    for (int i = 0; i < robot->getGeneralizedCoordinateDim(); i++)
    {
        currentPosition(i) = robot->getGeneralizedCoordinate()[i] ;
    }

    /// set joint stand position
    goalPosition << SharedMemory.damping_x, SharedMemory.damping_y, SharedMemory.damping_z, SharedMemory.damping_w, SharedMemory.damping_i, SharedMemory.damping_j, SharedMemory.damping_k, SharedMemory.damping_angle_1, SharedMemory.damping_angle_2, SharedMemory.damping_angle_3, SharedMemory.damping_angle_4, SharedMemory.damping_angle_5, SharedMemory.damping_angle_6, SharedMemory.damping_angle_7, SharedMemory.damping_angle_8, SharedMemory.damping_angle_9, SharedMemory.damping_angle_10, SharedMemory.damping_angle_11, SharedMemory.damping_angle_12;
    /// create trajectory
    for (int i = 0; i < robot->getGeneralizedCoordinateDim(); i++)
    {
        trajectoryGenerator[i].updateTrajectory(currentPosition[i],goalPosition[i],setTime.localtime,timeDuration) ;
    }

    std::cout << " change posture ! " << std::endl;

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
        usleep(10000);
        if (setTime.localtime == timeDuration)
            break;
    }
    std::cout << robot->getGeneralizedCoordinate() << "\n" <<std::endl;
    robot->setPdTarget(jointPositionTarget, jointVelocityTarget);
}

void robotController::setStand(raisim::World *world, raisim::ArticulatedSystem *robot)
{
    world->setGravity({0,0,-9.0});
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

    /// set joint stand position
    goalPosition << 0, 0, 0.35, 1, 0, 0, 0, -0.00523599, 0.794125 ,-1.59523,-0.00523599, 0.802851, -1.59872, 0.00698132, 0.724311, -1.4, 0.0174533, 0.731293, -1.4;

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
        robot->setGeneralizedCoordinate(jointPositionTarget);
        robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
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

void robotController::setStand_2(raisim::World *world, raisim::ArticulatedSystem *robot)
{
    world->setGravity({0,0,-9.0});
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

    /// set joint stand position
//    goalPosition << 0, 0, 0.35, 1, 0, 0, 0, -0.00523599, 0.794125 ,-1.59523,-0.00523599, 0.802851, -1.59872, 0.00698132, 0.724311, -1.4, 0.0174533, 0.731293, -1.4;
    // x,y,z,w,i,j,k,angle_1~12
    goalPosition << 0, 0, 0.35, 1, 0, 0, 0, -0.00523599, 0.794125 ,-1.59523,-0.00523599, 0.802851, -1.59872, 0.00698132, 0.724311, -1.4, 0.0174533, 0.731293, -1.4;

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
        usleep(10000);
        if (setTime.localtime == timeDuration)
            break;
    }
    //std::cout << robot->getGeneralizedCoordinate() << "\n" <<std::endl;
    robot->setPdTarget(jointPositionTarget, jointVelocityTarget);
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