//
// Created by tina on 23. 11. 7.
//

#ifndef EE3100704_PROJECTS_JOINTCONTROLL_HPP
#define EE3100704_PROJECTS_JOINTCONTROLL_HPP

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "setTime.hpp"
#include "cubicTrajectoryGenerator.hpp"
#include "/home/percy/robot_ws/EE3100704/examples/include/Variables/SharedMemory.h"

class robotController {
public:
    void setInitialState(raisim::ArticulatedSystem* robot, Eigen::VectorXd initialPosition);
    void setPDgain(Eigen::VectorXd Pgain, Eigen::VectorXd Dgain);
    void setFixedBasePosition(raisim::World* world, raisim::ArticulatedSystem* robot, float timeDuration);
    void setFloatingBasePosition(raisim::World* world, raisim::ArticulatedSystem* robot, float timeDuration);
    void setTestMotion(raisim::World *world, raisim::ArticulatedSystem *robot);
    void setTestMotion_2(raisim::World *world, raisim::ArticulatedSystem *robot);
    void setStand(raisim::World* world, raisim::ArticulatedSystem* robot);
    void setStand_2(raisim::World *world, raisim::ArticulatedSystem *robot);
    void setSit(raisim::World* world, raisim::ArticulatedSystem* robot);
    void setDampingMotion(raisim::World* world, raisim::ArticulatedSystem* robot);
    void setTrot(raisim::World* world, raisim::ArticulatedSystem* robot);
    void setPDControl(raisim::World* world, raisim::ArticulatedSystem* robot);

private:
    void setBasePose();

private:
    float d2r = 3.141592/180;
    int baseQuaternion = 7;

    Eigen::VectorXd basePose;

    Eigen::VectorXd mPgain;
    Eigen::VectorXd mDgain;
};


#endif //EE3100704_PROJECTS_JOINTCONTROLL_HPP
