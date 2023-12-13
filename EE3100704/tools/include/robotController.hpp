//
// Created by tina on 23. 11. 7.
//

#ifndef EE3100704_PROJECTS_JOINTCONTROLL_HPP
#define EE3100704_PROJECTS_JOINTCONTROLL_HPP

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "setTime.hpp"
#include "cubicTrajectoryGenerator.h"
#include "forceTrajectoryGenerator.h"
#include "/home/percy/robot_ws/EE3100704/examples/include/Variables/SharedMemory.h"
#include "cmath"

class robotController {
public:
    void setInitialState(raisim::ArticulatedSystem* robot, Eigen::VectorXd initialPosition);
    void setPDgain(Eigen::VectorXd Pgain, Eigen::VectorXd Dgain);
    void setFold(raisim::World* world, raisim::ArticulatedSystem* robot);
    void setFold_2(raisim::World *world, raisim::ArticulatedSystem *robot);
    void Force_stand(raisim::World *world, raisim::ArticulatedSystem *robot);
    void Dynamics_Stand(raisim::World *world, raisim::ArticulatedSystem *robot);

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
