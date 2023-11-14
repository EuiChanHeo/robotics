//
// Created by percy on 23. 11. 12.
//

#ifndef EE3100704_PROJECTS_CANINE_H
#define EE3100704_PROJECTS_CANINE_H

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "cubicTrajectoryGenerator.hpp"
#include "robotController.hpp"
#include "setObstacle.hpp"
#include "Variables/SharedMemory.h"
//#include "/home/percy/robot_ws/EE3100704/examples/include/Variables/SharedMemory.h"
#include "../src/robot_UI/robot_ui/mainwindow.h"
#include <QApplication>
#include <QThread>

class Canine{

private:

public:

    void RunSimul();
    void RunPart();
};
#endif //EE3100704_PROJECTS_CANINE_H
