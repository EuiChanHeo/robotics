#include "../include/canine.h"

SHM sharedMemory;
std::chrono::steady_clock::time_point startTime;

void Canine::RunSimul()
{
}

void Canine::GetDuration(){
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    sharedMemory.duration = static_cast<double>(elapsedTime.count());
}

void Canine::RunPart()
{
    startTime = std::chrono::steady_clock::now();
    raisim::World world;
    world.setTimeStep(0.001);
    world.setGravity({0.0,0.0,-9.8});

    auto ground = world.addGround();

    auto canine = world.addArticulatedSystem("/home/percy/robot_ws/EE3100704/examples/rsc/canine/urdf/canineV4_2.urdf");
    raisim::RaisimServer server(&world);
    server.focusOn(canine);
    server.launchServer();
    canine->setName("canine");

    setObstacle setObstacle;
    setObstacle.setWall(&world, 0.1, 1, 0.5, 3.0, 1.2, 0.0, 0.25);

    // set joint Initialization
    Eigen::VectorXd initialJointPosition(canine->getGeneralizedCoordinateDim()), jointVelocityTarget(canine->getDOF());
    initialJointPosition << sharedMemory.init_x, sharedMemory.init_y, sharedMemory.init_z, sharedMemory.init_w, sharedMemory.init_i, sharedMemory.init_j, sharedMemory.init_k, sharedMemory.init_angle_1, sharedMemory.init_angle_2, sharedMemory.init_angle_3, sharedMemory.init_angle_4, sharedMemory.init_angle_5, sharedMemory.init_angle_6, sharedMemory.init_angle_7, sharedMemory.init_angle_8, sharedMemory.init_angle_9, sharedMemory.init_angle_10, sharedMemory.init_angle_11, sharedMemory.init_angle_12;
    jointVelocityTarget.setZero();

    Eigen::VectorXd jointPgain(canine->getDOF()), jointDgain(canine->getDOF());
    jointPgain.tail(12).setConstant(50.0);
    jointDgain.tail(12).setConstant(1.0);
    sleep(1);
    canine->setGeneralizedCoordinate(initialJointPosition);
    canine->setGeneralizedForce(Eigen::VectorXd::Zero(canine->getDOF()));
    canine->setPdGains(jointPgain, jointDgain);
    canine->setPdTarget(initialJointPosition, jointVelocityTarget);

    sleep(2);

    robotController controller;

    controller.setPDgain(jointPgain,jointDgain);
    controller.setStand_2(&world, canine);
//    controller.torque_Stand_2(&world, canine);
//    controller.setSit(&world, canine);

    for (int i=0; i<2000000; i++)
    {
        RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
        server.integrateWorldThreadSafe();
    }

    server.killServer();

}

class CommunicationThread : public QThread
{
public:
    void run() override
    {
        Canine test;
        test.RunPart();
    }

};

int main (int argc, char* argv[]) {
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);

    QApplication a(argc, argv);
    MainWindow w;
    CommunicationThread communicationThread;
    communicationThread.start();

    w.show();

    return a.exec();

}
