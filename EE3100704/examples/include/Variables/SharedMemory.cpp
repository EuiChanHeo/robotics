//
// Created by percy on 23. 12. 14.
//

#include "SharedMemory.h"

void initializeSHM(SHM& sharedMemory, int vectorSize)
{
    sharedMemory.currentPosition = new Eigen::VectorXd(vectorSize);
}

void releaseSHM(SHM& sharedMemory)
{
    delete sharedMemory.currentPosition;
}