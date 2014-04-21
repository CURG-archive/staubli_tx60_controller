#ifndef STAUBLISTATE_H
#define STAUBLISTATE_H

#include "TX60L.h"

class StaubliState
{
    public:
        std::vector<double> currentJoints;
        std::vector<double> currentCartesianPosition;

        StaubliState()
        {
            currentJoints.resize(6);
            currentCartesianPosition.resize(6);
        }

        void updateState(TX60L staubli)
        {
            staubli.GetRobotJoints(currentJoints);
            staubli.GetRobotCartesianPosition(currentCartesianPosition);
        }
};


#endif
