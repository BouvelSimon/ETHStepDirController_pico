#ifndef TRAJECTORY

#define TRAJECTORY
#define TRAJTYPE_DEFAULT 0
#define TRAJTYPE_LINEAR 1
#define TRAJTYPE_SPLINE 2
#define TRAJTYPE_BEZIER 3

/*
#define TIMETYPE_ABSOLUTE 0
#define TIMETYPE_NEXT 1
#define TIMETYPE_NOW 2
*/

#include <Arduino.h>

class trajectory{
  public:
    trajectory();
    trajectory(uint32_t*,int32_t*,uint8_t, uint8_t,uint16_t);
    void defineTrajectory(uint32_t TimePoints[],int32_t PositionPoints[],uint8_t nPoints, uint8_t TrajectoryType,uint16_t id);
    void shiftTimePoints(uint32_t TimeShift);
    int32_t evaluate(uint32_t CurrentTime);
    uint32_t getEndTime();
    uint16_t getId();
    int32_t getEndPosition();

  private:
    uint32_t timePoints[256];
    int32_t positionPoints[256];
    uint8_t nPoints;
    uint8_t trajectoryType;
    uint16_t id;
    int32_t lastEvaluation;
};

class trajectoryList{
  public:
    trajectoryList();
    void deleteTrajectories();
    void appendTrajectory(uint32_t TimePoints[],int32_t PositionPoints[],uint8_t NPoints, uint8_t TrajectoryType,uint16_t Id);
    void go();
    bool isGoing();
    int32_t evaluate();
    uint16_t getActiveTrajectoryId();
    uint8_t getNTrajectories();
    int32_t getLastPosition();
    void setLastEvaluation(int32_t); // this is useful to keep the object up to date when motion is managed by something else
    
  private:
    uint8_t nTrajectories;
    trajectory trajectories[2];
    uint32_t startTime;
    int32_t lastEvaluation;
    bool isActive;
    void deleteFirstAndShift();
    bool going;
};


#endif