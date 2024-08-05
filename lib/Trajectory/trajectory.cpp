
#include <Arduino.h>
#include "trajectory.h"

trajectory::trajectory(){
  id=0;
  trajectoryType=TRAJTYPE_DEFAULT;
  nPoints=0;
  lastEvaluation=0;
}


trajectory::trajectory(uint32_t TimePoints[],int32_t PositionPoints[],uint8_t NPoints, uint8_t TrajectoryType,uint16_t Id){
  id=Id;
  trajectoryType=TrajectoryType;
  nPoints=NPoints;
  for(uint8_t i=0;i<nPoints;i++){
    timePoints[i]=TimePoints[i];
    positionPoints[i]=PositionPoints[i];
  }
  lastEvaluation=0;
}

void trajectory::defineTrajectory(uint32_t TimePoints[],int32_t PositionPoints[],uint8_t NPoints, uint8_t TrajectoryType,uint16_t Id){
  id=Id;
  trajectoryType=TrajectoryType;
  nPoints=NPoints;
  for(uint8_t i=0;i<nPoints;i++){
    timePoints[i]=TimePoints[i];
    positionPoints[i]=PositionPoints[i];
  }
}

int32_t trajectory::evaluate(uint32_t CurrentTime_ms){
  switch (trajectoryType){
  case TRAJTYPE_DEFAULT:
    return lastEvaluation;

  case TRAJTYPE_LINEAR:
    // Let us first look for the current point index.
    int32_t indexFound=-1;
    for(uint16_t i=0;i<nPoints-1;i++){
      if(CurrentTime_ms>timePoints[i] && CurrentTime_ms<=timePoints[i+1]){
        indexFound=i;
      }
    }
    if(indexFound!=-1){
      lastEvaluation=positionPoints[indexFound]+int32_t(float(CurrentTime_ms+1-timePoints[indexFound])/float(timePoints[indexFound+1]-timePoints[indexFound])*(positionPoints[indexFound+1]-positionPoints[indexFound]));// Note : The +1 is to go slightly (1ms) above alloted time in order to be able to reach final value. Otherwise, final value will never be reached
    }
    
    //if(CurrentTime_ms<timePoints[1] && timePoints[0]<CurrentTime_ms) // critical todo :this allows only for two points, it must be modified
    //  lastEvaluation=positionPoints[0]+int32_t(float(CurrentTime_ms+1-timePoints[0])/float(timePoints[1]-timePoints[0])*(positionPoints[1]-positionPoints[0]));// Note : The +1 is to go slightly (1ms) above alloted time in order to be able to reach final value. Otherwise, final value will never be reached
    return lastEvaluation;
    break;
    
  // to do : implement spline and bezier curves
    
  }
  return 0;
}

void trajectory::shiftTimePoints(uint32_t TimeShift){
  for(uint8_t i=0;i<nPoints;i++){
    timePoints[i]+=TimeShift;
  }
}

uint32_t trajectory::getEndTime(){
  return timePoints[nPoints-1];
}

int32_t trajectory::getEndPosition(){
  return positionPoints[nPoints-1];
}

uint16_t trajectory::getId(){
  return id;
}

trajectoryList::trajectoryList(){
  lastEvaluation=0;
  nTrajectories=0;
  going=false;
}

void trajectoryList::deleteTrajectories(){
  int32_t tempPos[]={0};
  uint32_t tempTime[]={0};
  trajectory tempTraj(tempTime,tempPos,0,TRAJTYPE_DEFAULT,0);
  trajectories[0]=tempTraj;
  trajectories[1]=tempTraj;
  nTrajectories=0;
  going=false;
}

void trajectoryList::appendTrajectory(uint32_t TimePoints[],int32_t PositionPoints[],uint8_t NPoints, uint8_t TrajectoryType,uint16_t Id){
  // notes on the TimePoints : 
  //  If a trajectory is already stored, then the first instant of the new trajectory is the last instant of the stored trajectory
  // Positions are always absolute though.

  uint32_t tempTimePoints[NPoints];

  switch (nTrajectories){    
    case 2: // No more space : nothing happens
      return;
      break;

    case 1: // One slot available, fill the second slot
      for(uint8_t i=0;i<NPoints;i++)
        tempTimePoints[i]=TimePoints[i]+trajectories[0].getEndTime();
      trajectories[1]=trajectory(tempTimePoints,PositionPoints,NPoints,TrajectoryType,Id);
      nTrajectories=2;
      break;

    case 0: // Two slots available, fill the first slot
      if(going){
        uint32_t currentTime=millis();
        for(uint8_t i=0;i<NPoints;i++)
          tempTimePoints[i]=TimePoints[i]+currentTime;
        trajectories[0]=trajectory(tempTimePoints,PositionPoints,NPoints,TrajectoryType,Id);
      }else{
        trajectories[0]=trajectory(TimePoints,PositionPoints,NPoints,TrajectoryType,Id);
      }
      nTrajectories=1;
  }  
}

int32_t trajectoryList::evaluate(){
  if(!going || trajectories[0].getId()==0) // if you are waiting for a trajectoryList::go() call, or there is no trajectory in the buffer, then the evaluation is the previous value
    return lastEvaluation;

  // Determine whether or not to delete an active trajectory
  if(millis()>trajectories[0].getEndTime()){
    deleteFirstAndShift();
    if(millis()>trajectories[0].getEndTime()){ // if it happens twice, then both trajectories are obsolete.
      return lastEvaluation;
    }
  }
  lastEvaluation=trajectories[0].evaluate(millis());

  return lastEvaluation;
}

uint16_t trajectoryList::getActiveTrajectoryId(){
  if(going)
    return trajectories[0].getId();
  else
    return 0;
}

void trajectoryList::deleteFirstAndShift(){
  trajectories[0]=trajectories[1];
  int32_t tempPos[]={0};
  uint32_t tempTime[]={0};
  trajectory tempTraj(tempTime,tempPos,0,TRAJTYPE_DEFAULT,0);
  trajectories[1]=tempTraj;

  if(nTrajectories>0)
    nTrajectories-=1;
}

void trajectoryList::go(){
  // Shift all the times of the trajectories stored to account for starting right now.
  uint32_t currentTime=millis();
  if(nTrajectories==1){
    trajectories[0].shiftTimePoints(currentTime);
    trajectories[0].getEndTime();
    going=true;
    return;
  }
  if(nTrajectories==2){
    trajectories[0].shiftTimePoints(currentTime);
    trajectories[1].shiftTimePoints(currentTime);
    going=true;
    return;
  }
  going=true;
}

bool trajectoryList::isGoing(){
  return going;
}

uint8_t trajectoryList::getNTrajectories(){
  return nTrajectories;
}

int32_t trajectoryList::getLastPosition(){
  if(nTrajectories==0){
    return lastEvaluation;
  }
  return trajectories[nTrajectories-1].getEndPosition();
}

void trajectoryList::setLastEvaluation(int32_t valueToSet){
  lastEvaluation=valueToSet;
}

