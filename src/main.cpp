#include <Arduino.h>
#include <Wire.h>
#include <trajectory.h>
#include <hardware/pio.h>
#include <encoderRead.pio.h>
#include <PIDcontroller.h>

// Hardware defines
#define I2C_SDA 16
#define I2C_SCL 17 
#define I2C_ADDRESS 35
#define ENABLE_PIN 18
#define ENCODER_A_PIN 3 // Because of how PIO works, ENCODER_B_PIN will be ENCODER_A_PIN+1
#define PULSE_PIN 13
#define DIR_PIN 12

// State related defines
#define STATE_BASICPOSITION 1
#define STATE_BASICVELOCITY 2
#define STATE_ADVANCEDPOSITION 3

// Messages identifiers for the slave
#define MSGTYPE_SETNEXTREQUESTSIZE 78
#define MSGTYPE_SETMODE 101 // Used for setting config mode : closed loop or open loop
#define MSGTYPE_CONTROLCOEFS 102
#define MSGTYPE_HWCONFIG 103
#define MSGTYPE_APPENDTRAJ 104
#define MSGTYPE_SETPOINT 105
#define MSGTYPE_MOTORPOWER 106
#define MSGTYPE_MOTION 107
#define MSGTYPE_JOG 108
#define MSGTYPE_SETMOTORPOS 109
#define MSGTYPE_SETENCODERPOS 110
#define MSGTYPE_REBOOTSLAVE 66

// Error codes : 
#define ERRCODE_LIMITABORT 90

// useful values for readability
#define CLOSEDLOOP_MODE 52
#define OPENLOOP_MODE 51
#define CONFIGURATION_MODE 60
#define MSG_GO 61
#define MSG_ABORT 62
#define MSG_STOP 63

// Open-loop PID parameters
#define OL_PID_KP 1.0f
#define OL_PID_KI 0.0f
#define OL_PID_KD 0.0f
#define OL_PID_TAU_MS 200

// Time constants for motor control
#define PULSE_TIME_US 50 // Number of microseconds a pulse stays HIGH. Also the max number of microseconds a pulse can stay low
const uint32_t SAMPLE_TIME_US=2*PULSE_TIME_US;

// Trajectory related defines : 
#define TRAJ_N_POINTS_MAX 128
#define RELATIVE_POSITION 10
#define ABSOLUTE_POSITION 11

// Useful unions for conversion of variable into bytes arrays
union i32_ui8 {
    int32_t i32;
    uint8_t ui8[4];
};

union float_ui8 {
    float f;
    uint8_t ui8[4];
};

union ui16_ui8 {
    uint16_t ui16;
    uint8_t ui8[2];
};

union ui32_ui8 {
    uint32_t ui32;
    uint8_t ui8[4];
};

// State of this microcontroller : type of control
static uint16_t g_currentState;

// I2C related global variables : 
static uint8_t g_i2cSendBuffer[46];
static uint8_t g_i2cReceiveBuffer[WIRE_BUFFER_SIZE];
static uint8_t g_i2cReceivedSize;
static uint8_t g_i2cNextRequestSize;
static uint8_t g_nextErrorCode;

// Motion global variables : 
volatile i32_ui8 g_currentMotorPulses;
volatile i32_ui8 g_currentSetPoint;
volatile float_ui8 g_currentEncoderPosition;
float_ui8 g_currentEvaluatedPosition;
volatile float g_encoderPositionOffset;
volatile ui16_ui8 g_curentActiveTraj;
volatile bool g_isMotorOn;
bool g_closedLoopEnabled;
static uint8_t g_dipSwitchConfig;
static float_ui8 g_Kp;
static float_ui8 g_Ki;
static float_ui8 g_Kd;
static float_ui8 g_maxVelocity;
static bool g_invertedMotor;
static bool g_invertedEncoder;
static ui32_ui8 g_motorStepsPerRev;
static ui32_ui8 g_encoderStepsPerRev;
static ui16_ui8 g_errorLimit;
static PIDcontroller g_pidController(OL_PID_KP,OL_PID_KI,OL_PID_KD,SAMPLE_TIME_US,200,0);
static float g_pulseFrequency;
static int8_t g_dir;
static int32_t g_evaluatedMotorPosition;

// Jof related global variables
float_ui8 g_jogSpeed;
uint32_t g_currentJogStartDate_us;
int32_t g_currentJogStartPosition;

// Trajectory related global variables
static trajectoryList g_trajectories;
static int32_t g_positionsBeingReceived[TRAJ_N_POINTS_MAX];
static uint32_t g_timesBeingReceived[TRAJ_N_POINTS_MAX];

// Assignment of a pio state machine
PIO pioEncoder = pio0; // pio 0 is used for encoder 1
uint8_t sm1 = 0;
volatile int32_t g_rawEncoder;

// Function declarations : 
void pio_irq_handler_Enc1();
void updatePulse();
void i2cRequestCallback();
void i2cReceiveCallback(int);
void processDataReceived(uint8_t);
void updateSendBuffer();
float evaluateEncoderPosition();
void checkErrorLimit();


void setup() { // core 0 is dedicated to motion control and encoder reading
  
  pinMode(PULSE_PIN,OUTPUT);
  pinMode(DIR_PIN,OUTPUT);
  pinMode(ENABLE_PIN,OUTPUT);

  // Default start state : 
  g_isMotorOn=false;
  digitalWrite(ENABLE_PIN,LOW);
  g_closedLoopEnabled=false;
  g_currentState=STATE_BASICPOSITION;
  g_encoderPositionOffset=0;

  // State machine initialization (PIO for encoder read)
  sm1 = pio_claim_unused_sm(pioEncoder, true);
  pio_gpio_init(pioEncoder, ENCODER_A_PIN);
  pio_gpio_init(pioEncoder, ENCODER_A_PIN+1);
  uint pioOffset = pio_add_program(pioEncoder, &pio_rotary_encoder_program);
  pio_sm_config c1 = pio_rotary_encoder_program_get_default_config(pioOffset);
  sm_config_set_in_pins(&c1, ENCODER_A_PIN);
  sm_config_set_in_shift(&c1, false, false, 0);
  irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler_Enc1);
  irq_set_enabled(PIO0_IRQ_0, true);
  pio0_hw->inte0 = PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM1_BITS;
  pio_sm_init(pioEncoder, sm1, 16, &c1);
  pio_sm_set_enabled(pioEncoder, sm1, true);

  Serial.begin(9600);
}

void loop() {
  g_currentEncoderPosition.f=evaluateEncoderPosition();

  switch(g_currentState){
    case STATE_ADVANCEDPOSITION:
      g_currentSetPoint.i32=g_trajectories.evaluate();
      g_curentActiveTraj.ui16=g_trajectories.getActiveTrajectoryId();
      break;
    case STATE_BASICPOSITION:
      // Nothing, the position has been set through the setPoint i2c command
      break;
    case STATE_BASICVELOCITY:
      g_currentSetPoint.i32=g_currentJogStartPosition+(micros()-g_currentJogStartDate_us)*0.000001*g_jogSpeed.f;
      break;
    default:
      break;
  }
  g_pidController.setSetPoint(g_currentSetPoint.i32);
  
  if(g_closedLoopEnabled){
    g_evaluatedMotorPosition=int32_t(evaluateEncoderPosition());
  }else{
	  g_evaluatedMotorPosition=g_currentMotorPulses.i32;
  }
  
  
  g_pulseFrequency=g_pidController.update(g_evaluatedMotorPosition);
  
  // Evaluating pulse frequency (positive float with max value after this chunk of code) and g_dir (either 1 or -1)
  if(g_pulseFrequency>=0 && !g_invertedMotor || g_pulseFrequency<0 && g_invertedMotor){
    g_dir=1;
  }else{
    g_dir=-1;
  }
  if(g_pulseFrequency<0)
	  g_pulseFrequency=-g_pulseFrequency;
  if(g_maxVelocity.f>0 && g_pulseFrequency>g_maxVelocity.f)
    g_pulseFrequency=g_maxVelocity.f;

  updatePulse();
  checkErrorLimit();
}


void setup1(){
  
  // Global variables initialization
  g_i2cReceivedSize=0;
  g_i2cNextRequestSize=0;
  g_errorLimit.ui16=0;
  g_encoderStepsPerRev.ui32=400;
  g_motorStepsPerRev.ui32=400;
  g_maxVelocity.f=4000;
  

  
  // Setting up i2c
  delay(100); // a short delay to make sure the I2C starts after setup() is finished 
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(i2cRequestCallback);
  Wire.onReceive(i2cReceiveCallback);

}

void loop1(){
  updateSendBuffer();
  if(g_i2cReceivedSize!=0){
    processDataReceived(g_i2cReceivedSize);
    g_i2cReceivedSize=0;
  }
}

void updatePulse(){
  static uint32_t lastRisingEdge=0;
  static bool currentlyPulsing=false;
  
  // Should the signal be pulled low?
  if(currentlyPulsing && micros()-lastRisingEdge>PULSE_TIME_US){
    digitalWrite(PULSE_PIN,LOW);
    currentlyPulsing=false;
  }

  // If the frequency is zero, there wont be a rising edge, we can stop here
  if(g_pulseFrequency==0)
    return;

  if(g_dir==1){
    digitalWrite(DIR_PIN,HIGH);
  }
  if(g_dir==-1){
    digitalWrite(DIR_PIN,LOW);
  }

  uint32_t periodMicroseconds=1000000.0f/g_pulseFrequency;
  if(periodMicroseconds<SAMPLE_TIME_US)
    periodMicroseconds=SAMPLE_TIME_US; //the minimum low time is equal to the high time

  // Should the signal be raised high?
  if(!currentlyPulsing && micros()-lastRisingEdge>periodMicroseconds){
    digitalWrite(PULSE_PIN,HIGH);
    lastRisingEdge=micros();
    currentlyPulsing=true;
    g_currentMotorPulses.i32=g_currentMotorPulses.i32+g_dir;
  }
}

void i2cReceiveCallback(int len) {
  uint8_t i=0;
  while (Wire.available()){
    g_i2cReceiveBuffer[i++]=Wire.read();
  }
  if(i>=len){
    g_i2cReceivedSize=len;
  }
}

void i2cRequestCallback() {
  Wire.write(g_i2cSendBuffer,g_i2cNextRequestSize);
  if(g_i2cNextRequestSize>17)
	g_nextErrorCode=0; // reset the error code once it has been sent to the master
}

void processDataReceived(uint8_t size){
  switch(g_i2cReceiveBuffer[0]){
    case MSGTYPE_SETNEXTREQUESTSIZE:{
      g_i2cNextRequestSize=g_i2cReceiveBuffer[1];
      break;
    }
    case MSGTYPE_SETMODE:{
      bool receivedMode=(g_i2cReceiveBuffer[1]==CLOSEDLOOP_MODE);
      if(g_closedLoopEnabled && !receivedMode){ // transition from closed loop to open loop
        g_closedLoopEnabled=false;
        g_pidController=PIDcontroller(OL_PID_KP,OL_PID_KI,OL_PID_KD,SAMPLE_TIME_US,200,0);
        g_currentMotorPulses.i32=g_evaluatedMotorPosition;
      }
      if(!g_closedLoopEnabled && receivedMode){ // transition from open loop to closed loop
        g_closedLoopEnabled=true;
        g_pidController=PIDcontroller(g_Kp.f,g_Ki.f,g_Kd.f,SAMPLE_TIME_US,200,0);
        if(!g_invertedEncoder)
          g_rawEncoder=int32_t((g_evaluatedMotorPosition-g_encoderPositionOffset)*float(g_encoderStepsPerRev.ui32)/float(g_motorStepsPerRev.ui32));
        else
          g_rawEncoder=-int32_t((g_evaluatedMotorPosition-g_encoderPositionOffset)*float(g_encoderStepsPerRev.ui32)/float(g_motorStepsPerRev.ui32));
      }
      break;
    }
    case MSGTYPE_MOTORPOWER:{
      g_isMotorOn=bool(g_i2cReceiveBuffer[1]);
      if(g_isMotorOn)
        digitalWrite(ENABLE_PIN,HIGH);
      else
        digitalWrite(ENABLE_PIN,LOW);
      break;
    }
    case MSGTYPE_CONTROLCOEFS:{
      g_Kp.ui8[0]=g_i2cReceiveBuffer[1];
      g_Kp.ui8[1]=g_i2cReceiveBuffer[2];
      g_Kp.ui8[2]=g_i2cReceiveBuffer[3];
      g_Kp.ui8[3]=g_i2cReceiveBuffer[4];
      g_Ki.ui8[0]=g_i2cReceiveBuffer[5];
      g_Ki.ui8[1]=g_i2cReceiveBuffer[6];
      g_Ki.ui8[2]=g_i2cReceiveBuffer[7];
      g_Ki.ui8[3]=g_i2cReceiveBuffer[8];
      g_Kd.ui8[0]=g_i2cReceiveBuffer[9];
      g_Kd.ui8[1]=g_i2cReceiveBuffer[10];
      g_Kd.ui8[2]=g_i2cReceiveBuffer[11];
      g_Kd.ui8[3]=g_i2cReceiveBuffer[12];
      g_maxVelocity.ui8[0]=g_i2cReceiveBuffer[13];
      g_maxVelocity.ui8[1]=g_i2cReceiveBuffer[14];
      g_maxVelocity.ui8[2]=g_i2cReceiveBuffer[15];
      g_maxVelocity.ui8[3]=g_i2cReceiveBuffer[16];
      g_errorLimit.ui8[0]=g_i2cReceiveBuffer[17];
      g_errorLimit.ui8[1]=g_i2cReceiveBuffer[18];
      g_pidController.setGains(g_Kp.f,g_Ki.f,g_Kd.f);
      break;
    }
    case MSGTYPE_HWCONFIG:{
      g_motorStepsPerRev.ui8[0]=g_i2cReceiveBuffer[1];
      g_motorStepsPerRev.ui8[1]=g_i2cReceiveBuffer[2];
      g_motorStepsPerRev.ui8[2]=g_i2cReceiveBuffer[3];
      g_motorStepsPerRev.ui8[3]=g_i2cReceiveBuffer[4];
      g_encoderStepsPerRev.ui8[0]=g_i2cReceiveBuffer[5];
      g_encoderStepsPerRev.ui8[1]=g_i2cReceiveBuffer[6];
      g_encoderStepsPerRev.ui8[2]=g_i2cReceiveBuffer[7];
      g_encoderStepsPerRev.ui8[3]=g_i2cReceiveBuffer[8];
      g_invertedMotor=g_i2cReceiveBuffer[9]!=0;
      g_invertedEncoder=g_i2cReceiveBuffer[10]!=0;

      g_currentState=STATE_BASICPOSITION;
      if(!g_closedLoopEnabled){
        g_pidController=PIDcontroller(OL_PID_KP,OL_PID_KI,OL_PID_KD,SAMPLE_TIME_US,200,0);
      }else{
        g_pidController=PIDcontroller(g_Kp.f,g_Ki.f,g_Kd.f,SAMPLE_TIME_US,200,0);
      }
      g_trajectories.deleteTrajectories();
      g_trajectories.setLastEvaluation(g_evaluatedMotorPosition);
      g_currentSetPoint.i32=g_evaluatedMotorPosition;
      break;
    }
    case MSGTYPE_MOTION :{
      if(g_i2cReceiveBuffer[1]==MSG_GO){
        g_currentState=STATE_ADVANCEDPOSITION;
        g_pidController=PIDcontroller(g_Kp.f,g_Ki.f,g_Kd.f,SAMPLE_TIME_US,200,0);
        g_trajectories.setLastEvaluation(g_evaluatedMotorPosition);
        if(!g_trajectories.isGoing())
          g_trajectories.go(); 
      }
      if(g_i2cReceiveBuffer[1]==MSG_STOP){
        g_currentState=STATE_ADVANCEDPOSITION;
        g_trajectories.deleteTrajectories();
        g_trajectories.setLastEvaluation(g_evaluatedMotorPosition);
      }
      if(g_i2cReceiveBuffer[1]==MSG_ABORT){
        g_closedLoopEnabled=false;
        g_currentState=STATE_BASICPOSITION;
        g_pidController=PIDcontroller(OL_PID_KP,OL_PID_KI,OL_PID_KD,SAMPLE_TIME_US,200,0);
        g_trajectories.deleteTrajectories();
        g_trajectories.setLastEvaluation(g_evaluatedMotorPosition);
        g_currentSetPoint.i32=g_evaluatedMotorPosition;
        g_isMotorOn=false;
        digitalWrite(ENABLE_PIN,LOW);
      }
      break;
    }
    case MSGTYPE_SETPOINT:{
      g_currentState=STATE_BASICPOSITION;
      g_trajectories.deleteTrajectories();
      g_trajectories.setLastEvaluation(g_evaluatedMotorPosition);
      g_currentSetPoint.ui8[0]=g_i2cReceiveBuffer[1];
      g_currentSetPoint.ui8[1]=g_i2cReceiveBuffer[2];
      g_currentSetPoint.ui8[2]=g_i2cReceiveBuffer[3];
      g_currentSetPoint.ui8[3]=g_i2cReceiveBuffer[4];
      break;
    }
    case MSGTYPE_JOG:{
      g_currentState=STATE_BASICVELOCITY;
      g_trajectories.deleteTrajectories();
      g_trajectories.setLastEvaluation(g_evaluatedMotorPosition);
      g_jogSpeed.ui8[0]=g_i2cReceiveBuffer[1];
      g_jogSpeed.ui8[1]=g_i2cReceiveBuffer[2];
      g_jogSpeed.ui8[2]=g_i2cReceiveBuffer[3];
      g_jogSpeed.ui8[3]=g_i2cReceiveBuffer[4];
      g_currentJogStartDate_us=micros();
      g_currentJogStartPosition=g_evaluatedMotorPosition;
      break;
    }
    case MSGTYPE_APPENDTRAJ:{
      uint8_t currentPointInTraj;
      uint8_t totalPointsInTraj;
      ui16_ui8 trajId;
      uint8_t trajType;
      i32_ui8 currentPosition;
      ui32_ui8 currentTime;
      currentPointInTraj=g_i2cReceiveBuffer[1];
      totalPointsInTraj=g_i2cReceiveBuffer[2];
      trajId.ui8[0]=g_i2cReceiveBuffer[3];
      trajId.ui8[1]=g_i2cReceiveBuffer[4];
      trajType=g_i2cReceiveBuffer[5];
      currentPosition.ui8[0]=g_i2cReceiveBuffer[6];
      currentPosition.ui8[1]=g_i2cReceiveBuffer[7];
      currentPosition.ui8[2]=g_i2cReceiveBuffer[8];
      currentPosition.ui8[3]=g_i2cReceiveBuffer[9];
      currentTime.ui8[0]=g_i2cReceiveBuffer[10];
      currentTime.ui8[1]=g_i2cReceiveBuffer[11];
      currentTime.ui8[2]=g_i2cReceiveBuffer[12];
      currentTime.ui8[3]=g_i2cReceiveBuffer[13];

      if(g_i2cReceiveBuffer[14]==RELATIVE_POSITION){
        currentPosition.i32+=g_trajectories.getLastPosition();
      }
      g_positionsBeingReceived[currentPointInTraj]=currentPosition.i32;
      g_timesBeingReceived[currentPointInTraj]=currentTime.ui32;
      if(currentPointInTraj>=totalPointsInTraj-1){ // Then we received the last point of the trajectory
        g_trajectories.appendTrajectory(g_timesBeingReceived,g_positionsBeingReceived,totalPointsInTraj,trajType,trajId.ui16);
        g_curentActiveTraj.ui16=g_trajectories.getActiveTrajectoryId();
      }
      break;
    }
    case MSGTYPE_REBOOTSLAVE:
      rp2040.reboot();
    default:
      break;
  }
  memset(g_i2cReceiveBuffer,0,size); // Empty the receive buffer
}

void updateSendBuffer(){

  g_i2cSendBuffer[0]=g_currentMotorPulses.ui8[0];
  g_i2cSendBuffer[1]=g_currentMotorPulses.ui8[1];
  g_i2cSendBuffer[2]=g_currentMotorPulses.ui8[2];
  g_i2cSendBuffer[3]=g_currentMotorPulses.ui8[3];

  g_i2cSendBuffer[4]=g_currentSetPoint.ui8[0];
  g_i2cSendBuffer[5]=g_currentSetPoint.ui8[1];
  g_i2cSendBuffer[6]=g_currentSetPoint.ui8[2];
  g_i2cSendBuffer[7]=g_currentSetPoint.ui8[3];

  g_i2cSendBuffer[8]=g_currentEncoderPosition.ui8[0];
  g_i2cSendBuffer[9]=g_currentEncoderPosition.ui8[1];
  g_i2cSendBuffer[10]=g_currentEncoderPosition.ui8[2];
  g_i2cSendBuffer[11]=g_currentEncoderPosition.ui8[3];

  g_i2cSendBuffer[12]=2-g_trajectories.getNTrajectories(); // The master expects the number of available slots
  
  g_i2cSendBuffer[13]=g_curentActiveTraj.ui8[0];
  g_i2cSendBuffer[14]=g_curentActiveTraj.ui8[1];
  
  g_i2cSendBuffer[15]=uint8_t(g_isMotorOn);

  g_i2cSendBuffer[16]=g_closedLoopEnabled;

  g_i2cSendBuffer[17]=g_nextErrorCode;
  
  g_i2cSendBuffer[18]=g_Kp.ui8[0];
  g_i2cSendBuffer[19]=g_Kp.ui8[1];
  g_i2cSendBuffer[20]=g_Kp.ui8[2];
  g_i2cSendBuffer[21]=g_Kp.ui8[3];
  
  g_i2cSendBuffer[22]=g_Ki.ui8[0];
  g_i2cSendBuffer[23]=g_Ki.ui8[1];
  g_i2cSendBuffer[24]=g_Ki.ui8[2];
  g_i2cSendBuffer[25]=g_Ki.ui8[3];
  
  g_i2cSendBuffer[26]=g_Kd.ui8[0];
  g_i2cSendBuffer[27]=g_Kd.ui8[1];
  g_i2cSendBuffer[28]=g_Kd.ui8[2];
  g_i2cSendBuffer[29]=g_Kd.ui8[3];

  g_i2cSendBuffer[30]=g_maxVelocity.ui8[0];
  g_i2cSendBuffer[31]=g_maxVelocity.ui8[1];
  g_i2cSendBuffer[32]=g_maxVelocity.ui8[2];
  g_i2cSendBuffer[33]=g_maxVelocity.ui8[3];

  g_i2cSendBuffer[34]=uint8_t(g_invertedMotor);
  g_i2cSendBuffer[35]=uint8_t(g_invertedEncoder);

  g_i2cSendBuffer[36]=g_motorStepsPerRev.ui8[0];
  g_i2cSendBuffer[37]=g_motorStepsPerRev.ui8[1];
  g_i2cSendBuffer[38]=g_motorStepsPerRev.ui8[2];
  g_i2cSendBuffer[39]=g_motorStepsPerRev.ui8[3];

  g_i2cSendBuffer[40]=g_encoderStepsPerRev.ui8[0];
  g_i2cSendBuffer[41]=g_encoderStepsPerRev.ui8[1];
  g_i2cSendBuffer[42]=g_encoderStepsPerRev.ui8[2];
  g_i2cSendBuffer[43]=g_encoderStepsPerRev.ui8[3];

  g_i2cSendBuffer[44]=g_errorLimit.ui8[0];
  g_i2cSendBuffer[45]=g_errorLimit.ui8[1];

}

void pio_irq_handler_Enc1(){
    // test if irq 0 was raised
    if (pio0_hw->irq & 1)
        --g_rawEncoder;
    // test if irq 1 was raised
    if (pio0_hw->irq & 2)
        ++g_rawEncoder;
    // clear both interrupts
    pio0_hw->irq = 3;
}

float evaluateEncoderPosition(){
  // The encoder position is returned converted in motor steps
  if(!g_invertedEncoder){
    return g_rawEncoder*float(g_motorStepsPerRev.ui32)/float(g_encoderStepsPerRev.ui32)+g_encoderPositionOffset;
  }else{
    return -g_rawEncoder*float(g_motorStepsPerRev.ui32)/float(g_encoderStepsPerRev.ui32)+g_encoderPositionOffset;
  }
}

void checkErrorLimit(){
  if(!g_closedLoopEnabled || g_errorLimit.ui16==0)
    return;

  if(g_currentEvaluatedPosition.f-g_currentSetPoint.i32>g_errorLimit.ui16 || g_currentEvaluatedPosition.f-g_currentSetPoint.i32<-g_errorLimit.ui16){
    g_closedLoopEnabled=false;
    g_currentState=STATE_BASICPOSITION;
    g_pidController=PIDcontroller(OL_PID_KP,OL_PID_KI,OL_PID_KD,SAMPLE_TIME_US,200,0);
    g_trajectories.deleteTrajectories();
    g_trajectories.setLastEvaluation(g_evaluatedMotorPosition);
    g_currentSetPoint.i32=g_evaluatedMotorPosition;
    g_isMotorOn=false;
    digitalWrite(ENABLE_PIN,LOW);
    g_nextErrorCode=ERRCODE_LIMITABORT;
  }
  return;
}