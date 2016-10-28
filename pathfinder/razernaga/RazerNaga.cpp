//
// Created by myrddin on 28/10/16.
//

#include "RazerNaga.h"
#include <libRobSock/RobSock.h>

using namespace std;

#define RUN        1
#define STOP       2
#define WAIT       3
#define RETURN     4
#define FINISHED   5

RazerNaga::RazerNaga(int &argc, char* argv[]) : RazerNaga(argc, argv, 0)
{
}

RazerNaga::RazerNaga(int &argc, char* argv[], int position) : RazerNaga(argc, argv, position, "localhost")
{
}

RazerNaga::RazerNaga(int &argc, char* argv[], int position, string host) :
        RazerNaga(argc, argv, position, host, {0.0, 60.0, -60.0, 180.0})
{
}

RazerNaga::RazerNaga(int &argc, char* argv[], int position, string host, vector<double> ir_sensor_angles) :
        QApplication(argc,argv), name("RazerNaga"), grid_position(position), host(host),
        ir_sensor_angles(ir_sensor_angles)
{
    if (InitRobot2(const_cast<char *>(name.c_str()), grid_position, &ir_sensor_angles[0],
                   const_cast<char *>(host.c_str())) == -1)
    {
        throw runtime_error("Failed to connect robot");
    }
    qApp->addLibraryPath("libRobSock");
    QObject::connect((QObject *)(Link()), SIGNAL(NewMessage()), this, SLOT(act()));
}

void DetermineAction(int beaconToFollow, double *lPow, double *rPow)
{
    static int counter=0;

    static double left, right, center;
    bool   beaconReady;
    static struct beaconMeasure beacon;
    static int    Ground;
    static bool   Collision ;

    /*Access to values from Sensors - Only ReadSensors() gets new values */
    if(IsObstacleReady(LEFT))
        left      = GetObstacleSensor(LEFT);
    if(IsObstacleReady(RIGHT))
        right     = GetObstacleSensor(RIGHT);
    if(IsObstacleReady(CENTER))
        center    = GetObstacleSensor(CENTER);

    beaconReady = IsBeaconReady(beaconToFollow);
    if(beaconReady) {
        beacon    = GetBeaconSensor(beaconToFollow);
    }

    if(IsGroundReady())
        Ground    = GetGroundSensor();
    if(IsBumperReady())
        Collision = GetBumperSensor();

    if(center>4.5 || right>4.5 || left>4.5 || Collision) { /* Close Obstacle - Rotate */
        if(counter % 400 < 200) {
            *lPow=0.06;
            *rPow=-0.06; }
        else {
            *lPow=-0.06;
            *rPow=0.06; }
    }
    else if(right>1.5) { /* Obstacle Near - Avoid */
        *lPow=0.0;
        *rPow=0.05;
    }
    else if(left>1.5) {
        *lPow=0.05;
        *rPow=0.0;
    }
    else {
        if(beaconReady && beacon.beaconVisible) {
            if(beacon.beaconDir>20.0) { /* turn to Beacon */
                *lPow=0.0;
                *rPow=0.1;
            }
            else if(beacon.beaconDir<-20.0) {
                *lPow=0.1;
                *rPow=0.0;
            }
            else { /* Full Speed Ahead */
                *lPow=0.1;
                *rPow=0.1;
            }
        }
        else { /* Full Speed Ahead */
            *lPow=0.1;
            *rPow=0.1;
        }
    }

    counter++;
}

void RazerNaga::act() {
    printf("Act...\n");
    static int state=STOP,stoppedState=RUN;
    double lPow, rPow;

    ReadSensors();

    if(GetFinished()) /* Simulator has received Finish() or Robot Removed */
    {
        exit(0);
        return;
    }
    if(state==STOP && GetStartButton()) state=stoppedState;  /* Start     */
    if(state!=STOP && GetStopButton())  {
        stoppedState=state;
        state=STOP; /* Interrupt */
    }

    switch (state) {
        case RUN:    /* Go */
            if( GetVisitingLed() ) state = WAIT;
            if(GetGroundSensor()==0) {         /* Visit Target */
                SetVisitingLed(true);
                printf("%s visited target at %d\n", name, GetTime());
            }

            else {
                DetermineAction(0,&lPow,&rPow);
                DriveMotors(lPow,rPow);
            }
            break;
        case WAIT: /* Wait for others to visit target */
            SetReturningLed(true);
            if(GetVisitingLed()) SetVisitingLed(false);
            if(GetReturningLed()) state = RETURN;
            DriveMotors(0.0,0.0);
            break;
        case RETURN: /* Return to home area */
            if(GetVisitingLed()) SetVisitingLed(false);
            SetReturningLed(false);

            // Wander
            DetermineAction(1,&lPow,&rPow);
            DriveMotors(lPow,rPow);

            break;

    }
}