#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "fs_spi.h"
#include "l6470.h"

// Some constants for my NEMA17 test motor.  These will likely need to be
// adjusted for other motors.  See the STM Evaluation Tool to make setting
// these values easier.
#define MAX_SPD 500.0
#define KVAL_HOLD 0x80
#define KVAL_ACC 0x71
#define KVAL_DEC 0x71
#define KVAL_RUN 0x71
#define INT_SPD 0x18C6
#define ST_SLP 0x16
#define SLP_ACC 0x62
#define SLP_DEC 0x62
#define M_STP 16
#define MOT_ACC 200.0

struct Motor_Group
{
    L6470* p_motA;
    L6470* p_motB;
};

int motorQuery(L6470& mot)
{
    // Display data from the L6470 device represented by [mot]
    // If the device returns either all 1's or no data then
    // it's either disabled or not present.
    
    int result = 0;
    int value = 0;
    int errCode = 0;
    
    printf("\nMotor: %X : ", (unsigned int)&mot);
    
    result = mot.getParam(dSPIN_CONFIG);
    printf("0x%04X\n", result);
    
    if (result >= 0xFFFF or result == 0)
        result = -1;
    else
        result = 0;
    
    if (!result)
    {
        value = mot.getStatus();
        printf("  Status:        0x%04X\n", value);
        value = mot.getConfig();
        printf("  Cfg:           0x%04X\n", value);
        value = mot.isBusy();
        printf("  Busy:          %d\n", value);
        errCode = mot.getError();
        printf("  Error:         %d\n", errCode);
        value = mot.getMicroSteps();
        printf("  Microsteps:    %d\n", value);
        value = mot.getAccel();
        printf("  Accel:         %d\n", value);
        value = mot.getMaxSpeed();
        printf("  MaxSpeed:      %d\n", value);
        value = mot.getParam(dSPIN_KVAL_HOLD);
        printf("  KHLD:          0x%02X\n", value);
        value = mot.getParam(dSPIN_KVAL_RUN);
        printf("  KRUN:          0x%02X\n", value);
        
        
        if (errCode)
            result = -1;
    }
    
    return result;
}

void motorsRamp(Motor_Group& grp, int dir, int stp, int dly)
{
    // Run a ramping pattern through the speeds to MAX_SPD
    // increment by the amount requested in stp.
    
    printf("\n\n  Speed Ramp - Dir: %d\n", dir);
    for(int i=0; i<=MAX_SPD; i+=stp)
    {
        grp.p_motA->run(dir, i);
        grp.p_motB->run(dir, i);
        printf("  Speed: %d\n", i);
        sleep(1);
    }
    sleep(dly);
    grp.p_motA->softStop();
    grp.p_motB->softStop();
}

void motorsMove(Motor_Group& grp, int dist)
{
    // Move both motors the requested distance.
    printf("\n\n  Distance Move -  Steps: %d \n", dist);
    grp.p_motA->move_FS(dist);
    grp.p_motB->move_FS(dist);
}

void motorRun(L6470& mot, int speed)
{
    // Run both motors at the speed requested
    // postive speed for forward.  Negative
    // speed for reverse.
    uint8_t dir = 0;
    if (speed < 0)
    {
        dir = 0;
        speed = abs(speed);
    }
    else
        dir = 1;
    
    mot.run(dir, speed);
}

int checkBusy(Motor_Group& grp)
{
    // Check if either motor is reporting busy and
    // return result
    int bsyA = grp.p_motA->isBusy();
    int bsyB = grp.p_motB->isBusy();
    return (bsyA | bsyB);
}

int main()
{
    L6470 motA(new FS_SPI(900000, "/dev/spidev1.0"));
    L6470 motB(new FS_SPI(900000, "/dev/spidev1.1"));
    motA.initMotion(M_STP, MAX_SPD, MOT_ACC, MOT_ACC);
    motA.initBEMF(KVAL_HOLD, KVAL_RUN, INT_SPD, ST_SLP, SLP_ACC);
    motB.initMotion(M_STP, MAX_SPD, MOT_ACC, MOT_ACC);
    motB.initBEMF(KVAL_HOLD, KVAL_RUN, INT_SPD, ST_SLP, SLP_ACC);
    printf("Created L6470 objects\n");
    

    int result  = 0;
    result = motorQuery(motA);
    if(result < 0) printf("Can't Initialize motA Driver");
    result += motorQuery(motB);
    if(result < 0) printf("Can't Initialize motB Driver");
    if(result < -1)
    {
        printf("No Drivers Found. Aborting!");
        abort();
    }
    
    // Create a motor group for the test functions
    Motor_Group motGrp;
    motGrp.p_motA = &motA;
    motGrp.p_motB = &motB;
    
    // Invert one motor to simulate use as robot
    // differential drive.
    motA.invert(1);
    motA.hardHiZ();
    motB.hardHiZ();

    printf("\n\nBegin Motor Driver Testing.\n");
    while(1)
    {
        // Ramp speed from zero to MAX_SPD value on both motors in both
        // directions.
        sleep(5);
        motorsRamp(motGrp, 1, 25, 5);
        
        sleep(5);
        motorsRamp(motGrp, 0, 25, 5);
        
        
        
        // Reset the motor position and perform distance move with both
        // Wait for the motion to complete before moving on.
        sleep(5);
        motA.setPosition(0);
        motB.setPosition(0);
        printf("\n\n  Distance Movement\n  Pos: [%d,%d]\n",
               motA.getPosition_FS(), motB.getPosition_FS());
        
        motorsMove(motGrp, 5000);
        printf("   Busy - waiting");
        while(checkBusy(motGrp))
        {
            printf(".");
            fflush(stdout);
            sleep(1);
        }
        printf("!\n");
        printf("  Pos: [%d,%d]\n", motA.getPosition_FS(), motB.getPosition_FS());
        motA.softHiZ();
        motB.softHiZ();
        
        
        // Move both motors back to home position.  Wait until complete.
        sleep(5);
        printf("\n\n  Goto Home Postion");
        motA.gotoHome();
        motB.gotoHome();
        while(checkBusy(motGrp))
        {
            printf(".");
            fflush(stdout);
            sleep(1);
        }
        printf("!\n");
        printf("  Pos: [%d,%d]\n", motA.getPosition_FS(), motB.getPosition_FS());
        motA.softHiZ();
        motB.softHiZ();
        
    }

    return 0;
}

/*
 Copyright (C) 2013 Kyle Crane
 
 Permission is hereby granted, free of charge, to any person obtaining a copy of
 this software and associated documentation files (the "Software"), to deal in
 the Software without restriction, including without limitation the rights to
 use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 of the Software, and to permit persons to whom the Software is furnished to do
 so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */
