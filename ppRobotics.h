//
//  ppRobotics.h
//  ppRobotics
//
//  Created by PPG Technologies on 5/20/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#ifndef _PPROBOTICS_H_
#define _PPROBOTICS_H_

#define INPUT
#define OUTPUT
#define TRUE 1
#define FALSE 0
#define DEFTOL 0.0001
#include "ppRobotics.h"


typedef float   PPFloat;
typedef int     PPInt;
typedef double  PPDouble;
typedef PPDouble PPMatrixElement;

typedef enum _PPJointType
{
    prismatic,
    revolute
} PPJointType;

typedef enum _PPError
{
    PPSuccess,
    PPFail
} PPError;

typedef struct _PPJoint
{
    PPDouble offset;
    PPDouble angle;
    PPDouble length;
    PPDouble twist;
} PPJoint;


typedef struct _PPContext
{
    PPJoint **joints;
} PPContext;

PPContext *PPCreateContext(void *joint, ...);

/*
 wrapper methods for cos and sin that accept angles in degrees rather than radians.
 */
PPDouble PPCos(INPUT PPDouble degree);
PPDouble PPSin(INPUT PPDouble degree);

PPError PPGetDHTableFromJoint(INPUT PPMatrixElement dh[][4], 
                              OUTPUT PPJoint *joint);

/*
 returns the joacobian of the system 'A' evaluated at state 'q'
 */
PPError PPGetJacobianAtState(INPUT PPContext *ctx,
                             INPUT PPMatrixElement q[4],
                             INPUT PPDouble t);

#pragma mark -
#pragma mark TODO
#pragma mark * add jacobian method
#pragma mark * add null space
#pragma mark * add sqp method (if required to calculate initial condition)
#pragma mark * add matrix inversion method
#pragma mark * add any methods required for objective function
#pragma mark * add testing methods

#pragma mark - Debug Methods
int PPDoMatricesMatch(INPUT PPMatrixElement dh1[][4], 
                      INPUT PPMatrixElement dh2[][4],
                      INPUT PPDouble tolerance);
int PPDoJointsMatch(INPUT PPJoint *joint1, INPUT PPJoint *joint2);
                    
void PPPrintJoint(INPUT PPJoint *joint);
void PPPrintDHTable(INPUT PPMatrixElement dh[][4]);

#endif