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
#define DEFTOL 0.00000001
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
    PPJointType type;
    PPDouble offset;
    PPDouble angle;
    PPDouble length;
    PPDouble twist;
} PPJoint;


typedef struct _PPContext
{
    PPJoint **joints;
} PPContext;

#pragma mark - Helper Methods
/*
 wrapper methods for cos and sin that accept angles in degrees rather than radians.
 */
PPDouble PPCos(INPUT PPDouble degree);
PPDouble PPSin(INPUT PPDouble degree);




#pragma mark - Core Methods

PPContext *PPCreateContext(INPUT void *joint, ...);

void PPDestroyContext(INPUT PPContext *ctx);

PPError PPEvaluateJoint(PPJoint *joint, PPDouble q);

PPError PPGetTransformationFromJoint(OUTPUT PPMatrixElement dh[][4], INPUT PPJoint *joint);

PPError PPGetJacobianAtState(INPUT PPContext *ctx, INPUT PPMatrixElement q[4], INPUT PPDouble t);

#pragma mark TODO
#pragma mark * add method to evaluate the entire transformation matrix
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
void PPPrintTransformation(INPUT PPMatrixElement dh[][4]);

#endif