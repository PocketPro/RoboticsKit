//
//  ppRobotics_utest.c
//  ppRobotics
//
//  Created by PPG Technologies on 5/20/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#include "ppRobotics_utest.h"
#include "ppRobotics.h"
#include <math.h>
#include "stdio.h"

int PPTestTransformation()
{
    PPJoint joint = {revolute, 25,15,10,45};
    PPMatrixElement transformation[4][4];
    
    PPGetTransformationFromJoint(transformation, &joint);
    
    PPPrintJoint(&joint);
    PPPrintTransformation(transformation);
    
    PPMatrixElement transformation_expected[4][4] = {{0.96593, -0.18301, 0.18301, 9.65926},
        {0.25882, 0.68301, -0.68301, 2.58819},
        {0, 0.70711, 0.70711, 25},
        {0,0,0,1}};
    
    if (PPDoMatricesMatch(transformation, transformation_expected, 0.0001))
    {
        return TRUE;
    }
    return FALSE;
}

int PPTestCreateContext()
{
    PPJoint j1 = {revolute, 10,10,10,10};
    PPJoint j2 = {revolute, 20,20,20,20};
    PPJoint *_j1, *_j2;
    PPContext *ctx = PPCreateContext(&j1, &j2, (void*)NULL);
    
    if (ctx == NULL)
    {
        return FALSE;
    }
    
    _j1 = ctx->joints[0]; 
    _j2 = ctx->joints[1];
    PPPrintJoint(_j1);
    PPPrintJoint(_j2);
    
    if (PPDoJointsMatch(&j1, _j1) &&  PPDoJointsMatch(&j2, _j2))
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}