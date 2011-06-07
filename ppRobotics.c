//
//  ppRobotics.c
//  ppRobotics
//
//  Created by PPG Technologies on 5/20/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#include "ppRobotics.h"
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>


PPDouble PPCos(INPUT PPDouble degree)
{
    return cos(degree*M_PI/180.0);
}

PPDouble PPSin(INPUT PPDouble degree)
{
    return sin(degree*M_PI/180.0);
}


PPContext *PPCreateContext(void *joint, ...)
{
    va_list ap;
    PPInt i, count;
    PPJoint **joints;
    PPContext *ctx;

    joints = malloc(sizeof(PPJoint*)*count);
    if (joints == NULL)
    {
        return NULL;
    }
    
    /* get the number of arguments */
    count = 1;
    va_start(ap, joint);
    while( va_arg(ap, void*) != NULL)
    {
        count++;
    }
    
    /* create a new context and population the joints array */
    va_start(ap, joint);
    joints[0] =  (PPJoint*)joint;
    for (i = 1; i < count; i++)
    {
        joints[i] =  va_arg(ap, void*);
    }
    va_end(ap);
    
    ctx->joints = joints;
    return ctx;
}

PPError PPGetDHTableFromJoint(OUTPUT PPMatrixElement dh[][4], INPUT PPJoint *joint)
{
    PPDouble angle = joint->angle;
    PPDouble twist = joint->twist;
    PPDouble offset = joint->offset;
    PPDouble length = joint->length;
    
    dh[0][0] = PPCos(angle);
    dh[0][1] = -PPSin(angle)*PPCos(twist);
    dh[0][2] = PPSin(angle)*PPSin(twist);
    dh[0][3] = length*PPCos(angle);
    dh[1][0] = PPSin(angle);
    dh[1][1] = PPCos(angle)*PPCos(twist);
    dh[1][2] = -PPCos(angle)*PPSin(twist);
    dh[1][3] = length*PPSin(angle);
    dh[2][0] = 0;
    dh[2][1] = PPSin(twist);
    dh[2][2] = PPCos(twist);
    dh[2][3] = offset;
    dh[3][0] = 0;
    dh[3][1] = 0;
    dh[3][2] = 0;
    dh[3][3] = 1;
    
    return PPSuccess;
    
}

PPError PPGetJacobianAtState(INPUT PPContext *ctx,
                             INPUT PPMatrixElement q[4],
                             INPUT PPDouble t)
{
    
}

#pragma mark - Debug Methods
int PPDoMatricesMatch(INPUT PPMatrixElement dh1[][4], 
                      INPUT PPMatrixElement dh2[][4], PPDouble tolerance)
{
    PPInt i, j;
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            if (fabs(dh1[i][j] - dh2[i][j]) > tolerance)
            {
                return FALSE;
            }
        }
    }
    return TRUE;
}

int PPDoJointsMatch(INPUT PPJoint *joint1, INPUT PPJoint *joint2)
{
    if (fabs(joint1->angle - joint2->angle) < DEFTOL
        && fabs(joint1->twist - joint2->twist) < DEFTOL
        && fabs(joint1->offset - joint2->offset) < DEFTOL
        && fabs(joint1->length - joint2->length) < DEFTOL)
    {
        return TRUE;
    }
    return FALSE;
}

void PPPrintJoint(INPUT PPJoint *joint)
{
    printf("{%6.2f,%6.2f,%6.2f,%6.2f}\r\n", 
           joint->offset, joint->angle, joint->length, joint->twist);
}

void PPPrintDHTable(INPUT PPMatrixElement dh[][4])
{
    printf("%8.4f%8.4f%8.4f%8.4f\r\n"
           "%8.4f%8.4f%8.4f%8.4f\r\n"
           "%8.4f%8.4f%8.4f%8.4f\r\n"
           "%8.4f%8.4f%8.4f%8.4f\r\n",
           dh[0][0], dh[0][1], dh[0][2], dh[0][3],
           dh[1][0], dh[1][1], dh[1][2], dh[1][3],
           dh[2][0], dh[2][1], dh[2][2], dh[2][3],
           dh[3][0], dh[3][1], dh[3][2], dh[3][3]);
}