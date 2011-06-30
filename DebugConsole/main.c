//
//  main.c
//  DebugConsole
//
//  Created by Eytan Moudahi on 11-05-24.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#include <stdio.h>
#include "ppRobotics.h"
#include "ppRobotics_utest.h"
#include "ppRobotics_math.h"
int main (int argc, const char * argv[])
{
    // insert code here...
    if (PPTestTransformation())
    {
        printf("PPTestDHTable passed\n");
    }
    else
    {
        printf("PPTestDHTable failed\n");
    }
    if (PPTestCreateContext())
    {
        printf("PPTestCreateContext passed\n");
    }
    else
    {
        printf("PPTestCreateContext failed\n");
    }
    
    return FALSE;
}

