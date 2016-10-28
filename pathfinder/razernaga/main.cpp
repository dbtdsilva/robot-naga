/*
    This file is part of ciberRatoToolsSrc.

    Copyright (C) 2001-2016 Universidade de Aveiro

    ciberRatoToolsSrc is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    ciberRatoToolsSrc is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/* main.cpp
 *
 * Basic Robot Agent
 * simple version for demonstration
 *
 * For more information about the CiberRato Robot Simulator 
 * please see http://microrato.ua.pt/ or contact us.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <iostream>

#include "RazerNaga.h"

using std::cerr;

#include "libRobSock/RobSock.h"
#include "robview.h"

int main(int argc, char** argv)
{
    char host[100]="localhost";
    char rob_name[20]="GUISample";
    int rob_id=1;
    printf("Compiled with Qt Version %s\n", QT_VERSION_STR);
    printf(" GUISample Robot \n Copyright 2002-2016 Universidade de Aveiro\n");
    fflush(stdout);

    /* processing arguments */
    while (argc > 2) // every option has a value, thus argc must be 1, 3, 5, ...
    {
        if (strcmp(argv[1], "-host") == 0)
        {
            strncpy(host, argv[2], 99);
            host[99]='\0';
        }
        else if (strcmp(argv[1], "-robname") == 0)
        {
            strncpy(rob_name, argv[2], 19);
            rob_name[19]='\0';
        }
        else if (strcmp(argv[1], "-pos") == 0)
        {
            if(sscanf(argv[2], "%d", &rob_id)!=1)
                argc=0; // error message will be printed
        }
        else
        {
            break; // the while
        }
        argc -= 2;
        argv += 2;
    }

    if (argc != 1)
    {
        fprintf(stderr, "Bad number of parameters\n"
                "SYNOPSIS: GUISample [-host hostname] [-robname robotname] [-pos posnumber]\n");
        return 1;
    }

    RazerNaga app(argc, argv);

    // create robot display widget
    //RobView robView(irSensorAngles, rob_name);

    // Connect event NewMessage to handler redrawRobot()
    //QObject::connect((QObject *)(Link()), SIGNAL(NewMessage()), &robView, SLOT(redrawRobot()));

    //robView.show();

    // process events
    return app.exec();
}
