#include <iostream>
#include "RazerNaga.h"
#include <SDL2/SDL.h>

using namespace std;

int main(int argc, char** argv)
{
    string host = "localhost";
    int rob_id = 0;
    cout << "Compiled with Qt Version " << QT_VERSION_STR << endl;
    cout << " RazerNaga Robot " << endl << " Copyright 2002-2016 Universidade de Aveiro" << endl;

    while (argc > 2) {
        if (strcmp(argv[1], "-host") == 0) {
            host = string(argv[2]);
        } else if (strcmp(argv[1], "-pos") == 0) {
            if (sscanf(argv[2], "%d", &rob_id) != 1)
                argc = 0;
        } else {
            break;
        }
        argc -= 2;
        argv += 2;
    }

    if (argc != 1) {
        cerr << "Bad number of parameters" << endl <<
                "SYNOPSIS: GUISample [-host hostname] [-robname robotname] [-pos posnumber]" << endl;
        return 1;
    }

    RazerNaga app(argc, argv, rob_id, host);
    return app.exec();
}
