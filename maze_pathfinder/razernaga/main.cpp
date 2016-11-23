//
// Created by Diogo Silva on 26/10/16.
//

#include <iostream>
#include <signal.h>
#include "RazerNaga.h"

using namespace std;

// This way to catch Unix signals was found in
// https://gist.github.com/azadkuh/a2ac6869661ebd3f8588
void catch_unix_signals(const std::vector<int>& quitSignals,
                      const std::vector<int>& ignoreSignals = std::vector<int>()) {
    auto handler = [](int sig) ->void {
        printf("\nquit the application (user request signal = %d).\n", sig);
        QCoreApplication::quit();
        exit(0);
    };

    for (int sig : ignoreSignals)
        signal(sig, SIG_IGN);

    for (int sig : quitSignals)
        signal(sig, handler);
}

int main(int argc, char** argv) {
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
                "SYNOPSIS: razernaga [-host hostname] [-robname robotname] [-pos posnumber]" << endl;
        return 1;
    }

    RazerNaga app(argc, argv, rob_id, host);
    catch_unix_signals({SIGQUIT, SIGINT, SIGTERM, SIGHUP});

    return app.exec();
}
