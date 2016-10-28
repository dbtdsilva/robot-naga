TEMPLATE		= app
TARGET			= razernega
DEFINES     	+= CIBERQTAPP
QT += gui widgets
CONFIG      	+= qt warn_on release thread
#CONFIG			+= qt warn_on debug thread
DEPENDPATH  	+= $$PWD/../cibertools-v2.2.2.rmi/libRobSock
INCLUDEPATH 	+= $$PWD/../cibertools-v2.2.2.rmi/libRobSock

win32 {
    DEFINES += MicWindows
    CONFIG(release, debug|release): LIBS += -L$$PWD/../cibertools-v2.2.2.rmi/libRobSock/release/ -lRobSock
    CONFIG(debug, debug|release): LIBS += -L$$PWD/../cibertools-v2.2.2.rmi/libRobSock/debug/ -lRobSock
}

symbian {
    LIBS += -lRobSock
}

unix {
    LIBS += -L$$PWD/../cibertools-v2.2.2.rmi/libRobSock/ -lRobSock
}

HEADERS	+= robview.h sampapp.h #keyhandler.h
SOURCES += main.cpp robview.cpp
