pcompile dumb-robot.c rmi-mr32.c
ldpic32 -w dumb-robot.hex -p $1
pterm -p $1
