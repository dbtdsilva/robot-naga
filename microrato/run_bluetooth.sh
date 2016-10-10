pcompile dumb-robot.c rmi-mr32.c bluetooth_comm.c
ldpic32 -w dumb-robot.hex -p $1
pterm -p /dev/rfcomm0
