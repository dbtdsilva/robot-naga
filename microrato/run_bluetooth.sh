set -o xtrace

device=${1:-/dev/ttyUSB0}
file=${2:-dumb-robot}

pcompile $file.c rmi-mr32.c bluetooth_comm.c
ldpic32 -w $file.hex -p $device
pterm -p /dev/rfcomm0
