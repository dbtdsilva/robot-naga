#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "rmi-mr32.h"

#define G_ML(val) val >> 4
#define G_L(val) val >> 3
#define G_C(val) val >> 2
#define G_R(val) val >> 1
#define G_MR(val) val >> 0

int parseLine();

int F1; //lado esquerdo
int F2; //lado direito

int parseLine(){
	if (tick20ms==1){
		tick20ms = 0;
		

		F1 = S1;
		F2 = S5;

		for (i = 0; i < 5; i++) {
			groundSensor = readLineSensors(0);
			printInt(groundSensor, 2 | 5 << 16);
			printf("\n");
		}
/*
		if (G_C(groundSensor) == 0) {
			setVel2(40, 40);
		} else if (G_R(groundSensor) == 1) {
			setVel2(0, 10);
		} else if (G_L(groundSensor) == 1) {
			setVel2(10, 0);
		}
		*/
/*
		if S3
			walk;
		else
			if S2
				rotate
			else
				if S4
					rotate
				else
					if S1
						rotate
					else
						if S5
							rotate
						else{
							NOT FOUND
							search
							if F1 or F2
								rotate
							else
								go to front (walk)

	*/					}
							
	}
	return 0;
}


//-------------auxiliares--------------------------

