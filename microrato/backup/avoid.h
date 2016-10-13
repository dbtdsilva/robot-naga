int parseSensors();
double map(double x, double in_min, double in_max, double out_min, double out_max);
int bla(double front, double left, double right);
double compute_angleL(double front, double left, double right);
double compute_angleR(double front, double left, double right);

typedef struct{
   bool ON;
   double angle;
   bool STOP;
   
 
}  Avoid; 

extern Avoid avoid_obst;


