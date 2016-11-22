int parseSensors();
int map(int x, int in_min, int in_max, int out_min, int out_max);
int bla(int front, int left, int right);
int compute_angleL(int intfront, int left, int right);
double compute_angleR(double front, double left, double right);

typedef struct{
   bool ON;
   double angle;
   bool STOP;
   
 
}  Avoid; 

extern Avoid avoid_obst;


