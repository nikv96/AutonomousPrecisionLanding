#include <stdio.h>
#include <math.h>

#define PI 3.14159
#define epsilon 0.0001

const float tmin = 0;
const float tmax = 2*PI;

void pos(float t);
void vel(float t);
float Speed(float t);
float arcLength(float t);


float x, y, d_y, d_x;

void pos(float t){
	x = 40*cos(t)/(sin(t)*sin(t)+1);
	y = 20*sin(2*t)/(sin(t)*sin(t)+1);
}

void vel(float t){

	d_x = (40*sin(t)*sin(t)*sin(t)-120*sin(t))/((1+sin(t)*sin(t))*(1+sin(t)*sin(t)));
	d_y = (40-120*sin(t)*sin(t))/((1+sin(t)*sin(t))*(1+sin(t)*sin(t)));

}

float Speed(float t){
	return (40/(sqrt(1+sin(t)*sin(t))));	
}

float arcLength(float t){
	float dT = 0.1;
	float integral = 0;
	float time = 0.0;	
	while(time<=t){
		integral += Speed(time)*dT;
		time += dT;
	}

	return integral;
}


float GetCurveParameter(float s){

	float Lmax = arcLength(tmax);
	float t = tmin + s*(tmax-tmin)/Lmax;
	
	float lower = tmin;
	float upper = tmax;
	int i;
	int imax = 100;
	for(i = 0; i < imax; i++)
	{
		float F = arcLength(t) - s;
		if (abs(F) < epsilon)
		{
			return t;
		}

		float DF = Speed(t);
		float tCandidate = t - F/DF;

		if(F>0)
		{
			upper = t;
			if(tCandidate <= lower)
			{
				t = 0.5*(upper+lower);
	
			}
			else
			{

				t = tCandidate;
			}
		}
		else 
		{
			lower = t;
			if (tCandidate >= upper)
			{
				t = 0.5*(upper+lower);
			}
			else 
			{		
				t = tCandidate;
			}
		}
		
	}
	
	return t;

}



int main(void)
{
	printf ("s value: ");
	float s;
	scanf("%f", &s);
	float t = GetCurveParameter(s);
	printf("The corresponding t value is %e\n", t); 	
}





