/*
 * Mechanic.h
 *
 *  Created on: May 17, 2023
 *      Author: tucha
 */

#ifndef INC_MECHANIC_H_
#define INC_MECHANIC_H_

//PV
struct Evaluation
{
	float Position;
	float Velocity;
}Setpoint,Measure;

// For Velocity Calculation
uint32_t T_prev = 0;
uint32_t T = 0;
float dT = 0;
float P_prev = 0;

// Time
uint32_t timestamp = 0;

// Read Encoder Pulse
int32_t QEIReadRaw;

// Encoder
void Read_Encoder()
{
		T = HAL_GetTick();
		dT = (T - T_prev)/1000.0;
		Measure.Position = (QEIReadRaw*6.11*2*3.14)/12000.0;
		Measure.Velocity = (Measure.Position - P_prev)/dT;
		P_prev = Measure.Position;
		T_prev = T;
}



#endif /* INC_MECHANIC_H_ */
