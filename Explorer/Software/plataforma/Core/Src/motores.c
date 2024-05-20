/*
 * motores.c
 *
 *  Created on: Nov 30, 2022
 *      Author: alber
 */


#include "globales.h"
#include "main.h"


void ControlMotores(void);
void GestorMotores(void);
void ActualizaVelocidad( int16_t  izq, int16_t der);


extern DAC_HandleTypeDef hdac;
//extern float consigna;

int16_t mandog[2]; // Para el control de los motores



GPIO_TypeDef *port_disable=GPIOB;
GPIO_TypeDef *port_direction=GPIOA;
GPIO_TypeDef *port_brake=GPIOC;


uint16_t pin_disable_motor[2]={GPIO_PIN_2,GPIO_PIN_1}; //Array con la información de los pines asociados a los pines que manejan el enable de los controladores de motor.
uint16_t pin_direction_motor[2]={GPIO_PIN_12,GPIO_PIN_11}; //Array con la información de los pines asociados a los pines que manejan el enable de los controladores de motor.
uint16_t pin_brake_motor[2]={GPIO_PIN_8,GPIO_PIN_9}; //Array con la información de los pines asociados a los pines que manejan el enable de los controladores de motor.
uint16_t pin_salida_motor[2] = {DAC_CHANNEL_1,DAC_CHANNEL_2};


void ActualizaVelocidad( int16_t  izq, int16_t der)
{

	vel[IZQUIERDO] = izq;
	vel[DERECHO] = der;
	consigna[IZQUIERDO] = (float)izq * consigna_vmax/VEL;
	consigna[DERECHO] = (float)der * consigna_vmax/VEL;

	//mandoglobal[IZQUIERDO]=izq/400;
	//mandoglobal[DERECHO] = der/400;

}


void GestorMotores(void)
{
	/*
	 * Función no bloqueante para el control de los motores
	 * */

	static uint8_t inicio;
	if(inicio==0)
	{
		inicio=1;
		ResetPID(0);
		ResetPID(1);
	}

	/*El Gestor de Motores llama cada 40ms al ControlMotores, que actualiza */
	if(tick.t_dac==40)
	{
		ControlMotores();
		tick.t_dac=0;
	}

}

void ResetPID(uint8_t n)
{
		pid[n].ed=0;
		pid[n].ei=0;
		pid[n].ep=0;
}



int16_t Algoritmo_PID(float vel_medida, float vel_consigna, uint8_t m )
{
	int16_t mando;
	float dif;
	pid[m].ep = vel_consigna-vel_medida;
	dif = pid[m].ep - pid[m].ed;
	mando = (int16_t) (Kp*pid[m].ep + Kd*dif + Ki*pid[m].ei);
	pid[m].ep = dif;
	if(mando <= VEL_MAX )
		pid[m].ei += pid[m].ep;

	if(mando<200)
		mando=200;
	else if(mando >VEL_MAX)
		mando = VEL_MAX ;
	return mando;
}


void ControlMotores(void)
{
	/*Esta función integra el PID con la operación de motor, se ejecuta cada 40ms*/

	#define TIEMPO_DESCONEXION_ARRANQUE 400
	//#define TIEMPO_DESCONEXION 3000
	#define MANDO_TRANSICION 500
	#define PALANTE GPIO_PIN_RESET
	#define PATRAS GPIO_PIN_SET


	/**********************************************************
	 * Información de la máquina de estados que gobierna el control
	 * de motores sin PID
	 * Nomenclatura:
	 * - Sx: nombre de estado
	 * 		Entrada(Valor): Próximo estado /Acción
	 * Nota: i-> 0 motor izquierdo,  1 motor derecho
	 * ********************************************************
	 *  S0: MOTOR PARADO
	 * 		vel[i] (>0) : S1 / t_arranque[i]=0, disable_motor[i] = 1, brake_motor[i]=0 ;
	 * 		vel[i] (<0) : S11 / t_arranque[i]=0, disable_motor[i] = 1, brake_motor[i]=0;
	 *      o.c : S0, disable_motor[i] = 1, brake_motor[i]=0 ;
	 *  S1: MOTOR ESPERA ADELANTE
	 *		vel[i] (==0) : S1 / disable_motor[i] = 1, brake_motor[i]=0 ;
	 *		t_arranque[i] (> Umbral) : S2 / mandog[i] = vel[i], disable_motor[i] = 1, brake_motor[i]=0 ;
	 * 		o.c : S1: disable_motor[i] = 1, brake_motor[i]=0 ;
	 *  S2: MOTOR ADELANTE
	 *  	vel[i] (==0) : S3 / mandog[i] = vel[i], direccion_motor[i]= AVANCE, disable_motor[i] = 0, brake_motor[i]=0 ;
	 * 		vel[i] (<0)  : S12	/ mandog[i] = vel[i], direccion_motor[i]= AVANCE, disable_motor[i] = 0, brake_motor[i]=0 ;
	 * 		o.c			 : S2 / / mandog[i] = vel[i], direccion_motor[i]= AVANCE, disable_motor[i] = 0, brake_motor[i]=0 ;
	 * 	S3: FRENADA
	 * 		sensor.freq[i] (<UMBRAL) : S1 / mandog[i]=0, brake_motor[i]=1;
	 * 		sensor.freq[i] (>UMBRAL) : S3 / mandog[i]=0, brake_motor[i]=1;
	 * 		oc : S3 / mandog[i]=0, brake_motor[i]=1;
	 * 	S11: MOTOR ESPERA ATRAS
	 *		vel[i] (==0) : S1 / disable_motor[i] = 1, brake_motor[i]=0 ;
	 *		t_arranque[i] (> Umbral) : S12 / mandog[i] = -vel[i], disable_motor[i] = 1, brake_motor[i]=0 ;
	 * 		o.c : S11: disable_motor[i] = 1, brake_motor[i]=0 ;
	 *  S12: MOTOR ATRAS
	 *  	vel[i] (==0) : S3 / mandog[i] = -vel[i], direccion_motor[i]= ATRAS, disable_motor[i] = 0, brake_motor[i]=0 ;
	 * 		vel[i] (>0)  : S2	/ mandog[i] = -vel[i], direccion_motor[i]= ATRAS, disable_motor[i] = 0, brake_motor[i]=0 ;
	 * 		o.c			 : S12 / / mandog[i] = -vel[i], direccion_motor[i]= ATRAS, disable_motor[i] = 0, brake_motor[i]=0 ;
	 *
	 *
	 */



	static  uint8_t marcha[2];
	//marchag[0] = marcha[0];
	//marchag[1] = marcha[1];

	uint8_t i;
		//Marcha puede tomar losadelante 0
		//Marcha atrás 1

		for(i=0;i<2;i++)
		{
			switch(marcha[i])
			{
			case 0: if(vel[i]>0)
					{
						marcha[i]=1;
						tmotor.t_arranque[i]=0;
					};
					if(vel[i]<0)
					{
						marcha[i]=11;
						tmotor.t_arranque[i]=0;
					}
					HAL_GPIO_WritePin(GPIOB , pin_disable_motor[i], GPIO_PIN_SET);
					HAL_GPIO_WritePin(port_brake, pin_brake_motor[i], GPIO_PIN_RESET);
					mandog[i]=0;
				break;
			case 1: if(vel[i]==0)
						marcha[i]=0;
					else
						if(tmotor.t_arranque[i]>=TIEMPO_DESCONEXION_ARRANQUE)
						{
							marcha[i]=2;
							ResetPID(i);
							tmotor.t_delay_pid[i]=0;
						}
					HAL_GPIO_WritePin(port_disable, pin_disable_motor[i], GPIO_PIN_SET);
					HAL_GPIO_WritePin(port_direction, pin_direction_motor[i], PALANTE);
					HAL_GPIO_WritePin(port_brake, pin_brake_motor[i], GPIO_PIN_RESET);
					mandog[i]=vel[i];
					break;
			case 2:
				HAL_GPIO_WritePin(port_disable,pin_disable_motor[i],GPIO_PIN_RESET);
				HAL_GPIO_WritePin(port_direction, pin_direction_motor[i], PALANTE);
				HAL_GPIO_WritePin(port_brake, pin_brake_motor[i], GPIO_PIN_RESET);
				/*if(vel[i]<=0)
					marcha[i]=3;*/
				if(vel[i]==0)
					marcha[i]=3;
				if(vel[i]<0)
				{	//marcha[i]=0; //Lo paso por el estado cero para que se rearme los controladores
					marcha[i]=12; //Lo que había antes
					ResetPID(i);
					tmotor.t_delay_pid[i]=0;
				}
				if(vel[i]>0)
				{
					if(tmotor.t_delay_pid[i]<2200)
						mandog[i]=vel[i];
					else
	//				mandog[i]=vel[i];
						mandog[i] = Algoritmo_PID(sensor.frequency[i], consigna[i], i);
				};
				break;

			case 3:
				HAL_GPIO_WritePin(port_brake, pin_brake_motor[i], GPIO_PIN_SET);
				mandog[i]=0;
				if(sensor.frequency[i]< UMBRAL_GIRO)
				{
	//				tick.t_arranque[i]=0;
					//if(vel[i]==0)
						marcha[i]=0;
				/*	if(vel[i]<0)
					{
						marcha[i] = 11;
					}
					if(vel[i]>0)
					{
						marcha[i]=1;
					}*/
				}
				break;
			case 11:
				HAL_GPIO_WritePin(port_direction,pin_direction_motor[i],PATRAS);
				HAL_GPIO_WritePin(port_brake, pin_brake_motor[i], GPIO_PIN_RESET);
				if(tmotor.t_arranque[i]>=TIEMPO_DESCONEXION_ARRANQUE)
				{
					marcha[i]=12;
					ResetPID(i);
					tmotor.t_delay_pid[i]=0;
				}
				if(vel[i]==0)
					marcha[i]=0;
				mandog[i]=-vel[i];
				break;
			case 12:
				HAL_GPIO_WritePin(port_direction,pin_direction_motor[i],PATRAS);
				HAL_GPIO_WritePin(port_disable,pin_disable_motor[i],GPIO_PIN_RESET);
				HAL_GPIO_WritePin(port_brake, pin_brake_motor[i], GPIO_PIN_RESET);
				mandog[i]=-vel[i];
				/*if(vel>=0)
					marcha[i]=3;*/
				if(vel[i]==0)
					marcha[i]=3;
				if(vel[i]>0)
				{
					//marcha[i]=0;  //Probar esto
					marcha[i]=2;
					ResetPID(i);
					tmotor.t_delay_pid[i]=0;
				}
				if(vel[i]<0)
				{
					if(tmotor.t_delay_pid[i]<2000)
						mandog[i]=-vel[i];
					else
	//				mandog[i]=vel[i];
						mandog[i] = Algoritmo_PID(sensor.frequency[i], consigna_vmin, i);
				}
				break;

			};

			//HAL_DAC_SetValue(&hdac, pin_salida_motor[i] ,DAC_ALIGN_12B_R , mandog[i]);
		}

		for(i=0;i<2;i++)
			HAL_DAC_SetValue(&hdac, pin_salida_motor[i] ,DAC_ALIGN_12B_R , mandog[i]);

}
