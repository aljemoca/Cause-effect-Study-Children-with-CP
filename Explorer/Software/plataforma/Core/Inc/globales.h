/*
 * globales.h
 *
 *  Created on: Dec 22, 2022
 *      Author: alber
 */

#ifndef INC_GLOBALES_H_
#define INC_GLOBALES_H_

#include "main.h"
/****************** General***********************/

#define TIEMPO_MOV 30000
#define TIEMPO_ROT 3000
#define TIEMPO_ATRAS 3000

#define DISTANCIA_OBSTACULO_1  55
#define DISTANCIA_OBSTACULO_2  65

#define IZQUIERDO 0
#define DERECHO 1
/********************HARDWARE****************/

#define LEDY_OFF 0
#define LEDY_ON 1
#define LEDY_ON_1_DE_2 2
#define LEDY_ON_1_DE_5 3
#define LEDG_OFF 0
#define LEDG_ON 0x10
#define LEDG_ON_1_DE_2 0x20
#define LEDG_ON_1_DE_5 0x30
#define LEDS_ERROR 0x77
#define LEDY_BLINK_SLOW LEDY_ON_1_DE_2
#define LEDY_BLINK_FAST LEDY_ON_1_DE_5
#define LEDG_BLINK_SLOW LEDG_ON_1_DE_2
#define LEDG_BLINK_FAST LEDG_ON_1_DE_5



#define TAM_BUFFER_PULSADOR 2


/*************** Defines de COMUNICACIONES ************/


//Definiciones relativas al paquete de comunicaciones básico. En la versión 1.0 sólo estarían
//configurados los comandos de Stop, Go, OBSTACULO

/*
 *    Comandos
 *
 *    ----------------------
 *    |    |             | P|
 *    ----------------------
 *    Bits 7:4-> Comando 4:Stop; 1:Vel; 2:Tiempo; $F: GO; 3:Obstaculo;6: Inactivo; $A0: OK; $C0: Status; $50: NOK
 *    Bits 3:1-> Para comando Stop,obstaculo 000
 *                   Para Vel, Tiempo:  000: Low; 001:High
 *    Bit 0 -> Paridad par
 *
 * */
#define TAM_BUF_COMUNICACIONES 4

#define BT_CONNECTED 1
#define BT_DISCONNECTED 0
#define BT_COMANDO_STOP 0x40
#define BT_COMANDO_GO 0XF0
#define BT_COMANDO_VEL 0X10
#define BT_COMANDO_TIEMPO 0x20
#define BT_COMANDO_OBSTACULO 0X30
#define BT_COMANDO_OK 0xA0
#define BT_COMANDO_NOK 0X50
#define BT_COMANDO_INACTIVO 0X60
#define BT_STATUS 0XC0

#define EVENTO_ESTADO 0
#define EVENTO_PULSADOR 1
#define EVENTO_MOVIMIENTO 2
/**************************************************/


/*********************************************
 *   Variables temporales de MAIN.C
 * ******************************************/

//Estructura que contiene las variables temporales para el SYSTICK
struct str_tick{
	uint32_t t_med_hall[2];
	uint32_t tiempo;
	uint32_t tiempo_backward;
	uint32_t t_blink[2];
	uint32_t t_rebote;
	uint32_t t_rebote_pul_eco;
	uint32_t t_dac;
	uint16_t tiempo_med;
	uint16_t tiempo_temporal_med;
} tick;
/*
 * Estructura que contiene parámetros de navegación, velocidad de los motores y
 * distancia para los sensores eco. Se inicializa en el fichero main.c en la
 * función void InicializaVariables(void), pero las constantes están en los
 * defines de este fichero. Todas las unidades temporales están en ms.
 * */
struct Param{
      uint32_t tiempo;
      uint32_t tiempo_rotate;
      uint32_t tiempo_atras;
      int16_t vel;
      uint16_t obs_near;
      uint16_t obs_far;
//    uint16_t obs_back;
} par;

/*********************************************
 *   Audio
 * ******************************************/
#define AUDIO_SILENCIO 3
#define AUDIO_MOTOR 4
#define AUDIO_FRENAZO 5
#define AUDIO_BEEPATRAS 2
#define AUDIO_CLAXON 1
#define N_track_max 5

void GestorAudio(void);
//int Play(uint8_t track);
uint8_t Play(uint8_t track);
uint8_t ResetMP3(void);
void PreparaPaquete(uint8_t command, uint16_t param);
uint16_t checksum(uint8_t *buffer);
uint8_t VolumeMP3(uint16_t val);
//uint8_t GeneraPulsoTrack(void);


struct gaudio
{
	uint32_t tiempo;
	uint8_t buf[11];
	uint8_t uart_tx_busy;
	//uint8_t track;
	uint8_t np;
} audio;


/*********************************************
 *   Motores
 * ******************************************/


//Ajustar las constantes
#define Kp 200.0  //Antes 400
#define Kd 0.0   //Antes 10
#define Ki	50.0  //Antes 50.0   200
#define VEL 400
#define VELR 300
#define VEL_MAX 1200
//Antes estaba a 1.1, después a 0.9 y ahora a 0.8
#define consigna_vmax 1.0
#define consigna_vmin 0.8

float consigna[2];

int16_t  vel[2];
uint8_t marchag[2];

struct tm{
	uint32_t t_delay_pid[2];
	uint32_t t_arranque[2];
} tmotor;

struct st_pid
{
/*	float kp;
	float kd;
	float ki;*/
	float ep;
	float ed;
	float ei;
} pid[2];

void ResetPID(uint8_t);
int16_t Algoritmo_PID(float vel_medida, float vel_consigna , uint8_t);


/*********************************************
 *   Sensores
 * ******************************************/
#define TAM_BUFFER_HALL 5
#define TAM_BUFFER_ECO 5
//Buffer tamaño potencia de dos
#define TAM_BUF_TIMER 8
#define UMBRAL_GIRO 0.1F


/*La estructura inferior contiene las medidas obtenidas de todos los sensores*/
struct str_sensor{
      uint32_t median_hall[2];      //Mediana de los tiempos medidos de los sensores hall calculado del buffer
      float frequency[2];	//Valores en flotante de la frecuencia de rotación de las ruedas
      uint32_t median_eco[5];   //Valores de las distancia de los sensores de eco
      uint16_t obs;	//Palabra con datos para identificar distintos niveles de posición de los obstáculos
} sensor;

uint8_t  medida_nueva_hall[2]; //Se ativan cuando hay medida nueva en los sensores hall
uint32_t hall[2];  // Tiempos medidos entre flancos de los sensores hall

/*********************************************
 *   Navegacion
 * ******************************************/
#define ST_INACTIVE 0
#define ST_ACTIVE 1

#define ST_FORWARD_MOVEMENT 2
#define ST_BACKWARD_MOVEMENT 3
#define ST_BACKWARD_GIRO 4
#define ST_BACKWARD_PARADA 6
#define ST_ERROR 7
#define ST_TRANSITION 5

//Antes 100
#define TIEMPO_TRANSICION 100

void GestorMotores(void);
void GestorComunicaciones(void);
void GestorSensores(void);
void GestorInterfaz(void);
void Navegacion(void);
void ActualizaVelocidad( int16_t  izq, int16_t der);
void DetectaMovimiento(void);
void TransmiteEvento(uint8_t tipo,uint8_t state);

struct tipo_estado
{
	uint8_t current;
	uint8_t next;
	uint32_t tiempo_estado;
} state;


/*********************************************
 *   Comunicaciones
 * ******************************************/
/*************** Defines de COMUNICACIONES ************/


//Definiciones relativas al paquete de comunicaciones básico. En la versión 1.0 sólo estarían
//configurados los comandos de Stop, Go, OBSTACULO

/*
 *    Comandos
 *
 *    ----------------------
 *    |    |             | P|
 *    ----------------------
 *    Bits 7:4-> Comando 4:Stop; 1:Vel; 2:Tiempo; $F: GO; 3:Obstaculo;6: Inactivo; $A0: OK; $C0: Status; $50: NOK
 *    Bits 3:1-> Para comando Stop,obstaculo 000
 *                   Para Vel, Tiempo:  000: Low; 001:High
 *    Bit 0 -> Paridad par
 *
 * */
#define TAM_BUF_COMUNICACIONES 4

#define BT_CONNECTED 1
#define BT_DISCONNECTED 0
#define BT_COMANDO_STOP 0x40
#define BT_COMANDO_GO 0XF0
#define BT_COMANDO_VEL 0X10
#define BT_COMANDO_TIEMPO 0x20
#define BT_COMANDO_OBSTACULO 0X30
#define BT_COMANDO_OK 0xA0
#define BT_COMANDO_NOK 0X50
#define BT_COMANDO_INACTIVO 0X60
#define BT_STATUS 0XC0

#define EVENTO_ESTADO 0
#define EVENTO_PULSADOR 1
#define EVENTO_MOVIMIENTO 2
/**************************************************/


struct str_com
{
	uint8_t bt_command;
	uint8_t bt_state;
	uint8_t  buf[TAM_BUF_COMUNICACIONES][11];  //Para la transmisión de datos. Podría darse el caso de que se acumen hasta 3 paquetes para transmitir
	uint8_t  pr,pw;  //Indices pasra identificar que parte del buffer se está leyendo durante el proceso de transmisión y está
			//disponible para depositar o escribir un nuevo paquete
	uint8_t np_buffer; //Indica el número de paquetes almacenados en buf (máximo 4, mínimo 0)
	uint8_t uart_tx_busy;
}com;


/*********************************************
 *   Hardware
 * ******************************************/


struct hardware{
	uint8_t user_input;
	uint8_t user_input_old;
	uint8_t leds;
	uint8_t n_puls;
	uint8_t dis_eco;
	uint8_t pulsador_dis_eco_old;
	uint8_t pulsador_dis_eco;
} hard;

#endif /* INC_GLOBALES_H_ */
