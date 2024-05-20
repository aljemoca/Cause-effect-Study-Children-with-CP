/*
 * navegacion.c
 *
 *  Created on: Dec 15, 2022
 *      Author: alber
 */
/*#include "main.h"
#include "motores.h"
#include "navegacion.h"
#include "sensores.h"
#include "comunicaciones.h"
#include "definiciones.h"
#include "hardware.h"
*/

#include "globales.h"


void Navegacion(void);
void ActualizaEstados(void);
void MaquinaEstados(void);

void Navegacion(void)
{
	static uint8_t inicio;

	if(inicio==0)
	{
		/* Solo se ejecuta de forma incicial
		 * Actualiza el estado de partida y futuro
		 * Trasmite el estado actual
		 * */
		state.current=ST_INACTIVE;
		state.next = ST_INACTIVE;
		TransmiteEvento(EVENTO_ESTADO,state.current);
		inicio=1;
		state.tiempo_estado=0;
	}


	if(state.tiempo_estado > TIEMPO_TRANSICION)
	{
		state.tiempo_estado = 0;
		MaquinaEstados();
	}


}


void MaquinaEstados(void)
{
	static uint8_t dir;


	switch(state.current)
	{
		/*En el INACTIVE, esperamos conexión de bluetooth. Si el Bluetooth
		 * se desconecta, este es el estado final. En este estado, el led
		 * verde está apagado y el amarillo parpadea a distinto ritmo si
		 * el bluetooth está o no conectado. Una vez conectado, la recepción
		 * del comando GO desde la aplicación remota, hace que se transite
		 * al estado ACTIVE*/
		case ST_INACTIVE:
			ActualizaVelocidad(0,0);
			if(com.bt_state == BT_DISCONNECTED)
				hard.leds = LEDY_BLINK_FAST|LEDG_OFF;
			else
			{
				if((sensor.obs&0x1F)  && (!hard.dis_eco))
					hard.leds = LEDY_BLINK_SLOW|LEDG_BLINK_SLOW;
				else
					hard.leds = LEDY_BLINK_SLOW|LEDG_OFF;
				if((com.bt_command & 0xF0) == BT_COMANDO_GO)
				{
					state.next = ST_ACTIVE;
					Play(AUDIO_SILENCIO);
				}
			}
			break;
		case ST_ACTIVE:
			/*
			 * En este estado el bluetooth está conectado y el usuario remoto
			 * permite la operación de la plataforma. Esperamos a que el usuario
			 * de la silla presione el pulsador. Una vez detectado, pasamos al
			 * estado de movimiento. El Led amarillo permanece iluminado, y
			 * el verde parpadeando a baja velocidad.
			 * */
			ActualizaVelocidad(0,0);
			hard.leds = LEDY_ON|LEDG_BLINK_SLOW;
			if(com.bt_state ==BT_DISCONNECTED || com.bt_command == BT_COMANDO_INACTIVO)
			{
				state.next = ST_INACTIVE;
				Play(AUDIO_SILENCIO);
			}
			else
			if(hard.n_puls  > 0)
			{
				state.next = ST_FORWARD_MOVEMENT;
				tick.tiempo=0;
				Play(AUDIO_MOTOR);
			}
			break;
		case ST_FORWARD_MOVEMENT:
			/*
			 * La plataforma se desplaza hacia adelante. Todos los LEDS activos
			 * Pasamos al estado INACTIVE si se desconeta el bluetooth
			 * Volvemos a ACTIVE si se recibe comando STOP o expira el tiempo de navegación
			 * Vamos a BACKWARD_GIRO si detectamos obstáculo delantero o se recibe comando
			 * obstáculo
			 *
			 * */
			ActualizaVelocidad(VEL,VEL);
			hard.leds = LEDY_ON|LEDG_ON;
			if(com.bt_state  == BT_DISCONNECTED || com.bt_command == BT_COMANDO_INACTIVO)
			{
				state.next=ST_INACTIVE;
				Play(AUDIO_SILENCIO);
			}
			else
			if((com.bt_command & 0XF0)== BT_COMANDO_STOP || tick.tiempo >= par.tiempo || ((sensor.obs&0x7) && (sensor.obs&0x18) && (!hard.dis_eco)))
			{
				state.next=ST_ACTIVE;
				Play(AUDIO_SILENCIO);
			}
			else
			if(((sensor.obs&0x7)&&(!hard.dis_eco) )|| ((com.bt_command &0XF0 )== BT_COMANDO_OBSTACULO))
			{
				state.next=ST_BACKWARD_MOVEMENT;
				tick.tiempo_backward=0;
				Play(AUDIO_BEEPATRAS);
			}
			else
				if(sensor.obs&0x700)
					ActualizaVelocidad(VELR,VELR);
				/*
			if(sensor.obs&0x300)
				ActualizaVelocidad(VEL,VELR);
			else
				if( sensor.obs&0x600)
					ActualizaVelocidad(VELR,VEL);
				else
					if( sensor.obs&0x200)
						ActualizaVelocidad(VELR,VELR);*/

			break;
		case ST_BACKWARD_GIRO:
			/*
			 * GIRO TRASERO para esquivar el obstáculo delantero.
			 * Led amarillo en ON y verde parpadeando rápido.
			 * Vamos a INACTIVE si se pierde conexión Bluetooth
			 * Vamos a ACTIVE si se recibe comando STOP o expira el tiempo de navegación
			 * Vamos a FORWARD si se recibe comando OBSTACULO o se detecta obstáculo trasero
			 * Vamos a BACKWARD si expira el tiempo de giro trasero
			 * */



			if(dir)
				ActualizaVelocidad(0,-VELR);
			else
				ActualizaVelocidad(-VELR,0);
			hard.leds = LEDY_ON|LEDG_BLINK_FAST;
			if(com.bt_state == BT_DISCONNECTED || com.bt_command == BT_COMANDO_INACTIVO)
			{
				state.next=ST_INACTIVE;
				Play(AUDIO_SILENCIO);
			}
			else
			if(((com.bt_command & 0XF0)== BT_COMANDO_STOP )|| tick.tiempo >=par.tiempo )
			{
				state.next=ST_ACTIVE;
				Play(AUDIO_SILENCIO);
			}else
			if(((sensor.obs&0x18)&&(!hard.dis_eco)) || ((com.bt_command & 0Xf0) == BT_COMANDO_OBSTACULO))
			{
				state.next=ST_FORWARD_MOVEMENT;
				Play(AUDIO_FRENAZO);
			}else
			if(tick.tiempo_backward >= par.tiempo_rotate)
			{
				state.next = ST_FORWARD_MOVEMENT;
				Play(AUDIO_MOTOR);
			}
			break;
		case ST_BACKWARD_MOVEMENT:
			/*
			 * MOVIMIENTO TRASERO .
			 * Led amarillo en ON y verde parpadeando rápido.
			 * Vamos a INACTIVE si se pierde conexión Bluetooth
			 * Vamos a ACTIVE si se recibe comando STOP o expira el tiempo de navegación
			 * Vamos a FORWARD si se recibe comando OBSTACULO, se detecta obstáculo trasero
			 * o expira el tiempo de navegacion trasera
			 * */
			ActualizaVelocidad(-VEL,-VEL);
			hard.leds = LEDY_ON|LEDG_BLINK_FAST;
			if(com.bt_state == BT_DISCONNECTED || com.bt_command == BT_COMANDO_INACTIVO)
			{
				state.next=ST_INACTIVE;
				Play(AUDIO_SILENCIO);
			}
			else
			if((com.bt_command&0XF0) == BT_COMANDO_STOP || tick.tiempo >=par.tiempo)
			{
				state.next=ST_ACTIVE;
				Play(AUDIO_SILENCIO);
			}else
			if(((sensor.obs&0x18)&&!hard.dis_eco) || ((com.bt_command & 0XF0) == BT_COMANDO_OBSTACULO) )
			{
				state.next=ST_FORWARD_MOVEMENT;
				Play(AUDIO_FRENAZO);
			}
			else
				if (  tick.tiempo_backward>=par.tiempo_atras)
				{
					tick.tiempo_backward=0;
					state.next = ST_BACKWARD_GIRO;
					if(sensor.obs&0x1)
								dir = 1;
							else if (sensor.obs&0x6)
								dir = 0;
					//Play(AUDIO_BEEPATRAS);
				}
			break;
	}
//	state.current=state.next;
	ActualizaEstados();
}

void ActualizaEstados(void)
{

	//<--- OLD --->
	//static uint8_t estado;

	/******************************************************************
	 * Esta función permite regular los cambios de estados de la función
	 * de Navegación y envía los mensajes cada vez que se produce una
	 * transición
	 *
	 * Si se detecta un cambio de estado, porque state.next es diferente
	 * del state.current, se activa un temporizador que, al transcurrir
	 * TIEMPO_TRANSICION, produce la actualización del estado y la transmisión del
	 * mensaje pertinente.
	 *
	 *
	 *****************************************************************/
	if(state.next != state.current)
	{
		state.current=state.next;
		TransmiteEvento(EVENTO_ESTADO,state.current);
		if(state.current==ST_ACTIVE)
			hard.n_puls=0;
		if(state.current!=ST_INACTIVE)
			com.bt_command=BT_COMANDO_GO;
		else
			com.bt_command= BT_COMANDO_STOP;
	}

/*
	switch(estado)
	{
	case 0:
		if(state.next == ST_FORWARD_MOVEMENT && state.current == ST_ACTIVE)
			state.current = state.next;
		if(state.next != state.current)
		{
			estado=1;
		//	state.tiempo_estado=0;
		}
		break;
	case 1:
		if(state.next == state.current)
			estado=0;
		if(state.tiempo_estado >=TIEMPO_TRANSICION)
		{
			estado=0;
			state.current=state.next;
			TransmiteEvento(EVENTO_ESTADO,state.current);
			if(state.current==ST_ACTIVE)
				hard.n_puls=0;
			if(state.current!=ST_INACTIVE)
				com.bt_command=BT_COMANDO_GO;
		}
		break;

	}
*/
}

