/*
 * hardware.c
 *
 *  Created on: Nov 23, 2022
 *      Author: alber
 */

#include "main.h"
#include "globales.h"
//#include "navegacion.h"

void ControlLeds(uint8_t valor);  //Control de los leds. Vea el fichero hardware.h para más detalles
void GestorInterfaz(void);
void ControlPulsador(void);
void ControlHabiliadorEco(void);

GPIO_TypeDef *port_led[2]={Led_amarillo_GPIO_Port,Led_verde_GPIO_Port};
uint16_t pin_led[2] ={Led_amarillo_Pin, Led_verde_Pin};


void GestorInterfaz(void)
{
	/*
	 *  Función no bloqueante que permite la gestión del hardware de interfaz
	 *  de ls Silla Exploradora. En concreto, gestiona la activación de los
	 *  leds y los conmutadores de usuario y de bloqueo de los sensores de
	 *  ultrasonidos.
	 *
	 * */
	static uint8_t primero;
	if(primero==0)
	{
		/*Actualiza variables por primera vez*/

		//Inicializamos los temporizadores para los leds
		tick.t_blink[0]=0;
		tick.t_blink[1]=0;

		/*Apagamos LEDS*/
		HAL_GPIO_WritePin(port_led[0], pin_led[0], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(port_led[0], pin_led[0], GPIO_PIN_RESET);
		primero=1;
		//Actualizamos la variable user_input con el valor del estado del conmutador de usuario
		hard.user_input=HAL_GPIO_ReadPin(User_button_GPIO_Port, User_button_Pin);
		//Iniciamos el tiempo de rebote
		tick.t_rebote=0;

		hard.n_puls=0; //Cuenta el número de pulsaciones de usuario.

		//Iniciamos el tiempo de rebote del pulsador de eliminación de eco
		tick.t_rebote_pul_eco=0;
		//hard.dis_eco=1;
		//Actualizamos la variable hard.dis_eco con el valor real
		hard.dis_eco=HAL_GPIO_ReadPin(Dis_eco_GPIO_Port, Dis_eco_Pin);
	}

	/*
	 * El gestor de interfaz llama a tres funciones no bloqueantes que permiten
	 * el control de los leds, el pulsador y el habilitador de eco.
	 * */
	ControlLeds(hard.leds);
	ControlPulsador();
	ControlHabiliadorEco();  //Sin testar
}

void ControlPulsador(void)
{
	/*
	 * Actualiza la información hard.user_input, hard.user_input_old, y hard.n_puls
	 * de la estructura hard. Cada 200ms se lee el puerto conectado con el pulsador
	 * de usuario. Esto se hace para evitar rebotes. Se actualiza hard.user_input_old
	 * con el valor actual hard.user_input, y éste, a su vez, se actualiza con la
	 * lectura del pin. Si ambos han cambiado, se transmite un EVENTO_PULSADOR, por
	 * el puerto bluetooth y si ese cambio ha sido en un flanco negativo, implica pulsación
	 * y se incrementa la variable hard.n_puls. Esta variable se tiene en cuenta en
	 * la función de Navegación.
	 * */
	if(tick.t_rebote>=200)
	{
		tick.t_rebote=0;
		hard.user_input_old = hard.user_input;
		hard.user_input=HAL_GPIO_ReadPin(User_button_GPIO_Port, User_button_Pin);
		if(hard.user_input != hard.user_input_old)
			TransmiteEvento(EVENTO_PULSADOR,hard.user_input);
		if(hard.user_input ==0 && hard.user_input_old ==1)
			hard.n_puls++;
	}

}

void ControlLeds(uint8_t valor)
{
	uint8_t i;

	/*
	 * Esta función recibe un byte en el el que nibble más significativo
	 * controla el led amarillo y el otro, el verde. En función de los valores
	 * que tenga cada nibble, se gestiona la iluminación de cada led atendiendo
	 * a los siguientes estados
	 *
	 *LED_OFF 0   Apagado
	  LED_ON 1  Encendido
	  LED_ON_1_DE_2 2   Intermitencia 0.5s
	  LED_ON_1_DE_5 3    Intermitencia 0.2s
	 *
	 *Cualquier otra situación, crea una alarma en los LEDS, mostrándose de forma
	 *alternante
	 *
	 * */

	#define LED_OFF 1
	#define LED_ON 0
	#define LED_ON_1_DE_2 2
	#define LED_ON_1_DE_5 3


	for(i=0;i<2;i++)
	{
		valor>>=4*i;
		switch(valor&0xF)
		{
		case LED_OFF:
			HAL_GPIO_WritePin(port_led[i], pin_led[i], GPIO_PIN_RESET);
			break;
		case LED_ON:
			HAL_GPIO_WritePin(port_led[i], pin_led[i], GPIO_PIN_SET);
			break;
  		case LED_ON_1_DE_2:
			if(tick.t_blink[i]>500)
			{
				HAL_GPIO_TogglePin(port_led[i], pin_led[i]);
				tick.t_blink[i]=0;
			}
			break;
		case LED_ON_1_DE_5:
			if(tick.t_blink[i]>200)
			{
				HAL_GPIO_TogglePin(port_led[i], pin_led[i]);
				tick.t_blink[i]=0;
			}
			break;
		default:
			if(tick.t_blink[0]>500)
			{
				  HAL_GPIO_TogglePin(port_led[i], pin_led[0]);
				  tick.t_blink[0]=0;
				  HAL_GPIO_WritePin(port_led[i], pin_led[1], !HAL_GPIO_ReadPin(GPIOC, pin_led[0]));
				  i=2;
			}
			break;
		}
	}

}



void ControlHabiliadorEco(void)
{
	/*
	 * Actualiza el valor del conmutador con el estado del conmutador cada
	 * 200ms.
	 * */
	if(tick.t_rebote_pul_eco>=200)
	{
		tick.t_rebote_pul_eco=0;
		hard.dis_eco = HAL_GPIO_ReadPin(Dis_eco_GPIO_Port, Dis_eco_Pin);
	}
}
