/*
 * comunicaciones.c
 *
 *  Created on: Nov 23, 2022
 *      Author: alber
 */



#include "globales.h"





//Definiciones relativas al paquete de comunicaciones básico. En la versión 1.0 sólo estarían
//configurados los comandos de Stop, Go, OBSTACULO

/*
 * 	Comandos
 *
 * 	----------------------
 * 	|		| 		  | P|
 * 	----------------------
 * 	Bits 7:4-> Comando 4:Stop; 1:Vel; 2:Tiempo; $F: GO; 3:Obstaculo; $A0: OK; $C0: Status; $50: NOK
 * 	Bits 3:1-> Para comando Stop,obstaculo 000
 * 			   Para Vel, Tiempo:  000: Low; 001:High
 * 	Bit 0 -> Paridad par
 *
 * */
void InicializaBuffersComunicaciones(void);
void GestorComunicaciones(void);
void TransmitePaquete(uint8_t *buf);
void TransmiteEvento(uint8_t tipo,uint8_t state);
uint8_t paridad_par (uint8_t dato);
void Bin2Ascii (uint16_t num, uint8_t *cad);
uint8_t LonCad(uint8_t *cad);


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/*void TransmitePaquete(uint8_t *buf)
{
	uint8_t lon=LonCad(buf);
	//HAL_UART_Transmit(&huart1, buf, lon,50 );
	HAL_UART_Transmit_DMA(&huart1, buf, lon);
	//HAL_UART_Transmit(&huart2, buf, lon,50 );
}*/


void InicializaBuffersComunicaciones(void)
{
	/*
	 * Inicializa buffers de comunicaciones para transmisión de datos bluetooth
	 * y configuración del reproductor de audio.
	 * */

	/*******************************/
	/*	Existen cuatro buffers de transmisión gestionados a través de los
	 * punteros de lectura, escritura. El número de paquetes en cola pueden llegar
	 * hasta 4, y eso está indicado por la variable np. Los paquetes tienen
	 * el siguiente formato
	 *
	 * Sv,tiempo,x
	 *
	 * v es un valor asociado al primer carácter, que puede ser S, state, M, movement
	 * or P, press. tiempo es un variable temporal en múltiplos de 100ms.
	 *
	 * La recepción es más simple, pues sólo recibe un byte que se guarda en
	 * bt_command
	 * */
	com.pr=0;
	com.pw=0;
	com.uart_tx_busy=0;
	com.bt_command=0;
	com.np_buffer=0;
	/********************************/
	//Esta variable indica que el buffer de tx del audio está ocupado
	audio.uart_tx_busy=0;
}


void GestorComunicaciones(void)
{
	/* Función no bloqueante que permite la gestión de las comunicaciones inalámbricas
	 * y del audio.
	 *
	 *
	 *
	 * */
	uint8_t lon;
	static uint8_t primera;
	if(primera==0)
	{
		/*Esta parte se ejecuta solo una vez, e inicializa las variables además
		 * de activar la interrupción por recepción.*/
		InicializaBuffersComunicaciones();
		HAL_UART_Receive_IT(&huart1, &com.bt_command, sizeof(uint8_t));
		primera=1;
	}

	/*Para la gestión de la transmisión de datos. Si la UART no está transmitiendo
	 * y la COLA de transmisión contiene paquetes, se calcula el tamaño de los mismos
	 * y se transmite por DMA, volviendo a poner activo el banderín de UART ocupada
	 * El puntero com.pr determina cual es el paquete a transmitir. Cuando se haya
	 * completado la transmisión, salta la rutina de interrupción de TX que se encarga
	 * de borrar el banderín e incrementar el com.pr hacia el siguiente paquete.*/
	if(!com.uart_tx_busy)
	{
		if(com.np_buffer>0)
		{
			lon=LonCad(com.buf[com.pr]);
			HAL_UART_Transmit_DMA(&huart1, com.buf[com.pr], lon );
			com.uart_tx_busy=1;
		}
	}

	/*Igual que el if anterior, pero con np de la estructura audio*/
	if(!audio.uart_tx_busy)
	{
		if(audio.np>0)
		{
			HAL_UART_Transmit_DMA(&huart3,audio.buf, 10);
			audio.uart_tx_busy=1;
		}
	}

	//Analizamos continuamente el estado de la conexión Bluetooth
	com.bt_state= HAL_GPIO_ReadPin(BT_State_GPIO_Port, BT_State_Pin);
}

void TransmiteEvento(uint8_t tipo,uint8_t state)
{
	/**********************************************
	 * Esta función recibe el tipo de mensaje que tiene que generar:
	 * 0: S (Estado de la máquina de estados de navegación)
	 * 1: P (Pulsación del conmutador)
	 * 2: M (Estado de movimiento)
	 *
	 * Formato enviado:  Tn,timex
	 * El primer caracter T puede ser S, P o M
	 * El segundo, n, depende del tipo. Para P puede ser 0 o 1 indicando que está pulsado o liberado
	 * para M puede ser 0 o 1 para identificar si la plataforma está parada o en movimiento
	 * mientras que para S puede ser 0, 1, 2 ,... dependiendo del estado
	 * El tercero es time, que muestra el tiempo transcurrido desde que se inicio el experimento
	 * con la plataforma expresado en múltiplos de  100ms.
	 * Ejemplo: P0,100x , S2,23x ,....
	 *
	 */
      uint8_t lon,i;

      uint8_t tiempochar[8];   //Tiempo transcurrido  en Asci
      if(com.np_buffer < 4 )
      {
    	  //Hay una COLA de 4 paquetes para transmitir. El número de datos en la COLA
    	  //viene dado por np_buffer
          Bin2Ascii (tick.tiempo_med, tiempochar); //Convertimos tiempo en ASCII
          //Se prepara el paquete a transmitir que se guarda en la posicion
          //pw de la COLA
          lon=LonCad(tiempochar);
          switch(tipo)
          {
            case 0: com.buf[com.pw][0]='S'; break;
            case 1: com.buf[com.pw][0]='P'; break;
            case 2: com.buf[com.pw][0]='M'; break;
          }
          com.buf[com.pw][1]='0'+state;
          com.buf[com.pw][2]=',';
          for(i=0;i<lon;i++)
              com.buf[com.pw][2+lon-i] = tiempochar[i];
          com.buf[com.pw][lon+3]='x';
          //buf[pw][lon+4]=' ';

          com.buf[com.pw][lon+4]=0;
          //Se procede a incrementar el puntero de escritura hacia la siguiente posicion
          //de la COLA
          com.pw = (com.pw+1) & (TAM_BUF_COMUNICACIONES-1);
          com.np_buffer++;
      }
}
/*
void TransmiteEstado(uint8_t tipo,uint8_t state)
{
	uint8_t  buf[11];
	uint8_t lon,i;
	char tiempochar[8];
	Bin2Ascii (tick.tiempo_med, tiempochar);
    lon=LonCad(tiempochar);
    if(tipo==0)

    	buf[0]='S';
    else
    	buf[0]='P';

    buf[1]='0'+state;
	buf[2]=',';
    for(i=0;i<lon;i++)
		  buf[2+lon-i] = tiempochar[i];
	buf[lon+3]='x';
	buf[lon+4]=0;
	TransmitePaquete(buf);
}*/



uint8_t paridad_par (uint8_t dato)
{
	uint8_t i,contador;

	contador=0;
	for(i=0;i<8;i++)
	{
		if(dato & 1)
			contador++;
		dato>>=1;
	}

	return (contador%2);
}



void Bin2Ascii (uint16_t num, uint8_t *cad)
{
	uint16_t resto , coc;
	uint16_t pos = 0;
	do{
		coc = num/10;
		resto = num%10;
		cad[pos] = resto+'0';
		pos+=1;
		num=coc;
	}while(coc !=0);
	cad[pos]=0;
}

uint8_t LonCad(uint8_t *cad)
{
	uint8_t lon=0;
	while(cad[lon]!=0)
		lon++;
	return lon;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	/*Cada vez que se recibe un dato por el puerto serie, se almacena en la
	 * variable bt_command*/
	if(huart->Instance == USART1)
	{
		HAL_UART_Receive_IT(huart, &com.bt_command, sizeof(uint8_t));
	}
}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef  *huart){
	/*
	 * Cuando se completa la transmisión de un paquete, salta esta interrupción
	 * Aquí se desactiva el banderín de uart_tx_busy, se incrementa módulo 4 el pr
	 * y se decrecemnta el número de elementos que hay almacenados en la COLA
	 * */
      if(huart->Instance == USART1)
      {
            //if(np_buffer>0)
            //{
                com.uart_tx_busy=0;
            	com.np_buffer--;
                com.pr= (com.pr+1)&(TAM_BUF_COMUNICACIONES-1);
            //}
        //    TransmitePaquete();

          //HAL_UART_Transmit_IT(huart, &bt_command, sizeof(uint8_t));

      }

      if(huart->Instance == USART3)
      {
    	  audio.uart_tx_busy=0;
    	  audio.np=0;
      }

}

