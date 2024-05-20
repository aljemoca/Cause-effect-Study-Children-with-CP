/*
 * sensores.c
 *
 *  Created on: Dec 1, 2022
 *      Author: alber
 */

/*
#include "sensores.h"
#include "definiciones.h"
#include "main.h"
#include "comunicaciones.h"
*/

#include "globales.h"


void GestorSensores(void);

//void Actualiza_Buffer_Medidas(void);
uint8_t WriteHallValue(uint8_t channel, uint32_t value);
uint32_t MedianN(uint8_t eco_hall, uint8_t channel, uint8_t tam);
void InicializaBufferUltrasonidos(void);
void InicializaBuffers(void);
uint8_t WriteEcoValue(uint8_t channel, uint16_t value);
void InicializaHallBuffer(void);
uint16_t DetectaObstaculo(void);
void SensoresNivel0(void);



//uint16_t pin_eco_delantero[3]={GPIO_PIN_6,GPIO_PIN_7,GPIO_PIN_8}; //Array que contiene los pines asociados a los ultrasonidos delanteros
uint16_t pin_eco_delantero[3]={GPIO_PIN_7,GPIO_PIN_8,GPIO_PIN_9}; //Array que contiene los pines asociados a los ultrasonidos delanteros
uint16_t pin_eco_trasero[2]={GPIO_PIN_6,GPIO_PIN_7};  //Array que contiene el nombre de los pines asociados a los ultrasonidos traseros

/*Se define una estructura de datos para almacenar los datos que se reciben de
 * los timers cada vez que ocurre una interrupción. Para la lectura de los
 * mismos se definen un puntero de lectura pr_timer, de escritura pw_timer
 * y la variable np_timer, que contiene el número de nuevos datos que se registran*/
struct data_timer
{
	uint16_t value;
	uint32_t valuel;
	uint8_t channel;
	uint8_t puerto;
	uint8_t tipo;
} buftimer[TAM_BUF_TIMER];


uint8_t pr_timer;
uint8_t pw_timer;
uint8_t n_timer;



uint8_t  medida_nueva_eco[5];  //Se activan cuando hay una medida nuevo en los sensores de eco

uint32_t hall_buffer[2][TAM_BUFFER_HALL]; //buffer de tiempos de sensores hall

uint16_t eco[5];
uint16_t frontal_eco[3];  //Tiempos de medida de los temporizadores asociados a los ultrasonidos frontales
uint16_t rear_eco[2]; //Tiempos de medida d elos temporizadores asociados a los ultrasonidos traseros
uint16_t eco_buffer[5][TAM_BUFFER_ECO];
//uint32_t median_eco[5];








void GestorSensores(void)
{
	/*
	 *  Función no bloqueante que permite la gestión de las medidas
	 *  de la velocidad de giro de las ruedas y de la distancia de los
	 *  obstáculos detectados por los sensores de ultrasonidos.
	 *
	 * */

	 static uint8_t primero;
	 uint8_t i;

	 if(primero==0)
	 {
		 primero=1;
		 InicializaBuffers();
	 }

	 SensoresNivel0(); //Gestión del nivel bajo de las medidas
     for(i=0;i<5;i++)
      {
    	 /*Cadda vez que se detecta una nueva medida de los
    	  * sensores de ultrasonido, se evalua la distancia
    	  * dividendo el tiempo entre 59. Esta medida se mete en
    	  * el buffer ECO para calcular la mediana, la cual se deposita
    	  * en sensor.median_eco[i]*/
            if(medida_nueva_eco[i]==1)
            {
                  medida_nueva_eco[i]=0;
                  eco[i]/=59;
                  WriteEcoValue(i, eco[i]);
                  sensor.median_eco[i]=MedianN(0,i,TAM_BUFFER_ECO);
//                  sensor.median_eco[i] = eco[i];

            }
      }
	for(i=0;i<2;i++)
	{
            if(medida_nueva_hall[i]==1)
            {
            	/*Se procede de forma similar a los sensores de ECO*/
                  medida_nueva_hall[i]=0;
                  WriteHallValue(i, hall[i]);
                  sensor.median_hall[i] = MedianN(1,i,TAM_BUFFER_HALL);
                  sensor.frequency[i]=1000000.0/(45.0*sensor.median_hall[i]);  //Tal y como tiene Félix

            }
      }


	sensor.obs = DetectaObstaculo(); //Detección de obstáculos
	DetectaMovimiento();  //Envía mensaje si hay cambio de parada-movimiento o de movimiento-parada

}




uint16_t DetectaObstaculo(void)
{
      uint16_t pos_obstaculo=0,calculo=0;
      uint8_t  i,j,k;
      uint16_t distancia[2] = {par.obs_near, par.obs_far}; //Cambiar el tamaño
      uint8_t cont;
      /*
       *                  15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
       *  pos_obstaculo =  x  x  x f4 f3 f2 f1 f0 x  x  x  n4 n3 n2 n1 n0
       *
       *  n = near
       *  f = far
       *  n2 = 1 si el sensor 2 detecta obstaculo a distancia cercana
       *  f0 = 1 si el sensor 0 detecta obstáculo a distancia lejana
       *
       * */


      for(i=0;i<=1;i++)  //Bucle para distancia cercana y lejana
            for(j=0;j<5;j++) //Bucle para los sensores
            {
                  cont=0;
                  for(k=0;k<TAM_BUFFER_ECO;k++)
                  {
                        if(eco_buffer[j][k]<distancia[i])
                              cont+=1;
                  }
                  if(cont>= (TAM_BUFFER_ECO/2))
                        calculo = ( 1<<(8*i) ) << j;
                  pos_obstaculo |= calculo;

            }
      return pos_obstaculo;

}


void DetectaMovimiento(void)
{

	static uint8_t state_movement;
	#define PARADO 0
	#define MOVIMIENTO 1

	/* **************************************************
	 * Esta función permite mandar mensajes de que la plataforma
	 * está en movimiento o parada. Para ello se definen dos estados
	 * PARADO, MOVIMIENTO
	 * Si en el estado PARADO, se detecta que alguna de las ruedas
	 * está en movimiento, se pasa al estado MOVIMIENTO y se envía el mensaje
	 * si en el estado MOVIMIENTO, se detecta que ambas ruedas están
	 * paradas, se pasa al estado PARADO y se manda un mensaje
	 *
	 ****************************************************/
	switch(state_movement)
	{
		case PARADO: if(sensor.frequency[0] > UMBRAL_GIRO || sensor.frequency[1]> UMBRAL_GIRO){
			state_movement=1;
	        TransmiteEvento(EVENTO_MOVIMIENTO, state_movement);}
	        break;
	    case MOVIMIENTO: if(sensor.frequency[0]< UMBRAL_GIRO && sensor.frequency[1]<UMBRAL_GIRO){
	        state_movement=0;
	        TransmiteEvento(EVENTO_MOVIMIENTO,state_movement);}
	    	break;
	 }
}

uint8_t WriteEcoValue(uint8_t channel, uint16_t value)
{
	static uint8_t tam_eco[5];
	//Variable estática que retiene el número de datos almacenados en
	//cada uno de los dos canales del buffer.

	uint8_t i;

/*Esta función recoge un valor nuevo procedente de alguno de los dos
 * sensores Hall y lo deposita en el buffer, desplazando los valores anteriores
 * cuando éste está lleno. Channel hace referencia a Hall de la rueda izquierda
 * o derecha, mientras que value es el valor de tiempo obtenido*/

	if(tam_eco[channel] < TAM_BUFFER_ECO)
	{
/* Mientras que alguno de los canales del buffer no esté lleno,
 * se produce el relleno del mismo sin más. La posición del buffer
 * que se ocupa está indicada por la misma variable tam_hall[channel],
 * que se incrementa, una vez se haya guardado el nuevo dato.
 * */
		eco_buffer[channel][tam_eco[channel]]=value;
		tam_eco[channel]+=1;
		}
	else
	{
/* Si alguno de los canales del buffer ya está lleno, antes de
 * proceder a introducir un nuevo dato, hay que desplazar su
 * contenido
 * */
		for(i=0;i<TAM_BUFFER_ECO-1;i++)
			eco_buffer[channel][i]= eco_buffer[channel][i+1];
		eco_buffer[channel][TAM_BUFFER_ECO-1] = value;
	}
	return tam_eco[channel];
}


void InicializaBufferUltrasonidos(void)
{
      uint8_t channel, pos;
      /*Cada uno de los canales del buffer que almacena los últimos valores
       * de Hall, se inicializa con UMBRAL_VEL_EXCESIVA*/
      for(channel=0;channel<5;channel++)
            for(pos=0;pos<TAM_BUFFER_ECO;pos++)
                  eco_buffer[channel][pos]=200;

}

void InicializaBuffers(void)
{

	/*
	 * Permite la inicialización de los buffers usados para registrar
	 * los valores de medida de los ultrasonidos y de los hall.
	 * */
      InicializaBufferUltrasonidos();
      InicializaHallBuffer();
      n_timer=0;
      pr_timer=0;
      pw_timer=0;
}

void InicializaHallBuffer(void)
{
      uint8_t channel, pos;
      /*Cada uno de los canales del buffer que almacena los últimos valores
       * de Hall, se inicializa con UMBRAL_VEL_EXCESIVA*/
      for(channel=0;channel<=1;channel++)
            for(pos=0;pos<TAM_BUFFER_HALL;pos++)
                  hall_buffer[channel][pos]=4294967295;
}


uint8_t WriteHallValue(uint8_t channel, uint32_t value)
{
      static uint8_t tam_hall[2];
      //Variable estática que retiene el número de datos almacenados en
      //cada uno de los dos canales del buffer.

      uint8_t i;

      /*Esta función recoge un valor nuevo procedente de alguno de los dos
       * sensores Hall y lo deposita en el buffer, desplazando los valores anteriores
       * cuando éste está lleno. Channel hace referencia a Hall de la rueda izquierda
       * o derecha, mientras que value es el valor de tiempo obtenido*/

      if(tam_hall[channel] < TAM_BUFFER_HALL)
      {
            /* Mientras que alguno de los canales del buffer no esté lleno,
             * se produce el relleno del mismo sin más. La posición del buffer
             * que se ocupa está indicada por la misma variable tam_hall[channel],
             * que se incrementa, una vez se haya guardado el nuevo dato.
             * */
            hall_buffer[channel][tam_hall[channel]]=value;
            tam_hall[channel]+=1;
      }
      else
      {
            /* Si alguno de los canales del buffer ya está lleno, antes de
             * proceder a introducir un nuevo dato, hay que desplazar su
             * contenido
             * */
            for(i=0;i<TAM_BUFFER_HALL-1;i++)
                  hall_buffer[channel][i]= hall_buffer[channel][i+1];
            hall_buffer[channel][TAM_BUFFER_HALL-1] = value;
      }
      return tam_hall[channel];

}


uint32_t MedianN(uint8_t eco_hall, uint8_t channel, uint8_t tam)
{
      uint8_t cambio;
      uint8_t t;
      uint32_t buffer[tam],temp;

      for(t=0;t<tam;t++)
            if(eco_hall)
                  buffer[t] = hall_buffer[channel][t];
            else
                  buffer[t] = eco_buffer[channel][t];
      do
      {
            cambio=0;
            for(t=0;t<tam-1;t++)
                  if(buffer[t]<buffer[t+1])
                  {
                        temp = buffer[t];
                        buffer[t] = buffer[t+1];
                        buffer[t+1] = temp;
                        cambio=1;
                  }
      }while(cambio==1);
      if(tam & 1)
            temp = buffer[tam/2];
      else
            temp = (buffer[tam/2] + buffer[tam/2-1])>>1;

      return temp;

}


void SensoresNivel0(void)
{
	/*Función no bloqueante que permite el procesamiento de los datos brutos
	 * que se encuentran en el buftimer*/


      static uint8_t state_hall[2];
      /* La variable state_hall indica el estado en el que están las medidas
       * en los sensores hall. Inicialmente están a cero, esperando la recepción
       * del primer flanco positivo. Cuando esto ocurre, se ponen a 1, guardan
       * el valor del registro ICP en la variable hall_value y esperan el segundo
       * flanco. Cuando éste llega, entonces se calcula la diferencia entre el
       * nuevo valor almacenado en ICP y la variable hall_value. Dicha differencia
       * se introduce en hall[channel] y en el buffer para calcular la mediana.
       * */
      static uint8_t state_eco[5];
      /* Se definen dos estados asociados a cada sensor de ultrasonidos delantero.
       * Estado inicial (0), donde se espera la recepción de un flanco positivo.
       * Estado final (1), donde se espera la recepción de un flanco negativo
       * La diferencia de tiempo entre ambos flancos es el tiempo de recepción del
       * eco, que está relacionado con la distancia del objeto.
       * En el estado 0, si se recibe un flanco positivo, se guarda el valor del
       * temporizador en frontal_eco_value[channel].
       * En el estado 1, si se recibe un flanco negativo, se calcula la diferencia
       * entre el valor del temporizador actual y el previo. Se actualiza frontal_eco[channel]
       *
       * */
      static uint32_t hall_value[2];
      /* Contiene los valores temporales del temporizador cuando se detectan los
       * flancos ascendentes procedentes de los sensores hall de cada rueda.
       * */
      static uint16_t eco_value[5];
      /* Contienen los valores temporales del temporizador cuando se detectan  los
       * flancos procedentes de los sensores de ultrasonidos delanteros.
       * */

      uint8_t channel;  //Identifica canal del temporizador.


      while(n_timer >0)
      {
    	  /*Si hay datos en el buffer, comenzamos su procesamiento*/
    	  channel= buftimer[pr_timer].channel;
    	  if(buftimer[pr_timer].tipo == 1)  //Para HALL
    	  {
    		  /*Cada vez que se recibe un dato, se pone a cero este
    		   * temporizador. Si el temporizador llega a 300ms, el
    		   * hall[channel] se pone al valor más alto posible, que
    		   * significa que la velocidad es 0*/
    		  tick.t_med_hall[channel]=0;
    		  if(hall[channel]==4294967295)
    		  {
    			  /*Si venimos de la situación en la que la rueda estaba
    			   * parada, inicializamos el estado para las medidas*/
    			  state_hall[channel]=0;
    		      hall[channel]-=1;
    		   }
    		   if(state_hall[channel]==0)
    		   {
    			   /*Tomamos medida del timer en esta primera
    			    * de las dos interrupciones*/
    			   state_hall[channel]=1;
    			   hall_value[channel]=buftimer[pr_timer].valuel;
    		   }else{
    			   /*En la segunda interrupción actualizarmos hall[channel]
    			    * con el tiempo real y lo señalizamos con la
    			    * medida_nueva_hall*/
    			   state_hall[channel]=0;
    			   hall[channel] = buftimer[pr_timer].valuel;
    			   if(hall[channel]>=hall_value[channel])
    				   hall[channel]-=hall_value[channel]+1;
    			   else
    			       hall[channel]+= (4294967295 - hall_value[channel]+1);
    			    medida_nueva_hall[channel]=1;
    		   }

    	  }
    	  if(buftimer[pr_timer].tipo==0 )
    	  {
    		  /*Se necesitan dos interrupciones (SUBIDA-> BAJADA) para
    		   * determinar la duración del pulso de eco, y con ello
    		   * el tiempo transcurrido*/
    		  if(state_eco[channel]==0 ){
    			  /*En el estado 0, si recibimos interrupción y
    			   * es de SUBIDA (puerto=1) capturamos timer y pasamos
    			   * al estado 1*/
    			  if( buftimer[pr_timer].puerto==1){
    				  state_eco[channel]=1;
                      eco_value[channel]=buftimer[pr_timer].value;
    			  }
    		   }
    		   else{
    			   /*En el estado 1, si puerto es 0 (BAJADA)
    			    * tomamos medida de eco[channel], señalizamos
    			    * medida_eco_nueva y volvemos al estado 0*/
    			   if(buftimer[pr_timer].puerto==0){
    				   state_eco[channel]=0;
                       eco[channel]=buftimer[pr_timer].value;
                       if(eco[channel]>eco_value[channel])
                    	   eco[channel]-=eco_value[channel]+1;
                       else
                           eco[channel]+= ( 65535 -eco_value[channel]+1);
                       if(eco[channel]<12000)
                    	   medida_nueva_eco[channel]=1;
    				}else
    					eco_value[channel]=buftimer[pr_timer].value;
    			}
    		}

    	  /*Actualizamos el estado de los buffers*/
       	  pr_timer = (pr_timer+1) & (TAM_BUF_TIMER-1);
    	  n_timer--;

      }



}



void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint8_t channel;  //Identifica canal del temporizador.

	/*Rutina de interrupción que se invoca cada vez que existe un evento
	 * en alguno de los timers. El timer2 se dedica a la medida de los HALL
	 * para la velocidad de las ruedas, cada flanco de subida
	 *
	 * El timer 4 se utiliza para los sensores de ultrasonidos delanteros.
	 * Se activa cada flanco (subida y bajada)
	 *
	 * El timer 8 se utilizar para los sensores de ultrasonidos traseros.
	 * Se activa cada flanco (subida y bajada)
	 *
	 * Esta rutina lee los timers y los guarda en el buftimer, una matriz
	 * de estructura del tipo data_timer, que contiene el tipo de resultado,
	 * el canal, el valor del tiempo transcurrido y el estado del puerto (1 o 0)
	 * para los sensores de ultrasonidos.
	 * */

	if(n_timer < TAM_BUF_TIMER)
	{
		/*  Si el número de elementos en el buffer es menor que el tamaño,
		 * se guarda el nuevo valor.
		 *
		 * */
      if(htim->Instance==TIM2)
      {
    	  /*Sensores de efecto Hall*/

    	  /*Se guarda en el buftimer información del canal. Tipo=1 para
    	   * hall y tipo=0 para ultrasonidos*/
    	  buftimer[pw_timer].tipo = 1;
    	  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    		  channel=1;
    	  else
    		  channel=0;
    	  buftimer[pw_timer].channel=channel;
    	  //buftimer[pw_timer].value =HAL_TIM_ReadCapturedValue(htim, (channel)*4); //Capturamos el valor del timer
    	  buftimer[pw_timer].valuel =HAL_TIM_ReadCapturedValue(htim, (1-channel)*4); //Capturamos el valor del timer
    	  /* Por otro lado TIM_CHANNEL_x sería número de channel *4
    	     #define TIM_CHANNEL_1      0x00000000U
    	     #define TIM_CHANNEL_2      0x00000004U
    	     #define TIM_CHANNEL_3      0x00000008U
    	     #define TIM_CHANNEL_4      0x0000000CU
    	   */
    	  //Incrementamos el puntero de escritura para el siguiente dato
    	  pw_timer = (pw_timer+1) & (TAM_BUF_TIMER-1);
    	  //Incrementamos el número de elementos guardados en el buffer
    	  n_timer ++;
      }

      if (htim->Instance==TIM4)
      {
/*    	  COMO EL CANAL 1 NO FUNCIONA EN LA ULTIMA VERSIÓN, SE HAN USADO LOS CANALES 2,3, Y 4, POR TANTO
    	               * SE HA ADAPTADO PARA LA NUEVA SITUACIÓN
    	               *
    	               *
    	               * */

    	  buftimer[pw_timer].tipo=0;  //tipo 0
    	  channel = (htim->Channel)>>2; //número de canal
    	  buftimer[pw_timer].channel=channel;
    	  buftimer[pw_timer].value = HAL_TIM_ReadCapturedValue(htim, 4*channel+4);
    	  buftimer[pw_timer].puerto = HAL_GPIO_ReadPin(GPIOB, pin_eco_delantero[channel]);
    	  pw_timer = (pw_timer+1) & (TAM_BUF_TIMER-1);
    	  n_timer ++;
      }

      if(htim->Instance==TIM8)
      {
             /*En los sensores de ultrasonidos traseros, el pin PC7 es el izquierdo (channel =0),
              * y el pin PC6 es el derecho (channel=1). Para el STM32, el PC6 es el canal 1,
              * con valor de 0 en htim->Channel y canal en la función HAL_TIM_ReadCapture
              * mientras que PC7 debe ser el canal 0 en nuestra asignación, pero tiene el
              * valor de 2 en htim->Channel y 4 en HAL_TIM_ReadCapture.
              * La matriz pin_eco_trasero está ordenada con PC7 en la posición 0 y PC6 en la posición 1
              * */

    	  	 channel = (htim->Channel)>>1; //Poner en el nuevo diseño hardware
    	  	 buftimer[pw_timer].tipo=0;
    	  	 buftimer[pw_timer].channel=channel+3;
    	  	 buftimer[pw_timer].value = HAL_TIM_ReadCapturedValue(htim, 4*channel);
    	  	 buftimer[pw_timer].puerto = HAL_GPIO_ReadPin(GPIOC, pin_eco_trasero[channel]);
    	  	 pw_timer = (pw_timer+1) & (TAM_BUF_TIMER-1);
    	  	 n_timer ++;
      }
	}
}

