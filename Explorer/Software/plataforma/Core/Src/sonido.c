#include "globales.h"
#include "main.h"


void GestorAudio(void)
{

	/*
	 * Función no bloqueante que permite la gestión del Audio.
	 * Establece varios estados iniciales. En el primer estado
	 * inicializamos el temporizador de audio. En el segundo estado
	 * una vez que han transcurrido los 1.6s necesarios para el
	 * arranque del reproductor, se envía el comando ResetMP3() y
	 * pasamos al estado 2, donde nuevamente se incializa el
	 * temporizador y esperamos 1.6s para actualizar el volumen del audio.
	 * A partir de ahora, no hace nada más.
	 *
	 * Este fichero provee de funciones que se usan en diferentes partes
	 * del código
	 *
	 *
	 * */
	static uint8_t primero;
	switch(primero)
	{
	case 0: audio.tiempo=0;
			primero=1;
			break;
	case 1: if(audio.tiempo >= 1600)
				if(ResetMP3())
					primero=2;
			break;
	case 2:	audio.tiempo=0;
			primero=3;
			break;
	case 3: if(audio.tiempo>=1600)
				if(VolumeMP3(50))  //Antes a 30
					primero=4;
			break;
	}
}

uint8_t Play(uint8_t track)
{
	/*
	 * Función no bloqueante que permite la reproducción
	 * de una pista concreta. Para ello hay que mandar un
	 * paquete de datos al reproductor. Esto se hace si
	 * el banderín de uart_tx_busy está a 0. En ese caso
	 * prepara un paquete, indica el np a transmitir y devuelve
	 * 1 para indicar que se ha ejecutado correctamente.
	 * Si la uart está ocupada, la función señaliza 0, para
	 * indicar que no se ha podido ejecutar. El envío lo
	 * gestiona el Gestor de Comunicaciones
	 *
	 * */
	 uint8_t suc=0;
	  if(!audio.uart_tx_busy)
	  {
	    PreparaPaquete(0x3,track);
	    audio.np=1;
	    suc=1;
	  }
	  return suc;
}

uint8_t VolumeMP3(uint16_t val)
{
	/*Tiene la misma estructura que Play. Prepara un paquete
	 * específico para la función VolumeMP3*/
  unsigned char suc=0;
  if(!audio.uart_tx_busy)
  {
    PreparaPaquete(0x6,val);
    audio.np=1;
    suc=1;
  }
  return suc;
}


uint8_t ResetMP3(void)
{
	/*Tiene la misma estructura que Play. Prepara un paquete
	 * específico para la función ResetMP3*/
  unsigned char suc=0;
  if(!audio.uart_tx_busy)
  {
    PreparaPaquete(0xC,0);
    audio.np=1;
    suc=1;
  }
  return suc;
}



void PreparaPaquete(uint8_t command, uint16_t param)
{
	/*
	 * El reproductor recibe una serie de paquetes para su configuración
	 * y para la reproducción de los diferentes archivos almacenados en
	 * su tarjeta
	 *
	 * El protocolo exige que el paquete tenga el siguiente formato
	 *
	 * 0x7e 0xff 0x6  command  0  parh  parl chksmh chksml 0xef
	 *
	 * command 0xc: Reset
	 * command 0x6: Volumen
	 * command 0x3: Play
	 *
	 * parh parl = parámetros asociados a Play y Volumen
	 * */
  unsigned int chksum;

  audio.buf[0]=0x7e;
  audio.buf[1]=0xff;
  audio.buf[2]=6;
  audio.buf[3]=command;
  audio.buf[4]=0;
  audio.buf[5]= (param>>8) &0xff;
  audio.buf[6]= param&0xff;
  chksum=checksum(audio.buf);
  audio.buf[7]= (chksum>>8) &0xff;
  audio.buf[8]= (chksum) & 0xff;
  audio.buf[9]= 0xef;

}


uint16_t checksum(uint8_t *buffer)
{
  int sum=0;
  unsigned char i;

  for(i=1;i<7;i++)
    sum+= (int)(buffer[i]);
  return (0xffff -sum)+1;

}

/*
void GestorAudio(void)
{
	static uint8_t primero;
	static uint8_t state;
	static uint8_t nsteps;

	if(!primero)
	{
		primero=1;
		audio.pista=0;
		audio.track=1;
		//HAL_GPIO_WritePin(Audio_GPIO_Port, Audio_Pin, GPIO_PIN_SET);
		//Pin a 1
	}else
	{
		switch(state)
		{
		case 0:
			if(audio.pista!=0)
			{
				state=1;
				if(audio.pista>audio.track)
		          nsteps= audio.pista-audio.track;
		        else
		          nsteps=N_track_max  - (audio.track-audio.pista);
			}
			break;
		case 1:
			nsteps=nsteps-GeneraPulsoTrack();
			if(nsteps==0)
				state=2;
			break;
		default:
			audio.track=audio.pista;
	  	    state=0;
			audio.pista=0;
		}

	}

}*/
/*
int Play(uint8_t track)
{
	if(audio.pista==0)
	{
		audio.pista = track;
		return 1;
	}
	return -1;
}

uint8_t GeneraPulsoTrack(void)
{
	static uint8_t state;
	uint8_t res=0;

	switch(state)
	{
	case 0:
		//Pin a 0
	//	HAL_GPIO_WritePin(Audio_GPIO_Port, Audio_Pin, GPIO_PIN_RESET);
		audio.tiempo=0;
		state=1;
		break;
	case 1:
		if(audio.tiempo>=200)
			state=2;
		break;
	case 2:
		//Pin a 1
	//	HAL_GPIO_WritePin(Audio_GPIO_Port, Audio_Pin, GPIO_PIN_SET);
		audio.tiempo=0;
		state=3;
	default:
		if(audio.tiempo >=200)
		{
			state=0;
			res=1;
		}
	}
	return res;
}
*/
