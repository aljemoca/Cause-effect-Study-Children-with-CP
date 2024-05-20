
/*
  KeyboardAndMouseControl

 Controls the mouse from five pushbuttons on an Arduino Leonardo, Micro or Due.

 Hardware:
 * 5 pushbuttons attached to D2, D3, D4, D5, D6

 The mouse movement is always relative. This sketch reads
 four pushbuttons, and uses them to set the movement of the mouse.

 WARNING:  When you use the Mouse.move() command, the Arduino takes
 over your mouse!  Make sure you have control before you use the mouse commands.

 created 15 Mar 2012
 modified 27 Mar 2012
 by Tom Igoe

 this code is in the public domain

 */

#include "Keyboard.h"
#include "Mouse.h"
/*
  Programa para la gestión del ordenador durante el experimento de la causa-efecto en ASPACE
  El sistema necesita estar conectado con la aplicación Bluetooth de Alex
  Cuando se le da a comenzar en dicha aplicación, recibe un comando GO, que transmite a su vez para
  identificar el momento de comienzo. El stop para el funcionamiento y manda el mensaje ST hacia 
  la aplicación. Con ello se sabe, en todo momento cuándo comienza y finaliza el ensayo.

  Existe un pulsador de cuidador y un pulsador de usuario. El pulsador del cuidador se presiona para
  sincronizar la aplicación con Youtube, una vez que se ha iniciado el ensayo (recepción comando GO)
  C0,tiempo,x, y P0,tiempo,x y P1,tiempo,x, junto con el tiempo son los mensajes que se reciben.
  Junto con el mensaje C0,tiempo,x, se manda un mensaje de estado, indicando en cuál está, pero sin
  alterarlo. Tanto el pulsador del cuidador como el del usuario, generan la transmisión de la constante 'k'
  para reproduccir o parar youtube.

  También se transmiten mensajes relacionados con el estado del Arduino, con el formato Sn,tiempo,x, donde
  n puede ser 0,1 y 2. En el estado S2, el Arduino está esperando una pulsación para poner en marcha el video.
  Durante la reproducción del vídeo, el Arduino está en el estado S1 que debe terminar una vez transcurrido
  el tiempo de exposición. Por consiguiente, el tiempo entre S2 y P0 identifica el tiempo de reacción, mientras
  que las pulsaciones P0-P1 durante S1 representan errores de pulsación.   

  Estados y eventos en el computador:

  
              |-----------------|      
  ------------|                 |------------------
  S2           S1                S2 
            

  S2: Espera a pulsación. Youtube parado
  S1: Reproducción Youtube durante tiempo estipulado
  P0: Pulsación usuario
  P1: Liberación usuario
  C0: Pulsación cuidador para sincronismo con ordenador

  Tiempo de reacción: Tiempo desde la transición S1-S2 a la pulsación P0
  Errores de comisión: Secuencias P0-P1 durante el estado S1
  
*/



//Tiempo de exposicion en ticks de 0.1 segundos
#define TIEMPO_EXPOSICION 300 
#define STOP 0X40
#define GO 0XF0

//Comentar la siguiente línea si no se quiere mandar los comandos al terminal
#define DEBUG 

// set pin numbers for the five buttons:
const int InternalButton = A1;
const int ExternalJack = 5;
const int BluetoothState = 2;
const int ledPin =  13; 

int valor=0;
unsigned long myTimeP;
unsigned long tick;
unsigned int tiempo_exposicion;
int state = LOW;


void setup() { // initialize the buttons' inputs:
  pinMode(InternalButton, INPUT_PULLUP);
  pinMode(ExternalJack, INPUT_PULLUP);
  pinMode(BluetoothState, INPUT);
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  Serial1.begin(9600);
  // initialize mouse control:
//  Mouse.begin();
  Keyboard.begin();
  myTimeP = millis();
  tick = 0; 
  state = digitalRead(ExternalJack);
  tiempo_exposicion=0;
  //Serial1.println('Hola');
}


void loop(){
  static unsigned long old_tick;
  static unsigned char run;  //Tres estados inicio, run, stop
  int botonusu,botoncuidador;
  char buffer[5];
  unsigned char a;
  char pulsacion=0;
  char pulsacion_cuidador=0;
  static char state_cuidador;
  
  unsigned long myTime = millis();  //Actualizamos los milisegundos
  if( (myTime-myTimeP)>=100)
  {  
  
    tick++;
    myTimeP = myTime;  
    if(tiempo_exposicion>0)
      tiempo_exposicion--;
    /* La variable tick se debe actualizar cada 100ms.
    */ 
   }

   if(Serial1.available()>0)
   {
      a=Serial1.read();
      #ifdef DEBUG
      Serial.println(a,HEX);
      #endif
      if(a == GO )
      {
          buffer[0]='G';
          buffer[1]='0';
          buffer[2]=',';
          buffer[3]=0;
          Serial1.print(buffer);
          Serial1.print(tick);
          Serial1.print('x');
          #ifdef DEBUG
          Serial.print(buffer);
          Serial.print(tick);
          Serial.print('x');
          #endif
              run=1;
              tiempo_exposicion = TIEMPO_EXPOSICION;
              Keyboard.write('k');
      } else            
      if(a == STOP)
      {
          buffer[0]='S';
          buffer[1]='T';
          buffer[2]=',';
          buffer[3]=0;
          Serial1.print(buffer);
          Serial1.print(tick);
          Serial1.print('x');
          #ifdef DEBUG
          Serial.print(buffer);
          Serial.print(tick);
          Serial.print('x');
          #endif
          if(run!=0)
          {
            run=0;
            tiempo_exposicion=0;
            Keyboard.write('k');
          }
       }
    }

  if( (tick -old_tick)>=2)
  {
    /* Cada 200ms se lee el pulsador externo para evitar rebotes*/
    old_tick = tick;
  //  Serial1.println(tick);
    botonusu= digitalRead(ExternalJack);
    botoncuidador = digitalRead(InternalButton);
  //  digitalWrite(ledPin,botonusu);

    switch(state_cuidador)
    {
      case 0:
          if(botoncuidador==0) 
          { 
            pulsacion_cuidador=1;
            state_cuidador=1;
              buffer[0]='C';
              buffer[1]='0';
              buffer[2]=',';
              buffer[3]=0;
              Serial1.print(buffer);
              Serial1.print(tick);
              Serial1.print('x');
              #ifdef DEBUG
              Serial.print(buffer);
              Serial.print(tick);
              Serial.println('x');
              #endif
                  //run=1;
                  //tiempo_exposicion = TIEMPO_EXPOSICION;
                  Keyboard.write('k');
                  buffer[0]='S';
                  buffer[1]=run+'0';
                  buffer[2]=',';
                  buffer[3]=0;
                  Serial1.print(buffer);
                  Serial1.print(tick);
                  Serial1.print('x');
                  #ifdef DEBUG
                  Serial.print(buffer);
                  Serial.print(tick);
                  Serial.println('x');
                  #endif
          }
          break;
       case 1:
           if(botoncuidador==1)
          {
            state_cuidador=0;
            }
        
          break;
      
      
      }
    
  
    switch(state)
    {
        case 1:  
            //Estado de reposo;
            if(botonusu==0)
            {
              state=0;
              buffer[0]='P';
              buffer[1]=state+'0';
              buffer[2]=',';
              buffer[3]=0;
              Serial1.print(buffer);
              Serial1.print(tick);
              Serial1.print('x');
              #ifdef DEBUG
              Serial.print(buffer);
              Serial.print(tick);
              Serial.println('x');
              #endif
              pulsacion=1;
              }
            break;
        case 0:
           if(botonusu==1)
           {
              state=1;
              buffer[0]='P';
              buffer[1]=state+'0';
              buffer[2]=',';
              buffer[3]=0;
              Serial1.print(buffer);
              Serial1.print(tick);
              Serial1.print('x');
              #ifdef DEBUG
              Serial.print(buffer);
              Serial.print(tick);
              Serial.println('x');
              #endif
            }
            break;
      }   

      switch(run)
      {
        /*  case 0: 
              if(pulsacion_cuidador==1)
              { 
                  run=1;
                  tiempo_exposicion = TIEMPO_EXPOSICION;
                  Keyboard.write('k');
                  buffer[0]='S';
                  buffer[1]=run+'0';
                  buffer[2]=',';
                  buffer[3]=0;
                  Serial1.print(buffer);
                  Serial1.print(tick);
                  Serial1.print('x');
                  Serial.print(buffer);
              Serial.print(tick);
              Serial.println('x');
              
               }
              break;*/
          case 1: 
              if(tiempo_exposicion==0)
              {
                  run=2;
                  Keyboard.write('k');
                  buffer[0]='S';
                  buffer[1]=run+'0';
                  buffer[2]=',';
                  buffer[3]=0;
                  Serial1.print(buffer);
                  Serial1.print(tick);
                  Serial1.print('x');
                  #ifdef DEBUG
              Serial.print(buffer);
              Serial.print(tick);
              Serial.println('x');
                #endif
                }        
              break;
         case 2:
              if(pulsacion)
              {
                  run=1;
                  Keyboard.write('k');
                  tiempo_exposicion = TIEMPO_EXPOSICION;
                  buffer[0]='S';
                  buffer[1]=run+'0';
                  buffer[2]=',';
                  buffer[3]=0;
                  Serial1.print(buffer);
                  Serial1.print(tick);
                  Serial1.print('x');
#ifdef DEBUG
              Serial.print(buffer);
              Serial.print(tick);
              Serial.println('x');
              #endif
              }
            
              break;
        
       }
    
  }

  

  
/*k
  switch(state)kkkkkk
  {
    case 0: if(tick>=100){
         state =HIGH;
         tick=0;
      }; break;
    
    case 1: if( tick>=100){
        state =0;
        tick=0;;
      }; break;
    };
  digitalWrite(ledPin,state);*/

  // use the pushbuttons to control the keyboard:
 /* if (digitalRead(upButton) == HIGH) {
    Keyboard.write('u');
  }
  if (digitalRead(downButton) == HIGH) {
    Keyboard.write('d');
  }
  if (digitalRead(leftButton) == HIGH) {
    Keyboard.write('l');
  }
  if (digitalRead(rightButton) == HIGH) {
    Keyboard.write('r');
  }
  if (digitalRead(mouseButton) == HIGH) {
    Keyboard.write('m');
  }
*/
}
