#include "DueTimer.h"

#define MaxBufSize 20*1024
#define freq  20000
#define Vmax 12
#define Voltage 10
#define fondo_escala 4100
#define HalfPeriod 1000

//Pines
int PinA = 3;
int PinB = 7;
int PWM_Pin = 8;
int enableA=2;

//Counter
int counter;

//mem for Uart 
int* Buffer;
int BufferIndex;


//Experiments
int actualExp;
int n_Exps;



//escribe el valor de la variable contador, que es el valor de los cambios de flanco de los 2 canales del encoder. 
//Por lo tanto habrá 24 cuentas del canalA y 24 del canalB. 48 cuentas por vuelta 
static void Timer_SaveData(){
  
  if(BufferIndex <= MaxBufSize){
     Buffer[BufferIndex]=counter;
     BufferIndex = BufferIndex + 1;
  }
  if(Buffer_index==HalfPerdiod){
    
    }

   
}
//pasa de voltaje al ciclo de trabajo del pwm correspondiente
int vToDTC(int V){
 
    return (fondo_escala*V)/Vmax;
 
  }
//configura los pines para que por el pin ulPin se genere una señal pwm a un ciclo de trabajo ulValue

void Pwm_Write(uint32_t ulPin, uint32_t ulValue){
  uint32_t chan = g_APinDescription[ulPin].ulPWMChannel;
    if ((g_pinStatus[ulPin] & 0xF) != PIN_STATUS_PWM) {
      // Setup PWM for this pin
      PIO_Configure(g_APinDescription[ulPin].pPort,
          g_APinDescription[ulPin].ulPinType,
          g_APinDescription[ulPin].ulPin,
          g_APinDescription[ulPin].ulPinConfiguration);
      PWMC_ConfigureChannel(PWM_INTERFACE, chan, PWM_CMR_CPRE_CLKA, 0, 0);
      PWMC_SetPeriod(PWM_INTERFACE, chan, fondo_escala);
      PWMC_SetDutyCycle(PWM_INTERFACE, chan, ulValue);
      PWMC_EnableChannel(PWM_INTERFACE, chan);
      g_pinStatus[ulPin] = (g_pinStatus[ulPin] & 0xF0) | PIN_STATUS_PWM;
    }
    PWMC_SetDutyCycle(PWM_INTERFACE, chan, ulValue);
    return;
  }
  
//Para el motor poniendo el ciclo de trabajo del pwm a 0
static void Timer_Voltage_Handler(){
   Pwm_Write(PWM_Pin,0);
}

// Cuenta las vueltas de los dos canales, por lo tanto habrá 24 cuentas del canalA y 24 del canalB. 48 cuentas por vuelta 

static void encoder(){
  counter = counter + 1;
  }

//envia los datos por el puerto serie guardados en el buffer.
//El formato usado es "indice,cuenta del encoder,experimento actual"
void SendData(){

  for(int i =0;i<BufferIndex;i++){
    
    Serial.print(i);
    Serial.print(",");
    Serial.print(Buffer[i]);
    Serial.print(",");
    Serial.println(actualExp);
  }
  }

  
void setup() {
  pinMode(PWM_Pin,OUTPUT)
  pinMode(PinA,INPUT);
  pinMode(PinB,INPUT);
  pinMode(enableA,OUTPUT);

  //configuracion del reloj del pwm para ajustar la frecuencia y el fondo de escala utilizado
  pmc_enable_periph_clk(PWM_INTERFACE_ID);
  PWMC_ConfigureClocks(freq * fondo_escala, 0, VARIANT_MCK);

  //guarda un espacio de memoria en arduino para guardar los datos generados
  Buffer = (int*)malloc(sizeof(int)* MaxBufSize);
  //inicializa variables
  BufferIndex = 0;
  counter = 0;
  actualExp = 0;
  //define el numero de experimentos que haremos para cada tension
  n_Exps = 11;

  //conecta cualquier cambio de flanco de los canales del encoder con la funcion encoder
  attachInterrupt(digitalPinToInterrupt(PinA),encoder,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PinB),encoder,CHANGE);

  //configura el puerto serie
  Serial.begin(9600);
  Serial.println("N,Enc,Exps");

  //configura las interrupciones
  //timer 2 interrumpe cada 4 segundos, cuando se dejan de tomar datos
  //timer 3 interrumpe cada 2 segundos, es la duración del pulso de activacion del motor
  //timer 4 interrumpe cada 1 ms, es el periodo de muestreo 

  Timer4.attachInterrupt(Timer_SaveData).setFrequency(1000).start();

  Pwm_Write(PWM_Pin,vToDTC(Voltage));
  digitalWrite(enableA,HIGH);
}

void loop() {
  //Hace el bucle hasta que se alcance el numero final de experimentos
  //El led se apaga mientras se envian los datos. Sirve simplemente como control
  while(1){
    if(actualExp<=n_Exps){
      switch(BufferIndex){
        case HalfPeriod:
          Timer_Voltage_Handler();
          break;
        case 2*HalfPeriod:
          Timer4.stop();
          SendData();
    
          //resetea el indice y pone a 0 el buffer 
          counter = 0;
          BufferIndex = 0;
          
          memset(Buffer,0,MaxBufSize*sizeof(int));
          
          actualExp = actualExp + 1 ;
          Timer4.start();
          Pwm_Write(PWM_Pin,vToDTC(Voltage));
          break;
        default:
          break;
        }
      }
  }
    /*
    if(tiempo == HalfPeriod){
      }
    
    if(ReadyToSend==1){
      
      
      //Pone el flag a 0, para los timers y envia los datos
      ReadyToSend = 0;
      Timer4.stop();
      Timer3.stop();
      Timer2.stop();
      SendData();

      //resetea el indice y pone a 0 el buffer 
      counter = 0;
      BufferIndex = 0;
      memset(Buffer,0,MaxBufSize*sizeof(int));
      
      actualExp = actualExp + 1 ;
      //Inician los timers
      Timer4.start();
      Timer3.start();
      Timer2.start();
      Pwm_Write(PWM_Pin,vToDTC(Voltage));
      
    }
  }
  */
}
