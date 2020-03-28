#include <DueTimer.h>

#define freq  20000
#define Vmax 12
#define pulsosPorVuelta 75*48
#define fondo_escala 4100
#define f_muestreo 1000

//constantes de proporcionalidad del controlador
#define K_p 10

//Pines
int PinA = 3;
int PinB = 7;
int PWM_Pin_A = 8;
int PWM_Pin_B = 9;
int Pin_enableA = 2;
//Counter
int counter;
int fin_counter;
int moviendose;
float error;
float errorAcumulado;


void setup() {
  
  Serial.begin(9600);

  pinMode(PinA,INPUT);  
  pinMode(PinB,INPUT);
  pinMode(PWM_Pin_A,OUTPUT);
  pinMode(PWM_Pin_B,OUTPUT);
  pinMode(Pin_enableA,OUTPUT);
  //configuracion del reloj del pwm para ajustar la frecuencia y el fondo de escala utilizado
  pmc_enable_periph_clk(PWM_INTERFACE_ID);
  PWMC_ConfigureClocks(freq * fondo_escala, 0, VARIANT_MCK);

  //inialización de variables
  counter = 0;
  fin_counter = 0;
  moviendose=0;
  error=0;
  //configuración de interrupciones
  attachInterrupt(digitalPinToInterrupt(PinA),encoderA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PinB),encoderB,CHANGE);
  Timer2.attachInterrupt(controlador).setFrequency(f_muestreo).start();

  //inicialmente el motor está parado
  Pwm_Write(PWM_Pin_A,vToDTC(0));
  Pwm_Write(PWM_Pin_B,vToDTC(0));
  digitalWrite (Pin_enableA,HIGH);
  Serial.println("configuracion lista");

}

void loop() {  
  while(1){
    //se pide un valor por pantalla
    if((Serial.available() > 0 )&& moviendose==0){
      float angulo= Serial.parseFloat();
      //esta comparación se haceya que si faltara no se recogen los valores correctamente.
      if(angulo!=0){

        //calcula los pulsos que le quedan por dar para llegar a la posición deseada y comienza a moverse.
        fin_counter=(pulsosPorVuelta*angulo)/TWO_PI;
        actualizaError();
        moviendose=1;
    }
  }
  
  float voltaje=error*K_p;

  //Si está en esta cota el motor se para, idealmente esto sería una comparación con 0 pero nunca llega a esta posición sino muy próxima.
  //La cota para que funcione es arbitraria. Yo he decidido que sea 0.1.
  if(error < 0.1 && error > -0.1 ){
    moviendose=0;
    moverMotor(0);
    error=0;
  }
  else{
    moverMotor(voltaje);
  }
  }
}
/*Rutinas de atencion a interrupciones*/
// Cuenta las vueltas de los dos canales, por lo tanto habrá 24 cuentas del canalA y 24 del canalB (48 cuentas) por vuelta 

static void encoderA(){
  if(digitalRead(PinB)== digitalRead(PinA)){
    counter = counter + 1;
  }else{
    counter = counter - 1;
  }
  }
static void encoderB(){
  if(digitalRead(PinB)!= digitalRead(PinA)){
    counter = counter + 1;
  }else{
    counter = counter - 1;
  }
  }

//actualiza el error siempre que se esté moviendo el motor
static void controlador(){
  if(moviendose==1){
    actualizaError();
  }
}

/*Funciones auxiliares*/
//función para mover el motor dependiendo del voltaje. 
//Aquí también se decide la dirección de giro en funcón de si voltaje es mayor o menor que 0.
void moverMotor(float voltaje){
 
  int ciclo_trabajo=vToDTC(voltaje);

  //comprobación para que no se sature el motor
  if(ciclo_trabajo>fondo_escala){
    ciclo_trabajo=fondo_escala;
  }
  else if(ciclo_trabajo<(-1)*fondo_escala){
    ciclo_trabajo=(-1)*fondo_escala;
  }
  if(voltaje>0){
    Pwm_Write(PWM_Pin_A,ciclo_trabajo);
    Pwm_Write(PWM_Pin_B,0);
  }
  else{
    Pwm_Write(PWM_Pin_B,ciclo_trabajo*(-1));
    Pwm_Write(PWM_Pin_A,0);
  }
}


//pasa de voltaje al ciclo de trabajo del pwm correspondiente
int vToDTC(float V){
    return (fondo_escala*V)/Vmax;
}

//acualiza el error actual
void actualizaError(){
    error=(float(fin_counter-counter))/abs(float(fin_counter));
    
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
