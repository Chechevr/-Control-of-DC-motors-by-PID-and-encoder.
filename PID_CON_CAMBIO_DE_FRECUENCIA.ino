#include <PWM.h>

//*******VARIABLES DEL PID***********//
const int senal_maestro = 12;          // Potenciometro de referencia
const int encoder_pot = A2;    // Potenciometro de posicion del motor
const byte PWMPin = 6;
const byte DirPin1 = 7;


int val = 0;
unsigned long val2;
unsigned long val3;
int encoder_val =0;
float kp = 7.25 ;//0.2
float ki = 0.02 ;//0.00000
float kd = 2.8;//2.00
float Theta, Theta_d;
int dt;
unsigned long t;
unsigned long t_prev = 0;
int val_prev =0;
float e, e_prev = 0, inte, inte_prev = 0;
float Vmax = 24;
float Vmin = -24;
float V = 0.1;

//************************************VARIABLES DE LA FRECUENCIA DE PWM**********************************//
//SE CAMBIO LA FRECUENCIA PARA QUE LA SEÑAL QUE SE ENCIE AL MOTOR NO SE ESCUCHE Y NO ESTE HACIENDO RUIDO//

int32_t frequency = 490; // Establezca la Frecuencia en Hertz (HZ), se pueden operar frecuencias de 10Hz a 300kHz aprox

//***Motor Driver Functions*****

void WriteDriverVoltage(float V, float Vmax) {
  int PWMval = int(255 * abs(V) / Vmax);
  if (PWMval > 255) {
    PWMval = 255;
  }
  if (V > 0) {
    digitalWrite(DirPin1, HIGH);
    
  }
  else if (V < 0) {
    digitalWrite(DirPin1, LOW);
    
  }
  else {
    digitalWrite(DirPin1, LOW);
    
  }
 
  pwmWrite(PWMPin, PWMval);
}


void setup() {
  Serial.begin(9600);
  pinMode(DirPin1, OUTPUT);
  pinMode(10, INPUT);
  InitTimersSafe(); 

  bool success = SetPinFrequencySafe(6, frequency);
  if(success) {
    Serial.print("Frecuencia configurada correctamente");
  }

}


void loop() {

  //***************PROCEDIMIENTO DEL PID***********************//
  val2 = pulseIn(senal_maestro, HIGH);                //SE LEE CON ESTA FUNCION PARA PODERR EVALUAR EL TIEMPO QUE RECIBE LOS PULSOS, YA QUE SE RECIBIRA SEÑAL DE PULSOS PWM DE UN POLOLU MINIMAESTRO
  val = map(val2,689,3926,100,900);//analogRead(potPin); // LEE V_out DEL POTENCIOMETRO DE REFERENCIA
  val3 = analogRead(encoder_pot);                     //COON LA FUNCION DE MAP QUE SE UTILIZA SE CONVIERTE LOS RANGOS DE VALORES LEIDOS A UN RANGO ESPECIFICO PARA QUE EL PID TENGA UNA MEJOR RESPUES, ASI COMO LA LECTURA SE ESPECIFICA YA QUE SABEMOS QUE LOS PULSOS PWM SON CORTOS DE 0-250
  encoder_val = map(val3,338,681,100,900);//analogRead(encoder_pot);              // LEE V_out DEL POTENCIOMETRO DE POSICION DEL MOTOR
  t = millis();
  dt = (t - t_prev);                                  // TIEMPOD DE PASO
  Theta = val;                                        // Theta= POSICION ANGULAR ACTUAL DEL MOTOR
  Theta_d = encoder_val;                              // Theta_d= POSICION ANGULAR DESEADA DEL MOTOR

  e = Theta_d - Theta;                                // ERROR
  inte = inte_prev + (dt * (e + e_prev) / 2);         // ERROR DE INTEGRACION
  V = kp * e + ki * inte + (kd * (e - e_prev) / dt) ; // FUNCION DE CONTROL

  if (V > Vmax) {
      V = Vmax;
      inte = inte_prev;
    }
    if (V < Vmin) {
      V = Vmin;
      inte = inte_prev;
      val_prev= val;
    }
  WriteDriverVoltage(V, Vmax);
  Serial.print("Motor:");Serial.println(Theta_d); Serial.print(" \t");
    Serial.print("Pot:");Serial.print(Theta); Serial.print(" \t ");
   t_prev = t;
    inte_prev = inte;
    e_prev = e;
    delay(10);



}
