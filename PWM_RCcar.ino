#include <ESP32Servo.h>  // ESP32 BOARDS
//#include <Servo.h>  // ARDUINO BOARDS

#include "PS4_controller.h"

#define PWM1_PIN 23
#define ESC_MAX_US 2000
#define ESC_STOP_US 1474
#define ESC_MIN_US 834
#define DEADBAND 20   // Zone morte autour du neutre ESC_STOP_US

#define PWM2_PIN 18      // PWM pour direction avec servomoteur
#define DIR_MAX_US 1780
#define DIR_MIN_US 1180
#define DIR_MID_US DIR_MIN_US + (DIR_MAX_US-DIR_MIN_US)/2

#define PIN_GAUCHE 9
#define PIN_DROITE 10

// VARIABLES POUR LE CONTROLE DU MOTEUR ARRIERE
Servo ESC;
int pwmVal;
bool start;
bool invert;
int gear;

// VARIABLES POUR LE CONTROLE DE DIRECTION AVEC SERVOMOTEUR
Servo DIR;
int pwmDir;


void setup(){

  Serial.begin(115200);

  start = false;
  invert = false;
  gear = 1;
  //pinMode(PIN_GAUCHE, OUTPUT);
  //pinMode(PIN_DROITE, OUTPUT);

  //BP32.forgetBluetoothKeys();
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);

  ESC.attach(PWM1_PIN, ESC_MIN_US, ESC_MAX_US);
  // --- Calibration automatique de l’ESC ---
  Serial.println("Calibration ESC...");
  ESC.writeMicroseconds(ESC_MAX_US);
  delay(2000);
  ESC.writeMicroseconds(ESC_MIN_US);
  delay(2000);
  ESC.writeMicroseconds(ESC_STOP_US);
  delay(2000);
  Serial.println("ESC calibré.");

  DIR.attach(PWM2_PIN, DIR_MIN_US, DIR_MAX_US);
  DIR.writeMicroseconds(DIR_MID_US);

}


void loop() {
  // put your main code here, to run repeatedly:

  bool dataUpdated = BP32.update();
  if (dataUpdated){
      processControllers();
  }

  vTaskDelay(1);

  // TEST APPAIRAGE MANETTE
  if (myControllers[0]) {

      if(myControllers[0]->l1()>0){
        gear = -1;
      }
      if(myControllers[0]->r1()>0){
        gear = 1;
      }

      if(myControllers[0]->x()){
        invert = true;
      }
      if(myControllers[0]->y()){
        invert = false;
      }

      if(myControllers[0]->b()){
        if(start == true){
          start = false;
          ESC.writeMicroseconds(ESC_STOP_US);

          myControllers[0]->setColorLED(255, 0, 0);
          Serial.println("<> Arret moteur <>");
        }
        else{
          start = true;
          ESC.writeMicroseconds(ESC_STOP_US);

          myControllers[0]->setColorLED(0, 255, 0);
          Serial.println("<> Demarrage moteur <>");

        }
      }
  }


  // CONTROLE MOTEUR SELON GEAR
  if(start == true){
    switch(gear){
      case -1:
        if(invert == true){
          pwmVal = map(myControllers[0]->throttle(), 0, 1020, ESC_STOP_US, ESC_MIN_US );
        }
        else{
          pwmVal = map(myControllers[0]->throttle(), 0, 1020, ESC_STOP_US, ESC_MAX_US );
        }
		    if(abs(pwmVal - ESC_STOP_US) < DEADBAND){
		      pwmVal = ESC_STOP_US;
		    }
        ESC.writeMicroseconds(pwmVal);
        //Serial.printf("Valeur PWM : %d \n", pwmVal);
        break;

      case 1:
        if(invert == true){
          pwmVal = map(myControllers[0]->throttle(), 0, 1020, ESC_STOP_US, ESC_MAX_US  );
        }
        else{
          pwmVal = map(myControllers[0]->throttle(), 0, 1020, ESC_STOP_US, ESC_MIN_US  );
        }
		    if(abs(pwmVal - ESC_STOP_US) < DEADBAND){
		      pwmVal = ESC_STOP_US;
		    }
        ESC.writeMicroseconds(pwmVal);
        //Serial.printf("Valeur PWM : %d \n", pwmVal);
        break;
    }


     // CONTROLE DIRECTION
    switch(invert){
      case true:

        pwmDir = map(myControllers[0]->axisX(), -512, 512, DIR_MAX_US, DIR_MIN_US  );
        DIR.writeMicroseconds(pwmDir);
        /*
        if(myControllers[0]->axisX() > 150 ){
          digitalWrite(PIN_DROITE, HIGH);
        }
        else{
          digitalWrite(PIN_DROITE, LOW);
        }

        if(myControllers[0]->axisX() < -150){
          digitalWrite(PIN_GAUCHE, HIGH);
        }
        else{
          digitalWrite(PIN_GAUCHE, LOW);
        }
        */
        break;

      case false:

        pwmDir = map(myControllers[0]->axisX(), -512, 512, DIR_MIN_US, DIR_MAX_US  );
        DIR.writeMicroseconds(pwmDir);

        /*
        if(myControllers[0]->axisX() > 150 ){
          digitalWrite(PIN_GAUCHE, HIGH);

        }
        else{
          digitalWrite(PIN_GAUCHE, LOW);
        }

        if(myControllers[0]->axisX() < -150){
          digitalWrite(PIN_DROITE, HIGH);
        }
        else{
          digitalWrite(PIN_DROITE, LOW);
        }
        */
        break;
    }
  }


}
