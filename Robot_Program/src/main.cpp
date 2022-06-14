/*--[MAIN PROGRAM FILE]----------------------------------------------------------------------------------
 *  Politechnika Śląska - Wydział Automatyki Elektroniki i Informatyki. Kierunek: Automatyka i Robotyka
 *  Specjalność: Technologie Informacyjne.
 *  Praca inżynierska: System sterowania robotem mobilnym z kołami szwedzkimi.
 * 
 *  Autor: Jakub Kaniowski   |   Plik utworzono: 15.08.2021 | Ujednolicono: 08.09.2021
 *-------------------------------------------------------------------------------------------------------
 *  Zawiera:
 * 
 *      Główną implementacje programu sterującego, obsługę przerwań.
 *      Wysyłanie danych poprzez Serial monitor, odbiór i wysyłkę danych przez sieć.
 *      Główne części programu:
 *        - funkcję setup();
 *        - funkcję loop();
 * 
 *------------------------------------------------------------------------------------------------------- 
 */

/* Biblioteki zewnętrzne */

#include <Arduino.h>
#include <analogWrite.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <HCSR04.h>
#include <WiFi.h>
#include <PubSubClient.h>


/* Własne dołączenia */
#include <Communication.h>
#include <Configuration.h>
#include <MotorsDC.h>
#include <MotorsPID.h>
#include <Sensors.h>

/* Inicjalizacje zmiennych */

String readString;
float usonic_distance = 1000; // zmienna dla mierzonej odległości jeżeli zamontowano czujnik u-dźw. | Variable for distance measurement 
int time_of_obstacle = 0;     // czas mierzony od wykrycia przeszkody, do minięcia przeszkody | Time of avoiding obstacle.
bool detected_obstacle = false; // Flaga definiująca tryb omijania przeszkody | Flag for avoiding obstacle

ENC_VAL E_VAL = {0, 0, 0, 0, 1, 1, 1, 1};
SPEED vMotor = {0, 0, 0, 0, 0, 0, 0, 0};
COMPASS compassData = {0, {0, 0, 0}, 0, 0, 0};
IMU imuData = {0, 0, 0, 0, 0, 0};

MotorPID PID_A = {0, 0, 0, 0, 0, 0, 0, 0, KP, KI, KD, TP, MIN_PID, MAX_PID, 0, 0, 0};
MotorPID PID_B = {0, 0, 0, 0, 0, 0, 0, 0, KP, KI, KD, TP, MIN_PID, MAX_PID, 0, 0, 0};
MotorPID PID_C = {0, 0, 0, 0, 0, 0, 0, 0, KP, KI, KD, TP, MIN_PID, MAX_PID, 0, 0, 0};
MotorPID PID_D = {0, 0, 0, 0, 0, 0, 0, 0, KP, KI, KD, TP, MIN_PID, MAX_PID, 0, 0, 0};

float count_A = 0;     
float speed_act_A = 0; 

unsigned long lastMilli = 0;      // Zmienna dla czasu petli | loop timing 
//unsigned long lastMilliPrint = 0; // loop timing 



int parsedData[DATA_SIZE] = {0}; // Tablica danych otrzymanych z sieci | Array for received data from network controller. 




//  Definicje przerwań  |  Interrupts 

void IRAM_ATTR ENC_A_ISR()
{
  E_VAL.A += (1 * E_VAL.A_dir);
}
void IRAM_ATTR ENC_B_ISR()
{
  E_VAL.B += (1 * E_VAL.B_dir);
}
void IRAM_ATTR ENC_C_ISR()
{
  E_VAL.C += (1 * E_VAL.C_dir);
}
void IRAM_ATTR ENC_D_ISR()
{
  E_VAL.D += (1 * E_VAL.D_dir);
}

//  Tryb awaryjny | Change Emergency Mode Interrupt

void IRAM_ATTR EMERGENCY_TASK()
{
  EMERGENCY_MODE_TRIGGER = true;
}



void emrg_function() // Funkcja trybu awaryjnego | Emergency mode function
{
  // Zatrzymaj wszystkie napędy | STOP ALL MOTORS
  analogWrite(M1_A, 255);
  analogWrite(M1_B, 255);
  analogWrite(M2_A, 255);
  analogWrite(M2_B, 255);
  analogWrite(M3_A, 255);
  analogWrite(M3_B, 255);
  analogWrite(M4_A, 255);
  analogWrite(M4_B, 255);

  //Fragment kodu odpowiadający za sprawdzanie deaktywacji. | CHECK IF PRESSING EMRG_BUTTON TO DEACTIVATE
  unsigned char counter_to_exit = 0;
  for (;;)
  {
    digitalWrite(BOARD_LED, HIGH);
    delay(100);
    if (digitalRead(EMERGENCY_SW) == 0)
    {
      counter_to_exit += 1;
    }
    if (counter_to_exit == 35)
    {
      Serial.println("CLOSING EMERGENCY MODE");

      for (int i = 0; i < 3; i++)
      {
        digitalWrite(BOARD_LED, HIGH);
        delay(500);
        digitalWrite(BOARD_LED, LOW);
        delay(500);
      }

      clearPID(PID_A);  //Wyczyszcenie wartosci zadanych do 0 | Clear setPoint to 0
      clearPID(PID_B);
      clearPID(PID_C);
      clearPID(PID_D);

      for(int i = 0; i < DATA_SIZE; i++){ //Wyczyszczenie danych otrzymanych przez siec | Clear array with data received from network
        parsedData[i] = 0;
      }

      digitalWrite(BOARD_LED, HIGH);
      EMERGENCY_MODE_TRIGGER = false;
      break;
    }
    Serial.println("!STOP! \t EMERGENCY MODE");
    digitalWrite(BOARD_LED, LOW);
    delay(100);
  }
}

// Informacja przez serialmonitor | Serial Print for debug

void printAll()
{
  Serial.println("" + (String)vMotor.vA + ", " + (String)vMotor.vB + ", " + (String)vMotor.vC + ", " + (String)vMotor.vD + "," + (String)compassData.x + "," + (String)compassData.y + "," + (String)compassData.z + "," + (String)imuData.acc_x + "," + (String)imuData.acc_y + "," + (String)imuData.acc_z + "," + (String)imuData.rot_x + "," + (String)imuData.rot_y + "," + (String)imuData.rot_z + "," + (String)PID_A.setSpeed + "," + (String)PID_B.setSpeed + "," + (String)PID_C.setSpeed + "," + (String)PID_D.setSpeed + "," + (String)PID_A.output + "," + (String)PID_B.output + "," + (String)PID_C.output + "," + (String)PID_D.output + "," + (String)PID_A.outputRPM + "," + (String)PID_B.outputRPM + "," + (String)PID_C.outputRPM + "," + (String)PID_D.outputRPM +","+(String)millisTime
  );
}


WiFiServer server(communication_port); //Port wejsciowy urządzenia. Utworzenie punktu | Input communication port
WiFiClient client_masterAPP;           //Port wyjściowy na który należy wysyłać dane | Output communication port (on remote host)

String currentLine = ""; //Tymczasowa zmienna dla odbieranych danych z sieci | temporary string 

void clientInput() // Nasłuchiwanie celem odbioru danych od operatora |  Read data from remote host
{
  WiFiClient client = server.available(); 
  char currentData[128] = {0}; 
  int i = 0;
  char temp_char;
  if (client)
  {                               
    while (client.connected())
    { 
      if (client.available())
      {                    
        temp_char = client.read(); 
        header += temp_char;

        if (temp_char == '\n')
        { 
          if (currentLine.length() == 0)
          {
            client.println("OK"); 
            client.println();
            break;
          }
          else
          { 
            currentLine = "";
          }
        }
        else if (temp_char != '\r')
        {              
          currentLine += temp_char; 
        }
        currentData[i] = temp_char; 
        i++;
      }
    }
    //Parsowanie otrzymanych danych | Parse data
    char *parsedString = &currentData[0];
    int i = 0;
    while (i < DATA_SIZE)
    {
      if (i < 1)
      {
        parsedData[i] = atoi(strtok(parsedString, ", "));
      }
      else
      {
        parsedData[i] = atoi(strtok(NULL, ", "));
      }
      i++;
    }
    header = "";
    client.stop();

  }
}

int reportData(String dataToSend) //Funkcja służąca do wysyłania danych do operatora | Function used for sending data to remote host
{
  int code = 0; // 0 - success, 1 - fail (timeout)
  if ((millis() - sendData_millis) >= SEND_TO_MASTERAPP_TRIGGER)
  {
    sendData_millis = millis();
    if (client_masterAPP.connect(masterAPP_IP_ADDRESS, masterAPP_communication_port, 20))
    { //20 - timeout
      client_masterAPP.println(dataToSend);
      client_masterAPP.stop();
    }
    else
    {
      code = 1;
    }
  }

  return code;
}

/*      SETUP FUNCTION    */

void setup()
{
  // Konfiguracja wyprowadzeń mikrokontrolera | Configuration of GPIO
  Serial.begin(115200);
  pinMode(M1_A, OUTPUT);
  pinMode(M1_B, OUTPUT);
  pinMode(M2_A, OUTPUT);
  pinMode(M2_B, OUTPUT);
  pinMode(M3_A, OUTPUT);
  pinMode(M3_B, OUTPUT);
  pinMode(M4_A, OUTPUT);
  pinMode(M4_B, OUTPUT);
  pinMode(E1, INPUT_PULLUP);
  pinMode(E2, INPUT_PULLUP);
  pinMode(E3, INPUT_PULLUP);
  pinMode(E4, INPUT_PULLUP);
  pinMode(EMERGENCY_SW, INPUT_PULLUP);
  pinMode(BOARD_LED, OUTPUT);
  pinMode(BUILT_IN_LED, OUTPUT);
  attachInterrupt(E1, ENC_A_ISR, RISING);
  attachInterrupt(E2, ENC_B_ISR, RISING);
  attachInterrupt(E3, ENC_C_ISR, RISING);
  attachInterrupt(E4, ENC_D_ISR, RISING);
  attachInterrupt(EMERGENCY_SW, EMERGENCY_TASK, FALLING);

  // Stworzenie punktu sieciowego | Create network point

  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(localIp, gateway, subnet);
  IPAddress IP = WiFi.softAPIP();
  delay(3000);
  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.begin();


  //  Inicjalizacja czujników IMU i magnetometru | Initialize IMU and compass sensor.
  for (int i = 0; i < SIZE; i++)
    compass.init();
    if (!mpu.begin())
    {
      Serial.println("Failed to find MPU6050 chip");
      while (1)
      {
        delay(10);
      }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);



  digitalWrite(BOARD_LED, HIGH); // Turn on main LED - Device is ready. 

  Serial.println("vA,vB,vC,vD,SPEED,cX,cY,cZ,aX,aY,aZ,rX,rY,rZ,time");
}

int zmienna_do_zmiany_predkosci_zadanej = 0; //for tests
long int timerForPrint = 0;                  //for tests
int timerForSerial = 0;                      //for tests


/*  Funkcja loop(), główny program | MAIN PROGRAM LOOP  */

void loop()
{
  unsigned long actual_start_time = millis(); //Pobiera czas z zegara RTU  |  Save loop time.

  if (EMERGENCY_MODE_TRIGGER == true) // Sprawdzanie czy wywołano tryb awaryjny | Check device emergency mode trigger
  {
    emrg_function();
  }

  //Przykład wykorzystania czujnika ultradzwiekowego do omijania przeszkod | Example of avoiding obstacles with ultrasonic sensor. If unused - comment this section
  /*
  if(usonic_distance < 10) // Wykrycie przeszkody unik w bok
  {
    detected_obstacle = true;
    parsedData[1] = -1; //Nadpisanie kierunków sterowania napędami - zmiana strategii sterowania
    parsedData[3] = 1;
    parsedData[5] = 1;
    parsedData[7] = -1;
    time_of_obstacle++;
  }

  if(usonic_distance > 10 && detected_obstacle == true)
  {
    static int shortDelay = 0;
    shortDelay++;
    if(shortDelay>100 && shortDelay<300){
      parsedData[1] = 1;
      parsedData[3] = 1;
      parsedData[5] = 1;
      parsedData[7] = 1;
    }
    if(shortDelay>310){ //powrot
      parsedData[1] = 1;
    parsedData[3] = -1;
    parsedData[5] = -1;
    parsedData[7] = 1;
    }

     if(shortDelay>(310+time_of_obstacle)){ //koniec powrotu
      parsedData[1] = 1;
      parsedData[3] = 1;
      parsedData[5] = 1;
      parsedData[7] = 1;
      detected_obstacle = false;
      time_of_obstacle = 0;
      shortDelay = 0;
      
    }
    
  }
  */

  // ------------------------------------

  if ((millis() - lastMilli) >= LOOPTIME) 
  {
    //lastMilli = millis();
    getSpeed(vMotor, E_VAL); //Odczyt liczników enkoderów, obliczanie prędkości napędów | Refresh encoders value, calculate velocity.
    radianSpeed(vMotor);
    speedByPID(PID_A, vMotor.vA); 
    speedByPID(PID_B, vMotor.vB);
    speedByPID(PID_C, vMotor.vC);
    speedByPID(PID_D, vMotor.vD);

  //  usonic_distance = usonic_sensor(); // Odczyt z czujnika ultradzwiekowego do omijania przeszkod |  Use ultrasonic distance sensor for avoiding obstacles. If unused - comment this line 


}

  clientInput(); //Nasluchiwanie danych od operatora | Data from network


  //Ustawienie wartosci zadanej wg. wartosci otrzymanych zdalnie | Send setPoint from received data:
  setSpeedforPID(parsedData[2], PID_A);
  setSpeedforPID(parsedData[4], PID_B);
  setSpeedforPID(parsedData[6], PID_C);
  setSpeedforPID(parsedData[8], PID_D);
  
  if (parsedData[0] == 1) //Sprawdzanie czy pierwsza wartosc z otrzymanych przez siec danych = 1. Jesli tak, zatrzmuje napedy | If first element from received data is 1 - stop all motors
  { 
    stopMotor();
  }
  else
  {
    moveMotor(speedByPID(PID_A, vMotor.vA), autoDirection(parsedData[1], E_VAL.A_dir), M1_A, M1_B);
    moveMotor(speedByPID(PID_B, vMotor.vB), autoDirection(parsedData[3], E_VAL.B_dir), M2_A, M2_B);
    moveMotor(speedByPID(PID_C, vMotor.vC), autoDirection(parsedData[5], E_VAL.C_dir), M3_A, M3_B);
    moveMotor(speedByPID(PID_D, vMotor.vD), autoDirection(parsedData[7], E_VAL.D_dir), M4_A, M4_B);
  }

  compass_xyz(compassData); //Odczyt wartosci z magnetometru | Use compass sensor
  acc_sensor(imuData);      //Odczyt wartosci z IMU |  Use IMU sensor

  /* SERIAL OUT */
  timerForSerial = millis();
  timerForPrint = millis() - PrintTime;
  if ((timerForPrint) >= 10)
  {
    acquisitionTime += PrintTime;
    millisTime = acquisitionTime / 1000;
    PrintTime = millis();
    printAll(); //Serial print
    
    reportData((String)vMotor.vA + ", " + (String)vMotor.vB + ", " + (String)vMotor.vC + ", " + (String)vMotor.vD + ", " + (String)compassData.x + ", " + (String)compassData.y + ", " + (String)compassData.z + ", " + (String)imuData.acc_x + ", " + (String)imuData.acc_y + ", " + (String)imuData.acc_z + ", " + (String)imuData.rot_x + ", " + (String)imuData.rot_y + ", " + (String)imuData.rot_z + ", " + (String)PID_A.setSpeed + ", " + (String)PID_B.setSpeed + ", " + (String)PID_C.setSpeed + ", " + (String)PID_D.setSpeed + ", " + (String)PID_A.output + ", " + (String)PID_B.output + ", " + (String)PID_C.output + ", " + (String)PID_D.output + ", " + (String)PID_A.outputRPM + ", " + (String)PID_B.outputRPM + ", " + (String)PID_C.outputRPM + ", " + (String)PID_D.outputRPM);
  }

  


  // Obliczanie czasy pętli | LoopTime

  loop_time = millis()-actual_start_time;
 
  if(loop_time <= 0)
  {
    Serial.println("ERROR TIME");
    loop_time = 10;
  }

}

/*  END FILE  */
