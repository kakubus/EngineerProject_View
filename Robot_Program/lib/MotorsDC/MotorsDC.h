/*--[DC MOTORS FILE]-------------------------------------------------------------------------------------
 *  Politechnika Śląska - Wydział Automatyki Elektroniki i Informatyki. Kierunek: Automatyka i Robotyka
 *  Specjalność: Technologie Informacyjne.
 *  Praca inżynierska: System sterowania robotem mobilnym z kołami szwedzkimi.
 * 
 *  Autor: Jakub Kaniowski   |   Plik utworzono: 20.08.2021 | Ujednolicono: 08.09.2021
 *-------------------------------------------------------------------------------------------------------
 *  Zawiera:
 * 
 *      Główne struktury danych dla napędu. Implementację obliczenia prędkości (RPM oraz kątowej) oraz
 *      implementację poruszania napędem (w zadeklarowanych kierunkach, przy in/dekrementacji enkodera)
 *      Wywoływane w pliku main.cpp.
 * 
 *------------------------------------------------------------------------------------------------------- 
 */


// Struktura obsługująca enkodery. Przechowuje zliczone wartości oraz kierunek obrotu koła (-1 - do tyłu, 1 - do przodu) | Encoder struct
// Rozdzielczość enkodera: 16-bit
// Gear ratio: 35,5
struct ENC_VAL 
{
  long int A;
  long int B;
  long int C;
  long int D;
  int A_dir;
  int B_dir;
  int C_dir;
  int D_dir;
};


// Struktura obsługująca wyliczenia prędkości liniowej oraz prędkości kątowej poszczególnych kół. | Struct for motor speed
struct SPEED
{
  float vA;
  float vB;
  float vC;
  float vD;
  float rotA;
  float rotB;
  float rotC;
  float rotD;
};



void getSpeed(struct SPEED &output_S, struct ENC_VAL &ENCODER_INPUT) //Funkcja obliczająca RPM napędów  | Function for calculate RPM speed
{
  static long countAnt_A = 0; //static long int -> sprawia że zmienna nie jest kasowana po zakończeniu funkcji.
  static long countAnt_B = 0; //Dzięki temu, można jak poniżej obliczać (ENCODER.A-countAnt_A) gdzie count[..]
  static long countAnt_C = 0; //posiada wartość z poprzedniego wywołania funkcji. 
  static long countAnt_D = 0;

  output_S.vA = ((ENCODER_INPUT.A - countAnt_A) * (60 * (1000 / loop_time))) / (16 * 36); //sprawdzić czy zamiana miejsc nie będzie miała wpływu na prędkość
  countAnt_A = ENCODER_INPUT.A;

  output_S.vB = ((ENCODER_INPUT.B - countAnt_B) * (60 * (1000 / loop_time))) / (16 * 36);
  countAnt_B = ENCODER_INPUT.B;

  output_S.vC = ((ENCODER_INPUT.C - countAnt_C) * (60 * (1000 / loop_time))) / (16 * 36);
  countAnt_C = ENCODER_INPUT.C;

  output_S.vD = ((ENCODER_INPUT.D - countAnt_D) * (60 * (1000 / loop_time))) / (16 * 36); //loop_time jest liczony na końcu pętli. Kompensuje różny czas wykonywania się programu
  countAnt_D = ENCODER_INPUT.D;
  //Serial.println("\nv_A " + (String)output_S.vA+"\n");
}

void radianSpeed(struct SPEED &output_S) //Funkcja przeliczająca  | Convert RPM to rad/s
{
  output_S.rotA = output_S.vA*0.1047198;
  output_S.rotB = output_S.vB*0.1047198;
  output_S.rotC = output_S.vC*0.1047198;
  output_S.rotD = output_S.vD*0.1047198;
  //Serial.println("rotA: " + (String)output_S.rotA+" rad/s " + "rotB: " + (String)output_S.rotB+" rad/s " + "rotC: " + (String)output_S.rotC+" rad/s " + "rotD: " + (String)output_S.rotD+" rad/s\n");
}


void moveMotor(unsigned char PWM, int direction, int A_motor, int B_motor) //Wysterowanie sterownika napędów - ruch napędu przód/tył  | Function for control motor driver. Sending PWM signal for steering DC motor.
{
  if (direction == 1)
  {
    analogWrite(A_motor, PWM);
    analogWrite(B_motor, 255);
  }
  if (direction == -1)
  {
    analogWrite(A_motor, 255);
    analogWrite(B_motor, PWM);
  }
}

int autoDirection(int remoteInput, int &E_dir) //Funkcja stworzona na potrzeby odbioru zdalnych parametrów co do zadanego kierunku. | Function for set direction automatically based on received data from network.
{
  if(remoteInput == 1)
  {
    E_dir = 1;
    return 1;
  }
  if(remoteInput == -1)
  {
    E_dir = -1;
    return -1;
  }
  return 1;
}

void stopMotor()  //Zatrzymanie wszystkich napędów | Stop DC motors.
{
  analogWrite(M1_A, 255);
  analogWrite(M1_B, 255);
  analogWrite(M2_A, 255);
  analogWrite(M2_B, 255);
  analogWrite(M3_A, 255);
  analogWrite(M3_B, 255);
  analogWrite(M4_A, 255);
  analogWrite(M4_B, 255);
}


//Stare rozwiązanie stosowane gdy programowano ruchy bezposrednio w programie sterownika \/ (zamiast funkcji autoDirection) | Old function for set direction

int goForward(int &E_dir)
{
  E_dir = 1;
  return 1;
}
int goBack(int &E_dir)
{
  E_dir = -1;
  return -1;
}

