/*--[CONFIGURATION FILE]----------------------------------------------------------------------------------
 *  Politechnika Śląska - Wydział Automatyki Elektroniki i Informatyki. Kierunek: Automatyka i Robotyka
 *  Specjalność: Technologie Informacyjne.
 *  Praca inżynierska: System sterowania robotem mobilnym z kołami szwedzkimi.
 * 
 *  Autor: Jakub Kaniowski   |   Plik utworzono: 08.09.2021
 *-------------------------------------------------------------------------------------------------------
 *  Zawiera:
 * 
 *      PINOUT poszczególnych używanych wyjść/wejść mikrokontrolera. (Opis w dokumentacji)
 *      Podstawowe definicje, typy wyliczeniowe, zmienne potrzebne do działania projektu.
 *      Podstawowe "zmienne środowiskowe" programu, w tym również dane sieciowe.
 * 
 *------------------------------------------------------------------------------------------------------- 
 */

int SPID = 150; //Predkosc - wykorzystywana na potrzeby testow
int value = 0;
int ChangeMotionTime = 0;
int PrintTime = 0;
long int acquisitionTime = 0;
float millisTime = 0; // czas przeliczany na ms od wlaczenia programu
int motionMode = 0;

bool EMERGENCY_MODE_TRIGGER = 0;    //  Flaga określająca stan wyjątkowy
int loop_time = 10;                 //  Obliczony czas wykonania 1 iteracji pętli

/*        PINOUTS               */

/*  Wyprowadzenia enkoderów | Encoder  */
#define E1 36 //A
#define E2 39 //B
#define E3 34 //C
#define E4 35 //D

/*  Wyprowadzenia sterownika | Motor DC */
#define M1_A 32 // LP
#define M1_B 33
#define M2_A 26 // PP
#define M2_B 25
#define M3_A 23 // LT
#define M3_B 19
#define M4_A 17 // PT
#define M4_B 18

/*     Definicje kierunków  |  Direction definitions   */

#define FORWARD 1
#define BACK -1

/*    Inne definicje    |   Other definitions       */
#define EMERGENCY_SW 13 //  NR GPIO dla Wyłącznika awaryjnego | Emergency button pin number
#define BOARD_LED 12    //  NR GPIO dla LED dot. statusu pojazdu |Main board led for presenting actual status of device
#define BUILT_IN_LED 2  //  Wbudowany LED na NodeMCU | Built-in led on ESP32

#define SIZE 10
#define SPEED_GLOBAL 160 //nieużywane aktualnie | unused
#define LOOPTIME 10 //10ms


/*      Parametry PID  |  PID values    */

#define KP 0.9
#define KI 1.3 
#define KD 0.5  
#define TP 0.1  
#define MIN_PID 0
#define MAX_PID 156


