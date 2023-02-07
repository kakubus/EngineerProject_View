/*--[PID IMPLEMENTATION]---------------------------------------------------------------------------------
 *  Politechnika Śląska - Wydział Automatyki Elektroniki i Informatyki. Kierunek: Automatyka i Robotyka
 *  Specjalność: Technologie Informacyjne.
 *  Praca inżynierska: System sterowania robotem mobilnym z kołami szwedzkimi.
 * 
 *  Autor: Jakub Kaniowski   |   Plik utworzono: 08.09.2021
 *-------------------------------------------------------------------------------------------------------
 *  Zawiera:
 * 
 *      Implementację algorytmu PID oraz potrzebne struktury danych, dla sterowania napędami robota.
 *      Główna funkcja wywoływana w pliku main.cpp.
 * 
 *------------------------------------------------------------------------------------------------------- 
 */

struct MotorPID // Struktura obliczeniowa PID, dla każdego napędu z osobna tj. pidMotorA, pidMotorB, pidMotorC, pidMotorD.. | Struct for PID controller
{
    unsigned char output;       // Wyregulowane wyjście (po konwersji, podawane na sterownik)
    float outputRPM;            // Obliczona prędkość wyjściowa RPM (przed konwersją)
    float setSpeed;             // predkosc zadana nie jako PWM tylko RPM
    float c_value;              // obecna wartość wyjścia
    float c_error;              // uchyb - obecny błąd
    float last_value;           // ostatnia wartość wyjścia
    float last_error;           // ostatni błąd
    float last_I;               // ostatnia wartosc całki
    float Kp;                   // Wzmocnienie członu proporcjonalnego
    float Ki;                   // Wzmocnienie członu całkującego
    float Kd;                   // Wzmocnienie członu różniczkującego
    float Tp;                   // Okres próbkowania
    float value_min, value_max; // Min i max wartość sterowania
    float P;
    float I;
    float D;
};

unsigned char speedByPID(struct MotorPID &motorPID, float velocity) // Funkcja wyznaczająca prędkość napędu - algorytm PID | Main function for PID alghorithm.
{
    static unsigned long lastTime = 0; //
    static int safetyTrigger = 0;
    int safetyTime = 1000;
    static bool flaga = false; // flaga określająca czy juz wykryto zablokowanie napedu
    static bool saveValue = 0; // zmienna zapamiętująca poprzednie nastawy przed wykryciem przeciazenia
    // motorPID.output = 0; // 0 - 255
    velocity = abs(velocity);
    float e;
    float de;
    float I;

    motorPID.c_value = velocity;
    e = motorPID.setSpeed - motorPID.c_value;
    I = motorPID.Ki * motorPID.Tp * (e + motorPID.last_error) / 2 + motorPID.last_I;
    de = (e - motorPID.last_error) / motorPID.Tp;
    motorPID.last_error = e;
    motorPID.outputRPM = motorPID.Kp * e + motorPID.Kd * de + I;
    if ((motorPID.outputRPM > motorPID.value_max) || (motorPID.outputRPM < motorPID.value_min))
    {
        if (motorPID.outputRPM > motorPID.value_max)
        {
            motorPID.outputRPM = motorPID.value_max;
        }
        else if (motorPID.outputRPM < motorPID.value_min)
        {
            motorPID.outputRPM = motorPID.value_min;
        }
    }
    else
    {
        motorPID.last_I = I;
    }

    motorPID.output = 255 - (motorPID.outputRPM * 255) / motorPID.value_max;
    if (motorPID.output < 0)
        motorPID.output = 0;

    // Zabezpieczenie przed zatrzymanym napędem | Check if DC motor is blocked
    static int ledValue = 0;

    saveValue = motorPID.outputRPM;
/* ---------------------------- WYLACZONY TRYB SAFE ---------------------------------*/
/*
    if ((motorPID.setSpeed > 11) && (motorPID.c_value < 11))
    {
        safetyTrigger++;

        if ((millis() - lastTime >= 400))
        {
            ledValue = 1;
            //  digitalWrite(BOARD_LED,HIGH);
            lastTime = millis();
            digitalWrite(BOARD_LED, ledValue);
        }
        else
        {
            ledValue = 0;
            digitalWrite(BOARD_LED, ledValue);
            //   digitalWrite(BOARD_LED,LOW);
        }

        if (safetyTrigger > 75 && flaga == false)
        {
            motorPID.output = 255;
            motorPID.outputRPM = 255;
            Serial.println("Motor is blocked! Safe mode is activated");
            Serial.println("\t" + (String)safetyTrigger + "\t" + (String)flaga) + " " + (String)motorPID.outputRPM;
            flaga = true;
            // digitalWrite(BOARD_LED, LOW);
            return 255;
        }
        else if (flaga == true)
        {
            if (safetyTrigger > 250)
            {
                flaga = false;
                safetyTrigger = 0;
                motorPID.outputRPM = saveValue;
                digitalWrite(BOARD_LED, HIGH);
                return 50; // proba "zerwania" napedu
            }

            motorPID.output = 255;
            motorPID.outputRPM = 255;
            // Serial.println("Naped zablokowany (2)!");
            // Serial.println("\t" + (String)safetyTrigger + "\t" + (String)flaga) + " " + (String)motorPID.outputRPM;
            return 255;
        }
    }
    else
    {
        // flaga = false;
        // safetyTrigger = 0;
    }
*/
    return motorPID.output;
}

void setSpeedforPID(float setPoint, struct MotorPID &motorPID) // Funkcja ustawiająca przekazane dane nt. wartosci zadanej. Sprawdza czy nie przekroczono zakresu. | Function for set point value. 
{
    if (setPoint > motorPID.value_max)
    {
        motorPID.setSpeed = motorPID.value_max;
        Serial.println("[PID]: Set point over range! Set max possible value.");
    }
    else if (setPoint < motorPID.value_min)
    {
        motorPID.setSpeed = motorPID.value_min;
        Serial.println("[PID]: Set point over range! Set min possible value.");
    }
    else
    {
        motorPID.setSpeed = setPoint;
    }
}

void clearPID(struct MotorPID &motorPID) // Funkcja czyszcząca wartość zadaną oraz wysterowanie sterownika napędów. | Function for clear setpoint and output value.
{
    motorPID.setSpeed = 0;
    motorPID.output = 0;
}

