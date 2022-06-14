/*--[SENSORS FILE]---------------------------------------------------------------------------------------
 *  Politechnika Śląska - Wydział Automatyki Elektroniki i Informatyki. Kierunek: Automatyka i Robotyka
 *  Specjalność: Technologie Informacyjne.
 *  Praca inżynierska: System sterowania robotem mobilnym z kołami szwedzkimi.
 * 
 *  Autor: Jakub Kaniowski   |   Plik utworzono: 28.08.2021 | Ujednolicono: 08.09.2021
 *-------------------------------------------------------------------------------------------------------
 *  Zawiera:
 * 
 *      Główne struktury danych dla urządzeń pomiarowych. Zaimplementowano prosty odczyt danych 
 *      z kompasu (magnetometru) oraz czujnika przyspieszeń IMU.
 *      Wywoływane w pliku main.cpp. Deklaracje konstruktorów znajdują się tutaj.
 * 
 *------------------------------------------------------------------------------------------------------- 
 */



  /**/
// Struktura przechowująca dane kompasu. Dokładne położenie w stopniach, lub kierunek geograficzny | Struct for magnetometer sensor
struct COMPASS
{
    byte azimuth; //zwraca dane liczbowe. wartości około 200 to np. południe. | Geographical direction as number
    char directionName[3];
    int x, y, z;
};

//Struktura przechowująca odczyty z IMU. Zawiera przyspieszenia poszczególnych osi oraz rotacje. | Struct for IMU sensor
struct IMU
{
    float acc_x, acc_y, acc_z;
    float rot_x, rot_y, rot_z;
    float temperature;
};

/*  Deklaracje konstruktorów  | Declarations of sensors libraries  */
QMC5883LCompass compass;
Adafruit_MPU6050 mpu;

/*  Okomentować w przypadku niekorzystania z czujnika Ultradźwiękowego  | Comment if ultrasonic sensor isn't available  */
//  UltraSonicDistanceSensor distanceSensor(21, 22);

/*  Definicje funkcji  | Function definitions */
void compass_xyz(COMPASS &dataCompass)
{
    compass.read();
    dataCompass.azimuth = compass.getAzimuth();
    dataCompass.x = compass.getX();
    dataCompass.y = compass.getY();
    dataCompass.z = compass.getZ();
    /* Debug
    compass.getDirection(dataCompass.directionName[3], dataCompass.azimuth);
    Serial.print(myArray[0]);
    Serial.print(myArray[1]);
    Serial.print(myArray[2]);
    Serial.print(a);
    Serial.println();
    Serial.print("X: ");
    Serial.print(x);
    Serial.print(" Y: ");
    Serial.print(y);
    Serial.print(" Z: ");
    Serial.print(z);
    Serial.println();
  */
}

void acc_sensor(IMU &dataIMU)
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    dataIMU.acc_x = a.acceleration.x;
    dataIMU.acc_y = a.acceleration.y;
    dataIMU.acc_z = a.acceleration.z;

    dataIMU.rot_x = g.gyro.x;
    dataIMU.rot_y = g.gyro.y;
    dataIMU.rot_z = g.gyro.z;

    dataIMU.temperature = temp.temperature;
    /* Debug
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");
    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");
  */
}

float usonic_sensor()  // Funkcja mierząca odległość | Function for measurement distance
{
  return distanceSensor.measureDistanceCm();
}

