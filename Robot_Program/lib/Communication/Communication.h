/*--[COMMUNICATION FILE]-------------------------------------------------------------------------------------
 *  Politechnika Śląska - Wydział Automatyki Elektroniki i Informatyki. Kierunek: Automatyka i Robotyka
 *  Specjalność: Technologie Informacyjne.
 *  Praca inżynierska: System sterowania robotem mobilnym z kołami szwedzkimi.
 * 
 *  Autor: Jakub Kaniowski   |   Plik utworzono: 20.10.2021 | Ujednolicono: 20.10.2021
 *-------------------------------------------------------------------------------------------------------
 *  Zawiera:
 * 
 *      Konfiguracje tworzonego punktu dostępowego sieci WIFI.
 * 
 *------------------------------------------------------------------------------------------------------- 
 */

#define SEND_TO_MASTERAPP_TRIGGER 50 //50 wysylka danych do operatora | 50ms send to remote host
#define DATA_SIZE 9 // spodziewana ilość przyjętych danych z sieci | number of received data from remote host

unsigned long int sendData_millis = 0;

char ssid[] = "ROBO-1";         //Nazwa SSID sieci | Name of created network
char password[] = "Robot1234";  //Hasło do sieci | Network password
int communication_port = 1000;  //Port komunikacyjny (odbierający dane) 


char masterAPP_IP [] = "192.168.0.2"; //Adres ip dla aplikacji sterującej na zdalnym hoscie | IP Address for remote controller
IPAddress masterAPP_IP_ADDRESS(192,168,0,2);
int masterAPP_communication_port = 60890; //Port komunikacyjny (wskazujący hosta)


String header;

IPAddress localIp(192,168,0,1);     //Adres IP urządzenia. | IP address for robot.
IPAddress gateway(192,168,0,1);     //Brama domyślna urządzenia (taka sama j/w) | Gateway address
IPAddress subnet(255,255,255,252);  //2 hostowa sieć (maska /30) | Netmask for 2 devices network


