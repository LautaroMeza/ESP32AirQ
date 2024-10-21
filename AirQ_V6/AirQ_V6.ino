#include <HardwareSerial.h>
#include <MHZ19.h>
#include "ZP16.h"
#include "ZH06.h"
#include <DHT.h>
#include <DHT_U.h>
#include "MQ7lib.h"
#include <ArduinoJson.h>
#include "time.h"
#include <esp_task_wdt.h>    //Watchdog
//Librerias para Conectarme a la base de datos.
#include <WiFi.h>
#include <FirebaseESP32.h>
// Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>


// RESERVAR TX2 GPIO17 ; RX2 GPIO16  --> IGUAL QUE EN IMAGEN PINOUT
StaticJsonDocument<256> jsonBuffer;

//definicion para watchdog
#define TWDT_TIMEOUT_MS 500000
#define CONFIG_FREERTOS_NUMBER_OF_CORES 2

// Definiciones para WiFi
#define WIFI_SSID "Wifi MEZA 2.4"
#define WIFI_PASSWORD "004398400268"

//Definiciones RTBD
#define DATABASE_URL "esp32-flutter-48c5b-default-rtdb.asia-southeast1.firebasedatabase.app/" //<databaseName>.firebaseio.com or <databaseName>.<region>.firebasedatabase.app
#define DATABASE_SECRET "vdUqM49LdA7uGT1IDxyF1hOb7JZFGwARqzzNtNxZ"
#define USER_MAIL "lautameza@hotmail.com"
#define USER_PWD  "marote123"
String uid = "94oShYXP9KWaxGYgICLRhC2FA4a2";
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

//Definiciones para DataLog
const char* ntpServer = "ar.pool.ntp.org";


//Definiciones para MQ7
#define placa "ESP-32"
#define Voltage_Resolution 4.75
#define ADC_voltage_resolution 1.325
#define gasPin 35 //Analog input 0 of your arduino
#define type "MQ-7" //MQ7
#define ADC_Bit_Resolution 12 // For arduino UNO/MEGA/NANO
#define RatioMQ7CleanAir 26.3 //27.5 //RS / R0 = 27.5 ppm  --> calculado por plot digitalizer
//Declaro el sensor
MQ7lib MQ7(placa,Voltage_Resolution, ADC_voltage_resolution, ADC_Bit_Resolution, gasPin, type);


// Definiciones para DHT11
#define DHTPIN 21  // pin donde leo dht
#define DHTTYPE DHT11 
DHT dht(DHTPIN, DHTTYPE);

//Definiciones para Sensores ZH06,MHZ19 y ZP16
HardwareSerial SerialPort(1); // incializo la UART1 
HardwareSerial SerialArd(2);
const uint8_t rx1 = 22; //asigno los pines para UART1
const uint8_t tx1 = 23;
const uint8_t rx2 = 16; //asigno los pines para UART2
const uint8_t tx2 = 17;
const uint8_t pinB = 32;
const uint8_t pinA = 33;
bool Wifi_Conn = false;
  ZP16 hcsens;
  ZH06 pmsens;
  MHZ19 co2sens;

//Variables para el programa
float valoresSens[8]; // los datos los ordeno asi--> hcho,co2,pm2.5,pm10,pm1.0,co, h,t

int logcounter =0;
uint8_t logcounteren=0;

//Interrupcion por TIMER
volatile int interruptCounter = 0;  //contador de interrupciones
volatile bool ingreso=false; // bandera que se modifica en la interrupcion
hw_timer_t * timer = NULL;    // variable que apuntará al timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; // Esta variable me permite sincronizar la Interrupcion y loop principal

void IRAM_ATTR onTimer(){  //Funcion ISR asociada a la interrupcion
  interruptCounter++;
  ingreso=true;
}
// Funcion para obtener el tiempo
unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}

esp_task_wdt_config_t twdt_config = {
        .timeout_ms = TWDT_TIMEOUT_MS,
        .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1,    // Bitmask of all cores
        .trigger_panic = true,
    };
void setup() {

  
  Serial.begin(115200,SERIAL_8N1); // inicializo UART0 para ver datos por monitor serie
  //Sensores MHZ-19, ZH06 y ZP16
  SerialPort.begin(9600,SERIAL_8N1,rx1,tx1); //inicializo la UART1 indicando baudios, tipo de comubnicacion, pines rx y tx
                                              //SERIAL_8N1 significa 8 bits de datos, paridad nula, 1 bit de stop 
  //Conexion con Arduino que domina la pantalla
  SerialArd.begin(9600,SERIAL_8N1,rx2,tx2); //inicializo la UART2 indicando baudios, tipo de comubnicacion, pines rx y tx
                                              //SERIAL_8N1 significa 8 bits de datos, paridad nula, 1 bit de stop 
 

   
    //esp_task_wdt_add(NULL);
  //Inicializacion de Sensores
  InicializarSensores();
  ConfigurarSensores();
     esp_task_wdt_deinit();
    Serial.println("watchdog deinit");
  configTime(0, 0, ntpServer);
  InitWiFi();
  InitBD();

 

  InitInterruptTimer();
  Firebase.getInt(fbdo,"/Contador"); 
  logcounter = fbdo.intData();
  CargaValoresMaximos();
}

void loop() 
{
  if(ingreso){
    switch(interruptCounter){
      case 1:{
        LeerSensores();}break;
      case 2:{
        Mostrar();
        CargarBD();
        CargarPantalla(); 
        interruptCounter =0;
        logcounteren++; 
        if(logcounteren>110){
          CargarRegistro();
          logcounteren=0;
        }
        }break;
      default:{
        interruptCounter =0;
      }break;
    }
    ingreso=false;
  }
 if(Serial.available()){         
   int r= Serial.read();
   if(r !='\n'){
     switch(r){
      case '1':{
        Serial.println("Interrupciones Activadas!");
        //timerAlarmEnable(timer);
        timerStart(timer);
      }break;
      case '2':{
        Serial.println("Interrupciones Desactivadas!");
        //timerAlarmDisable(timer);
        timerStop(timer);
      }break;
     }
    }
  }

}
void LeerSensores() {
  
  HabilitaSensores('h');           
  if(hcsens.Lectura()){
      valoresSens[0]=hcsens.GetHCHOppm();
  }else{
    Serial.println("no Hubo lectura HCHO");
    }  // Leo HCHO
  HabilitaSensores('c');
  valoresSens[1]=co2sens.getCO2();     // Leo Co2, 
  valoresSens[1]=co2sens.getCO2();
  HabilitaSensores('p');
  pmsens.lectura();
  if(pmsens.lectura())
  {
  
  valoresSens[2]= pmsens.GetPMug(0);  // Leo PM2.5 en ug/m3
  valoresSens[3]= pmsens.GetPMug(1); // Leo PM10 en ug/m3
  valoresSens[4]= pmsens.GetPMug(2); // Leo PM1.0 en ug/m3

  }else{
    Serial.println("sin lectura PM");
  }
  MQ7.update();
  valoresSens[5]= MQ7.readSensor();
  valoresSens[6] = dht.readHumidity(); // lectura de humedad en %
  valoresSens[7]= dht.readTemperature(); // lectura de temperatura en Celsius
   
}
void CargarBD(){ //Cargo en la base de datos los valores de los sensores

  
  Firebase.set(fbdo,"/Actual/HCHO",valoresSens[0]);
  Firebase.set(fbdo,"/Actual/CO2",valoresSens[1]);
  Firebase.set(fbdo,"/Actual/PM2_5",valoresSens[2]);
  Firebase.set(fbdo,"/Actual/PM_10",valoresSens[3]);
  Firebase.set(fbdo,"/Actual/PM_1",valoresSens[4]);  
  Firebase.set(fbdo,"/Actual/CO",valoresSens[5]);
  Firebase.set(fbdo,"/Actual/Humedad",valoresSens[6]);
  Firebase.set(fbdo,"/Actual/Temperatura",valoresSens[7]);  
  
}
void CargaValoresMaximos(){ // Valores maximos significa el maximo permisible de concentracion en ppm. o en su defecto la maxima concentracion detectable por el sensor.
  Firebase.set(fbdo,"/ValoresMaximos/HCHO",3); //8.15 es el maximo medible por el sensor -- 0.5 a 3 ppm empiezan los problenas
  Firebase.set(fbdo,"/ValoresMaximos/CO2",10000); // 2000;5000;10000 son los rangos que puede medir el sensor -- maximo es 15mil pero por 8 horas, se permite 5000.
  Firebase.set(fbdo,"/ValoresMaximos/CO",50);  // 4000 es el rango maximo medible -- 50 ppm en 8 horas
  Firebase.set(fbdo,"/ValoresMaximos/PM_1",25); // supuestamente es 1000 ug/m3
  Firebase.set(fbdo,"/ValoresMaximos/PM2_5",70); // supuestamente es 1000 ug/m3 -- 
  Firebase.set(fbdo,"/ValoresMaximos/PM1_0",50); // supuestamente es 1000 ug/m3 -- Para la salud por 8 horas
  Firebase.set(fbdo,"/ValoresMaximos/Temperatura",50); // 50 °C maximo del sensor
  Firebase.set(fbdo,"/ValoresMaximos/Humedad",90);  // 90 % maximo del sensor

}
void CargarRegistro(){
  int timestamp = getTime();
  if(logcounter<120){
     Firebase.set(fbdo,"/Registros/"+String(logcounter)+"/HCHO",valoresSens[0]);
     Firebase.set(fbdo,"/Registros/"+String(logcounter)+"/CO2",valoresSens[1]);
     Firebase.set(fbdo,"/Registros/"+String(logcounter)+"/PM2_5",valoresSens[2]);
     Firebase.set(fbdo,"/Registros/"+String(logcounter)+"/PM_1",valoresSens[3]);
     Firebase.set(fbdo,"/Registros/"+String(logcounter)+"/PM1_0",valoresSens[4]);
     Firebase.set(fbdo,"/Registros/"+String(logcounter)+"/CO",valoresSens[5]);
     Firebase.set(fbdo,"/Registros/"+String(logcounter)+"/Humedad",valoresSens[6]);
     Firebase.set(fbdo,"/Registros/"+String(logcounter)+"/Temperatura",valoresSens[7]);
     Firebase.set(fbdo,"/Registros/"+String(logcounter)+"/TimeStamp",timestamp);
     Firebase.setInt(fbdo,"/Contador",logcounter);
     logcounter++;

  }else
  {
    logcounter=0;
  }
}
void CargarPantalla(){ //Envio por puerto serie los valores de los sensores
  JsonObject json = jsonBuffer.createNestedObject();  //Creo objeto JSon
  json["Operacion"]="m";
  json["HCHO"] = valoresSens[0];
  json["CO2"] = valoresSens[1];
  json["PM 2.5"] = valoresSens[2];
  json["PM 10"] = valoresSens[3];
  json["PM 1"] = valoresSens[4];
  json["CO"]=valoresSens[5];
  json["%H"] = valoresSens[6];
  json["TEMP"] = valoresSens[7];
  json["Coneccion"] = Wifi_Conn;
  serializeJson(json,SerialArd);
  
  jsonBuffer.clear();
  SerialArd.flush();
   /*StaticJsonDocument<256> jsonBuffer;
  jsonBuffer["Operacion"]="m";
  jsonBuffer["HcHo"] = valoresSens[0];
  jsonBuffer["CO2"] = valoresSens[1];
  jsonBuffer["PM 2.5"] = valoresSens[2];
  jsonBuffer["PM 10"] = valoresSens[3];
  jsonBuffer["PM 1"] = valoresSens[4];
  jsonBuffer["CO"]=valoresSens[5];
  jsonBuffer["%H"] = valoresSens[6];
  jsonBuffer["ºC"] = valoresSens[7];
  String buff;
  serializeJson(jsonBuffer,SerialArd);
  SerialArd.flush();*/  //Funciona
  
  
  
}
void Mostrar(){  //Escribo en monitor serie los valores leidos.

  Serial.print("HCHO:  ");
  Serial.print(valoresSens[0]);
  Serial.println("ppm");
  Serial.print("CO2:  ");
  Serial.print(valoresSens[1]);
  Serial.println("ppm");
  Serial.print("PM2.5: ");
  Serial.print(valoresSens[2]);
  Serial.println("ug/m3");
  Serial.print("PM10: ");
  Serial.print(valoresSens[3]);
  Serial.println("ug/m3");
  Serial.print("PM1.0:  ");
  Serial.print(valoresSens[4]);
  Serial.println("ug/m3");
  Serial.print("CO:  ");
  Serial.print(valoresSens[5]);
  Serial.println("ppm");
  Serial.print("H:  ");
  Serial.print(valoresSens[6]);
  Serial.println("%");
  Serial.print("T:  ");
  Serial.print(valoresSens[7]);
  Serial.println("°C");
  Serial.println("...............................................................");
}
void ConfigurarSensores(){

  CalibrarMQ7();
  HabilitaSensores('p');
  delay(10);
  pmsens.ToQA();
  pmsens.ToQA();
  HabilitaSensores('h');
  delay(10);
  hcsens.ToQA();
  hcsens.ToQA();
  HabilitaSensores('c');
  co2sens.setRange(10000);
  co2sens.setRange(10000);
  MQ7.serialDebug(true);
}
void InicializarSensores(){
  pmsens.begin(SerialPort);
  co2sens.begin(SerialPort);	
  hcsens.begin(SerialPort);	  
  Serial.flush();

  pinMode(pinA,OUTPUT);
  pinMode(pinB,OUTPUT);
  // DHT11
  dht.begin();

  //MQ7
  Configura_adc(); 
  MQ7.setRegressionMethod(1); 
  MQ7.setA(98.045);
  MQ7.setB(-1.533);
  MQ7.setRL(1);
  MQ7.init();
}
void HabilitaSensores(char opc){ // c: MHZ19-CO2 ; h: ZP16-HCHO ; p: ZH06-PM
  switch(opc)
  {
    case 'c':{
      digitalWrite(pinA,HIGH);
      digitalWrite(pinB,LOW);

    }break;
    case 'h':{
      digitalWrite(pinA,LOW);
      digitalWrite(pinB,HIGH);
    }break;
    case 'p':{
      digitalWrite(pinA,LOW);
      digitalWrite(pinB,LOW);
    }break;
    default:{    // para dejar habilitado un canal sin uso
      digitalWrite(pinA,HIGH);
      digitalWrite(pinB,HIGH);
    }break;

  }
}
void MuestraTrama(unsigned char *rec){
    Serial.println("Trama recibida");
   for(int i=0; i<(sizeof(rec)-1);i++)
      {
        Serial.print(rec[i],HEX);
        Serial.print("-");
      }
      Serial.println(rec[8],HEX);
    
 
}
void Configura_adc(){
  analogReadResolution(12);
  analogSetAttenuation(ADC_2_5db); // Set la atenuacion de todos los canales de adc pueden ser 0, 2.5, 6 ,11 db (MIRAR LA TABLA ANTES)
  analogSetPinAttenuation(gasPin,ADC_2_5db); // Set la atenuacion de un solo pin.
          // 11 db --> 3.1 V
          // 6 db --> 1.835
          // 2.5 db --> 1.325
          // 0 db -->  1 V
}
void CalibrarMQ7(){
   // Calibramos y Calculamos el R0
  Serial.print("Preparando Sensores. Espere");
  float calcR0 = 0;
  for(int i = 1; i<=30; i ++)
  {
    MQ7.update(); // actualizo el dato, esp32 lee el dato del pin analogico
    calcR0 += MQ7.calibrate(RatioMQ7CleanAir);
    Serial.print(".");
    delay(50);
  }
  MQ7.setR0(calcR0/30);
  Serial.println("  done!.");
     
  if(isinf(calcR0)) {
    Serial.println("Error: Fallo de coneccion MQ7, R0 es infinito (circuito abierto detectado) checkear cableado y alimentacion");
    esp_task_wdt_init(&twdt_config);
    esp_task_wdt_add(NULL);
  while(1);
    }
  if(calcR0 == 0){
    Serial.println("Error: Fallo de coneccion MQ7, R0 es cero (lectura Analog pin da cero) checkear cableado y alimentacion");
    esp_task_wdt_init(&twdt_config);
       esp_task_wdt_add(NULL);
   while(1);
    }
  esp_task_wdt_deinit();
}
void InitWiFi(){
    //inicializacion de WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(300);
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Wifi_Conn = true;
    Serial.println();
    
    Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

}
void InitBD(){
      //Inicializacion de Base de datos y conexion con FireBase
    config.database_url = DATABASE_URL;
    config.signer.tokens.legacy_token = DATABASE_SECRET;
    Firebase.reconnectNetwork(true);
    fbdo.setBSSLBufferSize(4096 /* Rx buffer size in bytes from 512 - 16384 */, 1024 /* Tx buffer size in bytes from 512 - 16384 */);
    Firebase.begin(&config, &auth);
}
void InitInterruptTimer(){  //interrupciones cada un segundo
  //timer = timerBegin(0,80,true); // 0 apunta al primer temporizador; 80 es el preescaler ; true  es cuenta ascendente
    timer =timerBegin(1e6);
  //timerAttachInterrupt(timer,&onTimer,true); // paso la variable timer (puntero al temporizador inicializado), la funcion ISR y true para activar por desborde
  timerAttachInterrupt(timer,&onTimer);
  // Activo la alarma y seteo el contador del temporizador 
  //timerAlarmWrite(timer,1e6,true); //Paso el timer inicializado, el valor del contador y true para que se reinicie la el contador del timer
  //timerAlarmEnable(timer); //luego de configurado lo habilito
  timerAlarm(timer, 1000000, true, 0);
  

}