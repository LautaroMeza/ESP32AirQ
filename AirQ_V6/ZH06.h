#ifndef ZH06_H
#define ZH06_H
#include <Arduino.h>


class ZH06
 {
  	private:
	
	 byte recibido[9];  // Array de datos que voy a recibir
	 uint8_t valores[3]; // datos recibidos
	 uint8_t qaMode[9]= {0xFF,0x01,0x78,0x41,0x00,0x00,0x00,0x00,0x46}; //paso al modo QA
   uint8_t initMode[9]= {0xFF,0x01,0x78,0x40,0x00,0x00,0x00,0x00,0x47}; //paso al modo initiative
	 uint8_t sleepMode[9]= {0xFF,0x01,0xA7,0x01,0x00,0x00,0x00,0x00,0x57};  //paso al modo sleep
	 uint8_t requestPM[9]= {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};   // Comando de peticion de datos en QA mode 
   
	 bool modQA;
	 bool modSleep;
   
	 Stream* SerialPort;
	 uint8_t checksum(uint8_t *buff, uint8_t len,bool QA);
   bool Peticion();
   bool CheckDdato();
	

	public:

	void begin(Stream &stream);
	void ToInitiative();
	void ToQA();
	uint8_t Toinactive(bool activ);
	float GetPMug(uint8_t pos); // devuelvo el dato de concentracion segun la posicion que indique
	float GetPMppm(uint8_t pos);
	uint8_t GetDato(uint8_t pos); // devuelve el dato crudo en la posicion indicada
  unsigned char* GetTrama();
  bool lectura();


 };
 #endif