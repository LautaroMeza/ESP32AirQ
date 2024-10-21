#ifndef ZP16_H
#define ZP16_H
#include <Arduino.h>
#define PesoHCHO 30.031

  
class ZP16
 {
	public:
	
	void begin(Stream &stream);
	void ToInitiative();
	void ToQA();
	float GetHCHOmg();
	float GetHCHOppm();
	uint8_t GetDato(uint8_t pos);
  unsigned char* GetTrama();
  bool Lectura();
	private:
	
	 uint8_t recibido[9];  // Array de datos que voy a recibir
   uint8_t activeOff[9] = {0xFF,0x00,0x78,0x41,0x00,0x00,0x00,0x00,0x47};  // Comando para pasar de Active mode a QA  mode
	 uint8_t activeOn[9]= {0xFF,0x00,0x78,0x40,0x00,0x00,0x00,0x00,0x48};  //  Comando para pasar de QA mode a Active mode
	 uint8_t request[9]= {0xFF,0x00,0x86,0x00,0x00,0x00,0x00,0x00,0x7A};   // Comando de peticion de datos en QA mode
	 Stream* SerialPort;
	 uint8_t checksum(uint8_t *buff, uint8_t len);
	 
   bool Peticion();
   bool CheckDato();
 };
 #endif
 