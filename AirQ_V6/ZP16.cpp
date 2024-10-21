#include "ZP16.h"

void ZP16::begin(Stream &serial)
{
	SerialPort = &serial;
	
}
bool ZP16::Lectura()
{
  uint8_t i =5;
  bool exito=false;
  while(i){
    i--;
    if(Peticion() && CheckDato())
    {
          exito=true;
          i=0;
    }
  }
    return exito;

}
bool ZP16::Peticion(){
  SerialPort->write(request,sizeof(request));
	SerialPort->flush();
  	if(SerialPort->available())
	{
		SerialPort->readBytes(recibido,sizeof(recibido));
    SerialPort->flush();
		return true;
	}else
		return false;
}
bool ZP16::CheckDato(){
  return (checksum(recibido,sizeof(recibido))== recibido[8] && recibido[0]==255 && recibido[1]==134 && recibido[7]==232);
}
void ZP16::ToInitiative()
{
	SerialPort->write(activeOn,sizeof(activeOn));
	SerialPort->flush();
}
void ZP16::ToQA()
{
	SerialPort->write(activeOff,sizeof(activeOff));
	SerialPort->flush();
}
float ZP16::GetHCHOmg()  // Calculo la concentracion en mg/m3 de HCHO
{
		return ((recibido[2]<<8 | recibido[3] )/10.0);
}

float ZP16::GetHCHOppm() //calculo basado en 25°C y 1 atmosfera
{
	return ((GetHCHOmg()*24.45)/PesoHCHO);
}

uint8_t ZP16::GetDato(uint8_t pos)
{
	return recibido[pos];
}
uint8_t ZP16::checksum(uint8_t *buff,uint8_t len) // checksum, calculado para verificar que el dato es correcto
{
  uint8_t j, tempq=0;
  for(j=1;j<len-1; j++)
  {
    tempq +=buff[j];
    
  }
  return((~tempq)+1);
}
unsigned char* ZP16::GetTrama(){
   Serial.println("Trama recibida");
   for(int i=0; i<(sizeof(recibido)-1);i++)
      {
        Serial.print(recibido[i],HEX);
        Serial.print("-");
      }
      Serial.println(recibido[8],HEX);
  return recibido;
}