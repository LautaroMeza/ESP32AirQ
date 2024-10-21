#include "ZH06.h"

void ZH06::begin(Stream &stream)
{
	SerialPort = &stream;
}
bool ZH06::lectura()
{
  uint8_t i =5;
  bool exito=false;
  while(i){
    i--;
    if(Peticion() && CheckDdato())
    {
          valores[0]= (recibido[2]<<8 | recibido[3]); //concentracion PM2.5 en ug
          valores[1]= (recibido[4]<<8 | recibido[5]); //concentracion PM10 en ug
          valores[2]= (recibido[6]<<8 | recibido[7]);//concentracion PM1.0 en ug
          exito=true;
          i=0;
    }
  }
    return exito;
 
}
bool ZH06::Peticion(){
  SerialPort->flush();
	SerialPort->write((requestPM),sizeof(requestPM));
  
	if(SerialPort->available())
	{
		SerialPort->readBytes(recibido,sizeof(recibido));

    SerialPort->flush();
    return true;
    }else
      return false;
}
bool ZH06::CheckDdato(){
return (checksum(recibido,sizeof(recibido),modQA)== recibido[8] && recibido[0]==255 && recibido[1]==134);
}
void ZH06::ToInitiative()
{
	SerialPort->write(initMode,sizeof(initMode));
  SerialPort->flush();
	modQA=false;
}
void ZH06::ToQA()
{
	SerialPort->write(qaMode,sizeof(qaMode));
  SerialPort->flush();
	modQA=true;
}
uint8_t ZH06::Toinactive(bool activ)
{
	uint8_t rec[9];
	if(activ)
		if(!modSleep)
		{
			SerialPort->write(sleepMode,sizeof(sleepMode));
      SerialPort->flush();
			modSleep =true;
		}
	else
	{
		if(modSleep)
		{
			sleepMode[3]= 0x00;
			sleepMode[8]=0x58;
			SerialPort->write(sleepMode,sizeof(sleepMode));
      SerialPort->flush();
			modSleep=false;
		}
	}
	if(SerialPort->available())
		SerialPort->readBytes(rec,sizeof(rec));
  SerialPort->flush();
	return rec[2];
	
}
float ZH06::GetPMug(uint8_t pos)
{
 
  return valores[pos];
}
float ZH06::GetPMppm(uint8_t pos)  // no me sirve porque los valores son muy cercanos a cero
{

  switch(pos)
  {
    case 0:
    {
      return ((24.45*(valores[pos]/1000))/3625); //PM2.5 en pppm
    }break;
    case 1:
    {
      return ((24.45*(valores[pos]/1000))/1450); //PM10 en pppm
    }break;
    case 2:
    {
      return ((24.45*(valores[pos]/1000))/145);  //PM1.0 en pppm
    }break;
  }
}
uint8_t ZH06::GetDato(uint8_t pos)
{
  return recibido[pos];
}
uint8_t ZH06::checksum(uint8_t *buff, uint8_t len,bool QA)
{
	uint8_t j, tempq=0;
  if(QA)
  {
   for(j=1;j<len-1; j++)
    {
     tempq +=buff[j];
    
    }
    tempq= ((~tempq)+1);
  }else
   {
    for(j=0;j<len-2; j++)
    {
     tempq +=buff[j];
    
    }
    tempq= (buff[30]<<8 | buff[31]);
   }
  return tempq;
}
unsigned char* ZH06::GetTrama(){
   Serial.println("Trama recibida");
   for(int i=0; i<(sizeof(recibido)-1);i++)
      {
        Serial.print(recibido[i],HEX);
        Serial.print("-");
      }
      Serial.println(recibido[8],HEX);
  return recibido;
}
