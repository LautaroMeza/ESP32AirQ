#include "MQ7lib.h"

MQ7lib::MQ7lib(String Placa, float Voltage_Resolution,float ADC_voltage_resolution, int ADC_Bit_Resolution, int pin, String type) {
  this->_pin = pin;
  Placa.toCharArray(this->_placa, 20);
  type.toCharArray(this->_type, 6);
  //this->_type = type; //MQ-2, MQ-3 ... MQ-309A
  //this->_placa = Placa;
  this-> _VOLT_RESOLUTION = Voltage_Resolution;
  this-> _ADC_voltage_resolution =  ADC_voltage_resolution;
  this-> _ADC_Bit_Resolution = ADC_Bit_Resolution;
}
MQ7lib::MQ7lib(String Placa, String type) {
  Placa.toCharArray(this->_placa, 20);
  type.toCharArray(this->_type, 6);
}
void MQ7lib::init()
{
  pinMode(_pin, INPUT);
}
void MQ7lib::setA(float a) {
  this->_a = a;
}
void MQ7lib::setB(float b) {
  this->_b = b;
}
void MQ7lib::setR0(float R0) {
  this->_R0 = R0;
}
void MQ7lib::setRL(float RL) {
  this->_RL = RL;
}
void MQ7lib::setADC(int value)
{
  this-> _sensor_volt = (value) * _ADC_voltage_resolution / ((pow(2, _ADC_Bit_Resolution)) - 1); 
  this-> _adc =  value;
}
void MQ7lib::setVoltResolution(float voltage_resolution)
{
  _VOLT_RESOLUTION = voltage_resolution;
}
void MQ7lib::setADCVoltResolution( float ADC_voltage_resolution)
{
	_ADC_voltage_resolution = ADC_voltage_resolution;
}
void MQ7lib::setRegressionMethod(int regressionMethod)
{
  //this->_regressionMethod = regressionMethod;
  this->_regressionMethod = regressionMethod;
}
float MQ7lib::getR0() {
  return _R0;
}
float MQ7lib::getRL() {
  return _RL;
}
float MQ7lib::getVoltResolution()
{
  return _VOLT_RESOLUTION;
}
String MQ7lib::getRegressionMethod()
{
  if(_regressionMethod == 1) return "Exponential";
  else return "Linear";
}
float MQ7lib::getA() {
  return _a;
}
float MQ7lib::getB() {
  return _b;
}
void MQ7lib::serialDebug(bool onSetup)
{
  if(onSetup)
  {
    Serial.println();
    Serial.println("************************************************************************************************************************************************");
    Serial.println("Lectura de datos de sensor MQ7");


    Serial.print("Sensor: "); Serial.println(_type);
    Serial.print("Alimentacion sensor: "); Serial.print(_VOLT_RESOLUTION); Serial.println(" VDC");
	Serial.print("voltaje ADC: "); Serial.print(_ADC_voltage_resolution); Serial.println(" VDC");
    Serial.print("Resolucion ADC: "); Serial.print(_ADC_Bit_Resolution); Serial.println(" Bits");
    Serial.print("R0: "); Serial.print(_R0); Serial.println(" KΩ");
    Serial.print("RL: "); Serial.print(_RL); Serial.println(" KΩ");

    Serial.print("Model: "); if(_regressionMethod == 1) Serial.println("Exponencial"); else Serial.println("Lineal");
    Serial.print(_type); Serial.print(" -> a: "); Serial.print(_a); Serial.print(" | b: "); Serial.println(_b);

    Serial.print("Placa de desarrollo: "); Serial.println(_placa);
  }
  else 
  {
    if(!_firstFlag)
    {
      Serial.print("| ********************************************************************"); Serial.print(_type); Serial.println("*********************************************************************|");
      Serial.println("|ADC_In | Equation_V_ADC | Voltage_ADC |        Equation_RS        | Resistance_RS  |    EQ_Ratio  | Ratio (RS/R0) | Equation_PPM |     PPM    |");
      _firstFlag = true;  //Headers are printed
    }
    else
    {
      Serial.print("|"); Serial.print(_adc);  Serial.print("| v = ADC*"); Serial.print(_ADC_voltage_resolution); Serial.print("/"); Serial.print((pow(2, _ADC_Bit_Resolution)) - 1); Serial.print("  |    "); Serial.print(_sensor_volt);
      Serial.print("     | RS = ((" ); Serial.print(_VOLT_RESOLUTION ); Serial.print("*RL)/Voltage) - RL|      "); Serial.print(_RS_Calc); Serial.print("     | Ratio = RS/R0|    ");
      Serial.print(_ratio);  Serial.print( "       |   ");
      if(_regressionMethod == 1) Serial.print("ratio*a + b");
      else Serial.print("pow(10, (log10(ratio)-b)/a)");
      Serial.print("  |   "); Serial.print(_PPM); Serial.println("  |");
    }
  }
}
void MQ7lib::update()
{
  _sensor_volt = this->getVoltage();
}
void MQ7lib::externalADCUpdate(float volt)
{
  _sensor_volt = volt;
}
float MQ7lib::validateEcuation(float ratioInput)
{
  //Serial.print("Ratio input: "); Serial.println(ratioInput);
  //Serial.print("a: "); Serial.println(_a);
  //Serial.print("b: "); Serial.println(_b);
  //Usage of this function: Unit test on ALgorithmTester example;
  if(_regressionMethod == 1) _PPM= _a*pow(ratioInput, _b);
  else 
  {
      // https://jayconsystems.com/blog/understanding-a-gas-sensor
      double ppm_log = (log10(ratioInput)-_b)/_a; //Get ppm value in linear scale according to the the ratio value  
      _PPM = pow(10, ppm_log); //Convert ppm value to log scale  
  }
  //Serial.println("Regression Method: "); Serial.println(_regressionMethod);
  //Serial.println("Result: "); Serial.println(_PPM);
  return _PPM;  
}
float MQ7lib::readSensor(bool isMQ303A, float correctionFactor, bool injected)
{
  //More explained in: https://jayconsystems.com/blog/understanding-a-gas-sensor
  if(isMQ303A) {
    _VOLT_RESOLUTION = _VOLT_RESOLUTION - 0.45; //Calculations for RS using mq303a sensor look wrong #42
  }
  _RS_Calc = ((_VOLT_RESOLUTION*_RL)/_sensor_volt)-_RL; //Get value of RS in a gas
  if(_RS_Calc < 0)  _RS_Calc = 0; //No negative values accepted.
  if(!injected) _ratio = _RS_Calc / this->_R0;   // Get ratio RS_gas/RS_air
  _ratio += correctionFactor;
  if(_ratio <= 0)  _ratio = 0; //No negative values accepted or upper datasheet recomendation.
  if(_regressionMethod == 1) _PPM= _a*pow(_ratio, _b); // <- Source excel analisis https://github.com/miguel5612/MQSensorsLib_Docs/tree/master/Internal_design_documents
  else 
  {
    // https://jayconsystems.com/blog/understanding-a-gas-sensor <- Source of linear ecuation
    double ppm_log = (log10(_ratio)-_b)/_a; //Get ppm value in linear scale according to the the ratio value  
    _PPM = pow(10, ppm_log); //Convert ppm value to log scale  
  }
  if(_PPM < 0)  _PPM = 0; //No negative values accepted or upper datasheet recomendation.
  //if(_PPM > 10000) _PPM = 99999999; //No negative values accepted or upper datasheet recomendation.
  return _PPM;
}
float MQ7lib::readSensorR0Rs()
{
 
  _RS_Calc = ((_VOLT_RESOLUTION*_RL)/_sensor_volt)-_RL; //Get value of RS in a gas
  if(_RS_Calc < 0)  _RS_Calc = 0; //No negative values accepted.
  _ratio = this->_R0/_RS_Calc;   // Get ratio RS_air/RS_gas <- INVERTED for MQ-131 issue 28 https://github.com/miguel5612/MQSensorsLib/issues/28
  if(_ratio <= 0)  _ratio = 0; //No negative values accepted or upper datasheet recomendation.
  if(_regressionMethod == 1) _PPM= _a*pow(_ratio, _b); // <- Source excel analisis https://github.com/miguel5612/MQSensorsLib_Docs/tree/master/Internal_design_documents
  else 
  {
   
    double ppm_log = (log10(_ratio)-_b)/_a; //Get ppm value in linear scale according to the the ratio value  
    _PPM = pow(10, ppm_log); //Convert ppm value to log scale  
  }
  if(_PPM < 0)  _PPM = 0; //No negative values accepted or upper datasheet recomendation.
  //if(_PPM > 10000) _PPM = 99999999; //No negative values accepted or upper datasheet recomendation.
  return _PPM;
}
float MQ7lib::getPPM(){
  return _PPM;
}
float MQ7lib::calibrate(float ratioInCleanAir) {
 
  /*
  V = I x R 
  VRL = [VC / (RS + RL)] x RL 
  VRL = (VC x RL) / (RS + RL) 
  Así que ahora resolvemos para RS: 
  VRL x (RS + RL) = VC x RL
  (VRL x RS) + (VRL x RL) = VC x RL 
  (VRL x RS) = (VC x RL) - (VRL x RL)
  RS = [(VC x RL) - (VRL x RL)] / VRL
  RS = [(VC x RL) / VRL] - RL
  */
  float RS_air; //Define variable for sensor resistance
  float R0; //Define variable for R0
  RS_air = ((_VOLT_RESOLUTION*_RL)/_sensor_volt)-_RL; //Calculate RS in fresh air
  if(RS_air < 0)  RS_air = 0; //No negative values accepted.
  R0 = RS_air/ratioInCleanAir; //Calculate R0 
  if(R0 < 0)  R0 = 0; //No negative values accepted.
  return R0;
}
float MQ7lib::getVoltage(bool read, bool injected, int value) {
  float voltage;
  if(read)
  {
    float avg = 0.0;
    for (int i = 0; i < retries; i ++) {
      _adc = analogRead(this->_pin);
      avg += _adc;
      delay(retry_interval);
    }
    voltage = (avg/ retries) * _ADC_voltage_resolution / ((pow(2, _ADC_Bit_Resolution)) - 1);
  }
  else if(!injected)
  {
    voltage = _sensor_volt;
  }
  else 
  {
    voltage = (value) * _ADC_voltage_resolution / ((pow(2, _ADC_Bit_Resolution)) - 1);
    _sensor_volt = voltage; //to work on testing
  }
  return voltage;
}
float MQ7lib:: setRsR0RatioGetPPM(float value)
{
  _ratio = value;
  return readSensor(false, 0, true);
}
float MQ7lib::getRS()
{
  _RS_Calc = ((_VOLT_RESOLUTION*_RL)/_sensor_volt)-_RL; //Get value of RS in a gas
  if(_RS_Calc < 0)  _RS_Calc = 0; //No negative values accepted.
  return _RS_Calc;
}

float MQ7lib::stringTofloat(String & str)
{
  return atof( str.c_str() );
}