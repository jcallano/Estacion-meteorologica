#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>


// pines gps
static const int RXPin = 10, TXPin = 11;
static const uint32_t GPSBaud = 4800;  //vel serial gps


unsigned long time;

// DHT22 sensor pins
#define DHTPIN 14
#define DHTTYPE DHT22
// DHT & BMP instances
DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);




// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
String posicion;
String fecha;


int sensorgasco = A0; // Pin donde esta conectado el sensor de gas MQ7 gas CO
int sensorhumo = A1; // Pin donde esta conectado el sensor de humo MQ2
int sensorluz = A2; // Pin donde esta conectado el sensor LDR
int sensorruido = A3; // Pin donde esta conectado el sensor nivel de ruido
int sensoroh = A4; // Pin donde esta conectado el sensor alcohol MQ3
int sensorpresipitacion = A5; // Pin donde esta conectado el sensor de precipitacion
int sensoruv = A6; // Pin donde esta conectado el sensor radiacion UV
int sensorcorriente = A7; // Pin donde esta conectado el sensor corriente que se usara para medir estado consumo bateria
int volbat = A8; // Pin donde esta conectado el sensor nivel de ruido
int volpanel = A9; // Pin donde esta conectado el sensor nivel de ruido
int volelectronica = A10; // Pin donde esta conectado el sensor nivel de ruido


/***************************************************
 *  Nombre Funcion:        displaySensorDetails
 *
 *  Returns:               Nada.
 *
 *  Parametros:            Ninguno
 *
 *  Descripcion:           Muestra informacion sobre el sensor bmp085 conectado, Si no hay sensor muestra error
 *
 ***************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");  
  Serial.println("------------------------------------");
  Serial.println("");
  //delay(500);
}




/***************************************************
 *  Nombre Funcion:        presionatmosferica
 *
 *  Returns:               float con valor de presion atmosferica detectado por el sensor bmp085.
 *
 *  Parametros:            Ninguno
 *
 *  Descripcion:           El sensor debe estar conectado por i2c. Encaso de arduino Mega pines 20,21; encaso Arduino nano pines A4 y A5
 ***************************************************/


float presionatmosferica(){
   sensors_event_t event;
   bmp.getEvent(&event);
    if (event.pressure){
    //Serial.print("Pressure:    ");
    //Serial.print(event.pressure);
    //Serial.println(" hPa");
   
    return event.pressure;
  }
  else
  {
    Serial.println("error sensor bmp85");
    return 0;
  }
}

/***************************************************
 *  Nombre Funcion:        temperaturaatmosferica
 *
 *  Returns:               float con valor de  temperatura detectado por el sensor bmp085.
 *
 *  Parametros:            Ninguno
 *
 *  Descripcion:           El sensor debe estar conectado por i2c. Encaso de arduino Mega pines 20,21; encaso Arduino nano pines A4 y A5
 ***************************************************/

float temperaturaatmosferica(){
   sensors_event_t event;
   bmp.getEvent(&event);
    if (event.pressure)
  {
    /* Display atmospheric pressue in hPa */
    //Serial.print("Pressure:    ");
    //Serial.print(event.pressure);
    //Serial.println(" hPa");
    
    /* Calculating altitude with reasonable accuracy requires pressure    *
     * sea level pressure for your position at the moment the data is     *
     * converted, as well as the ambient temperature in degress           *
     * celcius.  If you don't have these values, a 'generic' value of     *
     * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
     * in sensors.h), but this isn't ideal and will give variable         *
     * results from one day to the next.                                  *
     *                                                                    *
     * You can usually find the current SLP value by looking at weather   *
     * websites or from environmental information centers near any major  *
     * airport.                                                           *
     *                                                                    *
     * For example, for Paris, France you can check the current mean      *
     * pressure and sea level at: http://bit.ly/16Au8ol                   */
     
    /* First we get the current temperature from the BMP085 */
    float temperature;
    bmp.getTemperature(&temperature);
    //Serial.print("Temperature: ");
    //Serial.print(temperature);
    //Serial.println(" C");
    return temperature;
    
  }
  else
  {
    Serial.println("Sensor error");
    return 0;
  }
}
  
  /***************************************************
 *  Nombre Funcion:        alturaporpresion
 *
 *  Returns:               float con valor de altura corregida por temepratura y presion suponiendo una atmosfera standar detectado por el sensor bmp085.
 *
 *  Parametros:            Ninguno
 *
 *  Descripcion:           El sensor debe estar conectado por i2c. Encaso de arduino Mega pines 20,21; encaso Arduino nano pines A4 y A5 y alimentado po 3.3V
 ***************************************************/
  
  float alturaporpresion(){
   sensors_event_t event;
   bmp.getEvent(&event);
    if (event.pressure)
  {
    /* Display atmospheric pressue in hPa */
    //Serial.print("Pressure:    ");
    //Serial.print(event.pressure);
    //Serial.println(" hPa");
    
    /* Calculating altitude with reasonable accuracy requires pressure    *
     * sea level pressure for your position at the moment the data is     *
     * converted, as well as the ambient temperature in degress           *
     * celcius.  If you don't have these values, a 'generic' value of     *
     * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
     * in sensors.h), but this isn't ideal and will give variable         *
     * results from one day to the next.                                  *
     *                                                                    *
     * You can usually find the current SLP value by looking at weather   *
     * websites or from environmental information centers near any major  *
     * airport.                                                           *
     *                                                                    *
     * For example, for Paris, France you can check the current mean      *
     * pressure and sea level at: http://bit.ly/16Au8ol                   */
     
    /* First we get the current temperature from the BMP085 */
    float temperature;
    bmp.getTemperature(&temperature);
    //Serial.print("Temperature: ");
    //Serial.print(temperature);
    //Serial.println(" C");

    /* Then convert the atmospheric pressure, SLP and temp to altitude    */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    //Serial.print("Altitude:    "); 
    //Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                       // event.pressure,
                                       // temperature)); 
    //Serial.println(" m");
    //Serial.println("");
    return bmp.pressureToAltitude(seaLevelPressure,event.pressure,temperature);
  }
  else
  {
    Serial.println("Sensor error");
    return 0;
  }
  }
  
  
  /***************************************************
 *  Nombre Funcion:        dthtemp
 *
 *  Returns:               float con valor de temperatura que da el sensor dht22 conectado en el pin digital 14. 
 *
 *  Parametros:            Ninguno
 *
 *  Descripcion:           El sensor debe estar conectado por one wire bus al pin D14
 ***************************************************/
  
  float dthtemp(){
    float h = dht.readHumidity();
    float t = dht.readTemperature();

  // check if returns are valid, if they are NaN (not a number) then something went wrong!
  if (isnan(t) || isnan(h)) {
    Serial.println("Failed to read from DHT");
  } else {
    //Serial.print("Humidity: "); 
    //Serial.print(h);
    //Serial.print(" %\t");
    //Serial.print("Temperature: "); 
    //Serial.print(t);
    //Serial.println(" *C");
  }
  return t;
  }
  
  
  /***************************************************
 *  Nombre Funcion:        dthhum
 *
 *  Returns:               float con valor de humedad que da el sensor dht22 conectado en el pin digital 14. 
 *
 *  Parametros:            Ninguno
 *
 *  Descripcion:           El sensor debe estar conectado por one wire bus al pin D14
 ***************************************************/
  
  float dthhum(){
    float h = dht.readHumidity();
    float t = dht.readTemperature();

  // check if returns are valid, if they are NaN (not a number) then something went wrong!
  if (isnan(t) || isnan(h)) {
    Serial.println("Failed to read from DHT");
  } else {
    //Serial.print("Humidity: "); 
    //Serial.print(h);
    //Serial.print(" %\t");
    //Serial.print("Temperature: "); 
    //Serial.print(t);
    //Serial.println(" *C");
  }
  return h;
  }
  
  /***************************************************
 *  Nombre Funcion:        setup
 *
 *  Returns:               nada. 
 *
 *  Parametros:            Ninguno
 *
 *  Descripcion:           Inicializa puerto serial 1 arduino para debug a 115.600. Inicializa puerto serial software para gps, inicializa dth22 y bmp085, muestras los datos del sensor bmp085
 
 ***************************************************/

   
   
 
  
void setup()
{
  Serial.begin(115200);
  
  pinMode(sensorgasco,INPUT);
  pinMode(sensorhumo,INPUT);
  pinMode(sensorluz,INPUT);
  pinMode(sensorruido,INPUT);
  pinMode(sensoroh,INPUT);
  pinMode(sensorpresipitacion,INPUT);
  pinMode(sensoruv,INPUT);
  pinMode(sensorcorriente,INPUT);
  pinMode(volbat,INPUT);
  pinMode(volpanel,INPUT);
  pinMode(volelectronica,INPUT);
  
  
  ss.begin(GPSBaud);
  // Initialize DHT sensor
  dht.begin();
  if(!bmp.begin()){
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
  }
  displaySensorDetails();
}
  
  
void loop(){
  
  //actualizamos el gps, si no hay informacion guarda 0;
  while (ss.available() > 0){  // vamos leyendo el buffer serial con la informacion nmea del gps
    gps.encode(ss.read());    // pasamos la informacion al la funcion que procesa la informacion para saber si hubo un dato de interes y actualizar
  }
  if (gps.location.isValid()) {
    posicion = String(gps.location.lat(),DEC) + String(gps.location.lng(),DEC);
    fecha = String(gps.date.year(),DEC) +'-'+ gps.date.month() +'-'+ gps.date.day()+'-' + gps.time.hour()+'-' + gps.time.minute();
  } else{
    posicion =String('0');
    fecha=String('0');
  }
  
    Serial.print("posicion= ");
    Serial.println(posicion);
    
    Serial.print("fecha= ");
    Serial.println(fecha);
    
    Serial.print("estimacion error= ");
    Serial.println(gps.hdop.value());
    
    Serial.print("altitud gps= ");
    Serial.println(gps.altitude.meters());
    
    Serial.print("presion atmosferica bmp085= ");
    Serial.println(presionatmosferica());
    
    Serial.print("temperatura bmp085= ");
    Serial.println(temperaturaatmosferica());
    
    Serial.print("altura x presion bmp085= ");
    Serial.println(alturaporpresion());
    
    Serial.print("temperatura DTH22= ");
    Serial.println(dthtemp());
    
    Serial.print("Humedad DTH22= ");
    Serial.println(dthhum());
     
    Serial.println("__________________"); 
    
     Serial.print("sensor gas C0= ");
     Serial.println(analogRead(sensorgasco));
     Serial.print("sensor humo= ");
     Serial.println(analogRead(sensorhumo));
     Serial.print("sensor luz= ");
     Serial.println(analogRead(sensorluz));
     Serial.print("sensor Ruido= ");
     Serial.println(analogRead(sensorruido));
     Serial.print("sensor alcohol= ");
     Serial.println(analogRead(sensoroh));
     Serial.print("sensor presipitacion= ");
     Serial.println(analogRead(sensorpresipitacion));
     Serial.print("sensor radiacion UV = ");
     Serial.println(analogRead(sensoruv));
     Serial.print("sensor corriente bateria= ");
     Serial.println(analogRead(sensorcorriente));
     Serial.print("sensor voltaje bateria = ");
     Serial.println(analogRead(volbat));
     Serial.print("sensor voltaje voltaje panel solar= ");
     Serial.println(analogRead(volpanel));
     Serial.print("sensor voltaje electroncia");
     Serial.println(analogRead(volelectronica));
      Serial.println("=========================="); 
}
