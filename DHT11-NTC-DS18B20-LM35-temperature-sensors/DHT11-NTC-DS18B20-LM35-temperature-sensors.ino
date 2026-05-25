#define LED 15
#define ONE_WIRE_BUS 22 // D4 of Xiao ESP32-C6
#include "OneWireESP32.h"
#include "DHT.h"
#define DHTPIN 23 // D5 of Xiao ESP32-C6
#define DHTTYPE DHT11   // DHT 11

DHT dht(DHTPIN, DHTTYPE);

OneWire32 ds(ONE_WIRE_BUS);
uint64_t ds18b20addr;
bool ds18b20Found = false;

unsigned long ledTimer = 0;
unsigned long sensorsTiming = 0;
unsigned long printTiming = 0;
const int elapsedPrint = 5000;
const int elapsedSensors = 2000;
const int elapsedLed = 300;
bool ledStatus= false;
float LM35 = 0.00;
float NTC = 0.00;
float DHT11read = 0;
float DS18B20 = 0;

// 3950 from here https://www.gotronic.fr/pj2-mf52type-1554.pdf
const double beta = 3950.0; 
const double r0 = 10000.0;
const double t0 = 273.0 + 25.0;
const double rx = r0 * exp(-beta/t0);
const double vcc = 3.43;
const double R = 9830.0; // measured it 05/24/2026
float smoothDHT11;
float smoothDS18B20;
float smoothNTC;
float smoothLM35;
uint8_t devices= 0;

class MovingAverage {
  private:
    int _numReadings;
    float *_readings;     
    int _readIndex = 0;
    float _total = 0.0;   

  public:
    MovingAverage(int size) {
      _numReadings = size;
      _readings = new float[_numReadings];
      for (int i = 0; i < _numReadings; i++) _readings[i] = 0.0;
    }

    ~MovingAverage() {  // free memory
      delete[] _readings;
    }

    float update(float newValue) {
      _total -= _readings[_readIndex];
      _readings[_readIndex] = newValue;
      _total += newValue;

      _readIndex++;
      if (_readIndex >= _numReadings) _readIndex = 0;

      return _total / (float)_numReadings; 
    }
};

void blinkLED(){
  if(millis() - ledTimer > elapsedLed){
    ledTimer += elapsedLed;
    if(ledStatus == false){
      ledStatus= true;
      digitalWrite(LED, HIGH);
    }else{
      ledStatus= false;
      digitalWrite(LED, LOW);
    }
  }
}
float readLM35(void){
  float total = 0;
  int samples = 20;

  for(int i = 0; i < samples; i++){
    // 1. Force a read on A1 first to intentionally exhaust any trapped 
    // cross-channel voltage coupling between A1 and A0
    analogReadMilliVolts(A1); 
    delayMicroseconds(200);

    // 2. Perform two dummy reads on A0 to let the ADC's internal 
    // sampling capacitor fully stabilize to the LM35's actual voltage
    analogReadMilliVolts(A0);
    delayMicroseconds(100);
    analogReadMilliVolts(A0);
    delayMicroseconds(100);
    
    // 3. Now collect the true, unpolluted sample
    total += analogReadMilliVolts(A0);
    delay(5); 
  }
  
  long mv = total / (float)samples;
  return mv / 10.00; // LM35 = 10mV per degree C
}
float readNTC(void){
  delayMicroseconds(50);
  float v = analogReadMilliVolts(A1) / 1000.0;
  float rt = (vcc * R) / v - R;
  float tempK = 1.0 / (
    (1.0 / t0) + (1.0 / beta) * log(rt / r0)
  );
  float tempC = tempK - 273.15;
  float ntcrawfinal = tempC;
  return ntcrawfinal;
}
float readDS18B20(){

  if(!ds18b20Found){
    devices = ds.search(&ds18b20addr, 1);
    return 0;
  }
  float temp;
  ds.request();
  delay(100);
  uint8_t err = ds.getTemp(ds18b20addr, temp);
  if(err){
    return 0;
  }else{
    if(temp > -20 && temp < 80){
      return temp;
    }else{
      return 0;
    }    
  }
  
}
float readDHT11(){
  float dht11raw =  dht.readTemperature();
  
  if (isnan(dht11raw)) {
    dht.begin();
    return 0;
  }else{
    return dht11raw;
  }
}
// do a moving average filter
MovingAverage tempDHT(5);
MovingAverage tempDS(5);
MovingAverage tempNTC(5);
MovingAverage tempLM(5);
void setup() {
  
  // put your setup code here, to run once:
  analogReadResolution(12);
  analogSetPinAttenuation(A0, ADC_11db);   // LM35
  analogSetPinAttenuation(A1, ADC_11db);  // NTC
  pinMode(LED, OUTPUT);
  dht.begin();
  Serial.begin(115200);
  devices = ds.search(&ds18b20addr, 1);
  Serial.print("Devices found: ");
  Serial.println(devices);
  if(devices > 0){
    ds18b20Found = true;
    Serial.print("DS18B20 found: 0x");
    Serial.println((unsigned long long)ds18b20addr, HEX);
  }else{
    Serial.println("No DS18B20 found");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  blinkLED(); // blink the LED for fun       
  
  if(millis() - sensorsTiming > elapsedSensors){ // read DHT11
    sensorsTiming += elapsedSensors;
    LM35= readLM35();
    NTC= readNTC();
    DS18B20 = readDS18B20(); 
    smoothLM35= tempLM.update(LM35);
    smoothNTC= tempNTC.update(NTC);
    smoothDS18B20= tempDS.update(DS18B20);

    DHT11read = readDHT11();
    smoothDHT11= tempDHT.update(DHT11read);     
  }
  if(millis() - printTiming > elapsedPrint){ // read DHT11
    printTiming += elapsedPrint;

    char buf[80];
    snprintf(buf, sizeof(buf), "LM35:%.2f,NTC:%.2f,DS18B20:%.2f,DHT11:%.2f",
     smoothLM35, smoothNTC, smoothDS18B20, smoothDHT11);
    Serial.println(buf);
  }  
}
