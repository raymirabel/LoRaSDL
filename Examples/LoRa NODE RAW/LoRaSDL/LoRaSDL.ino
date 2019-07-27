/**************************************************************************
 @FILE:         LoRaSDL.ino
 @AUTHOR:       Raimundo Alfonso
 @COMPANY:      Ray Ingeniería Electronica, S.L.
 @DESCRIPTION:  Ejemplo de uso para el nodo LoRaSDL y emoncms (https://emoncms.org)
                Example of use for the LoRaSDL node and emoncms (https://emoncms.org)
  
 @LICENCE DETAILS:
  Este sketch está basada en software libre. Tu puedes redistribuir
  y/o modificar esta sketch bajo los términos de licencia GNU.

  Esta programa se distribuye con la esperanza de que sea útil,
  pero SIN NINGUNA GARANTÍA, incluso sin la garantía implícita de
  COMERCIALIZACIÓN O PARA UN PROPÓSITO PARTICULAR.
  Consulte los términos de licencia GNU para más detalles:
                                                                       
  http://www.gnu.org/licenses/gpl-3.0.txt

  This sketch is based on free software. You can redistribute
  and/or modify this library under the terms of the GNU license.

  This software is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY, even without the implied warranty of
  MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU license terms for more details:   
  
  http://www.gnu.org/licenses/gpl-3.0.txt

 @VERSIONS:
  17-07-2019 - v1.00 : Primera versión
  
**************************************************************************/

#define  FIRMWARE_VERSION "1.00"
#define  HARDWARE_VERSION "190507"

#include <SPI.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <EEPROM.h>
#include <RHReliableDatagram.h>     // https://www.airspayce.com/mikem/arduino/RadioHead/index.html
#include <RH_RF95.h>                // https://www.airspayce.com/mikem/arduino/RadioHead/index.html
#include <LowPower.h>               // https://github.com/rocketscream/Low-Power
#include <SHT2x.h>                  // https://github.com/raymirabel/SHT2x

// Defines what sensors are available in the hardware...
//#define SHT21
#define LIDAR

// Structure of default configuration parameters. This parameters are stored in internal eeprom...
typedef struct {
  int     sendTime        = 30;     // [1...9999] Send time delay (minutes)
  byte    rfPower         = 14;     // [5...23]   RF power (5:min...23:max)
  byte    rfRetries       = 3;      // [0...20]   Data send retries (0 = no retries)
  byte    rfNode          = 0;      // [0...250]  Node id
  byte    rfPan           = 100;    // [0...250]  PAN id (Only nodes with this same number are visible)
  boolean dipsw           = true;   // [on|off]   rfNode dipswitch mode:
                                    //            - true:  rfNode = dipswitch + rfPan (the rfNode parameter is ignored)
                                    //            - false: rfNode = rfNode + rfPan 
  boolean led             = true;   // [on|off]   led test mode:
                                    //            - true:  led test mode turn on when the node reads sensors and transmits payload.
                                    //            - false: led test is always off
  byte    lid_filter      = 3;      // [0...9]    Lidar filter (0:min...10:max)
  byte    lid_resolution  = 1;      // [1...100]  Lidar resolution (cm)
  byte    lid_repeat      = 3;      // [1...10]   Lidar measurement repeats
} stConfig;
stConfig config;

// Identify the node
#define DEVICE_ID      5      // 5 = LoRaSDL
//#define DEVICE_ID      6      // 6 = LoRaTH
//#define DEVICE_ID      7      // 7 = LoRaX1
//#define DEVICE_ID      8      // 8 = LoRaHALL

// Hardware definitions...
#define SERVER_ADDRESS 250
#define RX        0
#define TX        1
#define RFM95_INT 2
#define DS18B20   3
#define RFM95_CS  4
#define GPIO1     5
#define RX_LIDAR  6
#define TX_LIDAR  7
#define EN_SENS   8
#define LED       9
#define EN_LIDAR  10
#define DIPSW0    A0
#define DIPSW1    A1
#define DIPSW2    A2
#define DIPSW3    A3
#define VBAT      A6
#define LDR       A7
#define DIR_EEPROM_CFG  10


// Payload structure...  
typedef struct {
  byte        pan_id;
  byte        device_id = DEVICE_ID;
  int         status;         // bit 0:  temperature & humidity SHT21 sensor OK or present
                              // bit 1:  lidar sensor OK o present    
                              // ...
                              // bit 15: device timeout
  int         rssi;           // x1   - dB
  int         distance;       // x1   - cm
  int         strength;       // 
  int         temperature;    // x10  - ºC
  int         humidity;       // x1   - %
  int         light;          // x1   - %
  int         battery;        // x100 - V
} Payload;
Payload theData;

// RFM95 transceiver configuration and instances...
#define RF95_FREQ 868.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, 0);


// Software serial port for MiniLidar
SoftwareSerial lidar(RX_LIDAR, TX_LIDAR); // RX, TX

// SHT21...
SHT2xClass SHT2x;


/**************************************************************************
 * SETUP
 *************************************************************************/  
void setup(){
  pinMode(EN_SENS,  OUTPUT);
  pinMode(EN_LIDAR, OUTPUT);
  pinMode(DS18B20,  INPUT);
  pinMode(LED, OUTPUT);
  pinMode(DIPSW0,INPUT);
  pinMode(DIPSW1,INPUT);  
  pinMode(DIPSW2,INPUT);  
  pinMode(DIPSW3,INPUT);  
  
  sens_off();
  lidar_off();
  led_test_off();

  // Check if the eeprom is empty...
  if(EEPROM.read(0) == 123){
    // Read the configuration parameters...
    EEPROM.get(DIR_EEPROM_CFG, config);
  }else{
    // If it is empty, it saves the configuration parameters by default...
    EEPROM.write(0,123);
    EEPROM.put(DIR_EEPROM_CFG, config);
  }

  // Init serial port...
  while (!Serial);
  Serial.begin(9600);

  // Init I2C bus...
  Wire.begin();
  // Desactivate internal pullups for I2C.
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);  

  // Init software serial port...
  lidar.begin(115200);
  // RX_LIDAR input at low for lower consumption...
  digitalWrite(RX_LIDAR, LOW);    
  digitalWrite(TX_LIDAR, LOW);    

  // Init payload struct...
  theData.pan_id   = config.rfPan;
  theData.distance = 0;
  theData.strength = 0;
  theData.temperature = 0;
  theData.humidity = 0;
  theData.light = 0;
  theData.status = 0; 

  // Init RFM95 module...
  radioInit();
  rf95.sleep();
  delay(10);  
 
  // Check the RX pin to see if you have to enter command line mode...
  if(digitalRead(RX)){
    led_commandLine();
    commandLine();
  }
  led_init();

  // Sleep 8 seconds before transmitting the first frame after power-up...
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
}

/**************************************************************************
 * LOOP
 *************************************************************************/ 
void loop(){
  // Turn on test led. This led shows us how long the node is awake...
  led_test_on();

  // Sensors read...
  lee_sensores();
      
  // Send payload to server...
  send_to_server();

  // Turn of test led...
  led_test_off();
  

  // Sleep x minutes...
  //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
  duerme(config.sendTime);
}


/**************************************************************************
 * FUNCTIONS
 *************************************************************************/ 

void lee_sensores(void){
  // Read battery level...
  lee_bateria();

  // Read light level...
  lee_ldr();

  // Read lidar sensor...
#ifdef LIDAR  
  lidar_on();
  readLidar();
  lidar_off();
  delay(10);
  while(lidar.available()) lidar.read();
  lidar.flush();  
#else
  theData.status &= ~0x02;
  delay(10);
#endif  

  // Read temperature and humidity...
#ifdef SHT21  
  leeSHT21();
#else
  theData.status &= ~0x01;
#endif  
}

boolean readLidar(void){
  boolean ok = false;
  int check;              //save check value
  int i;
  int uart[9];            //save data measured by LiDAR
  const int HEADER=0x59;  //frame header of data package
  unsigned long timeout = millis();
  int d[10];
  int s[10];
  int p = 0;

  do{
    if (lidar.available()){       //check if serial port has data input
      if(lidar.read()==HEADER){   //assess data package frame header 0x59
        uart[0]=HEADER;
        if(lidar.read()==HEADER){ //assess data package frame header 0x59
          uart[1]=HEADER;
          for(i=2;i<9;i++){       //save data in array
            uart[i]=lidar.read();
          }
          check=uart[0]+uart[1]+uart[2]+uart[3]+uart[4]+uart[5]+uart[6]+uart[7];
          if(uart[8]==(check&0xff)){      //verify the received data as per protocol
            d[p]=uart[2]+uart[3]*256;     //calculate distance value
            s[p]=uart[4]+uart[5]*256;     //calculate signal strength value
#ifdef DEBUG
            Serial.print("dist = ");
            Serial.print(d[p]);           //output measure distance value of LiDAR
            Serial.print('\t');
            Serial.print("strength = ");
            Serial.print(s[p]);           //output signal strength value
            Serial.print('\n');
#endif
            p++;
            if(p >= config.lid_repeat) ok = true;
          }
        }
      }
    }
    while(lidar.available()) lidar.read();
    lidar.flush();  
  }while(((millis() - timeout) < 3000) && (ok == false));
//  Serial.println(millis() - timeout);

  if(ok){
    theData.strength = 0;
    // Escoge el que tiene mayor señal...
    for(p=0;p<config.lid_repeat;p++){
      if(s[p] > theData.strength){
        theData.strength = s[p];
        theData.distance = d[p];
      }
#ifdef DEBUG   
      Serial.print(s[p]);
      Serial.print("\t");
      Serial.println(d[p]);
#endif    
    }
    // Filtra...
    theData.distance = filterIIR(theData.distance,config.lid_filter);
    
    // Ajusta resolucion...
    theData.distance = theData.distance / config.lid_resolution;
    theData.distance = theData.distance * config.lid_resolution;
    theData.status |= 0x02;
    return(true);
  }else{
    theData.distance = 1;
    theData.strength = 0;    
    theData.status &= ~0x02;
    return(false);
  }
}


void lidar_on(void){
  digitalWrite(RX_LIDAR, HIGH);   
  digitalWrite(TX_LIDAR, HIGH);     
  digitalWrite(EN_LIDAR, HIGH);
  delay(1);
}
void lidar_off(void){
  digitalWrite(EN_LIDAR, LOW);
  // Entrada RX_LIDAR a bajo para menor consumo...
  digitalWrite(RX_LIDAR, LOW);    
  digitalWrite(TX_LIDAR, LOW);    
}
void sens_on(void){
  digitalWrite(EN_SENS, HIGH);
  delay(10);
}
void sens_off(void){
  digitalWrite(EN_SENS, LOW);
}

void leeSHT21(void){
  unsigned long timeOutSensor = 0; 
  float t;
  float h;
  boolean temperature_ok = false;
  boolean humidity_ok = false;

  sens_on();
  SHT2x.PrepareTemperatureNoHold();
  timeOutSensor = millis();  
  do{ 
    if(SHT2x.GetTemperatureNoHold(&t)){
      temperature_ok = true;     
    }
  }while(temperature_ok == false && (millis() - timeOutSensor) < 100L);
  
  SHT2x.PrepareHumidityNoHold();
  timeOutSensor = millis();  
  do{ 
    if(SHT2x.GetHumidityNoHold(&h)){
      humidity_ok = true;   
    }
  }while(humidity_ok == false && (millis() - timeOutSensor) < 100L);

  if(temperature_ok && humidity_ok){
    theData.temperature = (int)(t * 10);
    theData.humidity = (int)h;  
    theData.status |= 0x01; 
  }else{
    theData.temperature = 0;
    theData.humidity = 0;      
    theData.status &= ~0x01;
  }
  sens_off();
}

void lee_ldr(void){
  sens_on();
  delay(1);
  theData.light = 100 - (unsigned int)(((unsigned long)(analogRead(LDR)) * 100L) / 1023L);
  sens_off();
}

void lee_bateria(void){
  unsigned long valores = 0;
  byte n;

  delay(1);
  // Lee 5 muestras...
  for(n=0;n<5;n++){
     valores += (unsigned int)(((unsigned long)(analogRead(VBAT)) * 330L) / 1023L);
     delay(1);
  }
  theData.battery = valores / 5;
}



