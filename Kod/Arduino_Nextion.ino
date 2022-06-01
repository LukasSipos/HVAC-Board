#include<Wire.h> 
#include "SparkFunHTU21D.h"
#include "RTClib.h"      // pouzite kniznice pre lahsiu pracu so senzormi, modulmi a komunikaciu s nimi
#include <SPI.h>
#include <SD.h>
  File myFile;    // vytvorenie objektu myFile pre pracu s kniznicou "SD.h"
  HTU21D myHumidity; // vytvorenie objeku myHumidity pre pracu s kniznicou "SparkFunHTU21D.h"
  RTC_DS1307 rtc;  //vytvorenie objeku rtc pre pracu s kniznicou "RTClib.h"
  unsigned long cas;  // premenna pre ukladanie 
  byte pole_tlacidlo[8];  // bytove pole pouzivane na ukladanie dat pri komunikacii s displejom
  int prepnutie;  // premenna pre ulozenie hodnoty tlacidla "Spustit zaznam dat" 
  float prud_teply_peltier; // premenna pre ulozenie prudu smerujuceho do PM1
  float prud_studeny_peltier;  // premenna pre ulozenie prudu smerujuceho do PM2
  float humd;  //premenna pre ulozenie aktualnej relativnej vlhkosti nameranej specifickym senzorom
  float temp;  //premenna pre ulozenie aktualnej teploty nameranej specifickym senzorom
  int PWM_duch_vent = 2;  
  int PWM_vent_peltier_teply = 3;
  int PWM_vent_peltier_studeny = 4;  //definovanie pinov pre pwm kontrolu
  int PWM_vent_kufrik = 5;
  int mosfer_peltier_teply = 6;
  int mosfet_peltier_studeny = 7;  
  int vent_peltier_teply = 7; 
  int duch_vent = 40;
  int vent_kufrik = 7;  // definovanie pinov pre snimanie otacok ventilatorov
  int vent_peltier_studeny = 37;
  int otacky_vent;  // premenna pouzivajuca sa na ulozenie konkretnej hodnoty otacok ventilatoru
  int prudPM1 = A4;
  int prudPM2 = A5;

//###funkcia pre posuvanie I2C zbernic na 8-kanalovom multiplexeri###
void TCA9548A(uint8_t bus){ 
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();   
}
//###funkcia pre vyhodnotenie poziadavky pre zaznam dat###
void zaznam_dat(){ 
  Serial.readBytesUntil('\n', pole_tlacidlo, 8);
  prepnutie = pole_tlacidlo[0];
  if (prepnutie == 1 && pole_tlacidlo[6] == 11){
    myFile = SD.open("text.txt", FILE_WRITE);
  } else if ( prepnutie == 0 && pole_tlacidlo[6] == 11){
    myFile.close();
  }}

void setup() {
  Serial.begin(9600); // otvorenie serioveho portu, nastava rychlost prenosu dát na 9600 bps

  pinMode(PWM_duch_vent, OUTPUT);
  pinMode(PWM_vent_peltier_teply, OUTPUT);
  pinMode(PWM_vent_peltier_studeny, OUTPUT);
  pinMode(PWM_vent_kufrik, OUTPUT);
  pinMode(mosfet_peltier_studeny, OUTPUT);   // definicia pinou ako vstupne alebo vystupne
  pinMode(vent_peltier_teply, INPUT);
  pinMode(duch_vent, INPUT);
  pinMode(vent_kufrik, INPUT);
  pinMode(vent_peltier_studeny, INPUT);
  pinMode(prudPM1, INPUT);
  pinMode(prudPM2, INPUT);

  Wire.begin();  //incializuje wire kniznicu 
  myHumidity.begin();  //incializuje kniznicu "SparkFunHTU21D.h"
  rtc.begin();  //incializuje wire kniznicu "RTClib.h"
  DateTime now = rtc.now(); // nastavenie rtc modulu na aktualny cas
}

void loop() {
  
   zaznam_dat(); // funkcia pre overovanie poziadavky pre ukladanie dat na SD kartu
  
  TCA9548A(0); // prva zbernica na 8-kanalovom multiplexeri
  myFile.print("T0: "); // pri splneni poziadavky zaznamu dat sa nasledujuci riadok zapise na SD kartu
  temp=myHumidity.readTemperature();  // ziskanie teploty zo senzoru pripojeneho na prvej zbernici
  Serial.print("T0.val=");  
  Serial.print(temp,1);    // zmenenie hodnoty objektu T0 na displeji s jednou desatinou ciarkou
  myFile.print(temp,1);   // ulozenie hodnoty teploty na SD kartu
  myFile.print(" ");
  Serial.write(0xff); 
  Serial.write(0xff);  // pri komunikacii s nextion displajom, je potrebne po kazdom prikaze pouzit nasledovne prikazy
  Serial.write(0xff);

  myFile.print(" V0: ");
  humd=myHumidity.readHumidity(); / ziskanie relativnej vlhkosti zo senzoru pripojeneho na prvej zbernici
  Serial.print("V0.val=");
  Serial.print(humd,1);
  myFile.print(humd, 1);
  myFile.print("  ");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  
  TCA9548A(1);  // druha zbernica na 8-kanalovom multiplexeri
  myFile.print("T1: ");
  temp=myHumidity.readTemperature();
  Serial.print("T1.val=");
  Serial.print(temp,1);
  myFile.print(temp,1);
  myFile.print(" ");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  myFile.print(" V1: ");
  humd=myHumidity.readHumidity();
  Serial.print("V1.val=");
  Serial.print(humd,1);
  myFile.print(humd, 1);
  myFile.print("  ");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
  
  TCA9548A(2);
  myFile.print("T2: ");
  temp=myHumidity.readTemperature();
  Serial.print("T2.val=");
  Serial.print(temp,1);
  myFile.print(temp,1);
  myFile.print(" ");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  TCA9548A(3);
  myFile.print("T3: ");
  temp=myHumidity.readTemperature();
  Serial.print("T3.val=");
  Serial.print(temp,1);
  myFile.print(temp,1);
  myFile.print(" ");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  myFile.print(" V3: ");
  humd=myHumidity.readHumidity();
  Serial.print("V3.val=");
  Serial.print(humd,1);
  myFile.print(humd, 1);
  myFile.print("  ");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
  
   TCA9548A(4);
  myFile.print("T4: ");
  temp=myHumidity.readTemperature();
  Serial.print("T4.val=");
  Serial.print(temp,1);
  myFile.print(temp,1);
  myFile.print(" ");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  myFile.print(" V4: ");
  humd=myHumidity.readHumidity();
  Serial.print("V4.val=");
  Serial.print(humd,1);
  myFile.print(humd, 1);
  myFile.print("  ");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

    TCA9548A(5);
  myFile.print("T5: ");
  temp=myHumidity.readTemperature();
  Serial.print("T5.val=");
  Serial.print(temp,1);
  myFile.print(temp,1);
  myFile.print(" ");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  myFile.print(" V5: ");
  humd=myHumidity.readHumidity();
  Serial.print("V5.val=");
  Serial.print(humd,1);
  myFile.print(humd, 1);
  myFile.print("  ");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

    TCA9548A(6);
  myFile.print("T6: ");
  temp=myHumidity.readTemperature();
  Serial.print("T6.val=");
  Serial.print(temp,1);
  myFile.print(temp,1);
  myFile.print(" ");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  myFile.print(" V6: ");
  humd=myHumidity.readHumidity();
  Serial.print("V6.val=");
  Serial.print(humd,1);
  myFile.print(humd, 1);
  myFile.print("  ");
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);
  
  TCA9548A(7); 
  myFile.print(now.hour(), DEC);  // zobrazenie aktualneho casu v hodinach
  myFile.print(':');  
  myFile.print(now.minute(), DEC);  // zobrazenie aktualneho casu v minutach
  myFile.print(':');  
  myFile.print(now.second(), DEC);  // zobrazenie aktualneho casu v sekundach
  myFile.println(); 
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);   

  read_Amps(prudPM1);                               
  prud_teply_peltier = Amps_Peak_Peak*0.3536*0.06; 
  Serial.print("P1.val=");   // pomocou ACS712 odmeranie vstupujuci prud do PM1
  Serial.print(prud_teply_peltier;  // a zapisanie tejto hodnoty do objektu P1 v displeji
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  read_Amps(prudPM2);                               
  prud_studeny_peltier = Amps_Peak_Peak*0.3536*0.06;  // pomocou ACS712 odmeranie vstupujuci prud do PM2
  Serial.print("P2.val=");
  Serial.print(prud_studeny_peltier);  // a zapisanie tejto hodnoty do objektu P2 v displeji
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.readBytesUntil('\n', pole_tlacidlo, 8);  // nacitanie bytov pri stlaceni tlacidla
  if (pole_tlacidlo[6] == 5 || pole_tlacidlo[6] == 4){ // overenie identity tlacidla
    PWM = 2.55 * pole_tlacidlo[0];                    
    analogWrite(mosfer_peltier_teply, PWM);    //nastavenie vykonu PM1 podla volby na displeji
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
  }

  Serial.readBytesUntil('\n', pole_tlacidlo, 8);
  if (pole_tlacidlo[6] == 3 || pole_tlacidlo[6] == 6){
    analogWrite(mosfer_peltier_studeny, (2.55 * pole_tlacidlo[0]);
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
  }

  Serial.readBytesUntil('\n', pole_tlacidlo, 8);  
  if (pole_tlacidlo[6] == 16 || pole_tlacidlo[6] == 15){  
    analogWrite(PWM_vent_peltier_teply, (2.55 * pole_tlacidlo[0]); 
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
  }
  
  meassure(2);  // snimanie otacok ventilatora chladiaceho PM1
  Serial.print("R1.val=");  //zapisanie tejto hodnoty do objektu R1 v displeji
  Serial.print(otacky_vet);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.readBytesUntil('\n', pole_tlacidlo, 8);
  if (pole_tlacidlo[6] == 14 || pole_tlacidlo[6] == 13){
    analogWrite(PWM_vent_peltier_studeny, (2.55 * pole_tlacidlo[0]);
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
  }
  
  meassure(40);
  Serial.print("R2.val=");
  Serial.print(otacky_vet);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.readBytesUntil('\n', pole_tlacidlo, 8);
  if (pole_tlacidlo[6] == 12 || pole_tlacidlo[6] == 11){
    analogWrite(PWM_vent_duch_vent, (2.55 * pole_tlacidlo[0]);
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);  
  }
  
  meassure(7);
  Serial.print("R3.val=");
  Serial.print(otacky_vet);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

  Serial.readBytesUntil('\n', pole_tlacidlo, 8);
  if (pole_tlacidlo[6] == 10 || pole_tlacidlo[6] == 9){
    analogWrite(PWM_vent_kufrik, (2.55 * pole_tlacidlo[0]);
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
  }
  
  meassure(37);
  Serial.print("R4.val=");
  Serial.print(otacky_vet);
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(0xff);

 delay(800);  // pozastavenie programu na 800 milisekund
}

 void meassure(int x) { // funkcia pre zistenie otáčok ventilátorov // prevzaté od: https://forum.arduino.cc/t/reading-fan-rpm-with-pwm/368626
  InterruptCounter = 0;
  attachInterrupt(digitalPinToInterrupt(x), countup, RISING);
  delay(1000);
  detachInterrupt(digitalPinToInterrupt(x));
  rpm = (InterruptCounter / 2) * 60;
  if (rpm<0){
    rpm=rpm*(-1);
  }
  myFile.print(" RPM");
  myFile.print(x);
  myFile.print(" ");
  myFile.println(rpm/10);
}

void countup() {
  InterruptCounter++;
}

void read_Amps(int x)            //funkcia citania spravnej hodnoty amperov vstupujuce do Peltierových modulov
{                           //prevzaté od:  https://create.arduino.cc/projecthub/SurtrTech/measure-any-ac-current-with-acs712-70aa85
  int cnt;          
  High_peak = 0;      
  Low_peak = 1024;
  
      for(cnt=0 ; cnt<300 ; cnt++)        
      {
        float ACS_Value = analogRead(x); 

        
        if(ACS_Value > High_peak)                
            {
              High_peak = ACS_Value;            
            }
        
        if(ACS_Value < Low_peak)                
            {
              Low_peak = ACS_Value;                      
              }
      }                                        
      
  Amps_Peak_Peak = High_peak - Low_peak;      
}
