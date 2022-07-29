#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_SleepyDog.h>

// for Feather32u4 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

//LED is a green LED (and the feather default) to indicate it can hear it's mate
//CATCH is the LED to indicate something has been within range and no 'clear' button has been pushed
#define LED 13
#define CATCH_LED 10
#define BUTTON 12

//defines frequencies of piezo tone
#define NOTE_C6  1047
#define NOTE_CS6 1109

#define RF95_FREQ 433
#define W_DOG_TIME 11000

void catch_alarm(void);


char CHAR_LUT[10] = {30, 31, 32, 33, 34, 35, 36, 37, 38, 39}; 
int INT_LUT[10] = {0, 1 , 2, 3, 4, 5, 6, 7, 8, 9};

// Change to 434.0 or other frequency, must match RX's freq!


// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


int distance_measure = 0;

//variable to time out waiting for serial to come online



void setup()
{

   Serial.begin(115200);
  //while (!Serial) {
  //  delay(1);
  //  serialWait++;
  // }
  delay(1000);
    
  //config pins for LEDs and Buttons, resets
  pinMode(LED, OUTPUT);
  pinMode(CATCH_LED, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  pinMode(RFM95_RST, OUTPUT);
  
  digitalWrite(RFM95_RST, HIGH);

  
  
  for(int i = 0; i<5; i++){
    //catch_alarm(); 
    Serial.println("LED ON");
    analogWrite(LED, 50);
    digitalWrite(CATCH_LED, HIGH);
    
    delay(100);

    Serial.println("****OFF****");
    digitalWrite(LED, LOW);
    digitalWrite(CATCH_LED, LOW);
    delay(100);
    
  }
  //setting a watching dog for ~1 minutes
  int countdownMS = Watchdog.enable(W_DOG_TIME);
  
  //Serial.println("Feather LoRa RX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  //Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, true);
}

void loop()
{
  if (rf95.available())
  {
    
    uint8_t buf[4];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      //packet was received, so turn on the green LED and reset the watchdog timer
      Watchdog.reset();
      analogWrite(LED, 50);
      
      //RH_RF95::printBuffer("Received: ", buf, len);
      
      //Serial.print("Got: ");
      //Serial.println((char*)buf);
      distance_measure = atoi(buf);
      //Serial.print("Distance detected: ");
      //Serial.println(distance_measure);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);

      // Send a reply
      //uint8_t data[] = "And hello back to you";
      //rf95.send(data, sizeof(data));
      //rf95.waitPacketSent();
      //Serial.println("Sent a reply");
      if(distance_measure < 255){
        Watchdog.reset();
        Serial.println("HOLY SHIT A CAT");
        
        catch_alarm();
        digitalWrite(CATCH_LED, HIGH);
        while(digitalRead(BUTTON) == HIGH){
            
            Serial.println("Sexy LED pulses up");
            for(int i = 0; i <255; i++){
                Watchdog.reset();
                analogWrite(CATCH_LED, i);
                delay(2);
            }
            Serial.println("Sexy LED pulses down");
            for(int i = 255; i>0; i--){
                Watchdog.reset();
                analogWrite(CATCH_LED, i);
                delay(2);
            }
         }
         digitalWrite(CATCH_LED, LOW);
      }
      //digitalWrite(LED, LOW);
    }
    else
    {
      Serial.println("Receive failed");
      
      
    }
    
  }
}

void catch_alarm(){
  // iterate over the notes of the melody:
  int melody[] = {
  NOTE_C6, NOTE_CS6
};

  int noteDurations[] = {
  4, 4
};
  for (int thisNote = 0; thisNote < 2; thisNote++) {

    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(5, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(8);
  }
  
}
