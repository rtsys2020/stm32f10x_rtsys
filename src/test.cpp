
//RCSwitch mySwitch = RCSwitch();
//#define RC_SWITCH_PIN PB9

#include "RCSwitch.h"

//#define CC11XX

#if defined(CC11XX)
#include <cc1100_arduino.h>
//init CC1100 constructor
CC1100 cc1100;

uint32_t rf_timecode = 0;
uint32_t rf_timecode_backup = 0;
//--------------------------[Global Task variables]--------------------------

//--------------------------[Global CC1100 variables]------------------------
uint8_t Tx_fifo[FIFOBUFFER], Rx_fifo[FIFOBUFFER];
uint8_t My_addr, Tx_addr, Rx_addr, Pktlen, pktlen, Lqi, Rssi;
uint8_t rx_addr,sender,lqi;
 int8_t rssi_dbm;
volatile uint8_t cc1101_packet_available;

void rf_available_int(void) ;
#endif

//Serial for GPS
//Serial1 for bluethooth and rs485
//Serial2 for GPRS


//#include <HMC5883L/HMC5883L.h>
//#include <Wire.h>
//
//
//#include <HMC5983/HMC5983.h>
//#include <Wire.h>
//HMC5883L compass;

//#include <HardWire.h>
//HardWire HWire(1, I2C_FAST_MODE); // I2c1
#include <PN532_SPI/PN532_SPI.h>
#include "PN532/PN532.h"

//#define _GSM_

#ifdef _GSM_
#include <GSM.h>

#define PINNUMBER ""
// initialize the library instance
GSM gsmAccess(true);     // include a 'true' parameter for debug enabled
GSMScanner scannerNetworks;
GSMModem modemTest;

// Save data variables
String IMEI = "";

// serial monitor result messages
String errortext = "ERROR";
#endif


//PN532_SPI pn532spi(SPIClass2, PB12);
//PN532 nfc(pn532spi);

//static void vLEDFlashTask(void *pvParameters) {
//    for (;;) {
//        vTaskDelay(1000);
//        digitalWrite(0, HIGH);
//        vTaskDelay(50);
//        digitalWrite(0, LOW);
//    }
//}

#ifdef _GSM_

void setup()
{
  // initialize serial communications
  Serial.begin(9600);
  Serial.println("GSM networks scanner");
  scannerNetworks.begin();

  // connection state
  boolean notConnected = true;

  // Start GSM shield
  // If your SIM has PIN, pass it as a parameter of begin() in quotes
  while(notConnected)
  {
    if(gsmAccess.begin(PINNUMBER)==GSM_READY)
      notConnected = false;
    else
    {
      Serial.println("Not connected");
      delay(1000);
    }
  }

  // get modem parameters
  // IMEI, modem unique identifier
  Serial.print("Modem IMEI: ");
  IMEI = modemTest.getIMEI();
  IMEI.replace("\n","");
  if(IMEI != NULL)
    Serial.println(IMEI);

  // currently connected carrier
  Serial.print("Current carrier: ");
  Serial.println(scannerNetworks.getCurrentCarrier());

  // returns strength and ber
  // signal strength in 0-31 scale. 31 means power > 51dBm
  // BER is the Bit Error Rate. 0-7 scale. 99=not detectable
  Serial.print("Signal Strength: ");
  Serial.print(scannerNetworks.getSignalStrength());
  Serial.println(" [0-31]");
}

void loop()
{
  // scan for existing networks, displays a list of networks
  Serial.println("Scanning available networks. May take some seconds.");

  Serial.println(scannerNetworks.readNetworks());

    // currently connected carrier
  Serial.print("Current carrier: ");
  Serial.println(scannerNetworks.getCurrentCarrier());

  // returns strength and ber
  // signal strength in 0-31 scale. 31 means power > 51dBm
  // BER is the Bit Error Rate. 0-7 scale. 99=not detectable
  Serial.print("Signal Strength: ");
  Serial.print(scannerNetworks.getSignalStrength());
  Serial.println(" [0-31]");

}


GSM_SMS sms;

// char array of the telephone number to send SMS
// change the number 1-212-555-1212 to a number
// you have access to
char remoteNumber[20]= "12125551212";

// char array of the message
char txtMsg[200]="Test";
void sendSMS(){

  Serial.print("Message to mobile number: ");
  Serial.println(remoteNumber);

  // sms text
  Serial.println("SENDING");
  Serial.println();
  Serial.println("Message:");
  Serial.println(txtMsg);

  // send the message
  sms.beginSMS(remoteNumber);
  sms.print(txtMsg);
  sms.endSMS();
  Serial.println("\nCOMPLETE!\n");
}


// APN information obrained from your network provider
#define GPRS_APN       "GPRS_APN" // replace with your GPRS APN
#define GPRS_LOGIN     "login"    // replace with your GPRS login
#define GPRS_PASSWORD  "password" // replace with your GPRS password

char server[] = "arduino.cc"; // the base URL
char path[] = "/latest.txt"; // the path
int port2 = 80; // the port, 80 for HTTP

// initialize the library instances
GSMClient client;
GPRS gprs;
void setup2()
{
  // initialize serial communications
  Serial.begin(9600);
  Serial.println("Starting Arduino web client.");
  // connection state
  boolean notConnected = true;

  // Start GSM shield
  // pass the PIN of your SIM as a parameter of gsmAccess.begin()
  while(notConnected)
  {
    if((gsmAccess.begin(PINNUMBER)==GSM_READY) &
        (gprs.attachGPRS(GPRS_APN, GPRS_LOGIN, GPRS_PASSWORD)==GPRS_READY))
      notConnected = false;
    else
    {
      Serial.println("Not connected");
      delay(1000);
    }
  }

  Serial.println("connecting...");

  // if you get a connection, report back via serial:
  if (client.connect(server, port2))
  {
    Serial.println("connected");
    // Make a HTTP request:
    client.print("GET ");
    client.print(path);
    client.println(" HTTP/1.0");
    client.println();
  }
  else
  {
    // if you didn't get a connection to the server:
    Serial.println("connection failed");
  }
}


void loop2()
{
  // if there are incoming bytes available
  // from the server, read them and print them:
  if (client.available())
  {
    char c = client.read();
    Serial.print(c);
  }

  // if the server's disconnected, stop the client:
  if (!client.available() && !client.connected())
  {
    Serial.println();
    Serial.println("disconnecting.");
    client.stop();

    // do nothing forevermore:
    for(;;)
      ;
  }
}



#else


#if defined(RC_SWITCH)

void init_rcSwitch()
{
  mySwitch.enableReceive(RC_SWITCH_PIN);  // Receiver on interrupt 0 => that is pin #2
}

void rcSwitch_Task() {
  if (mySwitch.available()) {
    mySwitch.getReceivedValue();
    mySwitch.getReceivedBitlength();
    mySwitch.getReceivedDelay();
    mySwitch.getReceivedRawdata();
    mySwitch.getReceivedProtocol();
    mySwitch.resetAvailable();
  }
}
#endif

#if defined(CC11XX)

void CC11XX_Setup()
{
  // init serial Port for debugging


  // init CC1101 RF-module and get My_address from EEPROM
  cc1100.begin(SPIClass2,PB9,My_addr);                   //inits RF module with main default settings

  cc1100.silde();                          //set to ILDE first
  cc1100.set_mode(0x04);                   //set modulation array mode
  cc1100.set_ISM(0x01);                    //set frequency
  cc1100.set_channel(0x01);                //set channel
  cc1100.set_output_power_level(0);        //set PA level in dbm
  cc1100.set_myaddr(0x03);                 //set my own address

  cc1100.spi_write_register(IOCFG2, 0x06); //set module in sync mode detection mode

  cc1100.show_main_settings();             //shows setting debug messages to UART
  cc1100.show_register_settings();         //shows current CC1101 register values
  cc1100.receive();                        //set to RECEIVE mode

  // init interrrupt function for available packet
  attachInterrupt(GDO2, rf_available_int, RISING);


}

void cc11xx_Task()
{
  //if valid package is received
  if(cc1101_packet_available == TRUE){
    rf_timecode = ((uint32_t)Rx_fifo[3] << 24) +
                  ((uint32_t)Rx_fifo[4] << 16) +
                  ((uint16_t)Rx_fifo[5] <<  8) +
                             Rx_fifo[6];
    Serial.print(F("TX_Timecode: "));Serial.print(rf_timecode);Serial.println(F("ms"));

    rf_timecode_backup = millis();
    cc1101_packet_available = FALSE;
  }

}

void rf_available_int(void)
{
  detachInterrupt(GDO2);
//  sei();
  uint32_t time_stamp = millis();                                   //generate time stamp

  if(cc1100.packet_available() == TRUE){
    cc1100.get_payload(Rx_fifo, pktlen, rx_addr, sender, rssi_dbm, lqi); //stores the payload data to Rx_fifo
    cc1101_packet_available = TRUE;                                      //set flag that an package is in RX buffer
  }
  Serial.print(F("rx_time: "));Serial.print(millis()-time_stamp);Serial.println(F("ms"));

  attachInterrupt(GDO2, rf_available_int, RISING);
}

#endif

#endif

#if 0


void setup(void) {
  Serial1.begin(115200);
  Serial1.println("Hello!");

//  nfc.begin();
//
//  uint32_t versiondata = nfc.getFirmwareVersion();
//  if (! versiondata) {
//    Serial1.print("Didn't find PN53x board");
//    while (1); // halt
//  }
//
//  // Got ok data, print it out!
//  Serial1.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);
//  Serial1.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);
//  Serial1.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
//
//  // Set the max number of retry attempts to read from a card
//  // This prevents us from waiting forever for a card, which is
//  // the default behaviour of the PN532.
//  nfc.setPassiveActivationRetries(0xFF);
//
//  // configure board to read RFID tags
//  nfc.SAMConfig();

  Serial1.println("Waiting for an ISO14443A card");
}

void loop(void) {
  boolean success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  int i=0;
//   Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
//   'uid' will be populated with the UID, and uidLength will indicate
//   if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
//  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength);
//
//  if (success) {
//    Serial1.println("Found a card!");
//    Serial1.print("UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
//    Serial1.print("UID Value: ");
//    for (uint8_t i=0; i < uidLength; i++)
//    {
//      Serial1.print(" 0x");Serial.print(uid[i], HEX);
//    }
//    Serial1.println("");
//
//    // wait until the card is taken away
//    while (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength)) {}
//  }
//  else
//  {
//    // PN532 probably timed out waiting for a card
//    Serial1.println("Timed out waiting for a card");
//  }
  i++;
  delay(100);
}

#endif
