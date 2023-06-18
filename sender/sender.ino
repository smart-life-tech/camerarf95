#include <SoftwareSerial.h>
#include <Adafruit_VC0706.h>
#include <SPI.h>
#include <SD.h>
#include <RH_RF95.h>

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2
#include <avr/wdt.h>
#define NAME "BBA"


RH_RF95 rf95(RFM95_CS, RFM95_INT);

#define RF95_FREQ 434.23
#define TXSIZE 82

SoftwareSerial camera = SoftwareSerial(6, 7);
Adafruit_VC0706 cam = Adafruit_VC0706(&camera);

#define chipSelect 4

struct PACKET
{
  // packet counter
  uint32_t counter;
  // number of bytes in packet
  uint16_t numBytes;
  // actual data
  byte data[TXSIZE];
  // crc
  uint32_t crc;
};

File photoFile;

const uint8_t SIZE = sizeof(NAME) - 1;
char fileName[] = NAME "000.jpg";

long packet_count = 0;
int rxerr = 0;

void setup()
{
  watchdogSetup();
  wdt_reset();
  Serial.begin(57600);

//  SPI.setClockDivider(SPI_CLOCK_DIV8);

  (cam.begin());

  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);

//  digitalWrite(RFM95_RST, LOW);
//  delay(50);
//  digitalWrite(RFM95_RST, HIGH);
//  delay(50);

  //delay(1500);

  cam.setImageSize(VC0706_640x480); // 160x120 320x240 640x480
  cam.setMotionDetect(false);

  // delay(1500);

  while (!rf95.init()) {
    Serial.println(F("LoRa failed"));
    while (1);
  }
  //Serial.println(F("LoRa OK!"));
  wdt_reset();
  if (!rf95.setFrequency(RF95_FREQ))
  {
    Serial.println(F("Frequency failed"));
    while (1);
  }

  //rf95.setModemConfig(RH_RF95::Bw125Cr45Sf2048);
  //Bw500Cr45Sf128, Bw31_25Cr48Sf512, Bw125Cr48Sf4096
  //Bw125Cr45Sf128


  //rf95.setTxPower(16, false);

  if (!SD.begin(chipSelect)) { //SD card present?
    Serial.println(F("Card failed"));
    return;
  }
  wdt_reset();
  delay(1000);

  while (SD.exists(fileName)) {
    if (fileName[SIZE + 2] != '9') {
      fileName[SIZE + 2]++;
      wdt_reset();
    } else if (fileName[SIZE + 1] != '9') {
      fileName[SIZE + 2] = '0';
      fileName[SIZE + 1]++;
      wdt_reset();
    } else if (fileName[SIZE] != '9') {
      fileName[SIZE + 1] = '0';
      fileName[SIZE]++;
      wdt_reset();
    } else {
      Serial.println(F("Can't create file name"));
      return;
    }
  }

  wdt_reset();
}

void watchdogSetup()
{

  //wdt_enable(WDT_PERIOD_4KCLK_gc);

  cli();

  wdt_reset();

  WDTCSR |= (1 << WDCE) | (1 << WDE);

  WDTCSR = (1 << WDIE) | (1 << WDE) | (1 << WDP3) | (0 << WDP2) | (0 << WDP1) | (0 << WDP0);
  sei();
}

void printPacket(PACKET *p)
{
  //  Serial.println(F("***TX PKT***"));
  //  Serial.print(F("Counter =  "));
  //  Serial.println(p->counter);
  //  Serial.print(F("numBytes = "));
  //  Serial.println(p->numBytes);
  //  Serial.print(F("crc =      "));
  //  Serial.println(p->crc, HEX);
  //  Serial.print("RX Errors = ");
  //  Serial.println(rxerr);
  //  Serial.println();
}

uint32_t calcCrc(byte *data, uint32_t len) {

  const uint32_t crc_table[16] =
  {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  uint32_t crc = ~0L;

  for (uint32_t cnt = 0 ; cnt < len  ; ++cnt)
  {
    crc = crc_table[(crc ^ data[cnt]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (data[cnt] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;

  }
  return crc;
}

void loop()
{
  wdt_reset();

  (cam.takePicture());
  //  Serial.println(F("Pic taken!"));
  photoFile = SD.open(fileName, FILE_WRITE);
  uint16_t jpglen = cam.frameLength();
  int32_t time = millis();
  while (jpglen > 0) {
    uint8_t *buffer;
    uint8_t bytesToRead = min(64, jpglen);
    buffer = cam.readPicture(bytesToRead);
    wdt_reset();
    photoFile.write(buffer, bytesToRead);
    jpglen -= bytesToRead;
  }

  photoFile.close();

  //Serial.println(F("Saved!!"));
  // Serial.println();

  delay(50);

  photoFile = SD.open(fileName);

  delay(100);

  if (photoFile)
  {
    PACKET txPacket;

    while (photoFile.available())
    {
      txPacket.numBytes = photoFile.read(txPacket.data, sizeof(txPacket.data));

      if (txPacket.numBytes < 0)
      {
        break;
      }

      txPacket.counter = packet_count;

      txPacket.crc = calcCrc((byte*)&txPacket, sizeof(txPacket) - sizeof(txPacket.crc));

      printPacket(&txPacket);

      //uint8_t buf[sizeof(PACKET)];
      //uint8_t len = sizeof(buf);

      uint8_t ok = 0x00;
      uint8_t ack = 2;

      if (rxerr >= 12 ) {
        delay(8000);
      }

      wdt_reset();

      rf95.send((byte*)&txPacket, sizeof(txPacket));

      rf95.waitPacketSent();

      (rf95.waitAvailableTimeout(1000));

      if (rf95.recv(ok, ack)) {
        digitalWrite(8, HIGH);

        //        Serial.println(F("*RX ACK*"));
        //        Serial.print(F("**ERROR = "));
        //        Serial.print(rxerr);
        //        Serial.println(F("**"));
        //        Serial.println();
        packet_count++;
        memset(&txPacket, 0, sizeof(txPacket));
        //delay(10);
        if (rxerr >= 12 ) {

          delay(6000);
        }
        digitalWrite(8, LOW);
      }
      else
      {
        rxerr ++;
        // Serial.println();
        // Serial.println(F("*RX ERROR*"));
        //delay(500);

        rf95.send((byte*)&txPacket, sizeof(txPacket));

        //  rf95.waitPacketSent();

        (rf95.waitAvailableTimeout(1000));

        if (rf95.recv(ok, ack)) {
          //rxerr ++;
          packet_count++;
          digitalWrite(8, HIGH);

          //          Serial.print(F("**PACKET ERROR = "));
          //          Serial.print(rxerr);
          //          Serial.println(F("**"));
          //          Serial.println();
          memset(&txPacket, 0, sizeof(txPacket));
          if (rxerr >= 12 ) {
            delay(6000);
          }
          digitalWrite(8, LOW);
        }
      }
    }
    photoFile.close();
    //   Serial.println(F("*COMPLETE*"));
    //   Serial.println();
    //delay(000);
    wdt_reset();
    //   Serial.println();
    delay(8000);
    return;
  }
  else
  {
    // if the file didn't open, print an error:
    // Serial.println(F("***FILE ERROR***"));
    delay(6000);
  }
}
