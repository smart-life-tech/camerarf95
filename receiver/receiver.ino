#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RH_RF95.h>
#include <avr/wdt.h>
#define RFM95_CS 8
#define RFM95_RST 9
#define RFM95_INT 2
#define FILE_BASE_NAME "TTS"
#define RF95_FREQ 434.23
#define TXBUFFERSIZE 82
#define chipSelect 10

RH_RF95 rf95(RFM95_CS, RFM95_INT);

const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[] = FILE_BASE_NAME "000.jpg";

uint8_t pckterr = 0;
uint32_t counter = 0;

uint8_t check[8] = {0x0, 0x1 , 0x3, 0x16, 0x1c, 0x8, 0x0};
uint8_t cross[8] = {0x0, 0x1b, 0xe, 0x4, 0xe, 0x1b, 0x0};

LiquidCrystal_I2C lcd(0x27, 20, 4);
File my_file;

struct PACKET
{
  uint32_t counter;
  uint16_t numBytes;
  byte data[TXBUFFERSIZE];
  uint32_t crc;
};

void setup()
{
  //watchdogSetup();
  //wdt_reset();
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A3, OUTPUT);

  digitalWrite(6, HIGH);
  pinMode(6, OUTPUT);

  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A3, LOW);

  Serial.begin(115200);

  //SPI.setClockDivider(SPI_CLOCK_DIV8);

  pinMode(RFM95_RST, OUTPUT);

  digitalWrite(RFM95_RST, LOW);
  delay(50);
  digitalWrite(RFM95_RST, HIGH);
  delay(50);

  lcd.begin();
  lcd.createChar(5, check);
  lcd.createChar(6, cross);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(F("CornOnTheBranch..."));
  lcd.setCursor(9, 3);
  lcd.print(F("...Shizzle!"));
  lcd.setCursor(0, 1);

  //rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);
  //Bw500Cr45Sf128, Bw31_25Cr48Sf512, Bw125Cr48Sf4096
  //Bw125Cr45Sf128
  //rf95.setTxPower(10, false);

  while (!rf95.init()) {

    Serial.println(F("LoRa failed"));
    while (1);
  }
  lcd.print(F("LoRa OK! "));
  lcd.write(5);
  lcd.print(F("    "));
  lcd.print(RF95_FREQ);

  //wdt_reset();
  if (!rf95.setFrequency(RF95_FREQ))
  {
    Serial.println(F("Frequency failed"));
    while (1);
  }

  delay(500);
  if (!SD.begin(chipSelect)) {

    Serial.println(F("Card failed"));
  } else {
    lcd.setCursor(0, 2);
    lcd.print(F("SD OK! "));
    lcd.write(5);
  }

  //wdt_reset();

  while (SD.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 2] != '9') {
      fileName[BASE_NAME_SIZE + 2]++;
      wdt_reset();
    } else if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 2] = '0';
      fileName[BASE_NAME_SIZE + 1]++;
      wdt_reset();
    } else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;

      //     wdt_reset();

    } else {
      //Serial.println(F("Can't create file name"));
      return;
    }
  }

  lcd.setCursor(10, 2);
  lcd.print(fileName);
  //wdt_reset();

}
//void watchdogSetup()
//{
//  cli();
//  wdt_reset();
//
//  WDTCSR |= (1 << WDCE) | (1 << WDE);
//
//  WDTCSR = (1 << WDIE) | (1 << WDE) | (1 << WDP3) | (0 << WDP2) | (0 << WDP1) | (0 << WDP0);
//  sei();
//}

uint8_t buf[sizeof(PACKET)];
uint8_t len = sizeof(buf);

bool firstPacket = true;
uint32_t packetNumber = 0;

int packeterror = 0;

uint8_t ok = 0x00;
uint8_t ack = 2;

void loop()
{

  if (rf95.recv(buf, &len))
  {
    if (counter <= 2) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("**PACKET RECEIVED**"));
      lcd.setCursor(0, 3);
      lcd.print(F("SNR = "));
          lcd.setCursor(13, 3);
    lcd.print(F("ERR = "));

    }

    lcd.setCursor(6, 3);
    int snr = RH_RF95_REG_19_PKT_SNR_VALUE;

    lcd.print(snr);
    lcd.print(F("dB"));

    lcd.setCursor(13, 3);
    lcd.print(F("ERR = "));
    lcd.print(pckterr);
    digitalWrite(A0, HIGH);

    //wdt_reset();

    //Serial.println();
    uint32_t crc = calcCrc(buf, sizeof(buf) - sizeof(crc));

    printPacket((PACKET*)buf);

    PACKET *rxPacket = (PACKET*)buf;

    if (rxPacket->counter != packetNumber)
    {
      digitalWrite(A3, HIGH);


      //lcd.setCursor(0, 3);
      //lcd.print(F("*PCKT ERROR = "));
      //lcd.print(F("Expected: "));
      //lcd.print(packetNumber);
      //lcd.setCursor(0, 2);
      //lcd.print(F("Specified: "));
      //lcd.print(rxPacket->counter);
      //Serial.println();

      pckterr ++;

      //Serial.println("*");

      packetNumber = rxPacket->counter;

      if (pckterr >= 11) {
        digitalWrite(6, LOW);
      }
    }
    if (crc != rxPacket->crc)
    {
      //      Serial.println(F("***CRC ERROR***"));
      //      Serial.print(F("calculated: "));
      //      Serial.println(crc, HEX);
      //      Serial.print(F("specified:  "));
      //      Serial.println(rxPacket->crc, HEX);
      delay(6000);
      return;
    }
    digitalWrite(A0, LOW);
    writefile(rxPacket->data, rxPacket->numBytes);
    packetNumber++;
    //rf95.waitPacketSent();
    //delay(100);
    rf95.send((byte)ok, ack);

    //wdt_reset();
  }
  return;
}
void printPacket(PACKET * p)
{
  if (counter <= 2) {
    lcd.setCursor(0, 1);
    lcd.print(F("PCKT "));
    lcd.setCursor(10, 2);
    lcd.print(fileName);
      //delay(10);
  }
  lcd.setCursor(5, 1);
  lcd.print(p->counter);
  lcd.print(F(" @ "));
  lcd.print(p->numBytes);
  lcd.print(F("B"));
  lcd.setCursor(0, 2);
  lcd.print(p->crc, HEX);
}
uint32_t calcCrc(byte * data, uint32_t len)
{
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
void writefile(byte * data, uint16_t len)
{
  my_file = SD.open(fileName, FILE_WRITE);
  if (my_file)
  {
    my_file.write(data, len);
    my_file.close();
    //Serial.print(data, len);
 //   if (counter <= 2) {
 //     lcd.setCursor(15, 1);
 //     lcd.print(F("SAVED"));
//      //Serial.println();
//      lcd.setCursor(10, 2);
//      lcd.print(fileName);
      //delay(10);
 //   }

    if (len <= 81) {
      wdt_reset();
      lcd.setCursor(0, 3);
      lcd.print(F("**COMPLETE**"));
      lcd.setCursor(10, 2);
      lcd.print(fileName);
      //Serial.println();
      //Serial.println();
      rf95.send((byte)ok, ack);
      //rf95.waitPacketSent();
      digitalWrite(A3, LOW);
      delay(100);
      digitalWrite(A1, HIGH);
      delay(100);
      digitalWrite(A1, LOW);
      delay(100);
      digitalWrite(A1, HIGH);
      delay(100);
      digitalWrite(A1, LOW);
      delay(100);
      digitalWrite(A1, HIGH);
      delay(100);
      digitalWrite(A1, LOW);
      digitalWrite(6, LOW);
    }
  }
}
