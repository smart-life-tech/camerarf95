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
#define CHIP_SELECT 10

RH_RF95 rf95(RFM95_CS, RFM95_INT);
LiquidCrystal_I2C lcd(0x27, 20, 4);
File photoFile;

struct PACKET {
  uint32_t counter;
  uint16_t numBytes;
  byte data[TXBUFFERSIZE];
  uint32_t crc;
};

const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[] = FILE_BASE_NAME "000.jpg";

uint32_t packetNumber = 0;

void setup() {
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A3, OUTPUT);
  digitalWrite(6, HIGH);
  pinMode(6, OUTPUT);
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A3, LOW);
  Serial.begin(115200);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(50);
  digitalWrite(RFM95_RST, HIGH);
  delay(50);

  lcd.begin(20, 4);
  lcd.createChar(5, {0x0, 0x1, 0x3, 0x16, 0x1c, 0x8, 0x0});
  lcd.createChar(6, {0x0, 0x1b, 0xe, 0x4, 0xe, 0x1b, 0x0});
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(F("CornOnTheBranch..."));
  lcd.setCursor(9, 3);
  lcd.print(F("...Shizzle!"));
  lcd.setCursor(0, 1);

  while (!rf95.init()) {
    Serial.println(F("LoRa failed"));
    while (1);
  }
  lcd.print(F("LoRa OK! "));
  lcd.write(5);
  lcd.print(F("    "));
  lcd.print(RF95_FREQ);

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println(F("Frequency failed"));
    while (1);
  }

  delay(500);
  if (!SD.begin(CHIP_SELECT)) {
    Serial.println(F("Card failed"));
  } else {
    lcd.setCursor(0, 2);
    lcd.print(F("SD OK! "));
    lcd.write(5);
  }

  while (SD.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 2] != '9') {
      fileName[BASE_NAME_SIZE + 2]++;
    } else if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 2] = '0';
      fileName[BASE_NAME_SIZE + 1]++;
    } else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    } else {
      return;
    }
  }

  lcd.setCursor(10, 2);
  lcd.print(fileName);
}

void loop() {
  if (rf95.available()) {
   
    uint8_t buf[sizeof(PACKET)];
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len)) {
        PACKET* rxPacket = (PACKET*)buf;
        
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("**PACKET RECEIVED**"));
        lcd.setCursor(0, 3);
        lcd.print(F("SNR = "));
        lcd.setCursor(13, 3);
        lcd.print(F("ERR = "));
        
        lcd.setCursor(6, 3);
        int snr = RH_RF95_REG_19_PKT_SNR_VALUE;
        lcd.print(snr);
        lcd.print(F("dB"));
        
        lcd.setCursor(13, 3);
        lcd.print(F("ERR = "));
        lcd.print(pckterr);
        digitalWrite(A0, HIGH);
        
        uint32_t crc = calcCrc(buf, sizeof(buf) - sizeof(crc));
        printPacket(rxPacket);
        
        if (rxPacket->counter != packetNumber) {
            digitalWrite(A3, HIGH);
            pckterr++;
            packetNumber = rxPacket->counter;
            
            if (pckterr >= 11) {
                digitalWrite(6, LOW);
            }
        }
        
        if (crc != rxPacket->crc) {
            delay(6000);
            return;
        }
        
        digitalWrite(A0, LOW);
        writefile(rxPacket->data, rxPacket->numBytes);
        packetNumber++;
        rf95.send((byte)ok, ack);
    }
}
