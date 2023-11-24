#include <SPI.h>
#include <SD.h>

// #define INCLUDE_METRICS
// #define DEBUG_BUILD

const unsigned long baudRate = 500000;
// const unsigned long baudRate = 115200;
//const unsigned long baudRate = 19200;

bool debugMode =  true;
#define trace false

#define FlashPageSize 64

#define F_SRA_OE 1
#define F_GEN_RES (1 << 1)
// #define F_ROM_CE (1 << 3)
#define F_GEN_TX_EN (1 << 4)
#define F_SRD_OE (1 << 5)
// #define F_ROM_OE (1 << 6)
#define F_SL165_LD (1 << 7)

#define ROM_CH 0
#define CNTR_CH 1
#define SD_CH 2

#define BUSY_LED A5

// // Direct Control
#define SPI_ADDR0 7
#define SPI_ADDR1 3
#define CNTR_SR595_LATCH 4
#define CNTR_SR595_OE 5

#define DTACK_OE_PIN 9
#define DTACK_VAL_PIN 10

// line buffer control
#define SR595_LD 6
#define PIN_ROM_WE 8
#define PIN_ROM_CE A3
#define PIN_ROM_OE A2

#define SD_CS_PIN A0

// int F_SRA_OE = 1,
//   F_GEN_RES = 1 << 1,
//   F_ROM_WE = 1 << 2,  
//   F_ROM_CE = 1 << 3,
//   F_GEN_TX_EN = 1 << 4,
//   F_SRD_OE = 1 << 5,
//   F_ROM_OE = 1 << 6,
//   F_SL165_LD = 1 << 7;

// int ROM_CH = 0,
//   CNTR_CH = 1,
//   SD_CH = 2;

// Direct Control
// int SPI_ADDR0 = 7;
// int SPI_ADDR1 = 3;
// int CNTR_SR595_LATCH = 4;
// int CNTR_SR595_OE = 5;

bool pendingFlash = false;

// we will reuse this buffer for all temp string operations to lower memory / stack usage
char g_tempReusableBuf[64];

#ifdef INCLUDE_METRICS
typedef struct {
  unsigned long control;
  unsigned long bufferWrite;
  unsigned long bytesReceive;
  unsigned long action;
  unsigned long parseBytes;
  unsigned long copyIncomingBuf;
} Metrics;
Metrics metrics;
#endif


void selectSPIChannel(int channel) {
  PORTD = (channel & 0b01) ? PORTD | (1 << SPI_ADDR0) : PORTD &~ (1 << SPI_ADDR0);
  PORTD = (channel & 0b10) ? PORTD | (1 << SPI_ADDR1) : PORTD &~ (1 << SPI_ADDR1);
  // set max spi speed (sd lib keeps on changing it)
  // SPCR = (1 << SPE) | (1 << MSTR);
  // SPSR |= (1 << SPI2X);

}

#define BYTE_TO_BINARY_PATTERN "0b%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 
char outText[11];

byte global_flags = 0xff;
inline void controlOut(byte flags, bool set, bool flush) {
#ifdef INCLUDE_METRICS  
  unsigned long start = micros();
#endif

  global_flags = set 
    ? global_flags | flags 
    : global_flags &~ flags;

  if (flush) {
    selectSPIChannel(CNTR_CH);
    // push values to shift register
    SPI.begin();
    SPI.transfer(global_flags);
    SPI.end();

    PORTD |= 1 << CNTR_SR595_LATCH;
    PORTD &= ~(1 << CNTR_SR595_LATCH);
  }

//  if (debugMode) {
//    Serial.print("DEBUG: Control: "); Serial.println(global_flags, BIN);
//  }
#ifdef INCLUDE_METRICS
  metrics.control += micros() - start;
#endif
}

//Serial Actions
enum {
//- Ping (availability)
  PING = 0,
//- Get 128 word(16bit) Page md5 Checksum 
  READ_128_PAGE_CRC,
  READ_128x128_PAGES,
//- Write 128 word(16bit) page (with data# polling)
  WRITE_128X_PAGES,
  WRITE_128X_PAGES_HEX,
//- Busy? (page write in progress)
  DEBUG_ON,
  DEBUG_OFF,
  WRITE_WORD,
// SEGA GENESIS control
  GENESIS_RESET,
  GENESIS_RESET_HOLD,
  GENESIS_RESET_RELEASE,
// TEST access to cartridge
  LIST_SD_FILES,
  FLASH_SD_ROM,
// no idea what is being asked
  UNKNOWN_ACTION
};

bool enabled = false;
// seize control and do not let genesis access it
void enableControl() {
  if (!enabled) {
    enabled = true;
    controlOut(F_SL165_LD, false, true);
    digitalWrite(SR595_LD, LOW);
    
    digitalWrite(PIN_ROM_WE, HIGH);

    pinMode(PIN_ROM_WE, OUTPUT);
    pinMode(PIN_ROM_OE, OUTPUT);
    pinMode(PIN_ROM_CE, OUTPUT);
    
    controlOut(F_GEN_TX_EN, true, true);

    // controlOut(0xff &~ F_SL165_LD, true, true);
#ifdef INCLUDE_METRICS    
    memset(&metrics, 0, sizeof(metrics));
#endif
    delay(100);
  }
}

// release control & let genesis access the memory
void disableControl() {
//  digitalWrite(SR595_LD, LOW);
//  controlOut(0xff, true, true);
  digitalWrite(SR595_LD, LOW);
  // digitalWrite(PIN_ROM_WE, HIGH);
  // go high impedience on WE pin
  pinMode(PIN_ROM_WE, INPUT);
  pinMode(PIN_ROM_OE, INPUT);
  pinMode(PIN_ROM_CE, INPUT);

  controlOut(0xff, true, true);
  controlOut(F_GEN_TX_EN, false, true);

  // selectSPIChannel(ROM_CH);
  
  enabled = false;
  delay(50);
}

int interruptReceived = 0;

void genInterrupt() {
  noInterrupts();
  interruptReceived = 1;
}

void lightSetup() {
  // pinMode(BUSY_LED, OUTPUT);
  // digitalWrite(BUSY_LED, HIGH);
  

  // set up interrupt
  const int intPin = 2;
  pinMode(intPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(intPin), genInterrupt, RISING);

  digitalWrite(DTACK_OE_PIN, HIGH);
  pinMode(DTACK_OE_PIN, OUTPUT);

  pinMode(DTACK_VAL_PIN, OUTPUT);
  digitalWrite(DTACK_VAL_PIN, HIGH);


  pinMode(SPI_ADDR0, OUTPUT);
  pinMode(SPI_ADDR1, OUTPUT);

  digitalWrite(CNTR_SR595_LATCH, LOW);
  pinMode(CNTR_SR595_LATCH, OUTPUT);

  digitalWrite(CNTR_SR595_OE, LOW);
  pinMode(CNTR_SR595_OE, OUTPUT);

  digitalWrite(SR595_LD, LOW);
  pinMode(SR595_LD, OUTPUT);

  digitalWrite(PIN_ROM_WE, HIGH);
  pinMode(PIN_ROM_WE, OUTPUT);

  controlOut(0xff, true, true);
  
  // rom setup so we do not get weird data
//  digitalWrite(F_GEN_RES, HIGH);
  controlOut(F_GEN_RES, true, true);

//  Serial.begin(57600);

  delay(100);

  enableControl();

  delay(1);

  disableControl();
}


void setup() {
  Serial.begin(baudRate);
  Serial.println();

  lightSetup();

  enableControl();

  initBootup();

  Serial.println(F("READY"));

  disableControl();

  // load default boot rom in SRAM
  genesisReset();
}

int parseAction(String stringAction) {
  if (stringAction.equals(F("PING"))) return PING;
  // if (stringAction.equals(F("READ_ROM_ID"))) return READ_ROM_ID;
  // if (stringAction.equals(F("READ_ROM_NAME"))) return READ_ROM_NAME;
  // if (stringAction.equals(F("READ_ROM_SIZE"))) return READ_ROM_SIZE;
  // if (stringAction.equals(F("READ_128_PAGE"))) return READ_128_PAGE;
  // if (stringAction.equals(F("READ_128_PAGE_CRC"))) return READ_128_PAGE_CRC;
  if (stringAction.equals(F("READ_128x128_PAGES"))) return READ_128x128_PAGES;
  // if (stringAction.equals(F("WRITE_128_PAGE"))) return WRITE_128_PAGE;
  if (stringAction.equals(F("WRITE_128X_PAGES"))) return WRITE_128X_PAGES;
  if (stringAction.equals(F("WRITE_128X_PAGES_HEX"))) return WRITE_128X_PAGES_HEX;
 
  // if (stringAction.equals(F("IS_ROM_BUSY"))) return IS_DEVICE_BUSY;
  if (stringAction.equals(F("DEBUG_ON"))) return DEBUG_ON;
  if (stringAction.equals(F("DEBUG_OFF"))) return DEBUG_OFF;
  // if (stringAction.equals(F("WRITE_WORD"))) return WRITE_WORD;
  // if (stringAction.equals(F("LOCK_ADDRESS"))) return LOCK_ADDRESS;
  // if (stringAction.equals(F("LOCK_ADDRESS_AND_DATA"))) return LOCK_ADDRESS_AND_DATA;
  // if (stringAction.equals(F("ENABLE_WRITE_PROTECT"))) return ENABLE_WRITE_PROTECT;
  // if (stringAction.equals(F("DISABLE_WRITE_PROTECT"))) return DISABLE_WRITE_PROTECT;
  
  // genesis control
  if (stringAction.equals(F("GENESIS_RESET"))) return GENESIS_RESET;
  if (stringAction.equals(F("GENESIS_RESET_HOLD"))) return GENESIS_RESET_HOLD;
  if (stringAction.equals(F("GENESIS_RESET_RELEASE"))) return GENESIS_RESET_RELEASE;

  // cart test
  // if (stringAction.equals(F("CART_LOCK_ADDRESS"))) return CART_LOCK_ADDRESS;
  // if (stringAction.equals(F("CART_READ_128_PAGE"))) return CART_READ_128_PAGE;
  // if (stringAction.equals(F("CART_READ_128x128_PAGES"))) return CART_READ_128x128_PAGES;
  if (stringAction.equals(F("LIST_SD_FILES"))) return LIST_SD_FILES;
  if (stringAction.equals(F("FLASH_SD_ROM"))) return FLASH_SD_ROM;
  

  return UNKNOWN_ACTION;
}

unsigned int v = 0xabcd;

void loop() {
  bool actionInvoked = false;
  // digitalWrite(BUSY_LED, LOW);

  if (interruptReceived) {


    // // load the 16bit interruptId from data lines
    // controlOut(F_SL165_LD, false, true);

    // // shift data in 
    // selectSPIChannel(ROM_CH);
    // SPI.begin();
    // unsigned int inValue = SPI.transfer16(0);
    // SPI.end();
    // unsigned int interruptId = (((inValue & 0xff) << 8) + (inValue >> 8)) & 0xffff;
    // TODO: figure out why this data is not coming in properly
    unsigned int interruptId = 0;

    Serial.print("Got interrupt: ");Serial.println(interruptId, HEX);
    // digitalWrite(BUSY_LED, HIGH);
    // Serial.println("interrupt!!!");

    enableControl();


    if (!actionInvoked) {
      handleGenSignal(interruptId);
    }

    // pinMode(9, INPUT);
    // release genesis by reseting flip flop
    disableControl();
    interruptReceived = 0;
  

    Serial.print("Released interrupt: ");Serial.println(interruptId, HEX);

    // digitalWrite(DTACK_VAL_PIN, LOW);
    digitalWrite(DTACK_OE_PIN, LOW);
    digitalWrite(DTACK_OE_PIN, HIGH);
    // digitalWrite(DTACK_VAL_PIN, HIGH);
    interrupts();
  }

  if (pendingFlash) {
    pendingFlash = false;
    enableControl(); flashAndStartROM(g_tempReusableBuf); disableControl(); genesisReset();
  }
 
  if (Serial.available() > 0) {

    String input = Serial.readStringUntil('\n');
#ifdef DEBUG_BUILD
    if (debugMode) {
      Serial.print(F("DEBUG: parsing raw input: "));Serial.println(input);
    }
#endif
    int action = parseAction(input);

#ifdef DEBUG_BUILD
    if (debugMode) {
      Serial.print(F("DEBUG: "));Serial.print(input);Serial.print(F("; decoded:"));Serial.println(action);
    }
#endif

    // digitalWrite(BUSY_LED, HIGH);
    actionInvoked = true;
    switch (action) {
      case PING: Serial.println(F("PONG")); break;
      // case READ_ROM_ID: enableControl(); outputDeviceId(); disableControl(); break;
      // case READ_ROM_NAME: enableControl(); outputDeviceName(); disableControl(); break;
      // case READ_ROM_SIZE: enableControl(); outputDeviceSize(); disableControl(); break;
      // case READ_128_PAGE: enableControl(); read128WordPage(false); disableControl(); break;
      // case READ_128_PAGE_CRC: enableControl(); read128WordPageCRC(); disableControl(); break;
      case READ_128x128_PAGES: enableControl(); read128x128WordPages(false); disableControl(); break;
      // case WRITE_128_PAGE: enableControl(); write128WordPage(); disableControl(); break;
      case WRITE_128X_PAGES: enableControl(); write128WordXPages(); disableControl(); break;
      case WRITE_128X_PAGES_HEX: enableControl(); write128WordXPages(true); disableControl(); break;

      // case DEBUG_ON: debugMode = true; Serial.println("DEBUG mode ON!"); break;
      // case DEBUG_OFF: debugMode = false; Serial.println("DEBUG mode OFF!"); break;


      // case WRITE_WORD: enableControl(); writeDataAtAddress(); break;
      // case LOCK_ADDRESS: enableControl(); lockAddress(false); break;
      // case LOCK_ADDRESS_AND_DATA: enableControl(); lockAddressAndData(); break;
      // case ENABLE_WRITE_PROTECT: writeProtect(true); break;
      // case DISABLE_WRITE_PROTECT: writeProtect(false); break;
      // Sega Genesis
      case GENESIS_RESET: genesisReset(); break;
      case GENESIS_RESET_HOLD: genesisResetHold(); break;
      case GENESIS_RESET_RELEASE: genesisResetRelease(); break;

      // read via cart access
//      case CART_LOCK_ADDRESS: disableControl(); lockAddress(true); break;
      // case CART_READ_128_PAGE: read128WordPage(true); break;
      // case CART_READ_128x128_PAGES: read128x128WordPages(true); break;

      case LIST_SD_FILES: listSDFiles(); break;
      case FLASH_SD_ROM: enableControl(); flashAndStartROM(NULL); disableControl(); genesisReset(); break;
      
      // Invalid action
      case UNKNOWN_ACTION: Serial.println(F("UNKNOWN_ACTION")); break;
      default: break;
    }
    actionInvoked = false;

//    if (debugMode) {
//      Serial.print(F("DEBUG: "));Serial.println(F("Waiting on new input..."));
//    }

  }
}

// ********************************
// ********************************
// ********************************
// Sega Genesis
// ********************************
// ********************************
// ********************************

void genesisReset() {
#ifdef DEBUG_BUILD
  if (debugMode) {
    Serial.println(F("DEBUG: reset->low"));
  }
#endif
  controlOut(F_GEN_RES, false, true);
  delay(100);
#ifdef DEBUG_BUILD
  if (debugMode) {
    Serial.println(F("DEBUG: reset->high"));
  }
#endif
  controlOut(F_GEN_RES, true, true);

  Serial.println(F("ACK_GENESIS"));
}

void genesisResetHold() {
#ifdef DEBUG_BUILD
  if (debugMode) {
    Serial.println(F("DEBUG: RESET_HOLD"));
  }
#endif
  controlOut(F_GEN_RES, false, true);
  Serial.println(F("ACK_GENESIS"));
}


void genesisResetRelease() {
#ifdef DEBUG_BUILD
  if (debugMode) {
    Serial.println(F("DEBUG: RESET_RELEASE"));
  }
#endif
  controlOut(F_GEN_RES, true, true);
  Serial.println(F("ACK_GENESIS"));
}

void initBootup() {
  // check if SRAM contains GENESIS ROM

  // load default ROM if not found (POWER ON)

  int base = 0x80;
//  outAddress(address);
  int value = in16bits(base);
#ifdef DEBUG_BUILD
  if (debugMode) {
    int value1 = in16bits(base + 1);
    int value2 = in16bits(base + 2);
    int value3 = in16bits(base + 3);
    Serial.print(F("DEBUG: initCheck: "));
    Serial.print(value, HEX);Serial.print(":");
    Serial.print(value1, HEX);Serial.print(":");
    Serial.print(value2, HEX);Serial.print(":");
    Serial.print(value3, HEX);
    Serial.println(F(", expected: 5345"));
  }
#endif
  if (value != 0x5345) {
#ifdef DEBUG_BUILD
    Serial.println("DEBUG: Loading bootrom...");
#endif
    strcpy(g_tempReusableBuf, "_BOOT.MD");
    pendingFlash = true;

    // flashAndStartROM("_BOOT.MD");
    delay(100);
  } else {
#ifdef DEBUG_BUILD
    Serial.println("DEBUG: Genesys rom found, booting...");
#endif
  }
  
}

// ********************************
// ********************************
// ********************************
// SD CARD
// ********************************
// ********************************
// ********************************

void listSDFiles() {
  selectSPIChannel(SD_CH);
  SD.begin(SD_CS_PIN);
  File root = SD.open("/");
  printDirectory(root);                             // list files on SD1
  root.close();
  Serial.println("ACK");
}

bool printDirectory(File dir, String prefix)               // lists the files and filesize on the SD card (only root)
{
  char romName[28];
  if (!dir) {
    Serial.print("Directory not open: "); Serial.println(prefix);
  }
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      return false;
    }

    if (entry.isDirectory()) {
      Serial.print(entry.name());
      Serial.println("/");
      printDirectory(entry, String(entry.name()) + "/");
    }
    else if (String(entry.name()).indexOf(".MD") > 0) {
      entry.seek(0x150);
      entry.read(romName, 47);
      romName[47] = '\0';

      Serial.print(prefix);
      Serial.print(entry.name());
      Serial.print(F(" | "));
      Serial.print(romName);
      Serial.print(F(" | "));
      Serial.println(entry.size(), DEC);
    }

    entry.close();
  }
  
  return true;
}

bool printDirectory(File dir) {
  printDirectory(dir, "");
}

void flashAndStartROM(char *inFilename) {
#ifdef DEBUG_BUILD
  selectSPIChannel(SD_CH);
  SD.begin(SD_CS_PIN);
  Serial.println(F("DEBUG::flashAndStartROM listing SD card contents"));

  File root = SD.open("/");
  printDirectory(root);                             // list files on SD1
  root.close();
#endif

#ifdef INCLUDE_METRICS
  unsigned long action_start = micros();
  unsigned long read_start = micros();
#endif

  char filename[32];
  memset(filename, 0, sizeof(filename));
  if (!inFilename) {
    int error = loadStringLine(filename, 32);
    if (error) {
      debugErrorOut(error);
      Serial.print(F("ERR_NO_FILENAME "));Serial.println(error, HEX);
      return;
    }
  }

  char *targetRom = inFilename ? inFilename : filename;

#ifdef DEBUG_BUILD
  if (debugMode) {
    Serial.print(F("DEBUG: Target file: "));Serial.println(targetRom);
  }
  Serial.print(F("DEBUG: Opening file rom 1:"));Serial.println(targetRom);
  // Serial.print(F("DEBUG: Opening file rom 1:"));Serial.println(inFilename);
#endif

//  unsigned long read_start = micros();

  selectSPIChannel(SD_CH);
  SD.begin(SD_CS_PIN);

  File file = SD.open(inFilename);
  if (!file) {
#ifdef DEBUG_BUILD
    if (debugMode) {
      Serial.println(F("DEBUG: Bailing as the file doesn't exist"));
    }
#endif
    Serial.println(F("NOT_FOUND"));
    return;
  } else {
    Serial.println(F("success"));
  }

#ifdef DEBUG_BUILD
  if (debugMode) {
    Serial.print(F("DEBUG: File: "));Serial.print(file.name());Serial.print("|");Serial.println(file.size());
  }
#endif

  char romName[25];
  file.seek(0x150);
  file.read(romName, 24);
  file.seek(0);
  romName[24] = '\0';

#ifdef INCLUDE_METRICS
  metrics.bytesReceive += micros() - read_start;
#endif

#ifdef DEBUG_BUILD
  if (debugMode) {
    Serial.print(F("DEBUG: Flashing ROM: ")); Serial.print(file.name()); Serial.print(" : "); Serial.println(romName);
  }
#endif

  int pages = file.size() / FlashPageSize / 2;
  int buf[FlashPageSize];
  for (int i = 0 ; i < pages; i++) {

#ifdef INCLUDE_METRICS
    read_start = micros();
#endif

    selectSPIChannel(SD_CH);
    
    unsigned long address = i * FlashPageSize;
    file.read(buf, FlashPageSize * 2);
  
#ifdef INCLUDE_METRICS
    metrics.bytesReceive += micros() - read_start;
#endif

    if (i%16==0) {
      Serial.print(F("Writing page: "));Serial.print(i);Serial.print(F(", at addr: "));Serial.print(address, HEX);
      Serial.print(F(", of pages: ")); Serial.println(pages);
    }

    writeBuffer(address, buf, FlashPageSize, true, true);
  }
  file.close();

#ifdef INCLUDE_METRICS
  metrics.action = micros() - action_start;
#endif

  Serial.println("ACK");

#ifdef INCLUDE_METRICS
  Serial.print(F("Control:"));Serial.print(metrics.control);Serial.print("|");
  Serial.print(F("Read:"));Serial.print(metrics.bytesReceive);Serial.print("|");
  Serial.print(F("Write:"));Serial.print(metrics.bufferWrite);Serial.print("|");
//  Serial.print("PARSE:");Serial.print(metrics.parseBytes);Serial.print("|");
//  Serial.print("COPY:");Serial.print(metrics.copyIncomingBuf);Serial.print("|");
  Serial.print(F("Action:"));Serial.print(metrics.action);Serial.print("|");
  Serial.println();
#endif
}


enum {
  SUCCESS = 0,
  ERROR_PARSE = 1,
  ERROR_TIMEOUT = 2,
  ERROR_ADDR_RANGE = 3
  
};

void debugErrorOut(int error) {
#ifdef DEBUG_BUILD
  if (debugMode) {
    Serial.print(F("DEBUG: got error "));
    switch (error) {
      case SUCCESS: Serial.println(F("SUCCESS")); break;
      case ERROR_PARSE: Serial.println(F("CAN NOT PARSE")); break;
      case ERROR_TIMEOUT: Serial.println(F("TIMEOUT")); break;
      case ERROR_ADDR_RANGE: Serial.println(F("ADDRESS RANGE ERROR")); break;
      default: Serial.print(F("UNKNOWN:")); Serial.println(error, HEX); break;
    }
  }
#endif
}

char g_inputBuf[32];

int loadStringLine(char *buf, int maxLength) {
  Serial.println(F("AWAIT_STR"));
  int error = SUCCESS;
  long start = millis();
  while (Serial.available() == 0) {
    if (millis() - start > 3000) {
      error = ERROR_TIMEOUT;
      Serial.println(F("DEBUG: loadAddress TIMEOUT, bailing"));
      return;
    }
  }

  Serial.readBytesUntil('\n', buf, maxLength);
  Serial.println("ACK_STR");

  return SUCCESS;
}

unsigned long loadAddress(int &error, bool alignPageBoundary, bool validateChipSize) {
  Serial.println(F("AWAIT_ADDR_HEX"));
  error = SUCCESS;

  // wait for 3 secs for address or bail with an error
  long start = millis();
  while (Serial.available() == 0) {
    if (millis() - start > 3000) {
      error = ERROR_TIMEOUT;
#ifdef DEBUG_BUILD
      Serial.println(F("DEBUG: loadAddress TIMEOUT, bailing"));
#endif
      return;
    }
  }

#ifdef DEBUG_BUILD
  Serial.println(F("DEBUG: Parsing serial input"));
#endif
  char inputBuf[8];
  // read in 24 bit value
  memset(inputBuf, 0, 8);
  Serial.readBytesUntil('\n', inputBuf, 7);
  // parse hex address that should be lower than chip size or it will be invalid address
  char *endChar = 0;
  unsigned long address = strtol(inputBuf, &endChar, 16);
  const char *str = inputBuf;
  int charsParsed = 0;
  while (*str && *str != *endChar) {
    ++charsParsed;
    ++str;
  }

#ifdef DEBUG_BUILD
  if (debugMode) {
    Serial.print(F("DEBUG: chars parsed:"));Serial.println(charsParsed);
  }
#endif

  // sink the rest of the stream until newline
  if (Serial.available() > 0) {
    Serial.readStringUntil('\n');
  }
  
  if (charsParsed == 0) {
    error = ERROR_PARSE;
    return 0;
  }

#ifdef DEBUG_BUILD
  if (debugMode) {
    Serial.print(F("DEBUG: Got address: 0x"));Serial.println(address, HEX);
  }
#endif

  Serial.println("ACK_ADDR");

#ifdef DEBUG_BUILD
  if (debugMode) {
    Serial.print(F("DEBUG: Returning address: 0x"));Serial.println(address, HEX);
  }
#endif

  if (alignPageBoundary) {
    // figure out the page boundary if 
    unsigned long pageStartAddress = address - address % 128;

#ifdef DEBUG_BUILD
    if (debugMode) {
      Serial.print(F("DEBUG: Page start address: 0x"));Serial.println(pageStartAddress, HEX);
    }
#endif

    return pageStartAddress;
  } else {
    return address;
  }
}

int loadAddress(int &error, bool alignPageBoundary) {
  loadAddress(error, alignPageBoundary, true);
}

int loadAddress(int &error) {
  loadAddress(error, false, true);
}

int loadDataBytes(unsigned int *buf, unsigned int numWords) {
#ifdef INCLUDE_METRICS
  unsigned long metrics_start = micros();
#endif
  int error = SUCCESS;

  Serial.println(F("AWAIT_DATA_HEX"));
  // wait for 3 secs for first byte or bail with an error
  long start = millis();
  while (Serial.available() == 0) {
    if (millis() - start > 3000) {
      return ERROR_TIMEOUT;
    }
  }
  
  int totalReadWords = numWords;
  int numBytes = numWords * 2;
//  int wordsRead = 0;
  int bytesRead = Serial.readBytes((byte*)buf, numBytes);

#ifdef DEBUG_BUILD
  if (debugMode) {
    Serial.print(F("DEBUG: bytesRead: "));Serial.println(bytesRead);
  }

  if (debugMode) {
    char tmpStr[5];

    Serial.print(F("Debug: ACK_DATA: "));
    for (unsigned long i = 0; i < numWords; i++) {
      sprintf(tmpStr, "%04X", buf[i]);
      Serial.print(tmpStr);
    }
    Serial.println();
  }
#endif

#ifdef INCLUDE_METRICS
  metrics.bytesReceive += micros() - metrics_start;
#endif

  Serial.println(F("ACK_DATA"));
  
  return SUCCESS;
}

int loadDataHex(unsigned int *buf, unsigned int bufSize) {

#ifdef INCLUDE_METRICS
  unsigned long metrics_start = micros();
#endif

  int error = SUCCESS;

  Serial.println(F("AWAIT_DATA_HEX"));

  // wait for 3 secs for first byte or bail with an error
  long start = millis();
  while (Serial.available() == 0) {
    if (millis() - start > 3000) {
      return ERROR_TIMEOUT;
    }
  }
  unsigned int otherBuf[128] = {0};

  int totalReadWords = bufSize;
  int wordsRead = 0;
  while (wordsRead < totalReadWords) {
    char input[5];
    int bytesRead = Serial.readBytes(input, 4);
    if (bytesRead < 4) {
#ifdef DEBUG_BUILD      
      if (debugMode) {
        Serial.print(F("DEBUG: bytesRead: "));Serial.print(bytesRead);Serial.print(F(", expected: 4"));
        Serial.print(F(", wordsRead:"));Serial.print(wordsRead);Serial.print(F(", expected:"));Serial.println(totalReadWords);
      }
#endif
      return ERROR_TIMEOUT;
    }

#ifdef INCLUDE_METRICS
    unsigned long parseb_start = micros();
#endif

    char *endChar = 0;
    unsigned int dataWord = strtol(input, &endChar, 16);
    const char *str = input;
    int charsParsed = 0;
    while (*str && *str != *endChar) {
      ++charsParsed;
      ++str;
    }
    if (charsParsed == 0) {
      return ERROR_PARSE;
    }
    otherBuf[wordsRead++] = flipIntBytes(dataWord);

#ifdef INCLUDE_METRICS
    metrics.parseBytes += micros() - parseb_start;
#endif

  }
#ifdef DEBUG_BUILD  
  if (debugMode) {
    Serial.println(F("DEBUG: about to ack data"));
  }
#endif
  Serial.println(F("ACK_DATA"));

  unsigned long copy_start = micros();
  memcpy(buf, otherBuf, bufSize * sizeof(int));

#ifdef INCLUDE_METRICS
  metrics.copyIncomingBuf += micros() - copy_start;
#endif

#ifdef DEBUG_BUILD
  if (debugMode) {
    char tmpStr[5];

    Serial.print(F("Debug: ACK_DATA: "));
    for (unsigned long i = 0; i < bufSize; i++) {
      sprintf(tmpStr, "%04X", buf[i]);
      Serial.print(tmpStr);
    }
    Serial.println();
  }
#endif

#ifdef INCLUDE_METRICS
  metrics.bytesReceive += micros() - metrics_start;
#endif

  return SUCCESS;
}

int loadData(unsigned int *buf, unsigned int numWords) {
//  return loadDataHex(buf, numWords);
  return loadDataBytes(buf, numWords);
}

int loadData(unsigned int *buf) {
  return loadData(buf, 128);
}

int loadPages(int &error) {
  Serial.println(F("AWAIT_PAGES_HEX"));
  error = SUCCESS;

  // wait for 3 secs for address or bail with an error
  long start = millis();
  while (Serial.available() == 0) {
    if (millis() - start > 3000) {
      error = ERROR_TIMEOUT;
      return;
    }
  }
    
  // read in 16 bit value
  char input[5];
  memset(input, 0, 5);
  Serial.readBytesUntil('\n', input, 4);
  // parse hex address that should be lower than chip size or it will be invalid address
  char *endChar = 0;
  unsigned int pages = strtol(input, &endChar, 16);
  const char *str = input;
  int charsParsed = 0;
  while (*str && *str != *endChar) {
    ++charsParsed;
    ++str;
  }

#ifdef DEBUG_BUILD  
  if (debugMode) {
    Serial.print(F("DEBUG: chars parsed:"));Serial.println(charsParsed);
  }
#endif

  // sink the rest of the stream until newline
  if (Serial.available() > 0) {
    Serial.readStringUntil('\n');
  }
  
  if (charsParsed == 0) {
    error = ERROR_PARSE;
    return 0;
  }

#ifdef DEBUG_BUILD
  if (debugMode) {
    Serial.print(F("DEBUG: Got pages: 0x"));Serial.println(pages, HEX);
  }
#endif
  
  Serial.println(F("ACK_PAGES"));
  return pages;
}

void read128WordPage(bool cartRead) {

  int error;
  unsigned long address = loadAddress(error, true, !cartRead);

  if (error) {
    debugErrorOut(error);
    Serial.print(F("ERR_NO_ADDR "));Serial.println(error, HEX);
    return;
  }

  int bytesToRead = 4;

#ifdef DEBUG_BUILD  
  if (debugMode) {
    char tmpStr[5];
    Serial.print("DEBUG: device:"); Serial.print(address, HEX); Serial.print(" bytes: "); 
    // output 128 words (we are using 2 chips each containing a byte) for the page
    for (unsigned long i = 0; i < bytesToRead; i++) {
      unsigned int val = in16bits(address + i);
      sprintf(tmpStr, "%04X", val);
      Serial.print(tmpStr);
    }
    Serial.println();
  }
#endif

  Serial.println("DATA_BEGIN");
  char tmpStr[5];
  // output 128 words (we are using 2 chips each containing a byte) for the page
  for (unsigned long i = 0; i < bytesToRead; i++) {
    unsigned int val = in16bits(address + i);
    sprintf(tmpStr, "%04X", val);
    Serial.print(tmpStr);
  }
  Serial.println();
}


void read128x128WordPages(bool cartRead) {
  int error;
  
  unsigned long address = loadAddress(error, false, !cartRead);
  
#ifdef DEBUG_BUILD  
  Serial.print(F("DEBUG: Returned address: 0x"));Serial.println(address, HEX);
  if (debugMode) {
    Serial.print(F("DEBUG: real address: 0x"));Serial.print(address, HEX);Serial.print(F(", clamped address: 0x"));Serial.println(address - address % 128, HEX);
  }
#endif

  address = address - address % 128;
  
  if (error) {
    debugErrorOut(error);
    Serial.print(F("ERR_NO_ADDR "));Serial.println(error, HEX);
    return;
  }

  unsigned long pagesToLoad = loadPages(error);
  if (error) {
    debugErrorOut(error);
    Serial.print(F("ERR_NO_PAGES "));Serial.println(error, HEX);
    return;
  }

  // output 128 words (we are using 2 chips each containing a byte) for the page
  unsigned char crc = 0;
  const long wordsPerPage = 128;
  char tmpStr[5];

#ifdef DEBUG_BUILD  
  if (debugMode) {
    Serial.print(F("DEBUG: pages to read: "));Serial.print(pagesToLoad);  
    Serial.println(F(", at addresses:"));
    for (unsigned long page = 0; page < pagesToLoad; page++) {
      unsigned long pageOffset = page * wordsPerPage;
      unsigned long read_at_address = address + pageOffset;
      Serial.print(F("DEBUG: page:"));Serial.print(page);Serial.print(F(", address:"));Serial.println(read_at_address);
    }
  }
  
  if (debugMode) {
    char tmpStr[5];
    Serial.print("DEBUG: device:"); Serial.print(address, HEX); Serial.print(" bytes: "); 
    // output 128 words (we are using 2 chips each containing a byte) for the page
    for (unsigned long i = 0; i < 128; i++) {
       unsigned int val = in16bits(address + i);
      sprintf(tmpStr, "%04X", val);
      Serial.print(tmpStr);
    }
    Serial.println();
  }
#endif
  
  Serial.println(F("DATA_BEGIN"));

  unsigned int data[130] = {0};

  for (unsigned long page = 0; page < pagesToLoad; page++) {
    unsigned long pageOffset = page * wordsPerPage;
    unsigned char page_crc = 0;
    for (unsigned long i = 0; i < wordsPerPage; i++) {
      unsigned long read_at_address = address + pageOffset + i;
      unsigned int val = in16bits(read_at_address);
      data[i] = val;
      while (Serial.availableForWrite() < 2) {}
    }

    for (unsigned long i = 0; i < wordsPerPage; i++) {
      unsigned char *vals = (unsigned char *)&data[i];
      Serial.write(vals[1]);
      Serial.write(vals[0]);
    }
    crc = calc_crc((unsigned char*)data, sizeof(int) * wordsPerPage, crc);
  }
  Serial.write(crc);
  Serial.println();
}

void read128WordPageCRC() {
  int error;
  unsigned long address = loadAddress(error, true);

  if (error) {
    debugErrorOut(error);
    Serial.print(F("ERR_NO_ADDR "));Serial.println(error, HEX);
    return;
  }

#ifdef DEBUG_BUILD  
  if (debugMode) {
    Serial.println(F("DEBUG: Reading page..."));
  }
#endif

  unsigned int data[128] = {0};
  for (unsigned int i = 0; i < 128; i++) {
    data[i] = in16bits(address + i);
  }

#ifdef DEBUG_BUILD  
  if (debugMode) {
    Serial.println(F("DEBUG: calc CRC..."));
  }
#endif

  unsigned char calculated_crc = calc_crc((unsigned char *)data, 128 * sizeof(int));
  Serial.println(calculated_crc, HEX);
}


void write128WordPage() {
  int error = SUCCESS;
  unsigned long address = loadAddress(error, true);

  if (error) {
    debugErrorOut(error);
    Serial.print(F("ERR_NO_ADDR "));Serial.println(error, HEX);
    return;
  }

  unsigned int data[128] = {0};
  error = loadData(data);
  if (error) {
    debugErrorOut(error);
#ifdef DEBUG_BUILD  
    if (debugMode) {
      char tmpStr[5];
      Serial.print(F("DEBUG: Received page: "));
      for (unsigned int i = 0; i < 128; i++) {
        sprintf(tmpStr, "%04X", data[i]);
        Serial.print(tmpStr);
      }
      Serial.println();
    }
#endif

    Serial.print(F("ERR_NO_DATA "));Serial.println(error, HEX);
    return;
  }

  writeBuffer(address, data, 128, true);

  Serial.println(F("ACK_DATA_WRITE"));

  unsigned char calculated_crc = calc_crc((unsigned char *)data, 128 * sizeof(int));
  Serial.println(calculated_crc, HEX);
  
}


void write128WordXPages(bool hex) {

#ifdef INCLUDE_METRICS
  unsigned long metrics_start = micros();
#endif

  int error = SUCCESS;
  unsigned long address = loadAddress(error, true);

  if (error) {
    debugErrorOut(error);
    Serial.print(F("ERR_NO_ADDR "));Serial.println(error, HEX);
    return;
  }

  unsigned int pagesToLoad = loadPages(error);
  if (error) {
    debugErrorOut(error);
    Serial.print(F("ERR_NO_PAGES "));Serial.println(error, HEX);
    return;
  }
  const int num128Pages = 1;
  unsigned int data[128] = {0};

  // we still have to write page by page via serial as atmega only has 2KB of ram
  unsigned char calculated_crc = 0;
  for (unsigned long page = 0; page < pagesToLoad; page += num128Pages) {
    error = hex ? loadDataHex(data, 128 * num128Pages) : loadDataBytes(data, 128 * num128Pages);
    if (error) {
      debugErrorOut(error);
#ifdef DEBUG_BUILD  
      if (debugMode) {
        char tmpStr[5];
        Serial.print(F("DEBUG: Received page: "));
        for (unsigned int i = 0; i < 128 * num128Pages; i++) {
          sprintf(tmpStr, "%04X", data[i]);
          Serial.print(tmpStr);
        }
        Serial.println();
      }
#endif

      Serial.print(F("ERR_NO_DATA "));Serial.println(error, HEX);
      return;
    }
    unsigned long page_address = address + page * 128;

#ifdef DEBUG_BUILD  
    if (debugMode) {
      char tmpStr[5];
      Serial.print(F("DEBUG: addr:")); Serial.print(page_address, HEX); Serial.print(F(" writing ")); Serial.print(num128Pages); Serial.print(F(" pages: "));
      // output 128 words (we are using 2 chips each containing a byte) for the page
      for (unsigned int i = 0; i < 128 * num128Pages; i++) {
        sprintf(tmpStr, "%04X", data[i]);
        Serial.print(tmpStr);
      }
      Serial.println();
    }

    Serial.println(F("DEBUG: Invoking write buffer"));
#endif

    int error = writeBuffer(page_address, data, 128 * num128Pages, true, true);
    if (error) {
      debugErrorOut(error);
      Serial.print(F("ERR_WRITE_ERROR "));Serial.println(error, HEX);
      return;
    }
    
    calculated_crc = calc_crc((unsigned char *)data, 128 * num128Pages * sizeof(int), calculated_crc);

#ifdef DEBUG_BUILD
    if (debugMode) {
      Serial.print(F("DEBUG: temp CRC: ")); Serial.println(calculated_crc, HEX);
    }
#endif
  }

  Serial.println(F("ACK_DATA_WRITE"));

  Serial.println(calculated_crc, HEX);

/*
 * typedef struct {
  unsigned long control;
  unsigned long bufferWrite;
  unsigned long bytesReceive;
} Metrics;
 */
#ifdef INCLUDE_METRICS
  metrics.action = micros() - metrics_start;

  Serial.print("CNTR:");Serial.print(metrics.control);Serial.print("|");
  Serial.print("RCV:");Serial.print(metrics.bytesReceive);Serial.print("|");
  Serial.print("WRT:");Serial.print(metrics.bufferWrite);Serial.print("|");
//  Serial.print("PARSE:");Serial.print(metrics.parseBytes);Serial.print("|");
//  Serial.print("COPY:");Serial.print(metrics.copyIncomingBuf);Serial.print("|");
  Serial.print("TOTAL:");Serial.print(metrics.action);Serial.print("|");
  Serial.println();
#else
  Serial.println(F("CNTR:0|RCV:0|WRT:0|TOTAL:0"));
#endif
}

void write128WordXPages() {
  write128WordXPages(false);
}

// ***************************************************************************************************************************************
// *****************************       MEMORY ACCESSS                                           ******************************************
// ***************************************************************************************************************************************
// Interaction with Device
// internals for working with the eeprom chips

inline int flipIntBytes(int in) {
  return (0xff00 & in << 8) + (0xff & in >> 8);
}

int writeBuffer(long addr24bit, int *buffer, int bufferSize, bool waitSuccess) {
  writeBuffer(addr24bit, buffer, bufferSize, waitSuccess, false);
}


int writeBuffer(long addr24bit, int *buffer, int bufferSize, bool waitSuccess, bool flipBytes) {

#ifdef INCLUDE_METRICS
  unsigned long start = micros();
#endif

#ifdef DEBUG_MODE
  if (debugMode) {
    Serial.print("DEBUG: flipBytes:");Serial.print(flipBytes);
    Serial.println();
  }
#endif
 
  digitalWrite(PIN_ROM_OE, HIGH);
  digitalWrite(PIN_ROM_CE, LOW);
  controlOut(F_SRA_OE | F_SRD_OE, false, true);

  for (unsigned long i = 0; i < bufferSize; i++) {
    unsigned long target_address = addr24bit + i;

    shiftOutAddressAndData(target_address, flipBytes ? flipIntBytes(buffer[i]) : buffer[i]);

    // we have PORTB pins are offset by 8
    // PORTB &= ~(1 << (8 - PIN_ROM_WE));
    // PORTB |= 1 << (8 - PIN_ROM_WE);
    digitalWrite(PIN_ROM_WE, LOW);
    digitalWrite(PIN_ROM_WE, HIGH);

    if (true) {
      continue;
    } else
    if (waitSuccess) {
      
      unsigned long page_start_address = long(i/128) * 128;
      bool pageBreak = target_address > 0 && ((target_address + 1) % 128 == 0 || i == bufferSize - 1);
      
      if (!pageBreak) continue;

#ifdef DEBUG_BUILD  
      if (debugMode) {
        char tmpStr[5];
        Serial.print(F("DEBUG: target_address: ")); Serial.print(target_address, HEX); 
        Serial.print(F(", > 0: ")); Serial.print(target_address > 0); 
        Serial.print(F(", target_address % 128 == 0: ")); Serial.print((target_address + 1) % 128 == 0);
        Serial.print(F(", i == bufferSize - 1: ")); Serial.println(i == bufferSize - 1); 

        Serial.print(F("DEBUG: PAGE WRITTEN:: index:")); Serial.print(i); Serial.print(F(", bufSize:")); Serial.print(bufferSize); 
        Serial.print(F(", addr:")); Serial.print(target_address, HEX); Serial.print(F(" writen @"));Serial.print(page_start_address);
        Serial.print(F(" of size: "));Serial.print(i - page_start_address + 1);
        Serial.print(F(", page bytes: \n"));
        // output 128 words (we are using 2 chips each containing a byte) for the page
        for (unsigned long y = page_start_address; y <= i; y++) {
          sprintf(tmpStr, "%04X", buffer[y]);
          Serial.print(tmpStr);
        }
        Serial.println();

      }
#endif      

      unsigned long start = millis();
      while(true) {
        // poll 
        unsigned int last_word = buffer[i];
        unsigned int in_data = in16bits(target_address);

#ifdef DEBUG_BUILD  
        if (debugMode) {
          Serial.print(F("DEBUG: in_data: ")); Serial.print(in_data, HEX);Serial.print(" from address: ");Serial.println(target_address, HEX);
          Serial.print(F("DEBUG: rom data: ")); Serial.print(in_data & 0x8080, BIN);Serial.print(" [");Serial.print(in_data, HEX);Serial.print("]");
          Serial.print(F(", buf data: ")); Serial.print(last_word & 0x8080, BIN);Serial.print(" [");Serial.print(last_word, HEX);Serial.print("]");
          Serial.print(F(", index: ")); Serial.print(target_address);
          Serial.print(F(", data match?: ")); Serial.println((in_data & 0x8080) == (last_word & 0x8080));
        }
#endif

        if (in_data == last_word) {
#ifdef DEBUG_BUILD  
          Serial.println(F("DEBUG: page write complete, moving on")); 
          Serial.print(F("DEBUG: PAGE READ:: \n"));
#endif

          char tmpStr[5];
          for (unsigned long dAddr = page_start_address; dAddr < page_start_address + 128; dAddr++) {
            sprintf(tmpStr, "%04X", in16bits(dAddr));
            Serial.print(tmpStr);
          }
          Serial.println();
          
          break; 
        }
        if (millis() - start > 1000) {
          return ERROR_TIMEOUT;
        }
      }
    }
  }

#ifdef INCLUDE_METRICS
  metrics.bufferWrite += micros() - start;
#endif

  return 0;
}

void outAddressAndData(long addr24bit, int data16bit) {
  outAddressAndData(addr24bit, data16bit, false);
}

void outAddressAndData(long addr24bit, int data16bit, bool flipBytes) {
  // disable buf addr & data OE
  controlOut(F_SRA_OE | F_SRD_OE, true, true);

  if (flipBytes) {
    data16bit = flipIntBytes(data16bit);
  }

  shiftOutAddressAndData(addr24bit, data16bit);
 
  // enble buf addr & data OE
  controlOut(F_SRA_OE | F_SRD_OE, false, true);
}

void outAddress(long addr24bit) {
  // disable addr & data OE
//  digitalWrite(F_SRA_OE, HIGH);
//  digitalWrite(F_SRD_OE, HIGH);
  controlOut(F_SRA_OE | F_SRD_OE, true, true);

  delayMicroseconds(1);

  shiftOutAddressAndData(addr24bit, 0);

  // enable data OE
  controlOut(F_SRA_OE, false, true);

  delayMicroseconds(1);
}

void shiftOutAddressAndData(long addr24bit, int data16bit) {

  // use SPI to shift in data
  byte addr1 = (addr24bit >> 16) & 0xff;
  byte addr2 = (addr24bit >> 8) & 0xff;
  byte addr3 = addr24bit & 0xff;
  byte data1 = (data16bit >> 8) & 0xff;
  byte data2 = data16bit;

#ifdef DEBUG_BUILD  
  if (debugMode && trace) {
    Serial.print(F("DEBUG:: Addr chunks: 0x"));
    Serial.print(addr1, HEX);
    Serial.print(addr2, HEX);
    Serial.print(addr3, HEX);
    Serial.print(F(" full:: "));Serial.print(addr24bit, HEX);
    Serial.println();    

    Serial.print(F("DEBUG:: Data chunks: 0x"));
    Serial.print(data1, HEX);
    Serial.print(data2, HEX);
    Serial.print(F(" full:: "));Serial.print(data16bit, HEX);
    Serial.println();    
  }
#endif

  // new board had 595s wired other way around so data bytes have to be swapped
  byte buf[5] = {data1, data2, addr1, addr2, addr3};
  
  selectSPIChannel(ROM_CH);
  SPI.begin();
  SPI.transfer(buf, 5);
  SPI.end();

  PORTD |= 1 << SR595_LD;
  PORTD &= ~(1 << SR595_LD);

}

unsigned int in16bits(long addr24bit) {
  // enable output on ROM
  controlOut(F_SRD_OE, true, true);
  
  outAddress(addr24bit);

  digitalWrite(PIN_ROM_OE, LOW);
  digitalWrite(PIN_ROM_CE, LOW);

  controlOut(F_SL165_LD, false, true);
  controlOut(F_SL165_LD, true, true);

  selectSPIChannel(ROM_CH);
  SPI.begin();
  unsigned int inValue = SPI.transfer16(0);
  SPI.end();

  unsigned int outValue = ((inValue & 0xff) << 8) + (inValue >> 8);
  return outValue & 0xffff;
}

// other utils
const unsigned char CRC7_POLY = 0x91;
unsigned char calc_crc(unsigned char message[], unsigned int len, unsigned char crc) {
  unsigned int i, j;
//  if (debugMode) {
//    Serial.print("DEBUG: calculating crc for buffer of size: ");Serial.println(len);
//  }
 
  for (i = 0; i < len; i++)
  {
    crc ^= message[i];
    for (j = 0; j < 8; j++)
    {
      if (crc & 1)
        crc ^= CRC7_POLY;
      crc >>= 1;
    }
    if (debugMode && (i % 4) == 0) {
//      Serial.print("DEBUG: ");Serial.print(i);Serial.print(": ");Serial.print(message[i], HEX); Serial.print(" "); Serial.println(crc);
    }
    
  }
  return crc;
}

unsigned char calc_crc(unsigned char message[], unsigned int len) {
  return calc_crc(message, len, 0);
}

void handleGenSignal(unsigned int interruptId) {
  

  // halve the address as we are reading words not bytes
  long addr = 0xd000 / 2;
  // long addr = 0xd000;

  unsigned int intCode = in16bits(addr);
  Serial.print("Got sega request: "); Serial.print(intCode, HEX);
  Serial.print(", interruptId: ");Serial.println(interruptId, HEX);

  Serial.print("Data at interop: ");
  for (int i = 0; i < 16; i++) {
    unsigned int val = in16bits(addr+ i);
    Serial.print(val, HEX);
  }
  Serial.println();


  // char inString[64];
  // int index = 0;
  // Serial.print("Read words: 0x");
  // while (true) {
  //   addr++;
  //   int inWord = in16bits(addr);
  //   Serial.print(inWord, HEX);
  //   inString[index++] = inWord >> 8;
  //   inString[index++] = inWord & 0xff;
  //   // look for null terminator of the string
  //   if (inWord & 0xff == 0 || inWord >> 8 == 0) break;
  // }
  // Serial.println();
  // Serial.print("Input string: ");Serial.println(inString);


  if (intCode == 0x0001) {
    unsigned int offset = in16bits(addr+1); 
    unsigned int count = in16bits(addr+2); 
    Serial.print("Listing files... from: ");
    Serial.print(offset);
    Serial.print(", numfiles: ");
    Serial.println(count);
    // write sd filenames to genesis
    selectSPIChannel(SD_CH);
    bool sdOpen = SD.begin(SD_CS_PIN);
    File root = SD.open("/");
    addr += 1;
    
    char strEntry[64];
    // memset(strEntry, 0, 64);
    // strEntry[0] = 1;
    // strEntry[1] = 1;
    // sprintf(strEntry + 2, "Files:");
    // sprintf(strEntry + 14, "[additional]");
    // writeBuffer(addr, (int *)strEntry, 32, true, true);
    // addr += 64;

    if (!sdOpen) {
      memset(strEntry, 0, 64);
      strEntry[0] = 1;
      strEntry[1] = 0;
      sprintf(strEntry + 2, "SD Failed");
      sprintf(strEntry + 14, "[additional]");
      writeBuffer(addr, (int *)strEntry, 32, true, true);
      addr += 64;
      Serial.println("Failed to open SD Card");
    }
    Serial.println("Adding files...");

    while (true) {
      selectSPIChannel(SD_CH);

      File entry =  root.openNextFile();
      // filter out Mac OS & hidden files
      String fnString = String(entry.name());
      if (fnString.startsWith("FSEVEN~1") ||
        fnString.startsWith("METADA~1") ||
        fnString.startsWith("TRASHE~1") ||
        fnString.startsWith("_BOOT")) {
        entry.close();
        continue;
      }

      if (!entry) {
        memset(strEntry, 0, 64);
        strEntry[0] = 1;
        strEntry[1] = 0;
        break;
      }

      if (offset > 0) {
        entry.close();
        offset--;
        continue;
      }

      // each entry header is 2 bytes / 1 word (easier to deal with words on 16bit system)
      // file name in 8.3 format (12 chars + null terminator / 6 words)
      // genesis game name (48 chars / 24 words)
      // padding word to aling with 32 words / 64 bytes per entry

      // if (entry.isDirectory()) {
        memset(strEntry, 0, 64);
        strEntry[0] = 1;
        strEntry[1] = entry.isDirectory() ? 2 : 3;
        memcpy(strEntry + 2, entry.name(), 12);
        strEntry[14] = 0;

        if (String(entry.name()).indexOf(".MD") > 0) {
          entry.seek(0x150);
          entry.read(strEntry + 15, 48);
        } else {
          memset(strEntry + 15, 0, 48);
        }
        // ensure there is null terminator for extended name
        strEntry[63] = 0;
        writeBuffer(addr, (int *)strEntry, 32, true, true);

        Serial.print("Added file: ");Serial.println(entry.name());


      addr += 32;

      // }
      entry.close();

      if (--count == 0) {
        break;
      }

    }
    root.close();



    // indicate end
    strEntry[0] = 1;
    strEntry[1] = 0;
    // memcpy(strEntry + 2, entry.name(), 12);
    memset(strEntry + 2, 0, 62);
    writeBuffer(addr, (int *)strEntry, 32, true, true);

  } else if (intCode == 0x0002) {
    // read filename & flash it
    int index = 0;
    Serial.print("Read words: 0x");
    while (true) {
      addr++;
      int inWord = in16bits(addr);
      Serial.print(inWord, HEX);
      g_tempReusableBuf[index++] = inWord >> 8;
      g_tempReusableBuf[index++] = inWord & 0xff;
      // look for null terminator of the string
      if (inWord & 0xff == 0 || inWord >> 8 == 0) break;
    }
    Serial.println();

    Serial.print("Request to flash SD file: ");Serial.println(g_tempReusableBuf);
    pendingFlash = true;

    // flashAndStartROM(filename);
  } else {
    char strOut[64];
    sprintf(strOut, "FrmArd: 0x%x", intCode + 1);
    // writeBuffer(addr + 1, (int *)strOut, sizeof(strOut) / 2, true, true);
  }

}
