#include <SPI.h>
#include <SD.h>

const unsigned long baudRate = 500000;
//const unsigned long baudRate = 115200;
//const unsigned long baudRate = 19200;

bool debugMode = true;
bool trace = false;

bool ValidateAddress = false;

int DeviceID = 0;

int F_SRA_OE = 1,
  F_GEN_RES = 1 << 1,
  F_ROM_WE = 1 << 2,  
  F_ROM_CE = 1 << 3,
  F_GEN_TX_EN = 1 << 4,
  F_SRD_OE = 1 << 5,
  F_ROM_OE = 1 << 6,
  F_SL165_LD = 1 << 7;

int ROM_CH = 0,
  CNTR_CH = 1,
  SD_CH = 2;

// Direct Control
int SPI_ADDR0 = 2;
int SPI_ADDR1 = 3;
int CNTR_SR595_LATCH = 4;
int CNTR_SR595_OE = 5;

typedef struct {
  unsigned long control;
  unsigned long bufferWrite;
  unsigned long bytesReceive;
  unsigned long action;
  unsigned long parseBytes;
  unsigned long copyIncomingBuf;
} Metrics;
Metrics metrics;


// line buffer control
int TEST_ENABLE = A5; // not used for now
int SR595_LD = 6;

void selectSPIChannel(int channel) {
  PORTD = (channel & 0b01) ? PORTD | (1 << SPI_ADDR0) : PORTD &~ (1 << SPI_ADDR0);
  PORTD = (channel & 0b10) ? PORTD | (1 << SPI_ADDR1) : PORTD &~ (1 << SPI_ADDR1);
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
  unsigned long start = micros();

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

  metrics.control += micros() - start;
}


//Serial Actions
enum {
//- Ping (availability)
  PING = 0,
//- Get EEPROM ID
  READ_ROM_ID,
//- Get EEPROM Name (hardcoded based on software ID)
  READ_ROM_NAME,
//- Get EEPROM Size (hardcoded based on software ID)
  READ_ROM_SIZE,
//- Read 128 word(16bit) page
  READ_128_PAGE,
//- Get 128 word(16bit) Page md5 Checksum 
  READ_128_PAGE_CRC,
  READ_128x128_PAGES,
//- Write 128 word(16bit) page (with data# polling)
  WRITE_128_PAGE,
  WRITE_128X_PAGES,
  WRITE_128X_PAGES_HEX,
//- Busy? (page write in progress)
  IS_DEVICE_BUSY,
  DEBUG_ON,
  DEBUG_OFF,
  WRITE_WORD,
  LOCK_ADDRESS,
  LOCK_ADDRESS_AND_DATA,
  ENABLE_WRITE_PROTECT,
  DISABLE_WRITE_PROTECT,
// SEGA GENESIS control
  GENESIS_RESET,
  GENESIS_RESET_HOLD,
  GENESIS_RESET_RELEASE,
// TEST access to cartridge
  CART_LOCK_ADDRESS,
  CART_READ_128_PAGE,
  CART_READ_128x128_PAGES,
  LIST_SD_FILES,
  FLASH_SD_ROM,
// no idea what is being asked
  UNKNOWN_ACTION
};

bool enabled = false;

void enableControl() {
  if (!enabled) {
    enabled = true;
    digitalWrite(SR595_LD, LOW);

    controlOut(0xff &~ F_SL165_LD, true, true);

    digitalWrite(TEST_ENABLE, HIGH);

    memset(&metrics, 0, sizeof(metrics));

    delay(100);
  }
}

void disableControl() {
//  digitalWrite(SR595_LD, LOW);
//  controlOut(0xff, true, true);
  digitalWrite(SR595_LD, LOW);
  controlOut(0xff, true, false);
  controlOut(F_GEN_TX_EN, false, true);

  
  enabled = false;
  delay(50);
//  delay(1000);
}

void readDummyToResetRom() {
  for (int i = 0; i < 256; i++) {
    in16bits(i);
  }
}


void setup() {

  Serial.begin(baudRate);
  Serial.println();

  pinMode(SPI_ADDR0, OUTPUT);
  pinMode(SPI_ADDR1, OUTPUT);

  digitalWrite(CNTR_SR595_LATCH, LOW);
  pinMode(CNTR_SR595_LATCH, OUTPUT);

  digitalWrite(CNTR_SR595_OE, LOW);
  pinMode(CNTR_SR595_OE, OUTPUT);

  digitalWrite(SR595_LD, LOW);
  pinMode(SR595_LD, OUTPUT);

  controlOut(0xff, true, true);
  
  // rom setup so we do not get weird data
//  digitalWrite(F_GEN_RES, HIGH);
  controlOut(F_GEN_RES, true, true);

  digitalWrite(TEST_ENABLE, HIGH);
  pinMode(TEST_ENABLE, OUTPUT);

//  Serial.begin(57600);

  delay(100);

  disableControl();

  delay(1);

  enableControl();

  Serial.print(F("Available actions: 0.."));Serial.println(UNKNOWN_ACTION - 1);
//  Serial.print("EEPROM: ");
//  outputDeviceName();
  DeviceID = readDeviceId();

  Serial.println(F("READY"));

  disableControl();

  genesisReset();
}

int parseAction(String stringAction) {
  if (stringAction.equals(F("PING"))) return PING;
  if (stringAction.equals(F("READ_ROM_ID"))) return READ_ROM_ID;
  if (stringAction.equals(F("READ_ROM_NAME"))) return READ_ROM_NAME;
  if (stringAction.equals(F("READ_ROM_SIZE"))) return READ_ROM_SIZE;
  if (stringAction.equals(F("READ_128_PAGE"))) return READ_128_PAGE;
  if (stringAction.equals(F("READ_128_PAGE_CRC"))) return READ_128_PAGE_CRC;
  if (stringAction.equals(F("READ_128x128_PAGES"))) return READ_128x128_PAGES;
  if (stringAction.equals(F("WRITE_128_PAGE"))) return WRITE_128_PAGE;
  if (stringAction.equals(F("WRITE_128X_PAGES"))) return WRITE_128X_PAGES;
  if (stringAction.equals(F("WRITE_128X_PAGES_HEX"))) return WRITE_128X_PAGES_HEX;
 
  if (stringAction.equals(F("IS_ROM_BUSY"))) return IS_DEVICE_BUSY;
  if (stringAction.equals(F("DEBUG_ON"))) return DEBUG_ON;
  if (stringAction.equals(F("DEBUG_OFF"))) return DEBUG_OFF;
  if (stringAction.equals(F("WRITE_WORD"))) return WRITE_WORD;
  if (stringAction.equals(F("LOCK_ADDRESS"))) return LOCK_ADDRESS;
  if (stringAction.equals(F("LOCK_ADDRESS_AND_DATA"))) return LOCK_ADDRESS_AND_DATA;
  if (stringAction.equals(F("ENABLE_WRITE_PROTECT"))) return ENABLE_WRITE_PROTECT;
  if (stringAction.equals(F("DISABLE_WRITE_PROTECT"))) return DISABLE_WRITE_PROTECT;
  
  // genesis control
  if (stringAction.equals(F("GENESIS_RESET"))) return GENESIS_RESET;
  if (stringAction.equals(F("GENESIS_RESET_HOLD"))) return GENESIS_RESET_HOLD;
  if (stringAction.equals(F("GENESIS_RESET_RELEASE"))) return GENESIS_RESET_RELEASE;

  // cart test
  if (stringAction.equals(F("CART_LOCK_ADDRESS"))) return CART_LOCK_ADDRESS;
  if (stringAction.equals(F("CART_READ_128_PAGE"))) return CART_READ_128_PAGE;
  if (stringAction.equals(F("CART_READ_128x128_PAGES"))) return CART_READ_128x128_PAGES;
  if (stringAction.equals(F("LIST_SD_FILES"))) return LIST_SD_FILES;
  if (stringAction.equals(F("FLASH_SD_ROM"))) return FLASH_SD_ROM;
  

  return UNKNOWN_ACTION;
}

void loop() {

  if (Serial.available() > 0) {

    String input = Serial.readStringUntil('\n');
    if (debugMode) {
      Serial.print(F("DEBUG: parsing raw input: "));Serial.println(input);
    }
    int action = parseAction(input);

    if (debugMode) {
      Serial.print(F("DEBUG: "));Serial.print(input);Serial.print(F("; decoded:"));Serial.println(action);
    }

    switch (action) {
      case PING: Serial.println(F("PONG")); break;
      case READ_ROM_ID: enableControl(); outputDeviceId(); disableControl(); break;
      case READ_ROM_NAME: enableControl(); outputDeviceName(); disableControl(); break;
      case READ_ROM_SIZE: enableControl(); outputDeviceSize(); disableControl(); break;
      case READ_128_PAGE: enableControl(); read128WordPage(false); disableControl(); break;
      case READ_128_PAGE_CRC: enableControl(); read128WordPageCRC(); disableControl(); break;
      case READ_128x128_PAGES: enableControl(); read128x128WordPages(false); disableControl(); break;
      case WRITE_128_PAGE: enableControl(); write128WordPage(); disableControl(); break;
      case WRITE_128X_PAGES: enableControl(); write128WordXPages(); disableControl(); break;
      case WRITE_128X_PAGES_HEX: enableControl(); write128WordXPages(true); disableControl(); break;
      case DEBUG_ON: debugMode = true; Serial.println("DEBUG mode ON!"); break;
      case DEBUG_OFF: debugMode = false; Serial.println("DEBUG mode OFF!"); break;

      case WRITE_WORD: enableControl(); writeDataAtAddress(); break;
      case LOCK_ADDRESS: enableControl(); lockAddress(false); break;
      case LOCK_ADDRESS_AND_DATA: enableControl(); lockAddressAndData(); break;
      case ENABLE_WRITE_PROTECT: writeProtect(true); break;
      case DISABLE_WRITE_PROTECT: writeProtect(false); break;
      // Sega Genesis
      case GENESIS_RESET: genesisReset(); break;
      case GENESIS_RESET_HOLD: genesisResetHold(); break;
      case GENESIS_RESET_RELEASE: genesisResetRelease(); break;

      // read via cart access
//      case CART_LOCK_ADDRESS: disableControl(); lockAddress(true); break;
      case CART_READ_128_PAGE: read128WordPage(true); break;
      case CART_READ_128x128_PAGES: read128x128WordPages(true); break;

      case LIST_SD_FILES: listSDFiles(); break;
      case FLASH_SD_ROM: enableControl(); flashAndStartROM(); disableControl(); genesisReset(); break;
      
      // Invalid action
      case UNKNOWN_ACTION: Serial.println(F("UNKNOWN_ACTION")); break;
      default: break;
    }
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
  if (debugMode) {
    Serial.println(F("DEBUG: reset->low"));
  }
  controlOut(F_GEN_RES, false, true);
  delay(100);
  if (debugMode) {
    Serial.println(F("DEBUG: reset->high"));
  }
  controlOut(F_ROM_CE, false, false);
  controlOut(F_GEN_RES, true, true);

  Serial.println(F("ACK_GENESIS"));
}

void genesisResetHold() {
  if (debugMode) {
    Serial.println(F("DEBUG: RESET_HOLD"));
  }
  controlOut(F_GEN_RES, false, true);
  Serial.println(F("ACK_GENESIS"));
}


void genesisResetRelease() {
  if (debugMode) {
    Serial.println(F("DEBUG: RESET_RELEASE"));
  }
  controlOut(F_GEN_RES, true, true);
  Serial.println(F("ACK_GENESIS"));
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
  SD.begin();
  File root = SD.open("/");
  printDirectory(root);                             // list files on SD1
  root.close();
  Serial.println("ACK");
}

bool printDirectory(File dir)               // lists the files and filesize on the SD card (only root)
{
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      return false;
    }
    if (String(entry.name()).indexOf(".MD") > 0) {
      char romName[32];
      entry.seek(0x150);
      entry.read(romName, 32);
      
      Serial.print(romName);
      Serial.print(F("\t"));
      Serial.println(entry.size(), DEC);
    }

    entry.close();
  }
  
  return true;
}

void flashAndStartROM() {
  
  int error;
  unsigned long targetIndex = loadAddress(error, false, false);

  if (error) {
    debugErrorOut(error);
    Serial.print(F("ERR_NO_ADDR "));Serial.println(error, HEX);
    return;
  }

  Serial.print("DEBUG: Target index: ");Serial.println(targetIndex);
  
  selectSPIChannel(SD_CH);
  SD.begin();
  File root = SD.open("/");
  int currentIndex = 0;
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    if (targetIndex != currentIndex++) continue;

    char romName[32];
    entry.seek(0x150);
    entry.read(romName, 32);

    Serial.print("DEBUG: Flashing ROM: "); Serial.print(entry.name()); Serial.println(romName);

    int pageSize = 128;
    int pages = entry.size() / pageSize / 2;
    int buf[pageSize];
    for (int i = 0 ; i < pages; i++) {
      selectSPIChannel(SD_CH);
      unsigned long address = i * pageSize;
      entry.seek(i * pageSize * 2);
      entry.read(buf, pageSize * 2);

      Serial.print("Writing page: ");Serial.print(i);Serial.print(", at addr: ");Serial.println(address, HEX);
      writeBuffer(address, buf, pageSize, true, true);
    }

    // read page by page and write to file
//    int writeBuffer(long addr24bit, int *buffer, int bufferSize, bool waitSuccess)


    break;
  }

  root.close();
  Serial.println("ACK");
}

// ********************************
// ********************************
// ********************************
// EEPROM
// ********************************
// ********************************
// ********************************

int readDeviceId() {
  // SRAM does not have software id entry and causes data corruption if attempted
  return 0;
  
  Serial.println(F("DEBUG: Reading device ID"));
  // sofwware id entry
  writeWordBoth(0x5555, 0xAAAA);
  writeWordBoth(0x2AAA, 0x5555);
  writeWordBoth(0x5555, 0x9090);
  delay(10);


  int rawVendor = in16bits(0);
  int rawDevice = in16bits(1);

  Serial.print(F("DEBUG: RAW vendor:"));Serial.print(rawVendor, HEX); Serial.print(F(", device: "));Serial.println(rawDevice, HEX); 

  byte vendorId = rawVendor;
  byte deviceId = rawDevice;


  // sofwware id exit
  writeWordBoth(0x5555, 0xAA);
  writeWordBoth(0x2AAA, 0x55);
  writeWordBoth(0x5555, 0xF0);

  Serial.print(F("Vendor: "));Serial.print(vendorId, HEX);Serial.print(F(", deviceId: "));Serial.println(deviceId, HEX);

  return vendorId << 8 | deviceId;
}

void writeProtect(bool enable) {
  if (!enable) {
    writeWordBoth(0x5555, 0xAA);
    writeWordBoth(0x2AAA, 0x55);
    writeWordBoth(0x5555, 0x80);
    writeWordBoth(0x5555, 0xAA);
    writeWordBoth(0x2AAA, 0x55);
    writeWordBoth(0x5555, 0x20);
    delay(10);
    Serial.println("WRITE_PROTECT_DISABLED");
  }
}

void outputDeviceId() {
  DeviceID = readDeviceId();
  byte vendorId = DeviceID >> 8;
  byte chipId = DeviceID & 0xff;
  Serial.print(vendorId, HEX);
  Serial.println(chipId, HEX);
}

void outputDeviceName() {
  DeviceID = readDeviceId();
  byte vendorId = DeviceID >> 8;
  byte chipId = DeviceID & 0xff;
  if (vendorId == 0xbf && chipId == 0x07) {
    Serial.println("SST29EE010");
  } else if (vendorId == 0x1f && chipId == 0x13) {
    Serial.println("AT49F040");
  } else {
    Serial.println("UNKNOWN");
  }
}

long getRomSize(int deviceId) {
  byte vendorId = deviceId >> 8;
  byte chipId = deviceId & 0xff;
  if (debugMode) {
    Serial.print(F("DEBUG: getRomSize:: vendorId:"));Serial.print(vendorId, HEX);Serial.print(F(";chipId:"));Serial.println(chipId, HEX);
  }
  long romSize = 0;
  if (vendorId == 0xbf && chipId == 0x07) {
    if (debugMode) {
      Serial.println(F("DEBUG: Found EEPROM SST SST29EE010"));
    }
    romSize = 262144;
  } else if (vendorId == 0x1f && chipId == 0x13) {
    if (debugMode) {
      Serial.println(F("DEBUG: Found EEPROM Atmel AT49F040"));
    }
    romSize = 262144 * 8;
  }
  if (debugMode) {
    Serial.print(F("DEBUG: romSize:"));Serial.println(romSize, HEX);
  }
  return romSize;
}

void outputDeviceSize() {
  long romSize = getRomSize(DeviceID);
  Serial.println(romSize, HEX);
}

enum {
  SUCCESS = 0,
  ERROR_PARSE = 1,
  ERROR_TIMEOUT = 2,
  ERROR_ADDR_RANGE = 3
  
};

void debugErrorOut(int error) {
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
}

unsigned long loadAddress(int &error, bool alignPageBoundary, bool validateChipSize) {
  validateChipSize = ValidateAddress ? validateChipSize : false;
  long romSize = getRomSize(DeviceID);
  
  Serial.println(F("AWAIT_ADDR_HEX"));
  error = SUCCESS;

  // wait for 3 secs for address or bail with an error
  long start = millis();
  while (Serial.available() == 0) {
    if (millis() - start > 13000) {
      error = ERROR_TIMEOUT;
      Serial.println(F("DEBUG: loadAddress TIMEOUT, bailing"));
      return;
    }
  }

  Serial.println(F("DEBUG: Parsing serial input"));

  // read in 24 bit value
  char input[8];
  memset(input, 0, 8);
  Serial.readBytesUntil('\n', input, 7);
  // parse hex address that should be lower than chip size or it will be invalid address
  char *endChar = 0;
  unsigned long address = strtol(input, &endChar, 16);
  const char *str = input;
  int charsParsed = 0;
  while (*str && *str != *endChar) {
    ++charsParsed;
    ++str;
  }
  if (debugMode) {
    Serial.print(F("DEBUG: chars parsed:"));Serial.println(charsParsed);
  }

  // sink the rest of the stream until newline
  if (Serial.available() > 0) {
    Serial.readStringUntil('\n');
  }
  
  if (charsParsed == 0) {
    error = ERROR_PARSE;
    return 0;
  }

  if (debugMode) {
    Serial.print(F("DEBUG: Got address: 0x"));Serial.println(address, HEX);
  }

  if (validateChipSize && address * 2 >= romSize) {
    Serial.println(F("DEBUG: Chip size validation failed"));
    error = ERROR_ADDR_RANGE;
    return 0;
  }

  Serial.println("ACK_ADDR");

  if (debugMode) {
    Serial.print(F("DEBUG: Returning address: 0x"));Serial.println(address, HEX);
  }

  if (alignPageBoundary) {
    // figure out the page boundary if 
    unsigned long pageStartAddress = address - address % 128;
    if (debugMode) {
      Serial.print(F("DEBUG: Page start address: 0x"));Serial.println(pageStartAddress, HEX);
    }
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
  unsigned long metrics_start = micros();
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
  if (debugMode) {
    Serial.print(F("DEBUG: bytesRead: "));Serial.println(bytesRead);
  }

//  while (wordsRead < totalReadWords) {
//      int bytesRead = Serial.readBytes(input, 4);

  if (debugMode) {
    char tmpStr[5];

    Serial.print(F("Debug: ACK_DATA: "));
    for (unsigned long i = 0; i < numWords; i++) {
      sprintf(tmpStr, "%04X", buf[i]);
      Serial.print(tmpStr);
    }
    Serial.println();
  }

  metrics.bytesReceive += micros() - metrics_start;

  Serial.println(F("ACK_DATA"));
  
  return SUCCESS;
}

int loadDataHex(unsigned int *buf, unsigned int bufSize) {
  unsigned long metrics_start = micros();
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
      if (debugMode) {
        Serial.print(F("DEBUG: bytesRead: "));Serial.print(bytesRead);Serial.print(F(", expected: 4"));
        Serial.print(F(", wordsRead:"));Serial.print(wordsRead);Serial.print(F(", expected:"));Serial.println(totalReadWords);
      }
      return ERROR_TIMEOUT;
    }

    unsigned long parseb_start = micros();
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
    metrics.parseBytes += micros() - parseb_start;
  }
  if (debugMode) {
    Serial.println(F("DEBUG: about to ack data"));
  }
  Serial.println(F("ACK_DATA"));

  unsigned long copy_start = micros();
  memcpy(buf, otherBuf, bufSize * sizeof(int));
  metrics.copyIncomingBuf += micros() - copy_start;

  if (debugMode) {
    char tmpStr[5];

    Serial.print(F("Debug: ACK_DATA: "));
    for (unsigned long i = 0; i < bufSize; i++) {
      sprintf(tmpStr, "%04X", buf[i]);
      Serial.print(tmpStr);
    }
    Serial.println();
  }

  metrics.bytesReceive += micros() - metrics_start;

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
  if (debugMode) {
    Serial.print(F("DEBUG: chars parsed:"));Serial.println(charsParsed);
  }

  // sink the rest of the stream until newline
  if (Serial.available() > 0) {
    Serial.readStringUntil('\n');
  }
  
  if (charsParsed == 0) {
    error = ERROR_PARSE;
    return 0;
  }

  if (debugMode) {
    Serial.print(F("DEBUG: Got pages: 0x"));Serial.println(pages, HEX);
  }
  
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
  if (debugMode) {
    char tmpStr[5];
    Serial.print("DEBUG: device:"); Serial.print(address, HEX); Serial.print(" bytes: "); 
    // output 128 words (we are using 2 chips each containing a byte) for the page
    for (unsigned long i = 0; i < bytesToRead; i++) {
      unsigned int val = cartRead ? in16bitsCart(address + i) : in16bits(address + i);
      sprintf(tmpStr, "%04X", val);
      Serial.print(tmpStr);
    }
    Serial.println();
  }

  Serial.println("DATA_BEGIN");
  char tmpStr[5];
  // output 128 words (we are using 2 chips each containing a byte) for the page
  for (unsigned long i = 0; i < bytesToRead; i++) {
    unsigned int val = cartRead ? in16bitsCart(address + i) : in16bits(address + i);
    sprintf(tmpStr, "%04X", val);
    Serial.print(tmpStr);
  }
  Serial.println();
}

void lockAddress(bool cart) {
  int error;
  unsigned long address = loadAddress(error, false, false);

  if (error) {
    debugErrorOut(error);
    Serial.print(F("ERR_NO_ADDR "));Serial.println(error, HEX);
    return;
  }


  if (debugMode) {
    Serial.print(F("DEBUG: Locking at: ")); Serial.println(address, HEX);
  }

//  digitalWrite(TEST_ENABLE, LOW);
  
  outAddress(address);

  if (cart) {
    controlOut(F_GEN_TX_EN, LOW, true);
    digitalWrite(TEST_ENABLE, LOW);
    if (debugMode) {
      Serial.println("DEEBUG: pulling bus enable low");
    }
  } else {
    controlOut(F_ROM_CE|F_ROM_OE, LOW, true);
  }

  if (debugMode) {
    // latch & read
    controlOut(F_SL165_LD, LOW, true);
    controlOut(F_SL165_LD, HIGH, true);

    selectSPIChannel(ROM_CH);
    SPI.begin();
    unsigned int inValue = SPI.transfer16(0);
    SPI.end();

    inValue = ((inValue & 0xff) << 8) + (inValue >> 8);
    Serial.print(F("DEBUG: Locked at: ")); Serial.print(address, HEX); Serial.print(F(", data: "));
    char tmpStr[5];
    sprintf(tmpStr, "%04X", inValue);
    Serial.println(tmpStr);
  }

//  digitalWrite(TEST_ENABLE, HIGH);

  Serial.println(F("ADDR_LOCKED"));
}

void writeDataAtAddress() {
  int error;
  unsigned long address = loadAddress(error, false, false);
  if (error) {
    debugErrorOut(error);
    Serial.print(F("ERR_NO_ADDR "));Serial.println(error, HEX);
    return;
  }

  unsigned int data;
  error = loadData(&data, 1);
  if (error) {
    debugErrorOut(error);
    Serial.print(F("ERR_NO_DATA "));Serial.println(error, HEX);
    return;
  }

//  outAddressAndData(address, data);
  writeWord(address, data);
//  writeValueTest(address, data);

  outAddress(address);
  controlOut(F_ROM_CE | F_ROM_OE, LOW, true);

  controlOut(F_SL165_LD, LOW, true);
  controlOut(F_SL165_LD, HIGH, true);

  selectSPIChannel(ROM_CH);
  SPI.begin();
  unsigned int inValue = SPI.transfer16(0);
  SPI.end();

  inValue = ((inValue & 0xff) << 8) + (inValue >> 8);
  Serial.print(F("DEBUG: Written at: ")); Serial.print(address, HEX); Serial.print(F(", data: "));
  char tmpStr[5];
  sprintf(tmpStr, "%04X", inValue);
  Serial.println(tmpStr);

  Serial.println(F("WRITE_WORD_ACK"));  
}

void lockAddressAndData() {
  int error;
  unsigned long address = loadAddress(error, false, false);
  if (error) {
    debugErrorOut(error);
    Serial.print(F("ERR_NO_ADDR "));Serial.println(error, HEX);
    return;
  }

  unsigned int data;
  error = loadData(&data, 1);
  if (error) {
    debugErrorOut(error);
    Serial.print(F("ERR_NO_DATA "));Serial.println(error, HEX);
    return;
  }

  if (debugMode) {
    Serial.print(F("DEBUG: Locking at: "));Serial.print(address, HEX);Serial.print(F(", data: "));  
//    unsigned int val = in16bits(address);
    char tmpStr[5];
    sprintf(tmpStr, "%04X", data);
    Serial.println(tmpStr);
  }

  // disable OE on chip
  controlOut(F_ROM_CE | F_ROM_CE, HIGH, true);

  outAddressAndData(address, data);

  // read back throught 165
  controlOut(F_SL165_LD, LOW, true);
  controlOut(F_SL165_LD, HIGH, true);

  selectSPIChannel(ROM_CH);
  SPI.begin();
  unsigned int inValue = SPI.transfer16(0);
  SPI.end();

  if (debugMode) {
    Serial.print(F("DEBUG: data as seen by 165: "));
    char tmpStr[5];
    sprintf(tmpStr, "%04X", inValue);
    Serial.println(tmpStr);
  }


  Serial.println(F("ADDR_AND_DATA_LOCKED"));
}


void read128x128WordPages(bool cartRead) {
  int error;
  
  unsigned long address = loadAddress(error, false, !cartRead);
  
  Serial.print(F("DEBUG: Returned address: 0x"));Serial.println(address, HEX);
  if (debugMode) {
    Serial.print(F("DEBUG: real address: 0x"));Serial.print(address, HEX);Serial.print(F(", clamped address: 0x"));Serial.println(address - address % 128, HEX);
  }
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
       unsigned int val = cartRead ? in16bitsCart(address + i) : in16bits(address + i);
      sprintf(tmpStr, "%04X", val);
      Serial.print(tmpStr);
    }
    Serial.println();
  }
  
  Serial.println(F("DATA_BEGIN"));

  unsigned int data[130] = {0};

  for (unsigned long page = 0; page < pagesToLoad; page++) {
    unsigned long pageOffset = page * wordsPerPage;
    unsigned char page_crc = 0;
    for (unsigned long i = 0; i < wordsPerPage; i++) {
      unsigned long read_at_address = address + pageOffset + i;
      unsigned int val = cartRead ? in16bitsCart(read_at_address) : in16bits(read_at_address);
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

  if (debugMode) {
    Serial.println(F("DEBUG: Reading page..."));
  }
  unsigned int data[128] = {0};
  for (unsigned int i = 0; i < 128; i++) {
    data[i] = in16bits(address + i);
  }

  if (debugMode) {
    Serial.println(F("DEBUG: calc CRC..."));
  }

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
    if (debugMode) {
      char tmpStr[5];
      Serial.print(F("DEBUG: Received page: "));
      for (unsigned int i = 0; i < 128; i++) {
        sprintf(tmpStr, "%04X", data[i]);
        Serial.print(tmpStr);
      }
      Serial.println();
    }
    Serial.print(F("ERR_NO_DATA "));Serial.println(error, HEX);
    return;
  }

  writeBuffer(address, data, 128, true);

  Serial.println(F("ACK_DATA_WRITE"));

  unsigned char calculated_crc = calc_crc((unsigned char *)data, 128 * sizeof(int));
  Serial.println(calculated_crc, HEX);
  
}


void write128WordXPages(bool hex) {

  unsigned long metrics_start = micros();
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
      if (debugMode) {
        char tmpStr[5];
        Serial.print(F("DEBUG: Received page: "));
        for (unsigned int i = 0; i < 128 * num128Pages; i++) {
          sprintf(tmpStr, "%04X", data[i]);
          Serial.print(tmpStr);
        }
        Serial.println();
      }
      Serial.print(F("ERR_NO_DATA "));Serial.println(error, HEX);
      return;
    }
    unsigned long page_address = address + page * 128;
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
    int error = writeBuffer(page_address, data, 128 * num128Pages, true, true);
    if (error) {
      debugErrorOut(error);
      Serial.print(F("ERR_WRITE_ERROR "));Serial.println(error, HEX);
      return;
    }
    
    calculated_crc = calc_crc((unsigned char *)data, 128 * num128Pages * sizeof(int), calculated_crc);

    if (debugMode) {
      Serial.print(F("DEBUG: temp CRC: ")); Serial.println(calculated_crc, HEX);
    }
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
  metrics.action = micros() - metrics_start;

  Serial.print("CNTR:");Serial.print(metrics.control);Serial.print("|");
  Serial.print("RCV:");Serial.print(metrics.bytesReceive);Serial.print("|");
  Serial.print("WRT:");Serial.print(metrics.bufferWrite);Serial.print("|");
  Serial.print("PARSE:");Serial.print(metrics.parseBytes);Serial.print("|");
  Serial.print("COPY:");Serial.print(metrics.copyIncomingBuf);Serial.print("|");
  Serial.print("TOTAL:");Serial.print(metrics.action);Serial.print("|");
  Serial.println();
}

void write128WordXPages() {
  write128WordXPages(false);
}

// ***************************************************************************************************************************************
// *****************************       MEMORY ACCESSS                                           ******************************************
// ***************************************************************************************************************************************
// Interaction with Device
// internals for working with the eeprom chips

void writeWord(long addr24bit, int value) {
  writeBuffer(addr24bit, &value, 1, false);
}

void writeWordBoth(long addr24bit, int value) {
  int output = value;
  writeBuffer(addr24bit, &output, 1, false);
}

//char outText[20];
void writeValueTest(long addr24bit, int value) {
  controlOut(F_ROM_OE | F_ROM_WE, true, false);
  // enable chip
  controlOut(F_ROM_CE, false, true);

  outAddressAndData(addr24bit, value);

  // blip write
  controlOut(F_ROM_WE, false, true);
  controlOut(F_ROM_WE, true, true);
}

inline int flipIntBytes(int in) {
  return (0xff00 & in << 8) + (0xff & in >> 8);
}

int writeBuffer(long addr24bit, int *buffer, int bufferSize, bool waitSuccess) {
  writeBuffer(addr24bit, buffer, bufferSize, waitSuccess, false);
}


/*
 * 
 * void outAddressAndData(long addr24bit, int data16bit) {
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
 * 
 */


int writeBuffer(long addr24bit, int *buffer, int bufferSize, bool waitSuccess, bool flipBytes) {
  unsigned long start = micros();

  if (debugMode) {
    Serial.print("DEBUG: flipBytes:");Serial.print(flipBytes);
    Serial.println();
  }

  controlOut(F_ROM_OE | F_ROM_WE | F_SRA_OE | F_SRD_OE, true, false);
  controlOut(F_ROM_CE, false, true);

  for (unsigned long i = 0; i < bufferSize; i++) {
    unsigned long target_address = addr24bit + i;

//    outAddressAndData(target_address, buffer[i], flipBytes);
//      int data16bit = flipBytes ? flipIntBytes(buffer[i]): buffer[i];

    shiftOutAddressAndData(target_address, flipBytes ? flipIntBytes(buffer[i]) : buffer[i]);

/*
    controlOut(F_SRA_OE | F_SRD_OE, false, true);
    controlOut(F_ROM_WE, false, true);
*/
    controlOut(F_ROM_WE | F_SRA_OE | F_SRD_OE, false, true);
    controlOut(F_ROM_WE | F_SRA_OE | F_SRD_OE, true, true);

    if (true) {
      continue;
    } else
    if (waitSuccess) {
      
      unsigned long page_start_address = long(i/128) * 128;
      bool pageBreak = target_address > 0 && ((target_address + 1) % 128 == 0 || i == bufferSize - 1);
      
      if (!pageBreak) continue;
    
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

      unsigned long start = millis();
      while(true) {
        // poll 
        unsigned int last_word = buffer[i];
        unsigned int in_data = in16bits(target_address);
        if (debugMode) {
          Serial.print(F("DEBUG: in_data: ")); Serial.print(in_data, HEX);Serial.print(" from address: ");Serial.println(target_address, HEX);
          Serial.print(F("DEBUG: rom data: ")); Serial.print(in_data & 0x8080, BIN);Serial.print(" [");Serial.print(in_data, HEX);Serial.print("]");
          Serial.print(F(", buf data: ")); Serial.print(last_word & 0x8080, BIN);Serial.print(" [");Serial.print(last_word, HEX);Serial.print("]");
          Serial.print(F(", index: ")); Serial.print(target_address);
          Serial.print(F(", data match?: ")); Serial.println((in_data & 0x8080) == (last_word & 0x8080));
        }

        if (in_data == last_word) {
          Serial.println(F("DEBUG: page write complete, moving on")); 

//          controlOut(F_ROM_CE, false, true);
//          controlOut(F_ROM_CE, true, true);

          Serial.print(F("DEBUG: PAGE READ:: \n"));
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
//        delay(1);
      }
    }

//    controlOut(F_ROM_CE, true, true);
  }

  metrics.bufferWrite += micros() - start;

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

  if (debugMode && trace) {
    Serial.print("DEBUG:: Addr chunks: 0x");
    Serial.print(addr1, HEX);
    Serial.print(addr2, HEX);
    Serial.print(addr3, HEX);
    Serial.print(" full:: ");Serial.print(addr24bit, HEX);
    Serial.println();    

    Serial.print("DEBUG:: Data chunks: 0x");
    Serial.print(data1, HEX);
    Serial.print(data2, HEX);
    Serial.print(" full:: ");Serial.print(data16bit, HEX);
    Serial.println();    
  }

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

  controlOut(F_ROM_CE | F_ROM_OE, false, true);
  controlOut(F_SL165_LD, false, true);
  controlOut(F_SL165_LD, true, true);

  selectSPIChannel(ROM_CH);
  SPI.begin();
  unsigned int inValue = SPI.transfer16(0);
  SPI.end();

  unsigned int outValue = ((inValue & 0xff) << 8) + (inValue >> 8);
  return outValue;
}

unsigned int in16bitsCart(long addr24bit) {
//  digitalWrite(F_SRD_OE, HIGH); // avoid bus fighting
//  digitalWrite(F_SRA_OE, HIGH);
//  
//  digitalWrite(TEST_ENABLE, LOW);
//
//  outAddress(addr24bit);
//
//  // enable output on ROM
//  digitalWrite(F_ROM_CE, LOW);
//  digitalWrite(F_ROM_OE, LOW);
//
//  digitalWrite(F_SL165_LD, LOW);
//  digitalWrite(F_SL165_LD, HIGH);
//
//  SPI.begin();
//  unsigned int inValue = SPI.transfer16(0);
//  SPI.end();
//
//  digitalWrite(TEST_ENABLE, HIGH);
//
//  unsigned int outValue = ((inValue & 0xff) << 8) + (inValue >> 8);
//  
//  return outValue;
  return 0;
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
