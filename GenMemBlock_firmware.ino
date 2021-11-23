#include <SPI.h>

const unsigned long baudRate = 500000;
//const unsigned long baudRate = 115200;
//const unsigned long baudRate = 19200;

bool debugMode = true;
int DeviceID = 0;

// 74hc165 control
int IN_SER_PIN = 12,
  IN_LATCH_PIN = A0,
  IN_CLK_PIN = 13;

int GENESIS_RESET_PIN = A2;

// rom control
int ROM_OE = 2,
  ROM_CE = 9,
  ROM_WE = 3;

// 74hc595 output buffer control
int OUT_BUF_CLR = 7,
  OUT_BUF_SER = 11,
  OUT_BUF_CLK = 13,
  OUT_BUF_LATCH = 4,
  OUT_BUF_ADDR_OE = 5,
  OUT_BUF_DATA_OE = 6,
  OUT_ENABLE = A1;

// line buffer control
int BUS_GEN_ENABLE = A4,
  BUS_DATA_DIR = A3;

int TEST_ENABLE = A5;


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
//- Busy? (page write in progress)
  IS_DEVICE_BUSY,
  DEBUG_ON,
  DEBUG_OFF,
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
// no idea what is being asked
  UNKNOWN_ACTION
};

bool enabled = false;

void enableControl() {
  if (!enabled) {
    enabled = true;
    digitalWrite(BUS_GEN_ENABLE, HIGH);

    digitalWrite(ROM_WE, HIGH);
    digitalWrite(ROM_OE, HIGH);
    digitalWrite(ROM_CE, HIGH);
    pinMode(ROM_OE, OUTPUT);
    pinMode(ROM_CE, OUTPUT);
    pinMode(ROM_WE, OUTPUT);
  
    // input setup
    pinMode(IN_SER_PIN, INPUT);
    pinMode(IN_LATCH_PIN, OUTPUT);
    pinMode(IN_CLK_PIN, OUTPUT);
    digitalWrite(IN_LATCH_PIN, HIGH);
  
    // output setup
    digitalWrite(OUT_BUF_CLR, HIGH);
    digitalWrite(OUT_BUF_LATCH, HIGH);
    digitalWrite(OUT_BUF_ADDR_OE, HIGH);
    digitalWrite(OUT_BUF_DATA_OE, HIGH);
    digitalWrite(OUT_ENABLE, LOW);    
    pinMode(OUT_BUF_SER, OUTPUT);
    pinMode(OUT_BUF_CLR, OUTPUT);
    pinMode(OUT_BUF_CLK, OUTPUT);
    pinMode(OUT_BUF_LATCH, OUTPUT);
    pinMode(OUT_BUF_ADDR_OE, OUTPUT);
    pinMode(OUT_BUF_DATA_OE, OUTPUT);
    pinMode(OUT_ENABLE, OUTPUT);

    digitalWrite(TEST_ENABLE, HIGH);

    delay(100);
  }
}

void disableControl() {
  
  digitalWrite(OUT_ENABLE, HIGH);
  digitalWrite(OUT_BUF_ADDR_OE, HIGH);
  digitalWrite(OUT_BUF_DATA_OE, HIGH);

  digitalWrite(ROM_OE, HIGH);
  digitalWrite(ROM_WE, HIGH);
  pinMode(ROM_OE, INPUT_PULLUP);
  pinMode(ROM_WE, INPUT_PULLUP);

  // input setup
  digitalWrite(IN_SER_PIN, HIGH);
  digitalWrite(IN_LATCH_PIN, HIGH);
  digitalWrite(IN_CLK_PIN, HIGH);

  digitalWrite(ROM_CE, LOW);

  digitalWrite(BUS_GEN_ENABLE, LOW);
  enabled = false;
  delay(50);
}

void readDummyToResetRom() {
  for (int i = 0; i < 256; i++) {
    in16bits(i);
  }
}


void setup() {
  // rom setup so we do not get weird data
  digitalWrite(GENESIS_RESET_PIN, HIGH);
  pinMode(GENESIS_RESET_PIN, OUTPUT);

  digitalWrite(BUS_GEN_ENABLE, HIGH);
  digitalWrite(BUS_DATA_DIR, LOW);
  pinMode(BUS_GEN_ENABLE, OUTPUT);
  pinMode(BUS_DATA_DIR, OUTPUT);

  digitalWrite(OUT_ENABLE, LOW);
  digitalWrite(TEST_ENABLE, HIGH);
  pinMode(OUT_ENABLE, OUTPUT);
  pinMode(TEST_ENABLE, OUTPUT);


//  Serial.begin(57600);
  Serial.begin(baudRate);
  Serial.println();

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
  if (stringAction.equals(F("IS_ROM_BUSY"))) return IS_DEVICE_BUSY;
  if (stringAction.equals(F("DEBUG_ON"))) return DEBUG_ON;
  if (stringAction.equals(F("DEBUG_OFF"))) return DEBUG_OFF;
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
      case DEBUG_ON: debugMode = true; Serial.println("DEBUG mode ON!"); break;
      case DEBUG_OFF: debugMode = false; Serial.println("DEBUG mode OFF!"); break;
      case LOCK_ADDRESS: lockAddress(false); break;
      case LOCK_ADDRESS_AND_DATA: lockAddressAndData(); break;
      case ENABLE_WRITE_PROTECT: writeProtect(true); break;
      case DISABLE_WRITE_PROTECT: writeProtect(false); break;
      // Sega Genesis
      case GENESIS_RESET: genesisReset(); break;
      case GENESIS_RESET_HOLD: genesisResetHold(); break;
      case GENESIS_RESET_RELEASE: genesisResetRelease(); break;

      // read via cart access
      case CART_LOCK_ADDRESS: lockAddress(true); break;
      case CART_READ_128_PAGE: read128WordPage(true); break;
      case CART_READ_128x128_PAGES: read128x128WordPages(true); break;
      
      // Invalid action
      case UNKNOWN_ACTION: Serial.println(F("UNKNOWN_ACTION")); break;
      default: break;
    }
  }
}

// ********************************
// Sega Genesis

void genesisReset() {
  if (debugMode) {
    Serial.println(F("DEBUG: reset->low"));
  }
  digitalWrite(GENESIS_RESET_PIN, LOW);
  delay(100);
  if (debugMode) {
    Serial.println(F("DEBUG: reset->high"));
  }
  digitalWrite(ROM_CE, LOW);
  pinMode(ROM_CE, OUTPUT);

  digitalWrite(GENESIS_RESET_PIN, HIGH);
  digitalWrite(GENESIS_RESET_PIN, HIGH);
  Serial.println(F("ACK_GENESIS"));
}

void genesisResetHold() {
  if (debugMode) {
    Serial.println(F("DEBUG: RESET_HOLD"));
  }
  digitalWrite(GENESIS_RESET_PIN, LOW);
  Serial.println(F("ACK_GENESIS"));
}


void genesisResetRelease() {
  if (debugMode) {
    Serial.println(F("DEBUG: RESET_RELEASE"));
  }
  digitalWrite(GENESIS_RESET_PIN, HIGH);
  Serial.println(F("ACK_GENESIS"));
}


// ********************************
// EEPROM

int readDeviceId() {
  // sofwware id entry
  writeWordBoth(0x5555, 0xAA);
  writeWordBoth(0x2AAA, 0x55);
  writeWordBoth(0x5555, 0x90);
  delay(10);
  
  byte vendorId = in16bits(0);
  byte deviceId = in16bits(1);

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
  byte vendorId = DeviceID >> 8;
  byte chipId = DeviceID & 0xff;
  Serial.print(vendorId, HEX);
  Serial.println(chipId, HEX);
}

void outputDeviceName() {
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

int loadData(unsigned int *buf, unsigned int bufSize) {
  int error = SUCCESS;

  Serial.println(F("AWAIT_DATA_HEX"));

  // wait for 3 secs for address or bail with an error
  long start = millis();
  while (Serial.available() == 0) {
    if (millis() - start > 3000) {
      return ERROR_TIMEOUT;
    }
  }
//  unsigned int otherBuf[128] = {0};
  unsigned int otherBuf[128] = {0};

  int totalReadWords = bufSize;
  int wordsRead = 0;
  while (wordsRead < totalReadWords) {
//    Serial.write(0x11); // XON
    char input[5];
    int bytesRead = Serial.readBytes(input, 4);
    if (bytesRead < 4) {
      if (debugMode) {
        Serial.print(F("DEBUG: bytesRead: "));Serial.print(bytesRead);Serial.print(F(", expected: 4"));
        Serial.print(F(", wordsRead:"));Serial.print(wordsRead);Serial.print(F(", expected:"));Serial.println(totalReadWords);
      }
      return ERROR_TIMEOUT;
    }
  
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
    otherBuf[wordsRead++] = dataWord;
  }
  Serial.println(F("ACK_DATA"));

  memcpy(buf, otherBuf, sizeof(otherBuf));

  
  if (debugMode) {
    char tmpStr[5];

    Serial.print(F("Debug: ACK_DATA: "));
    for (unsigned long i = 0; i < bufSize; i++) {
      sprintf(tmpStr, "%04X", buf[i]);
      Serial.print(tmpStr);
    }
    Serial.println();

//    Serial.print(F("Debug: ACK_DATA other: "));
//    for (unsigned long i = 0; i < bufSize; i++) {
//      sprintf(tmpStr, "%04X", otherBuf[i]);
//      Serial.print(tmpStr);
//    }
//    Serial.println();
  }


//  Serial.write(0x11); // XON


//  Serial.print("Bytes left on the stream: ");Serial.println(Serial.available());

  // sink the rest of the stream until newline
//  if (Serial.available() > 0) {
//    Serial.readStringUntil('\n');
//  }

  return SUCCESS;
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

  Serial.println("DATA_BEGIN");
  char tmpStr[5];
  int bytesToRead = 128;

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
    Serial.print(F("DEBUG: Locking at: "));Serial.print(address, HEX);Serial.print(F(", data: "));
    unsigned int val = cart? in16bitsCart(address) : in16bits(address);
    char tmpStr[5];
    sprintf(tmpStr, "%04X", val);
    Serial.println(tmpStr);
  }

  digitalWrite(TEST_ENABLE, LOW);
  outAddress(address);
//  digitalWrite(TEST_ENABLE, HIGH);

  Serial.println(F("ADDR_LOCKED"));
}

void lockAddressAndData() {
  int error;
  unsigned long address = loadAddress(error, false);
  if (error) {
    debugErrorOut(error);
    Serial.print(F("ERR_NO_ADDR "));Serial.println(error, HEX);
    return;
  }

  unsigned int data = 0;
  error = loadData(&data, 1);
  if (error) {
    debugErrorOut(error);
    Serial.print(F("ERR_NO_DATA "));Serial.println(error, HEX);
    return;
  }

  if (debugMode) {
    Serial.print(F("DEBUG: Locking at: "));Serial.print(address, HEX);Serial.print(F(", data: "));
    unsigned int val = in16bits(address);
    char tmpStr[5];
    sprintf(tmpStr, "%04X", data);
    Serial.println(tmpStr);
  }

  outAddressAndData(address, data);

  // disable OE on chip
  digitalWrite(ROM_CE, LOW);
  digitalWrite(ROM_OE, HIGH);

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

  Serial.println(F("DATA_BEGIN"));

  unsigned int data[128] = {0};

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

//  if (debugMode) {
//    char tmpStr[5];
//    Serial.print(F("DEBUG: addr:")); Serial.print(address, HEX); Serial.print(F(" writePages: "));
//    // output 128 words (we are using 2 chips each containing a byte) for the page
//    for (unsigned int i = 0; i < 128; i++) {
//      sprintf(tmpStr, "%04X", data[i]);
//      Serial.print(tmpStr);
//    }
//    Serial.println();
//  }

  writeBuffer(address, data, 128, true);

  Serial.println(F("ACK_DATA_WRITE"));

  unsigned char calculated_crc = calc_crc((unsigned char *)data, 128 * sizeof(int));
  Serial.println(calculated_crc, HEX);
  
}


void write128WordXPages() {
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
    error = loadData(data, 128 * num128Pages);
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
    int error = writeBuffer(page_address, data, 128 * num128Pages, true);
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

}


// internals for working with the eeprom chips

void writeWord(long addr24bit, int value) {
  writeBuffer(addr24bit, &value, 1, false);
}

void writeWordBoth(long addr24bit, int value) {
  int output = value + (value << 8);
  writeBuffer(addr24bit, &output, 1, false);
}

int writeBuffer(long addr24bit, int *buffer, int bufferSize, bool waitSuccess) {
  digitalWrite(ROM_OE, HIGH);
  digitalWrite(ROM_CE, LOW);
  digitalWrite(ROM_WE, HIGH);

  for (unsigned long i = 0; i < bufferSize; i++) {
    unsigned long target_address = addr24bit + i;
    
    outAddressAndData(target_address, buffer[i]);
    
    digitalWrite(ROM_WE, LOW);
    delayMicroseconds(10);
    digitalWrite(ROM_WE, HIGH);
    delayMicroseconds(10);
    digitalWrite(OUT_BUF_ADDR_OE, HIGH);
    digitalWrite(OUT_BUF_DATA_OE, HIGH);

    unsigned long page_start_address = long(i/128) * 128;
    bool pageBreak = target_address > 0 && ((target_address + 1) % 128 == 0 || i == bufferSize - 1);

    if (waitSuccess && pageBreak) {
    
      if (debugMode) {
        char tmpStr[5];
        Serial.print(F("DEBUG: target_address > 0: ")); Serial.print(target_address > 0); 
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

//        if ((in_data & 0x8080) == (last_word & 0x8080)) {
        if (in_data == last_word) {
          Serial.println(F("DEBUG: page write complete, moving on")); 

//          disableControl();
//          delay(1);
//          enableControl();

          digitalWrite(ROM_CE, LOW);
          digitalWrite(ROM_CE, HIGH);

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

  }
  
  digitalWrite(ROM_CE, HIGH);
  return 0;
}

void outAddressAndData(long addr24bit, int data16bit) {
  // disable buf addr & data OE
  digitalWrite(OUT_BUF_ADDR_OE, HIGH);
  digitalWrite(OUT_BUF_DATA_OE, HIGH);

  shiftOutAddressAndData(addr24bit, data16bit);
  
  // enble buf addr & data OE
  digitalWrite(OUT_BUF_ADDR_OE, LOW);
  digitalWrite(OUT_BUF_DATA_OE, LOW);
}

void outAddress(long addr24bit) {
  // disable addr & data OE
  digitalWrite(OUT_BUF_ADDR_OE, HIGH);
  digitalWrite(OUT_BUF_DATA_OE, HIGH);
  delayMicroseconds(1);

  shiftOutAddressAndData(addr24bit, 0);

  // enable data OE
  digitalWrite(OUT_BUF_ADDR_OE, LOW);
  delayMicroseconds(1);
}

void shiftOutAddressAndData(long addr24bit, int data16bit) {
  digitalWrite(OUT_BUF_CLR, LOW);

  // pretty sure clock pulse is needed here for the d-type flip flop that latches reset values in
  digitalWrite(OUT_BUF_CLK, LOW);
  digitalWrite(OUT_BUF_CLK, HIGH);
  digitalWrite(OUT_BUF_CLK, LOW);
  
  digitalWrite(OUT_BUF_CLR, HIGH);

  // disable latch as we want all 40 bits to be shifted in befor we do that
  digitalWrite(OUT_BUF_LATCH, LOW);

  // use SPI to shift in data
  byte addr1 = addr24bit >> 16 & 1;
  byte addr2 = addr24bit >> 8;
  byte addr3 = addr24bit;
  byte data1 = data16bit >> 8;
  byte data2 = data16bit;
  SPI.begin();
  byte buf[5] = {data1, data2, addr1, addr2, addr3};
  SPI.transfer(buf, 5);
  SPI.end();

  // pulse latch
  digitalWrite(OUT_BUF_LATCH, HIGH);
  digitalWrite(OUT_BUF_LATCH, LOW);
}

unsigned int in16bits(long addr24bit) {
  // enable output on ROM
  digitalWrite(OUT_BUF_DATA_OE, HIGH); // avoid bus fighting
  
  outAddress(addr24bit);

  digitalWrite(ROM_CE, LOW);
  digitalWrite(ROM_OE, LOW);

  digitalWrite(IN_LATCH_PIN, LOW);
  digitalWrite(IN_LATCH_PIN, HIGH);

  SPI.begin();
  unsigned int inValue = SPI.transfer16(0);
  SPI.end();

  return inValue;
}

unsigned int in16bitsCart(long addr24bit) {
  // enable output on ROM
  digitalWrite(OUT_BUF_DATA_OE, HIGH); // avoid bus fighting
  digitalWrite(TEST_ENABLE, LOW);

  outAddress(addr24bit);

//  digitalWrite(ROM_CE, LOW);
//  digitalWrite(ROM_OE, LOW);

  digitalWrite(IN_LATCH_PIN, LOW);
//  delay(1);
  digitalWrite(IN_LATCH_PIN, HIGH);

  // we have already latched in data
  digitalWrite(OUT_BUF_ADDR_OE, HIGH);
//  delay(1);

  SPI.begin();
  unsigned int inValue = SPI.transfer16(0);
  SPI.end();

  digitalWrite(TEST_ENABLE, HIGH);

  unsigned int outValue = ((inValue & 0xff) << 8) + (inValue >> 8);
  
  return outValue;
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
