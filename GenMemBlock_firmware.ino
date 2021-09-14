#include <SPI.h>

bool debugMode = true;
int DeviceID = 0;

// 74hc165 control
int IN_SER_PIN = 12,
  IN_LATCH_PIN = A0,
  IN_CLK_PIN = 13;

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
  OUT_BUF_DATA_OE = 6;


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
//- Busy? (page write in progress)
  IS_DEVICE_BUSY,
  DEBUG_ON,
  DEBUG_OFF,
  LOCK_ADDRESS,
  LOCK_ADDRESS_AND_DATA,
  ENABLE_WRITE_PROTECT,
  DISABLE_WRITE_PROTECT,
// no idea what is being asked
  UNKNOWN_ACTION
};


void setup() {
  // rom setup so we do not get weird data
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
  pinMode(OUT_BUF_SER, OUTPUT);
  pinMode(OUT_BUF_CLR, OUTPUT);
  pinMode(OUT_BUF_CLK, OUTPUT);
  pinMode(OUT_BUF_LATCH, OUTPUT);
  pinMode(OUT_BUF_ADDR_OE, OUTPUT);
  pinMode(OUT_BUF_DATA_OE, OUTPUT);

//  Serial.begin(57600);
  Serial.begin(500000);
  Serial.println();

  delay(1000);

  DeviceID = readDeviceId();

  Serial.print("Available actions: 0..");Serial.println(UNKNOWN_ACTION - 1);
  Serial.print("EEPROM: ");
  outputDeviceName();
  Serial.println("READY");
}

int parseAction(String stringAction) {
  if (stringAction.equals("PING")) return PING;
  if (stringAction.equals("READ_ROM_ID")) return READ_ROM_ID;
  if (stringAction.equals("READ_ROM_NAME")) return READ_ROM_NAME;
  if (stringAction.equals("READ_ROM_SIZE")) return READ_ROM_SIZE;
  if (stringAction.equals("READ_128_PAGE")) return READ_128_PAGE;
  if (stringAction.equals("READ_128_PAGE_CRC")) return READ_128_PAGE_CRC;
  if (stringAction.equals("READ_128x128_PAGES")) return READ_128x128_PAGES;
  if (stringAction.equals("WRITE_128_PAGE")) return WRITE_128_PAGE;
  if (stringAction.equals("IS_ROM_BUSY")) return IS_DEVICE_BUSY;
  if (stringAction.equals("DEBUG_ON")) return DEBUG_ON;
  if (stringAction.equals("DEBUG_OFF")) return DEBUG_OFF;
  if (stringAction.equals("LOCK_ADDRESS")) return LOCK_ADDRESS;
  if (stringAction.equals("LOCK_ADDRESS_AND_DATA")) return LOCK_ADDRESS_AND_DATA;
  if (stringAction.equals("ENABLE_WRITE_PROTECT")) return ENABLE_WRITE_PROTECT;
  if (stringAction.equals("DISABLE_WRITE_PROTECT")) return DISABLE_WRITE_PROTECT;
  return UNKNOWN_ACTION;
}

void loop() {

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    if (debugMode) {
      Serial.print("DEBUG: parsing raw input: ");Serial.println(input);
    }
    int action = parseAction(input);

    if (debugMode) {
      Serial.print("DEBUG: ");Serial.print(input);Serial.print("; decoded:");Serial.println(action);
    }

    switch (action) {
      case PING: Serial.println("PONG"); break;
      case READ_ROM_ID: outputDeviceId(); break;
      case READ_ROM_NAME: outputDeviceName(); break;
      case READ_ROM_SIZE: outputDeviceSize(); break;
      case READ_128_PAGE: read128WordPage(); break;
      case READ_128_PAGE_CRC: read128WordPageCRC(); break;
      case READ_128x128_PAGES: read128x128WordPages(); break;
      case WRITE_128_PAGE: write128WordPage(); break;
      case DEBUG_ON: debugMode = true; Serial.println("DEBUG mode ON!"); break;
      case DEBUG_OFF: debugMode = false; Serial.println("DEBUG mode OFF!"); break;
      case LOCK_ADDRESS: lockAddress(); break;
      case LOCK_ADDRESS_AND_DATA: lockAddressAndData(); break;
      case ENABLE_WRITE_PROTECT: writeProtect(true); break;
      case DISABLE_WRITE_PROTECT: writeProtect(false); break;
      case UNKNOWN_ACTION: Serial.println("UNKNOWN_ACTION"); break;
      default: break;
    }

  }

}

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
    Serial.print("DEBUG: getRomSize:: vendorId:");Serial.print(vendorId, HEX);Serial.print(";chipId:");Serial.println(chipId, HEX);
  }
  long romSize = 0;
  if (vendorId == 0xbf && chipId == 0x07) {
    if (debugMode) {
      Serial.println("DEBUG: Found EEPROM SST SST29EE010");
    }
    romSize = 262144;
  } else if (vendorId == 0x1f && chipId == 0x13) {
    if (debugMode) {
      Serial.println("DEBUG: Found EEPROM Atmel AT49F040");
    }
    romSize = 262144 * 8;
  }
  if (debugMode) {
    Serial.print("DEBUG: romSize:");Serial.println(romSize, HEX);
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
    Serial.print("DEBUG: got error ");
    switch (error) {
      case SUCCESS: Serial.println("SUCCESS"); break;
      case ERROR_PARSE: Serial.println("CAN NOT PARSE"); break;
      case ERROR_TIMEOUT: Serial.println("TIMEOUT"); break;
      case ERROR_ADDR_RANGE: Serial.println("ADDRESS RANGE ERROR"); break;
      default: Serial.print("UNKNOWN:"); Serial.println(error, HEX); break;
    }
  }
}

unsigned long loadAddress(int &error, bool alignPageBoundary) {

  long romSize = getRomSize(DeviceID);
  
  Serial.println("AWAIT_ADDR_HEX");
  error = SUCCESS;

  // wait for 3 secs for address or bail with an error
  long start = millis();
  while (Serial.available() == 0) {
    if (millis() - start > 3000) {
      error = ERROR_TIMEOUT;
      return;
    }
  }
    
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
    Serial.print("DEBUG: characters parsed: ");Serial.println(charsParsed);
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
    Serial.print("DEBUG: Got address: 0x");Serial.println(address, HEX);
  }

  if (address >= romSize) {
    error = ERROR_ADDR_RANGE;
    return 0;
  }
  
  Serial.println("ACK_ADDR");
  if (alignPageBoundary) {
    // figure out the page boundary if 
    unsigned long pageStartAddress = address - address % 128;
    if (debugMode) {
      Serial.print("DEBUG: Page start address: 0x");Serial.println(pageStartAddress, HEX);
    }
    return pageStartAddress;
  } else {
    return address;
  }
}

int loadAddress(int &error) {
  loadAddress(error, false);
}

int loadData(unsigned int *buf, unsigned int bufSize) {
  int error = SUCCESS;

  Serial.println("AWAIT_DATA_HEX");

  // wait for 3 secs for address or bail with an error
  long start = millis();
  while (Serial.available() == 0) {
    if (millis() - start > 3000) {
      return ERROR_TIMEOUT;
    }
  }

  int totalReadWords = bufSize;
  int wordsRead = 0;
  while (wordsRead < totalReadWords) {
//    Serial.write(0x11); // XON
    char input[5];
    int bytesRead = Serial.readBytes(input, 4);
//    Serial.write(0x13); // XOFF
    if (bytesRead < 4) {
      if (debugMode) {
        Serial.print("DEBUG: bytesRead: ");Serial.print(bytesRead);Serial.print(", expected: 4");
        Serial.print(", wordsRead:");Serial.print(wordsRead);Serial.print(", expected:");Serial.println(totalReadWords);
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
    buf[wordsRead++] = dataWord;
  }
  Serial.println("ACK_DATA");
//  Serial.write(0x11); // XON


//  Serial.print("Bytes left on the stream: ");Serial.println(Serial.available());

  // sink the rest of the stream until newline
  if (Serial.available() > 0) {
    Serial.readStringUntil('\n');
  }

  return SUCCESS;
}

int loadData(unsigned int *buf) {
  return loadData(buf, 128);
}

int loadPages(int &error) {
  Serial.println("AWAIT_PAGES_HEX");
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
    Serial.print("DEBUG: characters parsed: ");Serial.println(charsParsed);
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
    Serial.print("DEBUG: Got pages: 0x");Serial.println(pages, HEX);
  }
  
  Serial.println("ACK_PAGES");
  return pages;
}


void read128WordPage() {

  int error;
  unsigned long address = loadAddress(error, true);

  if (error) {
    debugErrorOut(error);
    Serial.print("ERR_NO_ADDR ");Serial.println(error, HEX);
    return;
  }

  if (debugMode) {
    char tmpStr[5];
    Serial.print("DEBUG: on device read:"); Serial.print(address, HEX); Serial.print(" bytes: "); 
    // output 128 words (we are using 2 chips each containing a byte) for the page
    for (unsigned long i = 0; i < 128; i++) {
       unsigned int val = in16bits(address + i);
      sprintf(tmpStr, "%04X", val);
      Serial.print(tmpStr);
    }
    Serial.println();
  }

  Serial.println("DATA_BEGIN");
  char tmpStr[5];
  int bytesToRead = 128;
  // output 128 words (we are using 2 chips each containing a byte) for the page
  for (unsigned int i = 0; i < bytesToRead; i++) {
    unsigned int val = in16bits(address + i);
    sprintf(tmpStr, "%04X", val);
    Serial.print(tmpStr);
  }
  Serial.println();
}


void lockAddress() {
  int error;
  unsigned long address = loadAddress(error, false);

  if (error) {
    debugErrorOut(error);
    Serial.print("ERR_NO_ADDR ");Serial.println(error, HEX);
    return;
  }

  if (debugMode) {
    Serial.print("DEBUG: Locking eeprom at address: ");Serial.print(address, HEX);Serial.print(", data: ");
    unsigned int val = in16bits(address);
    char tmpStr[5];
    sprintf(tmpStr, "%04X", val);
    Serial.println(tmpStr);
  }

  outAddress(address);

  Serial.println("ADDR_LOCKED");
}

void lockAddressAndData() {
  int error;
  unsigned long address = loadAddress(error, false);
  if (error) {
    debugErrorOut(error);
    Serial.print("ERR_NO_ADDR ");Serial.println(error, HEX);
    return;
  }

  unsigned int data = 0;
  error = loadData(&data, 1);
  if (error) {
    debugErrorOut(error);
    Serial.print("ERR_NO_DATA ");Serial.println(error, HEX);
    return;
  }

  if (debugMode) {
    Serial.print("DEBUG: Locking eeprom at address: ");Serial.print(address, HEX);Serial.print(", data: ");
    unsigned int val = in16bits(address);
    char tmpStr[5];
    sprintf(tmpStr, "%04X", data);
    Serial.println(tmpStr);
  }

  outAddressAndData(address, data);

  // disable OE on chip
  digitalWrite(ROM_CE, LOW);
  digitalWrite(ROM_OE, HIGH);

  Serial.println("ADDR_AND_DATA_LOCKED");
}


void read128x128WordPages() {
  int error;
  unsigned long address = loadAddress(error, false);
  address = address - address % 128;

  if (error) {
    debugErrorOut(error);
    Serial.print("ERR_NO_ADDR ");Serial.println(error, HEX);
    return;
  }


  unsigned long pagesToLoad = loadPages(error);
  if (error) {
    debugErrorOut(error);
    Serial.print("ERR_NO_PAGES ");Serial.println(error, HEX);
    return;
  }

  // output 128 words (we are using 2 chips each containing a byte) for the page
  unsigned char crc = 0;
  const long wordsPerPage = 128;
  unsigned int pageBuf[wordsPerPage];
  char tmpStr[5];

  if (debugMode) {
    Serial.print("DEBUG: Will read pages: ");Serial.print(pagesToLoad);  
    Serial.println(", at addresses:");
    for (unsigned long page = 0; page < pagesToLoad; page++) {
      unsigned long pageOffset = page * wordsPerPage;
      unsigned long read_at_address = address + pageOffset;
      Serial.print("DEBUG: page:");Serial.print(page);Serial.print(", address:");Serial.println(read_at_address);
    }
  }

  Serial.println("DATA_BEGIN");

  for (unsigned long page = 0; page < pagesToLoad; page++) {
    unsigned long pageOffset = page * wordsPerPage;
    for (unsigned long i = 0; i < wordsPerPage; i++) {
      unsigned long read_at_address = address + pageOffset + i;
      unsigned int val = in16bits(read_at_address);
      unsigned char *vals = (unsigned char *)&val;
      pageBuf[i] = val;
      while (Serial.availableForWrite() < 2) {}
      Serial.write(vals[1]);
      Serial.write(vals[0]);
    }
    crc = calc_crc((unsigned char*)pageBuf, sizeof(pageBuf), crc);
  }
  Serial.write(crc);
  Serial.println();
}

void read128WordPageCRC() {
  int error;
  unsigned long address = loadAddress(error, true);

  if (error) {
    debugErrorOut(error);
    Serial.print("ERR_NO_ADDR ");Serial.println(error, HEX);
    return;
  }

  if (debugMode) {
    Serial.println("DEBUG: Reading page...");
  }

  unsigned int buf[128];
  for (unsigned int i = 0; i < 128; i++) {
    buf[i] = in16bits(address + i);
  }

  if (debugMode) {
    Serial.println("DEBUG: Got words & calculating CRC...");
  }

  unsigned char calculated_crc = calc_crc((unsigned char *)buf, 128 * sizeof(int));
  Serial.println(calculated_crc, HEX);
}

void write128WordPage() {
  int error = SUCCESS;
  unsigned long address = loadAddress(error, true);

  if (error) {
    debugErrorOut(error);
    Serial.print("ERR_NO_ADDR ");Serial.println(error, HEX);
    return;
  }

  unsigned int data[128] = {0};
  error = loadData(data);
  if (error) {
    debugErrorOut(error);
    if (debugMode) {
      char tmpStr[5];
      Serial.print("DEBUG: Received page: ");
      for (unsigned int i = 0; i < 128; i++) {
        sprintf(tmpStr, "%04X", data[i]);
        Serial.print(tmpStr);
      }
      Serial.println();
    }
    Serial.print("ERR_NO_DATA ");Serial.println(error, HEX);
    return;
  }

  if (debugMode) {
    char tmpStr[5];
    Serial.print("DEBUG: addr:"); Serial.print(address, HEX); Serial.print(" writePage: "); 
    // output 128 words (we are using 2 chips each containing a byte) for the page
    for (unsigned int i = 0; i < 128; i++) {
      sprintf(tmpStr, "%04X", data[i]);
      Serial.print(tmpStr);
    }
    Serial.println();
  }

  writeBuffer(address, data, 128);

  Serial.println("ACK_DATA_WRITE");

  unsigned char calculated_crc = calc_crc((unsigned char *)data, 128 * sizeof(int));
  Serial.println(calculated_crc, HEX);
}


// internals for working with the eeprom chips

void writeWord(long addr24bit, int value) {
  writeBuffer(addr24bit, &value, 1);
}

void writeWordBoth(long addr24bit, int value) {
  int output = value + (value << 8);
  writeBuffer(addr24bit, &output, 1);
}

void writeBuffer(long addr24bit, int *buffer, int bufferSize) {
  digitalWrite(ROM_OE, HIGH);
  digitalWrite(ROM_CE, LOW);
  digitalWrite(ROM_WE, HIGH);

  for (int i = 0; i < bufferSize; i++) {
    outAddressAndData(addr24bit + i, buffer[i]);
    digitalWrite(ROM_WE, LOW);
    digitalWrite(ROM_WE, HIGH);
    digitalWrite(OUT_BUF_ADDR_OE, HIGH);
    digitalWrite(OUT_BUF_DATA_OE, HIGH);
  }
  
  digitalWrite(ROM_CE, HIGH);
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

  shiftOutAddressAndData(addr24bit, 0);

  // enable data OE
  digitalWrite(OUT_BUF_ADDR_OE, LOW);
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
  unsigned int inValue = SPI.transfer16(1);
  SPI.end();

  return inValue;
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
