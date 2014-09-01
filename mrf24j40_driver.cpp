#include <Arduino.h>
#include <mrf24j40_driver.h>

MRFDriver::MRFDriver(int pin_cs, int pin_int) {
  if(pin_cs == 9) {
	_portb_cs = 0b00000010;
	_pin_cs = 1;
  } else if(pin_cs == 10) {
    _portb_cs = 0b00000100;
	_pin_cs = 2;
  }
  _macBSN = random(sizeof(unsigned long));
  _seq_num =  1;

  pinMode(pin_cs,OUTPUT);
  pinMode(pin_int,INPUT);

  resetSS();
}

void MRFDriver::setSS(void) {
  PORTB &= ~_BV(_pin_cs);
}

void MRFDriver::resetSS(void) {
  PORTB |=  _BV(_pin_cs);
}

byte MRFDriver::read_short(byte address) {
  setSS();
  // 0 top for short addressing, 0 bottom for read
  SPI.transfer(address<<1 & 0b01111110);
  byte ret = SPI.transfer(0x0);
  resetSS();
  return ret;
}

byte MRFDriver::read_long(word address) {
  setSS();
  byte ahigh = address >> 3;
  byte alow = address << 5;
  SPI.transfer(0x80 | ahigh); // high bit for long
  SPI.transfer(alow);
  byte ret = SPI.transfer(0);
  resetSS();
  return ret;
}

void MRFDriver::write_short(byte address, byte data) {
    // 0 for top address, 1 bottom for write
	setSS();
    SPI.transfer((address<<1 & 0b01111110) | 0x01);
    SPI.transfer(data);
	resetSS();
}

void MRFDriver::write_long(word address, byte data) {
  setSS();
  byte ahigh = address >> 3;
  byte alow = address << 5;
  SPI.transfer(0x80 | ahigh); // high bit for long
  SPI.transfer(alow | 0x10); // last bit for write
  SPI.transfer(data);
  resetSS();
}

void MRFDriver::write_addr16(byte* addr16) {
  write_short(MRF_SADRL, addr16[0]);
  write_short(MRF_SADRH, addr16[1]);
  
  Serial.print("addr16: ");
  Serial.print(read_short(MRF_SADRH), HEX);
  Serial.println(read_short(MRF_SADRL), HEX);
}

void MRFDriver::writeAddress(void) {
  byte temp;
  //load EEPROM into local variable

  local.addr[0]  = EEPROM.read(EEPROM_MADR7);
  local.addr[1]  = EEPROM.read(EEPROM_MADR6);
  local.addr[2]  = EEPROM.read(EEPROM_MADR5);
  local.addr[3]  = EEPROM.read(EEPROM_MADR4);
  local.addr[4]  = EEPROM.read(EEPROM_MADR3);
  local.addr[5]  = EEPROM.read(EEPROM_MADR2);
  local.addr[6]  = EEPROM.read(EEPROM_MADR1);
  local.addr[7]  = EEPROM.read(EEPROM_MADR0);

  //write address to mrf transiever
  write_short(MRF_EADR0,local.addr[0]);
  write_short(MRF_EADR1,local.addr[1]);
  write_short(MRF_EADR2,local.addr[2]);
  write_short(MRF_EADR3,local.addr[3]);
  write_short(MRF_EADR4,local.addr[4]);
  write_short(MRF_EADR5,local.addr[5]);
  write_short(MRF_EADR6,local.addr[6]);
  write_short(MRF_EADR7,local.addr[7]);

  //print the address so we know the mrf got it
  Serial.print("addr64: ");
  temp = read_short(MRF_EADR7);
  if (temp < 16) Serial.print("0");
  Serial.print(temp, HEX);
  temp = read_short(MRF_EADR6);
  if (temp < 16) Serial.print("0");
  Serial.print(temp, HEX);
  temp = read_short(MRF_EADR5);
  if (temp < 16) Serial.print("0");
  Serial.print(temp, HEX);
  temp = read_short(MRF_EADR4);
  if (temp < 16) Serial.print("0");
  Serial.print(temp, HEX);
  temp = read_short(MRF_EADR3);
  if (temp < 16) Serial.print("0");
  Serial.print(temp, HEX);
  temp = read_short(MRF_EADR2);
  if (temp < 16) Serial.print("0");
  Serial.print(temp, HEX);
  temp = read_short(MRF_EADR1);
  if (temp < 16) Serial.print("0");
  Serial.print(temp, HEX);
  temp = read_short(MRF_EADR0);
  if (temp < 16) Serial.print("0");
  Serial.println(temp, HEX);
}

void MRFDriver::set_channel(byte channel) {
  write_long(MRF_RFCON0, (((channel - 11) << 4) | 0x03));
}

void MRFDriver::set_AES_key(void) {
  uint16_t i;
  uint16_t j;

  i=0x280;  //0x280 to 0x28F
  j=0x2B0; //0x2B0 to 0x2BF

  write_long(i++,MRF_AES0);
  write_long(i++,MRF_AES1);
  write_long(i++,MRF_AES2);
  write_long(i++,MRF_AES3);
  write_long(i++,MRF_AES4);
  write_long(i++,MRF_AES5);
  write_long(i++,MRF_AES6);
  write_long(i++,MRF_AES7);
  write_long(i++,MRF_AES8);
  write_long(i++,MRF_AES9);
  write_long(i++,MRF_AES10);
  write_long(i++,MRF_AES11);
  write_long(i++,MRF_AES12);
  write_long(i++,MRF_AES13);
  write_long(i++,MRF_AES14);
  write_long(i++,MRF_AES15);

  write_long(j++,MRF_AES0);
  write_long(j++,MRF_AES1);
  write_long(j++,MRF_AES2);
  write_long(j++,MRF_AES3);
  write_long(j++,MRF_AES4);
  write_long(j++,MRF_AES5);
  write_long(j++,MRF_AES6);
  write_long(j++,MRF_AES7);
  write_long(j++,MRF_AES8);
  write_long(j++,MRF_AES9);
  write_long(j++,MRF_AES10);
  write_long(j++,MRF_AES11);
  write_long(j++,MRF_AES12);
  write_long(j++,MRF_AES13);
  write_long(j++,MRF_AES14);
  write_long(j++,MRF_AES15);
}

void MRFDriver::reset(void) {
  write_short(MRF_SOFTRST, 0x07); // Perform a software reset
}

void MRFDriver::tx_ready(void) {
  byte header;
  byte txncon;

  // ??? <5> | sec en <1> | ack req <1> | tx rdy <1>
  txncon = 0x01;
  header = read_long(0x02);

  if((header & 0x20) == 0x20) {
    txncon = txncon | 0x02;
  }

  if((header & 0x04) == 0x04) {
    txncon = txncon | 0x04;
  }

  write_short(MRF_TXNCON,txncon);
}

void MRFDriver::tx_status(void) {
  byte txstat;

  txstat = read_short(MRF_TXSTAT);

  if((txstat & 0x01) == 0x01){
    Serial.print("ERR: TX RETRIES: ");
    Serial.println((txstat & 0xC0) >> 6);

	if((txstat & 0x20) == 0x20) {
      Serial.println("ERR: TX CHAN BUSY");
    }
  }
}

void MRFDriver::rx_disable(void) {
    write_short(MRF_BBREG1, 0x04); // RXDECINV - disable receiver
}

void MRFDriver::rx_enable(void) {
    write_short(MRF_BBREG1, 0x00); // RXDECINV - enable receiver
}

void MRFDriver::rx_flush(void) {
  write_short(MRF_RXFLUSH, 0x01);
}

void MRFDriver::decrypt(void) {
  byte temp;

  setSS();
  temp = read_short(MRF_SECCON0);
  resetSS();

  write_short(MRF_SECCON0, temp |= 0x40);
}

byte MRFDriver::get_interrupts(void) {
  byte interupts;

  setSS();
  interupts = read_short(MRF_INTSTAT);
  resetSS();
  return interupts;
}

void MRFDriver::proc_interrupt(void) {
  byte last_interrupt = get_interrupts();
  
  if(last_interrupt & MRF_I_RXIF) {
    Serial.println("rxxing...");
    rx_toBuffer();
  }
  if(last_interrupt & MRF_I_TXNIF) {
    Serial.println("txxing...");
    tx_status();
  }
  if(last_interrupt & MRF_I_SECIF) {
    //Serial.println("decrypting...");
    decrypt();
  }
    
  if(last_interrupt & 0xE6) {
    Serial.print("ERR: INT 0x");
    Serial.println(last_interrupt & 0xE6);
  }
    
  int_mrf=0;
}

void MRFDriver::rx_toBuffer(void) {
  byte i;
  //byte len;
  uint16_t ptr;

  if(_rx_count > 0) {
    Serial.print("rx !clr: ");
	Serial.println(_rx_count);
  }

  if((read_short(MRF_RXSR) & 0x04) == 0x04) {
    Serial.println("DECRYPTION ERROR");
    rx_flush();
  }
  else {
    noInterrupts();
    rx_disable();

	ptr=0x300;
    packet.frm_len = read_long(ptr++);
	
    packet.frm_ctrl1 = read_long(ptr++);
    packet.frm_ctrl2 = read_long(ptr++);
    packet.seq_num = read_long(ptr++);

    //if its not a beacon
    if((packet.frm_ctrl1 & 0b00000111)  != 0b00000000) {
      //Dest PAN ID
      packet.dest_pan[0] = read_long(ptr++);
      packet.dest_pan[1] = read_long(ptr++);
	  
      //Destintation Address
	  switch(packet.frm_ctrl2 & 0b00001100) {
	    case 0x0:  //dest addr not included
	      break;
	    case 0x08:  // short addr
	      ptr+=2;
	      break;
	    case 0x0C:  //long addr
	      for(i=0;i<8;i++) {
            packet.dest_addr[i] = read_long(ptr++);
          }
	      break;
	  }

    } else {
	  zeroAddress(packet.dest_addr);
	  zeroPAN(packet.dest_pan);
    }

    //Source PAN ID
    switch(packet.frm_ctrl1 & 0b01000000) {
      case 0x00:  //no pan compression
        packet.src_pan[0] = read_long(ptr++);
        packet.src_pan[1] = read_long(ptr++);
        break;
      case 0x40:  //pan compression
	    setPAN(packet.src_pan,packet.dest_pan);
        break;
    }

    //Source Address
	switch(packet.frm_ctrl2 & 0b11000000) {
      case 0x0:  //src addr not included
        break;
      case 0x80:  //short addr
	    ptr+=2;
        break;
      case 0xC0:  //long addr
        for(i=0;i<8;i++) {
          packet.src_addr[i] = read_long(ptr++);;
        }
        break;
      default:
        zeroAddress(packet.src_addr);
    }
	  
    //create data array
	packet.data_len = packet.frm_len-(ptr-0x300)-1;
	for(i=0;i<packet.data_len;i++) {
	  packet.data[i] = read_long(ptr++);
	}

	packet.crc1 = read_long(ptr++);
    packet.crc2 = read_long(ptr++);
    packet.lqi = read_long(ptr++);
    packet.rssi = read_long(ptr++);

	rx_enable();
    interrupts();

	_rx_count++;
	
    Serial.print("RSSI: ");
    Serial.println(packet.rssi);
    Serial.print("LQI: ");
    Serial.println(packet.lqi);
    Serial.print("CRC1: ");
    Serial.println(packet.crc1);
    Serial.print("CRC2: ");
    Serial.println(packet.crc2);
  }
}

// function to print a device address
void MRFDriver::printAddress(DeviceAddress a) {
  uint8_t i;

  for(i=0;i<8;i++) {
    // zero pad the address if necessary
    if (a[7-i] < 16) Serial.print("0");
    Serial.print(a[7-i], HEX);
  }
  Serial.println("");
}

void MRFDriver::zeroAddress(byte* a) {
  memset(a, 0, 8);
}

bool MRFDriver::compareAddress(byte* a, byte* b) {
  if(memcmp(a, b, 8) == 0) {
    return 1;
  } else {
    return 0;
  }
}

void MRFDriver::setAddress(byte* a, byte* b) {
  memcpy(a, b, 8);
}

// function to print a device address
void MRFDriver::printPAN(DeviceAddress a) {
  uint8_t i;

  for(i=0;i<2;i++) {
    // zero pad the address if necessary
    if (a[1-i] < 16) Serial.print("0");
    Serial.print(a[1-i], HEX);
  }
  Serial.println("");
}

void MRFDriver::zeroPAN(byte* a) {
  memset(a, 0, 2);
}

bool MRFDriver::comparePAN(byte* a, byte* b) {
  if(memcmp(a, b, 2) == 0) {
    return 1;
  } else {
    return 0;
  }
}

void MRFDriver::setPAN(byte* a, byte* b) {
  memcpy(a, b, 2);
}

void MRFDriver::writePAN(PanID panid) {
  write_short(MRF_PANIDL, panid[0]);
  write_short(MRF_PANIDH, panid[1]);

  Serial.print("panid: ");
  Serial.print(read_short(MRF_PANIDH), HEX);
  Serial.println(read_short(MRF_PANIDL), HEX);
}

void MRFDriver::EnergyDetect(void) {
  byte rssi_stat;
  uint8_t i;
  
  write_long(MRF_TESTMODE, 0x08);	// Disable automatic switch on PA/LNA
  write_short(MRF_TRISGPIO, 0x0F);	// Set GPIO direction to OUTPUT (control PA/LNA)
  write_short(MRF_GPIO, 0x0C);		// Enable LNA, disable
 
  for(i=0;i<16;i++) {
    set_channel(i); //Select channel
	
	write_short(MRF_BBREG6, 0x80);	// set RSSIMODE1 to initiate RSSI measurement
	
    rssi_stat = read_short(MRF_BBREG6);
	while((rssi_stat & 0x01) != 0x01) {
      rssi_stat = read_short(MRF_BBREG6);
    }
 
    rssi_stat = read_long(MRF_RSSI);
	
	Serial.print("Chan: ");
	Serial.print(i, HEX);
	Serial.print(" RSSI: ");
	Serial.println(rssi_stat, HEX);
  }
 
  write_short(MRF_BBREG6, 0x80);	// enable RSSI on received packets again after energy scan is finished
  write_short(MRF_GPIO, 0x00);
  write_short(MRF_TRISGPIO, 0x00);	// Set GPIO direction to INPUT
  write_long(MRF_TESTMODE, 0x0F);	// setup for automatic PA/LNA control
}