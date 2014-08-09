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

void MRFDriver::init(void){
  write_short(MRF_GPIO, 0x00);
  write_short(MRF_TRISGPIO, 0x00);
  write_long(MRF_TESTMODE, 0x0F);

  write_short(MRF_PACON0, 0x29);
  write_short(MRF_PACON1, 0x02);
  write_short(MRF_PACON2, 0x98); // Initialize FIFOEN = 1 and TXONTS = 0x6.
  write_short(MRF_TXSTBL, 0x95); // Initialize RFSTBL = 0x9.

  write_long(MRF_RFCON0, 0x03); // Initialize RFOPT = 0x03.
  write_long(MRF_RFCON1, 0x02); // Initialize VCOOPT = 0x02.
  write_long(MRF_RFCON2, 0x80); // Enable PLL (PLLEN = 1).
  write_long(MRF_RFCON6, 0x90); // Initialize TXFIL = 1 and 20MRECVR = 1.
  write_long(MRF_RFCON7, 0x80); // Initialize SLPCLKSEL = 0x2 (100 kHz Internal oscillator).
  write_long(MRF_RFCON8, 0x10); // Initialize RFVCO = 1.
  write_long(MRF_SLPCON1, 0x21); // Initialize CLKOUTEN = 1 and SLPCLKDIV = 0x01.

  // Configuration for nonbeacon-enabled devices (see Section 3.8 Beacon-Enabled and
  // Nonbeacon-Enabled Networks)
  write_short(MRF_TXMCR, 0x1C);  //Clear Slotted mode

  //Security
  set_AES_key(); //install the tx and rx security key
  //write_short(MRF_SECCON0,0x12); //enable AES-CCM-128 on the TXFIFO and RXFIFO
  //write_short(MRF_SECCON0,0x09); //AES-CTR
  write_short(MRF_SECCON0,0); //NONE

  write_short(MRF_BBREG2, 0x80); // Set CCA mode to ED
  write_short(MRF_CCAEDTH, 0x60); // Set CCA ED threshold.
  write_short(MRF_BBREG6, 0x40); // Set appended RSSI value to RXFIFO.

  // Initialize interrupts
  write_long(MRF_SLPCON0, 0x01); //Interrupt on falling edge and disable sleep clock
  write_short(MRF_INTCON, 0xE6); //Enable SEC, RX, and TX Interrupts

  set_channel(7);
  write_long(MRF_RFCON3, 0x40); //Select TX Power
  write_short(MRF_RFCTL, 0x04); //Reset RF state machine.
  write_short(MRF_RFCTL, 0x00);

  delay(1); //delay at least 192usec

  set_addr64();
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

word MRFDriver::read_pan(void) {
  byte panh = read_short(MRF_PANIDH);
  return panh << 8 | read_short(MRF_PANIDL);
}

void MRFDriver::write_pan(word panid) {
  mrf_panid[0] = panid & 0xff;
  mrf_panid[1] = panid >> 8;

  write_short(MRF_PANIDL, mrf_panid[0]);
  write_short(MRF_PANIDH, mrf_panid[1]);

  Serial.print("panid: ");
  Serial.print(read_short(MRF_PANIDH), HEX);
  Serial.println(read_short(MRF_PANIDL), HEX);
}

void MRFDriver::write_addr16(word address16) {
  write_short(MRF_SADRH, address16 >> 8);
  write_short(MRF_SADRL, address16 & 0xff);

  Serial.print("addr16: ");
  Serial.print(read_short(MRF_SADRH), HEX);
  Serial.println(read_short(MRF_SADRL), HEX);
}

word MRFDriver::read_addr16(void) {
  byte a16h = read_short(MRF_SADRH);
  return a16h << 8 | read_short(MRF_SADRL);
}

void MRFDriver::set_addr64(void) {
  byte temp;
  //load EEPROM into local variable

  mrf_mac[0]  = EEPROM.read(EEPROM_MADR7);
  mrf_mac[1]  = EEPROM.read(EEPROM_MADR6);
  mrf_mac[2]  = EEPROM.read(EEPROM_MADR5);
  mrf_mac[3]  = EEPROM.read(EEPROM_MADR4);
  mrf_mac[4]  = EEPROM.read(EEPROM_MADR3);
  mrf_mac[5]  = EEPROM.read(EEPROM_MADR2);
  mrf_mac[6]  = EEPROM.read(EEPROM_MADR1);
  mrf_mac[7]  = EEPROM.read(EEPROM_MADR0);

  //write address to mrf transiever
  write_short(MRF_EADR0,mrf_mac[0]);
  write_short(MRF_EADR1,mrf_mac[1]);
  write_short(MRF_EADR2,mrf_mac[2]);
  write_short(MRF_EADR3,mrf_mac[3]);
  write_short(MRF_EADR4,mrf_mac[4]);
  write_short(MRF_EADR5,mrf_mac[5]);
  write_short(MRF_EADR6,mrf_mac[6]);
  write_short(MRF_EADR7,mrf_mac[7]);

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
    //Serial.println("rxxing...");
    rx_toBuffer();
  }
  if(last_interrupt & MRF_I_TXNIF) {
    //Serial.println("txxing...");
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
      packet.dest_pan = read_long(ptr++);
      packet.dest_pan |= (read_long(ptr++) << 8);

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
	  packet.dest_pan = 0;
	  memset(packet.dest_addr, 0, 8);
    }

    //Source PAN ID
    switch(packet.frm_ctrl1 & 0b01000000) {
      case 0x00:  //no pan compression
        packet.src_pan = read_long(ptr++);
        packet.src_pan |= (read_long(ptr++) << 8);
        break;
      case 0x40:  //pan compression
	    packet.src_pan = packet.dest_pan;
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
        memset(packet.src_addr, 0, 8);
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