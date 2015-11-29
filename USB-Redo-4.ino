#include <cdcftdi.h>
#include <usbhub.h>
#include <SPI.h>

//#include "pgmstrings.h"
// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

//#define DEVADDR 22

/* Forward declarations for state machine states */
void do_nothing( void );
void wait_ready( void );
void init_device( void );
void device_loop( void );
uint8_t galvant_xfer( char* str, uint8_t* buf );
void print_buf( uint8_t nbytes, uint8_t* buf );

class FTDIAsync : 
public FTDIAsyncOper
{
public:
  virtual uint8_t OnInit(FTDI *pftdi);
  virtual uint8_t OnRelease(FTDI *pftdi) {
  };
};

uint8_t FTDIAsync::OnInit(FTDI *pftdi)
{
  uint8_t rcode = 0;
  rcode = pftdi->SetBaudRate(460800);
  if (rcode)
  {
    ErrorMessage<uint8_t>(PSTR("SetBaudRate"), rcode);
    return rcode;
  }
  rcode = pftdi->SetFlowControl(FTDI_SIO_DISABLE_FLOW_CTRL);
  if (rcode)
    ErrorMessage<uint8_t>(PSTR("SetFlowControl"), rcode);
  return rcode;
}

USB              Usb;
//USBHub         Hub(&Usb);
FTDIAsync        FtdiAsync;
FTDI             Ftdi(&Usb, &FtdiAsync);

typedef void (*State)(void);

State state_p = NULL;  //state machine state pointer 

void setup()
{
  Serial.begin( 9600 );
  while (!Serial); // Wait for serial port to connect with built-in USB CDC serial connection
  Serial.println("Start");
  if (Usb.Init() == -1)
    Serial.println("OSC did not start.");
  delay( 100 );
  state_p = &wait_ready;
}

void loop()
{
  Usb.Task();
  (*state_p)();
}

void do_nothing() {
}

/* Wait till USB is ready */
void wait_ready() {
  if( Usb.getUsbTaskState() == USB_STATE_RUNNING )  {
    Serial.println("\r\nUSB Ready");
    state_p = &init_device;
    delay( 500 );
  }
}

/* Set device address and put device in known state */
void init_device() {
  uint8_t  buf[64];  //receive buffer
/* Detecting Galvant dongle */
  {
    char strbuf[] = "++ver\r";
    char verstr[] = "Version";  //a string expected from a device
    Serial.println("Checking GPIB Adapter");
    uint8_t recv = galvant_xfer( strbuf, buf );
    if (recv > 2) {
      if( !memcmp( buf+2, verstr, 7 )) {
        Serial.println(" * Galvant device present");
        print_buf( recv, buf );
    } else {
      Serial.println("\r\nWRONG DEVICE on the USB bus...Remove and restart!");
      while(1);
      }
    }//if( recv > 2...
  }//detecting Galvant dongle

/* Setting device address */
    {
      Serial.print(" * setting GPIB address ");
      galvant_xfer( "++addr 3\r", NULL );
      uint8_t recv = galvant_xfer( "++addr\r", buf );
      if (recv) {
        print_buf( recv, buf );
      }
    }
/* Set ++auto to 0 */
    {
      Serial.print(" * set ++auto to 0 ");
      galvant_xfer( "++auto 1\r", NULL );
      uint8_t recv = galvant_xfer( "++auto\r", buf );
      if (recv) {
        print_buf( recv, buf );
      }
    }
/* Set ++eos to 0 could be 0,1,2,3*/
    {
      Serial.print(" * set ++eos to 0 ");
      galvant_xfer( "++eos 0\r", NULL );
      uint8_t recv = galvant_xfer( "++eos\r", buf );
      if (recv) {
        print_buf( recv, buf );
      }
    }
/* Set ++eoi to 1 */
    {
      Serial.print(" * set ++eoi to 1 ");
      galvant_xfer( "++eoi 1\r", NULL );
      uint8_t recv = galvant_xfer( "++eoi\r", buf );
      if (recv) {
        print_buf( recv, buf );
      }
    }
/* Set ++eot_enable */
    {
      Serial.print(" * set ++eot_enable to ");
      //galvant_xfer( "++eo 1\r", NULL );
      uint8_t recv = galvant_xfer( "++eot_enable\r", buf );
      if (recv) {
        print_buf( recv, buf );
      }
    }
/* Set ++read_tmo_ms */
    {
      Serial.print(" * set ++read_tmo_ms to ");
      uint8_t recv = galvant_xfer( "++read_tmo_ms\r", buf );
      if (recv) {
        print_buf( recv, buf );
      }
    }  
    state_p = &device_loop;
}

/* Called indefinitely from the loop after device init has completed */
void device_loop() {
  //char cmd[] = "vl\r";
  char cmd[32];
  memcpy(cmd, "123.4vl\r", 9);
  Serial.println( cmd );
  galvant_xfer( cmd, NULL );
}

/* Transfers a piece of data to the GPIB adapter and reads reply, if expected */
/* str must be null-terminated, buf must hold 64 bytes */
/* returns number of bytes read or zero if no read has been performed */
uint8_t galvant_xfer( char* str, uint8_t* buf ) {
  uint8_t rcode = Ftdi.SndData(strlen(str), (uint8_t*)str);
  if (rcode)
    ErrorMessage<uint8_t>(PSTR("SndData"), rcode);
  //delay(200);
  delay( 750 ); 
  if( buf ) {  //response expected
    for (uint8_t i=0; i<64; i++) {
      buf[i] = 0;
    }
    uint16_t rcvd = 64;
    rcode = Ftdi.RcvData(&rcvd, buf);
    if (rcode && rcode != hrNAK) {
      ErrorMessage<uint8_t>(PSTR("Ret"), rcode);
    }
    return( rcvd );
  } else {
    return( 0 );
  }//if( buf...  
}
 
void print_buf( uint8_t nbytes, uint8_t* buf ) {
    while( nbytes-- ) {  
      Serial.write( *buf++);
    }    
    Serial.print("\r\n");
}

  







