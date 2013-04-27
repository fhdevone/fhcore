#include <Arduino.h>
/*

 FloatHub Arduino Code
 (c) 2011 Thor Sigvaldason
 (begun June 6, 2011)
 
 
  Nov. 6  2012
  
  Working _extremely_ well now, including active sensor, data storage when GPRS unavailable, etc.
  Remaining task; cleanout enough string stuff, then implement NMEA input on Serial2.


 
  March, 2013

  Moved to a proper text-based build environment and versioning system on git-hub.


  April, 2013
  
  Brought in encrypted/$FHS code that was working back in July and made it trunk
  It encrypts content using aes-128-cbc encryption

 
 
*/

#include <Wire.h>
#include "libs/Adafruit_BMP/Adafruit_BMP085.h"
#include <EEPROM.h>
#include <stdio.h>
#include <Time.h>
#include "libs/AES/AES.h"
#include "libs/Base64/Base64.h"


/*
  Code version
*/

#define  CODE_MAJOR_VERSION  0
#define  CODE_MINOR_VERSION  1

/*
  Compile time debug flags
*/

//#define GPRS_DEBUG_ON
//#define GPS_DEBUG_ON
//#define PUMP_DEBUG_ON

/*
  Some AES variables
*/

AES aes;
byte iv[16];
byte volatile_iv[16];
byte plain[512];
byte cipher[512];

/*
  Function protytpes
  (for functions below that are used before fully defined)
  
*/

void bmp_read(void);
void help_info(String some_info);
void display_current_variables();
void queue_pump_message(int which_pump, bool is_on);
void echo_info(String some_info);
void queue_detailed_message();
void encode_latest_message_to_send(void);
float eeprom_read_float(int address);
void slide_memory(int start, int how_many, int what_width);
void debug_info(String some_info);

/*
void pop_off_pump_message();
void pop_off_detailed_message();
*/

/*
  ID and connection values
*/

//  String        default_float_hub_id           = "throwout";
//  String        default_float_hub_server       = "sigvaldason.homeip.net";
    unsigned int  default_float_hub_server_port  = 9009;
//  String        default_gprs_apn               = "wap.cingular";
//  String        default_gprs_username          = "WAP@CINGULARGPRS.COM";
//  String        default_gprs_password          = "CINGULAR1";

#define  PUMP_STATE_COUNTER_LOCATION  189
#define  PUMP_STATE_START_LOCATION    190
#define  PUMP_STATE_DATA_WIDTH        5
#define  PUMP_STATE_NUMB_LOCATIONS    50

#define  DETAILED_STATE_COUNTER_LOCATION  440
#define  DETAILED_STATE_START_LOCATION    441
#define  DETAILED_STATE_DATA_WIDTH        49
#define  DETAILED_STATE_NUMB_LOCATIONS    74

#define  CHALLENGE_RESPONSE_START_LOCATION  4067
#define  CHALLENGE_RESPONSE_NUMB_LOCATIONS  20

  String        float_hub_id;
  String        float_hub_server;
  unsigned int  float_hub_server_port;
  String        gprs_apn;
  String        gprs_username;
  String        gprs_password;
  unsigned long boot_counter;
  unsigned long last_detailed_eeprom_write;
  byte          detailed_state_count;
  byte          pump_state_count;
  byte          float_hub_aes_key[16]; 

/*
  Status LED's
*/

#define RED_LED	    12
#define YELLOW_LED  11
#define GREEN_LED   10

 /*
   Some global character arrays
 */
 
  String   new_message = "";
  char     temp_string[20];  


  bool gprs_communications_on = true;

/*
    Overall timing parameters
*/

  long sensor_sample_interval = 20000;    //  Check temperature, pressure, every 20 seconds
  long gps_interval = 50;                 //  Read GPS serial every 1/20 second
  long voltage_interval = 5000;           //  Check batteries/chargers every 5 second
  long gprs_interval = 500;               //  Check GPRS every 500 milliseconds
  long pump_interval = 300;               //  Check pump state every 300 milliseconds
  long active_reporting_interval = 30000; //  When in use, report data every 30 seconds
  long idle_reporting_interval = 600000;  //  When idle, report data every 10 minutes
//  long idle_reporting_interval = 30000; 
  long console_reporting_interval = 5000; //  Report to USB console every 5 seconds  
  long console_interval = 500;            //  Check console for input every 400 milliseconds
  long gprs_watchdog_interval = 90000;    //  Reboot the GPRS module after 90 seconds of no progress
  long led_update_interval = 200;         //  Update the LED's every 200 miliseconds
  boolean green_led_state = false;         //  For cycling on and off  
  
  long sensor_previous_timestamp = 0;
  long gps_previous_timestamp = 0;
  long voltage_previous_timestamp = 0;
  long gprs_previous_timestamp = 0;
  long pump_previous_timestamp = 0;
  long previous_active_timestamp = 0;
  long previous_idle_timestamp = 0;
  long previous_console_timestamp = 0; 
  long console_previous_timestamp = 0;
  long led_previous_timestamp = 0;

  bool currently_active = true;
  
  /*
      GPRS flags that describe current state of communication
  */
  
  long gprs_watchdog_timestamp = 0;
  int  max_gprs_buffer = 300;
  String gprs_latest_message_to_send = "";
 
  //const int message_stack_max_count = 5;
  //String message_stack[message_stack_max_count];
  

  enum connection_state
  {
    waiting_for_sind,
    waiting_for_gprs_attachment,
    waiting_for_pdp_ack,
    waiting_for_pco_ack,
    waiting_for_pdp_context_active,
    waiting_for_remote_host_ack,
    waiting_for_string_format_ack,
    we_be_connected
  };
  
  connection_state gprs_connection_state = waiting_for_sind;
  
  enum communication_state
  {
    idle,
    waiting_for_socket_ack,
    waiting_for_socket_connection,
    waiting_for_data_sent_ack,
    waiting_for_response,
    waiting_for_socket_close_ack
  };
  
  communication_state gprs_communication_state = idle;

/*

  We use I2C (Wire) for Pressure and Temp
  
*/

Adafruit_BMP085 bmp;

float temperature;
float pressure;

/*
  Some global variables used in parsing from the GPS module
*/

int  max_gps_buffer = 300;
String gps_parse_buffer;
String gps_read_buffer;

bool           gps_valid = false;
String         gps_utc = "";          //  UTC time and date
unsigned long  gps_utc_unix = 0;
String         gps_latitude = "";
String         gps_longitude = "";
String         gps_sog = "";          //  Speed over ground
String         gps_bearing_true ="";  //  Not magnetic!
String         gps_siv = "";          //  Number of satellites in view
String         gps_hdp = "";          //  Horizontal dillution of precision (how good is our fix)
String         gps_altitude = "";     //  We should be able to build a tide table out of this.

/*
  We keep a running average of position to do a little pythagorean calculation to tell if we're moving or not
*/

float           latitude_history[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float          longitude_history[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

/*
  Global variables for battery banks/chargers/pumps
*/

float  battery_one;
float  battery_two;
float  battery_three;

float  charger_one;
float  charger_two;
float  charger_three;

enum pump_state
{
  unknown,
  on,
  off
};

pump_state pump_one_state = off;
pump_state pump_two_state = off;
pump_state pump_three_state = off;

/*
    Setup SPI for barometric and temperature
*/


void bmp_setup()
{
  bmp.begin();    
}

void gps_setup()
{
  //
  //  Setup gps on serial device 3, and make it send only GGA and RMC NMEA sentences
  //

  Serial3.begin(4800);
  delay(1000);
  //Serial3.println("$PSTMNMEACONFIG,0,4800,66,1");
  Serial3.println("$PSRF103,00,00,02,01*26");  //  GGA ON every 2 seconds
  Serial3.println("$PSRF103,01,00,00,01*25");  //  GLL OFF
  Serial3.println("$PSRF103,02,00,00,01*26");  //  GSA OFF
  Serial3.println("$PSRF103,03,00,00,01*27");  //  GSV OFF
  Serial3.println("$PSRF103,04,00,02,01*22");  //  RMC ON every 2 seconds
  Serial3.println("$PSRF103,05,00,00,01*21");  //  VTG Off
}

void gprs_setup()
{
  //
  //  Setup GPRS cellular data on Serial1
  //
  
  Serial1.begin(9600);
  //gprs_sind = false;
  //gprs_ready_to_start = false;
  //gprs_tcp_connected = false;
  //gprs_readback = false;
  gprs_latest_message_to_send = "";
  //for(int i = 0; i < message_stack_max_count; i++)
  //{
  //  message_stack[i] = "";
  //}
  gprs_connection_state = waiting_for_sind;
  gprs_communication_state = idle;
  gprs_watchdog_timestamp = millis();
}

void init_eeprom_memory()
{

  int i;

  //
  //  This function is called only during factory reset or intial startup
  //
    
  //
  //  Set boot counter to 0
  //

  for(i = 6; i < 10; i++)
  {
    EEPROM.write(i,0);
  }
  
  //
  //  Set default id
  //
  
  EEPROM.write(10, 't');
  EEPROM.write(11, 'h');
  EEPROM.write(12, 'r');
  EEPROM.write(13, 'o');
  EEPROM.write(14, 'w');
  EEPROM.write(15, 'o');
  EEPROM.write(16, 'u');
  EEPROM.write(17, 't');
  
  //
  //  Set default server
  //
  

  EEPROM.write(18, 's');
  EEPROM.write(19, 'i');
  EEPROM.write(20, 'g');
  EEPROM.write(21, 'v');
  EEPROM.write(22, 'a');
  EEPROM.write(23, 'l');
  EEPROM.write(24, 'd');
  EEPROM.write(25, 'a');
  EEPROM.write(26, 's');
  EEPROM.write(27, 'o');
  EEPROM.write(28, 'n');
  EEPROM.write(29, '.');
  EEPROM.write(30, 'h');
  EEPROM.write(31, 'o');
  EEPROM.write(32, 'm');
  EEPROM.write(33, 'e');
  EEPROM.write(34, 'i');
  EEPROM.write(35, 'p');
  EEPROM.write(36, '.');
  EEPROM.write(37, 'n');
  EEPROM.write(38, 'e');
  EEPROM.write(39, 't');

  EEPROM.write(40, '\0');
  
  //
  //  Set default port
  //
  
  EEPROM.write(88, highByte(default_float_hub_server_port));
  EEPROM.write(89, lowByte(default_float_hub_server_port));
  
  //
  //  set default gprs apn
  //
  
  EEPROM.write(90, 'w');
  EEPROM.write(91, 'a');
  EEPROM.write(92, 'p');
  EEPROM.write(93, '.');
  EEPROM.write(94, 'c');
  EEPROM.write(95, 'i');
  EEPROM.write(96, 'n');
  EEPROM.write(97, 'g');
  EEPROM.write(98, 'u');
  EEPROM.write(99, 'l');
  EEPROM.write(100, 'a');
  EEPROM.write(101, 'r');
  
  EEPROM.write(102, '\0');
  
  //for(i = 0; i < default_gprs_apn.length(); i++)
  //{
  //  EEPROM.write(280 + i, default_gprs_apn[i]);
  //}
  //EEPROM.write(280+i, '\0');
  
  //
  //  set default gprs username
  //
  
  EEPROM.write(130, 'W');
  EEPROM.write(131, 'A');
  EEPROM.write(132, 'P');
  EEPROM.write(133, '@');
  EEPROM.write(134, 'C');
  EEPROM.write(135, 'I');
  EEPROM.write(136, 'N');
  EEPROM.write(137, 'G');
  EEPROM.write(138, 'U');
  EEPROM.write(139, 'L');
  EEPROM.write(140, 'A');
  EEPROM.write(141, 'R');
  EEPROM.write(142, 'G');
  EEPROM.write(143, 'P');
  EEPROM.write(144, 'R');
  EEPROM.write(145, 'S');
  EEPROM.write(146, '.');
  EEPROM.write(147, 'C');
  EEPROM.write(148, 'O');
  EEPROM.write(149, 'M');

  EEPROM.write(150, '\0');
  
  //for(i = 0; i < default_gprs_username.length(); i++)
  //{
  //  EEPROM.write(536 + i, default_gprs_username[i]);
  //}
  //EEPROM.write(536+i, '\0');
  
  //
  //  set default gprs password
  //
  
  EEPROM.write(170, 'C');
  EEPROM.write(171, 'I');
  EEPROM.write(172, 'N');
  EEPROM.write(173, 'G');
  EEPROM.write(174, 'U');
  EEPROM.write(175, 'L');
  EEPROM.write(176, 'A');
  EEPROM.write(177, 'R');
  EEPROM.write(178, '1');

  EEPROM.write(179, '\0');
  
  
  //for(i = 0; i < default_gprs_password.length(); i++)
  //{
  //  EEPROM.write(792 + i, default_gprs_password[i]);
  //}
  //EEPROM.write(792+i, '\0');
 
  //
  //  Reset EPROM means no stored data to upload either
  // 
  
  EEPROM.write(PUMP_STATE_COUNTER_LOCATION, 0);
  EEPROM.write(DETAILED_STATE_COUNTER_LOCATION, 0);
  
  //
  //  Set to some kind of default AES key
  //
  
  EEPROM.write(4080, 0x77);
  EEPROM.write(4081, 0x0A);
  EEPROM.write(4082, 0x8A);
  EEPROM.write(4083, 0x65);
  EEPROM.write(4084, 0xDA);
  EEPROM.write(4085, 0x15);
  EEPROM.write(4086, 0x6D);
  EEPROM.write(4087, 0x24);
  EEPROM.write(4088, 0xEE);
  EEPROM.write(4089, 0x2A);
  EEPROM.write(4090, 0x09);
  EEPROM.write(4091, 0x32);
  EEPROM.write(4092, 0x77);
  EEPROM.write(4093, 0x53);
  EEPROM.write(4094, 0x01);
  EEPROM.write(4095, 0x42);


  //
  //	In case our cellular/gprs unit came from factory with wrong GPS
  //	settings, set the BAND to 7 (GSM frequencies 850 & 1900 ).  The can
  //	be changed on the console with f= frequency command.
  //

  Serial1.println("AT+SBAND=7");
  

  //
  //  Do this last to show EEPROM set
  //

  for(int i = 0; i < 6; i++) {
    EEPROM.write(i, 42); }
  

}

void write_eeprom_memory()
{

  int i;

  //
  //  This just pushes current variables (e.g. float hub id) to EEPROM 
  //
    
  //
  //  Store id
  //
  
  for(i = 0; i < 8; i++)
  {
    EEPROM.write(10 + i, float_hub_id[i]);
  }
  
  //
  //  Store server
  //
  
  for(i = 0; i < min(float_hub_server.length(), 69); i++)
  {
    EEPROM.write(18 + i, float_hub_server[i]);
  }
  EEPROM.write(18+i, '\0');
  
  //
  //  Store port
  //
  
  EEPROM.write(88, highByte(float_hub_server_port));
  EEPROM.write(89, lowByte(float_hub_server_port));
  
  //
  //  Store gprs apn
  //
  
  for(i = 0; i < min(gprs_apn.length(),39); i++)
  {
    EEPROM.write(90 + i, gprs_apn[i]);
  }
  EEPROM.write(90+i, '\0');
  
  //
  //  Store username
  //
  
  for(i = 0; i < min(gprs_username.length(), 39); i++)
  {
    EEPROM.write(130 + i, gprs_username[i]);
  }
  EEPROM.write(130+i, '\0');
  
  //
  //  Store password
  //
  
  for(i = 0; i < min(gprs_password.length(), 18); i++)
  {
    EEPROM.write(170 + i, gprs_password[i]);
  }
  EEPROM.write(170+i, '\0');
  
  //
  //  Store AES key
  //
  
  for(i = 0; i < 16; i++)
  {
    EEPROM.write(4080 + i, float_hub_aes_key[i]);
  }
}

void read_eeprom_memory()
{
  char next_char;
  //
  //  As part of startup, set variables from non-volatile EEPROM
  //



  //
  //  Read, augment, the write back boot counter
  //

  boot_counter  = (unsigned int) EEPROM.read(9);
  boot_counter += (unsigned int) EEPROM.read(8) * 256;
  boot_counter += (unsigned int) EEPROM.read(7) * 65536;
  boot_counter += (unsigned int) EEPROM.read(6) * 16777216;

  boot_counter += 1;
  
  byte byte_zero   = byte(boot_counter);
  byte byte_one    = byte(boot_counter >> 8);
  byte byte_two    = byte(boot_counter >> 16);
  byte byte_three  = byte(boot_counter >> 24);
  
  EEPROM.write(9, byte_zero);
  EEPROM.write(8, byte_one);
  EEPROM.write(7, byte_two);
  EEPROM.write(6, byte_three);
  
  //
  //  Read floathub id
  //
  
  int i;
  float_hub_id = "";
  for(i = 0; i < 8; i++)
  {
    next_char = EEPROM.read(10 + i);
    float_hub_id += next_char;
  }
  
  //
  //  Read floathub server
  //
  
  float_hub_server = "";
  for(i = 0; i < 70; i++)
  {
    next_char = EEPROM.read(18 + i);    
    if(next_char == '\0')
    {
      break;
    }
    float_hub_server += next_char;
  }
   
  //
  //  Get port
  //
  
  float_hub_server_port  = EEPROM.read(89);
  float_hub_server_port += EEPROM.read(88) * 256;

  //
  //  Get gprs apn
  //
  
  gprs_apn = "";
  for(i = 0; i < 40; i++)
  {
    next_char = EEPROM.read(90 + i);
    if(next_char == '\0')
    {
      break;
    }
    gprs_apn += next_char;
  }
   
  //
  //  Get gprs username
  //
  
  gprs_username = "";
  for(i = 0; i < 40; i++)
  {
    next_char = EEPROM.read(130 + i);
    if(next_char == '\0')
    {
      break;
    }
    gprs_username += next_char;
  }
   
  //
  //  Get gprs password
  //
  
  gprs_password = "";
  for(i = 0; i < 19; i++)
  {
    next_char = EEPROM.read(170 + i);
    if(next_char == '\0')
    {
      break;
    }
    gprs_password += next_char;
  }
  
  //
  //  Get stored data counts;
  //
  
  detailed_state_count = EEPROM.read(DETAILED_STATE_COUNTER_LOCATION);
  pump_state_count = EEPROM.read(PUMP_STATE_COUNTER_LOCATION);
 
  if(detailed_state_count > 0)
  {
      //
      //  Need to make note of time of most recent state stored in eeprom
      //
      
      last_detailed_eeprom_write  = (unsigned int) EEPROM.read(DETAILED_STATE_START_LOCATION + (detailed_state_count * DETAILED_STATE_DATA_WIDTH) + 3);
      last_detailed_eeprom_write += (unsigned int) EEPROM.read(DETAILED_STATE_START_LOCATION + (detailed_state_count * DETAILED_STATE_DATA_WIDTH) + 2) * 256;
      last_detailed_eeprom_write += (unsigned int) EEPROM.read(DETAILED_STATE_START_LOCATION + (detailed_state_count * DETAILED_STATE_DATA_WIDTH) + 1) * 65536;
      last_detailed_eeprom_write += (unsigned int) EEPROM.read(DETAILED_STATE_START_LOCATION + (detailed_state_count * DETAILED_STATE_DATA_WIDTH) + 0) * 16777216;
  }
  
  //
  //  Read AES key
  //
  
  for(i = 0; i < 16; i++)
  {
    float_hub_aes_key[i] = EEPROM.read(4080 + i);
  }    
}  

void setup()
{

  bmp_setup();
  gps_setup();
  gprs_setup();
  
  //
  //  Setup main serial port for local data monitoring
  //
  
  Serial.begin(9600);
  delay(200);
  
  //
  //  Seed the randoim number generator
  //
  
  randomSeed(analogRead(0));
  
  //
  //  Setup LED pins, Red always on to show power
  //
  
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  
  
  //
  //  Handle EEPROM logic for persistant settings
  //
  
  int a = 0;
  for(int i = 0; i < 6; i++)
  {
    a = EEPROM.read(i);
    if(a != 42)
    {
      init_eeprom_memory();
      break;
    }
  }
  
  last_detailed_eeprom_write = 0;
  detailed_state_count = 0;
  pump_state_count = 0;
  read_eeprom_memory();
  
  //
  //  Do one sensor read for temperature and barometric so first console messages have this data
  //
  
  bmp_read();
  
  //
  //  Announce we are up
  //
  

  help_info("Up and running...");
  new_message = "(code=";
  new_message += CODE_MAJOR_VERSION;
  new_message += ".";
  new_message += CODE_MINOR_VERSION;
  
  new_message += ",b=";
  new_message += boot_counter;
  new_message += ",dq=";
  new_message += (int)detailed_state_count;
  new_message += ",pq=";
  new_message += (int)pump_state_count;
  new_message += ")";
  help_info(new_message);
  new_message = "";
  display_current_variables();  
  
}

void bmp_read()
{
  pressure = bmp.readPressure() * 0.000295300;
  temperature = (1.8 * bmp.readTemperature()) + 32 - 5.6;
}


void parse_gps_buffer_as_rmc()
{
  //Serial.println("%%%%%%%% entering parse_gps_buffer_as_rmc()");
  //Serial.print("Buffer: \"");
  //Serial.print(gps_parse_buffer);
  //Serial.println("\"");
  int time_start = 6;
  int time_break = gps_parse_buffer.indexOf('.', time_start + 1);
  int status_start = gps_parse_buffer.indexOf(',', time_start + 1);
  int lat_start =  gps_parse_buffer.indexOf(',', status_start + 1);
  int nors_start =  gps_parse_buffer.indexOf(',', lat_start + 1);
  int lon_start =  gps_parse_buffer.indexOf(',', nors_start + 1);
  int wore_start = gps_parse_buffer.indexOf(',', lon_start + 1);
  int sog_start = gps_parse_buffer.indexOf(',', wore_start + 1);
  int tmg_start = gps_parse_buffer.indexOf(',', sog_start + 1); 
  int date_start = gps_parse_buffer.indexOf(',', tmg_start + 1);
  int var_start =  gps_parse_buffer.indexOf(',', date_start + 1);
  
  if( time_start < 0    ||    time_start >= gps_parse_buffer.length()    ||
      time_break < 0    ||    time_break >= gps_parse_buffer.length()    ||
      status_start < 0  ||    status_start >= gps_parse_buffer.length()  ||
      lat_start < 0     ||    lat_start >= gps_parse_buffer.length()     ||
      nors_start < 0    ||    nors_start >= gps_parse_buffer.length()    ||
      lon_start < 0     ||    lon_start >= gps_parse_buffer.length()     ||
      wore_start < 0    ||    wore_start >= gps_parse_buffer.length()    ||
      sog_start < 0     ||    sog_start >= gps_parse_buffer.length()     ||
      tmg_start < 0     ||    tmg_start >= gps_parse_buffer.length()     ||
      date_start < 0    ||    date_start >= gps_parse_buffer.length()    ||
      var_start < 0     ||    var_start >= gps_parse_buffer.length()     )
  {
#ifdef GPS_DEBUG_ON
      debug_info("Bad RMC string (below), will not parse");
      debug_info(gps_parse_buffer);
#endif
      return;
  }
      
     
  gps_utc  = gps_parse_buffer.substring(time_start + 1, time_break);
  gps_utc += gps_parse_buffer.substring(date_start + 1, var_start - 2);
  gps_utc += "20";
  gps_utc += gps_parse_buffer.substring(var_start - 2, var_start);
  
  String is_valid = gps_parse_buffer.substring(status_start + 1, lat_start);
  
  /*
    Break GPS time/date into integer parts
  */

  memset(temp_string, 0, 20 * sizeof(char));
  gps_utc.substring(0,2).toCharArray(temp_string, 3);
  int  gps_time_hour = atoi(temp_string);
  
  memset(temp_string, 0, 20 * sizeof(char));
  gps_utc.substring(2,4).toCharArray(temp_string, 3);
  int  gps_time_minute = atoi(temp_string);

  memset(temp_string, 0, 20 * sizeof(char));
  gps_utc.substring(4,6).toCharArray(temp_string, 3);
  int  gps_time_second = atoi(temp_string);
  
  memset(temp_string, 0, 20 * sizeof(char));
  gps_utc.substring(6,8).toCharArray(temp_string, 3);  
  int  gps_time_day = atoi(temp_string);
  
  memset(temp_string, 0, 20 * sizeof(char));
  gps_utc.substring(8,10).toCharArray(temp_string, 3);
  int  gps_time_month = atoi(temp_string);

  memset(temp_string, 0, 20 * sizeof(char));
  gps_utc.substring(10,14).toCharArray(temp_string, 5);
  int  gps_time_year = atoi(temp_string);
  
  /*
    As long as the year is "reasonable", assume we have something close to right time
  */

  if(gps_time_year > 2010)
  {
    setTime(gps_time_hour,gps_time_minute,gps_time_second,gps_time_day,gps_time_month,gps_time_year);
    gps_utc_unix = now();
    //Serial.print("UNIX time: ");
    //Serial.println(now());
    //slide_memory(DETAILED_STATE_START_LOCATION, DETAILED_STATE_NUMB_LOCATIONS, DETAILED_STATE_DATA_WIDTH);
    //slide_memory(PUMP_STATE_START_LOCATION, PUMP_STATE_NUMB_LOCATIONS, PUMP_STATE_DATA_WIDTH);
    //Serial.println(now());
  }
  if(is_valid == "A")
  {
    if(!gps_valid) 
    {
#ifdef GPS_DEBUG_ON
      debug_info("Got gps fix");
#endif
    }
    gps_valid = true;
  }
  else
  {
    if(gps_valid)
    {
#ifdef GPS_DEBUG_ON
      debug_info("Lost gps fix");
#endif
    }
    gps_valid = false;
  }
  

  gps_sog  = gps_parse_buffer.substring(sog_start + 1, tmg_start);
  gps_bearing_true  = gps_parse_buffer.substring(tmg_start + 1, date_start);
 
  //
  //  Use speed over ground to 
  //
 

  if(gps_valid)
  {
    memset(temp_string, 0, 20 * sizeof(char));
    gps_latitude.substring(0,2).toCharArray(temp_string, 3);
    int  int_one = atoi(temp_string);
  
    memset(temp_string, 0, 20 * sizeof(char));
    gps_latitude.substring(3,11).toCharArray(temp_string, 9);
    float  float_one = atof(temp_string);
    
    float current_latitude = int_one + (float_one / 60.0);
    
    //Serial.print("Lat: "); Serial.println(current_latitude,5);

    memset(temp_string, 0, 20 * sizeof(char));
    gps_longitude.substring(0,3).toCharArray(temp_string, 4);
    int_one = atoi(temp_string);

    memset(temp_string, 0, 20 * sizeof(char));
    gps_longitude.substring(4,12).toCharArray(temp_string, 9);
    float_one = atof(temp_string);

    float current_longitude = int_one + (float_one / 60.0);
    //Serial.print("Lon: "); Serial.println(current_longitude,5);
    
    
    //
    //  Calculate average of past positions
    //
    
    float average_latitude = 0.0;
    float average_longitude = 0.0;
    for(int i = 0; i < 5; i++)
    {
      average_latitude += latitude_history[i];
      average_longitude += longitude_history[i];
    }
    
    average_latitude = average_latitude / 5.0;
    average_longitude = average_longitude / 5.0;
    
    //Serial.print("Avg: "); Serial.print(average_latitude,5); Serial.print(", "); Serial.println(average_longitude,5);

    //
    //  Find pythagorean distance from average
    //
    
    float distance = sqrt(pow(average_latitude - current_latitude, 2) + pow(average_longitude - current_longitude, 2));
    //Serial.print("Distance: "); Serial.println(distance, 10);
    if(distance > 0.0001)  //  1/10,000th of a degree is roughly 35 feet
    {
      currently_active = true;
    }
    else
    {
      currently_active = false;
    }


    //
    //  Slide averages
    //
    
    for(int i = 0; i < 4; i++)
    {
      latitude_history[i] = latitude_history[i + 1];
      longitude_history[i] = longitude_history[i + 1];
    }
    latitude_history[4] = current_latitude;
    longitude_history[4] = current_longitude;
  }
  else  
  {    
    currently_active = false;
  }
 
  //Serial.println("%%%%%%%% leaving parse_gps_buffer_as_rmc()");
 
}

void parse_gps_buffer_as_gga()
{

  //Serial.println("%%%%%%%% entering parse_gps_buffer_as_gga()");
  //Serial.print("Buffer: \"");
  //Serial.print(gps_parse_buffer);
  //Serial.println("\"");
  int time_start = 6;
  int lat_start =  gps_parse_buffer.indexOf(',', time_start + 1);
  int nors_start =  gps_parse_buffer.indexOf(',', lat_start + 1);
  int lon_start =  gps_parse_buffer.indexOf(',', nors_start + 1);
  int wore_start = gps_parse_buffer.indexOf(',', lon_start + 1);
  int qual_start = gps_parse_buffer.indexOf(',', wore_start + 1);
  int siv_start = gps_parse_buffer.indexOf(',', qual_start + 1); 
  int hdp_start = gps_parse_buffer.indexOf(',', siv_start + 1);
  int alt_start =  gps_parse_buffer.indexOf(',', hdp_start + 1);
  int altu_start = gps_parse_buffer.indexOf(',', alt_start + 1);
     
  if( time_start < 0 ||   time_start >= gps_parse_buffer.length()   ||
      lat_start < 0  ||   lat_start >= gps_parse_buffer.length()    ||
      nors_start < 0 ||   nors_start >= gps_parse_buffer.length()   ||
      lon_start < 0  ||   lon_start >= gps_parse_buffer.length()    ||
      wore_start < 0 ||   wore_start >= gps_parse_buffer.length()   ||
      qual_start < 0 ||   qual_start >= gps_parse_buffer.length()   ||
      siv_start < 0  ||   siv_start >= gps_parse_buffer.length()    ||
      hdp_start < 0  ||   hdp_start >= gps_parse_buffer.length()    ||
      alt_start < 0  ||   alt_start >= gps_parse_buffer.length()    ||
      altu_start < 0 ||   altu_start >= gps_parse_buffer.length())
  {
#ifdef GPS_DEBUG_ON
      debug_info("Bad GGA string (below), will not parse");
      debug_info(gps_parse_buffer);
#endif
      return;
  }
      
    
  gps_latitude  = gps_parse_buffer.substring(lat_start + 1, lat_start + 3);
  gps_latitude += " ";
  gps_latitude += gps_parse_buffer.substring(lat_start + 3, nors_start);
  while(gps_latitude.length() < 11)
  {
    gps_latitude += "0";
  }
  gps_latitude += gps_parse_buffer.substring(nors_start + 1, lon_start);
    
  gps_longitude  = gps_parse_buffer.substring(lon_start + 1, lon_start + 4);
  gps_longitude += " ";
  gps_longitude += gps_parse_buffer.substring(lon_start + 4, wore_start);
  while(gps_longitude.length() < 12)
  {
    gps_longitude += "0";
  }
  gps_longitude += gps_parse_buffer.substring(wore_start + 1, qual_start);
  
  gps_siv  = gps_parse_buffer.substring(siv_start + 1, hdp_start);
  gps_hdp  = gps_parse_buffer.substring(hdp_start + 1, alt_start);
  gps_altitude  = gps_parse_buffer.substring(alt_start + 1, altu_start);
  
  //Serial.println("%%%%%%%% leaving parse_gps_buffer_as_gga()");
}

void gps_read()
{

#ifdef GPS_DEBUG_ON
  debug_info("------------------------- GPS START BUFFER ---------------------------------------------");
  debug_info(gps_read_buffer);
#endif
  bool new_data = false;
  while(Serial3.available() && gps_read_buffer.length() < max_gps_buffer)
  {
     int incoming_byte = Serial3.read();
     if(incoming_byte > 31 && incoming_byte < 127)
     {
       gps_read_buffer += String((char) incoming_byte);
       new_data = true;
     }
  }
  
  int asterix_cut_point = gps_read_buffer.indexOf('*');

  if(gps_read_buffer.length() > 0 && new_data == true)
  {

#ifdef GPS_DEBUG_ON
    debug_info("------------------------- GPS MID BUFFER ---------------------------------------------");
    debug_info(gps_read_buffer);
#endif
  }
  
  if(asterix_cut_point > -1)
  {
    int rmc_cut_point = gps_read_buffer.indexOf("$GPRMC,");
    int gga_cut_point = gps_read_buffer.indexOf("$GPGGA,");

    if(rmc_cut_point > -1 && rmc_cut_point < asterix_cut_point)
    {
      gps_parse_buffer = gps_read_buffer.substring(rmc_cut_point, asterix_cut_point + 1);
      gps_read_buffer = gps_read_buffer.substring(min(asterix_cut_point + 3,gps_read_buffer.length()), gps_read_buffer.length());
      parse_gps_buffer_as_rmc();
      gps_parse_buffer = "";
    }
    else if(gga_cut_point > -1 && gga_cut_point < asterix_cut_point)
    {
      gps_parse_buffer = gps_read_buffer.substring(gga_cut_point, asterix_cut_point + 1);
      gps_read_buffer = gps_read_buffer.substring(min(asterix_cut_point + 3,gps_read_buffer.length()), gps_read_buffer.length());
      parse_gps_buffer_as_gga();
      gps_parse_buffer = "";
    }
    else
    {
      gps_read_buffer = gps_read_buffer.substring(min(asterix_cut_point + 3,gps_read_buffer.length()), gps_read_buffer.length());
    }
  }

  //
  //  Out of sync? Garbled? Throw it out, start again
  //
  
  if(gps_read_buffer.length() >= max_gps_buffer - 1 )
  {
    gps_read_buffer = "";
  }

#ifdef GPS_DEBUG_ON
  debug_info(gps_read_buffer);
  debug_info("------------------------- END GPS READ BUFFER -----------------------------------------");
#endif

}

void voltage_read()
{
  battery_one 	= analogRead(3) / 37.213; 
  battery_two 	= analogRead(2) / 37.213;
  battery_three = analogRead(1) / 37.213; 

  charger_one 	= analogRead(6) / 37.213; 
  charger_two 	= analogRead(5) / 37.213; 
  charger_three	= analogRead(4) / 37.213;
}


void individual_pump_read(int pump_number, pump_state &state, int analog_input)
{
  float pump_value = analogRead(analog_input) / 37.213;
  if(pump_value > 2.0)
  {
     if(state == unknown)
     {
       state = on;
     }
     else if(state == off)
     {
#ifdef PUMP_DEBUG_ON
       debug_info("Pump turned on!");
#endif
       new_message = "$FHB:";
       new_message += float_hub_id;
       new_message += "$";
       if(gps_valid || timeStatus() != timeNotSet)
       {
         new_message += ",U:";
         new_message += gps_utc;          
       }
       new_message += ",P:";
       new_message += pump_number;
       new_message += "1";
       
       queue_pump_message(1,true);
       echo_info(new_message);
       state = on;       
     }
  }
  else
  {
     if(state == unknown)
     {
       state = off;
     }
     else if(state == on)
     {
#ifdef PUMP_DEBUG_ON
       debug_info("Pump turned off!");
#endif
       new_message = "$FHB:";
       new_message += float_hub_id;
       new_message += "$";

       if(gps_valid|| timeStatus() != timeNotSet)
       {
         new_message += ",U:";
         new_message += gps_utc;          
       }
       new_message += ",P:";
       new_message += pump_number;
       new_message += "0";
       queue_pump_message(1,0);
       echo_info(new_message);
       state = off;
     }
  }
}


void pump_read()
{
  individual_pump_read(1, pump_one_state, 9);
  individual_pump_read(2, pump_two_state, 8);
  individual_pump_read(3, pump_three_state, 7);
}


void report_state(bool console_only)
{
  //Serial.println("%%%%%%%% entering report_state");
  //
  //   Fill the latest GPRS message with the latest data to send
  //
  
  if(console_only)
  {
    new_message = "$FHC:";
  }
  else
  {
    new_message = "$FHA:";
  }

  new_message += float_hub_id;
  new_message += "$";
  if(gps_valid == true || timeStatus() != timeNotSet )
  {
     new_message += ",U:";
     new_message += gps_utc;
  }
  
  new_message += ",T:";
  memset(temp_string, 0, 20 * sizeof(char));
  dtostrf(temperature,5,2,temp_string);
  new_message += String(temp_string);

  new_message += ",P:";
  memset(temp_string, 0, 20 * sizeof(char));
  dtostrf(pressure,4,2,temp_string);
  new_message += String(temp_string);
  


  if(gps_valid == true)
  {
    new_message += ",L:";
    new_message += gps_latitude;
    
    new_message += ",O:";
    new_message += gps_longitude;

    new_message += ",A:";
    new_message += gps_altitude;

    new_message += ",H:";
    new_message += gps_hdp;
    
    new_message += ",S:";
    new_message += gps_sog;

    new_message += ",B:";
    new_message += gps_bearing_true;
  }

  if(gps_siv.length() > 0)
  {
    new_message += String(",N:");
    new_message += gps_siv;
  }

  if(battery_one > 0.2)
  {
    new_message += ",V:";
    memset(temp_string, 0, 20 * sizeof(char));
    dtostrf(battery_one,4,2,temp_string);
    new_message += String(temp_string);
  }

  if(battery_two > 0.2)
  {
 
    new_message += ",W:";
    memset(temp_string, 0, 20 * sizeof(char));
    dtostrf(battery_two,4,2,temp_string);
    new_message += String(temp_string);
  }

  if(battery_three > 0.2)
  {
    new_message += ",X:";
    memset(temp_string, 0, 20 * sizeof(char));
    dtostrf(battery_three,4,2,temp_string);
    new_message += String(temp_string);
  }

  if(charger_one > 0.2)
  {
    new_message += ",C:";
    memset(temp_string, 0, 20 * sizeof(char));
    dtostrf(charger_one,4,2,temp_string);
    new_message += String(temp_string);
  }

  if(charger_two > 0.2)
  {
    new_message += ",D:";
    memset(temp_string, 0, 20 * sizeof(char));
    dtostrf(charger_two,4,2,temp_string);
    new_message += String(temp_string);
  }

  if(charger_three > 0.2)
  {
    new_message += ",E:";
    memset(temp_string, 0, 20 * sizeof(char));
    dtostrf(charger_three,4,2,temp_string);
    new_message += String(temp_string);
  }


  if(!console_only)
  {
    queue_detailed_message();
  }
  echo_info(new_message);
  //Serial.println("%%%%%%%% leaving report_state");
}

/*
void print_message_queue()
{
    //debug_info("** message queue dump **");
    //debug_info(gprs_latest_message_to_send);

    //for(int i = 0; i < message_stack_max_count; i++)
    //{
    //  if(message_stack[i].length() > 0)
    //  {
    //    debug_info(message_stack[i]);
    //  }
    //}

}
*/

void pop_off_pump_message()
{
  gprs_latest_message_to_send = "";
  if(pump_state_count < 1)
  {
    return;
  }
  
  int which_location = PUMP_STATE_START_LOCATION + (PUMP_STATE_DATA_WIDTH * (pump_state_count - 1));
  int handy = 0;

  unsigned long an_unsigned_long = (unsigned int) EEPROM.read(which_location + 3);
  an_unsigned_long += (unsigned int) EEPROM.read(which_location + 2) * 256;
  an_unsigned_long += (unsigned int) EEPROM.read(which_location + 1) * 65536;
  an_unsigned_long += (unsigned int) EEPROM.read(which_location + 0) * 16777216;
    
  gprs_latest_message_to_send += "$FHB:";
  gprs_latest_message_to_send += float_hub_id;
  gprs_latest_message_to_send += "$,U:";
  
  //
  //  Reconstitute date time string
  //
  
  handy = hour(an_unsigned_long);
  if(handy < 10)
  {
     gprs_latest_message_to_send += "0";
  }
  gprs_latest_message_to_send += handy;
  handy = minute(an_unsigned_long);
  if(handy < 10)
  {
     gprs_latest_message_to_send += "0";
  }
  gprs_latest_message_to_send += handy;
  handy = second(an_unsigned_long);
  if(handy < 10)
  {
     gprs_latest_message_to_send += "0";
  }
  gprs_latest_message_to_send += handy;
  handy = day(an_unsigned_long);
  if(handy < 10)
  {
     gprs_latest_message_to_send += "0";
  }
  gprs_latest_message_to_send += handy;
  handy = month(an_unsigned_long);
  if(handy < 10)
  {
     gprs_latest_message_to_send += "0";
  }
  gprs_latest_message_to_send += handy;
  gprs_latest_message_to_send += year(an_unsigned_long);
  
  //
  //  pump number, then on or off
  //
  
  handy = EEPROM.read(which_location + 4);
  gprs_latest_message_to_send += ",P:";
  if((handy & B00000100) && (handy & B00000010))
  {
    gprs_latest_message_to_send += "3";
  }
  else if(handy & B00000100)
  {
    gprs_latest_message_to_send += "2";
  }
  else if(handy & B00000010)
  {
    gprs_latest_message_to_send += "1";
  }
  
  if(handy & B00000001)
  {
     gprs_latest_message_to_send += "1";
  }
  else
  {
     gprs_latest_message_to_send += "0";
  }
  
  pump_state_count = pump_state_count - 1;
  EEPROM.write(PUMP_STATE_COUNTER_LOCATION, pump_state_count);
  encode_latest_message_to_send();
}


void pop_off_detailed_message()
{
  gprs_latest_message_to_send = "";
  if(detailed_state_count < 1)
  {
    return;
  }
  
  int which_location = DETAILED_STATE_START_LOCATION + (DETAILED_STATE_DATA_WIDTH * (detailed_state_count - 1));
  int handy = 0;

  unsigned long an_unsigned_long = (unsigned int) EEPROM.read(which_location + 3);
  an_unsigned_long += (unsigned int) EEPROM.read(which_location + 2) * 256;
  an_unsigned_long += (unsigned int) EEPROM.read(which_location + 1) * 65536;
  an_unsigned_long += (unsigned int) EEPROM.read(which_location + 0) * 16777216;
  
  /*
  Serial.print(an_unsigned_long); Serial.println(" built from: ");
  Serial.print(which_location + 3); Serial.print(" --> "); Serial.print((unsigned int) EEPROM.read(which_location + 3)); Serial.println(" * 1");
  Serial.print(which_location + 2); Serial.print(" --> "); Serial.print((unsigned int) EEPROM.read(which_location + 2)); Serial.println(" * 256");
  Serial.print(which_location + 1); Serial.print(" --> "); Serial.print((unsigned int) EEPROM.read(which_location + 1)); Serial.println(" * 65536");
  Serial.print(which_location + 0); Serial.print(" --> "); Serial.print((unsigned int) EEPROM.read(which_location + 0)); Serial.println(" * 16777216");
  */
  
  gprs_latest_message_to_send += "$FHA:";
  gprs_latest_message_to_send += float_hub_id;
  gprs_latest_message_to_send += "$,U:";
  
  //
  //  Reconstitute date time string
  //
  
  handy = hour(an_unsigned_long);
  if(handy < 10)
  {
     gprs_latest_message_to_send += "0";
  }
  gprs_latest_message_to_send += handy;
  handy = minute(an_unsigned_long);
  if(handy < 10)
  {
     gprs_latest_message_to_send += "0";
  }
  gprs_latest_message_to_send += handy;
  handy = second(an_unsigned_long);
  if(handy < 10)
  {
     gprs_latest_message_to_send += "0";
  }
  gprs_latest_message_to_send += handy;
  handy = day(an_unsigned_long);
  if(handy < 10)
  {
     gprs_latest_message_to_send += "0";
  }
  gprs_latest_message_to_send += handy;
  handy = month(an_unsigned_long);
  if(handy < 10)
  {
     gprs_latest_message_to_send += "0";
  }
  gprs_latest_message_to_send += handy;
  gprs_latest_message_to_send += year(an_unsigned_long);
  
  //
  //  temperature
  //
  
  handy = EEPROM.read(which_location + 4) - 40 ;
  gprs_latest_message_to_send += ",T:";
  gprs_latest_message_to_send += handy;
  gprs_latest_message_to_send += ".";
  handy = EEPROM.read(which_location + 5);
  if(handy < 10)
  {
    gprs_latest_message_to_send += "0";
  }  
  gprs_latest_message_to_send += handy;
  
  //
  //  pressure
  //
  
  handy = EEPROM.read(which_location + 6);
  gprs_latest_message_to_send += ",P:";
  gprs_latest_message_to_send += handy;
  gprs_latest_message_to_send += ".";
  handy = EEPROM.read(which_location + 7);
  if(handy < 10)
  {
    gprs_latest_message_to_send += "0";
  }
  gprs_latest_message_to_send += handy;
  
  //
  //  N/S, E/W, and valid or not for lat/lon
  //
  
  handy = EEPROM.read(which_location + 20);
  float float_one = 0.0;

  if(handy & B00000100)
  {
    
    //
    //  Latitude
    //
  
    an_unsigned_long = EEPROM.read(which_location + 9);
    an_unsigned_long += EEPROM.read(which_location + 8) * 256;
    gprs_latest_message_to_send += ",L:";
    if(an_unsigned_long < 10)
    {
      gprs_latest_message_to_send += "0";
    }
    gprs_latest_message_to_send += an_unsigned_long;
    gprs_latest_message_to_send += " ";
    memset(temp_string, 0, 20 * sizeof(char));
    float_one = eeprom_read_float(which_location + 10);
    dtostrf(float_one,7,5,temp_string);
    gprs_latest_message_to_send += temp_string;
    if(handy & B00000001)
    {
      gprs_latest_message_to_send += "N";
    }
    else
    {
      gprs_latest_message_to_send += "S";
    }
  
    //
    //  Longitude
    //
  
    an_unsigned_long = EEPROM.read(which_location + 15);
    an_unsigned_long += EEPROM.read(which_location + 14) * 256;
    gprs_latest_message_to_send += ",O:";
    if(an_unsigned_long < 10)
    {
      gprs_latest_message_to_send += "00";
    }
    else if(an_unsigned_long < 100)
    {
      gprs_latest_message_to_send += "0";
    }    
    gprs_latest_message_to_send += an_unsigned_long;
    gprs_latest_message_to_send += " ";
    memset(temp_string, 0, 20 * sizeof(char));
    float_one = eeprom_read_float(which_location + 16);
    dtostrf(float_one,7,5,temp_string);
    gprs_latest_message_to_send += temp_string;
    if(handy & B00000001)
    {
      gprs_latest_message_to_send += "W";
    }
    else
    {
      gprs_latest_message_to_send += "E";
    }
  
    //
    //  Altitude
    //
  
    handy = EEPROM.read(which_location + 22);
    handy += EEPROM.read(which_location + 21) * 256;
    gprs_latest_message_to_send += ",A:";
    gprs_latest_message_to_send += handy;
    gprs_latest_message_to_send += ".";
    handy=EEPROM.read(which_location + 23);
    gprs_latest_message_to_send += handy;
  
    //
    //  Horizontal precision
    //
  
    gprs_latest_message_to_send += ",H:";
    gprs_latest_message_to_send += (int) EEPROM.read(which_location + 24);
    gprs_latest_message_to_send += ".";
    gprs_latest_message_to_send += (int) EEPROM.read(which_location + 25);
  
    //
    //  Speed
    //
  
    gprs_latest_message_to_send += ",S:";
    gprs_latest_message_to_send += (int) EEPROM.read(which_location + 26);
    gprs_latest_message_to_send += ".";
    gprs_latest_message_to_send += (int) EEPROM.read(which_location + 27);
  
    //
    //  Bearing
    //
  
  
    handy = EEPROM.read(which_location + 29);
    handy += EEPROM.read(which_location + 28) * 256;
    gprs_latest_message_to_send += ",B:";
    gprs_latest_message_to_send += handy;
    gprs_latest_message_to_send += ".";
    handy=EEPROM.read(which_location + 30);
    if(handy < 10)
    {
      gprs_latest_message_to_send += "0";
    }
    gprs_latest_message_to_send += handy;
  }
  
  //
  //  Satellites in View
  //
  gprs_latest_message_to_send += ",N:";
  handy=EEPROM.read(which_location + 31);
  if(handy < 10)
  {
    gprs_latest_message_to_send += "0";
  }
  gprs_latest_message_to_send += handy;
  
  //
  //  Battery One
  //
  
   handy=EEPROM.read(which_location + 32);
   if(handy > 0)
   {
     gprs_latest_message_to_send += ",V:";
     float_one = ((handy + 1.0) / 100.0) * 12.0;
     memset(temp_string, 0, 20 * sizeof(char));
     dtostrf(float_one,4,2,temp_string);
     gprs_latest_message_to_send += temp_string; 
   }
  
  //
  //  Battery Two
  //
  
   handy=EEPROM.read(which_location + 33);
   if(handy > 0)
   {
     gprs_latest_message_to_send += ",W:";
     float_one = ((handy + 1.0) / 100.0) * 12.0;
     memset(temp_string, 0, 20 * sizeof(char));
     dtostrf(float_one,4,2,temp_string);
     gprs_latest_message_to_send += temp_string; 
   }
  
  //
  //  Battery Three
  //
  
   handy=EEPROM.read(which_location + 34);
   if(handy > 0)
   {
     gprs_latest_message_to_send += ",X:";
     float_one = ((handy + 1.0) / 100.0) * 12.0;
     memset(temp_string, 0, 20 * sizeof(char));
     dtostrf(float_one,4,2,temp_string);
     gprs_latest_message_to_send += temp_string; 
   }

  //
  //  Charger One
  //
  
  handy=EEPROM.read(which_location + 35);
  if(handy > 0)   
  {
    gprs_latest_message_to_send += ",C:";
    float_one = ((handy + 1.0) / 100.0) * 12.0;
    memset(temp_string, 0, 20 * sizeof(char));
    dtostrf(float_one,4,2,temp_string);
    gprs_latest_message_to_send += temp_string; 
  }
  
  //
  //  Charger Two
  //
  
  handy=EEPROM.read(which_location + 36);
  if(handy > 0)
  {
    gprs_latest_message_to_send += ",D:";
    float_one = ((handy + 1.0) / 100.0) * 12.0;
    memset(temp_string, 0, 20 * sizeof(char));
    dtostrf(float_one,4,2,temp_string);
    gprs_latest_message_to_send += temp_string; 
  }
  
  //
  //  Charger Three
  //
  
  handy=EEPROM.read(which_location + 37);
  if(handy > 0)
  {
    gprs_latest_message_to_send += ",E:";
    float_one = ((handy + 1.0) / 100.0) * 12.0;
    memset(temp_string, 0, 20 * sizeof(char));
    dtostrf(float_one,4,2,temp_string);
    gprs_latest_message_to_send += temp_string; 
  }
  
  
  detailed_state_count = detailed_state_count - 1;
  EEPROM.write(DETAILED_STATE_COUNTER_LOCATION, detailed_state_count);
  
  encode_latest_message_to_send();
}



void pop_off_message_queue()
{
 
  if(detailed_state_count > pump_state_count)
  {
    pop_off_detailed_message();    
  }
  else
  {
    pop_off_pump_message();
  }
}

float eeprom_read_float(int address)
{
  float value = 0.0;
  byte* p = (byte*)(void*)&value;
  for(int i = 0; i < 4; i++)
  {
    *p++ = EEPROM.read(address++);
  }
  return value;
}

void eeprom_write_float(int address, float value)
{
  byte* p = (byte*)(void*)&value;
  for(int i=0; i< 4; i++)
  {
    EEPROM.write(address++,*p++);
  } 
}

void write_pump_state(int which_position, int which_pump, bool is_on)
{


  which_position = which_position - 1;
  if(which_position < 0)
  {
    which_position = 0;
  }
  
  which_position = PUMP_STATE_START_LOCATION + (which_position * PUMP_STATE_DATA_WIDTH);
  
  //
  //  Write pump state values into EEPROM memory, starting with the UTC time/date
  //  
  
  byte byte_zero   = byte(gps_utc_unix);
  byte byte_one    = byte(gps_utc_unix >> 8);
  byte byte_two    = byte(gps_utc_unix >> 16);
  byte byte_three  = byte(gps_utc_unix >> 24);  
  
  EEPROM.write(which_position + 3, byte_zero);
  EEPROM.write(which_position + 2, byte_one);
  EEPROM.write(which_position + 1, byte_two);
  EEPROM.write(which_position + 0, byte_three); 


  //
  //  Now we set a byte to show which pump, and whether it went on or off
  //
  
  byte_zero = 0;
  if(which_pump == 1)
  {
    byte_zero |= B00000010;
  }
  else if (which_pump == 2)
  {
    byte_zero |= B00000100;
  }
  else if (which_pump == 3)
  {
    byte_zero |= B00000110;
  }
  
  if(is_on)
  {
     byte_zero |= B00000001;
  }

  EEPROM.write(which_position + 4, byte_zero);

}


void write_detailed_state(int which_position)
{

  which_position = which_position - 1;
  if(which_position < 0)
  {
    which_position = 0;
  }
  
  which_position = DETAILED_STATE_START_LOCATION + (which_position * DETAILED_STATE_DATA_WIDTH);
  
  //
  //  Write current state values into EEPROM memory
  //  
  
  byte byte_zero   = byte(gps_utc_unix);
  byte byte_one    = byte(gps_utc_unix >> 8);
  byte byte_two    = byte(gps_utc_unix >> 16);
  byte byte_three  = byte(gps_utc_unix >> 24);
 
  /*
  Serial.print(gps_utc_unix); Serial.println(" becomes: ");
  
  Serial.print(which_position + 3); Serial.print(" --> "); Serial.println((int)byte_zero);
  Serial.print(which_position + 2); Serial.print(" --> "); Serial.println((int)byte_one);
  Serial.print(which_position + 1); Serial.print(" --> "); Serial.println((int)byte_two);
  Serial.print(which_position + 0); Serial.print(" --> "); Serial.println((int)byte_three);
  */
  
  
  EEPROM.write(which_position + 3, byte_zero);
  EEPROM.write(which_position + 2, byte_one);
  EEPROM.write(which_position + 1, byte_two);
  EEPROM.write(which_position + 0, byte_three); 
  
  
  //  Temperature two int's for either side of the decimal place with + 40 to handle Farenheit down to -40
 
  byte_one = (int) (temperature + 40);
  byte_two = (int)((temperature - ((int) temperature)) * 100);
 
  EEPROM.write(which_position + 4, byte_one);
  EEPROM.write(which_position + 5, byte_two);
 
 //  Pressure two int's
 
  byte_one = (int) (pressure);
  byte_two = (int) ((pressure - ((int) pressure)) * 100);
 
  EEPROM.write(which_position + 6, byte_one);
  EEPROM.write(which_position + 7, byte_two);
 
 //  latitude is int (2 bytes) for whole degrees, then a float (4 bytes) for decimal minutes

      
  memset(temp_string, 0, 20 * sizeof(char));
  gps_latitude.substring(0,2).toCharArray(temp_string, 3);
  int  int_one = atoi(temp_string);
  
  EEPROM.write(which_position + 8, highByte(int_one));
  EEPROM.write(which_position + 9, lowByte(int_one));
  
  memset(temp_string, 0, 20 * sizeof(char));
  gps_latitude.substring(3,11).toCharArray(temp_string, 9);
  float  float_one = atof(temp_string);
  
  eeprom_write_float(which_position + 10, float_one);

  //  longitude is the same, int (2 bytes) for while degrees, 4 byte float for decimal minutes
  
  memset(temp_string, 0, 20 * sizeof(char));
  gps_longitude.substring(0,3).toCharArray(temp_string, 4);
  int_one = atoi(temp_string);
  
  EEPROM.write(which_position + 14, highByte(int_one));
  EEPROM.write(which_position + 15, lowByte(int_one));

  memset(temp_string, 0, 20 * sizeof(char));
  gps_longitude.substring(4,12).toCharArray(temp_string, 9);
  float_one = atof(temp_string);
  
  eeprom_write_float(which_position + 16, float_one);
  

  //  One bit for N versus South (lat), one for E versus W (long) + one bit for valid or not
 
  
  byte_zero = 0;
  
  if(gps_latitude[11] = 'N')
  {
    byte_zero |= B00000001;
  }
  if(gps_longitude[12] = 'W')
  {
    byte_zero |= B00000010;
  }
  if(gps_valid)
  {
    byte_zero |= B00000100;
  }
  
  EEPROM.write(which_position + 20, byte_zero);
  
  
  //  Altitude is a signed int (in meters), plus one decimal places in a byte. 
  
  memset(temp_string, 0, 20 * sizeof(char));
  gps_altitude.substring(0,gps_altitude.length()).toCharArray(temp_string, 19);
  float_one = atof(temp_string);
  int_one = (int) (float_one);
  byte_one = (int)((float_one - ((int) float_one)) * 10);
  
  EEPROM.write(which_position + 21, highByte(int_one));
  EEPROM.write(which_position + 22, lowByte(int_one));
  EEPROM.write(which_position + 23, byte_one);  
    

  //  Horizontal Dillution of precision    
    
  memset(temp_string, 0, 20 * sizeof(char));
  gps_hdp.substring(0,gps_hdp.length()).toCharArray(temp_string, 19);
  float_one = atof(temp_string);
  byte_zero = (int) (float_one);
  byte_one  = (int)((float_one - ((int) float_one)) * 10);
  EEPROM.write(which_position + 24, byte_zero);
  EEPROM.write(which_position + 25, byte_one);
  
  //  Speed over ground
    
  memset(temp_string, 0, 20 * sizeof(char));
  gps_sog.substring(0,gps_sog.length()).toCharArray(temp_string, 19);
  float_one = atof(temp_string);
  byte_zero = (int) (float_one);
  byte_one  = (int)((float_one - ((int) float_one)) * 100);
  EEPROM.write(which_position + 26, byte_zero);
  EEPROM.write(which_position + 27, byte_one);
    
  //  Bearing is an int (degrees ), plus first two decimal places in a byte. 
  
  memset(temp_string, 0, 20 * sizeof(char));
  gps_bearing_true.substring(0,gps_bearing_true.length()).toCharArray(temp_string, 19);
  float_one = atof(temp_string);
  int_one = (int) (float_one);
  byte_one = (int)((float_one - ((int) float_one)) * 100);
  
  EEPROM.write(which_position + 28, highByte(int_one));
  EEPROM.write(which_position + 29, lowByte(int_one));
  EEPROM.write(which_position + 30, byte_one);   
   
  //  Satellites in view is pretty easy
  
  memset(temp_string, 0, 20 * sizeof(char));
  gps_siv.substring(0,gps_siv.length()).toCharArray(temp_string, 19);
  byte_one = atoi(temp_string);
  EEPROM.write(which_position + 31, byte_one);
    
  //  Battery One, percentage of 12 volts

  if(battery_one > 0.2)
  {
    byte_one = (int) ((battery_one / 12.0) * 100);
    EEPROM.write(which_position + 32, byte_one);
  }
  else
  {
    EEPROM.write(which_position + 32, 0);
  }  
  
  //  Battery 2
  
  if(battery_two > 0.2)
  {
    byte_one = (int) ((battery_two / 12.0) * 100);
    EEPROM.write(which_position + 33, byte_one);
  }
  else
  {
    EEPROM.write(which_position + 33, 0);
  }  
    
    
  //  Battery 3
  
  if(battery_three > 0.2)
  {
    byte_one = (int) ((battery_three / 12.0) * 100);
    EEPROM.write(which_position + 34, byte_one);
  }
  else
  {
    EEPROM.write(which_position + 34, 0);
  }  
    
   //  Charge 1
  
  if(charger_one > 0.2)
  {
    byte_one = (int) ((charger_one / 12.0) * 100);
    EEPROM.write(which_position + 35, byte_one);
  }
  else
  {
    EEPROM.write(which_position + 35, 0);
  }  
    
   //  Charge 2
  
  if(charger_two > 0.2)
  {
    byte_one = (int) ((charger_two / 12.0) * 100);
    EEPROM.write(which_position + 36, byte_one);
  }
  else
  {
    EEPROM.write(which_position + 36, 0);
  }  
    
   //  Charge 3
  
  if(charger_three > 0.2)
  {
    byte_one = (int) ((charger_three / 12.0) * 100);
    EEPROM.write(which_position + 37, byte_one);
  }
  else
  {
    EEPROM.write(which_position + 37, 0);
  }  
    
     
  
    
  /*
    Break GPS time/date into integer parts
  */

  /*
  memset(temp_string, 0, 20 * sizeof(char));
  gps_utc.substring(0,2).toCharArray(temp_string, 3);
  int  gps_time_hour = atoi(temp_string);
  
  memset(temp_string, 0, 20 * sizeof(char));
  gps_utc.substring(2,4).toCharArray(temp_string, 3);
  int  gps_time_minute = atoi(temp_string);

  memset(temp_string, 0, 20 * sizeof(char));
  gps_utc.substring(4,6).toCharArray(temp_string, 3);
  int  gps_time_second = atoi(temp_string);
  
  memset(temp_string, 0, 20 * sizeof(char));
  gps_utc.substring(6,8).toCharArray(temp_string, 3);  
  int  gps_time_day = atoi(temp_string);
  
  memset(temp_string, 0, 20 * sizeof(char));
  gps_utc.substring(8,10).toCharArray(temp_string, 3);
  int  gps_time_month = atoi(temp_string);

  memset(temp_string, 0, 20 * sizeof(char));
  gps_utc.substring(10,14).toCharArray(temp_string, 5);
  int  gps_time_year = atoi(temp_string);
  */
}

void queue_detailed_message()
{
  //Serial.println("%%%%%%%% entering queue_message");
  //gprs_latest_message_to_send = a_message;
  
  if(gprs_latest_message_to_send.length() == 0)
  {
    gprs_latest_message_to_send = new_message;
    encode_latest_message_to_send();
  }
  else
  {
    //
    //  Need to store state in EEPROM if it's been an hour
    //  
//    if(last_detailed_eeprom_write - now() > 60 * 60)

    //Serial.print("Last eeprom write: "); Serial.println(last_detailed_eeprom_write);
    //Serial.print("Now: "); Serial.println(now());

    if(now() - last_detailed_eeprom_write > 60 * 60)
    {
        last_detailed_eeprom_write = now();
        if(detailed_state_count >= DETAILED_STATE_NUMB_LOCATIONS)
        {
          slide_memory(DETAILED_STATE_START_LOCATION, DETAILED_STATE_NUMB_LOCATIONS, DETAILED_STATE_DATA_WIDTH);
          write_detailed_state(detailed_state_count);
        }
        else
        {
          write_detailed_state(detailed_state_count + 1);
          detailed_state_count++;
          EEPROM.write(DETAILED_STATE_COUNTER_LOCATION,detailed_state_count);
        }
        //Serial.print("Queue: ");
        //Serial.println((int)detailed_state_count);       
    }
    
    
/*
    bool stored_somewhere = false;
    for(int i=0; i < message_stack_max_count; i++)
    {
      if(message_stack[i].length() == 0)
      {
        message_stack[i] = new_message;
        stored_somewhere = true;
        break;
      }
    }
    if(!stored_somewhere)
    {
      gprs_latest_message_to_send = message_stack[0];
      for(int i = 0; i < message_stack_max_count - 1; i++)
      {
        message_stack[i] = message_stack[i+1];
      }
      message_stack[message_stack_max_count - 1] = new_message;
    }
*/
  }
  
  //Serial.println("%%%%%%%% leaving queue_message");

}


void queue_pump_message(int which_pump, bool is_on)
{
  //Serial.println("%%%%%%%% entering queue_pump_message");
  
  if(gprs_latest_message_to_send.length() == 0)
  {
    gprs_latest_message_to_send = new_message;
    encode_latest_message_to_send();
  }
  else
  {
    //
    //  Need to store pump state in EEPROM
    //  

    if(pump_state_count >= PUMP_STATE_NUMB_LOCATIONS)
    {
      slide_memory(PUMP_STATE_START_LOCATION, PUMP_STATE_NUMB_LOCATIONS, PUMP_STATE_DATA_WIDTH);
      write_pump_state(pump_state_count, which_pump, is_on);
    }
    else
    {
      write_pump_state(pump_state_count + 1, which_pump, is_on);
      pump_state_count++;
      EEPROM.write(PUMP_STATE_COUNTER_LOCATION,pump_state_count);
    }
    //Serial.print("Pump Queue: ");
    //Serial.println((int)pump_state_count);       
  }
}


void slide_memory(int start, int how_many, int what_width)
{
  for(int i = start; i < start + ((how_many - 1) * what_width); i = i + what_width)
  {
    for(int j = 0; j < what_width; j++)
    {
      EEPROM.write(i+j,EEPROM.read(i+j+what_width));
    }
  }
}

void gprs_read()
{
  //Serial.println("%%%%%% Entering GPRS read %%%%%%%");
  if(gprs_communications_on == false)
  {
    //Serial.println("%%%%%%%% Quiting GPRS read %%%%%%%%");
    return;
  }
  unsigned long current_timestamp = millis();
  String gprs_read_buffer;

  while(Serial1.available() && gprs_read_buffer.length() < max_gprs_buffer)
  {
     gprs_read_buffer += (char) Serial1.read();
  }
  if(gprs_read_buffer.length() > 0)
  {
    
#ifdef GPRS_DEBUG_ON
    debug_info("\\/ GPRS-DEBUG  \\/");
    debug_info(gprs_read_buffer);
#endif
              
    if(gprs_read_buffer.indexOf("CME ERROR") > 0)
    {
#ifdef GPRS_DEBUG_ON
      debug_info("*** Rebooting GPRS   ***");
#endif
      Serial1.println("AT+CFUN=0,1");
      gprs_connection_state = waiting_for_sind;
      gprs_communication_state = idle;
      gprs_watchdog_timestamp = current_timestamp;
    }   
    else if(gprs_read_buffer.indexOf("+SIND: 4") > 0)
    {
      gprs_connection_state = waiting_for_gprs_attachment;
      gprs_watchdog_timestamp = current_timestamp;
      Serial1.println("AT+CGATT?");
    }
    else if(gprs_read_buffer.indexOf("+SIND:") > 0)
    {
      gprs_connection_state = waiting_for_sind;
    }
    else if(gprs_read_buffer.indexOf("+CGATT: 1") > 0)
    {
      String apn_string = "AT+CGDCONT=1,\"IP\",\"";
      apn_string += gprs_apn;
      apn_string += "\"";
      Serial1.println(apn_string);
      gprs_connection_state = waiting_for_pdp_ack;
      gprs_watchdog_timestamp = current_timestamp;
    }
    else if(gprs_connection_state == waiting_for_gprs_attachment)
    {
      Serial1.println("AT+CGATT?");
    }
    else if(gprs_connection_state == waiting_for_pdp_ack && gprs_read_buffer.indexOf("OK") > 0)
    {
      gprs_connection_state = waiting_for_pco_ack;
      gprs_watchdog_timestamp = current_timestamp;
      String pco_string = "AT+CGPCO=0,\"";
      pco_string += gprs_username;
      pco_string += "\",\"";
      pco_string += gprs_password;
      pco_string += "\",1";
      Serial1.println(pco_string);
    }
    else if(gprs_connection_state == waiting_for_pco_ack && gprs_read_buffer.indexOf("OK") > 0)
    {
      gprs_connection_state = waiting_for_pdp_context_active;
      gprs_watchdog_timestamp = current_timestamp;
      Serial1.println("AT+CGACT=1,1");
#ifdef GPRS_DEBUG_ON
      debug_info("Waiting on ACT=1,1...");
#endif
    }
    else if(gprs_connection_state == waiting_for_pdp_context_active && gprs_read_buffer.indexOf("OK") > 0)
    {
      String host_string = "AT+SDATACONF=1,\"TCP\",\"";
      host_string += float_hub_server;
      host_string += "\",";
      host_string += String(float_hub_server_port);      
      Serial1.println(host_string);
      gprs_connection_state = waiting_for_remote_host_ack;
      gprs_watchdog_timestamp = current_timestamp;
    }
    else if(gprs_connection_state == waiting_for_remote_host_ack && gprs_read_buffer.indexOf("OK") > 0)
    {
      Serial1.println("AT+SDATARXMD=1,1");
      gprs_connection_state =  waiting_for_string_format_ack;
      gprs_watchdog_timestamp = current_timestamp;
    }
    else if(gprs_connection_state == waiting_for_string_format_ack && gprs_read_buffer.indexOf("OK") > 0)
    {
      gprs_connection_state = we_be_connected;
      gprs_watchdog_timestamp = current_timestamp;
#ifdef GPRS_DEBUG_ON
      debug_info("We got internets!");
#endif
    }
  }
  
  if(gprs_connection_state == we_be_connected)
  {
      //
      //  OK, if I'm in here I know I _can_ send data, but what state of sending am I in (maybe already on the middle of doing so)
      //
      
      if(gprs_communication_state == idle && gprs_latest_message_to_send.length() < 1)
      {
        gprs_watchdog_timestamp = current_timestamp;
      }
      else if(gprs_communication_state == idle && gprs_latest_message_to_send.length() > 0)
      {
        Serial1.println("AT+SDATASTART=1,1");
        gprs_communication_state = waiting_for_socket_ack;
        gprs_watchdog_timestamp = current_timestamp;
      }
      else if(gprs_communication_state == waiting_for_socket_ack && gprs_read_buffer.indexOf("OK") > 0)
      {
        Serial1.println("AT+SDATASTATUS=1");
        gprs_communication_state = waiting_for_socket_connection;
        gprs_watchdog_timestamp = current_timestamp;
      }
      else if(gprs_communication_state == waiting_for_socket_connection)
      {
        if(gprs_read_buffer.indexOf("+SOCKSTATUS:  1,1") > 0)
        {
          String formatted_message = "AT+SSTRSEND=1,\"";
          formatted_message += gprs_latest_message_to_send;
          formatted_message += "\"";
          //Serial.print("Sending ");
          //Serial.println(formatted_message);
          Serial1.println(formatted_message);
          gprs_communication_state = waiting_for_data_sent_ack;
          gprs_watchdog_timestamp = current_timestamp;
        }
        else
        {
          Serial1.println("AT+SDATASTATUS=1");
        }
      }
      else if(gprs_communication_state == waiting_for_data_sent_ack && gprs_read_buffer.indexOf("OK") > 0)
      {                
        gprs_communication_state = waiting_for_response;
        gprs_watchdog_timestamp = current_timestamp;
      }
      else if(gprs_communication_state == waiting_for_response)
      {
        if(gprs_read_buffer.indexOf("$FHR$ OK") > 0)
        {
          pop_off_message_queue();
          if(gprs_latest_message_to_send.length() > 0)
          {
            String formatted_message = "AT+SSTRSEND=1,\"";
            formatted_message += gprs_latest_message_to_send;
            formatted_message += "\"";
            //Serial.print("Sending::::::::::");
            //Serial.println(formatted_message);
            Serial1.println(formatted_message);
            gprs_communication_state = waiting_for_data_sent_ack;
            gprs_watchdog_timestamp = current_timestamp;
          }
          else
          {
            gprs_communication_state = waiting_for_socket_close_ack;
            gprs_watchdog_timestamp = current_timestamp;
            Serial1.println("AT+SDATASTART=1,0");
          }
        }
        else //(gprs_read_buffer.indexOf("+STCPD:") > 0)
        {
          Serial1.println("AT+SDATAREAD=1");
        }
      }
      else if(gprs_communication_state == waiting_for_socket_close_ack && gprs_read_buffer.indexOf("OK") > 0)
      {
        gprs_communication_state = idle;
        gprs_watchdog_timestamp = current_timestamp;
      }
  }
  
  //
  //  Check the watchdog timer and give up all hope if it's
  //
  
  if(current_timestamp - gprs_watchdog_timestamp > gprs_watchdog_interval)
  {
#ifdef GPRS_DEBUG_ON
    debug_info("*** Rebooting GPRS (Watchdog)  ***");
#endif
    Serial1.println("AT+CFUN=0,1");
    gprs_connection_state = waiting_for_sind;
    gprs_communication_state = idle;
    gprs_watchdog_timestamp = current_timestamp;
  }
  //Serial.println("%%%%%%%% Done GPRS Read %%%%%%%%");
}

void console_read()
{
  //Serial.println("%%%%%%%% entering console_read");
  int i;
  String console_buffer;
  String display_string;
  while(Serial.available() && console_buffer.length() < 255)
  {
     console_buffer += (char) Serial.read();
  }
  if (console_buffer.length() > 0)
  {
/*
    if(console_buffer.charAt(0) == 'e')
    {
      local_console_mode = echo_mode;
      //help_info("Entering echo mode");
    }
    else if(console_buffer.charAt(0) == 'c')
    {
      local_console_mode = command_mode;
      //help_info("Entering command mode");
    }
    else if(console_buffer.charAt(0) == 'd')
    {
      local_console_mode = debug_mode;
      //help_info("Entering debug mode");
    }
*/
    if(console_buffer.charAt(0) == 'q')
    {
      gprs_communications_on = false;
      //help_info("GPRS off");
    }

    /*
    else if(console_buffer.charAt(0) == 'h')
    {
      help_info("  ");
      help_info("  Main Commands");
      help_info("  ");
      help_info("  c - Enter command mode");
      help_info("  d - Enter debug mode");
      help_info("  e - Enter echo (normal) mode");
      help_info("  h - Display this help");
      help_info("  v - Display current variable values");
      help_info("  ");
      help_info("  GPRS Communications");
      help_info("  ");
      help_info("  b - Brodcast (turn GPRS communications on)");
      help_info("  q - Quiet    (turn GPRS communications off)");
      help_info("  ");
      help_info("  Command Mode");
      help_info("  ");
      help_info("  i=xxxxxxxx - Set floathub user id to xxxxxxxx");
      help_info("  s=foo.com  - Set floathub server to foo.com");
      help_info("  p=bar      - Set floathub server port number to bar");
      help_info("  a=foo.wap  - Set GPRS APN to foo.wap");
      help_info("  u=foobar   - Set GPRS username to foobar");
      help_info("  w=barfoo   - Set GPRS password to barfoo");
      help_info("  k=0affee   - Set AES key to sixteen hex pairs");
      help_info("  f=7        - Set GSM frequency band");
      help_info("  factory    - factory reset");
      help_info("  ");
    }
    */
    else if(console_buffer.charAt(0) == 'b')
    {
      //help_info("GPRS on");
      gprs_communications_on = true;
#ifdef GPRS_DEBUG_ON
      debug_info("*** Rebooting GPRS (Watchdog)  ***");
#endif
      Serial1.println("AT+CFUN=0,1");
      gprs_connection_state = waiting_for_sind;
      gprs_communication_state = idle;
      gprs_watchdog_timestamp = millis();
    }
    else if(console_buffer.charAt(0) == 'v')
    {
      display_current_variables();
    }    
    else 
    {
      console_buffer = console_buffer.substring(0,console_buffer.indexOf('\r'));
      console_buffer = console_buffer.substring(0,console_buffer.indexOf('\n'));
      if(console_buffer.startsWith("factory"))
      {
        help_info("Doing factory reset ...");
        init_eeprom_memory();
        read_eeprom_memory();
        Serial1.println("AT+CFUN=0,1");
        gprs_connection_state = waiting_for_sind;
        gprs_communication_state = idle;
        gprs_watchdog_timestamp = millis();
      }
      else if(console_buffer.startsWith("s=") && console_buffer.length() > 2)
      {
        bool bad_chars = false;
        
        for(i=2; i < console_buffer.length(); i++)
        {
          if(console_buffer[i] < 45 ||
             console_buffer[i] > 122)
          {
            help_info("Bad input");
            bad_chars = true;
            break;
          }        
        }
        if(!bad_chars)
        {
          float_hub_server = console_buffer.substring(2);
          write_eeprom_memory();
          display_string = "s=";
          display_string += float_hub_server;
          help_info(display_string);
        }        
      }
      else if(console_buffer.startsWith("a=") && console_buffer.length() > 1)
      {
        bool bad_chars = false;
        
        for(i=2; i < console_buffer.length(); i++)
        {
          if(console_buffer[i] < 45 ||
             console_buffer[i] > 122)
          {
            help_info("Bad input");
            bad_chars = true;
            break;
          }        
        }
        if(!bad_chars)
        {
          gprs_apn = console_buffer.substring(2);
          write_eeprom_memory();
          display_string = "a=";
          display_string += gprs_apn;
          help_info(display_string);
        }        
      }
      else if(console_buffer.startsWith("u=") && console_buffer.length() > 1)
      {
        bool bad_chars = false;
        
        for(i=2; i < console_buffer.length(); i++)
        {
          if(console_buffer[i] < 45 ||
             console_buffer[i] > 122)
          {
            help_info("Bad input");
            bad_chars = true;
            break;
          }        
        }
        if(!bad_chars)
        {
          gprs_username = console_buffer.substring(2);
          write_eeprom_memory();
          display_string = "u=";
          display_string += gprs_username;
          help_info(display_string);
        }        
      }
      else if(console_buffer.startsWith("w=") && console_buffer.length() > 1)
      {
        bool bad_chars = false;
        
        for(i=2; i < console_buffer.length(); i++)
        {
          if(console_buffer[i] < 45 ||
             console_buffer[i] > 122)
          {
            help_info("Bad input");
            bad_chars = true;
            break;
          }        
        }
        if(!bad_chars)
        {
          gprs_password = console_buffer.substring(2);
          write_eeprom_memory();
          display_string = "w=";
          display_string += gprs_password;
          help_info(display_string);
        }        
      }
      else if(console_buffer.startsWith("p=") && console_buffer.length() > 2)
      {
          char temp_array[console_buffer.length() - 1];
          
          console_buffer.substring(2).toCharArray(temp_array, console_buffer.length() - 1);
          float_hub_server_port = atoi(temp_array);
          write_eeprom_memory();
          display_string = "p=";
          display_string += float_hub_server_port;
          help_info(display_string);
      }
      else if(console_buffer.startsWith("i=") && console_buffer.length() >= 10)
      {        
        
        bool bad_chars = false;
        
        for(i=0; i < 8; i++)
        {
          if(console_buffer[i+2] < 37 ||
             console_buffer[i+2] > 126)
          {
            help_info("Bad input");
            bad_chars = true;
            break;
          }        
        }
        if(!bad_chars)
        {
          float_hub_id = console_buffer.substring(2,10);
          write_eeprom_memory();
          display_string = "i=";
          display_string += float_hub_id;
          help_info(display_string);
        }
      }
      else if(console_buffer.startsWith("k=") && console_buffer.length() == 34)
      {        
        
        bool bad_chars = false;
        
        for(i=0; i < 32; i++)
        {
          if(
              console_buffer[i+2] < '0' &&
              console_buffer[i+2] > '9' &&
              console_buffer[i+2] < 'a' &&
              console_buffer[i+2] > 'f' 
            )          
          {
            help_info("Bad input");
            bad_chars = true;
            break;
          }        
        }
     
        if(!bad_chars)
        {
          display_string = "k=";
          for(i = 0; i < 16; i++)
          {
            int new_value = 0;
            if (console_buffer[2 + (i * 2)] <='9')
            {
                new_value = (console_buffer[2 + (i * 2)] - '0' ) * 16;
            }
            else
            {
                new_value = (console_buffer[2 + (i * 2)] - 'a' + 10) * 16;
            }
    
            if (console_buffer[3 + (i * 2)] <='9')
            {
                new_value += console_buffer[3 + (i * 2)] - '0';
            }
            else
            {
                new_value += console_buffer[3 + (i * 2)] - 'a' + 10;
            }

            float_hub_aes_key[i] = new_value;
            if(float_hub_aes_key[i] < 16)
            {
              display_string += "0";
            }
            display_string += String(float_hub_aes_key[i], HEX);
          }

          write_eeprom_memory();
          help_info(display_string);
        }    
      }
      else if(console_buffer.startsWith("f=") && console_buffer.length() > 2)
      {
          int new_value = -1;
          new_value = console_buffer[2];
          if(new_value < 0 || new_value > 8)
          {
            Serial1.print("AT+SBAND=");
            Serial1.println((char) new_value);
            display_string = "f=";
            display_string += (char) new_value;
            help_info(display_string);
          }
          else
          {
            help_info("Bad input");
          }
      }
    }
  }  
  //Serial.println("%%%%%%%% leaving console_read");
}

void display_current_variables()
{
  int i;
  String line = "i=";
  line += float_hub_id;
  help_info(line);
  
  line = "s=";
  line += float_hub_server;
  help_info(line);
  
  line = "p=";
  line += float_hub_server_port;
  help_info(line);
  
  line = "a=";
  line += gprs_apn;
  help_info(line);
  
  line = "u=";
  line += gprs_username;
  help_info(line);
  
  line = "w=";
  line += gprs_password;
  help_info(line);
  
  line = "k=";
  for(i = 0; i < 16; i++)
  {
    if(float_hub_aes_key[i] < 16)
    {
      line += "0";
    }
    line += String(float_hub_aes_key[i], HEX);
    line += " ";
  }
  help_info(line);
}

void help_info(String some_info)
{
   Serial.print("$FHH:");
   Serial.print(float_hub_id);
   Serial.print("$    ");
   Serial.println(some_info);
}

void echo_info(String some_info)
{
  Serial.println(some_info);
}

void debug_info(String some_info)
{
  some_info.replace('\n','|');
  some_info.replace('\r','|');
  Serial.print("$FHD:");
  Serial.print(float_hub_id);
  Serial.print("$    ");
  Serial.println(some_info);
}

void update_leds()
{

  if(gps_valid)
  {
    digitalWrite(YELLOW_LED, HIGH);
  }
  else
  {
    digitalWrite(YELLOW_LED, LOW);
  }

  if(gprs_communications_on == false)
  {
    digitalWrite(GREEN_LED, LOW);
    return;
  } 

  //
  //  Match Green to state
  //
  
  if(gprs_connection_state == we_be_connected)
  {
    if(gprs_communication_state == idle)
    {
      digitalWrite(GREEN_LED, HIGH);
    }
    else
    {
      if(green_led_state == true)
      {
        digitalWrite(GREEN_LED, LOW);
        green_led_state = false;
      }
      else
      {
        digitalWrite(GREEN_LED, HIGH);
        green_led_state = true;
      }
    }
  }
  else
  {
    digitalWrite(GREEN_LED, LOW);
  }
}

void encode_latest_message_to_send()
{


  //
  //  Take the latest message that is set up to go over GPRS and AES encode it, then Base-64 convert it
  //
    
  int i;
  int cipher_length, base64_length;
  //AES aes;
 
  aes.set_key (float_hub_aes_key, 128) ;
  
  //byte iv[16];
  //byte volatile_iv[16];
  //byte plain[256];
  //byte cipher[256];

  for(i = 0; i < 16; i++)
  {
    iv[i] = random(0, 256);
    volatile_iv[i] = iv[i];
  }
  

  //
  //  Copy current gprs message to plain
  //
  
  
  for(i = 0; i < gprs_latest_message_to_send.length(); i++)
  {
    plain[i] = gprs_latest_message_to_send.charAt(i);
  }
  cipher_length = i;
  if(cipher_length % 16 == 0)
  {
    for(i = 0; i < 16; i++)
    {
      plain[cipher_length + i] = 16;
    }
    cipher_length += 16;
  }
  else
  {
    for(i = 0; i < 16 - (cipher_length % 16); i++)
    {
      plain[cipher_length + i] = 16 - (cipher_length % 16);
    }
    cipher_length += i;
  }

  //
  //  Encrypt current message with AES CBC
  // 
 
  aes.cbc_encrypt (plain, cipher,  cipher_length / 4, volatile_iv) ;
  
  //
  //  Now we reuse the plain text array to store cipher with the 
  //  initialization vector at the beginning
  //

  
  for(i = 0; i < 16; i++)
  {
    plain[i] = iv[i];
  }
  
  for(i = 0; i < cipher_length; i++)
  {
    plain[16 + i] = cipher[i];
  }
  
  //
  //  Now convert that long line of bytes in plain to base 64, recylcing the cipher array to hold it
  //
  

  base64_length = base64_encode( (char *) cipher, (char *) plain, cipher_length + 16);
  cipher[base64_length] = '\0';
  gprs_latest_message_to_send = "$FHS:";
  gprs_latest_message_to_send += float_hub_id;
  gprs_latest_message_to_send += "$,";
  for(i = 0; i < base64_length; i++)
  {
    gprs_latest_message_to_send += (char) cipher[i];
  }
  
  //
  //  Clean up?
  // 
  aes.clean();  //  Not sure if that does anything useful or not.   

}


void loop()
{
  /*
      Obviously this is main execution loop. There are a number of values we read and actions we take base on timing
  */
   
  unsigned long current_timestamp = millis();
 
  if(current_timestamp - sensor_previous_timestamp >  sensor_sample_interval)
  {
    sensor_previous_timestamp = current_timestamp;
    bmp_read();
  } 

  if(current_timestamp - gps_previous_timestamp >  gps_interval)
  {
    gps_previous_timestamp = current_timestamp;
    gps_read();
  } 

  if(current_timestamp - voltage_previous_timestamp >  voltage_interval)
  {
    voltage_previous_timestamp = current_timestamp;
    voltage_read();
  } 

  if(current_timestamp - pump_previous_timestamp >  pump_interval)
  {
    pump_previous_timestamp = current_timestamp;
    pump_read();
  } 

  if(current_timestamp - gprs_previous_timestamp > gprs_interval)
  {
    gprs_previous_timestamp = current_timestamp;
    gprs_read();
  }
  
  if(current_timestamp - console_previous_timestamp > console_interval)
  {
    console_previous_timestamp = current_timestamp;
    console_read();
  }

  /*
      Reporting routines
  */
  
  if(currently_active == true)
  {
    if(current_timestamp - previous_active_timestamp >  active_reporting_interval)
    {
      previous_active_timestamp = current_timestamp;
      report_state(false);
    }
  }
  else
  {
    if(current_timestamp - previous_idle_timestamp >  idle_reporting_interval)
    {
      previous_idle_timestamp = current_timestamp;
      report_state(false);
    }
  }
  if(current_timestamp - previous_console_timestamp >  console_reporting_interval)
  {
    previous_console_timestamp = current_timestamp;
    report_state(true);
  }
  
  /*
    Handle LED's to show communication state
  */
  
  if(current_timestamp - led_previous_timestamp > led_update_interval)
  {
    led_previous_timestamp = current_timestamp;
    update_leds();
  }
}

