/* 
Copyright (C) 2011,2012 Robert DeSantis
hopluvr at gmail dot com

This file is part of LED Array for Arduino.
 
LED Array is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or (at your
option) any later version.
 
LED Array is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
License for more details.
 
You should have received a copy of the GNU General Public License
along with LED Array; see the file _COPYING.txt.  If not, write to
the Free Software Foundation, Inc., 59 Temple Place - Suite 330, Boston,
MA 02111-1307, USA.
*/

#include <avr/pgmspace.h>
#include <EEPROM.h>

#include "Tlc5940.h"
#include "tlc_fades.h"

#define DEBUG                   false

#define MAGIC_NUMBER            0xFC0A   // Used to validate EEPROM contents

#define PANEL_NONE              0
#define PANEL_BINARY            1
#define PANEL_TEXT              2

#define BUTTON_PANEL            PANEL_BINARY

#define VALUE_FADE              1
#define LED_CHANNELS            16
#define BRIGHTNESS_LEVELS       16 
#define MAX_SPEED               25

#define MAX_RUNTIME_MINUTES     10       // With no human intervention we shutdown after this period

#define BUTTON_1_INPUT          4
#define BUTTON_2_INPUT          5
#define BUTTON_3_INPUT          6

#define SPKR_INPUT              A0       // Speaker's analog input

// DMX structures and constants
typedef struct {
  uint8_t      dmx_program;          // Program number (0-18) * 10
  uint8_t      dmx_subprogram;       // Subprogram * 10
  uint8_t      dmx_color;            // Color Macro (0-19) * 10  
  uint8_t      dmx_red;              // Red intensity
  uint8_t      dmx_green;            // Green intensity
  uint8_t      dmx_blue;             // Blue intensity
  uint8_t      dmx_brightness;       // Brightness (0-15) * 10
  uint8_t      dmx_speed;            // Speed Macro (0-20) * 10   
} DMX_CHANNELS; 

#define DMX_RX_STATUS_PIN   8                        // DMX status output pin
#define DMX_BASE_ADDRESS    1                        // Default DMX base address
#define DMX_NUM_CHANNELS    sizeof(DMX_CHANNELS)     // Number of DMX channels

// Master Modes
typedef enum {
  mode_slave = 0,               // Slave mode
  mode_master = 1,              // Master mode (broadcasts program changes)
  mode_stand_alone = 2,         // Stand-alone unit
  mode_dmx = 3,                 // DMX receiver
  
  NUM_MASTER_MODES              // Number of master modes
} MasterMode;

// panel Modes

typedef enum {
  panel_program = 0,            // Select program
  panel_subprogram = 1,         // Subprogram
  panel_color = 2,              // Set color
  panel_speed = 3,              // Set speed
  panel_brightness = 4,         // Set brightness  
  panel_master = 5,             // Select master mode (slave, master, dmx, stand-alone)
  panel_dmx_address = 6,        // DMX address
  panel_switching = 7,          // Program auto switch (0=Never, else minutes)
  
  NUM_PANEL_OPTIONS             // Number of panel options              
} PanelMode;

// Animation flags

typedef enum {
  ANIMATE_NORMAL = 0x01,
  ANIMATE_REVERSE = 0x02
} AnimateFlags;

// Programs
typedef enum  {
  Off = 0,                      // All off  
  Test_mode = 1,                // Test all 
  Sound_activated = 2,          // Colors change speed based on the sound
  Strobe = 3,                   // All cells strobe with selectable rate
  Strobe_random = 4,            // Cells flash same color randomly
  Sparkle = 5,                  // Random patterns, color, speaker speed  
  Shapes = 6,                   // Show sequence of shapes
  Twirlers = 7,                 // Twirling / spinning 
  Chase = 8,                    // Lights that chase around the board
  Growing_spots = 9,            // Spots that "grow"
  Falling_light = 10,           // Field of color with falling white lights 
  Squares = 11,                 // Cycling squares 
  Lines = 12,                   // Lines
  Fill = 13,                    // Various fills
  Meters = 14,                  // Lame attempt a peak meters
  Patterns = 15,                // Patterns,
  ColorFade = 16,               // All cells cycle through a subset of colors with fading
  ColorFill = 17,               // All LEDs - same color 
  
  NUM_PROGRAMS                  // ALWAYS LAST - NUMBER OF PROGRAMS
} LightShow;

// Colors

#define COLORS           20

#define CUSTOM_COLOR     0
#define BLACK            1
#define LT_GRAY          2
#define GRAY             3
#define RED              4
#define YELLOW           8
#define LIME             9
#define GREEN            10
#define BLUE             14
#define MAGENTA          16
#define WHITE            18
#define AUTO_COLOR       19

prog_uchar color_table[COLORS][3] PROGMEM = {
  { 0x00, 0x00, 0x00 },                      // Custom color (user RGB)
  { 0x00, 0x00, 0x00 },                      // Black  
  { 0x10, 0x10, 0x10 },                      // Light Gray
  { 0x80, 0x80, 0x80 },                      // Gray
  { 0xFF, 0x00, 0x00 },                      // Red (P)
  { 0xFF, 0x20, 0x00 },                      // Blood orange    
  { 0xFF, 0x40, 0x00 },                      // Orange  
  { 0xFF, 0x40, 0x20 },                      // Salmon    
  { 0xFF, 0xFF, 0x00 },                      // Yellow
  { 0x30, 0xFF, 0x05 },                      // Lime
  { 0x00, 0xFF, 0x00 },                      // Green (P)
  { 0x00, 0xFF, 0x20 },                      // Teal
  { 0x10, 0xFF, 0x7F },                      // Cyan
  { 0x00, 0x80, 0x80 },                      // Light blue
  { 0x00, 0x00, 0xFF },                      // Blue (P)
  { 0x40, 0x00, 0xFF },                      // Violet  
  { 0xFF, 0x00, 0xFF },                      // Magenta
  { 0xFF, 0x00, 0x20 },                      // Hot pink
  { 0xFF, 0xFF, 0xFF },                      // White
  { 0x00, 0x00, 0x00 }                       // Automatic color selection  
};

const int GRAY_SCALE[] = { BLACK, LT_GRAY, GRAY, -1 };
    
volatile byte         brightness = BRIGHTNESS_LEVELS/2;  // 0-16
volatile LightShow    program = Test_mode;
volatile MasterMode   master_mode = mode_slave;
volatile PanelMode    button_mode = panel_program;
volatile uint32_t     master_shutdown = 0;
volatile uint16_t     fault = 0;
volatile boolean      restartCurrentProgram = false;
volatile byte         switch_minutes = 2;

typedef struct memory {
  uint16_t   magic_number;
  uint16_t   program;
  uint16_t   master_mode;
  byte       brightness;
  byte       switch_minutes;
  uint16_t   dmx_address;
  byte       sub_program[NUM_PROGRAMS];
  byte       speed[NUM_PROGRAMS];  
  byte       color_mode[NUM_PROGRAMS];  
} EEPROM_MEMORY;

extern void setupForMasterMode( void );

class ProgramControl;

class Program {
  
public:    
  LightShow        m_id;
  byte             m_max_subprogram;
  byte             m_subprogram;
  byte             m_color_mode;
  byte             m_speed;
  void             (*m_function)( ProgramControl * );
  boolean          m_includeInAuto;
  
 Program( LightShow id, byte max_subprogram, int color_mode, void (*function)( ProgramControl * ), bool includeInAuto ) :
    m_id(id),
    m_max_subprogram(max_subprogram),
    m_subprogram(0),
    m_color_mode(color_mode),
    m_speed(0),
    m_function(function),
    m_includeInAuto(includeInAuto)
  {}
  
  void incSubProgram(void) {
    m_subprogram = (m_subprogram + 1) % (m_max_subprogram);
  }
  
  void decSubProgram(void) {
    m_subprogram = (m_subprogram + m_max_subprogram - 1) % (m_max_subprogram);
  }
  
  inline int getSubProgram(void) {
    return m_subprogram;
  }
  
  inline void setSubProgram(int subp) {
    m_subprogram = subp;
  }   
  
  void incSpeed(void) {
    m_speed = (m_speed + 1) % (MAX_SPEED+1);
  }
  
  void decSpeed(void) {
    m_speed = (m_speed + (MAX_SPEED+1) - 1) % (MAX_SPEED+1);
  }  
  
  inline int getSpeed(void) {
    return m_speed;
  }
  
  inline void setSpeed(byte new_speed) {
    m_speed = new_speed;
  }  
  
  inline int getMaxSubProgram(void) {
    return m_max_subprogram;
  }

  inline boolean isInAutoMode() {
    return m_includeInAuto;
  }
  
  void incColorMode(void) {
    m_color_mode = (m_color_mode + 1) % COLORS;
  }
  
  void decColorMode(void) {
    m_color_mode = (m_color_mode + COLORS - 1) % COLORS;
  }
  
  inline int getColorMode(void) {
    return m_color_mode;
  }
  
  inline void setColorMode(int color) {
    m_color_mode = color; 
  }  
  
  void run(void);
};

class ProgramControl {
  unsigned             m_check;
  
  Program*             m_program;
  unsigned long        m_stop_time;
  unsigned long        m_switch_time;
  
  // For RS-232 slave mode
  int                  m_register;          // 0 = program, 1=subprogram, 2=color, 3=speed, 4=brightness
  int                  m_reg_value[6];      // Value or -1 means not set
 
public:
  ProgramControl( Program * p, unsigned long run_time_seconds ):
    m_program(p)
  {
    unsigned long time = millis();
    m_stop_time = run_time_seconds > 0 ? time + (run_time_seconds*1000LU) : 0;
    m_switch_time = (switch_minutes == 0) ? 0 : time + (switch_minutes * 60 * 1000LU);
    m_check = 0; 
    
    resetSlave();
  }
  
  void resetSlave() {
    m_register = -1;
    
    for ( int i=0; i < sizeof(m_reg_value)/sizeof(int); i++ )
      m_reg_value[i] = -1;    
  }
  
  boolean isRunning(void);
  void run(void);
  void slaveReadSerial(void);
  void dmxChannelUpdate(void);      // Latch DMX channel values into running program values
  
  Program *getCurrentProgram() {
    return m_program;
  }
  
  byte getSubProgram() {
    return m_program->getSubProgram();
  }
  
  byte getSpeed() {
    return m_program->getSpeed();
  }  
};

class SoundSampler {
  
#define SPKR_LOUDEST        140        // Speaker output for "loudest" sound - can get higher numbers but this is the ceiling for effects
  
#define PEAK_HOLD_MS 2000
#define PEAK_DROP_MS 350  

  uint32_t m_peak_hold;
  uint32_t m_peak;
  uint32_t m_lastPeak;
  uint16_t m_peak_hold_ms;
  uint16_t m_peak_drop_ms;

public:  
  SoundSampler( uint16_t peak_hold_ms=PEAK_HOLD_MS, uint16_t peak_drop_ms=PEAK_DROP_MS ) : 
    m_lastPeak(1),
    m_peak(1),
    m_peak_hold( millis() ),
    m_peak_hold_ms( peak_hold_ms ),
    m_peak_drop_ms( peak_drop_ms )
  {
  }
  
  uint32_t getPeak( void ) {
   return m_peak;
  }
  
  int sample( void );
  
  void sdelay( int ms ) {
    long finish = millis() + ms;
    do {
      sample();
      delay(1);
    }
    while ( millis() < finish );
  }  
  
  int peak2duration( int low=0, int high=10000);    
};
 
extern void OffProgram( ProgramControl * p );
extern void TestModeProgram( ProgramControl * p );
extern void SoundActiveProgram( ProgramControl * p );
extern void ColorFadeProgram( ProgramControl * p );
extern void StrobeSyncProgram( ProgramControl * p );
extern void RandomStrobeProgram( ProgramControl * p );
extern void ShapesProgram( ProgramControl * p );
extern void Growing_spotsProgram( ProgramControl * p );
extern void Chase_program( ProgramControl * p );
extern void ColorFillProgram( ProgramControl * p );
extern void TwirlersProgram( ProgramControl * p );
extern void SquaresProgram( ProgramControl * p );
extern void LinesProgram( ProgramControl * p );
extern void MeterProgram( ProgramControl * p );
extern void PatternsProgram( ProgramControl * p );
extern void FallingLightProgram( ProgramControl * p );
extern void SparkleProgram( ProgramControl * p );
extern void FillProgram( ProgramControl * p );

Program programs[ NUM_PROGRAMS ] = {
  Program( Off,               0,   AUTO_COLOR,      OffProgram,                false ),
  Program( Test_mode,         0,   AUTO_COLOR,      TestModeProgram,           false ),
  Program( Sound_activated,   2,   AUTO_COLOR,      SoundActiveProgram,        true ),
  Program( Strobe,            25,  WHITE,           StrobeSyncProgram,         true ),
  Program( Strobe_random,     16,  WHITE,           RandomStrobeProgram,       true ), 
  Program( Sparkle,           0,   AUTO_COLOR,      SparkleProgram,            true ),  
  Program( Shapes,            2,   AUTO_COLOR,      ShapesProgram,             true ),
  Program( Twirlers,          5,   AUTO_COLOR,      TwirlersProgram,           true ),
  Program( Chase,             5,   AUTO_COLOR,      Chase_program,             true ),  
  Program( Growing_spots,     0,   AUTO_COLOR,      Growing_spotsProgram,      true ),  
  Program( Falling_light,     0,   AUTO_COLOR,      FallingLightProgram,       true ),
  Program( Squares,           0,   AUTO_COLOR,      SquaresProgram,            true ),
  Program( Lines,             9,   AUTO_COLOR,      LinesProgram,              true ),
  Program( Fill,              4,   AUTO_COLOR,      FillProgram,               true ),
  Program( Meters,            0,   RED,             MeterProgram,              false ),
  Program( Patterns,          16,  AUTO_COLOR,      PatternsProgram,           true ),
  Program( ColorFade,         0,   AUTO_COLOR,      ColorFadeProgram,          true ),  
  Program( ColorFill,         0,   AUTO_COLOR,      ColorFillProgram,          true ) 
};

// const unsigned NUMBER_BITMASKS[10] = { 0xF99F, 0x2222, 0xF1CF, 0xF17F, 0xAAF2, 0xF87F, 0xF8FF, 0xF111, 0xF9F9, 0xF9F1 };

prog_char prog_signon[] PROGMEM = "LED ARRAY v1.10   \n";
prog_char prog_master_label[] PROGMEM = "master = ";
prog_char prog_program_label[] PROGMEM = "program = ";
prog_char prog_brightness_label[] PROGMEM = "brightness = ";
prog_char prog_color_label[] PROGMEM = "color mode = ";
prog_char prog_speed_label[] PROGMEM = "speed = ";
prog_char prog_panel_label[] PROGMEM = "panel = ";
prog_char prog_switch_label[] PROGMEM = "switching = ";
prog_char prog_no_saved_state[] PROGMEM = "No saved state";
prog_char prog_new_peak[] PROGMEM = "New peak ";
prog_char prog_new_low[] PROGMEM = "New low ";
prog_char prog_ack[] PROGMEM = "ACK";
prog_char prog_nak[] PROGMEM = "NAK";

prog_char cmd_program[] PROGMEM = "p";
prog_char cmd_subprogram[] PROGMEM = "u";
prog_char cmd_color[] PROGMEM = "c";
prog_char cmd_speed[] PROGMEM = "s";
prog_char cmd_brightness[] PROGMEM = "b";
prog_char cmd_end[] PROGMEM = ".";

// DMX receiver definitions and ISR

enum {IDLE, BREAK, STARTB, STARTADR};                   // DMX states

volatile uint8_t    dmx_data[DMX_NUM_CHANNELS];         // Array of DMX values (size = # of channels to track)
volatile uint16_t   dmx_address = DMX_BASE_ADDRESS;     // Starting DMX address
volatile uint8_t    gDmxState = IDLE;

#if BUTTON_PANEL == PANEL_BINARY

#define USER_SIGNAL_LED_1       A1       // User output LED #1
#define USER_SIGNAL_LED_2       A2       // User output LED #2  
#define USER_SIGNAL_LED_3       A3       // User output LED #3
#define USER_SIGNAL_LED_4       A4       // User output LED #4
#define USER_SIGNAL_LED_5       A5       // User output LED #5

#endif

#if BUTTON_PANEL == PANEL_TEXT

// MAX pins

#define MAX7219_DIN      A2        // Green  
#define MAX7219_LOAD     A3        // Yellow
#define MAX7219_CLOCK    A4        // White

// MAX7219 registers

#define MAX7219_REG_DIGIT0        0x01
#define MAX7219_REG_DIGIT1        0x02
#define MAX7219_REG_DIGIT2        0x03
#define MAX7219_REG_DIGIT3        0x04
#define MAX7219_REG_DIGIT4        0x05
#define MAX7219_REG_DIGIT5        0x06
#define MAX7219_REG_DIGIT6        0x07
#define MAX7219_REG_DIGIT7        0x08
#define MAX7219_REG_DECODEMODE    0x09
#define MAX7219_REG_INTENSITY     0x0a
#define MAX7219_REG_SCAN_LIMIT    0x0b
#define MAX7219_REG_SHUTDOWN      0x0c
#define MAX7219_REG_DISPLAY_TEST  0x0f

//     6
//   1   5
//     0
//   2   4
//     3     7

// reg 4: 0x10 = top dot, 0x20 = bottom of colon, 0x40 = top of colon

prog_uchar font[] PROGMEM = {
  0x00, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00,     /* 0x20 - 0x2F */  
  0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x73, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00,     /* 0x30 - 0x3F */
  0x00, 0x77, 0x7F, 0x4E, 0x3D, 0x4F, 0x47, 0x5E, 0x37, 0x06, 0x3C, 0x00, 0x0E, 0x76, 0x76, 0x7E,     /* 0x40 - 0x4F UPPER */
  0x67, 0xFE, 0x05, 0x5B, 0x46, 0x3E, 0x00, 0x00, 0x00, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08,     /* 0x50 - 0x5F UPPER */  
  0x00, 0x77, 0x1F, 0x0D, 0x3D, 0x4F, 0x47, 0x7B, 0x17, 0x04, 0x3C, 0x00, 0x0E, 0x15, 0x15, 0x1D,     /* 0x60 - 0x6F LOWER */
  0x67, 0x9D, 0x05, 0x5B, 0x07, 0x1C, 0x00, 0x00, 0x00, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08      /* 0x70 - 0x7F LOWER */  
 }; 
 
prog_char prog_fault[] PROGMEM = "ERR"; 
prog_char prog_max_master[] PROGMEM = "P-00";    
prog_char prog_max_brightness[] PROGMEM = "br00";    
prog_char prog_max_program[] PROGMEM = "Pr00"; 
prog_char prog_max_color[] PROGMEM = "Co00"; 
prog_char prog_max_subprogram[] PROGMEM = "Su00"; 
prog_char prog_max_speed[] PROGMEM = "SP00"; 
prog_char prog_max_switching[] PROGMEM = "CP00"; 
prog_char prog_dmx_address[] PROGMEM = "d000"; 
prog_char prog_max_greeting[] PROGMEM = "HELLO   "; 

// ----------------------------------------------------------------------------------------- MaxSetRegister
//
void MaxSetRegister( byte reg, byte col ) {    
  digitalWrite( MAX7219_LOAD, LOW );       // begin load     
  
  shiftOut( MAX7219_DIN,  MAX7219_CLOCK, MSBFIRST, reg );
  shiftOut( MAX7219_DIN,  MAX7219_CLOCK, MSBFIRST, col );

  digitalWrite( MAX7219_LOAD,HIGH );       // end load
}

// ----------------------------------------------------------------------------------------- MaxWrite
//
void MaxWrite( const char * output ) {
   int index = 0;
   
  for ( ; output[index] != 0; index++ ) {
    if ( index < 4 )
       MaxCharOut(MAX7219_REG_DIGIT0 + index,  output[index] );
    else {
      if ( index == 4 )
        delay( 1000 );
      else
        delay( 300 );
        
      for ( int i=0; i < 4; i++ )
         MaxCharOut(MAX7219_REG_DIGIT0 + i,  output[index-3+i] );        
    }
  }
  
  while ( index < 4 ) {
    MaxCharOut(MAX7219_REG_DIGIT0 + index, ' ' );    
    index++;
  }  
}

// ----------------------------------------------------------------------------------------- MaxCharOut
//
void MaxCharOut( byte reg, char ch ) {
  byte bitmap = 0;
  
  if ( ch >= ' ' && ch < 0x80)
    bitmap = pgm_read_byte_near( font + (ch-' ') );
   
  MaxSetRegister( reg, bitmap );
}

#endif

// ----------------------------------------------------------------------------------------- getProgram
//
Program *getProgram( LightShow ls = program  ) {
  for ( int i=0; i < NUM_PROGRAMS; i++ )
    if ( programs[i].m_id == (LightShow)ls )
      return &programs[i];

  fault |= 0x20;
  
  signalStateToUser();
  
  return NULL;
}

// ----------------------------------------------------------------------------------------- getColorChannel
//
byte getColorChannel( int color, int channel ) {
 
  if ( color != CUSTOM_COLOR )
    return pgm_read_byte_near( &color_table[color][channel] ); 
  
  DMX_CHANNELS* channels = (DMX_CHANNELS*)dmx_data;

  if ( channel == 0 )
    return channels->dmx_red;
  else if ( channel == 1 )
    return channels->dmx_green;
  else
    return channels->dmx_blue;   
}

// ----------------------------------------------------------------------------------------- setup
//
void setup()
{
  Tlc.init( 0);

  Serial.begin(9600); // open serial port, set the baud rate to 9600 bps   
  
  EEPROM_MEMORY memory;

  char *p = (char *)&memory;
  for ( int address=0; address < sizeof( EEPROM_MEMORY ); address++ )
    *p++ = EEPROM.read( address );
    
  if ( MAGIC_NUMBER == memory.magic_number ) {
    program = (LightShow)memory.program;
    if ( program < 0 || program > NUM_PROGRAMS )
      program = Test_mode;
    
    brightness = memory.brightness;
    if ( brightness < 0 || brightness >= BRIGHTNESS_LEVELS )
      brightness = BRIGHTNESS_LEVELS/2;
    master_mode = (MasterMode)memory.master_mode; 
    switch_minutes = memory.switch_minutes;
    dmx_address = memory.dmx_address;
    
    for ( int i=0; i < NUM_PROGRAMS; i++ ) {
      getProgram((LightShow)i)->setSubProgram( memory.sub_program[i] );
      getProgram((LightShow)i)->setColorMode( memory.color_mode[i] );
      getProgram((LightShow)i)->setSpeed( memory.speed[i] );     
    }
  }
  else {  
    if ( DEBUG ) {
      Serial.println( getProgString( prog_no_saved_state ) );
    }
  }
  
#if BUTTON_PANEL != PANEL_NONE  
  attachInterrupt(0, buttonPressed, RISING); // Interrupt on pin 2
#endif

  pinMode( SPKR_INPUT, INPUT );
  
#if BUTTON_PANEL == PANEL_TEXT
  // Initialize the max 7219
    
  pinMode( MAX7219_DIN, OUTPUT );
  pinMode( MAX7219_CLOCK, OUTPUT );
  pinMode( MAX7219_LOAD, OUTPUT );

  MaxSetRegister(MAX7219_REG_SHUTDOWN, 0x00);        // In shutdown mode
  
  MaxSetRegister(MAX7219_REG_SCAN_LIMIT, 0x05);      
  MaxSetRegister(MAX7219_REG_DECODEMODE, 0x00);      // using an led matrix (not digits)
  MaxSetRegister(MAX7219_REG_DISPLAY_TEST, 0x00);    // no display test
  
  for ( int digit=1; digit<=8; digit++)              // empty registers, turn all LEDs off 
    MaxSetRegister( digit, 0 );

  MaxSetRegister(MAX7219_REG_INTENSITY, 0x08 );      // range: 0x00 to 0x0f
  
  MaxSetRegister(MAX7219_REG_SHUTDOWN, 0x01);        // not in shutdown mode

  MaxWrite( getProgString( prog_signon ) );
  MaxWrite( getProgString(prog_max_greeting ) ); 
#endif

#if BUTTON_PANEL == PANEL_BINARY
  pinMode( USER_SIGNAL_LED_2, OUTPUT );  
  pinMode( USER_SIGNAL_LED_3, OUTPUT );  
  pinMode( USER_SIGNAL_LED_4, OUTPUT ); 
#endif

  pinMode( BUTTON_1_INPUT, INPUT );
  pinMode( BUTTON_2_INPUT, INPUT );
  pinMode( BUTTON_3_INPUT, INPUT );
  
  randomSeed( analogRead(SPKR_INPUT) );
  
  reset_shutdown_timer();
  
  if ( DEBUG ) {
    Serial.println( getProgString( prog_signon ) );
    reportState();
  }
  
  setupForMasterMode();
}

// ----------------------------------------------------------------------------------------- setupForMasterMode
//
void setupForMasterMode() {
  if ( master_mode == mode_dmx ) {                // Setup the serial USART for DMX receive
    memset( (void*)dmx_data, 0, sizeof(dmx_data) );
    
    // Note that with this setup, you need to remove the RX line to reflash the arduino 
    Serial.begin( 250000 );                       // Enable serial reception with a 250k rate
  }
  else {
    Serial.begin(9600); // open serial port, set the baud rate to 9600 bps
  }
  
  pinMode( DMX_RX_STATUS_PIN, OUTPUT );         // Blinks on incoming DMX data   
  digitalWrite( DMX_RX_STATUS_PIN, LOW );  
}


// ----------------------------------------------------------------------------------------- getProgString
//
char * getProgString( void * pstr ) {
  static char strbuffer[35];
  strbuffer[sizeof(strbuffer)-1] = '\0';
  strcpy_P(strbuffer, (char*)pstr );
  if ( strbuffer[sizeof(strbuffer)-1] != '\0' ) {
      fault |= 0x30;  
      signalStateToUser();      
  }
  return strbuffer;
}

// ----------------------------------------------------------------------------------------- reportState
//
void reportState( void ) {
  if ( master_mode != mode_master ) {
    Serial.print( getProgString( prog_master_label ) );
    Serial.println( master_mode );
    Serial.print( getProgString( prog_program_label ) );
    Serial.print( program );
    Serial.print( "." );
    Serial.println( getProgram()->getSubProgram() );
    Serial.print( getProgString( prog_brightness_label ) );
    Serial.println( brightness );    
    Serial.print( getProgString( prog_color_label ) );
    Serial.println( getProgram()->getColorMode() );
    Serial.print( getProgString( prog_speed_label ) );
    Serial.println( getProgram()->getSpeed() );   
#if BUTTON_PANEL != PANEL_NONE  
    Serial.print( getProgString( prog_panel_label ) );
    Serial.println( button_mode );  
#endif  
    Serial.print( getProgString( prog_switch_label ) );
    Serial.println( (int)switch_minutes );     
    Serial.println( );
  }
}

// ----------------------------------------------------------------------------------------- loop
//
void loop()
{ 
  for ( ;; ) {
    signalStateToUser();

    Program * p = getProgram();
    
    if ( p )
       p->run();
    
    clear_all();
  }
}

// ----------------------------------------------------------------------------------------- OffProgram
//
void OffProgram( class ProgramControl * ctl ) {
  char value = 0;
  long next = 0;
  
#if BUTTON_PANEL == PANEL_TEXT
      MaxWrite( "" );
#endif
  
  while ( ctl->isRunning() ) { 
    if ( button_mode == panel_program ) {
      if ( millis() > next ) {
         
#if BUTTON_PANEL == PANEL_TEXT
        MaxSetRegister( MAX7219_REG_DIGIT4, value & 1 ? 0x20 : 0x40 );
        value ^= 1;
#endif

#if BUTTON_PANEL == PANEL_BINARY
         if ( (value & 0x1F) == 0 )
           value = 1;
         else
           value <<= 1;
         
         signalUser( value );
#endif
         next = millis () + 300;  
      }   
    }

    delay( 10 );  
  }
}

// ----------------------------------------------------------------------------------------- TestModeProgram
//
void TestModeProgram( ProgramControl * ctl ) {
  
  int led = 0;
  int color = RED;
  int mode = 0;

  while (  ctl->isRunning() ) {
    for ( TLC_CHANNEL_TYPE channel=0; channel <  LED_CHANNELS; channel++ ) {
      int target_color;
      
      if ( mode == 0 ) {
        target_color = channel == led ?  color : BLACK;
      }
      else {
        target_color = channel <= led ?  color : BLACK;        
      }
      
      set_color( channel, target_color  );
    }

    Tlc.update();  
    delay( 70 );
    
    if ( ++led == LED_CHANNELS ) {
      led = 0;
      if ( ++color == WHITE+1 ) {
        color = RED;
        mode = (mode +1) % 2;    
      }   
    }
  }
}

// ----------------------------------------------------------------------------------------- MeterProgram
//
void MeterProgram( ProgramControl * ctl ) { 
  SoundSampler sound(100, 100);

  while ( ctl->isRunning() ) {
    int level = map( sound.getPeak(), 1, SPKR_LOUDEST, 0, 16 );
    
    int color = LIME;
    
    for ( TLC_CHANNEL_TYPE channel=0; channel < LED_CHANNELS; channel++ ) {
        if ( channel > 12 )
          color = RED;
          
        set_color( channel,  (channel+1) <= level ? color : BLACK );
    }    
    
    Tlc.update();      
    sound.sdelay( 10 );     
  }
}

// ----------------------------------------------------------------------------------------- StrobeSyncProgram
//
void StrobeSyncProgram( ProgramControl * ctl ) {
  while (  ctl->isRunning() ) {
    int color = choose_random_color( GRAY_SCALE );
    
    overlay_shape( 0xFFFF, color );
    
    Tlc.update();  
    delay( ctl->getSpeed() == 0 ? 50 : ctl->getSpeed() * 100 );
    
    Tlc.clear();
    Tlc.update();
     
    delay( ctl->getSubProgram() == 0 ? 50 : ctl->getSubProgram() * 100 );
  }
}

// ----------------------------------------------------------------------------------------- RandomStrobeProgram
//
void RandomStrobeProgram( ProgramControl * ctl ) { 
  SoundSampler sound;
  
  int color = choose_random_color( GRAY_SCALE );
  
  Tlc.clear();  
  
  while ( ctl->isRunning() ) {
    if ( random(100) < 5 ) {                              // 5% chance to change color
      color = choose_random_color( GRAY_SCALE );
    }
  
    int spots = ctl->getSubProgram();
    if ( spots == 0 )
      spots = random(4);

    for ( int j=0; j < spots; j++ ) {   
      int target = random(LED_CHANNELS);
      set_color( target, color );
    }
    
    Tlc.update(); 
      
    sound.sdelay( 50 );   
       
    Tlc.clear();       
    Tlc.update();
           
    // Delay with lights off
    int run_speed = map( sound.getPeak(), 0, SPKR_LOUDEST, MAX_SPEED-1, 1 );
    
    sound.sdelay( run_speed * 20 ); 
   }
}

// ----------------------------------------------------------------------------------------- SparkleProgram
//
void SparkleProgram( ProgramControl * ctl ) {
  
  SoundSampler sound; 
  
  while ( ctl->isRunning() ) {   
    unsigned int shape = random(0xFFFF);
    
    int color = choose_random_color( GRAY_SCALE );
  
    show_shape( shape, color, BLACK );
    Tlc.update();    

    int run_speed = ctl->getSpeed();

    if ( run_speed == 0 ) {    
      sound.sample();          
      run_speed = map( sound.getPeak(), 0, SPKR_LOUDEST, MAX_SPEED-1, 1 ); 
    }
    
    sound.sdelay( (run_speed*10) + 50);
  }
}

// ----------------------------------------------------------------------------------------- ColorFillProgram
//
void ColorFillProgram( ProgramControl * ctl ) {
  int color = -1;

  while (  ctl->isRunning() ) {
    int target_color = choose_random_color( GRAY_SCALE );

    if ( color != target_color || color == CUSTOM_COLOR ) {    // Last check is for DMX RGB updates
      for ( TLC_CHANNEL_TYPE channel=0; channel <  LED_CHANNELS; channel++ ) {
        set_color( channel, target_color );
      }
  
      Tlc.update();  
      color = target_color;
    }
    
    delay( 10 );
  }
}

// ----------------------------------------------------------------------------------------- ColorFadeProgram
//
void ColorFadeProgram( ProgramControl * ctl ) {

  int color = 0;
  int duration;
  boolean down = true;
  
  SoundSampler sound;  
  
  while ( ctl->isRunning() ) {
    sound.sdelay( 1 );
    
    if ( !tlc_updateFades() ) {
      if ( down ) {
        color = choose_random_color( GRAY_SCALE );
       
        duration = sound.peak2duration();
      }
      else
        duration = -duration; 
     
      overlay_shape_fade( 0xFFFF, color, duration );  
      
      down = !down;
    }
  }
}

// ----------------------------------------------------------------------------------------- Growing_spotsProgram
//
void Growing_spotsProgram( ProgramControl * ctl ) {
  
  const int FADE_TIME = 1000;
  
  TLC_CHANNEL_TYPE grow_channel = 0;
  int color;
  long next_fade = 0;
         
  while ( ctl->isRunning() ) {

    if ( millis() > next_fade ) {
      remove_all_fades();  
      
      set_color( grow_channel+1, LT_GRAY );       
      set_color( grow_channel, LT_GRAY );
      
      grow_channel = (TLC_CHANNEL_TYPE)(random(LED_CHANNELS) & 0xFE );
      
      color = choose_random_color( GRAY_SCALE );

      rgb_fade( grow_channel, color, -FADE_TIME, 10000 );
      rgb_fade( grow_channel+1, color, -FADE_TIME, 10000 );
      next_fade = millis() + FADE_TIME;    
    }
    
    tlc_updateFades();
    
    TLC_CHANNEL_TYPE flash;
        
    do {
      flash = random(LED_CHANNELS);
    } 
    while ( flash == grow_channel || flash == grow_channel+1 );
    
    set_color( flash, random(COLORS-2)+1 );
    delay( 50 );
    tlc_updateFades();
        
    set_color( flash, LT_GRAY );  
    delay(40);
  }
}

// ----------------------------------------------------------------------------------------- FallingLightProgram
//
void FallingLightProgram( ProgramControl * ctl ) {

  int color = 0;
  
  SoundSampler sound; 
  
  byte cells[4] = { 3, 3, 3, 3 };
  long next[4] = { 0, 0, 0, 0 }; 
  int dur[4] = { 0, 0, 0, 0 };
  int colors[4] = { WHITE, WHITE, WHITE, WHITE };
  int peak[4] = { 0,0,0,0 };
  
  while ( ctl->isRunning() ) {  
    long ms = millis();
        
    for ( int c=0; c < 4; c++ ) {  
      int duration = sound.peak2duration( 150, 2000 );
                
      if ( duration > peak[c] || next[c] < ms ) { 
        set_color( cells[c]*4+c, BLACK );           
        cells[c] = ( cells[c] + 1 ) % 4;
        
        if ( cells[c] == 0 ) {
          dur[c] = (duration+random(duration))/4;    
          colors[c] = choose_random_color( GRAY_SCALE );  
        }
        
        next[c] = millis() + dur[c];   
        peak[c] = duration;            
        
        set_color( cells[c]*4+c, colors[c] );         
      }
      
      Tlc.update();        
      sound.sdelay(1);
    }
  }
}

// ----------------------------------------------------------------------------------------- SquaresProgram
//
void SquaresProgram( ProgramControl * ctl ) {

  static prog_uint16_t shapes[4] PROGMEM = { 0xEAE0, 0x7570, 0x0EAE, 0x0757 }; 
  
  SoundSampler sound;  
  
  int pos, lastPos;
  
  while ( ctl->isRunning() ) {
    sound.sample();
    
    int color = choose_random_color( GRAY_SCALE );
    
    while ( (pos = random(4)) == lastPos )
      ;
      
    lastPos = pos;
    
    prog_uint16_t shape = pgm_read_word_near( shapes + pos );    
  
    int duration = sound.peak2duration( 200, 2000 );
    
    overlay_shape_fade( shape, color, duration*2 );

    long stop_time = millis() + duration;
       
    do {
      tlc_updateFades();       
      sound.sdelay(1);
    }
    while ( ctl->isRunning() && stop_time > millis() );
  }
}

// ----------------------------------------------------------------------------------------- SoundActiveProgram
//
void SoundActiveProgram( ProgramControl * ctl ) { 
  int strobe_state = 0;
  int strobe_color = WHITE;
  
  SoundSampler sound;
  
  int last_duration = 0;
  
  while (  ctl->isRunning() ) {
      int delta = sound.sample();
      
      int peak = sound.getPeak();

      if ( peak >= SPKR_LOUDEST ) {       
        remove_all_fades( );

        long finish = millis() + 1500;
        int target = -1;
        int strobe_program = 1;    // random(2);
          
        do {
          if (strobe_program == 0 )
            target = rand() % LED_CHANNELS;
     
          strobe_color = choose_random_color( GRAY_SCALE );    
          
          for ( TLC_CHANNEL_TYPE channel=0; channel < LED_CHANNELS; channel++ )
              set_color( channel,  (target < 0 || channel == target) ? strobe_color : BLACK );
                    
          Tlc.update();  
          sound.sdelay( 100 );          
          Tlc.clear();
          Tlc.update();
          sound.sdelay( 50 );        
        }
        while ( ctl->isRunning() && millis() < finish );
      }
      else {             
        if ( tlc_fadeBufferSize < TLC_FADE_BUFFER_LENGTH - 2) {   
          int duration = sound.peak2duration(); 
          
          if ( duration < last_duration ) {
            remove_all_fades();     
          }  
          
          for ( TLC_CHANNEL_TYPE led=0; led < LED_CHANNELS; led++ ) {
             if (!tlc_isFading(led) && !tlc_isFading(led+LED_CHANNELS) && !tlc_isFading(led+(LED_CHANNELS*2)) ) {    
                int channel_duration = duration;
                if ( delta < 0 )
                  channel_duration += random(duration/2);
                else
                  channel_duration += random(duration/4);    
                
                int led_color = choose_random_color( NULL );     
                
                rgb_fade( led, led_color, channel_duration, 0 );
                
                last_duration = duration;      // Last_duration is last _set_ duration
             }     
          }
         
      }
        
      tlc_updateFades();
     }
  }
}

// ----------------------------------------------------------------------------------------- PatternsProgram
//

#define SPRITE_SHAPE          0
#define SPRITE_OVERLAY        1
#define SPRITE_OVERLAY_FADE   2
#define SPRITE_PLAY           0x8000

typedef struct {
  unsigned int method;             // How to apply the shape bits
  unsigned int shape;              // Bit mask of LEDs to light 4 row x 4 columns
  unsigned int color;              // Color for the bits
  int time;                        // Time in MS (negative simply means fade in rather than fade out)
} SPRITE;

void PatternsProgram( ProgramControl * ctl )
{
  static prog_uint16_t sprites[] PROGMEM = { 
    SPRITE_OVERLAY,                     0xF99F,      RED,        0,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x0660,      GREEN,      -1500,
  
    SPRITE_OVERLAY_FADE,                0xF99F,      RED,        -1000,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x0660,      GREEN,       1500,
  
    SPRITE_OVERLAY,                     0xF99F,      BLUE,        0,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x0660,      GREEN,      -1500,  
  
    SPRITE_OVERLAY_FADE,                0xF99F,      GREEN,      -1000,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x0660,      BLUE,        1500,  
  
    SPRITE_OVERLAY_FADE,                0x9669,      BLUE,       -1000,
    SPRITE_OVERLAY | SPRITE_PLAY,       0x6996,      BLACK,       1000,  
  
    SPRITE_OVERLAY_FADE,                0x6006,      GREEN,      -2000,  
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x0990,      WHITE,      -2000,    

    SPRITE_OVERLAY_FADE,                0xA5A5,      RED,         2000,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x5A5A,      GREEN,      -2000,
     
    SPRITE_OVERLAY_FADE,                0xA5A5,      GREEN,       2000,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x5A5A,      RED,         -2000,  

    SPRITE_OVERLAY_FADE,                0xA5A5,      RED,         2000,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x5A5A,      GREEN,       -2000, 

    SPRITE_OVERLAY_FADE,                0xA5A5,      BLUE,        2000,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x5A5A,      GREEN,       -2000,  
      
    SPRITE_OVERLAY_FADE,                0xA5A5,      GREEN,       2000,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x5A5A,      BLUE,        -2000,    

    SPRITE_OVERLAY_FADE,                0xA5A5,      BLUE,        2000,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x5A5A,      GREEN,       -2000,    
  
    SPRITE_SHAPE,                       0xFFFF,      BLACK,        0,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x000F,      BLUE,        -500,  
     
    SPRITE_OVERLAY_FADE,                0x00F0,      BLUE,       -500,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x000F,      BLUE,        500,     
    
    SPRITE_OVERLAY_FADE,                0x0F00,      BLUE,       -500,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x00F0,      BLUE,        500,       
    
    SPRITE_OVERLAY_FADE,                0xF000,      BLUE,       -500,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x0F00,      BLUE,        500,     
     
    SPRITE_OVERLAY_FADE,                0xF000,      BLUE,        500,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x000F,      LIME,        -500, 

    SPRITE_OVERLAY_FADE,                0x00F0,      LIME,       -500,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x000F,      LIME,        500,     
    
    SPRITE_OVERLAY_FADE,                0x0F00,      LIME,       -500,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x00F0,      LIME,        500,       
    
    SPRITE_OVERLAY_FADE,                0xF000,      LIME,       -500,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x0F00,      LIME,        500,

    SPRITE_OVERLAY_FADE,                0xF000,      LIME,        500,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x000F,      BLUE,        -500, 

    SPRITE_OVERLAY_FADE,                0x00F0,      BLUE,       -500,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x000F,      BLUE,        500,     
    
    SPRITE_OVERLAY_FADE,                0x0F00,      BLUE,       -500,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x00F0,      BLUE,        500,       
    
    SPRITE_OVERLAY_FADE,                0xF000,      BLUE,       -500,
    SPRITE_OVERLAY_FADE | SPRITE_PLAY,  0x0F00,      BLUE,        500,        
          
  };

  play_sprites( ctl, (sizeof(sprites)/sizeof(SPRITE)), sprites );
}

// ----------------------------------------------------------------------------------------- ShapesProgram
//
void ShapesProgram( ProgramControl * ctl ) {

  static prog_uint16_t shapes[] PROGMEM = { 0xF000, 0x0F00, 0x00F0, 0x000F, 0x8888, 0x4444, 0x2222, 0x1111, 0x8000, 0x4800, 0x2480, 0x1248, 0x0124, 0x0012, 0x0001 }; // , 0xF99F, 0x0660
  
  static const int shape_count = (sizeof(shapes)/sizeof(unsigned));

  byte COLOR_COMBO[][2] = {
    { BLUE, BLACK },
    { RED, GRAY },
    { WHITE, BLACK },
    { BLUE, LT_GRAY },
    { GREEN, BLACK },
    { WHITE, BLACK },
    { MAGENTA, BLACK },
    { BLACK, BLUE }
  };

  animate( ctl, 1, shape_count, (unsigned *)shapes, 8, (byte *)COLOR_COMBO, ANIMATE_NORMAL );   
}

// ----------------------------------------------------------------------------------------- TwirlersProgram
//
void TwirlersProgram( ProgramControl * ctl ) {

  static const int SHAPES = 4;
  
  static prog_uint16_t shapes[SHAPES][4] PROGMEM = { 
      { 0x8420, 0x4440, 0x2480, 0x0E00 },
      { 0x0421, 0x0222, 0x0124, 0x0070 },
      { 0x0842, 0x0444, 0x0248, 0x00E0 },
      { 0x4210, 0x2220, 0x1240, 0x0700 }
  }; 
  
  animate( ctl, SHAPES, 4, (unsigned *)shapes, 0, NULL, ANIMATE_NORMAL );   
}

// ----------------------------------------------------------------------------------------- LinesProgram
//
void LinesProgram( ProgramControl * ctl ) {

  static const int SHAPES = 8;
  
  static prog_uint16_t shapes[SHAPES][9] PROGMEM = { 
   
    { 0xF0F0, 0x0F0F, 0, 0, 0, 0 ,0, 0, 0 },  
    { 0xAAAA, 0x5555, 0, 0, 0, 0 ,0, 0, 0 },      
    { 0x8000, 0x4800, 0xA480, 0x5A48, 0x25A4, 0x125A, 0x0125, 0x00012, 0x0001 },    
    { 0x9999, 0x6666, 0, 0, 0, 0 ,0, 0, 0 }, 
    { 0xF00F, 0x0FF0, 0, 0, 0, 0 ,0 ,0, 0 },
    { 0x8000, 0x4800, 0x2480, 0x1248, 0x0124, 0x0012, 0x0001, 0, 0 }, 
    { 0x8888, 0x4444, 0x2222, 0x1111, 0, 0, 0, 0, 0 },
    { 0xF000, 0x0F00, 0x00F0, 0x000F, 0, 0, 0, 0, 0 }
 
  }; 
  
  animate( ctl, SHAPES, 9, (unsigned *)shapes, 0, NULL, ANIMATE_NORMAL );   
}

// ----------------------------------------------------------------------------------------- FillProgram
//
void FillProgram( ProgramControl * ctl ) {

  static const int SHAPES = 3;
  
  static prog_uint16_t shapes[SHAPES][16] PROGMEM = { 
      { 0x8000, 0x8800, 0x8880, 0x8888, 0x888C, 0x88CC, 0x8CCC, 0xCCCC, 0xECCC, 0xEECC, 0xEEEC, 0xEEEE, 0xEEEF, 0xEEFF, 0xEFFF, 0xFFFF },    
      { 0x8000, 0xC000, 0xE000, 0xF000, 0xF100, 0xF110, 0xF111, 0xF113, 0xF117, 0xF11F, 0xF19F, 0xF99F, 0xFD9F, 0xFF9F, 0xFFBF, 0xFFFF },
      { 0x1000, 0x3000, 0x7000, 0xF000, 0xF800, 0xFC00, 0xFE00, 0xFF00, 0xFF10, 0xFF30, 0xFF70, 0xFFF0, 0xFFF8, 0xFFFC, 0xFFFE, 0xFFFF }

      // { 0x8001, 0xC003, 0xE007, 0xF00F, 0xF18F, 0xF3CF, 0xF7EF, 0xFFFF, 0,      0,      0,      0,      0,      0,      0,      0      }
  }; 
  
  animate( ctl, SHAPES, 16, (unsigned *)shapes, 0, NULL, ANIMATE_REVERSE );   
}

// ----------------------------------------------------------------------------------------- Chase_program
//
void Chase_program( ProgramControl * ctl ) {
  static const int SHAPES = 5;
  static const int CELLS = 12;
  
  static prog_uint16_t shapes[SHAPES][CELLS] PROGMEM = { 
      { 0x8000, 0x4000, 0x2000, 0x1000, 0x0100, 0x0010, 0x0001, 0x0002, 0x0004, 0x0008, 0x0080, 0x0800 },
      { 0x8000, 0x4000, 0x2000, 0x1000, 0x0100, 0x0010, 0x0020, 0x0040, 0x0080, 0x0800, 0, 0 },
      { 0x0800, 0x0400, 0x0200, 0x0100, 0x0010, 0x0001, 0x0002, 0x0004, 0x0008, 0x0080, 0, 0 },
      { 0x8001, 0x4002, 0x2004, 0x1008, 0x0180, 0x0810, 0x8001, 0x4002, 0x2004, 0x1008, 0x0180, 0x0810 },
      { 0x0801, 0x0402, 0x0204, 0x0108, 0x0090, 0x0801, 0x0402, 0x0204, 0x0108, 0x0090, 0, 0 }
  }; 
  
  animate( ctl, SHAPES, CELLS, (unsigned *)shapes, 0, NULL, ANIMATE_NORMAL );
}

// ----------------------------------------------------------------------------------------- choose_random_color
//
int choose_random_color( const int *prohibit )
{
   int choice = getProgram()->getColorMode();
   
   if ( choice != AUTO_COLOR )
     return choice;
   
   for ( ;; ) {
     choice = random(COLORS-2)+1;
       
     if ( prohibit ) {
       bool prohibited = false;
       
       for ( const int * p = prohibit; *p != -1; p++ ) {
          if ( *p == choice ) {
            prohibited = true;
            break;
          }
       }
       
       if ( prohibited )
         continue;
     }
     
     break;
   }
   
   return choice;  
}

// ----------------------------------------------------------------------------------------- remove_all_fades
//
void remove_all_fades( ) {
  for ( TLC_CHANNEL_TYPE ch=0; ch < LED_CHANNELS; ch++ )
    for ( int i=0; i < 3; i++ )
      tlc_removeFades(ch + (LED_CHANNELS*i));       
}

// ----------------------------------------------------------------------------------------- clear_all
// Remove fades and all colors
//
void clear_all()
{       
  remove_all_fades();
  Tlc.clear();
  Tlc.update();
}

// ----------------------------------------------------------------------------------------- set_color
//
void set_color( int address, int color ) {
  Tlc.set(address+0, getColorChannel( color, 0 ) * brightness );
  Tlc.set(address+LED_CHANNELS, getColorChannel( color, 1 ) * brightness);
  Tlc.set(address+(LED_CHANNELS*2), getColorChannel( color, 2 ) * brightness);
}

// ----------------------------------------------------------------------------------------- rgb_fade
//
void rgb_fade( int address, int color, int duration, int value_fade ) {
   
#if 0   
  if ( false ) {
    Serial.print( address );
    Serial.print( " " );
    Serial.print( color );
    Serial.print( " " );  
    Serial.print( duration );
  }
#endif

  bool reverse_fade = false;
  uint32_t startMillis = millis()+30;    
    
  if ( duration < 0 ) {
    duration *= -1;
    reverse_fade = true;
  }
    
  for ( int c=0; c < 3; c++ ) {
     int channel = address + (LED_CHANNELS*c);
     
     uint32_t endMillis = startMillis + duration;
      
     int value = ((int)getColorChannel( color, c )) * brightness;

#if 0      
     if ( false ) {
       Serial.print( " " );
       Serial.print( (int)getColorChannel( color, c ) );
       Serial.print( "(" );
       Serial.print( channel );    
       Serial.print( ")" );      
     }
#endif

    tlc_removeFades( channel );   
    Tlc.set(channel,0);

    int end_value = value_fade > 0 ? value / value_fade : 0;
      
    if ( !reverse_fade )
      tlc_addFade(channel, value, end_value, startMillis, endMillis);  
    else
      tlc_addFade(channel, end_value, value, startMillis, endMillis);        
  }
  
#if 0    
  if ( false ) {
     Serial.println(  );
  }  
#endif  
}

// ----------------------------------------------------------------------------------------- show_shape
//
void show_shape( unsigned int shape, int fg_color, int bg_color ) { 
  for ( TLC_CHANNEL_TYPE channel=LED_CHANNELS; channel--; shape >>= 1 ) {
    bool on = shape & 1;
    set_color( channel, ( on ) ? fg_color : bg_color );
  }
}

// ----------------------------------------------------------------------------------------- overlay_fade
//
void overlay_shape( unsigned int shape, int fg_color ) {
  for ( TLC_CHANNEL_TYPE channel=LED_CHANNELS; channel--; shape >>= 1 ) {
    if ( shape & 1 ) {
      set_color( channel, fg_color );
    }
  }
}

// ----------------------------------------------------------------------------------------- overlay_shape_fade
//
void overlay_shape_fade( unsigned int shape, int fg_color, int duration ) {
  for ( TLC_CHANNEL_TYPE channel=LED_CHANNELS; channel--; shape >>= 1 ) {
    if ( shape & 1 ) {
      rgb_fade( channel, fg_color, duration, 0 ); 
    }
  }
}

// ----------------------------------------------------------------------------------------- animate
//
void animate( ProgramControl * ctl, 
              const int num_shapes, const int parts, prog_uint16_t *glype_data,
              const int colors, byte *color_pairs_table, int flags )
{
  int shape = 0;
  int cell = 0;
  int dir = 0;
  
  byte fg_color = -1;
  byte bg_color = -1;
          
  int base_time = map( parts, 4, 16, 30, 10 );

  SoundSampler sound;

  int shape_parts = 0;      
  int glyphs_left = 0;

  animate_choose_color( &fg_color, &bg_color, colors, color_pairs_table );  
      
  while ( ctl->isRunning() ) {
    if ( glyphs_left  > 0 ) { 
      prog_uint16_t s = pgm_read_word_near( glype_data + (shape*parts)+cell );
      show_shape( s, fg_color, bg_color );    
      Tlc.update();
    
      int run_speed = ctl->getSpeed();

      if ( run_speed == 0 ) {    
        sound.sample();          
        run_speed = map( sound.getPeak(), 0, SPKR_LOUDEST, MAX_SPEED-1, 1 ); 
      }
      
      sound.sdelay( (run_speed*10) + base_time );
        
      if ( dir == 0 )
        cell = ( cell + 1 ) % shape_parts;
      else 
        cell = ( cell + shape_parts - 1 ) % shape_parts;     
       
      glyphs_left--;
    }
    else {                         // Next animation
      if ( ctl->getSubProgram() == 0 ) {  
        if ( random(100) < 5 )    // Switch animations?
          shape = random(num_shapes);
      }
      else {
        shape = ctl->getSubProgram()-1;
      }
      
      // Figure out how many cells in this glyph
      for ( shape_parts = parts; shape_parts > 1 && pgm_read_word_near( glype_data + (shape*parts)+shape_parts-1 ) == 0; )
        shape_parts--;      
        
      glyphs_left = shape_parts;
    
      if ( (flags & ANIMATE_REVERSE) || random(100) < 40 )       // Switch directions
        dir = ( dir + 1 ) % 2;
        
      cell = ( dir > 0 ) ? shape_parts - 1 : 0;                  // Initialize cell
      
      if ( dir == 0 || !(flags & ANIMATE_REVERSE)) {    
        if ( random(100) < 10 )                                   // Switch colors?
          restartCurrentProgram = true;
      }       
    }       
  }  
}

void animate_choose_color( byte *fg_color, byte *bg_color, const int colors, byte * color_pairs_table ) {
  if ( colors == 0 ) {
    *fg_color = choose_random_color( GRAY_SCALE );
    *bg_color = BLACK;
  }
  else {
    int color = random(colors);
    *fg_color = color_pairs_table[color * 2];
    *bg_color = color_pairs_table[color * 2 + 1];    
  }
}

// ----------------------------------------------------------------------------------------- play_sprites
//
void play_sprites( ProgramControl * ctl, uint16_t num_sprites, prog_uint16_t* sprites )
{  
  SoundSampler sound; 
  
  int sprite_index = 0;
  long runtime = 0;
  prog_uint16_t* data_ptr = sprites;
          
  while ( ctl->isRunning() ) {   
   
    if ( millis() > runtime ) {
      runtime = 0;
        
      remove_all_fades();      
      
      SPRITE sprite;
      
      do {        
        prog_uint16_t * p = (prog_uint16_t *)&sprite;
        for ( int i=0; i < sizeof(SPRITE)/sizeof(prog_uint16_t); i++ ) {
          *p++ = pgm_read_word_near( data_ptr++ );
        }
        sprite_index++;
     
#if 0
        Serial.print( sprite_index );
        Serial.print( ": method=" );      
        Serial.print( sprite.method, HEX );
        Serial.print( " shape=" );
        Serial.print( sprite.shape, HEX );
        Serial.print( " color=" );
        Serial.print( sprite.color, DEC );
        Serial.print( " time=" );
        Serial.println( sprite.time, DEC );
#endif

        switch ( sprite.method & 0xFF ) {
          case SPRITE_SHAPE:
            show_shape( sprite.shape, sprite.color & 0xFF, sprite.color >> 8 );
            break;

          case SPRITE_OVERLAY:
            overlay_shape( sprite.shape, sprite.color );  
            break;   

         case SPRITE_OVERLAY_FADE:
            overlay_shape_fade( sprite.shape, sprite.color, sprite.time );
            break;
        }
        
        runtime = max( runtime, abs( sprite.time ) );
       }
      while ( !(sprite.method & SPRITE_PLAY) && (sprite_index < num_sprites) );     
     
      if ( tlc_fadeBufferSize == 0 )
        Tlc.update();
    
      if ( sprite_index >= num_sprites ){
        data_ptr = sprites;
        sprite_index = 0;
      }
      
      runtime += millis();
    }
    
    tlc_updateFades();    
    
    sound.sdelay( 10 );
  }
}

// ----------------------------------------------------------------------------------------- write_memory
//
void write_memory( ) {
  EEPROM_MEMORY memory;

  memory.magic_number = MAGIC_NUMBER;
  memory.program = program;
  memory.brightness = brightness;
  memory.master_mode = master_mode;
  memory.switch_minutes = switch_minutes;
  memory.dmx_address = dmx_address;
  
  for ( int ls=0; ls < NUM_PROGRAMS; ls++ ) {
    memory.sub_program[ls] = getProgram((LightShow)ls)->getSubProgram();
    memory.color_mode[ls] = getProgram((LightShow)ls)->getColorMode();
    memory.speed[ls] = getProgram((LightShow)ls)->getSpeed();
  }
  
  char *p = (char *)&memory;
  for ( int address=0; address < sizeof( EEPROM_MEMORY ); address++ )
    EEPROM.write( address, *p++ );
}  

// ----------------------------------------------------------------------------------------- signalUser
//
void signalUser( int value ) {
  
#if BUTTON_PANEL == PANEL_TEXT
  if ( fault != 0 ) 
    MaxWrite( getProgString( prog_fault ) );
#endif

#if BUTTON_PANEL == PANEL_BINARY
  value |= fault;
  
  digitalWrite( A4, LOW );
  shiftOut( A2, A3, MSBFIRST, value );
  digitalWrite( A4, HIGH );
#endif  
}

// ----------------------------------------------------------------------------------------- signalStateToUser
//
void signalStateToUser( ) {
  
#if BUTTON_PANEL == PANEL_TEXT

  MaxSetRegister( MAX7219_REG_DIGIT4, 0 );

  char *output;
  int value;
  
  switch ( button_mode ) {
    case panel_master:
      output = getProgString( prog_max_master );
      value = master_mode+1;    
      break;
    
    case panel_brightness:
      output = getProgString( prog_max_brightness );
      value = brightness;
      break;
    
    case panel_program: 
      output = getProgString( prog_max_program ); 
      value = program;
      break;      
      
    case panel_color:
      output = getProgString( prog_max_color ); 
      value = getProgram()->getColorMode();
      break;
      
    case panel_subprogram: 
      output = getProgString( prog_max_subprogram ); 
      value = getProgram()->getSubProgram();
      break;       

    case panel_speed: 
      output = getProgString( prog_max_speed ); 
      value = getProgram()->getSpeed();
      break;      
     
    case panel_switching:
      output = getProgString( prog_max_switching );
      value = switch_minutes;
      break; 
     
    case panel_dmx_address:
      output = getProgString( prog_dmx_address );
      value = dmx_address;    
      output[1] = (value/100) + '0';
      value %= 100;
      break;
   }
  
  // This is a hack to get the panel value out - exploits the program string buffer
  
  output[2] = (value/10) + '0';
  output[3] = (value%10) + '0';  
  
  MaxWrite( output );
#endif

#if BUTTON_PANEL == PANEL_BINARY  
  switch ( button_mode ) {
    case panel_master:
      signalUser( master_mode+1 );
      break;
    
    case panel_brightness:
      signalUser( brightness );
      break;
    
    case panel_program: 
      signalUser( program );
      break;      
      
    case panel_color:
      signalUser( getProgram()->getColorMode() );
      break;
      
    case panel_subprogram: 
      signalUser( getProgram()->getSubProgram() );
      break;       

    case panel_speed: 
      signalUser( getProgram()->getSpeed() );
      break;      
     
    case panel_switching:
      signalUser( switch_minutes );
      break;  
    
    case panel_dmx_address:
      signalUser( dmx_address );
      break;
  }  
#endif   
}

#if BUTTON_PANEL == PANEL_BINARY
static prog_uint16_t mode_patterns[NUM_PANEL_OPTIONS] PROGMEM = { 0x8000, 0xC000, 0xE000, 0xF000, 0xF800, 0xFC00, 0xFE00, 0xFF00 };
#endif

// ----------------------------------------------------------------------------------------- buttonPressed
//
void buttonPressed( ) {
  int b1 = digitalRead( BUTTON_1_INPUT );
  int b2 = digitalRead( BUTTON_2_INPUT );
  int b3 = digitalRead( BUTTON_3_INPUT );            // Mode Button program, bright, master
  
#if 1  
  if ( DEBUG ) {
    Serial.print( "Interrupt " );
    Serial.print( b1 );
    Serial.print( " " );
    Serial.print( b2 );
    Serial.print( " " );
    Serial.println( b3 );      
  }
#endif

  if ( b1 ) {              
    button_mode = (PanelMode)((button_mode + 1) % NUM_PANEL_OPTIONS);
    
    signalStateToUser();  
    
    Tlc.clear();
    
#if BUTTON_PANEL == PANEL_BINARY    
    show_shape( pgm_read_word_near( mode_patterns + button_mode ), WHITE, BLACK );
#endif

    Tlc.update();
    
    while( digitalRead( BUTTON_1_INPUT ) )
      ;

    clear_all();
    
    if ( DEBUG ) {    
      reportState();
    }
    return;
  }
  
  if ( !b2 && !b3 )
    return;
    
  switch ( button_mode ) {
    case panel_master:                      // Master mode
       if ( b3 )                            // Change master mode
          master_mode = (MasterMode)((master_mode+1) % NUM_MASTER_MODES);
        else
          master_mode = (MasterMode)((master_mode+NUM_MASTER_MODES-1) % NUM_MASTER_MODES);
          
       setupForMasterMode();                // Setup ports and LEDs
       
       break;
    
    case panel_brightness:
       if ( b3 )                            // Change brightness
        brightness = (brightness+1) % BRIGHTNESS_LEVELS;
      else
        brightness = (brightness+(BRIGHTNESS_LEVELS-1)) % BRIGHTNESS_LEVELS ;
      break;
          
    case panel_dmx_address:
      dmx_address--;
      if ( b3 )                            // Change DMX address
        dmx_address = (dmx_address+1) % 512;
      else
        dmx_address = (dmx_address+511) % 512;
      dmx_address++;
      break;
     
    case panel_color: {
      Program *p = getProgram( program );
      if ( p ) {  
        if ( b3 )
          p->incColorMode();
        else
          p->decColorMode(); 
        restartCurrentProgram = true;       
      }          
      break;    
    }
    
    case panel_speed: {
      Program *p = getProgram( program );
      if ( p ) {  
        if ( b3 )
          p->incSpeed();
        else
          p->decSpeed();          
      }          
      break;    
    }    
    
    case panel_program: 
      if ( b3 )
        program = (LightShow)((program+1) % NUM_PROGRAMS);
      else
        program = (LightShow)((program+NUM_PROGRAMS-1) % NUM_PROGRAMS);               
      break;
 
    case panel_subprogram: {  
      Program *p = getProgram( program );
      if ( p ) {
        if ( b3 )
          p->incSubProgram();
        else
          p->decSubProgram();  
        restartCurrentProgram = true;           
      }
      break;
    }
    
    case panel_switching: {  
      if ( b3 )
        switch_minutes = (switch_minutes + 1) % 30;
      else
        switch_minutes = (switch_minutes+30-1) % 30;  
      restartCurrentProgram = true;      
      break;
    }
  }

  write_memory();
       
  signalStateToUser();
  
  reset_shutdown_timer();
  
  if ( DEBUG ) {    
    reportState();
  }  
}

// ----------------------------------------------------------------------------------------- reset_shutdown_timer
//
void reset_shutdown_timer() {
    master_shutdown = millis() + (MAX_RUNTIME_MINUTES * 60LU * 1000LU );
}

// ----------------------------------------------------------------------------------------- SoundSampler::sample
//
int SoundSampler::sample( void ) {

  int spkr = analogRead(SPKR_INPUT);
  
  int delta = 0;        // Amount of change in peak (neg = falling, pos = rising, zero = no change)
  
  if ( spkr > SPKR_LOUDEST )
    spkr = SPKR_LOUDEST;
   
  int ten_pct_of_peak = m_peak / 10;
  if ( ten_pct_of_peak < 1 )
    ten_pct_of_peak = 1;
    
  if ( spkr < m_peak-ten_pct_of_peak || spkr > m_peak + ten_pct_of_peak  ) {  
   uint32_t ms = millis();
   
    if ( spkr > m_peak ) { 
      delta = spkr - m_peak;    
      m_peak = spkr;
      
      m_peak_hold = ms + m_peak_hold_ms;
    
      if ( DEBUG == true ) {
        Serial.print( getProgString( prog_new_peak ) );
        Serial.println( m_peak );
      }   
    }
    else if ( m_peak > 1 && ms > m_peak_hold ) {
      delta = -ten_pct_of_peak;
      m_peak -= ten_pct_of_peak;
  
      m_peak_hold = ms + m_peak_drop_ms;        
       
      if ( DEBUG == true) {
        Serial.print( getProgString( prog_new_low ) );
        Serial.println( m_peak );
      } 
    }
    
    m_lastPeak = m_peak;
  } 
  
  return delta;
}

// ----------------------------------------------------------------------------------------- SoundSampler::peak2duration
//
int SoundSampler::peak2duration( int low, int high ) {
  int duration;
    
  if ( m_peak >= SPKR_LOUDEST-5 )
    duration = 100;
  else if ( m_peak >= ((SPKR_LOUDEST*2)/3) )
    duration = 250;
  else if ( m_peak >= (SPKR_LOUDEST/2) )
    duration = 400;
  else if ( m_peak >= (SPKR_LOUDEST/4) )
    duration = 800;
  else if ( m_peak >= 5 )
    duration = 1000;
  else
    duration = 4000;  

  if ( duration < low )
    duration = low;
  else if ( duration > high )
    duration = high;

#if 0    
  if ( DEBUG ) { 
    Serial.print( m_peak );
    Serial.print( " .... " );
    Serial.println( duration );
  }
#endif

  return duration;
}

// ----------------------------------------------------------------------------------------- Program::run
//
void Program::run(void) {
  ProgramControl ctl( this, 0 );
  restartCurrentProgram = false;

  if ( master_mode == mode_master ) {
    Serial.print( getProgString(cmd_program) );
    Serial.print( program, DEC );
    Serial.print( getProgString(cmd_subprogram) );
    Serial.print( getProgram()->getSubProgram(), DEC );    
    Serial.print( getProgString(cmd_color) );
    Serial.print( getProgram()->getColorMode(), DEC );      
    Serial.print( getProgString(cmd_speed) );
    Serial.print( getProgram()->getSpeed(), DEC );  
    Serial.print( getProgString(cmd_brightness) );
    Serial.print( brightness, DEC );  
    Serial.println( getProgString(cmd_end) );          

    delay(350);            // Apparently we need to wait for the bits - this is an approximation
  }

  m_function( &ctl ); 
}

// ----------------------------------------------------------------------------------------- ProgramControl::slaveReadSerial
//    
void ProgramControl::slaveReadSerial() {   
    while ( Serial.available() > 0 ) {
      char ch = Serial.read();
      
      switch ( ch ) {
        case 'P':                                          // Program
        case 'p':      m_register = 0;        break;
        case 'u':                                          // Sub program
        case 'U':      m_register = 1;        break;  
        case 'c':                                          // Color
        case 'C':      m_register = 2;        break; 
        case 's':                                          // Speed
        case 'S':      m_register = 3;        break;         
        case 'b':                                          // Brightness
        case 'B':      m_register = 4;        break;          
        case 'm':                                          // Switch minutes
        case 'M':      m_register = 5;        break;
        
        case '.':    
          if ( m_register != -1 ) {
            boolean error = false;
            
            if ( m_reg_value[0] != -1 ) {                  // Set program
              if ( m_reg_value[0] < NUM_PROGRAMS )          
                program = (LightShow)m_reg_value[0];
              else
                error = true;
            }
            
            if ( !error && m_reg_value[1] != -1 ) {        // Set subprogram
              Program *p = getProgram( program );
              if ( m_reg_value[1] < p->getMaxSubProgram() ) {
                p->setSubProgram( m_reg_value[1] );  
                restartCurrentProgram = true;          
              }       
              else
                error = true;             
            }  
            
            if ( !error && m_reg_value[2] != -1  ) {     // Set color
              if ( m_reg_value[2] < COLORS ) {        
                getProgram( program )->setColorMode( m_reg_value[2] );
                restartCurrentProgram = true;
              }
              else
                error = true;
            }  
            
            if ( !error && m_reg_value[3] != -1 ) {       // Speed
              if ( m_reg_value[3] <= MAX_SPEED )         
                getProgram( program )->setSpeed( m_reg_value[3] );
              else
                error = true;
            } 
            
            if ( !error && m_reg_value[4] != -1 ) {       // Brightness
              if ( m_reg_value[4] >= 0 && m_reg_value[4] < BRIGHTNESS_LEVELS )         
                brightness = m_reg_value[4];
              else
                error = true;
            } 
            
            if ( !error && m_reg_value[5] != -1 ) {       // Switch timer
              if ( m_reg_value[5] >= 0 && m_reg_value[5] < 30 ) {
                switch_minutes = m_reg_value[5];
                restartCurrentProgram = (program == m_program->m_id);        
              }        
              else
                error = true;
            }            
            
            Serial.println( getProgString( !error ? prog_ack : prog_nak ) );            

            resetSlave();   
            reportState();            
            signalStateToUser();
            write_memory();              
            reset_shutdown_timer();            
          }       
          break;
         
        default:
          if ( isdigit(ch) && m_register != -1 ) { 
             if ( m_reg_value[ m_register ] == -1 )
                m_reg_value[ m_register ] = 0;            
             m_reg_value[ m_register ] = (m_reg_value[ m_register ] * 10) + (ch-'0');
          }
          else {
             m_register = -1;
             Serial.println( getProgString( prog_nak ) );
          }
          
        case '\n':        // Ignore white space
        case '\r':    
        case ' ':    
          break; 
          
        case '/':
          button_mode = (PanelMode)((button_mode + 1) % NUM_PANEL_OPTIONS);    
          signalStateToUser();          
          break;

      case '?':
          reportState();      
          break;
      }
    }
}

// ----------------------------------------------------------------------------------------- ProgramControl::dmxChannelUpdate
//
void ProgramControl::dmxChannelUpdate()
{
  DMX_CHANNELS* channels = (DMX_CHANNELS*)dmx_data;

  uint8_t dmx_program = channels->dmx_program / 10;
  uint8_t dmx_subprogram = channels->dmx_subprogram / 10;
  uint8_t dmx_color = channels->dmx_color / 10;
  uint8_t dmx_brightness = channels->dmx_brightness / 10;
  uint8_t dmx_speed = channels->dmx_speed / 10;
   
  if ( dmx_program >= NUM_PROGRAMS )
    dmx_program = ColorFill;
    
  program = (LightShow)dmx_program;                        // Set program
  Program *p = getProgram( program );    
   
  bool changed = true;
  
  if ( dmx_color >= COLORS )
    dmx_color = AUTO_COLOR;
    
  if ( dmx_speed >= MAX_SPEED )
    dmx_speed = MAX_SPEED-1;
    
  if ( dmx_brightness >= BRIGHTNESS_LEVELS )
    dmx_brightness = BRIGHTNESS_LEVELS-1;
    
  if ( p->getMaxSubProgram() == 0 )
    dmx_subprogram = 0;
  else if ( dmx_subprogram >= p->getMaxSubProgram() )
    dmx_subprogram = p->getMaxSubProgram()-1;

  if ( dmx_subprogram != p->getSubProgram() ) {            // Set subprogram
    p->setSubProgram( dmx_subprogram );   
    restartCurrentProgram = m_program->getMaxSubProgram() > 0;      
    changed = true;
  }

  if ( dmx_color != p->getColorMode()  ) {                 // Set color
    p->setColorMode( dmx_color );
    restartCurrentProgram = true;      
    changed = true;
  }
            
  if ( dmx_speed != p->getSpeed() ) {                     // Speed
    p->setSpeed( dmx_speed );
    changed = true;
  }
           
  if ( dmx_brightness != brightness ) {                   // Brightness
    brightness = dmx_brightness;
    changed = true;
  }
  
  if ( changed || program != m_program->m_id ) {
    reset_shutdown_timer();
  }
}

// ----------------------------------------------------------------------------------------- ProgramControl::isRunning
//
boolean ProgramControl::isRunning()
{    
  if ( master_mode == mode_slave ) {
    slaveReadSerial();
  }
  else if ( master_mode == mode_dmx ) {
    dmxChannelUpdate();
  }
  
  if ( restartCurrentProgram )
    return false;    

  // Handle program restart, auto shutdown, and auto program change
  
  if ( ++m_check >= 1000 ) {       // Not sure how expensive millis() is (yet)
    unsigned long time = millis();
         
    if ( time >= master_shutdown ) {
      program = Off;
    }
    else if ( m_stop_time > 0 && time >= m_stop_time ) {
      return false;
    }
    else {
      if ( m_switch_time != 0 && program != Off && time > m_switch_time && master_mode != mode_slave && master_mode != mode_dmx ) {
        do {
          program = (LightShow)random(NUM_PROGRAMS);
        }
        while ( !getProgram(program)->isInAutoMode() );
      }
    }

    m_check = 0;
  }
 
  return program == m_program->m_id;
}

// USART ISR - Since there can only be one of these, you need to comment out the ISR in 
// ..\arduino-1.0.1\hardware\arduino\cores\arduino\HardwareSerial.cpp.  These are easily
// identified as they also start with "ISR(...)"
//
// This handler is largely based on http://arduino.cc/playground/DMX/Ardmx and adapted to
// arduinio 1.01 for the UNO.
//
ISR(USART_RX_vect) {
  static  uint16_t DmxCount;
  
  if ( master_mode != mode_dmx )
    return;

  digitalWrite( DMX_RX_STATUS_PIN, HIGH );      // This is purely cosmetic and can be removed    
    
  uint8_t  USARTstate= UCSR0A;                  // Get USART state before data!
  uint8_t  DmxByte   = UDR0;	                // Get data
    
  if ( USARTstate & (1<<FE0) ) {                // Check for break
    DmxCount = dmx_address;		        // Reset channel counter (count channels before start address)
    gDmxState = BREAK;
  }
  else {
    switch ( gDmxState ) {
      case BREAK:
        if ( DmxByte == 0 ) 
          gDmxState = STARTB;                     // Normal start code detected
        else 
          gDmxState = IDLE;
        break;
  
      case STARTB:
        if ( --DmxCount == 0 ) {	          // Start address reached?
          DmxCount = 1;		                  // Set up counter for required channels
          dmx_data[0] = DmxByte;                  // Get 1st DMX channel of device
          gDmxState = STARTADR;
        }
        break;
  
      case STARTADR:
        dmx_data[DmxCount++]= DmxByte;            // Get channel value
        if ( DmxCount >= sizeof(dmx_data) )       // All channels received?
          gDmxState = IDLE;                       //   wait for next break        
        break;
    }	        
  }
  
  digitalWrite( DMX_RX_STATUS_PIN, LOW );  				
}

