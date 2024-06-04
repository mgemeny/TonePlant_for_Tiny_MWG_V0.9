//#define PROCESSORmega328 // You can move the timers around but beware that the output pins will change if you set TONEonCTC1A or change pwm_mode = CHA | CHB | DIFF. Your timer one is 16 bit timers on this CPU
                         // Timer2 is 8 bit.
                         // With this processor enabled valid options are ((no TIMEonCTC0) (time defaults to CTC1) TONEonCTC2 and (pwm_mode = CHA | CHB | DIFF )) or (TIMEonCTC0 and (TONEonCTC2 | TONEonCTC1) and (pwm_mode = CHA | CHB | DIFF ))
                         //
//#define PROCESSORtiny84  //This 14 pin processor only has 2 timers, zero and one, and timer one is a 16 bit timer (same as the mega328 above).
                         // So, you must set TIMEonCTC0, TONEonCTC1A, and pwm_mode = CHA.
                         // 
                         //
#define PROCESSORtiny85  //This 8 pin processor only has 2 timers, zero and one, and both timers are 8 bit.
                         // With this processor enabled the only valid options are TIMEonCTC0 and TONEonCTC1 and pwm_mode = CHA
                         // Your tone is on DIP pin 6, your tuner pot goes on DIP pin 3, and your function selector pot goes on DIP pin 7
                         //
                         // To summarize some of the differences on processors above, they all have an 8 bit timer0, the 328 has two additional timers, T1 is 16bit, T2 is 8bit .
                         // Both tiny processors only have two timers 0 and 1, in the 84 timer 1 is 16 bit and in the 85 timer one is 8 bit.
                         //
#define TIMEonCTC0       // This will move the timer INT to Compare Register B of CTC0 without breaking mills or mocros of the Arduino environment
                         // This clears timer1 for PWM on processors that only have 2 timers like the ATtiny84 or ATtiny85 but should still be OK for ATmega328
                         // This will also change the waveform steps from 40,000Hz to 37,714.2957Hz with a step of 7 ticks on the compare register, or 19,230.7692Hz with a step of 13 ticks
                         //
//#define TONEonCTC2
#define TONEonCTC1      // This will move the PWM output signal to timer one
                         // I know, it's obnoxious in that it stomps on your serial port on an Uno, but it's one of the very few options that would also work on an ATtiny85 or ATtiny84
                         // this should be OK on any of the 3 processors above. But you must set TIMEonCTC0 above
                         //
//
// Remenber to set pwm_mode below
//

                         //
#include "EEPROM.h" // Adds nothing if you don't use it
/*                                                                        
 * MWG_V1 changes user interface to pots                                                                          
 * MWG_V2 adds a tuner pot Tonegen just grabs the global value and runs with it.
 * MWG_V3 Timer moved with TIMEonCTC0, gitter fixed, tuned to match the original. Now moving on to Tiny85_V4 to move the PWM to timer 1 for the Tiny84-85
 * MWG_V4 Added processor types, setOCRnloutput routines, 
 * MWG V5 tested moving timers all around on UNO with various pwm_modes (now moving on to attiny85 testing and debuging) 1/11/2024
 * MWG V6 Completed testing and tuning for ATtiny85! WOW! Dial Tone from an 8 pin chip! Moving on to the ATtiny84 where we'll have more pins! 1/12/2024
 * MWG V7 Moved tuner from A3 to A2. ATtiny 84 mostly done. discovered that TIMEonCT1 became broken for the Mega somewhere along the way. We'll look at that in V8 as we work on the user interface.
 * MWG V8 Just adds comments and cleans things up a bit.
 * MWG V0.9 changes version number convention
 *//*
  Telephone Switch Tone Plant Generator
  by: Don Froula
  7/20/2019
  Tonegen library supports up to 4 simultaneous tones
  on a single output pin. The output uses a 40KHz sampling frequency and 62.5KHz PWM frequency.
  These frequencies must be filtered from the output with a low pass filter.
  The PWM library generates tones with a sine wave/square wave lookup table, varying the pulse width of the output according
  to values in the 8-bit table. The low pass filter integrates the varying pulse width to a varying
  analog voltage. Nominal voltage at silence ("127") is 2.5 volts. Using differential output modes
  increases this to 5 volts, doubling the tone output voltage and increasing the volume.
*/
#define SET(x,y) (x |=(1<<y))                  //-Bit set/clear macros
#define CLR(x,y) (x &= (~(1<<y)))             // |
#define CHK(x,y) (x & (1<<y))                 // |
#define TOG(x,y) (x^=(1<<y))                  //-+

volatile signed long tunerValue=0; //This is a tuning value from a pot that tonegen will use
//#include "tonegen.h"  //Tone generation library

const unsigned char mLowtone =      B0000;    //Modulated Low Tone (600x120)
const unsigned char mRinging1 =     B0001;    //Modulated Ringing 1 (420x40)
const unsigned char mRinging2 =     B0010;    //Modulated Ringing 2 (500x40)
const unsigned char hz500 =         B0011;    //Old High Tone (500)
const unsigned char pDialtone =     B0100;    //Precise Dial Tone (350+440)
const unsigned char pRinging =      B0101;    //Precise Ringing (440+480)
const unsigned char pLowtone =      B0110;    //Precise Low Tone (480+620)
const unsigned char rohtones =      B0111;    //Receiver Off-Hook (ROH) tone (1400+2060+2450+2600)
const unsigned char hz480 =         B1000;    //New High Tone (480)
const unsigned char hz1004 =        B1001;    //Test Tone (1004)
const unsigned char ukOlddialtone = B1010;    //UK Old Dial Tone (400x33)
const unsigned char ukOldringing =  B1011;    //UK Old Ringing (400x133)
const unsigned char ukpDialtone =   B1100;    //UK Precise Dial Tone (350+450)
const unsigned char ukpRinging =    B1101;    //UK Ringing (400+450)
const unsigned char hz400 =         B1110;    //UK Equipment Engaged Tone/Congestion/Number Unavailable tone (400)
const unsigned char crybaby =       B1111;    //Crybaby (US Vacant Number) (200 to 400 to 200 at 1 Hz, interrupted every 6 seconds for 500ms)

const unsigned char  continuous =      B0000;  //Always ON (Continuous Tone)
const unsigned char  precisereorder =  B0001;  //250 ON 250 OFF (Precise Reorder)
const unsigned char  tollreorder =     B0010;  //200 ON 300 OFF (Toll Reorder)
const unsigned char  localreorder =    B0011;  //300 ON 200 OFF (Local Reorder)
const unsigned char  busy =            B0100;  //500 ON 500 OFF (Busy)
const unsigned char  roh =             B0101;  //100 ON 100 OFF (Receiver off hook)
const unsigned char  ring =            B0110;  //2000 ON 4000 OFF (Standard US Ring)
const unsigned char  partyline2 =      B0111;  //1000 ON 1000 OFF 1000 ON 3000 OFF (No. 5 Xbar Coded Ring 2)
const unsigned char  partyline3 =      B1000;  //1500 ON 500 OFF 500 ON 3500 OFF (No. 5 Xbar Coded Ring 3)
const unsigned char  partyline4 =      B1001;  //1500 ON 500 OFF 500 ON 500 OFF 500 ON 2500 OFF (No. 5 Xbar Coded Ring 4)
const unsigned char  partyline5 =      B1010;  //1500 ON 500 OFF 500 ON 500 OFF 1500 ON 1500 OFF (No. 5 Xbar Coded Ring 5)
const unsigned char  ukring =          B1011;  //400 ON 200 OFF 400 ON 2000 OFF (UK Ring)
const unsigned char  ukinvertedring =  B1100;  //400 OFF 200 ON 400 OFF 2000 ON (UK Inverted Ring)
const unsigned char  ukbusy =          B1101;  //375 ON 375 OFF (UK Busy)
const unsigned char  ukreorder =       B1110;  //400 ON 350 OFF 225 ON 525 OFF (UK Reorder)
const unsigned char  ukoldbusy =       B1111;  //750 ON 750 OFF (UK Old Busy)

const byte pots =3; // The number of pots that we have on this system
byte pot =0; // Our pointer to which pot we are working with
//int potPin [pots]; // The Analog pin number of the particular pots
byte potPin [pots]; // The Analog pin number of the particular pots
int potLastRead [pots]; //Our working veriables to filter the readings
byte pot0to255 [pots]; // Our output for other routines to use
byte scratch; //A scratch variable to let setup loop run in the values

void readonepot()
/* This code was tested with a Bourns 2399 25 Turn, 50K trimmer pot.
 * If you use a one more than 50K, your results may be less stable.
 * If you can afford the current and use one smaller than 50k, the results should be more stable.
 * If you use a pot less than 25 turns, you'll likely be dissapointed, although 10K or less and 20 turns could be workable
 * We'd recommend a 25 turn 2.2k, set pot in the middle of setting desired, and test for stable results after reset.
 * If you are off center of setting desired, it may take time to lock in or may not lock in at all to the setting desired after reset.*/
{

#ifndef PROCESSORtiny84 // if NOT an ATtiny84
  if (potLastRead[pot] != analogRead(potPin [pot])) { //If it is not the same, only then try again
  potLastRead [pot] = analogRead(potPin [pot])  ;     //we would have to have read it as diffenent from before twice to have changed it
                                                                   //making it a majority vote, 2 of 3 between the prior run and these two trys
  }
#endif

#ifdef PROCESSORtiny84 // if it IS an ATtiny84
  //   This processor seems to have different behavior if the analog pins are left floating. It seems that if you read a floating pin and then try to read an adjacent connected pin,
  //   you seem to get a bazzar reading. As a result, we turned on pull up resistors on our analog input pins. 
  //   As a result of that work around, we can no longer get to a reading of zero, so well fix that here.
  //
  //   This map function adds 312 bytes to the code. If you find yourself needing that space and haven't used map anywhere else, you may consider a different approach.
  //
  if (potLastRead[pot] !=      map(    analogRead(potPin [pot])  ,12,1023, 0,1023 )       ) { //If it is not the same, only then try again
  potLastRead [pot] =          map(    analogRead(potPin [pot])  ,12,1023, 0,1023 )           ; //we would have to have read it as diffenent from before twice to have changed it
                                                                   //making it a majority vote, 2 of 3 between the prior run and these two trys
  }
#endif

  
  pot0to255 [pot]=((potLastRead [pot]+1 + pot0to255 [pot]*4)/2)/4; // Just a tiny bit more digital filtering or averaging for gitter and divide by 4
}

void readallofthepots()
{
for ( pot = 0; pot < pots; ++pot ) {
 readonepot();
  }
  //tunerValue = (potLastRead [1] -512)*50; //produce a tuning value to be passed to a modified tonegen (A2 is [1])
  tunerValue = (potLastRead [1] -512)*15; //produce a tuning value to be passed to a modified tonegen (A2 is [1])
  //tunerValue = 0;            //<------------------ TESTING
}



unsigned char pwm_mode; //Storage for PWM libray output mode and ouput pin assignments

unsigned char selected_tone = mLowtone;
unsigned char selected_cadence = continuous;

// This could probably be moved to tonegen and perhaps be put in the code itself manipulating the registers directly
// These routines will let PWM outputs be set as output. The registers and bits map to the Digital Pin and not the PWM source.
// What's more is that it varies based on the processor, so we'll just do this once and reference these routines.

void setOCR1Aoutput()
{
 #ifdef PROCESSORmega328
  // OCR1A PWM output is on PB1 (DIP pin 15)
  SET(DDRB, DDB1);                                   //-PWM pin B1
  //pinMode(PB1, OUTPUT);//That didn't work but the one above did.
 #endif

 #ifdef PROCESSORtiny84
  // OCR1A PWM output is on PA6 (DIP pin 7) Note that this is the only one on Data Direction Register A
  SET(DDRA, DDA6);                                   //-PWM pin A6 DIP pin 7
  //pinMode(PA6, OUTPUT); 
 #endif

 #ifdef PROCESSORtiny85
  // OCR1A PWM output is on PB1 (DIP pin 6)
  //SET(DDRB, DDB1);
  pinMode(PB1, OUTPUT);  //That seems to work
 #endif
}

void setOCR1Boutput()
{
 #ifdef PROCESSORmega328
  // OCR1B PWM output is on PB2 (DIP pin 16)
  SET(DDRB, DDB2); //But that works
  //pinMode(PB2, OUTPUT); //That didn't work
 #endif

 #ifdef PROCESSORtiny84
  // OCR1B PWM output is on PA5 (DIP pin 8)
  SET(DDRB, DDA5);
  //pinMode(PA5, OUTPUT);// 
 #endif

 #ifdef PROCESSORtiny85
  // OCR1B PWM output is on PB4 (DIP pin 3)
  //SET(DDRB, 0x10);
  //DDRB = DDRB | B00010000;  //Set Data Direction Bit DDB4
  //pinMode(PB4, OUTPUT); //The tuner pot goes here, so we really don't ever want to do this.
 #endif
}

void setOCR2Aoutput()
// This only exists on the mega328 
// if you have a mismatch the between the IDE and selected processor above, it just will not compile.
{
 #ifdef PROCESSORmega328
  // OCR2A PWM output is on PB3 (DIP pin 17)
  //SET(DDRB, DDB3);
  pinMode(PB3, OUTPUT);
 #endif
}

void setOCR2Boutput()
// This only exists on the mega328 
{
 #ifdef PROCESSORmega328
  // OCR2B PWM output is on PD3 (DIP pin 5) Note that this is the only one on Data Direction Register D
  //SET(DDRD, DDD3);
  pinMode(PD3, OUTPUT);
 #endif
}

#include "tonegen.h"  //Tone generation library
//Instaniate PWM tone generator
tonegen tonePlayer;


void setup() {

//  EEPROM.get(0, scratch);// Thesting to see how much it adds to the code size. This only adds 30 BYTES! Yeay!
/*   Serial.begin(9600); // Initialize serial port for debugging
   Serial.print("\n  Reset - \J \m");
  Serial.print("\n  TCCR0A - \t");
  Serial.println(TCCR0A, HEX);
  Serial.print("\n  TCCR0B - \t");
  Serial.println(TCCR0B, HEX);

  CLR (TCCR0A, WGM00); 
    CLR (TCCR0A, WGM01); 
    CLR (TCCR0B, WGM02); 

      Serial.print("\n  TCCR0A - \t");
  Serial.println(TCCR0A, HEX);
  Serial.print("\n  TCCR0B - \t");
  Serial.println(TCCR0B, HEX);*/

  // Configure the tone select pins for input - Upper nibble of Port D, Atmega328p pins 13,12,11,6 (from MSB to LSB)
//  DDRD = DDRD &
//         ~ (
//           (1 << DDD4) |  // pinMode( 4, INPUT ); // Set to input - Pin 6
//           (1 << DDD5) |  // pinMode( 5, INPUT ); // Set to input - Pin 11
//           (1 << DDD6) |  // pinMode( 6, INPUT ); // Set to input - Pin 12
//           (1 << DDD7)    // pinMode( 7, INPUT ); // Set to input - Pin 13
//         );

  // Enable the pullups
//  PORTD = PORTD |
//          (
//            (1 << PORTD4) |  // digitalWrite( 4, HIGH ); // Enable the pullup
//            (1 << PORTD5) |  // digitalWrite( 5, HIGH ); // Enable the pullup
//            (1 << PORTD6) |  // digitalWrite( 6, HIGH ); // Enable the pullup
//            (1 << PORTD7)    // digitalWrite( 7, HIGH ); // Enable the pullup
//          );

//  // Configure the cadence select pins for input - Lower nibble of Port B, Atmega328p pins 17,16,15,14 (from MSB to LSB)
//  DDRB = DDRB &
//         ~ (
//           (1 << DDB0) |  // pinMode( 8, INPUT ); // Set to input - Pin 14
//           (1 << DDB1) |  // pinMode( 9, INPUT ); // Set to input - Pin 15
//           (1 << DDB2) |  // pinMode( 10, INPUT ); // Set to input - Pin 16
//           (1 << DDB3)    // pinMode( 11, INPUT ); // Set to input - Pin 17
//         );

  // Enable the pullups
//  PORTB = PORTB |
//          (
//            (1 << PORTB0) |  // digitalWrite( 8, HIGH ); // Enable the pullup
//            (1 << PORTB1) |  // digitalWrite( 9, HIGH ); // Enable the pullup
//            (1 << PORTB2) |  // digitalWrite( 10, HIGH ); // Enable the pullup
//            (1 << PORTB3)    // digitalWrite( 11, HIGH ); // Enable the pullup

//          );

#ifdef PROCESSORmega328
  //Set audio output physical pin 5 as output
  //DDRD |= (1 << DDD3);
  //setOCR2Boutput();

  //Set relay driver physical pin 4 as output
  DDRD |= (1 << DDD2);

  //Set cadence LED pin as output
  DDRB |= (1 << DDB5);

  //Turn off relay output
  PORTD = PORTD & B11111011;

  //Turn off cadence LED
  PORTB = PORTB & B11011111;
#endif

#ifdef PROCESSORtiny85
  //setOCR1Aoutput(); // This gets done by tonegen so we really don,t need to do it here.
  pinMode(PB0, OUTPUT); // Set DIP pin 5 as output for the cadence
#endif



#ifdef PROCESSORtiny84
//SET(DDRB, DDB0);     // Set DIP pin 13 as output for the cadence
//pinMode(PB0, OUTPUT); // Set DIP pin 13 as output for the cadence
#endif



//Uncomment ONE of the following three lines to set audio output mode and pin
  //pwm_mode = CHA;  //Single ended signal in CHA pin (11) - Physical Pin 17 on chip
  //pwm_mode = CHB;  //Single ended signal on CHB pin (3) - Physical Pin 5 on chip (or if define TONEonCTC1 Arduino pin 10 of Uno)
  pwm_mode = DIFF; //Differntial signal on CHA and CHB pins (11,3) - Physical Pin 17 and Physical Pin 5 on chip

  tonePlayer.begin(pwm_mode); //Sets up output pins and single-ended or differential mode, according to the uncommented mode above.

  //Set default wave table for each voice
  tonePlayer.setWave(SINE, SINE, SINE, SQUARE);

  tonePlayer.setVolume(60, 60, 60, 60);




// set up the user interface pots

#ifdef PROCESSORtiny84
potPin [0]=A1; // set the pot pin for this pot to A1 (Function)
potPin [1]=A2; // set the pot pin this pot to A2 (Tuner)
potPin [2]=A3; // set the pot pin this pot to A3
#endif

#ifdef PROCESSORtiny85
potPin [0]=A1; // set the pot pin for this pot to A1 (Function)
potPin [1]=A3; // set the pot pin this pot to A2 (Tuner)
potPin [2]=A3; // set the pot pin this pot to A3
#endif

#ifdef PROCESSORmega328
potPin [0]=A1; // set the pot pin for this pot to A1 (Function)
potPin [1]=A2; // set the pot pin this pot to A2 (Tuner)
potPin [2]=A3; // set the pot pin this pot to A3
#endif



 #ifdef PROCESSORtiny84
pinMode(1, INPUT_PULLUP);
pinMode(2, INPUT_PULLUP);
pinMode(3, INPUT_PULLUP);
 #endif

// and run in the values before we start making tones
for ( scratch = 0; scratch < 20; ++scratch ) {
 readallofthepots();
  }
}


//Main loop
void loop()
{

  // Read all input pins and set tone and cadence variables
  
  //Read Port D, shift to the right four places, take the complement, and mask the upper 4 bits
//  selected_tone = (~((PIND & ( (1 << PIND4) | (1 << PIND5) | (1 << PIND6) | (1 << PIND7) )) >> 4) & B00001111);

  //Read Port B, take the complement, and mask the upper 4 bits
//  selected_cadence = (~(PINB & ( (1 << PINB0) | (1 << PINB1) | (1 << PINB2) | (1 << PINB3) )) & B00001111);

readallofthepots(); // 
selected_tone = (pot0to255[0] & B00001111);
selected_cadence = (pot0to255[0] & B11110000)/16;


//selected_tone = 0b00001001; // <-------------TESTING 1004
//selected_cadence = 0b00000000; Steady
//    Dial Tone
//selected_tone = 0b00000100; // pDialtone
//selected_cadence = 0b00000000; //Steady 
//    Busy
//selected_tone = 0b00000110; // Busy
//selected_cadence = 0b00000100;
//    Ring
//selected_tone = 0b00000101; // pRing
//selected_cadence = 0b00000110;
  
  playcadence();

}
//End of loop


void playcadence()
{
  switch (selected_cadence) {
    case continuous:
      playtone(0);
      break;
    case precisereorder:
      playtone(250);
      delay(250);
      break;
    case tollreorder:
      playtone(200);
      delay(300);
      break;
    case localreorder:
      playtone(300);
      delay(200);
      break;
    case busy:
      playtone(500);
      delay(500);
      break;
    case roh:
      playtone(100);
      delay(100);
      break;
    case ring:
      playtone(2000);
      delay(4000);
      break;
    case partyline2:
      playtone(1000);
      delay(1000);
      playtone(1000);
      delay(3000);
      break;
    case partyline3:
      playtone(1500);
      delay(500);
      playtone(500);
      delay(3500);
      break;
    case partyline4:
      playtone(1500);
      delay(500);
      playtone(500);
      delay(500);
      playtone(500);
      delay(2500);
      break;
    case partyline5:
      playtone(1500);
      delay(500);
      playtone(500);
      delay(500);
      playtone(1500);
      delay(1500);
      break;
    case ukring:
      playtone(400);
      delay(200);
      playtone(400);
      delay(2000);
      break;
    case ukinvertedring:
      playtone(200);
      delay(400);
      playtone(2000);
      delay(400);
      break;
    case ukbusy:
      playtone(375);
      delay(375);
      break;
    case ukreorder:
      playtone(400);
      delay(350);
      playtone(225);
      delay(525);
      break;
    case ukoldbusy:
      playtone(750);
      delay(750);
      break;
    default:
      playtone(0);
      break;
  }
}


// Total volume on a tone for all four voices MAY NOT EXCEED 250!!!!!!!!
void playtone(unsigned long length)
{
  switch (selected_tone) {
    case mLowtone:
      tonePlayer.setWave(SINE, SINE, SINE, SQUARE);
      //tonePlayer.setVolume(80, 80, 58, 22);
      tonePlayer.setVolume(80, 80, 80, 0);
      playPWM(480, 720, 600, 0, length);
      break;
    case mRinging1:
      tonePlayer.setWave(SINE, SINE, SINE, SQUARE);
      tonePlayer.setVolume(80, 80, 80, 0);
      playPWM(380, 460, 420, 0, length);
      break;
    case mRinging2:
      tonePlayer.setWave(SINE, SINE, SINE, SQUARE);
      //tonePlayer.setVolume(80, 80, 68, 12);
      tonePlayer.setVolume(80, 80, 80, 0);
      playPWM(460, 540, 500, 0, length);
      break;
    case hz500:
      tonePlayer.setWave(SINE, SINE, SINE, SQUARE);
      tonePlayer.setVolume(240, 0, 0, 0);
      playPWM(500, 0, 0, 0, length); // <----Was 500
      break;
    case pDialtone:
      tonePlayer.setWave(SINE, SINE, SINE, SQUARE);
      tonePlayer.setVolume(120, 120, 0, 0);
      playPWM(350, 440, 0, 0, length);
      break;
    case pRinging:
      tonePlayer.setWave(SINE, SINE, SINE, SQUARE);
      tonePlayer.setVolume(120, 120, 0, 0);
      playPWM(440, 480, 0, 0, length);
      break;
    case pLowtone:
      tonePlayer.setWave(SINE, SINE, SINE, SQUARE);
      tonePlayer.setVolume(120, 120, 0, 0);
      playPWM(480, 620, 0, 0, length);
      break;
    case rohtones:
      tonePlayer.setWave(SINE, SINE, SINE, SINE);
      tonePlayer.setVolume(60, 60, 60, 60);
      playPWM(1400, 2060, 2450, 2600, length);
      break;  
    case hz480:
      tonePlayer.setWave(SINE, SINE, SINE, SQUARE);
      tonePlayer.setVolume(240, 0, 0, 0);
      playPWM(480, 0, 0, 0, length);
      break;
    case hz1004:
      tonePlayer.setWave(SINE, SINE, SINE, SQUARE);
      tonePlayer.setVolume(240, 0, 0, 0);
      playPWM(1004, 0, 0, 0, length);
      break;
    case ukOlddialtone:
      //tonePlayer.setWave(SQUARE, SQUARE, SQUARE, SQUARE);
      tonePlayer.setWave(SINE, SINE, SINE, SINE); //<====== Testing Yea, square is a problem and gets stuck cadensing and won't let the function selector pot sweep past it.
      tonePlayer.setVolume(20, 20, 20, 180);
      playPWM(434, 366, 400, 33, length);
      break;
    case ukOldringing:
      //tonePlayer.setWave(SQUARE, SQUARE, SQUARE, SQUARE);
      tonePlayer.setWave(SINE, SINE, SINE, SINE); //<====== Testing Yea, square is a problem and gets stuck cadensing and won't let the function selector pot sweep past it.
      tonePlayer.setVolume(20, 20, 20, 180);
      //playPWM(433,367,400,133,length);
      playPWM(434, 366, 400, 133, length);
      break;
    case ukpDialtone:
      tonePlayer.setWave(SINE, SINE, SINE, SQUARE);
      tonePlayer.setVolume(120, 120, 0, 0);
      playPWM(350, 450, 0, 0, length);
      break;
    case ukpRinging:
      tonePlayer.setWave(SINE, SINE, SINE, SQUARE);
      tonePlayer.setVolume(120, 120, 0, 0);
      playPWM(400, 450, 0, 0, length);
      break;
    case hz400:
      tonePlayer.setWave(SINE, SINE, SINE, SQUARE);
      tonePlayer.setVolume(240, 0, 0, 0);
      playPWM(400, 0, 0, 0, length);
      break;
    case crybaby:
    if (selected_cadence == continuous) {  //Only play crybaby if set to continuous cadence, otherwise silence
      tonePlayer.setWave(RAMP, SINE, SQUARE, SQUARE);
      tonePlayer.setVolume(250, 0, 0, 0);
      starttones();
      for (int j = 1; j <= 6; j++) { //This was making things hard with pots and getting stuck
        for (int i = 0; i < 100; i++) {
           //Read exponential table to mimic capacitor discharge of the real Crybaby circuit (frequency rises)
           changetones(pgm_read_float(discharge_sequence + i), 0, 0, 0);
           delay(5);
           } // end of i loop
         for (int i = 0; i < 100; i++) {
          //Read exponential table to mimic capacitor charge of the real Crybaby circuit (frequency falls)
           changetones(pgm_read_float(charge_sequence + i), 0, 0, 0);
           delay(5);
         } //end of I loop
        } //This was making things hard with pots and getting stuck
      stoptones();
      //delay(500);
      }
      else {  //Play silence
        tonePlayer.setWave(SINE, SINE, SINE, SINE);
        tonePlayer.setVolume(120, 0, 0, 0);
        playPWM(400, 0, 0, 0, length);
        }
      break;
  }
}


//Play PWM-generated mixed tones
void playPWM(float freq1, float freq2, float freq3, float freq4, unsigned long length)
{
  tonePlayer.setFrequency(0, freq1); //Set the new Tone 1 frequency
  tonePlayer.setFrequency(1, freq2); //Set the new Tone 2 frequency
  tonePlayer.setFrequency(2, freq3); //Set the new Tone 3 frequency
  tonePlayer.setFrequency(3, freq4); //Set the new Tone 4 frequency
  tonePlayer.resume(); //Turn on PWM interrupts
  
  #ifdef PROCESSORmega328
    PORTD = PORTD | B00000100;  //Turn on relay output
    PORTB = PORTB | B00100000;  //Turn on cadence LED
  #endif

  #ifdef PROCESSORtiny85
    digitalWrite(PB0, HIGH); //Turn on cadence LED and/or relay
  #endif
   
  if (length > 0) {
    delay(length); //Wait for the tone duration
    tonePlayer.suspend(); //Turn off PWM interrupts
    
    #ifdef PROCESSORmega328
      PORTD = PORTD & B11111011;  //Turn off relay output
      PORTB = PORTB & B11011111;  //Turn off cadence LED
    #endif

    #ifdef PROCESSORtiny85
      digitalWrite(PB0, LOW); //Turn off cadence LED and/or relay
    #endif
    
    tonePlayer.setFrequency(0, 0); //Set for silence
    tonePlayer.setFrequency(1, 0); //Set for silence
    tonePlayer.setFrequency(2, 0); //Set for silence
    tonePlayer.setFrequency(3, 0); //Set for silence
  }
}

//Change frequencies
void changetones(float freq1, float freq2, float freq3, float freq4)
{
  tonePlayer.setFrequency(0, freq1); //Set the new Tone 1 frequency
  tonePlayer.setFrequency(1, freq2); //Set the new Tone 2 frequency
  tonePlayer.setFrequency(2, freq3); //Set the new Tone 3 frequency
  tonePlayer.setFrequency(3, freq4); //Set the new Tone 4 frequency
}

void starttones()
{
  tonePlayer.setFrequency(0, 0); //Set for silence
  tonePlayer.setFrequency(1, 0); //Set for silence
  tonePlayer.setFrequency(2, 0); //Set for silence
  tonePlayer.setFrequency(3, 0); //Set for silence
  tonePlayer.resume(); //Turn on PWM interrupts
  
  #ifdef PROCESSORmega328
    PORTD = PORTD | B00000100;  //Turn on relay output
    PORTB = PORTB | B00100000;  //Turn on cadence LED
  #endif
  
  #ifdef PROCESSORtiny85
    digitalWrite(PB0, HIGH); //Turn on cadence LED and/or relay
  #endif
  
}

void stoptones()
{
  tonePlayer.suspend(); //Turn off PWM interrupts
  
  #ifdef PROCESSORmega328
    PORTD = PORTD & B11111011;  //Turn off relay output
    PORTB = PORTB & B11011111;  //Turn off cadence LED
  #endif
  
  #ifdef PROCESSORtiny85
    digitalWrite(PB0, LOW); //Turn off cadence LED and/or relay
  #endif
  
  tonePlayer.setFrequency(0, 0); //Set for silence
  tonePlayer.setFrequency(1, 0); //Set for silence
  tonePlayer.setFrequency(2, 0); //Set for silence
  tonePlayer.setFrequency(3, 0); //Set for silence
}

/*
                                                        ATmega328
                                                       +---\/---+
                                         (RESET)  PC6  |1     28|  PC5  (ADC5 / SCL)
                                           (RXD)  PD0  |2     27|  PC4  (ADC4 / SDA)
                                           (TXD)  PD1  |3     26|  PC3  (ADC3) <--------------------- Tuner Pot IN
                                          (INT0)  PD2  |4     25|  PC2  (ADC2) 
                                          (INT1)  PD3  |5     24|  PC1  (ADC1) <--------------------- Function Selector Pot IN
                                      (XCK / T0)  PD4  |6     23|  PC0  (ADC0)
                                                  VCC  |7     22|  GND
                                                  GND  |8     21|  AREF
                                 (XTAL1 / TOSC1)  PB6  |9     20|  AVCC
                                 (XTAL2 / TOSC2)  PB7  |10    19|  PB5  (SCK)
                                            (T1)  PD5  |11    18|  PB4  (MISO)
                                          (AIN0)  PD6  |12    17|  PB3  (MOSI / OC2)
                                          (AIN1)  PD7  |13    16|  PB2  (SS / OC1B)
                                          (ICP1)  PB0  |14    15|  PB1  (OC1A)
                                                       +--------+


This Processor has enough pins for additional cadence outputs. It only has 2 timers and timer0 is tied up with millis() and too slow for us, so we're limited to 2 tones out.
It also will not run 16MHz without an external OSC so we're running it at 8MHz. At this speed, it really can't do the Receiver-Off-Hook tone.
An external OSC will take 2 pins, so the net gain by steppin up to this chip is 4 additional IO pins over the ATtiny85

                                                        ATtiny84
                                                       +---\/---+
Vcc 5V IN   --------------------->                VCC  |1     14|  GND         <--------------------- Ground IN
                                         (XTAL1)  PB0  |2     13|  PA0  (ADC0)
                                         (XTAL2)  PB1  |3     12|  PA1  (ADC1) <--------------------- Function Selector Pot IN
                                         (RESET)  PB3  |4     11|  PA2  (ADC2) <--------------------- Tuner Pot IN
                                   (INT0 / OC0A)  PB2  |5     10|  PA3  (ADC3) <--------------------- Tuner Pot IN (may actually be here, please check this)
                                   (ADC7 / OC0B)  PA7  |6      9|  PA4  (ADC4 / SCK / SCL)
PWM Audio OUT -------------------> (ADC6 / OC1A)  PA6  |7      8|  PA5  (ADC5 / OC1B / MISO)) <------ Future PWM Audio2 out
                                                       +--------+



                                                        ATtiny85
                                                       +---\/---+
                                  (RESET / ADC0)  PB5  |1*     8|  VCC         <--------------------- Vcc 5V IN
Tuner Pot IN  ------------------>         (ADC3)  PB3  |2      7|  PB2  (ADC1) <--------------------- Function Selector Pot IN
Future PWM Audio2 OUT | Cadence2 > (OC1B / ADC2)  PB4  |3      6|  PB1  (OC0B / OC1A) <-------------- PWM Audio OUT
Ground IN  --------------------->                 GND  |4      5|  PB0  (OC0A) <--------------------- Cadence OUT 
                                                       +--------+


FUTURE CONSIDERATIONS

"TIMER_TO_USE_FOR_MILLIS" could be worth exploring for some ATtiny cores.

The Bourns resister array 4610X-R2R-103LF (1R 2R resister ladder array aka parallel D to A converter) could be handy for the user interface.


*/
