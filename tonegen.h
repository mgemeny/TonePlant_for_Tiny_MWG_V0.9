
#ifndef _TONEGEN
#define _TONEGEN
//*************************************************************************************
//  Arduino Tone Generator MODIFIED by M. W. Gemeny for his TonePlant
//  MWG V2 added a tuning pot in this version, July 15 2022.
//  MWG V5 tested moving timers all around on UNO with various pwm_modes 1/11/2024
//  MWG V6 finished attiny85 support
//  MWG V7 ATtiny84 support
// Generates four simultaneous square/sine wave tones at any relative amplitude
//*************************************************************************************
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "tables.h"

#define DIFF 1
#define CHA 2
#define CHB 3

#define SINE      0
#define SQUARE    1
#define RAMP      2

#define FS 40000.0                            //-Rate must be evenly divisable into 16,000,000. Higher rates have worse frequency error, but better resolution. Not amy issue with 32-bit phase accumulators.
                                              //-Must be at least twice the highest frequency. Frequency resolution is FS/(2^32). (2^32) = the size of the phase accumulator.

#define SET(x,y) (x |=(1<<y))		        	  	//-Bit set/clear macros
#define CLR(x,y) (x &= (~(1<<y)))       			// |
#define CHK(x,y) (x & (1<<y))           			// |
#define TOG(x,y) (x^=(1<<y))            			//-+

volatile unsigned long PCW[4] = {
  0, 0, 0, 0};			            //-Wave phase accumulators - Index into wave table (upper 8 bits of 32)
volatile unsigned long FTW[4] = {
  0, 0, 0, 0};                  //-Wave frequency tuning words - Actual frequency being produced
volatile unsigned char AMP[4] = {
  0, 0, 0, 0};                  //-Wave amplitudes [0-255]
volatile unsigned int wavs[4];  //-Wave table selector [address of wave table in memory]
volatile unsigned char output_mode; //45 bytes above

//*********************************************************************************************
//  Audio driver interrupt
//*********************************************************************************************

#if defined( TIM0_COMPB_vect ) && ! defined( TIMER0_COMPB_vect )
#define TIMER0_COMPB_vect TIM0_COMPB_vect//                <<<--------------------  work around for bug in the David Mellis attiny core (Ver 1.0.2) for the ATtiny84 MWG 1/14/2024
#endif

#ifndef TIMEonCTC0 // You have chosen timer 1 for your sampling interupts
//Then timer must be on CTC1 as usual
SIGNAL(TIMER1_COMPA_vect)
#endif

#ifdef TIMEonCTC0 // You have chosen timer 0 for your sampling interupts
//Then timer must be on CTC0
SIGNAL(TIMER0_COMPB_vect)
#endif

{
  //---------------------------------------------------------
  //  Audio mixer - Total must not be > 250 to avoid clipping
  //---------------------------------------------------------
  //pgm_read_byte reads a byte from the wave table in program memory. The function requires the memory address of the byte to be retrieved.
  //Here, the start address of the wave table array, wavs[X], is added to the most-significant 8 bits of the current value of the (32-bit phase accumulator + 32-bit tuning word).
  //This is the address of the retrieved wave table value.
  //The wave table signed 256-bit value is then multiplied by a 256-bit unsigned volume adjustment constant and divided by 256 to render a level-adjusted wave table sample.

//#ifdef TIMEonCTC0  
#if defined(TIMEonCTC0) && defined(PROCESSORmega328)  
                     // if timer 0 then we are not clearing the counter, and we cannot clear the counter without breaking the Ardino time functions 
                     // so we have to move the compare register up with every interrupt to schedule the next interrupt
OCR0B = OCR0B + 7;   // Schedule the next interupt in 7*64 clock ticks
#endif

#ifdef PROCESSORtiny85 // note that TIMEonCTC0 is a MUST for this processor
OCR0B = OCR0B + 10;   // Schedule the next interupt in 10*64 clock ticks (Perhaps a more efficient core would get is get this back to 7, but 7 is it's too fast here. 9 works, but 8 does not.)
//OCR0B = OCR0B + 20;   // Schedule the next interupt in 7*64 clock ticks <-----------Testing ATtiny fesability at internal 8MHz
#endif


// Add Tiny84 support here <--------------------------------------------------------- Place 1
#ifdef PROCESSORtiny84 // note that TIMEonCTC0 is a MUST for this processor
OCR0B = OCR0B + 12;     // Schedule the next interupt in 12*64 clock ticks 
#endif



#ifndef TONEonCTC1  // NOT CTC1, so ... It must be CTC2

//   PORTD = PORTD | B00000100;  //Turn on relay output. This is to check your CPU time spent here with a scope.

  OCR2A = OCR2B = 127 +  //Start at 127 or 2.5 volts as zero level after capacitor coupling the output. Output voltage swings 5 volts p-p.
    (
    (((signed char)pgm_read_byte(wavs[0] + ((PCW[0]+=FTW[0]) >> 24)) * AMP[0]) >> 8) +
    (((signed char)pgm_read_byte(wavs[1] + ((PCW[1]+=FTW[1]) >> 24)) * AMP[1]) >> 8) +
    (((signed char)pgm_read_byte(wavs[2] + ((PCW[2]+=FTW[2]) >> 24)) * AMP[2]) >> 8) +
    (((signed char)pgm_read_byte(wavs[3] + ((PCW[3]+=FTW[3]) >> 24)) * AMP[3]) >> 8)
    );

//   PORTD = PORTD & B11111011;  //Turn off relay output. This is to check your CPU time spent here with a scope.

#endif
#ifdef TONEonCTC1                               // The tone belongs on CTC1!
//digitalWrite(PB0, HIGH); //Turn on cadence LED and/or relay.  Wow this adds 134 bytes to the code. Probably need to go back to poking registers.

  OCR1A = OCR1B = 127 +  //Start at 127 or 2.5 volts as zero level after capacitor coupling the output. Output voltage swings 5 volts p-p.
    (
    (((signed char)pgm_read_byte(wavs[0] + ((PCW[0]+=FTW[0]) >> 24)) * AMP[0]) >> 8) +
    (((signed char)pgm_read_byte(wavs[1] + ((PCW[1]+=FTW[1]) >> 24)) * AMP[1]) >> 8) +
    (((signed char)pgm_read_byte(wavs[2] + ((PCW[2]+=FTW[2]) >> 24)) * AMP[2]) >> 8) +
    (((signed char)pgm_read_byte(wavs[3] + ((PCW[3]+=FTW[3]) >> 24)) * AMP[3]) >> 8)
    );
//digitalWrite(PB0, LOW); //Turn on cadence LED and/or relay. This is to check your CPU time spent here with a scope.
#endif
 
}



class tonegen
{
private:

public:

  tonegen()
  {
  }

  //*********************************************************************
  //  Startup default (  This "default" method of starting tonegen has NOT been implemented, tested, or debuged for the ATtiny84 or ATtiny85 processors. )
  //*********************************************************************

  void begin()
  {
    #ifndef TIMEonCTC0 // NOT CTC0 so it must be a Mega CPU
       output_mode=CHA;
       TCCR1A = 0x00;                                  //-Start audio interrupt
       TCCR1B = 0x09;
       TCCR1C = 0x00;
       OCR1A=16000000.0 / FS;			                    //-Auto sample rate
       SET(TIMSK1, OCIE1A);                            //-Start audio interrupt
       sei();                                          //-+
    #endif

    #if defined(TIMEonCTC0) && defined(PROCESSORmega328)  // It's a Mega and we are sharing CTC0 for this interrupt
       //Timer 0 is used for PWM audio clock
       output_mode=CHA;
       OCR0B=7;                                         //-Auto sample rate 7 times the 64 scaling 448 as opposed to 400
       SET(TIMSK0, OCIE0B);                            //-Start audio interrupt    
       CLR (TCCR0A, WGM00); 
       CLR (TCCR0A, WGM01); 
       CLR (TCCR0B, WGM02); 
       sei();                                          //-+
    #endif

   #if defined(TIMEonCTC0)  && defined (PROCESSORtiny85)                        // if it is an tiny85, then ...
      //output_mode=CHA;
      //OCR0B=7;                                         //-Auto sample rate 7 times the 64 scaling 448 as opposed to 400
      //SET(TIMSK, OCIE0B);                            //-Start audio interrupt
      //CLR (TCCR0A, WGM00); 
      //CLR (TCCR0A, WGM01); 
      //CLR (TCCR0B, WGM02); 
      //sei();                                          //-+
    #endif


// Add Tiny84 support here <--------------------------------------------------------- Place 2      This needs work. We're not using this section
    #if defined(TIMEonCTC0)  && defined (PROCESSORtiny84)
       //output_mode=CHA;
       //OCR0B=7;                                         //-Auto sample rate 7 times the 64 scaling 448 as opposed to 400
       //SET(TIMSK0, OCIE0B);                            //-Start audio interrupt    
       //CLR (TCCR0A, WGM00); 
       //CLR (TCCR0A, WGM01); 
       //CLR (TCCR0B, WGM02); 
       //sei();                                          //-+
    #endif

// 

    #if defined(TONEonCTC2) && defined(PROCESSORmega328)           //It must be CTC2  on CHA pin (11) of ATmega328
      TCCR2A = 0x83;                                  //-8 bit audio PWM
      TCCR2B = 0x01;                                  // |
      OCR2A = OCR2B = 127;                            //-+
      SET(DDRB, 3);                                   //-PWM pin
     #endif

     #if defined(TONEonCTC1) && defined(PROCESSORmega328)
       TCCR1A = 0x83;                                  //-8 bit audio PWM
       TCCR1B = 0x01;                                  // |
       OCR1A = OCR1B = 127;                            //-+
       //SET(DDRB, 10);                                   //-PWM pin
       setOCR1Boutput();
       //setOCR1Aoutput(); // This could conflict with the default serial interface Receive
     #endif
     
      #if defined(TONEonCTC1) && defined(PROCESSORtiny85)          // on CHB pin ( 9 ) of ATmega328, DIP pin 7 of Tiny84, or DIP pin 6 of Tiny85
       //TCCR1A = 0x83;                                  //-8 bit audio PWM
       ////TCCR1B = 0x01;                                  // |
       //TCCR1 = 1<<PWM1A | 1<<COM1A1 | 1<<COM1A0| 1<<CS10; // 0b01010001 0h51 FastPWM-A, CompOutMode-1 and ClkSel 1-to-1
       //OCR1A = OCR1B = 127;                            //-+
       ////SET(DDRB, 3);                                 //-PWM pin
       //setOCR1Aoutput();
      #endif


// Add Tiny84 support here <--------------------------------------------------------- Place 3
  
  }

  //*********************************************************************
  //  Startup, selecting various output modes  ( All 3 modes {DIFF, CHA, and CHB} have been tested with the new TONEonCTC1 for the Mega328. But only CHA has been done for the ATtiny processors.)
  //*********************************************************************

  void begin(unsigned char d)
  {
  //******************* First - Set up our lower timer for the wave form interupt clock CTC0 or CTC1 considering CPU type
  // 
    #ifndef TIMEonCTC0 // NOT CTC0  so MUST be CTC1. It must be a Mega CPU
       //Timer 1 is used for PWM audio clock
       TCCR1A = 0x00;                                  //COM1cp bits (Compare output pins) 0, (Wave Gen Mode) WGM11-12 mode bits 0
       TCCR1B = 0x09;                                  // WGM13 0, WGM12 1, with WGM above 0, that sets mode "Clear TCNT1 on OCR1A match". CS12-CS10=1 Clock one-to-one
       TCCR1C = 0x00;                                  // No forcing of output compare
       OCR1A=16000000.0 / FS;			                    //-Auto sample rate 16MHz and 40kHz is 400 
       SET(TIMSK1, OCIE1A);                            //-Start audio interrupt
       sei();                                          //-+
    #endif

    #if defined(TIMEonCTC0) && defined(PROCESSORmega328)  // It's a Mega and we are sharing CTC0 for this interrupt
      //Timer 0 is used for PWM audio clock
       OCR0B=7;                                         //-Auto sample rate 7 times the 64 scaling 448 as opposed to 400
       SET(TIMSK0, OCIE0B);                            //-Start audio interrupt    
       CLR (TCCR0A, WGM00); 
       CLR (TCCR0A, WGM01); 
       CLR (TCCR0B, WGM02); 
       sei();                                          //-+
    #endif

    #if defined(TIMEonCTC0)  && defined (PROCESSORtiny85)                        // if it is an tiny85, then ...
       OCR0B=10;                                         //-Auto sample rate 10 times the 64 scaling (Perhaps a more efficient core would get is get this back to 7, but it's too fast here)
                                                        // 7 was just the starting point. It's the interupt handler that really steps it
       SET(TIMSK, OCIE0B);                            //-Start audio interrupt
       CLR (TCCR0A, WGM00); 
       CLR (TCCR0A, WGM01); 
       CLR (TCCR0B, WGM02);
       sei();                                          //-+
    #endif


// Add Tiny84 support here <--------------------------------------------------------- Place 4                  This works!
#if defined(TIMEonCTC0)  && defined (PROCESSORtiny84)                        // if it is an tiny84, then ...
       OCR0B=10;                                         //-Auto sample rate 10 times the 64 scaling 920 as opposed to 400 (We moved it to 12. Oh well)
       //SET(TIMSK0, OCIE0B);                            //-Start audio interrupt    
       CLR (TCCR0A, WGM00); 
       CLR (TCCR0A, WGM01); //                                                                      <-----------------------------------------------------TESTING in this area
       CLR (TCCR0B, WGM02); 
       //TCCR0B = 1;
       //SET (TCCR0A, WGM00);                                                                        // Finally got my interupts working with a change in the handler
       //SET (TCCR0A, WGM01);
       //SET (TCCR0B, WGM02);
       
       SET(TIMSK0, OCIE0B);                            //-Start audio interrupt   
       //SET(TIMSK0, SC02);                            //-Start audio interrupt  
        //TIMSK0 = TIMSK0 | (1<<2);             //Enable it by hand
       sei();                                          //-+
#endif





    output_mode=d;

//******************* Then - Set up our upper timer CTC1 or CTC2 for our PWM timer(s) considering A side, B side, or both, and CPU type
//                                                                              Note: only 1A supported for ATtiny85 so far
    switch(d)
    {
    case DIFF:                                        //-Differntial signal
      #ifndef TONEonCTC1    // It must be CTC2 on CHA and CHB pins (11,3) of ATmega328
       TCCR2A = 0xB3;                                  //-8 bit audio PWM sets 
       TCCR2B = 0x01;                                  // |
       OCR2A = OCR2B = 127;                            //-+
       //setOCR2Aoutput()
       //setOCR2Boutput()
       SET(DDRB, 3);                                   //-PWM pin
       SET(DDRD, 3);                                   //-PWM pin
      #endif
      
      #if defined(TONEonCTC1) && defined(PROCESSORmega328)  // on CHA and CHB pins (9,10) of ATmega328, DIP pins 7,8 of Tiny84, or DIP pins 6,3 of Tiny85 (but we'll probably move 3 to 5 with interrupts) 
       //TCCR1A = 0xB3;                                  //-8 bit audio PWM sets                  <---- Testing
       TCCR1A = 0xA1;                                  //-8 bit audio PWM sets                  <---- That's better
       //TCCR1B = 0x01;                                  // | Sets only clock one to one           <---- Testing
       TCCR1B = 0x09;                                  // | Sets only clock one to one          <---- That's better
       OCR1A = OCR1B = 127;                            //-+
       setOCR1Aoutput();                                //-PWM pin
       setOCR1Boutput();                                //-PWM pin
      #endif

      //This probably does not work yet
      #if defined(TONEonCTC1)  && defined (PROCESSORtiny85)                        // if it is an tiny85,DIP pins 6,3 of Tiny85 (but we'll probably move 3 to 5 with interrupts)
      TCCR1 = 1<<PWM1A | 1<<COM1A1 | 1<<COM1A0| 1<<CS10; // 0b01010001 0h51 FastPWM-A, CompOutMode-1 and ClkSel 1-to-1 // That should be good for the A side
      GTCCR = 0<<TSM | 1<<PWM1B | 0<<COM1B1 | 0<<COM1B0 | 0<<FOC1B | 0<<FOC1A | 0<<PSR1 | 0<<PSR0; //That probably needs work for the B side as differential
      OCR1A = OCR1B = 127;                            //-+
      pinMode(1, OUTPUT); // make the pin on output Dip pin 6
      //pinMode(4, OUTPUT); // Differential not supported on this processor and we need this as an input
      #endif






// Add Tiny84 support here <--------------------------------------------------------- Place 5       This needs work. We're not using this section yet
#if defined(TONEonCTC1)  && defined (PROCESSORtiny84)                        // if it is an tiny84 
//       //TCCR1A = 0xB3;                                  //-8 bit audio PWM sets                  <---- Testing
//       TCCR1A = 0xA1;                                  //-8 bit audio PWM sets                  <---- That's better
//       //TCCR1B = 0x01;                                  // | Sets only clock one to one           <---- Testing
//       TCCR1B = 0x09;                                  // | Sets only clock one to one          <---- That's better
//       OCR1A = OCR1B = 127;                            //-+
//       setOCR1Aoutput();                                //-PWM pin
//       setOCR1Boutput();                                //-PWM pin
      #endif




      
      break;

    case CHB:                                         // -Single ended signal 
      #ifndef TONEonCTC1   //It must be CTC2 on CHB pin (3) of ATmega328
      // It must be on CTC2 and it must be a Mega processor
       TCCR2A = 0x23;                                  //-8 bit audio PWM
       TCCR2B = 0x01;                                  // |
       OCR2A = OCR2B = 127;                            //-+
       SET(DDRD, 3);                                   //-PWM pin
      #endif
   
      #if defined(TONEonCTC1) && defined(PROCESSORmega328)    // on CHB pin (10) of ATmega328, 
       //TCCR1A = 0x23;                                  //-8 bit audio PWM                  <---- Testing
       TCCR1A = 0x21;                                  //-8 bit audio PWM                   <---- That's better
       //TCCR1B = 0x09;                                  // |                                <---- Testing
       TCCR1B = 0x01;                                  // |                                 <---- That's better
       OCR1A = OCR1B = 127;                            //-+
       //SET(DDRD, 3);                                   //-PWM pin
       setOCR1Boutput();                                //-PWM pin
      #endif

      //This option is not yet supported on an ATtiny85 processor
      #if defined(TONEonCTC1) && defined (PROCESSORtiny85)            // on CHB pin (10) of ATmega328, DIP pin 8 of Tiny84, or DIP pin 3 of Tiny85 (but we'll probably move 3 to 5 with interrupts)
       TCCR1A = 0x23;                                  //-8 bit audio PWM <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<This should choke
       //TCCR1B = 0x01;                                  // |<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<This should choke
       OCR1A = OCR1B = 127;                            //-+
       //SET(DDRD, 3);                                   //-PWM pin
       setOCR1Boutput();                                //-PWM pin
      #endif





// Add Tiny84 support here <--------------------------------------------------------- Place 6      DIP pin 8 of Tiny84 This needs work. We're not using this section yet
      #if defined(TONEonCTC1) && defined (PROCESSORtiny84)
//        //TCCR1A = 0x23;                                  //-8 bit audio PWM                  <---- Testing
//        TCCR1A = 0x21;                                  //-8 bit audio PWM                   <---- That's better
//        //TCCR1B = 0x09;                                  // |                                <---- Testing
//        TCCR1B = 0x01;                                  // |                                 <---- That's better
//        OCR1A = OCR1B = 127;                            //-+
//        //SET(DDRD, 3);                                   //-PWM pin
//        setOCR1Boutput();                                //-PWM pin
      #endif

      break;



    // This is where we have been testing for the ATtiny processors

      
    case CHA:                                          //-Single ended signal
    default:
      output_mode=CHA;                                //-Single ended signal
      
      #ifdef TONEonCTC2                              //It must be CTC2  on CHA pin (11) of ATmega328
       TCCR2A = 0x83;                                  //-8 bit audio PWM
       TCCR2B = 0x01;                                  // |
       OCR2A = OCR2B = 127;                            //-+
       SET(DDRB, 3);                                   //-PWM pin
      #endif
     
      #if defined(TONEonCTC1) && defined (PROCESSORmega328)                                // on CHB pin ( 9 ) of ATmega328, DIP pin 7 of Tiny84, or DIP pin 6 of Tiny85 
       //TCCR1A = 0x83;                                  //-8 bit audio PWM                  <---- Testing
       TCCR1A = 0x81;                                  //-8 bit audio PWM                   <---- That's better
       //TCCR1B = 0x01;                                  // |                                <---- Testing
       TCCR1B = 0x09;                                  // |                                 <---- That's better
       OCR1A = OCR1B = 127;                            //-+
       //SET(DDRB, 3);                                 //-PWM pin
       setOCR1Aoutput();
      #endif

      #if defined (TONEonCTC1) && defined (PROCESSORtiny85)                              // on CHB pin ( 9 ) of ATmega328, DIP pin 7 of Tiny84, or DIP pin 6 of Tiny85
       //TCCR1A = 0x83;                                  //-8 bit audio PWM <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<This should choke
       //TCCR1A =0x41                                    // My core accepts that
       //TCCR1 = 0x83;                                   // TCCR1 seems to be called TCCR1A in the Core that I'm using for the ATTINY85
       //TCCR1B = 0x01;                                  // |

/*// Set up timer/PWM items
  PLLCSR |= (1 << PLLE); //Enable PLL

  //Wait for PLL to lock
  while ((PLLCSR & (1<<PLOCK)) == 0x00)
    {
        // Do nothing until plock bit is set
    }

  // Enable clock source bit
  PLLCSR |= (1 << PCKE);*/

       // I really dont't know this timer very well and it's quite different from timers on other processors
       // This may not be the best configuration but it seems to work
       //TCCR1 = 1<<PWM1A | 0<<COM1A1 | 1<<COM1A0| 1<<CS10; // 0b01010001 0h51 FastPWM-A, CompOutMode-1 and ClkSel 1-to-1
       //TCCR1 = (0<<CTC1) | (1<<PWM1A) | (0<<COM1A1) | (1<<COM1A0) | (0<<CS13) | (0<<CS12) | (0<<CS11) | (1<<CS10) ; // This stumped me, it gave tone on both PB1 and PB0! I didn't read that it could do that.
       TCCR1 = (0<<CTC1) | (1<<PWM1A) | (1<<COM1A1) | (0<<COM1A0) | (0<<CS13) | (0<<CS12) | (0<<CS11) | (1<<CS10) ; // 0b01010001 0h51 FastPWM-A, CompOutMode-1 and ClkSel 1-to-1
       OCR1A = OCR1B = 127;                            //-+
       OCR1C = 0xFF;
       GTCCR =  (0<<TSM) | (0<<PWM1B) | (0<<COM1B1) | (0<<COM1B0) | (0<<FOC1B) | (0<<FOC1A) | (0<<PSR1) | (0<<PSR0); //<------ TESTING
       //GTCCR = GTCCR | (1<<PSR1); // Kick it!
       //SET(DDRB, 3);                                 //-PWM pin
       setOCR1Aoutput();
      #endif





// Add Tiny84 support here <--------------------------------------------------------- Place 7  This is where all of the work and testing has been done on this processor. It WORKS!!!!!
      #if defined (TONEonCTC1) && defined (PROCESSORtiny84) 
      
       //TCCR1A = 0x83;                                  //-8 bit audio PWM                  
       //TCCR1A = 0x81;                                  //-8 bit audio PWM                   <---- Was on Mega
       TCCR1A = (1<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<3) | (0<<2) | (0<<WGM11) | (1<<WGM10); // Tiny84 is very much like Mega328 TCCR1A but very different from tiny85
       //                                                                                                          // COM bits are largly the same, only a slight difference for COM1x1-0 = 0,1
       //TCCR1B = 0x01;                                  // |                               
       //TCCR1B = 0x09;                                  // |                                 <---- Was on Mega
       TCCR1B = (0<<ICNC1) | (0<<ICES1) | (0<<5) | (0<<WGM13) | (1<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10); // This too seems layed out like the Mega328
       //                                                                                                       // WGM1(3-0) seem the same, CS are the same
       OCR1A = OCR1B = 127;                            //-+
       //SET(DDRB, 3);                                 //-PWM pin
       setOCR1Aoutput();
       
      #endif




 
      break;
    }
  }



  //*********************************************************************
  //  Setup waves
  //*********************************************************************

  void setWave(unsigned char wave1, unsigned char wave2, unsigned char wave3, unsigned char wave4)
  {

    if(wave1 == SQUARE) {wavs[0] = SquareTable;} 
      else if (wave1 == SINE){wavs[0] = SinTable;} 
      else if (wave1 == RAMP){wavs[0] = RampTable;}
      else {wavs[0] = SinTable;}
    if(wave2 == SQUARE) {wavs[1] = SquareTable;} 
      else if (wave2 == SINE){wavs[1] = SinTable;} 
      else if (wave2 == RAMP){wavs[1] = RampTable;}
      else {wavs[1] = SinTable;}
    if(wave3 == SQUARE) {wavs[2] = SquareTable;} 
      else if (wave3 == SINE){wavs[2] = SinTable;} 
      else if (wave3 == RAMP){wavs[2] = RampTable;}
      else {wavs[2] = SinTable;}   
    if(wave4 == SQUARE) {wavs[3] = SquareTable;} 
      else if (wave4 == SINE){wavs[3] = SinTable;} 
      else if (wave4 == RAMP){wavs[3] = RampTable;}
      else {wavs[3] = SinTable;}

  }


  //*********************************************************************
  //  Set frequency direct
  //*********************************************************************

  void setFrequency(unsigned char voice,float f)
  {
    //32-bit phase accumulator provides 0.000009313 Hz. resolution at 40KHz sampling rate (FS/(2^32))
    //FTW[voice]=f/(FS/4294967295.0); //frequency/(sample_rate/max_32bit_count)
    //f * 1/(40,000/4294967295.0) = f * 107,374.182375
    // Moving to timer 0 with about a 20kHz (20,833.333Hz) sampling rate will change this to about 206158
    // 16,000,000 clock / Arduino 64 scaling / 12 tick steps on the compare interupts = 20,833.333Hz steps
    // 37,714.2957Hz with a step of 7 ticks on the compare register  
    // or 19,230.7692Hz with a step of 13 ticks
    #ifndef TIMEonCTC0
      //Time is on CTC1
      FTW[voice]=f * (107374.182375 + tunerValue);
      //FTW[voice]=f * (214748.36475 + tunerValue);
    #endif
    
    //#ifdef TIMEonCTC0
    #if defined(TIMEonCTC0) && defined(PROCESSORmega328)  // It's a Mega and we are sharing CTC0 for this interrupt
      FTW[voice]=f * (120100.8957 + tunerValue); // gives 1004.3 when above timer was 1004.3 with whatever tuner value Step of 7
     #endif

     #ifdef PROCESSORtiny85
      //FTW[voice]=f * (217346.1272 + tunerValue);//<--------TESTING step of 13
      FTW[voice]=f * (167253.3368 + tunerValue);//<-------- step of 10 Attiny85 works will at internal 16MHz
      //FTW[voice]=f * (334506.6736 + tunerValue);//<--------TESTING for 8MHz internal for Attiny84. First tested steps of 20 and then slowed the clock steps of 10 ATtiny84 Just testing here on the 85 for feasibility
     #endif






// Add Tiny84 support here <--------------------------------------------------------- Place 8
     #ifdef PROCESSORtiny84
      //FTW[voice]=f * (217346.1272 + tunerValue);//<--------TESTING step of 13 Attiny84 only if you have an external 16MHz clock
      //FTW[voice]=f * (167253.3368 + tunerValue);//<-------- step of 10 Attiny84 only if you have an external 16MHz clock
      //FTW[voice]=f * (334506.6736 + tunerValue);//<-------- 8MHz internal max for Attiny84, re're stuck with steps of 10 Too Fast
      //FTW[voice]=f * (402209.0000 + tunerValue);//Gave 983 for 1004
      //FTW[voice]=f * (410800.0000 + tunerValue);//Gave 1025 for 1004
      //FTW[voice]=f * (406500.0000 + tunerValue);//Gave 1015
      //FTW[voice]=f * (402100.0000 + tunerValue); //Gave 1005.2 for 1004
      //FTW[voice]=f * (401620.0000 + tunerValue);//Gave 1002.5
      //FTW[voice]=f * (401860.0000 );// gives 1004.5
      //FTW[voice]=f * (401850.0000 );// gives 1004.4
      //FTW[voice]=f * (401700.0000 );// gives a touch high
      FTW[voice]=f * (401675.0000 + tunerValue);// gives right close but lets shoot for 1002.5 without the tooner and rounded
      //FTW[voice]=f * (401675.0000);//That's roght at 1004 but with some buzz
      //FTW[voice]=f * (401625.0000);// that's 1003.7 or 8
      //FTW[voice]=f * (401500.0000);//1003.5
      //FTW[voice]=f * (401400.0000);//1003.2
      //FTW[voice]=f * (401250.0000);//1003.0
      //FTW[voice]=f * (401000.0000);//1002.2
      //FTW[voice]=f * (401125.0000);//
     #endif

  }

  
  //*********************************************************************
  //  Set volume
  //*********************************************************************

  void setVolume(unsigned char vol0, unsigned char vol1, unsigned char vol2,unsigned char vol3)
  {
    AMP[0] = vol0;
    AMP[1] = vol1;
    AMP[2] = vol2;
    AMP[3] = vol3;
  }
  //*********************************************************************
  //  Suspend/resume tonegen
  //*********************************************************************

  void suspend()
  {
   //  CLR(TIMSK1, OCIE1A);                            //-Stop audio interrupt 
   
   #ifndef TONEonCTC1
   //Must be CTC2
    while(OCR2A != 127){                            //-Ramp down voltage to zero for click reduction
      if(OCR2A > 127){OCR2A = OCR2B -= 1;}
      if(OCR2A < 127){OCR2A = OCR2B += 1;}
      delayMicroseconds(8);                         //Sets ramp down speed, 1.008 mS worst case (126*0.008)
    }
    #endif
    
    #ifdef TONEonCTC1
     while(OCR1A != 127){                            //-Ramp down voltage to zero for click reduction
      if(OCR1A > 127){OCR1A = OCR1B -= 1;}
      if(OCR1A < 127){OCR1A = OCR1B += 1;}
      delayMicroseconds(8);                         //Sets ramp down speed, 1.008 mS worst case (126*0.008)
    }
    #endif  
    
    #ifndef TIMEonCTC0                                //The timier must be on Timer 1
      //Must be on timer 1
      CLR(TIMSK1, OCIE1A);                            //-Stop audio interrupt 
    #endif
    
    #if defined(TIMEonCTC0) && defined(PROCESSORmega328)
      CLR(TIMSK0, OCIE0B);                            //-Stop audio interrupt 
    #endif
    
    #if defined(PROCESSORtiny85) && defined(TIMEonCTC0)    // TIMEonCTC0 && PROCESSORtiny85 if it is an tiny85, then ...
      CLR(TIMSK, OCIE0B);                            //-Stop audio interrupt 
    #endif





// Add Tiny84 support here <--------------------------------------------------------- Place 9
    #if defined(PROCESSORtiny84) && defined(TIMEonCTC0)    // TIMEonCTC0 && PROCESSORtiny84 if it is an tiny84, then ...
      CLR(TIMSK0, OCIE0B);                            //-Stop audio interrupt 
    #endif





    //Resetting phase accumulators to zero assures voltage always starts at zero for consistent "on" click.
    PCW[0] = PCW[1] = PCW[2] = PCW[3] = 0;          //-Reset phase accumulators to avoid weird on/off click phasing patterns
  }
  
  void resume()
  {
    #ifndef TIMEonCTC0
      SET(TIMSK1, OCIE1A);                            //-Start audio interrupt
    #endif
    
    #if defined(TIMEonCTC0) && defined(PROCESSORmega328)  // if it's NOT a tiny85, then ...
      SET(TIMSK0, OCIE0B);                            //-Start audio interrupt
    #endif
    
    #if defined(TIMEonCTC0)  && defined (PROCESSORtiny85)                        // if it is an tiny85, then ...
      SET(TIMSK, OCIE0B);                            //-Start audio interrupt
    #endif
    


// Add Tiny84 support here <--------------------------------------------------------- Place 10
    #if defined(TIMEonCTC0)  && defined (PROCESSORtiny84)                        // if it is an tiny84, then ...
      SET(TIMSK0, OCIE0B);                            //-Start audio interrupt
    #endif


 
/*    CLR(TIMSK0, TOIE0);                            // - for testing, disable the overflow which breaks Arduino time functions (I'm just rracking jitter)*/
    
  }

};

#endif
