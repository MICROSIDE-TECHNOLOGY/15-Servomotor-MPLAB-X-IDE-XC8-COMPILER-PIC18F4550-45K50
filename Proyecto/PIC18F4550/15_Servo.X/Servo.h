/************************************************************************************************
Company:
Microside Technology Inc.
File Name:
Servo.c
Product Revision  :  1
Driver Version    :  0.9beta1

Disclaimer:
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
** OF MERCHANTABILITY, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
** TO THE WARRANTIES FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
** IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM,
** DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
** OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
** OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
************************************************************************************************/

/*!
 * @file  Servo.h
 * @brief Small Servo library using Timer1 interruption technique.
          For testing purposes only DO NOT USE on producion.
 */
 
// TODO: This library throws a warning for a possible race condition, fix by 
//       initial release

#ifndef _SERVO_H_
#define _SERVO_H_

// -------------------------------------------------------------- PUBLIC MACROS

// If needed to be used with a different FOSC, define this value to the
// appropiate frequency in Hertz.
#ifdef _XTAL_FREQ
#define SERVO_FOSC _XTAL_FREQ
#endif

#ifndef SERVO_FOSC
#define SERVO_FOSC 48000000
#endif

// Hobby servos work by measuring the duty cycle from 1ms to 2ms, then adjusting
// the position proportionately from 0 to 180, however most of them have different
// offsets, calibrate the following values to match your servo position 

// Seconds that generate a 0� turn
#ifndef SERVO_MIN_POS
#define SERVO_MIN_POS 0.0004
#endif

// Seconds that generate a 180� turn
#ifndef SERVO_MAX_POS
#define SERVO_MAX_POS 0.0027
#endif

// Interval between 0� and 180� turn in seconds
#define SERVO_DUTY_CYCLE_POS (SERVO_MAX_POS - SERVO_MIN_POS)

// Time needed to complement a 50Hz
#define SERVO_FREQ_COMPLEMENT 0.02 - SERVO_MAX_POS

// Timer1 preescaler used, bigger values reduce control accuracy, values too
// small might result in errors.
#ifndef SERVO_T_PREESCALER
#define SERVO_T_PREESCALER 2
#endif

// Values for Timer1 interruption 
#define SERVO_MIN_POS_T1_VALUE (uint16_t)(65535 - ( (SERVO_FOSC * SERVO_MIN_POS)/(4 * SERVO_T_PREESCALER) ))

#define SERVO_FREQ_COMPLEMENT_VALUE (uint16_t)(65535 - ( (SERVO_FOSC * SERVO_FREQ_COMPLEMENT)/(4 * SERVO_T_PREESCALER) ))

#define SERVO_MAX_COMPLEMENT (uint16_t)(65535 - ( ( SERVO_FOSC * ( SERVO_DUTY_CYCLE_POS ) ) / (4 * SERVO_T_PREESCALER) ) )

// ------------------------------------------------------------------ VARIABLES

// Counter used to keep track of the pin level and the preloaded value for Timer1
volatile uint8_t __servo_array_pointer = 0;

// Signal generation array and pin status
// TODO: implement multiple server control, it should be doable to control up to 16 servos
// using only Timer1, rewriting this section might also reduce memory consumtion
uint16_t __servoSignal[4] = { SERVO_MIN_POS_T1_VALUE, 65535, SERVO_MAX_COMPLEMENT, SERVO_FREQ_COMPLEMENT_VALUE };
uint8_t  __onOffMap[4] = {1,1,0,0};

// Output pin
uint8_t  *__servo_port = NULL;
uint8_t  __servo_pin = 0;

// ----------------------------------------------- PUBLIC FUNCTION DECLARATIONS

/**
 * @brief Interrupt handler.
 * 
 * @description This function preload Timer1 next value and changes the output
 *              pin
 */
inline void servo_ISR() {
    __servo_array_pointer++;
    if(__servo_array_pointer > 3) __servo_array_pointer = 0;
    T1CONbits.TMR1ON = 0; //Stops timer1
    
    // Preload timer1 with next interrupt time
    TMR1 = __servoSignal[__servo_array_pointer];
    
    if ( __onOffMap[__servo_array_pointer] == 1 )
        *__servo_port &= !__servo_pin;
    else
        *__servo_port |= __servo_pin;
    // clear flag
    PIR1bits.TMR1IF = 0;
    //Enables Timer1 interrupts
    PIE1bits.TMR1IE = 1; // Enables Timer1 interrupt
    T1CONbits.TMR1ON = 1; //Starts timer1
}

/**
 * @brief Servo signal setup.
 * 
 * @param Output pin
 *
 * @description This function sets Timer1, Global interruptions and the output
 *              pin
 */
void setup_servo( volatile void *port, uint8_t pin ) {
    uint8_t *p_port = (uint8_t *)port;
   __servo_array_pointer = 0;
   
   INTCONbits.GIE = 0; // Disables interrupts
   
   T1CONbits.TMR1ON = 0; //Stops timer1
   T1CONbits.TMR1CS = 0; //Internal clock FOSC/4
   T1CONbits.T1SYNC = 0; // Sync off
   T1CONbits.T1OSCEN = 0; // T1 Oscillator is off
   T1CONbits.T1CKPS = SERVO_T_PREESCALER; // Preescaler value
   T1CONbits.RD16 = 1; // 16 bit operation
   
   PIR1bits.TMR1IF = 0; // Clears Timer1 interrupt flag
   PIE1bits.TMR1IE = 1; // Enables Timer1 interrupt
   INTCONbits.PEIE = 1;
   INTCONbits.GIE = 1; // Enable Global interrupts
   
   TMR1 = __servoSignal[0];
   
   //enable_interrupts( INT_TIMER1 );
   //enable_interrupts( GLOBAL );
   
   __servo_port = p_port;
   __servo_pin = pin;
   
   //output_HIGH (__servo_pin);
   
   //setup_timer_1 ( T1_INTERNAL | T1_DIV_BY_2 );
   T1CONbits.TMR1ON = 1; //Starts timer1
}

/**
 * @brief Sets control signal in seconds.
 * 
 * @param Seconds
 *
 * @description This function calculates Timer1 overflow value for the given
 *              seconds, then loads this value on Timer1 overflow registers
 */
void set_servo_ms( float seconds ) {
   if( seconds > SERVO_DUTY_CYCLE_POS )
      seconds = SERVO_DUTY_CYCLE_POS;
   if( seconds < 0 )
      seconds = 0;
   uint16_t starting_point = (uint16_t)(65535 - ( (SERVO_FOSC * seconds)/(4 * SERVO_T_PREESCALER) ));
   uint16_t complement = (uint16_t)(65535 - ( (SERVO_FOSC * (SERVO_DUTY_CYCLE_POS - seconds))/(4 * SERVO_T_PREESCALER) ));
   __servoSignal[1] = starting_point;
   __servoSignal[2] = complement;
}

/**
 * @brief Sets the servo angle.
 * 
 * @param angle in degrees
 *
 * @description This function calculates the time in seconds that turn the
 *              servo to the input angle
 */
void set_servo_angle ( int angle ) {
   if( angle > 180 )
      angle = 180;
   float __t = (SERVO_DUTY_CYCLE_POS/180.0) * (float)angle;
   set_servo_ms( __t );
}

#endif // _SERVO_H_