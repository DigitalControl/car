/* This program is for controlling the 2015 robot for CPTR 215,
 * but it is in C, not Assembly.
 *
 * For testing, you need to set the bluetooth
 * terminal to send /n, or /n at the end of a send.  The app
 * I used on my Android phone is from:
 * https://github.com/Sash0k/bluetooth-spp-terminal
 *
 * It will use the Blueberry android program for control.  The
 * command strings  are given below:
 *
 * s,10,10		Speed of left motor, right motor.  The numbers
 * 				are between -100 and 100.
 *
 * The HC-06 bluetoooth module I used to test this is connected
 * to the +5V and GND on the launchpad.  The RXD of the HC-06 is
 * connected to P3.3 and TXD of the HC-06 is connected to P3.4.
 *
 * Note the changes to the interrupt as compared to the G2 Launchpad.
 */

#include <msp430.h>

#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <math.h>
#include <stdio.h>

/* Global Variables */
#define KP 50 // Proportional constant for feedback and control.
#define KI 5.0 // Sum constant for feedback and control.
#define KD 10.0 // Difference constant for feedback and control.
#define BETA 1.0  // Feedback gain. Might be dangerous making it > 1.

#define VERBOSE 1
#define RedLed BIT0
#define GreenLed BIT7
#define samplePeriodT 100 // In TimerA0 counts (1/32,768)s.
float position_left = 0.0;
float position_right = 0.0;
int lspeed, rspeed;

// At the moment, just hard-code in a command
char input[100] = "s,-1,-1";
int end_of_cmd = 1;

/*
char input[100];  // From the bluetooth UART
unsigned int RXByteCtr = 0;
int end_of_cmd = 0; // Flag that we are receiving characters.
void transmit(const char *str); // Routine to send characters to the UART.
int missed_k = 2;  // Start out disconnected.  This tells how many seconds we have missed connection to the UART.
*/

// Truth tables from:
//   https://www.bananarobotics.com/shop/How-to-use-the-L298N-Dual-H-Bridge-Motor-Driver
void leftForward() {
    P8OUT |= BIT1;
    P8OUT &= ~BIT2;
}
void leftBackward() {
    P8OUT &= ~BIT1;
    P8OUT |= BIT2;
}
void rightForward() {
    P2OUT |= BIT3;
    P3OUT &= ~BIT7;
}
void rightBackward() {
    P2OUT &= ~BIT3;
    P3OUT |= BIT7;
}
void leftBrake() {
    P8OUT &= ~BIT1;
    P8OUT &= ~BIT2;
}
void rightBrake() {
    P2OUT &= ~BIT3;
    P3OUT &= ~BIT7;
}

/*
 * main.c
 */
void main(void) {
	char *command; // command is a pointer to a character telling the command (s for speed).
	const char comma[2] = ",";
	const char enter[3] = { '\r', '\n',0 };  // For debugging!
	char *lspeedc, *rspeedc;
	const char information[100] = "Rob's Robot 2015\r\n";
	const char unrecognized_string[100] =
			"Unrecognized string sent.  Try again.\r\n";
	const char ok[100] = "Ok!\r\n";
	/* Set up Registers:
	* First for the Watchdog timer and the indicator LEDs...
	*/
    WDTCTL = WDTPW+WDTHOLD;                   // Stop WDT
    /*
     * This section sets the DCO to 12 MHz, so we can use 230400 baud.
     */
    UCSCTL3 |= SELREF_2;                      // Set DCO FLL reference = REFO
    UCSCTL4 |= SELA_2;                        // Set ACLK = REFO
    __bis_SR_register(SCG0);                  // Disable the FLL control loop
    UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
    UCSCTL1 = DCORSEL_5;                      // Select DCO range 24MHz operation
    UCSCTL2 = FLLD_1 + 374;                	  // Set DCO Multiplier for 12MHz
                                              // (N + 1) * FLLRef = Fdco
                                              // (374 + 1) * 32768 = 12MHz
                                              // Set FLL Div = fDCOCLK/2
    __bic_SR_register(SCG0);                  // Enable the FLL control loop

    // Worst-case settling time for the DCO when the DCO range bits have been
    // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
    // UG for optimization.
    // 32 x 32 x 12 MHz / 32,768 Hz = 375000 = MCLK cycles for DCO to settle
    __delay_cycles(375000);//
    // Loop until XT1,XT2 & DCO fault flag is cleared
    do
    {
      UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
                                              // Clear XT2,XT1,DCO fault flags
      SFRIFG1 &= ~OFIFG;                      // Clear fault flags
    }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag

    // Regular setup below:
	P1DIR |= RedLed; //Make P1.0 an output so we can use the red LED
	P4DIR |= GreenLed; //Make P4.7 an output so we can use the red LED
	P1OUT &= ~RedLed;  //Turn off the red LED
	P4OUT &= ~GreenLed;  //Turn off the green LED
	//P3DIR |= BIT7;	// Left motor direction.
    /*
	P8DIR |= BIT2 + BIT1;  // Left and right motor direction respectively.
	//P3OUT |= BIT7; // Burned out P3.7
	P8OUT |= BIT2; // Right wheel
	P8OUT |= BIT1; // Left wheel
    */

    // Set motor direction outputs to be output pins
    P8DIR |= BIT2 + BIT1;
    P2DIR |= BIT3;
    P3DIR |= BIT7;

    // We need another high pin. So, let's use P6.5, right next to 3.3V
    P6DIR |= BIT5;
    P6OUT |= BIT5;

    // Set them to go forward
    leftForward();
    rightForward();

	/* Initialize the UART */
	P3SEL |= BIT3 + BIT4;                     // P3.4,5 = USCI_A0 TXD/RXD
	UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
	UCA0CTL1 |= UCSSEL_2;                     // SMCLK
	UCA0BR0 = 52;                              // 12MHz 230400 (see User's Guide)
	UCA0BR1 = 0;                              // 12MHz 230400  (page 955)
	UCA0MCTL = UCBRS_0 + UCBRF_13;   // Modln UCBRSx=0, UCBRFx=0, oversample
	UCA0CTL1 &= ~UCSWRST;                     // Initialize USCI state machine
	UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt

	/* Initialize the 1 second still_connected timer here. Use TimerA1.  This
	 * is so the robot can tell it has lost connection with the phone.  If it does,
	 * then it stops the robot.
	 */
	TA1CTL = 0; 			//Turn off timer TA1, clear int flag, divide by 4
	TA1CTL |= TACLR;		// Clear the timer TAR.
	TA1CTL |= TASSEL__ACLK;	// Use ACLK (32 KHz).  This means 2 seconds a period.
	TA1CTL |= MC0;			// Set timer mode to count to TA1CCR0.
	TA1CTL |= TAIE; 		// Enable timer TA1 interrupts.
	TA1CCR0 = 0x7fff;		// Make it count up to 1/2 maximum, meaning 1 second per interrupt.

	/* Put timer stuff here for the PWM of each motor.
	 * We use Timer2 with left motor PWM from P2.4 and controlled by
	 * TA2CCR1, and the right PWM from P2.5 and controlled by TA2CCR2.
	 * It's all automatic, no need to make an ISR. 						*/
	P2SEL |= BIT4 + BIT5;             // P2.4 and P2.5 options select
	P2DIR |= BIT4 + BIT5;             // P2.4 and P2.5 output
	TA2CCR0 = 100;                    // PWM Period/2
	TA2CCTL1 = OUTMOD_6;              // CCR1 toggle/set
	TA2CCR1 = 0;                      // CCR1 PWM duty cycle (stopped initially)
	TA2CCTL2 = OUTMOD_6;              // CCR2 toggle/set
	TA2CCR2 = 0;                      // CCR2 PWM duty cycle (stopped initially)
	TA2CTL = TASSEL_2 + MC_3 +ID_3 + TACLR; // SMCLK, up-down mode, clear TAR /8 to compensate 12 MHz.

	/* Put the timer stuff here to read the time per period on the encoders.
	 * Use TA0 because it has two Capture inputs at P1.2 (TA0.1) and P1.3 (TA0.2).
	 * We will use compare mode of this timer to measure the time for one
	 * window and bar on the wheel to go by the shaft angle encoder.
	 *
	 * P8.2 (right) and P8.1 (left) determine direction of the H bridge.
	 *
	 * We could send the velocity and position data to the phone for further
	 * analysis if we want.
	 *
	 * We will impliment feedback on the velocity of each wheel so following the command
	 * from the phone will be more precise, rather than open loop.  Hopefully
	 * this will take care of the problem where differently charged batteries
	 * make the robot behave differently.
	 *
	 * Calculations show that a pretty fast speed corresponds to a period of 0.01s, (see wiki)
	 * or 100 Hz.  We will use the ACLK with 32.768 KHz.  We won't divide it down.
	 * We will use continuous mode.  We will use another timer interrupt to store the data,
	 * and do the feedback.  This will give us time sampled data.  This timer needs to be
	 * quicker than the capture timer, because for slow speeds, the capture may need a lot
	 * of counts.  We will use the same TA0 for this by adding counts to TA0CCR3 to make
	 * a sample clock.  At every tick of this clock, we will do feedback and control in the
	 * TIMER0_A1 interrupt subroutine.
	 *
	 * To setup the TA0CCTLx, we want compare, synchronous, continuous, enable TAIE.
	 *
	 * See pages 476 of slau208o.pdf.
	 */
	P1DIR &= ~(BIT2+BIT3); //P1.2 and P1.3 are input.
	P1SEL |= (BIT2+BIT3); // Connect P1.2 to TA0.1 and P1.3 to TA0.2.
	TA0CCTL1 = CM_1 + SCS + CAP + CCIE;
	TA0CCTL2 = CM_1 + SCS + CAP + CCIE;
	TA0CCR3 = samplePeriodT; // Sampling clock frequency is 32768/(samplePeriodT)
	TA0CCTL3 = SCCI + CCIE;  // Set up the sampling clock.
	TA0CTL = TASSEL_1 + MC_2 + TACLR + TAIE;  // ACLK, contmode, clear TAR, enable interrupt

	_enable_interrupts();
	printf("This is a debug message!\n");
	while (1) { /* We may need to replace this forever loop with sleeps and use interrupts. */
		if (end_of_cmd == 1) {//Check if end_of_cmd is 1, if so, we have a whole line.
			// Parse that line.  The first token should be the command.
			if(VERBOSE) printf("R: %s\n",input); // Show what we are receiving on the console for debugging.
			command = strtok(input, comma);
			if(VERBOSE){
				/*
				transmit(command);// For debugging
				transmit(enter);// For debugging
				*/
			}
			switch (*command) {
			case 'r': //Reset was sent.
				WDTCTL = WDT_MRST_0_064;
				break;
			case 'k': // Blueberry poles the device to make sure it is in range with 'k'.
                /*
				missed_k = 0; // Reset number missed to zero.
				if(VERBOSE) transmit(ok);
                */
				break;
			case 's':
				lspeedc = strtok(NULL, comma);
				lspeed = atoi(lspeedc); // lspeed ranges from -100 to 100.
				// Do we need to check for errors.  Probably not.  atoi returns 0
				// if there is no number.
				rspeedc = strtok(NULL,NULL);
				rspeed = atoi(rspeedc);
				if(VERBOSE) {
					printf("Speed Changed: lspeed = %d .  rspeed = %d \n",lspeed, rspeed);
				}
				if((abs(rspeed)>100)||(abs(lspeed)>100)) {
					if(VERBOSE) printf("Unrecognized speed!\n");
					break; // We don't want to store a bad speed.
				}
				// We need to start without feedback, because the encoders won't interrupt
				// until they start moving.
				if (lspeed < 0) { // pwm P2.4 goes to IA on HG7881
					//P1OUT &= ~RedLed; // Turn off LED for debugging
					//P3OUT &= ~BIT7; // Low means backwards.
					//P8OUT &= ~BIT1; // Low means backwards.
					leftBackward();
					//TA2CCR1= (int)(100-((abs(lspeed)*0.34f+66))+0.5f);
					TA2CCR1=(100-abs(lspeed)); // TA2CCR1 ranges from 0 to 100.
					// You must invert the PWM when the direction changes.
				} else { // Direction goes to IB on HG7881
					//P1OUT |= RedLed; // Turn it on.
					//P3OUT |= BIT7; // High means forwards.
					//P8OUT |= BIT1; // High means forwards.
					leftForward();
					//TA2CCR1= (int)((abs(lspeed)*0.34f+66)+0.5f);
					TA2CCR1=abs(lspeed);
				}
				if (rspeed < 0) { // pwm P2.5 goes to IA on HG7881
					//P4OUT &= ~GreenLed; // Turn off LED for debugging
					//P8OUT &= ~BIT2; // Low means backwards.
					rightBackward();
					//TA2CCR2 = (int)((100-(abs(rspeed)*0.34f+66))+0.5f);
					TA2CCR2=(100-abs(rspeed));
				} else { // Direction goes to IB on HG7881
					//P4OUT |= GreenLed; // Turn it on.
					//P8OUT |= BIT2; // High means forwards.
					rightForward();
					//TA2CCR2 = (int)((abs(rspeed)*0.34f+66)+0.5f);
					TA2CCR2 = abs(rspeed);
				}
				break;
			case 'i':
                /*
				transmit(information);
                */
				break;
			default:
                /*
				transmit(unrecognized_string);
                */
				break;
			}
			end_of_cmd = 0; //Reset end_of_cmd
            /*
			RXByteCtr = 0; //Reset Receive Byte counter
            */
		}
	}
}

//USCI A receiver interrupt
// The stuff immediately below is to make it compatible with GCC, TI or IAR
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    /*
    //Check if the UCA0RXBUF is different from 0x0A which is Enter key from keyboard.
	if (UCA0RXBUF != 0x0A){
		input[RXByteCtr++] = UCA0RXBUF;
		//If it is not ENTER, load received character
		//to current input string element
	}
	else { // It was ENTER (/n)
		end_of_cmd = 1; // end_of_cmd is a flag telling if the whole line has
		// been received yet or not.
		// If it is not, set end_of_cmd
		input[RXByteCtr] = 0;//This wipes out the \r with a 0.  This is funny.  There should be no /r.
	}	//Add null character at the end of input string
    */
}

/*
void transmit(const char *str) { //Consider doing this with an interrupt too.
	while (*str != 0) {	//Do this to the end of the string
		while (!(UCTXIFG & UCA0IFG)); // Wait here until the transmit interrupt flag is set
		UCA0TXBUF = *str++;       //Load UCA0TXBUF with current string element
	}	//then go to the next element
}
*/


// Timer1_A1 Interrupt Vector (TAIV) handler (Used for ensuring connection to the phone)
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A1_VECTOR))) TIMER1_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    /*
	switch(__even_in_range(TA1IV,14))
	{
		case 0: break;// No interrupt
		case 2: break;// CCR1 not used
		case 4: break;// CCR2 not used
		case 6: break;// reserved
		case 8: break;// reserved
		case 10: break;// reserved
		case 12: break;// reserved
		case 14:// overflow
			if(missed_k++ >= 1) { // We need to miss at least two to panic.
			 // Stop!  We lost connection to the mother ship!
				TA2CCR1 = 0; //Stop motors.
				TA2CCR2 = 0;
				lspeed = 0;
				rspeed = 0;
				//if(VERBOSE) printf("Lost connection with phone! \n");
			}
			break;
		default: break;
	}
    */
}

// Timer0_A1 Interrupt Vector (TAIV) handler (Read encoders, and do feedback.)
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) TIMER0_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
	static float speed_left = 0.0;
	static float speed_right = 0.0;
	static unsigned int lastTA0CCR1 = 0;
	static unsigned int lastTA0CCR2 = 0;
	static unsigned int saveTA0IV;
	float sum_lsp_error = 0.0;
	float sum_rsp_error = 0.0;
	float dif_lsp_error = 0.0;
	float dif_rsp_error = 0.0;
	float lsp_error = 0.0, rsp_error = 0.0;
	static int encoder_left_changed = 0;
	static int encoder_right_changed = 0;

	//P4OUT ^= GreenLed; // Turn it on. (for debugging)
	saveTA0IV = TA0IV;  // TAxIV changes can be changed after reading if another interrupt is pending.
	//printf("TA0IV is: %d\n",saveTA0IV);
	switch(__even_in_range(saveTA0IV,14))
	{
		case 0:
			break;// No interrupt
		case 2: // CCR1 (Capture for left wheel.)
			speed_left = 32768.0*BETA/(TA0CCR1-lastTA0CCR1);  // Set it so fast is 100.  100*32768*.01/TACCR1;
			// What happens if TA0CCR1 wraps around?  It seems to work in a test because unsigned int is 16 bits.
			lastTA0CCR1 = TA0CCR1;
			encoder_left_changed = 1;
			TA0CCTL1 &= ~CCIFG; // Clear CCIFG.
			P4OUT ^= GreenLed; // Turn it on. (for debugging)
			break;
		case 4: // CCR2 (Capture for right wheel.)
			speed_right = 32768.0*BETA/(TA0CCR2-lastTA0CCR2);
			lastTA0CCR2 = TA0CCR2;
			encoder_right_changed = 1;
			TA0CCTL2 &= ~CCIFG; // Clear CCIFG.
			P1OUT ^= RedLed; // For debugging.
			break;
		case 6: // CCR3 (Sampling time)
			TA0CCR3 += samplePeriodT; // It will automatically wrap around.
			/* Do control here.
			*/
			dif_lsp_error = lspeed-speed_left-lsp_error;
			dif_rsp_error = rspeed-speed_right-rsp_error;
			lsp_error = lspeed-speed_left;
			rsp_error = rspeed-speed_right;
			sum_lsp_error += lsp_error;
			sum_rsp_error += rsp_error;
			if (lspeed < 0) { // pwm P2.4 goes to IA on HG7881
				//P1OUT &= ~RedLed; // Turn off LED for debugging
                leftBackward();
				TA2CCR1 = (unsigned int)(100-fminf(100.0, abs(KP*lsp_error+KI*sum_lsp_error+KD*dif_lsp_error)));
				// You must invert the PWM when the direction changes.
			} else if (lspeed == 0) { // Reset to stop if speed == 0.
				lsp_error = 0.0;
				sum_lsp_error = 0.0;
				dif_lsp_error = 0.0;
				TA2CCR1 = 0;
			}
			else { // Direction goes to IB on HG7881
				//P1OUT |= RedLed; // Turn it on. (for debugging)
                leftForward();
				TA2CCR1 = (unsigned int)fminf(100.0,abs(KP*lsp_error+KI*sum_lsp_error+KD*dif_lsp_error));
				// fminf used to make sure we don't go over 100.
			}
			if (rspeed < 0) { // pwm P2.5 goes to IA on HG7881
				//P4OUT &= ~GreenLed; // Turn off LED for debugging
                rightBackward();
				TA2CCR2 = (unsigned int)(100-fminf(100.0, abs(KP*rsp_error+KI*sum_rsp_error+KD*dif_rsp_error)));
			} else if (rspeed == 0) {
				rsp_error = 0.0;
				sum_rsp_error = 0.0;
				dif_rsp_error = 0.0;
				TA2CCR2 = 0;
			}
			else { // Direction goes to IB on HG7881
				//P4OUT |= GreenLed; // Turn it on. (for debugging)
                rightForward();
				TA2CCR2 = (unsigned int)fminf(100.0, abs(KP*rsp_error+KI*sum_rsp_error+KD*dif_rsp_error));
			}
			TA0CCTL3 &= ~CCIFG; // Clear CCIFG
			break;// reserved
		case 8:
			break;// reserved
		case 10:
			break;// reserved
		case 12:
			break;// reserved
		case 14:// overflow
			//if (VERBOSE) printf("No capture in 4 seconds.\n");
			//P1OUT ^= RedLed; // For debugging.
			if(!(encoder_left_changed)){
				speed_left = 0.0; // If it goes for 4 seconds without moving one period, it is stopped
				lsp_error = 0.0;
				sum_lsp_error = 0.0;
			}
			else {
				encoder_left_changed = 0;
			}
			if(!(encoder_right_changed)){
				speed_right = 0.0;
				rsp_error = 0.0;
				sum_rsp_error = 0.0;
			}
			else {
				encoder_right_changed = 0;
			}
			break;
		default:
			break;
	}
}
