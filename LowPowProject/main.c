#include <msp430fr5969.h>

float sampling(unsigned int channel);

   unsigned int cycle;
   float VBAT;
   float VOC;
   int tmp;

void main(void)
{

   WDTCTL = WDTPW + WDTHOLD;

   //Ports configuration
   P1DIR = 0xFF; //P1.4 as Digt Output (P1.4=SHDN)
   P1DIR &= ~BIT6; //P1.6 is input (P1.6 receive Cout the output of the comparator)
   P1OUT = 0x00;
   P2DIR = 0xFF;
   P2OUT = 0x00;
   P3DIR = 0xFF; //P3.5 as output for the enable of the comparator
   P3OUT = 0x00;
   P4DIR = 0xFF;
   P4OUT = 0x00;
   PJDIR = 0xFFFF;
   PJOUT = 0x00;

 /*Not lowpower friendly
   P1DIR |= BIT4; //Set P1.4 as output & P1.6 as input
   P1DIR |= BIT5;
   P1OUT |= BIT5;*/

   //P1.5 comp on as comparator for pwm
   P1SEL1 |= 0x00; //P1.5 CCR2 OUT
   P1SEL0 |= BIT5; //P1.5 CCR2 OUT

   //Set the analog port
   P4SEL1 |= BIT2; //P4.2 as A10
   P4SEL0 |= BIT2; //P4.2 as A10
   P2SEL1 |= BIT4; //P2.4 as A7
   P2SEL0 |= BIT4; //P2.4 as A7

   PJSEL1 |= 0x00;
   PJSEL0 |= BIT4+BIT5; //Configure PJ.4 and PJ.5 for LFXT
   PM5CTL0 &= ~LOCKLPM5;

   //Variables configuration
   cycle = 1;
   tmp = 0;
   VBAT = 0;
   VOC = 0;

   //Test SHDN
   P1OUT &= ~BIT4; //SHDN = 0
   P1OUT |= BIT4; //SHDN = 1


     // Setup Clocks
     CSCTL0 = CSKEY; // Unlock CS registers
     CSCTL1 = DCOFSEL_6; // Set DCO to 8MHz
     CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK; // set ACLK = XT1; MCLK = DCO
     CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1; // Set all dividers to 1
     CSCTL4 &= ~LFXTOFF;    // Enable LFXT1
     CSCTL6 &= ~SMCLKREQEN; //SMCLKREQEN=0 to disable SMCLK in LPM3
     do
     {
       CSCTL5 &= ~LFXTOFFG;    // Clear XT1 fault flag
       SFRIFG1 &= ~OFIFG;
     }while (SFRIFG1&OFIFG);  // Test oscillator fault flag
     CSCTL0_H = 0;   // Lock CS registers

   //Timer configuration
   TA0CCR0=64000;  //Timer A0 threshold limit=64000
   TA0CCR1 = 16;
   TA0CTL= MC_1|ID_0|TASSEL_1|TACLR; //ACLK is assigned to Timer A0 which works in Up Mode

   TB0CCR0 = 25;
   TB0CCR2 = 10; //Duty cycle 10/25
   TB0CCTL2 = OUTMOD_6; //PWM (toggle/set)
   TB0CTL = TBSSEL__SMCLK | MC__UP;   // SMCLK, UP mode

   //ADC CONFIGURATION
   ADC12CTL0 = ADC12SHT0_6+ADC12ON + ADC12MSC; //128 ADCLK
   ADC12CTL1 = ADC12SHP+ADC12SSEL_2+ADC12CONSEQ_1+ADC12PDIV_1; //SMCLK with ADC12SSEL_2 and SMCLK/4 with ADC12PDIV_1 , single conversion
   ADC12CTL2 =ADC12RES_2; //12 bits

   ADC12CTL3 = ADC12CSTARTADD_10; //Select the channel
   ADC12MCTL10 = ADC12VRSEL_0+ADC12INCH_10; //VREF V+=AVCC V-=AVSS with channel 10
   ADC12MCTL11 = ADC12VRSEL_0+ADC12INCH_7+ADC12EOS; //VREF V+=AVCC V-=AVSS with channel 7

   //Masks configuration
   P1IFG &= ~BIT6; //P1.6 IFG flag (cleared)
   P1IE  |= BIT6; //Enable interrupt for P1.6
   TA0CTL |= TAIE; //enable interrupt timer A
   //TA0CCTL0 |= CCIE;  // TACCR0 interrupt enabled
   TA0CCTL1 |= CCIE;  // TACCR1 interrupt enabled

   //Disable the comparator
   P3OUT &= ~BIT5;


   __bis_SR_register(GIE); //Enable interrupt
   __bis_SR_register(LPM3_bits); //Enter in LPM3 = only ACLK available (enough because timerA uses ACLK and interrupts are enabled in this mode)

   while(1){};
}


// Timer0_A0 interrupt service routine
#pragma vector = TIMER0_A1_VECTOR //comparator A1
__interrupt void Timer0_A1_ISR (void)
{
    switch(__even_in_range(TA0IV, TA0IV_TAIFG))
      {
        case TA0IV_NONE:   break;   // No interrupt
        case TA0IV_TACCR1: // if interrupts comes from CCR1
            //TA0CTL &= ~TAIFG;
            TA0CCTL1 &= ~CCIFG;  // No interrupt pending (clean the flag of interrupt)

            //Enable comparator
            P3OUT |= BIT5;

            //P1.5 CCR2
            //P1.5 comp on as comparator for pwm
            P1SEL1 |= 0x00; //P1.5 CCR2 OUT
            P1SEL0 |= BIT5; //P1.5 CCR2 OUT


            //Clear the Low power mode which was activated
            __low_power_mode_off_on_exit(); // Restore Active Mode on return
            //Clear the LPM3 mode
            //__bic_SR_register_on_exit(LPM3_bits);
            //Enter in LPM0
            __bis_SR_register_on_exit(LPM0_bits); //LPM0 because we need the pwm which work with SMCLK
            break;

        case TA0IV_TAIFG:// overflow
            TA0CTL &= ~TAIFG;
            P1IFG &= ~BIT6;
            TA0CCR0=64000;
            if ((P1OUT&0x10) == 0x10){ //If charging the capa = P1.4 = 1
                VBAT = sampling(11);
                if (VBAT >= 4.2) { //Voltage battery>4.2
                    P1IE  &= ~BIT6; //Disable interrupt for P1.6
                }else{
                  VOC = sampling(10);
                  TB0CCR2 = sampling(1); //Calculate the new value for the duty cycle
                  P1OUT &= ~BIT4; //SHDN = 0 -> discharge capa and charge the battery
                  P1IES |= BIT6; //Falling edge
                  P1IE  |= BIT6; //Enable interrupt for P1.6
                  TA0CCTL1 = CCIE;  // TACCR1 interrupt enabled

                  //Clear the Low power mode which was activated
                  __low_power_mode_off_on_exit(); // Restore Active Mode on return
                  //Clear the LPM3 mode
                  //__bic_SR_register_on_exit(LPM3_bits);
                  //Enter in LPM0
                  __bis_SR_register_on_exit(LPM0_bits);
                }

            }else{
                P1OUT |= BIT4;
                P1IES &= ~BIT6;
                //TA0R = 0;
            }

            break;

        default: break;
      }
}

// Interrupt P1.6
#pragma vector=PORT1_VECTOR
__interrupt  void S2(void)
{
    TA0CTL &= ~TAIFG;
    P1IFG &= ~BIT6; //P1.6 flag is cleared

    if ((P1OUT&0x10) == 0x10){ //If charging = P1.4 = 1

        cycle = cycle + 1;
        P1IES |= BIT6; //Falling edge (high to low transition)

        if (cycle<=200){
            P1OUT &= ~BIT4; //SHDN = 0
            }else{
                cycle = 0;
                P1IE &= ~BIT6; //Disable interrupt to reach Voc
                TA0CCR0 = 6*TA0R;
                TA0CCTL1 &= ~CCIE;  // TACCR1 interrupt disabled

                //P1.5=1 as DIGT OUTPUT we change P1SEL
                P1SEL1 |= 0x00; //P1.5 is not CCR2 OUT
                P1SEL0 &= ~BIT5; //P1.5 is not CCR2 OUT
                P1OUT |= BIT5; //P1.5 = 1 to not cross twice Vin

                //Clear the Low power mode which was activated
                __low_power_mode_off_on_exit(); // Restore Active Mode on return
                //Clear the LPM3 mode
                //__bic_SR_register_on_exit(LPM3_bits);
                //Enter in LPM3
                __bis_SR_register_on_exit(LPM3_bits); //Only need ACLK for timerA
            }
    }else{ //if discharging
        P1OUT |= BIT4; //SHDN = VCC
        P1IES &= ~BIT6; //Rising edge (low to high transition)
        tmp = TA0R; //save the value of TA0R
        //TA0R = 0;


        if((TA0CCR1-tmp)>5){
            TA0CCR1 = (TA0R>>3)*7; // =TAOR*7/8
                    }else{
                        TA0CCR1=16;
                    }

        TA0R = 0;

        //Disable the comparator
        P3OUT &= ~BIT5;

        //P1.5=1 DIGT OUTPUT
        P1SEL1 |= 0x00; //P1.5 is not CCR2 OUT
        P1SEL0 &= ~BIT5; //P1.5 is not CCR2 OUT
        P1OUT |= BIT5; //P1.5 = 1

        //Clear the Low power mode which was activated
        __low_power_mode_off_on_exit(); // Restore Active Mode on return
        //Clear the LPM3 mode
        //__bic_SR_register_on_exit(LPM3_bits);
        //Enter in LPM3
        __bis_SR_register_on_exit(LPM3_bits);
    }


}

float sampling(unsigned int channel)
{
    float x;
    ADC12CTL0 |= ADC12SC+ADC12ENC;
    while((ADC12CTL0&ADC12SC)==ADC12SC){};
    ADC12CTL0 &= ~ADC12ENC;
    if (channel==11){
       x=((ADC12MEM11 *80)>>17);//0.7*3.6/4096
       return(x);
    }
    if (channel==10){
        x=((ADC12MEM10 * 15)>>14);//3.6/4096
        return(x);
    }
    if (channel==1){
        x=((ADC12MEM10 * 1152)>>16)-(95/4); //x*0.01758-23.75
        //x=(x>>16)-(95/4);
        return(x);
        }
if(x<100)
    x=100;
return(x);
}



