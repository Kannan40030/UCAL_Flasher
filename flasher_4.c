//*********************************************************************
// 		Program for Four wheeler Flasher With LED Tail Lamp 24V
//*********************************************************************
// Microcontroller - PIC16F1823
// Internal RC OSC - 8 Mhz
//
//*********************************************************************
// 					Revision History
//*********************************************************************
//	Revision 0.0 
//  Date 02/06/2016
// 	With Short Ckt Protection & Over Voltage Cut OFF.
//*********************************************************************
// 					Pinout Details
//*********************************************************************
// Pin No		Functionality						Usage
//
// 1			VDD									VDD
// 2			RA5									NC
// 3			RA4									I/P (Analog Signal)SHUNT CURRENT
// 4			RA3									I/P (RESET)
// 5			RC5									NC
// 6			RC4									I/P	RH_SW_INPUT
// 7			RC3									I/P LH_SW_INPUT
// 8			RC2									O/P RELAY_LH
// 9			RC1									O/P RELAY_RH
// 10			RC0									I/P (ANALOG) V_SENSE
// 11			RA2									I/P HW_SW_INPUT
// 12			RA1									PROG_C
// 13			RA0									PROG_D
// 14			VSS									VSS
//
//	TRISIO = 0b00011110;
//
//*********************************************************************
//						Includes
//*********************************************************************
#include <pic.h>
#include <math.h>
//*********************************************************************
//						Definitions
//*********************************************************************
// Renaming of PORT Bits

static volatile bit		LED_LH   		@ (unsigned)&PORTA*8+5;
static volatile bit		SHUNT_CUR		@ (unsigned)&PORTA*8+4;		
static volatile bit		L_SHUNT_CUR		@ (unsigned)&PORTA*8+2;		

static volatile bit		LED_RH  		@ (unsigned)&PORTC*8+5;
static volatile bit		LH_SW_IN		@ (unsigned)&PORTC*8+4;		
static volatile bit		RH_SW_IN		@ (unsigned)&PORTC*8+3;		
static volatile bit		RELAY_LH		@ (unsigned)&PORTC*8+2;		
static volatile bit		RELAY_RH		@ (unsigned)&PORTC*8+1;	
static volatile bit		V_SENSE			@ (unsigned)&PORTC*8+0;	

//__CONFIG (FOSC_INTOSC & WDTE_OFF & PWRTE_ON & MCLRE_ON & CP_ON & CLKOUTEN_OFF & BOREN_ON & FCMEN_OFF & STVREN_ON & PLLEN_OFF & BORV_HI & LVP_OFF);

//__CONFIG (FOSC_INTOSC & WDTE_OFF & PWRTE_ON & MCLRE_OFF & CP_ON & CLKOUTEN_OFF & BOREN_ON & FCMEN_OFF & STVREN_ON & PLLEN_OFF & BORV_HI & LVP_OFF);


__CONFIG (FOSC_INTOSC & WDTE_OFF & PWRTE_ON & MCLRE_OFF & CP_ON & CLKOUTEN_OFF & BOREN_ON & FCMEN_OFF & STVREN_ON & PLLEN_OFF & BORV_25 & LVP_OFF);

//*****************************************************************************************************************************
//									Constants
//*****************************************************************************************************************************
//
//*****************************************************************************************************************************
//						Variables
//*****************************************************************************************************************************
unsigned char a;
unsigned char Counter=0;
unsigned char DelayCountHigh;
unsigned char DelayCountLow;

unsigned int Adc_Value;
unsigned int Delay_Count;
unsigned int Working_Voltage_H;
unsigned int Working_Voltage_L;
unsigned int One_Bulb_Fail_Current;
unsigned int LED_Fail_Current;
unsigned int SC_Current;
unsigned int Actual_Current;
unsigned int LED_Load_Current;

union Myint
{
	int Count;
 	char Byte[2];
};
union Myint ADC;

static bit Flasher_On_Flag;
static bit Short_Ckt_Flag;
static bit RH_SW_ON_FLAG;
static bit LH_SW_ON_FLAG;
static bit HW_SW_IN_Flag;
static bit UL_Flag;

//*********************************************************************
//						Function prototypes
//*********************************************************************
void initialise(void);  //cmd
//*********************************************************************
//						Main Program
//*********************************************************************

void main (void) 
	{
            initialise();
            LED_LH = 1;
            LED_RH = 1;
            RELAY_LH = 1;
            RELAY_RH = 1;
            T0IE = 0;
            DelayCountHigh = 60;
            DelayCountLow = 177;
            SC_Current = 850;
            One_Bulb_Fail_Current = 1007;    // 1005	
            LED_Fail_Current = 1022;        // 1018
            Working_Voltage_H = 300;
            Working_Voltage_L = 620;
            RH_SW_ON_FLAG = 0;
            HW_SW_IN_Flag = 0;
            Flasher_On_Flag = 0;
            Short_Ckt_Flag = 0;
            UL_Flag = 0;
/*            
            while(1)
            {
                RELAY_RH = 0;
                LED_RH = 0;
                RELAY_LH = 0;
                LED_LH = 0;  

                ADCON0 = 0b10001001;      // AN2 Channel Selected (Load Current Check for LED).
//               ADCON0 = 0b10001101;      // AN3 Channel Selected (Load Current Check for Lamps).                
                for(a=0;a<=5;a++);
                GO_nDONE = 1;
                while(GO_nDONE == 1)
                {
                    asm ("nop");
                }
                ADC.Byte[1] = ADRESH;
                ADC.Byte[0] = ADRESL;
                Adc_Value = ADC.Count;
                LED_Load_Current = Adc_Value;                
                
                
 //               ADCON0 = 0b10001101;      // AN3 Channel Selected (Load Current Check for Lamps).
 //               for(a=0;a<=5;a++);
 //               GO_nDONE = 1;
 //               while(GO_nDONE == 1)
 //               {
 //                   asm ("nop");
 //               }
 //               ADC.Byte[1] = ADRESH;
 //               ADC.Byte[0] = ADRESL;
 //               Adc_Value = ADC.Count;
 //               Actual_Current = Adc_Value;
            }
*/           
            while(1)
            {
					for(Delay_Count = 0; Delay_Count < 2500; Delay_Count ++);	//Delay to avoid Lamp starting rush current.
                    ADCON0 = 0b10010001;      // AN4 Channel Selected (Supply Voltage Check).
                    for(a=0;a<=5;a++);
                    GO_nDONE = 1;
                    while(GO_nDONE == 1)
                    {
                         asm ("nop");
                    }
                    ADC.Byte[1] = ADRESH;
                    ADC.Byte[0] = ADRESL;
                    Adc_Value = ADC.Count;

 			if((Adc_Value >=631) && (Adc_Value <=680))
			{
					One_Bulb_Fail_Current =1007;  
					LED_Fail_Current =1022;
			}
   if((Adc_Value >=300) && (Adc_Value <=350))
{
One_Bulb_Fail_Current =1007;  LED_Fail_Current =1018;
}

//                if(IGN_IN == 0)
//                {
                if((RH_SW_IN == 0) && (LH_SW_IN == 0))
                {
                    HW_SW_IN_Flag = 1;
                    DelayCountHigh = 60;
                    DelayCountLow = 177;    
                    
                    if((RELAY_RH == 0) || (RELAY_LH == 0))
                        {
                            for(Delay_Count = 0; Delay_Count < 100; Delay_Count ++);
                            ADCON0 = 0b10001101;      // AN3 Channel Selected (Load Current Check)
                            for(a=0;a<=5;a++);
                            GO_nDONE = 1;
                            while(GO_nDONE == 1)
                            {
                                asm ("nop");
                            }

                            ADC.Byte[1] = ADRESH;
                            ADC.Byte[0] = ADRESL;
                            Adc_Value = ADC.Count;
                            Actual_Current = Adc_Value;

                            if(Actual_Current < SC_Current)
                            {
                                Short_Ckt_Flag = 1;
                                RELAY_RH = 1;
                                LED_RH = 1;
                                RELAY_LH = 1;
                                LED_LH = 1;                                
                                Counter = 0;
                                while((RH_SW_IN == 0) && (LH_SW_IN == 0));
                            }
                    }
                }
                else
                {
                    HW_SW_IN_Flag = 0;
                    if(RH_SW_IN == 0)
                    {
                        RH_SW_ON_FLAG=1;
                        if(RELAY_RH == 0)
                        {
                            LED_RH = 0;
                            for(Delay_Count = 0; Delay_Count < 100; Delay_Count ++);
                            ADCON0 = 0b10001101;      // AN3 Channel Selected (Load Current Check)
                            for(a=0;a<=5;a++);
                            GO_nDONE = 1;
                            while(GO_nDONE == 1)
                            {
                                asm ("nop");
                            }

                            ADC.Byte[1] = ADRESH;
                            ADC.Byte[0] = ADRESL;
                            Adc_Value = ADC.Count;
                            Actual_Current = Adc_Value;

                            if(Actual_Current < SC_Current)
                            {
                                Short_Ckt_Flag = 1;
                                RELAY_RH = 1;
                                LED_RH = 1;
                                Counter = 0;
                                while(RH_SW_IN == 0);
                            }

                            for(Delay_Count = 0; Delay_Count < 12500; Delay_Count ++);	//Delay to avoid Lamp starting rush current.
                            ADCON0 = 0b10010001;      // AN4 Channel Selected (Supply Voltage Check).
                            for(a=0;a<=5;a++);
                            GO_nDONE = 1;
                            while(GO_nDONE == 1)
                            {
                                asm ("nop");
                            }
                            ADC.Byte[1] = ADRESH;
                            ADC.Byte[0] = ADRESL;
                            Adc_Value = ADC.Count;
    /*
                            if(Adc_Value < Working_Voltage_H)
                            {
                                    Flasher_On_Flag = 0;
                                    Relay = 1;
                                    Lamp = 0;
                            }
                            if(Adc_Value > Working_Voltage_L)
                            {
                                    Short_Ckt_Flag = 1;
                                    Relay = 1;
                                    Lamp = 0;
                                    Counter = 0;
                            }
    */
                            ADCON0 = 0b10001101;      // AN3 Channel Selected (Load Current Check for Lamps).
                            for(a=0;a<=5;a++);
                            GO_nDONE = 1;
                            while(GO_nDONE == 1)
                            {
                                asm ("nop");
                            }
                            ADC.Byte[1] = ADRESH;
                            ADC.Byte[0] = ADRESL;
                            Adc_Value = ADC.Count;
                            Actual_Current = Adc_Value;
                            
                            ADCON0 = 0b10001001;      // AN2 Channel Selected (Load Current Check for LED).
                            for(a=0;a<=5;a++);
                            GO_nDONE = 1;
                            while(GO_nDONE == 1)
                            {
                                asm ("nop");
                            }
                            ADC.Byte[1] = ADRESH;
                            ADC.Byte[0] = ADRESL;
                            Adc_Value = ADC.Count;
                            LED_Load_Current = Adc_Value;                            
 
                            if(Actual_Current > SC_Current) 
                            {
                                if((Actual_Current <= One_Bulb_Fail_Current) && (LED_Load_Current <= LED_Fail_Current))
                                {
                                    DelayCountHigh = 60;
                                    DelayCountLow = 177;
                                }
                                else
                                {
                                    DelayCountHigh = 158;	// Need to Change
                                    DelayCountLow = 88;
                                }
                            }
                            else
                            {
                                Short_Ckt_Flag = 1;
                                RELAY_RH = 1;
                                LED_RH = 1;
                                Counter = 0;
                                while(RH_SW_IN == 0);
                            }
                            while(RELAY_RH == 0);
                            LED_RH = 1;
                            for(Delay_Count = 0; Delay_Count < 5000; Delay_Count ++);
                        }
                      else
                        {
                            RELAY_RH = 1;
                            LED_RH = 1;
                        }
                    }
                    else
                    {
                        RH_SW_ON_FLAG=0;
                    }

                    if(LH_SW_IN == 0)
                    {
                        LH_SW_ON_FLAG = 1;
                        if(RELAY_LH == 0)
                        {
                            LED_LH = 0;
                            for(Delay_Count = 0; Delay_Count < 100; Delay_Count ++);
                            ADCON0 = 0b10001101;      // AN3 Channel Selected (Load Current Check)
                            for(a=0;a<=5;a++);
                            GO_nDONE = 1;
                            while(GO_nDONE == 1)
                            {
                                asm ("nop");
                            }
                            ADC.Byte[1] = ADRESH;
                            ADC.Byte[0] = ADRESL;
                            Adc_Value = ADC.Count;
                            Actual_Current = Adc_Value;

                            if(Actual_Current < SC_Current)
                            {
                                Short_Ckt_Flag = 1;
                                RELAY_LH = 1;
                                LED_LH = 1;
                                Counter = 0;
                                while(LH_SW_IN == 0);
                            }

                            for(Delay_Count = 0; Delay_Count < 12500; Delay_Count ++);	//Delay to avoid Lamp starting rush current.

                            ADCON0 = 0b10010001;      // AN4 Channel Selected (Supply Voltage Check).
                            for(a=0;a<=5;a++);
                            GO_nDONE = 1;
                            while(GO_nDONE == 1)
                            {
                                asm ("nop");
                            }
                            ADC.Byte[1] = ADRESH;
                            ADC.Byte[0] = ADRESL;
                            Adc_Value = ADC.Count;
        /*
                            if(Adc_Value < Working_Voltage_H)
                                {
                                        Flasher_On_Flag = 0;
                                        Relay = 1;
                                        Lamp = 0;
                                }

                            if(Adc_Value > Working_Voltage_L)
                                {
                                        Short_Ckt_Flag = 1;
                                        Relay = 1;
                                        Lamp = 0;
                                        Counter = 0;
                                }
         */
                            ADCON0 = 0b10001101;      // AN3 Channel Selected (Load Current Check).
                            for(a=0;a<=5;a++);
                            GO_nDONE = 1;
                            while(GO_nDONE == 1)
                            {
                                asm ("nop");
                            }
                            ADC.Byte[1] = ADRESH;
                            ADC.Byte[0] = ADRESL;
                            Adc_Value = ADC.Count;                            
                            Actual_Current = Adc_Value;
                            
                            ADCON0 = 0b10001001;      // AN2 Channel Selected (Load Current Check for LED).//ADCON0 = 0b10001001;
                            for(a=0;a<=5;a++);
                            GO_nDONE = 1;
                            while(GO_nDONE == 1)
                            {
                                asm ("nop");
                            }
                            ADC.Byte[1] = ADRESH;
                            ADC.Byte[0] = ADRESL;
                            Adc_Value = ADC.Count;
                            LED_Load_Current = Adc_Value;                              

                            if(Actual_Current > SC_Current)
                            {
                                if((Actual_Current <= One_Bulb_Fail_Current) && (LED_Load_Current <= LED_Fail_Current))
                                {
                                    DelayCountHigh = 60;
                                    DelayCountLow = 177;
                                }
                                else
                                {
                                    DelayCountHigh = 158;	// Need to Change
                                    DelayCountLow = 88;
                                }
                            }
                            else
                            {
                             
                                Short_Ckt_Flag = 1;
                                RELAY_LH = 1;
                                LED_LH = 1;
                                Counter = 0;
                                while(LH_SW_IN == 0);
                            }
                            while(RELAY_LH == 0);
                            LED_LH = 1;
                            for(Delay_Count = 0; Delay_Count < 5000; Delay_Count ++);
                        }
                        else
                        {
                            RELAY_LH = 1;
                            LED_LH = 1;
                        }
                    }
                   else
                    {
                        LH_SW_ON_FLAG=0;
                    }
                }
//                }
//                else
//                {
//                    if(HW_SW_IN_Flag == 0)
//                    {
//                       RELAY_RH = 1;
//                       RELAY_LH = 1;
//                    }
//                }
                
//                if(HW_SW_IN == 0)
//                {
//                    HW_SW_IN_Flag = 1; 
//                    CLU_SIG = 1;
//                    DelayCountHigh = 60;
//                    DelayCountLow = 177; 
//                    
//                    if((RELAY_RH == 0) || (RELAY_LH == 0))
//                    {
//                        for(Delay_Count = 0; Delay_Count < 100; Delay_Count ++);
//                        ADCON0 = 0b10001101;      // AN3 Channel Selected (Load Current Check)
//                        for(a=0;a<=5;a++);
//                        GO_nDONE = 1;
//                        while(GO_nDONE == 1)
//                        {
//                            asm ("nop");
//                        }
//
//                        ADC.Byte[1] = ADRESH;
//                        ADC.Byte[0] = ADRESL;
//                        Adc_Value = ADC.Count;
//                        Actual_Current = Adc_Value;
//
//                        if(Actual_Current < SC_Current)
//                        {
//                            Short_Ckt_Flag = 1;
//                            RELAY_RH = 1;
//                            RELAY_LH = 1;
//                            Counter = 0;
//                            while(HW_SW_IN == 0);
//                        }
//                    }
//                }
//                else
//                {
//                    HW_SW_IN_Flag = 0;
//                    CLU_SIG = 0;
//                }
            }
        }
//*********************************************************************
//						Subroutines
//*********************************************************************
void initialise(void)
	{
		OSCCON=0X73;
		// Switch off ADC.
		ADCON1 = 0b11010000;
		ANSELA = 0b00010100;
		ANSELC = 0b00000001;
	
		// Initialise PORT Pins
		CM1CON0 = 0b00000000;
		CM1CON1 = 0b00000000;
		CM2CON0 = 0b00000000;
		CM2CON1 = 0b00000000;

        TRISA = 0b00011100;
		TRISC = 0b00011001;
			
		// Initialse the Timer1 and its interrupts
		TMR1H = 60;
		TMR1L = 177;
		T1CON = 0b0000101;				// Prescaler is 1:1, TMR1 ON.
		PIR1 = 0;						// Clear the interrupt flags
		PIE1 = 0b00000001; 				// Enable TMR1IE
		TMR1ON = 1;
	
		// Initialise the interrupt on RA2/INT
		OPTION_REG = 0b10000000;		// Pull up enable, Int on falling edge
										// TMR0 clock - int, Prescaler to WDT
		INTCON = 0b01000000;			// PEIE Enabled; RA2/INT - Enabled
		T0IE = 0;
		GIE = 1;
	}
//*********************************************************************
//						Interrupt Service routines
//*********************************************************************
void interrupt isr(void)
{
if (INTCONbits.INTF == 1)			// Positive pulse interrupt
        {
            INTCONbits.INTF = 0;		// Clear Flag
//            HW_SW_IN_Flag =! HW_SW_IN_Flag;
        }

if (TMR1IF == 1)			// TMR1 Overflow Interrupt
        {
            TMR1ON = 0;
            TMR1H = DelayCountHigh;
            TMR1L = DelayCountLow;
            TMR1ON = 1;

            Counter ++;
            if(Counter >= 14)
            {
                Counter = 0;
                //RELAY_RH =! RELAY_RH;

                if(HW_SW_IN_Flag == 1)
                {                    
                   if(UL_Flag == 0)
                    {
                        RELAY_RH = 0;
                        RELAY_LH = 0;
                        LED_LH = 0;
                        LED_RH = 0;
                        UL_Flag = 1;
                    }
                   else
                    {
                        RELAY_RH = 1;
                        RELAY_LH = 1;
                        LED_LH = 1;
                        LED_RH = 1;
                        UL_Flag = 0;
                    }
                }
                else
                {
                    if(RH_SW_ON_FLAG == 1)
                    {
                        RELAY_RH =! RELAY_RH;
                    }
                    else
                    {
                        RELAY_RH = 1;
                        LED_RH = 1;
                    }

                    if(LH_SW_ON_FLAG == 1)
                    {
                        RELAY_LH =! RELAY_LH;
                    }
                    else
                    {
                        RELAY_LH = 1;
                        LED_LH = 1;
                    }
                }
            }
            TMR1IF = 0;			// Clear Flag
        }
PIR1 = 0;
}


