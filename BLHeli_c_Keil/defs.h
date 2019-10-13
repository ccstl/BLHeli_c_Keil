
//*********************
// PORT 1 definitions *
//*********************
//			EQU	7	;i
//			EQU	6	;i
#define ApwmFET		EQU	5	;o
#define BpwmFET		EQU	4	;o
#define CpwmFET		EQU	3	;o
#define CcomFET		EQU	2	;o
#define BcomFET		EQU	1	;o
#define AcomFET		EQU	0	;o

#define P1_DIGITAL	 	(1 SHL ApwmFET)+(1 SHL BpwmFET)+(1 SHL CpwmFET)+(1 SHL AcomFET)+(1 SHL BcomFET)+(1 SHL CcomFET)
#define P1_INIT			  (1 SHL AcomFET)+(1 SHL BcomFET)+(1 SHL CcomFET)
#define P1_PUSHPULL	 	(1 SHL ApwmFET)+(1 SHL BpwmFET)+(1 SHL CpwmFET)+(1 SHL AcomFET)+(1 SHL BcomFET)+(1 SHL CcomFET)
#define P1_SKIP			 	0x3F

#IF FETON_DELAY != 0
#define All_pwmFETs_Off
	P1.ApwmFET = 0;
	P1.BpwmFET = 0;
	P1.CpwmFET = 0;
#ELSE
#define All_pwmFETs_Off
	P1.AcomFET = 1;
	P1.BcomFET = 1;
	P1.CcomFET = 1;
#ENDIF

#define Set_Pwm_Polarity PCA0POL = 0x00;				// Damping noninverted, pwm noninverted


#IF FETON_DELAY == 0
#define Enable_Power_Pwm_Module	PCA0CPM0 = 0x4A;				// Enable comparator of module, enable match, set pwm mode
#ELSE
#define Enable_Power_Pwm_Module	PCA0CPM1 = 0x42;				// Enable comparator of module, set pwm mode
#ENDIF


#IF FETON_DELAY == 0
#define Enable_Damp_Pwm_Module PCA0CPM1 = 0x00;				// Disable
#ELSE
#define Enable_Damp_Pwm_Module PCA0CPM0 = 0x42;				// Enable comparator of module, set pwm mode
#ENDIF

#define Set_Power_Pwm_Regs
#IF FETON_DELAY == 0
	PCA0CPL0 = Power_Pwm_Reg_L;
	PCA0CPH0 = Power_Pwm_Reg_H;
#ELSE
	PCA0CPL1 = Power_Pwm_Reg_L >> 1;
	PCA0CPH1 = Power_Pwm_Reg_H >> 1;
#ENDIF

#define Set_Damp_Pwm_Regs MACRO
#IF FETON_DELAY == 0
	PCA0CPL1 = Damp_Pwm_Reg_L;
	PCA0CPH1 = Damp_Pwm_Reg_H;
#ELSE
	PCA0CPL0 = Damp_Pwm_Reg_L >> 1;
	PCA0CPH0 = Damp_Pwm_Reg_H >> 1;
#ENDIF
		
#define Clear_COVF_Interrupt PCA0PWM &= 0x0DF;

#define Clear_CCF_Interrupt PCA0CN0 &= 0x0FE;// CCF interrupt is only used for FETON_DELAY == 0

#define Enable_COVF_Interrupt PCA0PWM |= 0x40;

#define Enable_CCF_Interrupt PCA0CPM0 |= 0x01;

#define Disable_COVF_Interrupt PCA0PWM |= 0xBF;

#define Disable_CCF_Interrupt PCA0CPM0 |= 0xFE;

#define ApwmFET_on
	P1.ApwmFET = 1;
#IF FETON_DELAY == 0
	P1.AcomFET = 0;
#ENDIF

#define ApwmFET_off MACRO
#IF FETON_DELAY != 0
	P1.ApwmFET = 0;
#ELSE
	P1.AcomFET = 1;
#ENDIF

#define BpwmFET_on
	P1.BpwmFET = 1;
#IF FETON_DELAY == 0
	P1.BcomFET = 0;
#ENDIF

#define BpwmFET_off
#IF FETON_DELAY != 0
	P1.BpwmFET = 0;
#ELSE
	P1.BcomFET = 1;
#ENDIF

#define CpwmFET_on
	P1.CpwmFET = 1;
#IF FETON_DELAY == 0
	P1.CcomFET = 0;
#ENDIF

#define CpwmFET_off
#IF FETON_DELAY != 0
	P1.CpwmFET = 0;
#ELSE
	P1.CcomFET = 1;
#ENDIF

#define All_pwmFETs_Off
#IF FETON_DELAY != 0
	P1.ApwmFET = 0;
	P1.BpwmFET = 0;
	P1.CpwmFET = 0;
#ELSE
	P1.AcomFET = 1;
	P1.BcomFET = 1;
	P1.CcomFET = 1;
#ENDIF

#define AcomFET_on
#IF FETON_DELAY == 0
	P1.ApwmFET = 0;
#ENDIF
	P1.AcomFET = 0;

#define AcomFET_off
	P1.AcomFET = 1;

#define BcomFET_on
#IF FETON_DELAY == 0
	P1.BpwmFET = 0;
#ENDIF
	P1.BcomFET = 0;

#define BcomFET_off P1.BcomFET = 1;

#define CcomFET_on
#IF FETON_DELAY == 0
	P1.CpwmFET = 0;
#ENDIF
	P1.CcomFET = 0;

#define CcomFET_off P1.CcomFET = 1;

#define All_comFETs_Off \
	P1.AcomFET = 1;\
	P1.BcomFET = 1;\
	P1.CcomFET = 1;


#define Set_Pwm_A
#IF FETON_DELAY == 0
	P1.AcomFET = 0;
	P1SKIP = 0x1F;
#ELSE
	P1SKIP = 0x1E;
#ENDIF

#define Set_Pwm_B
#IF FETON_DELAY == 0
	P1.BcomFET = 0;
	P1SKIP = 0x2F;
#ELSE
	P1SKIP = 0x2D;
#ENDIF

#define Set_Pwm_C
#IF FETON_DELAY == 0
	P1.CcomFET = 0;
	P1SKIP = 0x37;
#ELSE
	P1SKIP = 0x33;
#ENDIF

#define Set_Pwms_Off P1SKIP = 0x3F;


#define Set_Comp_Phase_A CMP0MX = 0x12;	// Set comparator multiplexer to phase A

#define Set_Comp_Phase_B CMP0MX = 0x32;	// Set comparator multiplexer to phase B

#define Set_Comp_Phase_C CMP0MX = 0x52;	// Set comparator multiplexer to phase C

//#define Read_Comp_Out mov	A, CMP0CN0	// Read comparator output

#define Start_Adc ADC0CN0 = 0x90;	// ADC start

#define Read_Adc_Result
	Temp1, ADC0L
	Temp2, ADC0H



