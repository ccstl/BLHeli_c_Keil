void switch_power_off()
{
	All_pwmFETs_Off		// Turn off all pwm fets
	All_comFETs_Off		// Turn off all commutation fets
	Set_Pwms_Off
}

void Initialize_PCA()
{
	PCA0CN0 = 0x40;				// PCA enabled
	PCA0MD = 0x08;				// PCA clock is system clock
#IF FETON_DELAY == 0
	#IF MCU_48MHZ == 0
		PCA0PWM = 0x82;				// PCA ARSEL set and 10bits pwm
	#ELSE
		PCA0PWM = 0x83;				// PCA ARSEL set and 11bits pwm
	#ENDIF
		PCA0CENT = 0x00;				// Edge aligned pwm
#ELSE
	#IF MCU_48MHZ == 0
		PCA0PWM = 0x81;				// PCA ARSEL set and 9bits pwm
	#ELSE
		PCA0PWM = 0x82;				// PCA ARSEL set and 10bits pwm
	#ENDIF
		PCA0CENT = 0x03;				// Center aligned pwm
#ENDIF
}

void Initialize_Xbar()
{
	XBR2 = 0x40;	// Xbar enabled			
	XBR1 = 0x02;	// CEX0 and CEX1 routed to pins		
}

void Initialize_Comparator()
{
	CMP0CN0 = 0x80;	// Comparator enabled, no hysteresis
	CMP0MD = 0x00;	// Comparator response time 100ns
}

void Initialize_Adc()
{
	REF0CN = 0x0C;	// Set vdd (3.3V) as reference. Enable temp sensor and bias
#IF MCU_48MHZ == 0
	ADC0CF = 0x59;	// ADC clock 2MHz, PGA gain 1
#ELSE
	ADC0CF = 0xB9;	// ADC clock 2MHz, PGA gain 1
#ENDIF
	ADC0MX = 0x10;	// Select temp sensor input
	ADC0CN0 = 0x80;	// ADC enabled 
	ADC0CN1 = 0x01;	// Common mode buffer enabled
}

void Set_MCU_Clk_24MHz()
	CLKSEL = 0x13;		// Set clock to 24MHz
	SFRPAGE = 0x10;
	PFE0CN = 0x00;		// Set flash timing for 24MHz
	SFRPAGE = 0x00;
	Clock_Set_At_48MHz = 0

void Set_MCU_Clk_48MHz()
	SFRPAGE = 0x10;
	PFE0CN = 0x30;		// Set flash timing for 48MHz
	SFRPAGE = 0x00;
	CLKSEL = 0x03;		// Set clock to 48MHz
	Clock_Set_At_48MHz = 1

