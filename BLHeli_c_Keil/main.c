
void main()
{
	// Disable the WDT.
	WDTCN = 0xDE;		// Disable watchdog
	WDTCN = 0xAD;		
	// Initialize stack
	SP = 0xc0;			// Stack = 64 upper bytes of RAM
	// Initialize VDD monitor
	VDM0CN |= 0x80;    	// Enable the VDD monitor
	RSTSRC = 0x06;   	// Set missing clock and VDD monitor as a reset source if not 1S capable
	// Set clock frequency
	CLKSEL = 0x00;		// Set clock divider to 1
	// Switch power off
	call	switch_power_off
	// Ports initialization
	P0 = P0_INIT;
	P0MDIN = P0_DIGITAL;
	P0MDOUT = P0_PUSHPULL;
	P0 = P0_INIT;
	P0SKIP = P0_SKIP;
	P1 = P1_INIT;
	P1MDIN = P1_DIGITAL;
	P1MDOUT = P1_PUSHPULL;
	P1 = P1_INIT;
	P1SKIP = P1_SKIP;
	P2MDOUT = P2_PUSHPULL;
	// Initialize the XBAR and related functionality
	Initialize_Xbar
	// Switch power off again, after initializing ports
	call	switch_power_off
	// Clear RAM
	clr	A				// Clear accumulator
	mov	Temp1, A			// Clear Temp1
	clear_ram:	
	mov	@Temp1, A			// Clear RAM
	djnz Temp1, clear_ram	// Is A not zero? - jump
	// Set default programmed parameters
	call	set_default_parameters
	// Read all programmed parameters
	call read_all_eeprom_parameters
	// Set beep strength
	mov	Temp1, #Pgm_Beep_Strength
	mov	Beep_Strength, @Temp1
	// Set initial arm variable
	mov	Initial_Arm, #1
	// Initializing beep
	clr	IE_EA			// Disable interrupts explicitly
	call wait200ms	
	call beep_f1
	call wait30ms
	call beep_f2
	call wait30ms
	call beep_f3
	call wait30ms
	call	led_control
}