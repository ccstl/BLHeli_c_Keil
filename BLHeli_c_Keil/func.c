//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Set pwm limit low rpm
//
// No assumptions
//
// Sets power limit for low rpms and disables demag for low rpms
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void set_pwm_limit_low_rpm()
{
	// Set pwm limit
	Temp1 = 0xFF;					// Default full power
	if(Flags1.STARTUP_PHASE) 
		Pwm_Limit_By_Rpm = Temp1;	// Exit if startup phase set

	if(!Pgm_Enable_Power_Prot)		// Check if low RPM power protection is enabled
		Pwm_Limit_By_Rpm = Temp1;	// Exit if disabled

	if(!Comm_Period4x_H)
		Pwm_Limit_By_Rpm = Temp1;		// Avoid divide by zero

	ta = 255/Comm_Period4x_H;
	tb = Low_Rpm_Pwr_Slope;
	if(!Flags1.INITIAL_RUN_PHASE)	// More protection for initial run phase 
		Temp1 = Low_Rpm_Pwr_Slope*255/Comm_Period4x_H;tb = Low_Rpm_Pwr_Slope;
	else Temp1 = 5*255/Comm_Period4x_H;tb = 5;
	
	if(!tb)Temp1 = 0xFF;				

	if(Temp1>Pwm_Limit_Beg) Pwm_Limit_By_Rpm = Temp1;
	Temp1 = Pwm_Limit_Beg;// Limit to min
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Set pwm limit high rpm
//
// No assumptions
//
// Sets power limit for high rpms
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void set_pwm_limit_high_rpm()
{
#IF MCU_48MHZ == 1
	if(Comm_Period4x_L>0xA0)// Limit Comm_Period to 160, which is 500k erpm
	{
		if(!Comm_Period4x_H) 
			Pwm_Limit_By_Rpm = Comm_Period4x_H + 1;
		return;
	}
#ELSE
	if(Comm_Period4x_L>0xE4)// Limit Comm_Period to 228, which is 350k erpm
	{
		if(!Comm_Period4x_H) 
			Pwm_Limit_By_Rpm = Comm_Period4x_H + 1;
		return;
	}
#ENDIF
	if(!Comm_Period4x_H)
		Pwm_Limit_By_Rpm = Comm_Period4x_H - 1;
	return;
}

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Check temperature, power supply voltage and limit power
;
; No assumptions
;
; Used to limit main motor power in order to maintain the required voltage
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
void check_temp_voltage_and_limit_power()
{
	Adc_Conversion_Cnt++;			// Increment conversion counter
	if(Adc_Conversion_Cnt<8)		// Is conversion count equal to temp rate?
	{	// No - check voltage
		// Increase pwm limit
		if(Pwm_Limit + 16 > 255) Pwm_Limit = 255;
		Pwm_Limit += 16;			// If not max - branch
		return;
	}

	// Wait for ADC conversion to complete
	while(!ADC0CN0_ADINT);
	// Read ADC result
	temp1 = ADC0L;
	temp2 = ADC0H;
	// Stop ADC
	//Stop_Adc //空宏定义

	Adc_Conversion_Cnt = 0;		// Yes - temperature check. Reset counter
	if(!Pgm_Enable_Temp_Prot)	return;			// Is temp protection enabled?

	if(ADC0H)		// Is temperature reading below 256?
	{
		if(ADC0L == Current_Average_Temp);	// Equal - no change

		if(ADC0L > Current_Average_Temp)// Above - increment average
		{	
			Current_Average_Temp++;
			if(Current_Average_Temp)
				Current_Average_Temp--;
			temp_average_updated();
		}
		if(!Current_Average_Temp)
			temp_average_updated();		// Below - decrement average if average is not already zero
	}
	// Yes -  decrement average
	if(!Current_Average_Temp)	temp_average_updated();	// Already zero - no change
	Current_Average_Temp--;			// Decrement 
	temp_average_updated();
}

void temp_average_updated()
{
	if(Current_Average_Temp < Temp_Prot_Limit)			// Is temperature below first limit?
		return;			// Yes - exit
	Pwm_Limit = 192;			// No - limit pwm

	if(Current_Average_Temp < TEMP_LIMIT_STEP/2)		// Is temperature below second limit
		return;			// Yes - exit
	Pwm_Limit = 128;			// No - limit pwm

	clr	C
	if(Current_Average_Temp < TEMP_LIMIT_STEP/2)		// Is temperature below third limit
		return;			// Yes - exit
	Pwm_Limit = 64;				// No - limit pwm


	if(Current_Average_Temp < TEMP_LIMIT_STEP/2)		// Is temperature below final limit
		return;			// Yes - exit
	Pwm_Limit = 0;				// No - limit pwm
	
	return;
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Set startup PWM routine
//
// Either the SETTLE_PHASE or the STEPPER_PHASE flag must be set
//
// Used for pwm control during startup
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void set_startup_pwm()
{
	// Adjust startup power
	Pwm_Limit_Beg = Pgm_Startup_Pwr_Decoded << 1;		// Multiply result by 2 (unity gain is 128)
	// Set initial pwm limit
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Initialize timing routine
//
// No assumptions
//
// Part of initialization before motor start
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void initialize_timing()
{
	Comm_Period4x_L = 0x00;				// Set commutation period registers
	Comm_Period4x_H = 0xF0;
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Calculate next commutation timing routine
//
// No assumptions
//
// Called immediately after each commutation
// Also sets up timer 3 to wait advance timing
// Two entry points are used
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void calc_next_comm_timing()		// Entry point for run phase
	// Read commutation time
	IE_EA = 0;
	TMR2CN0_TR2 = 0;		// Timer 2 disabled
	Temp1 = TMR2L;		// Load timer value
	Temp2 = TMR2H;	
	Temp3 = Timer2_X;
	if(!TMR2CN0_TF2H) TMR2CN0_TR2 = 1;	// Check if interrupt is pending
	Temp3++;			// If it is pending, then timer has already wrapped
	TMR2CN0_TR2 = 1;		// Timer 2 enabled
	IE_EA = 1;
#IF MCU_48MHZ == 1
	Temp3 = Temp3 >> 1;
	Temp2 = Temp2 >> 1;
	Temp1 = Temp1 >> 1;
#ENDIF
	// Calculate this commutation time
	Temp4 = Prev_Comm_L;
	Temp5 = Prev_Comm_H;
	Prev_Comm_L = Temp1;		// Store timestamp as previous commutation
	Prev_Comm_H = Temp2;

	Temp1 = Temp1 - Temp4;	// Calculate the new commutation time
	A = Temp2 - Temp5;
	if(Flags1.STARTUP_PHASE) calc_next_comm_startup

IF MCU_48MHZ == 1
	Temp2 = (Temp2 - Temp5)&0x7F;
ENDIF
	Temp2 = Temp2 - Temp5;
	if(!Flags1.HIGH_RPM) calc_next_comm_normal	// Branch if high rpm
	calc_next_comm_timing_fast
	calc_next_comm_normal

calc_next_comm_startup:
	Temp6 = Prev_Comm_X;
	Prev_Comm_X = Temp3;		// Store extended timestamp as previous commutation
	Temp2 = A;
	A = Temp3 - Temp6;				// Calculate the new extended commutation time
IF MCU_48MHZ == 1
	Temp3 = (Temp3 - Temp6)&0x7F;
ENDIF
	Temp3 = Temp3 - Temp6;
	if(!A)calc_next_comm_startup_no_X

	Temp1 = 0xFF;
	Temp2 = 0xFF;
	calc_next_comm_startup_average

calc_next_comm_startup_no_X:
	Temp7 = Prev_Prev_Comm_L;
	Temp8 = Prev_Prev_Comm_H;
	Prev_Prev_Comm_L = Temp4;
	Prev_Prev_Comm_H = Temp5;
	Temp1 = Prev_Comm_L;		// Reload this commutation time	
	Temp2 = Prev_Comm_H;

	Temp1 = Temp1 - Temp7;			// Calculate the new commutation time based upon the two last commutations (to reduce sensitivity to offset)
	Temp2 = Temp2 - Temp8;

calc_next_comm_startup_average:
	Temp4 = Comm_Period4x_H >> 1;		// Average with previous and save
	Temp3 = Comm_Period4x_L >> 1;

	Comm_Period4x_L = Temp1 + Temp3;
	Comm_Period4x_H = Temp2 + Temp4;
	if(Comm_Period4x_L&0x01 == 0 || Comm_Period4x_H&0x01)
		calc_new_wait_times_setup
	Comm_Period4x_L = 0xFF;
	Comm_Period4x_H = 0xFF;
	calc_new_wait_times_setup

calc_next_comm_normal:
	// Calculate new commutation time 
	Temp3 = Comm_Period4x_L;	// Comm_Period4x(-l-h) holds the time of 4 commutations
	Temp4 = Comm_Period4x_H;
	Temp5 = Comm_Period4x_L;	// Copy variables
	Temp6 = Comm_Period4x_H;
	Temp7 = #4;				// Divide Comm_Period4x 4 times as default
	Temp8 = #2;				// Divide new commutation time 2 times as default

	if(Temp4 < 0x04)	calc_next_comm_avg_period_div;
	Temp7--;				// Reduce averaging time constant for low speeds
	Temp8--;

	if(Temp4 < 0x08)	calc_next_comm_avg_period_div;

	if(Flags1.INITIAL_RUN_PHASE) calc_next_comm_avg_period_div	// Do not average very fast during initial run
	Temp7--;				// Reduce averaging time constant more for even lower speeds
	Temp8--;

calc_next_comm_avg_period_div:
	Temp6 =	Temp6 >> 1;				// Divide by 2
	Temp5 =	Temp5 >> 1;

	if(Temp7--) calc_next_comm_avg_period_div

	Temp3 = Temp3 - Temp5;				// Subtract a fraction
	Temp4 = Temp4 - Temp6;

	if(!Temp8) calc_next_comm_new_period_div_done // Divide new time

calc_next_comm_new_period_div:					
	Temp2 =	Temp2 >> 1;					// Divide by 2
	Temp1 =	Temp1 >> 1;
	if(Temp8--) calc_next_comm_new_period_div

calc_next_comm_new_period_div_done:
	Temp3 = Temp1 + Temp3;				// Add the divided new time
	Temp4 = Temp2 + Temp4;
	
	Comm_Period4x_L = Temp3;	// Store Comm_Period4x_X
	Comm_Period4x_H = Temp4;
	if(Temp2 + Temp4 < 0xFF)	calc_new_wait_times_setup	// If period larger than 0xffff - go to slow case

	Temp4 = 0xFF;
	Comm_Period4x_L = Temp4;	// Set commutation period registers to very slow timing (0xffff)
	Comm_Period4x_H = Temp4;

calc_new_wait_times_setup:	
	// Set high rpm bit (if above 156k erpm)
	mov	A, Temp4
	subb	A, #2
	if(Temp4 > 2)
		if(!Flags1.STARTUP_PHASE) calc_new_wait_per_startup_done

	Flags1.HIGH_RPM = 1; 		// Set high rpm bit

	// Load programmed commutation timing
	if(!Flags1.STARTUP_PHASE) calc_new_wait_per_startup_done	// Set dedicated timing during startup

	Temp8 = 3;
	calc_new_wait_per_demag_done

calc_new_wait_per_startup_done:
	Temp8 = Pgm_Comm_Timing;				// Store in Temp8
	// Check demag metric
	if(Demag_Detected_Metric < 130)	calc_new_wait_per_demag_done
	Temp8++;				// Increase timing

	if(Demag_Detected_Metric >160) Temp8++;				// Increase timing again

	if(Temp8 > 6)				// Limit timing to max
		Temp8 = 5;				// Set timing to max

calc_new_wait_per_demag_done:
	// Set timing reduction
	Temp7 = 2;
	// Load current commutation timing
	Temp2 = Comm_Period4x_H >> 4;		// Divide 4 times

	Temp1 = (Comm_Period4x_H << 4)&0xF0;

	Temp1 = (Comm_Period4x_L >> 4)&0xF0 + Temp1;

	Temp3 = Temp1 - Temp7;

	Temp4 = Temp2;				

	if(Temp1 < Temp7)	load_min_time			// Check that result is still positive

	clr	C
	if(Temp3 < 1 || Temp4 < 0)	calc_new_wait_times_exit	// Check that result is still above minumum

load_min_time:
	Temp3 = 1;
	Temp4 = 0;

calc_new_wait_times_exit:	
	wait_advance_timing();


// Fast calculation (Comm_Period4x_H less than 2)
calc_next_comm_timing_fast:			
	// Calculate new commutation time
	Temp3 = Comm_Period4x_L;	// Comm_Period4x(-l-h) holds the time of 4 commutations
	Temp4 = Comm_Period4x_H;
	Temp7 = swap Temp4				// Divide by 2 4 times

	Temp5 = (Temp3 >> 4)&0x0F|Temp7;

	Temp3 = Temp3 - Temp5;				// Subtract a fraction

	Temp1 = Temp1 >> 2;// Divide by 2 2 times
	Temp3 = Temp3 + Temp1;				// Add the divided new time

	Comm_Period4x_L = Temp3;	// Store Comm_Period4x_X
	Comm_Period4x_H = Temp4;

	if(Temp4 > 2)				// If erpm below 156k - go to normal case
		clr	Flags1.HIGH_RPM 		// Clear high rpm bit

	// Set timing reduction
	Temp1 = 2
	Temp7 = swap Temp4				// Divide by 2 4 times
	Temp4 = 0
	Temp3 = (Temp3 >> 4)&0x0F|Temp7;
	
	if(Temp3 < Temp1)
		Temp3 = 1;		// Check that result is still positive
	Temp3 = Temp3 - Temp1;

	if(Temp3 < 1)
		calc_new_wait_times_fast_done	// Check that result is still above minumum
	Temp3 = 1;
	Temp8 = Pgm_Comm_Timing;	// Load timing setting Store in Temp8
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Wait advance timing routine
//
// No assumptions
// NOTE: Be VERY careful if using temp registers. They are passed over this routine
//
// Waits for the advance timing to elapse and sets up the next zero cross wait
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
wait_advance_timing()
{	
	while(!Flags0.T3_PENDING);

	// Setup next wait time
	TMR3RLL = Wt_ZC_Tout_Start_L;
	TMR3RLH = Wt_ZC_Tout_Start_H;
	Flags0.T3_PENDING = 1;
	EIE1 |= 0x80;	// Enable timer 3 interrupts
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Calculate new wait times routine
//
// No assumptions
//
// Calculates new wait times
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void calc_new_wait_times()	
{
	Temp1 = -Temp3;				// Negate
	Temp2 = -Temp4;
#IF MCU_48MHZ == 1
	clr	C
	Temp1 = Temp1 << 1;				// Multiply by 2
	Temp2 = Temp2 << 1;
#ENDIF
	if(Flags1.HIGH_RPM)	// Branch if high rpm
		calc_new_wait_times_fast;

	Temp3 = Temp1;				// Copy values
	Temp4 = Temp2;
	//setb	C					// Negative numbers - set carry
	Temp6 = Temp2 >> 1;					// Divide by 2
	Temp5 = Temp1 >> 1;

	Wt_Zc_Tout_Start_L = Temp1; //Set 15deg time for zero cross scan timeout
	Wt_Zc_Tout_Start_H = Temp2;
	clr	C
	// (Temp8 has Pgm_Comm_Timing)
	if(Temp8 == 3)				// Is timing normal?
		store_times_decrease	// Yes - branch

	if(Temp8&0x01)				
		adjust_timing_two_steps	// If an odd number - branch

	Temp1 = Temp1 + Temp5;				// Add 7.5deg and store in Temp1/2
	Temp2 = Temp6 + Temp2;
	Temp3 = Temp5;				// Store 7.5deg in Temp3/4
	Temp4 = Temp6;
	store_times_up_or_down

adjust_timing_two_steps:
	Temp1 = Temp1 << 1;				// Add 15deg and store in Temp1/2
	Temp2 = Temp2 << 1;
	
	Temp1++;
	Temp2++;

	Temp3 = -1;				// Store minimum time in Temp3/4
	Temp4 = 0xFF;

store_times_up_or_down:				
	if(Temp8 <= 3)					// Is timing higher than normal?
		store_times_decrease		// No - branch

store_times_increase:
	Wt_Comm_Start_L = Temp3;		// Now commutation time (~60deg) divided by 4 (~15deg nominal)
	Wt_Comm_Start_H = Temp4;
	Wt_Adv_Start_L = Temp1;		// New commutation advance time (~15deg nominal)
	Wt_Adv_Start_H = Temp2;
	Wt_Zc_Scan_Start_L = Temp5;	// Use this value for zero cross scan delay (7.5deg)
	Wt_Zc_Scan_Start_H = Temp6;
	ljmp	wait_before_zc_scan;

store_times_decrease:
	Wt_Comm_Start_L = Temp1		// Now commutation time (~60deg) divided by 4 (~15deg nominal)
	Wt_Comm_Start_H = Temp2
	Wt_Adv_Start_L = Temp3		// New commutation advance time (~15deg nominal)
	Wt_Adv_Start_H = Temp4
	Wt_Zc_Scan_Start_L = Temp5	// Use this value for zero cross scan delay (7.5deg)
	Wt_Zc_Scan_Start_H = Temp6
	if(!Flags1.STARTUP_PHASE) store_times_exit

	Wt_Comm_Start_L = 0xF0;		// Set very short delays for all but advance time during startup, in order to widen zero cross capture range
	Wt_Comm_Start_H = 0xFF;
	Wt_Zc_Scan_Start_L = 0xF0;
	Wt_Zc_Scan_Start_H = 0xFF;
	Wt_Zc_Tout_Start_L = 0xF0;
	Wt_Zc_Tout_Start_H = 0xFF;

store_times_exit:
	ljmp	wait_before_zc_scan


calc_new_wait_times_fast:	
	Temp3 = Temp1;				// Copy values

	// Negative numbers - set carry
	Temp5 = Temp1 >> 1;				// Divide by 2

	Wt_Zc_Tout_Start_L = Temp1; // Set 15deg time for zero cross scan timeout

	// (Temp8 has Pgm_Comm_Timing)
	if(Temp8 == 3)				// Is timing normal?
		store_times_decrease_fast// Yes - branch

	if(Temp8&0x01)				
		adjust_timing_two_steps_fast	// If an odd number - branch
	Temp1 = Temp1 + Temp5;				// Add 7.5deg and store in Temp1
	Temp3 = Temp5;				// Store 7.5deg in Temp3
	ajmp	store_times_up_or_down_fast

adjust_timing_two_steps_fast:
	Temp1 = (Temp1 >> 1) + 1;				// Add 15deg and store in Temp1
	Temp3 = -1;			// Store minimum time in Temp3

store_times_up_or_down_fast:
	mov	A, Temp8				
	if(Temp8 < 3)				// Is timing higher than normal?
		store_times_decrease_fast// No - branch

store_times_increase_fast:
	Wt_Comm_Start_L = Temp3;		// Now commutation time (~60deg) divided by 4 (~15deg nominal)
	Wt_Adv_Start_L = Temp1;		// New commutation advance time (~15deg nominal)
	Wt_Zc_Scan_Start_L = Temp5;	// Use this value for zero cross scan delay (7.5deg)
	ljmp	wait_before_zc_scan

store_times_decrease_fast:
	Wt_Comm_Start_L = Temp1;		// Now commutation time (~60deg) divided by 4 (~15deg nominal)
	Wt_Adv_Start_L = Temp3;		// New commutation advance time (~15deg nominal)
	Wt_Zc_Scan_Start_L = Temp5;	// Use this value for zero cross scan delay (7.5deg)
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Wait before zero cross scan routine
//
// No assumptions
//
// Waits for the zero cross scan wait time to elapse
// Also sets up timer 3 for the zero cross scan timeout time
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void wait_before_zc_scan()
{	
	while(Flags0.T3_PENDING);

	Startup_Zc_Timeout_Cntd = 2;

	Flags0.T3_PENDING = 1;
	EIE1 |= 0x80;			// Enable timer 3 interrupts
	if(Flags1 & ((1<<STARTUP_PHASE)+(1<<INITIAL_RUN_PHASE))==0)
		return;		

	Temp1 = Comm_Period4x_L;	// Set long timeout when starting
	Temp2 = Comm_Period4x_H;

	Temp2 = Temp2 >> 1;
	Temp1 = Temp1 >> 1;

#IF MCU_48MHZ == 0
	Temp2 = Temp2 >> 1;
	Temp1 = Temp1 >> 1;
#ENDIF
	if(Flags1.STARTUP_PHASE)
		Temp2 = Temp2 + 0x40;				// Increase timeout somewhat to avoid false wind up

	IE_EA = 0;
	EIE1 &= 0x7F;			// Disable timer 3 interrupts
	TMR3CN0 = 0x00;			// Timer 3 disabled and interrupt flag cleared

	TMR3L = -Temp1;				// Set timeout
	TMR3H = -Temp2;

	TMR3CN0 = 0x04;			// Timer 3 enabled and interrupt flag cleared
	Flags0.T3_PENDING = 1;
	EIE1 |= 80;			// Enable timer 3 interrupts
	IE_EA = 1;

	return;
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Wait for comparator to go low/high routines
//
// No assumptions
//
// Waits for the zero cross scan wait time to elapse
// Then scans for comparator going low/high
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void wait_for_comp_out_low()
{
	Flags0.DEMAG_DETECTED = 1;		// Set demag detected flag as default
	Comparator_Read_Cnt = 0;		// Reset number of comparator reads
	Bit_Access = 0x00;			// Desired comparator output
	if(Flags1.DIR_CHANGE_BRAKE)
		Bit_Access = 0x40;		
	wait_for_comp_out_start

wait_for_comp_out_high:
	Flags0.DEMAG_DETECTED = 1;		// Set demag detected flag as default
	Comparator_Read_Cnt = 0;		// Reset number of comparator reads
	Bit_Access = 0x40;				// Desired comparator output
	if(Flags1.DIR_CHANGE_BRAKE)
		Bit_Access = 0x00;

wait_for_comp_out_start:
	// Set number of comparator readings
	Temp1 = 1;					// Number of OK readings required
	Temp2 = 1;					// Max number of readings required
	if(Flags1.HIGH_RPM) comp_scale_samples	// Branch if high rpm

	mov	A, Flags1					// Clear demag detected flag if start phases
	if(Flags1 & ((1 << STARTUP_PHASE)+(1 << INITIAL_RUN_PHASE)))
		Flags0.DEMAG_DETECTED = 0;

	Temp2 = 0x20; 				// Too low value (~<15) causes rough running at pwm harmonics. Too high a value (~>35) causes the RCT4215 630 to run rough on full throttle
	if(!Comm_Period4x_H>>1)			// Set number of readings higher for lower speeds
		Comm_Period4x_H++;
	Temp1 = Comm_Period4x_H;

	if(Comm_Period4x_H > 20)
		Temp1 = 0x20;
	
	if(!Flags1.STARTUP_PHASE) comp_scale_samples
	Temp1 = 0x27;				// Set many samples during startup, approximately one pwm period
	Temp2 = 0x27;

comp_scale_samples:
#IF MCU_48MHZ == 1
	Temp1 = Temp1 << 1;
	Temp2 = Temp2 << 1;
#ENDIF
comp_check_timeout:
	if(Flags0.T3_PENDING) comp_check_timeout_not_timed_out		// Has zero cross scan timeout elapsed?

	if(!Comparator_Read_Cnt)			// Check that comparator has been read
		comp_check_timeout_not_timed_out	// If not read - branch

	if(!Flags1.STARTUP_PHASE) comp_check_timeout_timeout_extended	// Extend timeout during startup

	if(Startup_Zc_Timeout_Cntd--) comp_check_timeout_extend_timeout

comp_check_timeout_timeout_extended:
	Flags0.COMP_TIMED_OUT = 1;
	ajmp	setup_comm_wait

comp_check_timeout_extend_timeout:
	call	setup_zc_scan_timeout
comp_check_timeout_not_timed_out:
	Comparator_Read_Cnt++;			// Increment comparator read count
	CMP0CN0 = CMP0CN0&0x40;					// Read comparator output
	if(CMP0CN0 != Bit_Access) comp_read_wrong
	ajmp	comp_read_ok
	
comp_read_wrong:
	if(!Flags1.STARTUP_PHASE) comp_read_wrong_not_startup

	Temp1++;					// Increment number of OK readings required
	mov	A, Temp1
	if(Temp1 > Temp2)					// If above initial requirement - do not increment further
		Temp1--;
	comp_check_timeout			// Continue to look for good ones

comp_read_wrong_not_startup:
	if(Flags0.DEMAG_DETECTED) comp_read_wrong_extend_timeout

	Temp1++					// Increment number of OK readings required

	if(Temp1 > Temp2)
		wait_for_comp_out_start		// If above initial requirement - go back and restart
	comp_check_timeout			// Otherwise - take another reading

comp_read_wrong_extend_timeout:
	Flags0.DEMAG_DETECTED = 0;		// Clear demag detected flag
	EIE1 &= 0x7F;				// Disable timer 3 interrupts
	TMR3CN0 = 0x00;				// Timer 3 disabled and interrupt flag cleared
	if(!Flags1.HIGH_RPM) comp_read_wrong_low_rpm	// Branch if not high rpm

	TMR3L = 0x00;				// Set timeout to ~1ms
#IF MCU_48MHZ == 1
	TMR3H = 0xF0;
#ELSE
	TMR3H = 0xF8;
#ENDIF
comp_read_wrong_timeout_set:
	TMR3CN0 = 0x04;				// Timer 3 enabled and interrupt flag cleared
	Flags0.T3_PENDING = 1;
	EIE1 |= 0x80;				// Enable timer 3 interrupts
	wait_for_comp_out_start		// If comparator output is not correct - go back and restart

comp_read_wrong_low_rpm:
	Temp7 = 0xFF;				// Default to long
#IF MCU_48MHZ == 1
	if(Comm_Period4x_H&0x80)// Set timeout to ~4x comm period 4x value
		comp_read_wrong_load_timeout
#ENDIF
	if(Comm_Period4x_H&0x80)
		comp_read_wrong_load_timeout

	if(Comm_Period4x_H&0x40)
		comp_read_wrong_load_timeout
#IF MCU_48MHZ == 1
	Temp7 = Comm_Period4x_H << 1;
#ENDIF
	Temp7 = Comm_Period4x_H << 2;

comp_read_wrong_load_timeout:
	TMR3L = 0;
	TMR3H = 255-Temp7;
	comp_read_wrong_timeout_set

comp_read_ok:
	clr	C
	if(Startup_Cnt < 1)				// Force a timeout for the first commutation
		wait_for_comp_out_start

	if(Flags0.DEMAG_DETECTED)	// Do not accept correct comparator output if it is demag
		wait_for_comp_out_start

	if(Temp1--) comp_check_timeout		// Decrement readings counter - repeat comparator reading if not zero
	Flags0.COMP_TIMED_OUT = 0;
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Setup commutation timing routine
//
// No assumptions
//
// Sets up and starts wait from commutation to zero cross
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void setup_comm_wait()
{
	IE_EA = 0;
	EIE1 &= 0x7F;		// Disable timer 3 interrupts
	TMR3CN0 = 0x00;		// Timer 3 disabled and interrupt flag cleared
	TMR3L = Wt_Comm_Start_L;
	TMR3H = Wt_Comm_Start_H;
	TMR3CN0 = 0x04;		// Timer 3 enabled and interrupt flag cleared
	// Setup next wait time
	TMR3RLL = Wt_Adv_Start_L;
	TMR3RLH = Wt_Adv_Start_H;
	Flags0.T3_PENDING = 1;
	EIE1 |= 0x80;		// Enable timer 3 interrupts
	IE_EA = 1;			// Enable interrupts again
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Evaluate comparator integrity
//
// No assumptions
//
// Checks comparator signal behaviour versus expected behaviour
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void evaluate_comparator_integrity()
{
	if(!(Flags1 & ((1 << STARTUP_PHASE)+(1 << INITIAL_RUN_PHASE))))
		eval_comp_check_timeout

	if(Flags1.INITIAL_RUN_PHASE)return;		// Do not increment beyond startup phase
	Startup_Cnt++;					// Increment counter

	if(!Flags0.COMP_TIMED_OUT) return;	// Has timeout elapsed?
	if(Flags1.DIR_CHANGE_BRAKE) return;	// Do not exit run mode if it is braking
	if(Flags0.DEMAG_DETECTED) return;	// Do not exit run mode if it is a demag situation
	SP--;								// Routine exit without "ret" command
	SP--;
	run_to_wait_for_power_on_fail			// Yes - exit run mode
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Wait for commutation routine
//
// No assumptions
//
// Waits from zero cross to commutation 
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void wait_for_comm()
{
	// Update demag metric
	Temp1 = 0;
	if(Flags0.DEMAG_DETECTED)
		Temp1 = 1;

	// Sliding average of 8, 256 when demag and 0 when not. Limited to minimum 120				
	Temp2 = Demag_Detected_Metric * 7>>8;// Multiply by 7
	// Add new value for current demag status
	B = Demag_Detected_Metric * 7>>8 + Temp1;
	// Divide by 8
	Demag_Detected_Metric = Temp2 >> 3;
	if(Demag_Detected_Metric < 120)				// Limit to minimum 120
		Demag_Detected_Metric = 120;

	if(Demag_Detected_Metric < Demag_Pwr_Off_Thresh)	// Check demag metric
		wait_for_comm_wait		// Cut power if many consecutive demags. This will help retain sync during hard accelerations

	All_pwmFETs_off
	Set_Pwms_Off

wait_for_comm_wait:
	if(Flags0.T3_PENDING)
		wait_for_comm_wait

	// Setup next wait time
	TMR3RLL = Wt_Zc_Scan_Start_L;
	TMR3RLH = Wt_Zc_Scan_Start_H;
	Flags0.T3_PENDING = 1;
	EIE1 |= 0x80;			// Enable timer 3 interrupts
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Commutation routines
//
// No assumptions
//
// Performs commutation switching 
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
// Comm phase 1 to comm phase 2
comm1comm2:	
	Set_RPM_Out
	if(Flags3.PGM_DIR_REV) comm12_rev();

	IE_EA = 0;				// Disable all interrupts
	BcomFET_off 				// Turn off comfet
	AcomFET_on				// Turn on comfet
	Set_Pwm_C					// To reapply power after a demag cut
	IE_EA = 1;
	Set_Comp_Phase_B 			// Set comparator phase
	return;

comm12_rev:	
	IE_EA = 0;				// Disable all interrupts
	BcomFET_off 				// Turn off comfet
	CcomFET_on				// Turn on comfet (reverse)
	Set_Pwm_A					// To reapply power after a demag cut
	IE_EA = 1;
	Set_Comp_Phase_B 			// Set comparator phase
	return;


// Comm phase 2 to comm phase 3
comm2comm3:	
	Clear_RPM_Out
	if(Flags3.PGM_DIR_REV) comm23_rev();

	IE_EA = 0;				// Disable all interrupts
	CpwmFET_off				// Turn off pwmfet
	Set_Pwm_B					// To reapply power after a demag cut
	AcomFET_on
	IE_EA = 1;
	Set_Comp_Phase_C 			// Set comparator phase
	return;

comm23_rev:
	IE_EA = 0;				// Disable all interrupts
	ApwmFET_off				// Turn off pwmfet (reverse)
	Set_Pwm_B					// To reapply power after a demag cut
	CcomFET_on
	IE_EA = 1;
	Set_Comp_Phase_A 			// Set comparator phase (reverse)
	return;


// Comm phase 3 to comm phase 4
comm3comm4:	
	Set_RPM_Out
	if(Flags3.PGM_DIR_REV) comm34_rev();

	IE_EA = 0;				// Disable all interrupts
	AcomFET_off 				// Turn off comfet
	CcomFET_on				// Turn on comfet
	Set_Pwm_B					// To reapply power after a demag cut
	IE_EA = 1;
	Set_Comp_Phase_A 			// Set comparator phase
	return;

comm34_rev:	
	IE_EA = 0;				// Disable all interrupts
	CcomFET_off 				// Turn off comfet (reverse)
	AcomFET_on				// Turn on comfet (reverse)
	Set_Pwm_B					// To reapply power after a demag cut
	IE_EA = 1;
	Set_Comp_Phase_C 			// Set comparator phase (reverse)
	return;


// Comm phase 4 to comm phase 5
comm4comm5:	
	Clear_RPM_Out
	if(Flags3.PGM_DIR_REV) comm45_rev();

	IE_EA = 0;				// Disable all interrupts
	BpwmFET_off				// Turn off pwmfet
	Set_Pwm_A					// To reapply power after a demag cut
	CcomFET_on
	IE_EA = 1;
	Set_Comp_Phase_B 			// Set comparator phase
	return;

comm45_rev:
	IE_EA = 0;				// Disable all interrupts
	BpwmFET_off				// Turn off pwmfet
	Set_Pwm_C
	AcomFET_on				// To reapply power after a demag cut
	IE_EA = 1;
	Set_Comp_Phase_B 			// Set comparator phase
	return;


// Comm phase 5 to comm phase 6
comm5comm6:	
	Set_RPM_Out
	if(Flags3.PGM_DIR_REV) comm56_rev();

	IE_EA = 0;				// Disable all interrupts
	CcomFET_off 				// Turn off comfet
	BcomFET_on				// Turn on comfet
	Set_Pwm_A					// To reapply power after a demag cut
	IE_EA = 1;
	Set_Comp_Phase_C 			// Set comparator phase
	return;

comm56_rev:
	IE_EA = 0;				// Disable all interrupts
	AcomFET_off 				// Turn off comfet (reverse)
	BcomFET_on				// Turn on comfet
	Set_Pwm_C					// To reapply power after a demag cut
	IE_EA = 1;
	Set_Comp_Phase_A 			// Set comparator phase (reverse)
	return;


// Comm phase 6 to comm phase 1
comm6comm1:	
	Clear_RPM_Out
	if(Flags3.PGM_DIR_REV) comm61_rev();

	IE_EA = 0;				// Disable all interrupts
	ApwmFET_off				// Turn off pwmfet
	Set_Pwm_C
	BcomFET_on				// To reapply power after a demag cut
	IE_EA = 1;
	Set_Comp_Phase_A 			// Set comparator phase
	return;

comm61_rev:
	IE_EA = 0;				// Disable all interrupts
	CpwmFET_off				// Turn off pwmfet (reverse)
	Set_Pwm_A
	BcomFET_on				// To reapply power after a demag cut
	IE_EA = 1;
	Set_Comp_Phase_C 			// Set comparator phase (reverse)
	return;

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Beeper routines (4 different entry points) 
//
// No assumptions
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
beep_f1:	// Entry point 1, load beeper frequency 1 settings
	Temp3 = 20;	// Off wait loop length
	Temp4 = 120;	// Number of beep pulses
	jmp	beep

beep_f2:	// Entry point 2, load beeper frequency 2 settings
	Temp3 = 16;
	Temp4 = 140;
	jmp	beep

beep_f3:	// Entry point 3, load beeper frequency 3 settings
	Temp3 = 13;
	Temp4 = 180;
	jmp	beep

beep_f4:	// Entry point 4, load beeper frequency 4 settings
	Temp3 = 11;
	Temp4 = 200;
	jmp	beep

beep:	// Beep loop start
	mov	A, Beep_Strength
	djnz	ACC, beep_start
	ret

beep_start:
	Temp2 = 2;
beep_onoff:
	clr	A
	BcomFET_off		// BcomFET off
	djnz	ACC, $		// Allow some time after comfet is turned off
	BpwmFET_on		// BpwmFET on (in order to charge the driver of the BcomFET)
	djnz	ACC, $		// Let the pwmfet be turned on a while
	BpwmFET_off		// BpwmFET off again
	djnz	ACC, $		// Allow some time after pwmfet is turned off
	BcomFET_on		// BcomFET on
	djnz	ACC, $		// Allow some time after comfet is turned on
	// Turn on pwmfet
	mov	A, Temp2
	jb	ACC.0, beep_apwmfet_on
	ApwmFET_on		// ApwmFET on
beep_apwmfet_on:
	jnb	ACC.0, beep_cpwmfet_on
	CpwmFET_on		// CpwmFET on
beep_cpwmfet_on:
	mov	A, Beep_Strength
	djnz	ACC, $		
	// Turn off pwmfet
	mov	A, Temp2
	jb	ACC.0, beep_apwmfet_off
	ApwmFET_off		// ApwmFET off
beep_apwmfet_off:
	jnb	ACC.0, beep_cpwmfet_off
	CpwmFET_off		// CpwmFET off
beep_cpwmfet_off:
	mov	A, #150		// 25祍 off
	djnz	ACC, $		
	djnz	Temp2, beep_onoff
	// Copy variable
	mov	A, Temp3
	mov	Temp1, A	
beep_off:		// Fets off loop
	djnz	ACC, $
	djnz	Temp1,	beep_off
	djnz	Temp4,	beep
	BcomFET_off		// BcomFET off
	ret



//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Decode settings
//
// No assumptions
//
// Decodes various settings
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void decode_settings()
{
	// Load programmed direction
	Flags3.PGM_BIDIR = 1;
	if(Pgm_Direction < 3) Flags3.PGM_BIDIR = 0;
	Flags3.PGM_DIR_REV = 0;
	
	mov	A, @Temp1
	jnb	ACC.1, ($+5)
	if(Pgm_Direction&0x02) Flags3.PGM_DIR_REV = 1;
	Flags3.PGM_BIDIR_REV = Flags3.PGM_DIR_REV;
	// Decode startup power
	Pgm_Startup_Pwr--;
	Pgm_Startup_Pwr_Decoded = STARTUP_POWER_TABLE[Pgm_Startup_Pwr];
	// Decode low rpm power slope
	Low_Rpm_Pwr_Slope = Pgm_Startup_Pwr;
	if(Pgm_Startup_Pwr > 2) Low_Rpm_Pwr_Slope = 2;
	// Decode demag compensation
	mov	Temp1, #Pgm_Demag_Comp		
	mov	A, @Temp1				
	Demag_Pwr_Off_Thresh = 255	// Set default

	if(Pgm_Demag_Comp != 2) decode_demag_high

	Demag_Pwr_Off_Thresh = 160;	// Settings for demag comp low

decode_demag_high:
	if(Pgm_Demag_Comp != 3) decode_demag_done

	Demag_Pwr_Off_Thresh = 130;	// Settings for demag comp high

decode_demag_done:
	// Decode temperature protection limit
	mov	Temp1, #Pgm_Enable_Temp_Prot
	mov	A, @Temp1
	mov	Temp1, A
	if(!Pgm_Enable_Temp_Prot)	decode_temp_done

	mov	A, #(TEMP_LIMIT-TEMP_LIMIT_STEP)
decode_temp_step:
	add	A, #TEMP_LIMIT_STEP
	djnz	Temp1, decode_temp_step

decode_temp_done:
	Temp_Prot_Limit = A;
	// Decode throttle cal
	mov	Temp1, #Pgm_Min_Throttle		// Throttle cal is in 4us units
	mov	A, @Temp1
	call	scale_throttle_cal
	mov	Min_Throttle_L, Temp1
	mov	Min_Throttle_H, Temp2
	mov	Temp1, #Pgm_Center_Throttle	// Throttle cal is in 4us units
	mov	A, @Temp1
	call	scale_throttle_cal
	mov	Center_Throttle_L, Temp1
	mov	Center_Throttle_H, Temp2
	mov	Temp1, #Pgm_Max_Throttle		// Throttle cal is in 4us units
	mov	A, @Temp1
	call	scale_throttle_cal
	mov	Max_Throttle_L, Temp1
	mov	Max_Throttle_H, Temp2
	call	switch_power_off
	ret
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Scale throttle cal
//
// No assumptions
//
// Scales a throttle cal value
// Input is ACC, output is Temp2/Temp1
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void scale_throttle_cal()
{
	mov	Temp3, A
	mov	B, #0Ch			// Calculate "3%" (for going from 1000us to numerical 1024)
	mul	AB
	mov	Temp4, B
	mov	A, Temp3
	clr	C				// Shift to 9 bits
	rlc	A
	mov	Temp1, A
	mov	A, #1
	rlc	A
	mov	Temp2, A
	mov	A, Temp1			// Shift to 10 bits
	clr	C
	rlc	A
	mov	Temp1, A
	mov	A, Temp2
	rlc	A
	mov	Temp2, A
	mov	A, Temp1			// Add "3%"
	clr	C
	add	A, Temp4
	mov	Temp1, A
	mov	A, Temp2
	addc	A, #0
	mov	Temp2, A
#IF MCU_48MHZ == 1
	mov	A, Temp1			// Shift to 11 bits
	clr	C
	rlc	A
	mov	Temp1, A
	mov	A, Temp2
	rlc	A
	mov	Temp2, A
#ENDIF
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Find throttle gains
//
// No assumptions
//
// Finds throttle gains for both directions in bidirectional mode
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void find_throttle_gains()
{
	// Check if full range is chosen
	if(!Flags2.RCP_FULL_RANGE) find_throttle_gains_normal

	Temp3 = 0;		// Min throttle
	Temp4 = 0;
	Temp5 = 255;	// Max throttle
	Temp6 = 0;
	Temp7 = 0;		// Deadband
	call	find_throttle_gain
	Throttle_Gain_M = Temp4;
	Throttle_Gain = Temp3;
	return;

find_throttle_gains_normal:
	// Check if bidirectional operation
	if(!Flags3.PGM_BIDIR) find_throttle_gains_bidir_done

	Temp3 = Pgm_Min_Throttle;
	Temp4 = 0;
	Temp5 = Pgm_Center_Throttle;
	Temp6 = 0;

	Temp3 = Temp3<<1;			// Scale gains in bidirectional
	Temp4 = Temp4<<1;
	Temp5 = Temp5<<1;
	Temp6 = Temp6<<1;
	Temp7 = 10;		// Compensate for deadband in bidirectional
	call	find_throttle_gain
	Throttle_Gain_BD_Rev_M = Temp4;
	Throttle_Gain_BD_Rev = Temp3;

find_throttle_gains_bidir_done:
	if(Flags3.PGM_BIDIR) Temp3 = Pgm_Center_Throttle;
	Temp3 = Pgm_Min_Throttle;

	Temp4 = 0;
	Temp5 = Pgm_Max_Throttle;
	Temp6 = 0;
	Temp7 = 0;			// No deadband
	if(!Flags3.PGM_BIDIR) find_throttle_gain_fwd

	Temp3 = Temp3<<1;// Scale gains in bidirectional
	Temp4 = Temp4<<1;
	Temp5 = Temp5<<1;
	Temp6 = Temp6<<1;

	Temp7 = 10;		// Compensate for deadband in bidirectional

find_throttle_gain_fwd:
	call	find_throttle_gain
	Throttle_Gain_M = Temp4;
	Throttle_Gain = Temp3;
	return;
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Find throttle gain
//
// The difference between max and min throttle must be more than 140us (a Pgm_xxx_Throttle difference of 35)
// Temp4/3 holds min throttle, Temp6/5 holds max throttle, Temp7 holds deadband, Temp4/Temp3 gives resulting gain
//
// Finds throttle gain from throttle calibration values
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void find_throttle_gain()
{
	// Subtract deadband from max
	Temp5 = Temp5 - Temp7;
	Temp6 = Temp6 - 0;
	// Calculate difference
	Temp5 = Temp5 - Temp3;
	Temp6 = Temp6 - Temp4;
	// Check that difference is minimum 35
	if(Temp5 <= 35 || Temp6 <= 0)
	{Temp5 = 35;
	Temp6 = 0;}

	// Check that difference is maximum 511
	if(Temp5 >= 255 || Temp6 >= 1)
	{Temp5 = 255;
	Temp6 = 1;}

	// Find gain
	Temp4 = 0xFF;
find_throttle_gain_loop:
	Temp4++;
	Temp3 = 0;
test_throttle_gain:
	Temp3++;
	if(Temp3)	test_throttle_gain_mult

	Temp5 = Temp5<<1;			// Set multiplier x2 and range /2
	Temp6 = Temp6<<1;
	ajmp	find_throttle_gain_loop

test_throttle_gain_mult:
	Temp7 = Temp5*Temp3;// A has difference, B has gain
	mov	A, Temp6
	mov	B, Temp3
	mul	AB
	if(Temp7 + Temp6*Temp3 < 124) test_throttle_gain

	if(!(~Temp3))	find_throttle_gain_loop

	return;
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Average throttle 
//
// Outputs result in Temp8
//
// Averages throttle calibration readings
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void average_throttle()
{
	Flags2.RCP_FULL_RANGE = 1;	// Set range to 1000-2020us
	call find_throttle_gains	// Set throttle gains
	call wait30ms		
	call wait30ms
	Temp3 = 0;
	Temp4 = 0;
	Temp5 = 16;		// Average 16 measurments
average_throttle_meas:
	call wait3ms			// Wait for new RC pulse value
	Temp3 =Temp3 + New_Rcp;		// Get new RC pulse value
	Temp4 =Temp4 + 0;
	if(Temp5--) average_throttle_meas

	Temp5 = 4;			// Shift 4 times
average_throttle_div:
	clr	C
	Temp4 = Temp4>>1;   		// Shift right 
	Temp3 = Temp3>>1;
	if(Temp5--) average_throttle_div

	Temp8 = Temp3;   		// Copy to Temp8
	if(Temp4)
	Temp8 = 0xFF;

	Flags2.RCP_FULL_RANGE = 0;
	call find_throttle_gains	// Set throttle gains
	return;
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// LED control
//
// No assumptions
//
// Controls LEDs
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void led_control()
{
	Temp2 = Pgm_LED_Control;
	if(Pgm_LED_Control&0x03) led_0_done
	Clear_LED_0
led_0_done:
	Set_LED_1
	if(Pgm_LED_Control&0x0C) led_1_done
	Clear_LED_1
led_1_done:
	Set_LED_2
	if(Pgm_LED_Control&0x30) led_2_done
	Clear_LED_2
led_2_done:
	Set_LED_3
	if(Pgm_LED_Control&0xC0) led_3_done
	Clear_LED_3
led_3_done:
	return;
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//**** **** **** **** **** **** **** **** **** **** **** **** ****
//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Main program start
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
//**** **** **** **** **** **** **** **** **** **** **** **** ****
//**** **** **** **** **** **** **** **** **** **** **** **** ****

void pgm_start()
{
	// Initialize flash keys to invalid values
	Flash_Key_1 = 0;
	Flash_Key_2 = 0;
	// Disable the WDT.
	WDTCN = 0xDE;		// Disable watchdog
	WDTCN = 0xAD;		
	// Initialize stack
	SP = 0xC0;			// Stack = 64 upper bytes of RAM
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
	Beep_Strength = Pgm_Beep_Strength;
	// Set initial arm variable
	Initial_Arm = 1;
	// Initializing beep
	IE_EA = 0;			// Disable interrupts explicitly
	call wait200ms	
	call beep_f1
	call wait30ms
	call beep_f2
	call wait30ms
	call beep_f3
	call wait30ms
	call led_control
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// No signal entry point
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
init_no_signal()
{
	// Disable interrupts explicitly
	IE_EA = 0;
	// Initialize flash keys to invalid values
	Flash_Key_1 = 0;
	Flash_Key_2 = 0;
	// Check if input signal is high for more than 15ms
	Temp1 = 250;
input_high_check_1:
	Temp2 = 250;
input_high_check_2:
	if(!RTX_PORT.RTX_PIN) bootloader_done	// Look for low
	djnz	Temp2, input_high_check_2
	djnz	Temp1, input_high_check_1

	ljmp	1C00h			// Jump to bootloader

bootloader_done:
	// Decode settings
	call	decode_settings
	// Find throttle gain from stored min and max settings
	call	find_throttle_gains
	// Set beep strength
	Beep_Strength = Pgm_Beep_Strength;
	// Switch power off
	call	switch_power_off
	// Set clock frequency
#IF MCU_48MHZ == 1
	Set_MCU_Clk_24MHz
#ENDIF
	// Setup timers for pwm input
	IT01CF = RTX_PIN;	// Route RCP input to INT0
	TCON = 0x11;		// Timer 0 run and INT0 edge triggered
	CKCON0 = 0x04;		// Timer 0 clock is system clock
	TMOD = 0x09;		// Timer 0 set to 16bits and gated by INT0
	TMR2CN0 = 0x04;		// Timer 2 enabled
	TMR3CN0 = 0x04;		// Timer 3 enabled
	Initialize_PCA			// Initialize PCA
	Set_Pwm_Polarity		// Set pwm polarity
	Enable_Power_Pwm_Module	// Enable power pwm module
	Enable_Damp_Pwm_Module	// Enable damping pwm module
	// Enable interrupts
#IF MCU_48MHZ == 0
	IE = 0x21;			// Enable timer 2 interrupts and INT0 interrupts
#ELSE
	IE = 0x23;			// Enable timer 0, timer 2 interrupts and INT0 interrupts
#ENDIF
	EIE1 = 0x90;		// Enable timer 3 and PCA0 interrupts
	IP = 0x01;			// High priority to INT0 interrupts
	// Initialize comparator
	Initialize_Comparator	// Initialize comparator
	// Initialize ADC
	Initialize_Adc			// Initialize ADC operation
	call	wait1ms
	IE_EA = 1;			// Enable all interrupts
	// Reset stall count
	Stall_Cnt = 0;
	// Initialize RC pulse
	Flags2.RCP_UPDATED = 0;		 	// Clear updated flag
	call wait200ms
	// Clear all shot flags
	Flags2.RCP_ONESHOT125 = 0;			// Clear OneShot125 flag
	Flags2.RCP_ONESHOT42 = 0;			// Clear OneShot42 flag
	Flags2.RCP_MULTISHOT = 0;			// Clear Multishot flag
	Flags2.RCP_DSHOT = 0;				// Clear DShot flag
	Dshot_Cmd = 0;					// Clear Dshot command
	Dshot_Cmd_Cnt = 0;				// Clear Dshot command count
	// Test whether signal is regular pwm
	Rcp_Outside_Range_Cnt = 0;		// Reset out of range counter
	call wait100ms						// Wait for new RC pulse
	if(Rcp_Outside_Range_Cnt < 10)			// Check how many pulses were outside normal range ("900-2235us")
	ajmp	validate_rcp_start

	// Test whether signal is OneShot125
	Flags2.RCP_ONESHOT125 = 1;			// Set OneShot125 flag
	Rcp_Outside_Range_Cnt = 0;		// Reset out of range counter
	call wait100ms						// Wait for new RC pulse

	if(Rcp_Outside_Range_Cnt < 10)			// Check how many pulses were outside normal range ("900-2235us")
	ajmp	validate_rcp_start

	// Test whether signal is OneShot42
	Flags2.RCP_ONESHOT125 = 0;
	Flags2.RCP_ONESHOT42 = 1;			// Set OneShot42 flag
	Rcp_Outside_Range_Cnt = 0;		// Reset out of range counter
	call wait100ms						// Wait for new RC pulse
	clr	C
	if(Rcp_Outside_Range_Cnt < 10)			// Check how many pulses were outside normal range ("900-2235us")
	ajmp	validate_rcp_start

	// Setup timers for DShot
	IT01CF = 0x80+(RTX_PIN << 4)+(RTX_PIN);	// Route RCP input to INT0/1, with INT1 inverted
	TCON = 0x51;		// Timer 0/1 run and INT0 edge triggered
	CKCON0 = 0x01;		// Timer 0/1 clock is system clock divided by 4 (for DShot150)
	TMOD = 0xAA;		// Timer 0/1 set to 8bits auto reload and gated by INT0
	TH0 = 0;			// Auto reload value zero
	TH1 = 0;
	// Setup interrupts for DShot
	IE_ET0 = 0;			// Disable timer 0 interrupts
	IE_ET1 = 1;			// Enable timer 1 interrupts
	IE_EX1 = 1;			// Enable int1 interrupts
	// Setup variables for DSshot150
#IF MCU_48MHZ == 1
	DShot_Timer_Preset = 128;			// Load DShot sync timer preset (for DShot150)
#ELSE
	DShot_Timer_Preset = 192;
#ENDIF
	DShot_Pwm_Thr = 0x20;				// Load DShot qualification pwm threshold (for DShot150)
	DShot_Frame_Length_Thr = 0x80;		// Load DShot frame length criteria
	// Test whether signal is DShot150
	Flags2.RCP_ONESHOT42 = 0;
	Flags2.RCP_DSHOT = 1;
	Rcp_Outside_Range_Cnt = 10;		// Set out of range counter
	call wait100ms						// Wait for new RC pulse
	DShot_Pwm_Thr = 16;				// Load DShot regular pwm threshold

	Dshot_Cmd = 0;
	Dshot_Cmd_Cnt = 0;
	if(Rcp_Outside_Range_Cnt < 10)			// Check if pulses were accepted
	validate_rcp_start

	// Setup variables for DShot300
	CKCON0 = 0x0C;					// Timer 0/1 clock is system clock (for DShot300)
#IF MCU_48MHZ == 1
	DShot_Timer_Preset = 0;			// Load DShot sync timer preset (for DShot300)
#ELSE
	DShot_Timer_Preset = 128;
#ENDIF
	DShot_Pwm_Thr = 40;				// Load DShot qualification pwm threshold (for DShot300)
	DShot_Frame_Length_Thr = 40;		// Load DShot frame length criteria
	// Test whether signal is DShot300
	Rcp_Outside_Range_Cnt = 10;		// Set out of range counter
	call wait100ms						// Wait for new RC pulse
	DShot_Pwm_Thr = 32;				// Load DShot regular pwm threshold

	Dshot_Cmd = 0;
	Dshot_Cmd_Cnt = 0;
	if(Rcp_Outside_Range_Cnt < 10)// Check if pulses were accepted
	validate_rcp_start

	// Setup variables for DShot600
	CKCON0 = 0x0C;					// Timer 0/1 clock is system clock (for DShot600)
#IF MCU_48MHZ == 1
	DShot_Timer_Preset = 128;			// Load DShot sync timer preset (for DShot600)
#ELSE
	DShot_Timer_Preset = 192;
#ENDIF
	DShot_Pwm_Thr = 20;				// Load DShot qualification pwm threshold (for DShot600)
	DShot_Frame_Length_Thr = 20;		// Load DShot frame length criteria
	// Test whether signal is DShot600
	Rcp_Outside_Range_Cnt = 10;		// Set out of range counter
	call wait100ms						// Wait for new RC pulse
	DShot_Pwm_Thr = 16;				// Load DShot regular pwm threshold

	Dshot_Cmd = 0;
	Dshot_Cmd_Cnt = 0;
	if(Rcp_Outside_Range_Cnt < 10)// Check if pulses were accepted
	validate_rcp_start

	// Setup timers for Multishot
	IT01CF = RTX_PIN;	// Route RCP input to INT0
	TCON = 0x11;		// Timer 0 run and INT0 edge triggered
	CKCON0 = 0x04;		// Timer 0 clock is system clock
	TMOD = 0x09;		// Timer 0 set to 16bits and gated by INT0
	// Setup interrupts for Multishot
	IE_ET0 = 1;			// Enable timer 0 interrupts
	IE_ET1 = 0;			// Disable timer 1 interrupts
	IE_EX1 = 0;			// Disable int1 interrupts
	// Test whether signal is Multishot
	Flags2.RCP_DSHOT = 0;
	Flags2.RCP_MULTISHOT = 1;			// Set Multishot flag
	Rcp_Outside_Range_Cnt = 0;		// Reset out of range counter
	call wait100ms						// Wait for new RC pulse

	if(Rcp_Outside_Range_Cnt < 10)			// Check how many pulses were outside normal range ("900-2235us")
	validate_rcp_start

	ajmp	init_no_signal

validate_rcp_start:
	// Validate RC pulse
	call wait3ms						// Wait for new RC pulse
	if(!Flags2.RCP_UPDATED)		// Is there an updated RC pulse available - proceed
	ljmp	init_no_signal					// Go back to detect input signal

	// Beep arm sequence start signal
	IE_EA = 0;						// Disable all interrupts
	call beep_f1					// Signal that RC pulse is ready
	call beep_f1
	call beep_f1
	IE_EA = 1;						// Enable all interrupts
	call wait200ms	

	// Arming sequence start
arming_start:
	if(!Flags2.RCP_DSHOT)	// Disable tx programming for DShot
	if(Flags3.PGM_BIDIR)

	ljmp	program_by_tx_checked	// jb Disable tx programming if bidirectional operation

	call wait3ms//jnb
	// Start programming mode entry if enabled
	if(Pgm_Enable_TX_Program>1)				// Is TX programming enabled?
	arming_initial_arm_check	// Yes - proceed

	jmp	program_by_tx_checked	// No - branch

arming_initial_arm_check:
	// Yes - check if it is initial arm sequence
	if(Initial_Arm >= 1)				// Is it the initial arm sequence?
		arming_check			// Yes - proceed

	jmp 	program_by_tx_checked	// No - branch

arming_check:
	// Initialize flash keys to valid values
	Flash_Key_1 = 0xA5;
	Flash_Key_2 = 0xF1;
	// Throttle calibration and tx program entry
	Temp8 = 2;				// Set 1 seconds wait time
throttle_high_cal:			
	Flags2.RCP_FULL_RANGE = 1;	// Set range to 1000-2020us
	call find_throttle_gains		// Set throttle gains
	call wait100ms				// Wait for new throttle value
	IE_EA = 0;				// Disable interrupts (freeze New_Rcp value)
	Flags2.RCP_FULL_RANGE = 0;	// Set programmed range
	call find_throttle_gains		// Set throttle gains

	// Load new RC pulse value
	// Is RC pulse above midstick?
	IE_EA = 1;				// Enable interrupts
	if(New_Rcp<255/2)	program_by_tx_checked	// No - branch

	call wait1ms		
	IE_EA = 0;				// Disable all interrupts
	call beep_f4
	IE_EA = 1;				// Enable all interrupts
	djnz	Temp8, throttle_high_cal	// Continue to wait

	call	average_throttle
	Pgm_Max_Throttle = Temp8;	// Store
	call wait200ms				
	call	success_beep

throttle_low_cal_start:
	Temp8 = 10;			// Set 3 seconds wait time
throttle_low_cal:			
	Flags2.RCP_FULL_RANGE = 1;	// Set range to 1000-2020us
	call find_throttle_gains		// Set throttle gains
	call wait100ms
	IE_EA = 0;				// Disable interrupts (freeze New_Rcp value)
	Flags2.RCP_FULL_RANGE = 0;	// Set programmed range
	call find_throttle_gains		// Set throttle gains

	// Load new RC pulse value
	// Below midstick?
	IE_EA = 1;				// Enable interrupts
	if(New_Rcp>255/2)	throttle_low_cal_start	// No - start over

	call wait1ms		
	IE_EA = 0;				// Disable all interrupts
	call beep_f1
	call wait10ms
	call beep_f1
	IE_EA = 1;				// Enable all interrupts
	djnz	Temp8, throttle_low_cal	// Continue to wait

	call	average_throttle

	// Add about 1%
	Pgm_Min_Throttle = Temp8 + 3;	// Store

	Temp1 = Temp8 + 3;				// Min throttle in Temp1

	// Subtract 35 (140us) from max throttle
	if(Pgm_Max_Throttle < 35)	program_by_tx_entry_limit
	// Subtract min from max
	if(Pgm_Max_Throttle > Temp1)	program_by_tx_entry_store

program_by_tx_entry_limit:
	// Load min
	// Make max 140us higher than min
	Pgm_Max_Throttle = Temp1 + 35;	// Store new max

program_by_tx_entry_store:
	call wait200ms				
	call erase_and_store_all_in_eeprom	
	call success_beep_inverted

program_by_tx_entry_wait:
	call wait100ms
	call find_throttle_gains		// Set throttle gains
	ljmp init_no_signal			// Go back

program_by_tx_checked:
	// Initialize flash keys to invalid values
	Flash_Key_1 = 0;
	Flash_Key_2 = 0;
	call wait100ms				// Wait for new throttle value

	// Load new RC pulse value
	// Below stop?
	if(New_Rcp < 1)	arm_end_beep			// Yes - proceed

	jmp	arming_start			// No - start over

arm_end_beep:
	// Beep arm sequence end signal
	IE_EA = 0;				// Disable all interrupts
	call beep_f4				// Signal that rcpulse is ready
	call beep_f4
	call beep_f4
	IE_EA = 1;				// Enable all interrupts
	call wait200ms

	// Clear initial arm variable
	Initial_Arm = 0;

	// Armed and waiting for power on
wait_for_power_on:
	Power_On_Wait_Cnt_L = 0;	// Clear wait counter
	Power_On_Wait_Cnt_H = 0;	
wait_for_power_on_loop:
	Power_On_Wait_Cnt_L++;		// Increment low wait counter

	if(~Power_On_Wait_Cnt_L) wait_for_power_on_no_beep// Counter wrapping (about 3 sec)

	Power_On_Wait_Cnt_H++;		// Increment high wait counter
	mov	Temp1, #Pgm_Beacon_Delay
	mov	A, @Temp1
	mov	Temp1, #25		// Approximately 1 min
	dec	A
	jz	beep_delay_set

	mov	Temp1, #50		// Approximately 2 min
	dec	A
	jz	beep_delay_set

	mov	Temp1, #125		// Approximately 5 min
	dec	A
	jz	beep_delay_set

	mov	Temp1, #250		// Approximately 10 min
	dec	A
	jz	beep_delay_set

	Power_On_Wait_Cnt_H = 0;		// Reset counter for infinite delay

beep_delay_set:
	// Check against chosen delay
	if(Power_On_Wait_Cnt_H < Temp1)	wait_for_power_on_no_beep// Has delay elapsed?

	call switch_power_off		// Switch power off in case braking is set
	call wait1ms
	Power_On_Wait_Cnt_H--;		// Decrement high wait counter
	Power_On_Wait_Cnt_L = 0;	// Set low wait counter
	Beep_Strength = Pgm_Beacon_Strength;
	IE_EA = 0;				// Disable all interrupts
	call beep_f4			// Signal that there is no signal
	IE_EA = 1;				// Enable all interrupts
	mov	Beep_Strength = Pgm_Beep_Strength;
	call wait100ms				// Wait for new RC pulse to be measured

wait_for_power_on_no_beep:
	call wait10ms
	// Load RC pulse timeout counter value
	if(Rcp_Timeout_Cntd)wait_for_power_on_not_missing	// If it is not zero - proceed

	jmp	init_no_signal				// If pulses missing - go back to detect input signal

wait_for_power_on_not_missing:
	// Load new RC pulse value
	// Higher than stop
	if(New_Rcp >= 1)	wait_for_power_on_nonzero	// Yes - proceed

	// 1 or higher
	if(Dshot_Cmd >= 1)	check_dshot_cmd		// Check Dshot command

	ljmp wait_for_power_on_loop	// If not Dshot command - start over

wait_for_power_on_nonzero:
	lcall wait100ms			// Wait to see if start pulse was only a glitch
	// Load RC pulse timeout counter value
	// If it is not zero - proceed
	if(Rcp_Timeout_Cntd) init_no_signal			// If it is zero (pulses missing) - go back to detect input signal

	Dshot_Cmd = 0;
	Dshot_Cmd_Cnt = 0;
	ljmp init_start

check_dshot_cmd:
	if(Dshot_Cmd != 1) dshot_beep_2

	IE_EA = 0;
	call switch_power_off		// Switch power off in case braking is set
	Beep_Strength = Pgm_Beacon_Strength;
	call beep_f1

	Beep_Strength = Pgm_Beep_Strength;
	IE_EA = 1;
	call wait100ms
	jmp clear_dshot_cmd

dshot_beep_2:
	if(Dshot_Cmd != 2) dshot_beep_3

	IE_EA = 0;
	call switch_power_off		// Switch power off in case braking is set
	Beep_Strength = Pgm_Beacon_Strength;
	call beep_f2
	Beep_Strength = Pgm_Beep_Strength;
	IE_EA = 1;
	call wait100ms	
	jmp clear_dshot_cmd

dshot_beep_3:
	if(Dshot_Cmd != 3) dshot_beep_4

	IE_EA = 0;
	call switch_power_off		// Switch power off in case braking is set
	Beep_Strength = Pgm_Beacon_Strength;
	call beep_f3
	Beep_Strength = Pgm_Beep_Strength;
	IE_EA = 1;
	call wait100ms	
	jmp clear_dshot_cmd

dshot_beep_4:
	if(Dshot_Cmd != 4) dshot_beep_5

	IE_EA = 0;
	call switch_power_off		// Switch power off in case braking is set
	Beep_Strength = Pgm_Beacon_Strength;
	call beep_f4
	Beep_Strength = Pgm_Beep_Strength;
	IE_EA = 1;
	call wait100ms		
	jmp clear_dshot_cmd

dshot_beep_5:
	if(Dshot_Cmd != 5) dshot_direction_1

	IE_EA = 0;
	call switch_power_off		// Switch power off in case braking is set
	Beep_Strength = Pgm_Beacon_Strength;
	call beep_f4
	Beep_Strength = Pgm_Beep_Strength;
	IE_EA = 1;
	call wait100ms	
	jmp clear_dshot_cmd

dshot_direction_1:
	if(Dshot_Cmd != 7) dshot_direction_2

	// Needs to receive it 6 times in a row
	// Same as "jc dont_clear_dshot_cmd"
	if(Dshot_Cmd_Cnt<6) wait_for_power_on_not_missing

	if(Flags3.PGM_BIDIR) Pgm_Direction = 1;
	Pgm_Direction = 3;
	Flags3.PGM_DIR_REV = 0;
	Flags3.PGM_BIDIR_REV = 0;
	jmp clear_dshot_cmd

dshot_direction_2:
	if(Dshot_Cmd != 8) dshot_direction_bidir_off

	// Needs to receive it 6 times in a row
	// Same as "jc dont_clear_dshot_cmd"
	if(Dshot_Cmd_Cnt<6) wait_for_power_on_not_missing

	if(Flags3.PGM_BIDIR) Pgm_Direction = 2;
	Pgm_Direction = 4;
	Flags3.PGM_DIR_REV = 1;
	Flags3.PGM_BIDIR_REV = 1;
	jmp clear_dshot_cmd

dshot_direction_bidir_off:
	if(Dshot_Cmd != 9) dshot_direction_bidir_on

	// Needs to receive it 6 times in a row
	// Same as "jc dont_clear_dshot_cmd"
	if(Dshot_Cmd_Cnt<6) wait_for_power_on_not_missing

	if(!Flags3.PGM_BIDIR) dshot_direction_bidir_on

	Pgm_Direction = Pgm_Direction - 2;
	Flags3.PGM_BIDIR = 0;
	jmp clear_dshot_cmd

dshot_direction_bidir_on:
	if(Dshot_Cmd != 10) dshot_direction_normal

	// Needs to receive it 6 times in a row
	// Same as "jc dont_clear_dshot_cmd"
	if(Dshot_Cmd_Cnt<6) wait_for_power_on_not_missing

	if(Flags3.PGM_BIDIR) dshot_direction_normal

	Pgm_Direction = Pgm_Direction + 2;
	Flags3.PGM_BIDIR = 1;
	jmp clear_dshot_cmd

dshot_direction_normal: 
	if(Dshot_Cmd != 20) dshot_direction_reverse

	// Needs to receive it 6 times in a row
	// Same as "jc dont_clear_dshot_cmd"
	if(Dshot_Cmd_Cnt<6) wait_for_power_on_not_missing

	IE_EA = 0;					// DPTR used in interrupts
	mov	DPTR, #Eep_Pgm_Direction		// Read from flash
	mov	A, #0
	movc	A, @A+DPTR
	IE_EA = 1;
	Pgm_Direction = Eep_Pgm_Direction;
	
	//rrc	A // Lsb to carry
	Flags3.PGM_DIR_REV = 0;
	Flags3.PGM_BIDIR_REV = 0;
	if(!(Eep_Pgm_Direction&0x01)) Flags3.PGM_DIR_REV = 1;
	if(!(Eep_Pgm_Direction&0x01)) Flags3.PGM_BIDIR_REV = 1;
	jmp clear_dshot_cmd

dshot_direction_reverse: 			// Temporary reverse
	if(Dshot_Cmd != 21) dshot_save_settings

	// Needs to receive it 6 times in a row
	if(Dshot_Cmd_Cnt<6) dont_clear_dshot_cmd
	
	IE_EA = 0;					// DPTR used in interrupts
	// Read from flash
	IE_EA = 1;
	if(Eep_Pgm_Direction == 1)
	Pgm_Direction = 2;
	if(Eep_Pgm_Direction == 2)
	Pgm_Direction = 1;
	if(Eep_Pgm_Direction == 3)
	Pgm_Direction = 4;
	if(Eep_Pgm_Direction == 4)
	Pgm_Direction = 3;

	//rrc	A	// Lsb to carry
	Flags3.PGM_DIR_REV = 0;
	Flags3.PGM_BIDIR_REV = 0;
	if(!(Eep_Pgm_Direction&0x01)) Flags3.PGM_DIR_REV = 1;
	if(!(Eep_Pgm_Direction&0x01)) Flags3.PGM_BIDIR_REV = 1;
	jmp clear_dshot_cmd

dshot_save_settings:
	if(Dshot_Cmd != 12) clear_dshot_cmd

	Flash_Key_1 = 0xA5;			// Initialize flash keys to valid values
	Flash_Key_2 = 0xF1;
	// Needs to receive it 6 times in a row
	if(Dshot_Cmd_Cnt<6) dont_clear_dshot_cmd

	call erase_and_store_all_in_eeprom
	IE_EA = 1;
	
clear_dshot_cmd:
	Dshot_Cmd = 0;
	Dshot_Cmd_Cnt = 0;

dont_clear_dshot_cmd:
	Flash_Key_1 = 0;			// Initialize flash keys to invalid values
	Flash_Key_2 = 0;
	jmp wait_for_power_on_not_missing
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Start entry point
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
init_start()
{
	IE_EA = 0;
	call switch_power_off
	IE_EA = 1;
	Adc_Conversion_Cnt = 0;
	Flags0 = 0;				// Clear flags0
	Flags1 = 0;				// Clear flags1
	Demag_Detected_Metric = 0;	// Clear demag metric
	//**** **** **** **** ****
	// Motor start beginning
	//**** **** **** **** **** 
	Adc_Conversion_Cnt = 8;				// Make sure a temp reading is done
	call wait1ms
	call start_adc_conversion
read_initial_temp:
	while(!ADC0CN0_ADINT);
	Read_Adc_Result						// Read initial temperature
	if(!Temp2)							// Is reading below 256?
	// Yes - set average temperature value to zero
	Current_Average_Temp = ADC0H;
	Current_Average_Temp = ADC0L;			// Set initial average temperature
	call check_temp_voltage_and_limit_power
	Adc_Conversion_Cnt = 8;				// Make sure a temp reading is done next time
	// Set up start operating conditions
	IE_EA = 0				// Disable interrupts
	call set_startup_pwm
	Pwm_Limit = Pwm_Limit_Beg;
	Pwm_Limit_By_Rpm = Pwm_Limit_Beg;
	IE_EA = 1;
	// Begin startup sequence
#IF MCU_48MHZ == 1
	Set_MCU_Clk_48MHz
#ENDIF
	if(!Flags3.PGM_BIDIR) init_start_bidir_done	// Check if bidirectional operation

	Flags3.PGM_DIR_REV = 0;			// Set spinning direction. Default fwd
	if(Flags2.RCP_DIR_REV)	// Check force direction
	Flags3.PGM_DIR_REV = 1;			// Set spinning direction

init_start_bidir_done:
	Flags1.STARTUP_PHASE = 1;		// Set startup phase flag
	Startup_Cnt = 0;			// Reset counter
	call comm5comm6				// Initialize commutation
	call comm6comm1
	call initialize_timing			// Initialize timing
	call calc_next_comm_timing		// Set virtual commutation point
	call initialize_timing			//Initialize timing
	call calc_next_comm_timing		
	call initialize_timing			// Initialize timing
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Run entry point
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****

// Run 1 = B(p-on) + C(n-pwm) - comparator A evaluated
// Out_cA changes from low to high
run1()
	call wait_for_comp_out_high	// Wait for high
//		setup_comm_wait		// Setup wait time from zero cross to commutation
//		evaluate_comparator_integrity	// Check whether comparator reading has been normal
	call wait_for_comm			// Wait from zero cross to commutation
	call comm1comm2			// Commutate
	call calc_next_comm_timing	// Calculate next timing and wait advance timing wait
//		wait_advance_timing		// Wait advance timing and start zero cross wait
//		calc_new_wait_times
//		wait_before_zc_scan		// Wait zero cross wait and start zero cross timeout

// Run 2 = A(p-on) + C(n-pwm) - comparator B evaluated
// Out_cB changes from high to low
run2()
	call wait_for_comp_out_low
//		setup_comm_wait
//		evaluate_comparator_integrity
	if(!Flags1.HIGH_RPM)	// Skip if high rpm
	lcall set_pwm_limit_low_rpm
	if(Flags1.HIGH_RPM)	// Do if high rpm
	lcall set_pwm_limit_high_rpm
	call wait_for_comm
	call comm2comm3
	call calc_next_comm_timing
//		wait_advance_timing
//		calc_new_wait_times
//		wait_before_zc_scan

// Run 3 = A(p-on) + B(n-pwm) - comparator C evaluated
// Out_cC changes from low to high
run3:
	call wait_for_comp_out_high
//		setup_comm_wait
//		evaluate_comparator_integrity
	call wait_for_comm
	call comm3comm4
	call calc_next_comm_timing
//		wait_advance_timing
//		calc_new_wait_times
//		wait_before_zc_scan

// Run 4 = C(p-on) + B(n-pwm) - comparator A evaluated
// Out_cA changes from high to low
run4:
	call wait_for_comp_out_low
//		setup_comm_wait
//		evaluate_comparator_integrity
	call wait_for_comm
	call comm4comm5
	call calc_next_comm_timing
//		wait_advance_timing
//		calc_new_wait_times
//		wait_before_zc_scan

// Run 5 = C(p-on) + A(n-pwm) - comparator B evaluated
// Out_cB changes from low to high
run5:
	call wait_for_comp_out_high
//		setup_comm_wait
//		evaluate_comparator_integrity
	call wait_for_comm
	call comm5comm6
	call calc_next_comm_timing
//		wait_advance_timing
//		calc_new_wait_times
//		wait_before_zc_scan

// Run 6 = B(p-on) + A(n-pwm) - comparator C evaluated
// Out_cC changes from high to low
run6:
	call start_adc_conversion
	call wait_for_comp_out_low
//		setup_comm_wait
//		evaluate_comparator_integrity
	call wait_for_comm
	call comm6comm1
	call check_temp_voltage_and_limit_power
	call calc_next_comm_timing
//		wait_advance_timing
//		calc_new_wait_times
//		wait_before_zc_scan

	// Check if it is direct startup
	if(!Flags1.STARTUP_PHASE) normal_run_checks

	// Set spoolup power variables
	Pwm_Limit = Pwm_Limit_Beg;		// Set initial max power
	// Check startup counter
	Temp2 = 24;				// Set nominal startup parameters
	Temp3 = 12;
	// Load counter
	// Is counter above requirement?
	if(Startup_Cnt < Temp2)	direct_start_check_rcp		// No - proceed

	Flags1.STARTUP_PHASE = 0;		// Clear startup phase flag
	Flags1.INITIAL_RUN_PHASE = 1;		// Set initial run phase flag
	Initial_Run_Rot_Cntd = Temp3;	// Set initial run rotation count
	Pwm_Limit = Pwm_Limit_Beg;
	Pwm_Limit_By_Rpm = Pwm_Limit_Beg;
	jmp	normal_run_checks

direct_start_check_rcp:
	// Load new pulse value
	// Check if pulse is below stop value
	if(New_Rcp >= 1)
	ljmp	run1						// Continue to run 
	jmp	run_to_wait_for_power_on

normal_run_checks:
	// Check if it is initial run phase
	if(!Flags1.INITIAL_RUN_PHASE) initial_run_phase_done	// If not initial run phase - branch
	if(Flags1.DIR_CHANGE_BRAKE) initial_run_phase_done	// If a direction change - branch

	// Decrement startup rotaton count
	Initial_Run_Rot_Cntd--;
	// Check number of initial rotations
	if(Initial_Run_Rot_Cntd) initial_run_check_startup_rot	// Branch if counter is not zero

	Flags1.INITIAL_RUN_PHASE = 0;		// Clear initial run phase flag
	Flags1.MOTOR_STARTED = 1;		// Set motor started
	jmp run1						// Continue with normal run

initial_run_check_startup_rot:
	// Not zero - store counter

	if(Flags3.PGM_BIDIR) initial_run_continue_run	// Check if bidirectional operation

	// Load new pulse value
	// Check if pulse is below stop value
	if(New_Rcp >= 1)

initial_run_continue_run:
	ljmp	run1						// Continue to run 

	jmp	run_to_wait_for_power_on

initial_run_phase_done:
	// Reset stall count
	Stall_Cnt = 0;
	// Exit run loop after a given time
	if(Flags3.PGM_BIDIR) run6_check_timeout	// Check if bidirectional operation

	Temp1 = 250;
	Temp2 = Pgm_Brake_On_Stop;
	if(Temp2) Temp1 = 3;					// About 100ms before stopping when brake is set

	// Load stop RC pulse counter low byte value
	// Is number of stop RC pulses above limit?
	if(Rcp_Stop_Cnt >= Temp1)	run_to_wait_for_power_on		// Yes, go back to wait for poweron

run6_check_timeout:
	// Load RC pulse timeout counter value
	if(!Rcp_Timeout_Cntd)	run_to_wait_for_power_on		// If it is zero - go back to wait for poweron

run6_check_dir:
	if(!Flags3.PGM_BIDIR) run6_check_speed		// Check if bidirectional operation

	if(Flags3.PGM_DIR_REV) run6_check_dir_rev		// Check if actual rotation direction
	if(Flags2.RCP_DIR_REV) run6_check_dir_change	// Matches force direction
	jmp	run6_check_speed

run6_check_dir_rev:
	if(!Flags2.RCP_DIR_REV) run6_check_dir_change
	jmp	run6_check_speed

run6_check_dir_change:
	if(Flags1.DIR_CHANGE_BRAKE) run6_check_speed

	Flags1.DIR_CHANGE_BRAKE = 1;		// Set brake flag
	Pwm_Limit = Pwm_Limit_Beg;		// Set max power while braking
	jmp	run4						// Go back to run 4, thereby changing force direction

run6_check_speed:
	Temp1 = 0xF0;				// Default minimum speed
	if(!Flags1.DIR_CHANGE_BRAKE) run6_brake_done// Is it a direction change?

	Pwm_Limit = Pwm_Limit_Beg; 	// Set max power while braking
	Temp1 = 0x20; 				// Bidirectional braking termination speed

run6_brake_done:
	clr	C
	mov	A, Comm_Period4x_H			// Is Comm_Period4x more than 32ms (~1220 eRPM)?
	subb	A, Temp1
	jnc	($+5)					// Yes - stop or turn direction 
	if(Comm_Period4x_H <= Temp1)run1						// No - go back to run 1

	if(!Flags1.DIR_CHANGE_BRAKE) run_to_wait_for_power_on	// If it is not a direction change - stop

	Flags1.DIR_CHANGE_BRAKE = 0;		// Clear brake flag
	Flags3.PGM_DIR_REV = 0;			// Set spinning direction. Default fwd
	if(Flags2.RCP_DIR_REV)	// Check force direction
	Flags3.PGM_DIR_REV = 1;			// Set spinning direction
	Flags1.INITIAL_RUN_PHASE = 1;
	Initial_Run_Rot_Cntd = 18;
	Pwm_Limit = Pwm_Limit_Beg;		// Set initial max power
	jmp	run1						// Go back to run 1 

run_to_wait_for_power_on_fail:	
	Stall_Cnt++;					// Increment stall count
	// Check if RCP is zero, then it is a normal stop			
	if(!New_Rcp) run_to_wait_for_power_on
	ajmp run_to_wait_for_power_on_stall_done

run_to_wait_for_power_on:	
	Stall_Cnt = 0;

run_to_wait_for_power_on_stall_done:
	IE_EA = 0;
	call switch_power_off
	Flags0 = 0;				// Clear flags0
	Flags1 = 0;				// Clear flags1
#IF MCU_48MHZ == 1
	Set_MCU_Clk_24MHz
#ENDIF
	IE_EA = 1;
	call	wait100ms					// Wait for pwm to be stopped
	call switch_power_off
	if(!Pgm_Brake_On_Stop) run_to_wait_for_power_on_brake_done

	AcomFET_on
	BcomFET_on
	CcomFET_on

run_to_wait_for_power_on_brake_done:
	if(Stall_Cnt < 4) jmp_wait_for_power_on
	jmp	init_no_signal

jmp_wait_for_power_on:
	jmp	wait_for_power_on			// Go back to wait for power on
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****

#include "BLHeliPgm.inc"				// Include source code for programming the ESC
#include "BLHeliBootLoad.inc"			// Include source code for bootloader

//**** **** **** **** **** **** **** **** **** **** **** **** ****


CSEG AT 19FDh
reset:
ljmp	pgm_start

END
