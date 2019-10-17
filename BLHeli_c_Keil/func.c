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













