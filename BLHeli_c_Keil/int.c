//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Timer 0 interrupt routine
//
// No assumptions
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void t0_int()
{
#IF MCU_48MHZ == 1
	Timer0_X++;
	return;
#ENDIF
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Timer 1 interrupt routine
//
// No assumptions
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void t1_int()
{
	IE_EA = 0;			// Disable all interrupts
	IE_EX0 = 0;			// Disable int0 interrupts
	EIE1 &= 0xEF;		// Disable pca interrupts
	TCON_TR1 = 0;		// Stop timer 1
	TL1 = DShot_Timer_Preset;	// Reset sync timer
	// Select register bank 1 for this interrupt
	// Will be pop'ed by int0 exit
	TMR2CN0_TR2 = 0;		// Timer 2 disabled
	Temp1 = TMR2L;		// Read timer value
	Temp2 = TMR2H;
	TMR2CN0_TR2 = 1;		// Timer 2 enabled
	IE_EA = 1;
	// Reset timer 0
	TL0 = 0;
	// Check frame time length
	Temp1 = Temp1 - DShot_Frame_Start_L;
	Temp2 = Temp2 - DShot_Frame_Start_H;
	// Divide by 2 (or 4 for 48MHz). Unit is then us
	Temp2 = Temp2>>1;
	Temp1 = Temp1>>1;
	if(Clock_Set_At_48MHz)
	{
		Temp2 = Temp2>>1;
		Temp1 = Temp1>>1;
	}
	if(Temp2) t1_int_msb_fail	// Frame too long
	if(Temp1 < DShot_Frame_Length_Thr) t1_int_msb_fail	// Frame too short
	if(Temp1 >= 2*DShot_Frame_Length_Thr) t1_int_msb_fail	// Frame too long

	// Check that correct number of pulses is received
	//mov	A, DPL			// Read current pointer
	//cjne	A, #16, t1_int_msb_fail

	// Decode transmitted data
	Temp5 = 0;			// Reset timestamp
	Temp4 = 0;			// High byte of receive buffer
	Temp3 = 0;			// Low byte of receive buffer
	Temp2 = 8;			// Number of bits per byte
	DPTR = 0;			// Set pointer
	Temp1 = DShot_Pwm_Thr;// DShot pulse width criteria
	if(!Clock_Set_At_48MHz)
	{
		Temp1 = Temp1>>1;			// Scale pulse width criteria
	}

	ajmp	t1_int_decode_msb

t1_int_msb_fail:
	DPTR = 0;		 	// Set pointer to start
	IE_EX0 = 1;			// Enable int0 interrupts
	IE_EX1 = 1;			// Enable int1 interrupts
	ajmp int0_int_outside_range

t1_int_decode_msb:
	// Decode DShot data Msb. Use more code space to save time (by not using loop)
	Decode_DShot_2Msb
	Decode_DShot_2Msb
	Decode_DShot_2Msb
	Decode_DShot_2Msb
	ajmp	t1_int_decode_lsb

t1_int_lsb_fail:
	DPTR = 0;		 	// Set pointer to start
	IE_EX0 = 1;			// Enable int0 interrupts
	IE_EX1 = 1;			// Enable int1 interrupts
	ajmp int0_int_outside_range

t1_int_decode_lsb:
	// Decode DShot data Lsb
	Decode_DShot_2Lsb
	Decode_DShot_2Lsb
	Decode_DShot_2Lsb
	Decode_DShot_2Lsb
	// XOR check (in inverted data, which is ok)
	mov	A, Temp4
	swap	A
	xrl	A, Temp4
	xrl	A, Temp3
	anl	A, #0F0h
	mov	Temp2, A
	mov	A, Temp3
	swap	A
	anl	A, #0F0h
	clr	C
	subb	A, Temp2
	//jz	t1_int_xor_ok		// XOR check
	if(!Temp2)
	{
		DPTR = 0;		 	// Set pointer to start
		IE_EX0 = 1;			// Enable int0 interrupts
		IE_EX1 = 1;			// Enable int1 interrupts
		ajmp int0_int_outside_range
	}
//t1_int_xor_ok:
	// Swap to be LSB aligned to 12 bits (and invert)
	mov	A, Temp4
	cpl	A
	swap A
	anl	A, #0F0h			// Low nibble of high byte
	mov	Temp2, A
	mov	A, Temp3
	cpl	A
	swap	A
	anl	A, #0Fh			// High nibble of low byte 
	orl	A, Temp2
	mov	Temp3, A
	mov	A, Temp4			// High nibble of high byte
	cpl	A
	swap A
	anl	A, #0Fh
	mov	Temp4, A
	// Subtract 96 (still 12 bits)
	Temp2 = Temp3;
	Temp3 = Temp3 - 96;
	Temp4 = Temp4 - 0;
	if(Temp2 < 96) //t1_normal_range
	{
		mov	A, Temp2  		// Check for 0 or dshot command
		Temp4 = 0;
		Temp3 = 0;
		Temp2 = 0;
		if(Temp2!=0) 	//t1_normal_range
		{
		// We are in the special dshot range
		// Divide by 2
			if(Temp2&0x01 == 0) //t1_dshot_set_cmd 	// Check for tlm bit set (if not telemetry, Temp2 will be zero and result in invalid command)
			{
				Temp2 = Temp2>>1;

			//subb A, Dshot_Cmd
			//t1_dshot_inc_cmd_cnt
			}
		//t1_dshot_set_cmd:
			if(Temp2 != Dshot_Cmd)
			{
			//mov A, Temp2
				Dshot_Cmd = Temp2;
				Dshot_Cmd_Cnt = 0;
				Temp2 = 0;
			//jmp t1_normal_range
			}
		//t1_dshot_inc_cmd_cnt:
			else
			{
				Dshot_Cmd_Cnt++;
			}
		}
	}
//t1_normal_range:
	// Check for bidirectional operation (0=stop, 96-2095->fwd, 2096-4095->rev)
	if(Flags3.PGM_BIDIR) //t1_int_not_bidir	// If not bidirectional operation - branch
	{
	// Subtract 2000 (still 12 bits)
	Temp1 = Temp3 - 0xD0;
	Temp2 = Temp4 - 0x07;
	if(Temp3 >= 0xD0 || Temp4 >= 0x07) //t1_int_bidir_fwd				// If result is negative - branch
	{
	Temp3 = Temp1;
	Temp4 = Temp2;
	if(!Flags2.RCP_DIR_REV) //t1_int_bidir_rev_chk	// If same direction - branch
		Flags2.RCP_DIR_REV = 1;
	ajmp t1_int_bidir_rev_chk
	}
//t1_int_bidir_fwd:
	if(Flags2.RCP_DIR_REV) //t1_int_bidir_rev_chk	// If same direction - branch
	Flags2.RCP_DIR_REV = 0;

//t1_int_bidir_rev_chk:
	if(!Flags3.PGM_BIDIR_REV)
	Flags2.RCP_DIR_REV = ~Flags2.RCP_DIR_REV;

	// Multiply throttle value by 2
	Temp3 = Temp3<<1;
	Temp4 = Temp4<<1;
	}
t1_int_not_bidir:
	// Generate 4/256
	Temp2 = Temp4 << 2;
	// Align to 11 bits
	Temp4 = Temp4<<1;
	Temp3 = Temp3<<1;
	// Scale from 2000 to 2048
	// Holds 4/128
	Temp3 = Temp3 + Temp2;
	Temp4 = Temp4 + 0;

	if(Temp4&0x04)
	{
	Temp3 = 0xFF;
	Temp4 = 0xFF;
	}
	// Boost pwm during direct start
	//mov	A, Flags1
	if(Flags1&((1 << STARTUP_PHASE)+(1 << INITIAL_RUN_PHASE))) //t1_int_startup_boosted
	{
		if(!Flags1.MOTOR_STARTED) //t1_int_startup_boosted	// Do not boost when changing direction in bidirectional mode
		{
			// Set 25% of max startup power as minimum power
			Temp2 = Pwm_Limit_Beg << 1;
			if(Temp4==0) //t1_int_startup_boost_stall
			{
				if(Temp2 >= Temp3) //t1_int_startup_boost_stall
					Temp3 = Temp2;
			}
		//t1_int_startup_boost_stall:
			mov	A, Stall_Cnt					// Add an extra power boost during start
			swap	A
			rlc	A
			add	A, Temp3
			mov	Temp3, A
			mov	A, Temp4
			addc	A, #0
			mov	Temp4, A
		}
	}
t1_int_startup_boosted:
	// Set 8bit value
	clr	C
	mov	A, Temp3
	rlc	A
	swap	A
	anl	A, #0Fh
	mov	Temp1, A
	mov	A, Temp4
	rlc	A
	swap	A
	anl	A, #0F0h
	orl	A, Temp1
	mov	Temp1, A
	jnz	t1_int_zero_rcp_checked	// New_Rcp (Temp1) is only zero if all 11 bits are zero

	mov	A, Temp3
	jz	t1_int_zero_rcp_checked

	mov	Temp1, #1

t1_int_zero_rcp_checked:
	// Align to 10 bits for 24MHz MCU
#IF MCU_48MHZ == 0
	Temp4 = Temp4 >> 1;
	Temp3 = Temp3 >> 1;
#ENDIF
	DPTR = 0;		 			// Set pointer to start
	IE_EX0 = 1;					// Enable int0 interrupts
	IE_EX1 = 1;					// Enable int1 interrupts
	// Decrement outside range counter
	mov	A, Rcp_Outside_Range_Cnt
	if(Rcp_Outside_Range_Cnt) Rcp_Outside_Range_Cnt--;

	ajmp int0_int_pulse_ready

t1_int_frame_fail:
	DPTR = 0;		 			// Set pointer to start
	IE_EX0 = 1;					// Enable int0 interrupts
	IE_EX1 = 1;					// Enable int1 interrupts
	ajmp int0_int_outside_range
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Timer 2 interrupt routine
//
// No assumptions
// Requirements: Temp variables can NOT be used since PSW.x is not set
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void t2_int()
{
	TMR2CN0_TF2H = 0;
	Timer2_X++;
#IF MCU_48MHZ == 1
	if(!Clock_Set_At_48MHz) Skip_T2_Int = 1;//Skip next interrupt
	// Check skip variable
	if(!Skip_T2_Int) Skip_T2_Int = 1;//Execute this interrupt

	Skip_T2_Int = 0;
	return;
#ENDIF
	if(!Rcp_Timeout_Cntd) Rcp_Timeout_Cntd--;
	if(!New_Rcp)
	{
		if(Rcp_Stop_Cnt < 0xFF) return;
		Rcp_Stop_Cnt = 0xFF;
	}
	Rcp_Stop_Cnt = 0;
	return;
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Timer 3 interrupt routine
//
// No assumptions
// Requirements: Temp variables can NOT be used since PSW.x is not set
//               ACC can not be used, as it is not pushed to stack
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void t3_int()
{
	IE_EA = 0;
	EIE1 &= 0x7F;
	TMR3RLL = 0xFA;
	TMR3RLH = 0xFF;
	Flag0.T3_PENDING = 0;
	TMR3CN0 = 0x7F;
	IE_EA = 1;
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Int0 interrupt routine
//
// No assumptions
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
int0_int()	// Used for RC pulse timing
{
	//mov	A, TL0			// Read pwm for DShot immediately
	// Test for DShot
	if(Flags2.RCP_DSHOT) //int0_int_not_dshot
	{
	TL1 = DShot_Timer_Preset;	// Reset sync timer
	movx	@DPTR, A			// Store pwm
	inc	DPTR
	pop	ACC
	return;
	}
	// Not DShot
//int0_int_not_dshot:
	IE_EA = 0;
	EIE1 &= 0xEF;		// Disable pca interrupts
	// Preserve registers through interrupt
	// Select register bank 1 for this interrupt 
	IE_EA = 1;
	// Get the counter values
	Get_Rcp_Capture_Values
	// Scale down to 10 bits (for 24MHz, and 11 bits for 48MHz)
	if(Flags2.RCP_MULTISHOT) //int0_int_fall_not_multishot
	{
	// Multishot - Multiply by 2 and add 1/16 and 1/32
		mov	A, Temp1		// Divide by 16
		swap A
		anl	A, #0Fh
		mov	Temp3, A
		mov	A, Temp2
		swap	A
		anl	A, #0F0h
		orl	A, Temp3
		mov	Temp3, A
		clr	C			// Make divided by 32
		rrc	A
		add	A, Temp3		// Add 1/16 to 1/32
		mov	Temp3, A
		// Multiply by 2
		Temp1 = Temp1<<1;
		Temp2 = Temp2<<1;
		// Add 1/16 and 1/32
		Temp3 = Temp1 + Temp3;
	#IF MCU_48MHZ == 0
		Temp4 = Temp2 + 0x03;		// Add to low end, to make signal look like 20-40us
	#ELSE
		Temp4 = Temp2 + 0x06;
	#ENDIF
		//ajmp int0_int_fall_gain_done
	}
	else
	{
//int0_int_fall_not_multishot:
		if(Flags2.RCP_ONESHOT42) //int0_int_fall_not_oneshot_42
		{
			// Oneshot42 - Add 2/256
			Temp1 = Temp1 << 1;
			Temp3 = Temp2 << 1;

			Temp3 = Temp1 + Temp3;
			Temp4 = Temp2 + 0;
			//ajmp	int0_int_fall_gain_done
		}
		else
		{
	//int0_int_fall_not_oneshot_42:
			if(Flags2.RCP_ONESHOT125) //int0_int_fall_not_oneshot_125
			{
				// Oneshot125 - multiply by 86/256
				mov	A, Temp1		// Multiply by 86 and divide by 256
				mov	B, #56h
				mul	AB
				mov	Temp3, B
				mov	A, Temp2
				mov	B, #56h
				mul	AB
				add	A, Temp3
				mov	Temp3, A
				xch	A, B
				addc	A, #0
				mov	Temp4, A
				//ajmp	int0_int_fall_gain_done
			}
			else
			{
			//int0_int_fall_not_oneshot_125:
				// Regular signal - multiply by 43/1024
			#IF MCU_48MHZ == 1
				Temp3 = Temp3 >> 1;		// Divide by 2
				Temp2 = Temp2 >> 1;
				Temp1 = Temp1 >> 1;
			#ENDIF
				mov	A, Temp1		// Multiply by 43 and divide by 1024
			#IF MCU_48MHZ == 0
				mov	B, #2Bh
			#ELSE
				mov	B, #56h		// Multiply by 86
			#ENDIF
				mul	AB
				mov	Temp3, B
				mov	A, Temp2
			#IF MCU_48MHZ == 0
				mov	B, #2Bh
			#ELSE
				mov	B, #56h		// Multiply by 86
			#ENDIF
				mul	AB
				add	A, Temp3
				mov	Temp3, A
				xch	A, B
				addc	A, #0
				clr	C	
				rrc	A			// Divide by 2 for total 512
				mov	Temp4, A
				mov	A, Temp3
				rrc	A
				mov	Temp3, A
				clr	C
				mov	A, Temp4		// Divide by 2 for total 1024
				rrc	A						
				mov	Temp4, A
				mov	A, Temp3
				rrc	A
				mov	Temp3, A
			}
		}
	}
//int0_int_fall_gain_done:
	// Check if 2235us or above (in order to ignore false pulses)
	mov	A, Temp4						// Is pulse 2235us or higher?
#IF MCU_48MHZ == 0
	if(Temp4 > 0x09) int0_int_outside_range// Yes - ignore pulse
	// Check if below 900us (in order to ignore false pulses)
	if(Temp3 >= 0x9A && Temp4 >= 0x03)	int0_int_check_full_range		// No - proceed
#ELSE
	if(Temp4 > 0x12) int0_int_outside_range// Yes - ignore pulse
	// Check if below 900us (in order to ignore false pulses)
	if(Temp3 >= 0x34 && Temp4 >= 0x07)	int0_int_check_full_range		// No - proceed
#ENDIF


int0_int_outside_range:
	Rcp_Outside_Range_Cnt++;
	if(Rcp_Outside_Range_Cnt) Rcp_Outside_Range_Cnt--;

	// Allow a given number of outside pulses
	if(Rcp_Outside_Range_Cnt < 50)
	ajmp int0_int_set_timeout			// If outside limits - ignore first pulses

	New_Rcp = 0;					// Set pulse length to zero
	ajmp int0_int_exit					// Exit without reseting timeout

int0_int_check_full_range:
	// Decrement outside range counter
	if(Rcp_Outside_Range_Cnt)
	Rcp_Outside_Range_Cnt--;

	// Calculate "1000us" plus throttle minimum
	if(Flags2.RCP_FULL_RANGE) //int0_int_set_min	// Check if full range is chosen
	{
	Temp5 = 0;						// Set 1000us as default minimum
#IF MCU_48MHZ == 0
	Temp6 = 4;
#ELSE
	Temp6 = 8;
#ENDIF
	//ajmp int0_int_calculate
	}
	else
	{
//int0_int_set_min:
	Temp5 = Min_Throttle_L;			// Min throttle value scaled
	Temp6 = Min_Throttle_H;
	if(!Flags3.PGM_BIDIR)
	{Temp5 = Center_Throttle_L;			// Center throttle value scaled
	Temp6 = Center_Throttle_H;}
	}
//int0_int_calculate:
	Temp3 = Temp3 - Temp5;						// Subtract minimum
	Temp4 = Temp4 - Temp6;
	mov	Bit_Access_Int.0, C
	Temp7 = Throttle_Gain;				// Load Temp7/Temp8 with throttle gain
	Temp8 = Throttle_Gain_M;
	if(Flags3.PGM_BIDIR) //int0_int_not_bidir	// If not bidirectional operation - branch
	{
	if(Temp3>0&&Temp4>0)//int0_int_bidir_fwd					// If result is positive - branch
	{
		if(Flags2.RCP_DIR_REV) //int0_int_bidir_rev_chk	// If same direction - branch
			Flags2.RCP_DIR_REV = 0;
	}
	else
	{
		if(!Flags2.RCP_DIR_REV) //int0_int_bidir_rev_chk	// If same direction - branch
			Flags2.RCP_DIR_REV = 1;
	//ajmp int0_int_bidir_rev_chk
	}
	
//int0_int_bidir_fwd:
	//if(Flags2.RCP_DIR_REV) //int0_int_bidir_rev_chk	// If same direction - branch
		//Flags2.RCP_DIR_REV = 0;

//int0_int_bidir_rev_chk:
	if(Flags2.RCP_DIR_REV)
	{
	Temp7 = Throttle_Gain_BD_Rev;		// Load Temp7/Temp8 with throttle gain for bidirectional reverse
	Temp8 = Throttle_Gain_BD_Rev_M;
	}
	if(!Flags3.PGM_BIDIR_REV) Flags2.RCP_DIR_REV = ~Flags2.RCP_DIR_REV;

	// Multiply throttle value by 2
	Temp3 = Temp3<<1;
	Temp4 = Temp4<<1;
	//mov	C, Bit_Access_Int.0
	//jnc	int0_int_bidir_do_deadband		// If result is positive - branch
	if(Bit_Access_Int.0)
	{Temp3 = ~Temp3 + 1;						// Change sign
	Temp4 = ~Temp4 + 1;}

//int0_int_bidir_do_deadband:
	// Subtract deadband
	mov	A, Temp3
#IF MCU_48MHZ == 0
	Temp3 = Temp3 - 0x40;
#ELSE
	Temp3 = Temp3 - 0x80;
#ENDIF
	Temp4 = Temp4 - 0x00;
	//jnc	int0_int_do_throttle_gain
	if(Temp4 < 0x00 || Temp3 < 0x40 || Temp3 < 0x80)
	{
	Temp1 = 0x00;
	Temp3 = 0x00;
	Temp4 = 0x00;
	ajmp	int0_int_do_throttle_gain
	}
	}
//int0_int_not_bidir:
	//mov	C, Bit_Access_Int.0
	//jnc	int0_int_do_throttle_gain		// If result is positive - branch
	if(Bit_Access_Int.0)
	{
//int0_int_unidir_neg:
	// Yes - set to minimum
	Temp1 = 0x00;
	Temp3 = 0x00;
	Temp4 = 0x00;
	ajmp	int0_int_pulse_ready
	}
int0_int_do_throttle_gain:
	// Boost pwm during direct start
	if(Flags1&((1 << STARTUP_PHASE)+(1 << INITIAL_RUN_PHASE)))
	{//int0_int_startup_boosted

	if(Flags1.MOTOR_STARTED) //int0_int_startup_boosted	// Do not boost when changing direction in bidirectional mode
	{
	// Set 25% of max startup power as minimum power
#IF MCU_48MHZ == 1
	Temp2 = Pwm_Limit_Beg << 1;
#ENDIF
	Temp2 = Pwm_Limit_Beg;
	if(!Temp4) //int0_int_startup_boost_stall
	{
	if(Temp2>Temp3) //int0_int_startup_boost_stall
		Temp3 = Temp2;
	}
//int0_int_startup_boost_stall:
	mov	A, Stall_Cnt					// Add an extra power boost during start
	swap	A
#IF MCU_48MHZ == 1
	rlc	A
#ENDIF
	add	A, Temp3
	mov	Temp3, A
	mov	A, Temp4
	addc	A, #0
	mov	Temp4, A
	}
	}
//int0_int_startup_boosted:
	//mov	A, Temp3						// Multiply throttle value by throttle gain
	//mov	B, Temp7						// Temp7 has Throttle_Gain
	//mul	AB
	//mov	Temp2, A
	//mov	Temp3, B
	Temp3Temp2 = Temp3*Temp7;
	mov	A, Temp4
	mov	B, Temp7						// Temp7 has Throttle_Gain
	mul	AB
	add	A, Temp3
	mov	Temp3, A
	xch	A, B
	addc	A, #0
	mov	Temp4, A
	clr	C							// Generate 8bit number
	mov	A, Temp4
	rrc	A
	mov	Temp6, A
	mov	A, Temp3
	rrc	A
	mov	Temp1, A
#IF MCU_48MHZ == 1
	//clr	C
	//mov	A, Temp6
	//rrc	A
	//mov	Temp6, A
	Temp6 = Temp6<<1;
	//mov	A, Temp1
	//rrc	A
	//mov	Temp1, A
	Temp1 = Temp1<<1;
#ENDIF
	inc	Temp8						// Temp8 has Throttle_Gain_M
//int0_int_gain_loop:
	do
	{
	A = Temp8;
	//dec	A
	A--;
	//jz	int0_int_gain_rcp_done			// Skip one multiply by 2 of New_Rcp
	
	//clr	C
	//mov	A, Temp1						// Multiply New_Rcp by 2
	//rlc	A
	//mov	Temp1, A
	if(A) Temp1 = Temp1<<1;

//int0_int_gain_rcp_done:
	//clr	C
	//mov	A, Temp2						// Multiply pwm by 2
	//rlc	A
	Temp2<<1;
	//mov	A, Temp3
	//rlc	A
	//mov	Temp3, A
	Temp3 = Temp3<<1;
	//mov	A, Temp4
	//rlc	A
	//mov	Temp4, A
	Temp4 = Temp4<<1;
	//djnz	Temp8, int0_int_gain_loop
	}while(Temp8--);
	//mov	A, Temp4
#IF MCU_48MHZ == 0
	if(Temp4&0x04) //int0_int_pulse_ready		// Check that RC pulse is within legal range
	{
	Temp1 = 0xFF;
	Temp3 = 0xFF;
	Temp4 = 3;
	}
#ELSE
	if(Temp4&0x08) //int0_int_pulse_ready
	{
	Temp1 = 0xFF;
	Temp3 = 0xFF;
	Temp4 = 7;
	}
#ENDIF

//int0_int_pulse_ready:
	New_Rcp = Temp1;					// Store new pulse length
	Flags2.RCP_UPDATED = 1;		 		// Set updated flag
	// Check if zero
	mov	A, Temp1						// Load new pulse value
	jz	($+5)							// Check if pulse is zero

	Rcp_Stop_Cnt = 0;				// Reset rcp stop counter

	// Set pwm limit
	clr	C
	//mov	A, Pwm_Limit					// Limit to the smallest
	//mov	Temp5, A						// Store limit in Temp5
	//subb	A, Pwm_Limit_By_Rpm
	//jc	($+4)
	if(Pwm_Limit >= Pwm_Limit_By_Rpm)
	Temp5 = Pwm_Limit_By_Rpm;			

	// Check against limit
	//clr	C
	//mov	A, Temp5
	//subb	A, New_Rcp
	//jnc	int0_int_set_pwm_registers
	if(Pwm_Limit_By_Rpm < New_Rcp)
	{
	mov	A, Temp5						// Multiply limit by 4 (8 for 48MHz MCUs)
#IF MCU_48MHZ == 0
	//mov	B, #4
	Pwm_Limit_By_Rpm*4;
	//mul	AB
	mov	Temp3, A
	mov	Temp4, B
#ELSE
	//mov	B, #8
	Pwm_Limit_By_Rpm*8;
	//mul	AB
	mov	Temp3, A
	mov	Temp4, B
#ENDIF
	}
//int0_int_set_pwm_registers:
	//mov	A, Temp3
	//cpl	A
	//mov	Temp1, A
	Temp1 = ~Temp3;
	//mov	A, Temp4
	//cpl	A
#IF MCU_48MHZ == 0
	//anl	A, #3
	Temp2 = (~Temp4)&0x03;
#ELSE
	//anl	A, #7
	Temp2 = (~Temp4)&0x07;
#ENDIF
	//mov	Temp2, A
#IF FETON_DELAY != 0
	//clr	C
	//mov	A, Temp1						// Skew damping fet timing
#IF MCU_48MHZ == 0
	//subb	A, #FETON_DELAY
	Temp3 = Temp1 - FETON_DELAY;
#ELSE
	//subb	A, #(FETON_DELAY<<1)
	Temp3 = Temp1 - (FETON_DELAY<<1);
#ENDIF
	//mov	Temp3, A
	//mov	A, Temp2
	//subb	A, #0	
	//mov	Temp4, A
	Temp4 = Temp2 - 0;
	//jnc	int0_int_set_pwm_damp_set
	if(Temp2 < 0)
	{
	Temp3 = 0;
	Temp4 = 0;
	}
//int0_int_set_pwm_damp_set:
#ENDIF
	Power_Pwm_Reg_L = Temp1;
	Power_Pwm_Reg_H = Temp2;
#IF FETON_DELAY != 0
	Damp_Pwm_Reg_L = Temp3;
	Damp_Pwm_Reg_H = Temp4;
#ENDIF
	Rcp_Timeout_Cntd = 10;			// Set timeout count
#IF FETON_DELAY != 0
	// Restore preserved registers
	Clear_COVF_Interrupt
	Enable_COVF_Interrupt				// Generate a pca interrupt
	EIE1 |= 0x10;					// Enable pca interrupts
	return;
#ELSE
	//mov	A, Current_Power_Pwm_Reg_H
#IF MCU_48MHZ == 0
	if(Current_Power_Pwm_Reg_H&0x02) //int0_int_set_pca_int_hi_pwm
	{
		// Restore preserved registers
		Clear_COVF_Interrupt;
		Enable_COVF_Interrupt;				// Generate a pca interrupt
		EIE1 |= 0x10;					// Enable pca interrupts
		return;
	}
#ELSE
	if(Current_Power_Pwm_Reg_H&0x04) //int0_int_set_pca_int_hi_pwm
	{
		// Restore preserved registers
		Clear_COVF_Interrupt;
		Enable_COVF_Interrupt;				// Generate a pca interrupt
		EIE1 |= 0x10;					// Enable pca interrupts
		return;
	}
#ENDIF

//int0_int_set_pca_int_hi_pwm:
	// Restore preserved registers
	Clear_CCF_Interrupt
	Enable_CCF_Interrupt				// Generate pca interrupt
	EIE1 |= 0x10;					// Enable pca interrupts
	return;
#ENDIF

int0_int_set_timeout:
	Rcp_Timeout_Cntd = 10;			// Set timeout count
int0_int_exit:
	// Restore preserved registers
	EIE1 |= 0x10;					// Enable pca interrupts
	return;
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// Int1 interrupt routine
//
// No assumptions
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void int1_int()
{
	//Used for RC pulse timing
	IE_EX1 = 0;
	TCON_TR1 = 1;
	TMR2CN0_TR2 = 0;
	DShot_Frame_Start_L = TMR2L;
	DShot_Frame_Start_H = TMR2H;
	TMR2CN0_TR2 = 1;
}

//**** **** **** **** **** **** **** **** **** **** **** **** ****
//
// PCA interrupt routine
//
// No assumptions
//
//**** **** **** **** **** **** **** **** **** **** **** **** ****
void pca_int()
{
	IE_EA = 0;
#IF FETON_DELAY != 0
	Temp1 = PCA0L;

#IF MCU_48MHZ == 0
	if(!Current_Power_Pwm_Reg_H&0x02)
	{
		if(PCA0H&0x02) pca_int_exit;
		if(PCA0H&0x01) pca_int_exit;
	}
	if(PCA0H&0x02) pca_int_exit;
	if(PCA0H&0x01) pca_int_exit;
	pca_int_set_pwm;
	if(PCA0H&0x02) pca_int_exit;
	if(PCA0H&0x01) pca_int_exit;
#ELSE
	if(!Current_Power_Pwm_Reg_H&0x04)
	{
		if(PCA0H&0x02) pca_int_exit;
		if(PCA0H&0x01) pca_int_exit;
	}
	if(PCA0H&0x04) pca_int_exit;
	if(PCA0H&0x02) pca_int_exit;
	pca_int_set_pwm;
	if(PCA0H&0x04) pca_int_exit;
	if(PCA0H&0x02) pca_int_exit;
#ENDIF
	Set_Power_Pwm_Regs;
	Set_Damp_Pwm_Regs;
	Current_Power_Pwm_Reg_H = Power_Pwm_Reg_H;
	Disable_COVF_Interrupt;
#ELSE
	Set_Power_Pwm_Regs;
	Current_Power_Pwm_Reg_H = Power_Pwm_Reg_H;
	Disable_COVF_Interrupt;
	Disable_CCF_Interrupt;
#ENDIF
	IE_EX0 = 1;
	if(Flags2.RCP_DSHOT) EIE1 = 0xEF;
	IE_EX1 = 1;
	EIE1 = 0xEF;
}

pca_int_exit()
{
	Clear_COVF_Interrupt;
#IF FETON_DELAY == 0
	Clear_CCF_Interrupt;
#ENDIF
	IE_EA = 1;
}


