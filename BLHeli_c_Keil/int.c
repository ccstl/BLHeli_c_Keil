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
#ELSE
	if(!Current_Power_Pwm_Reg_H&0x04)
	{
		if(PCA0H&0x02) pca_int_exit;
		if(PCA0H&0x01) pca_int_exit;
	}
#ENDIF

#IF MCU_48MHZ == 0
	if(PCA0H&0x02) pca_int_exit;
	if(PCA0H&0x01) pca_int_exit;
#ELSE
	if(PCA0H&0x04) pca_int_exit;
	if(PCA0H&0x02) pca_int_exit;
#ENDIF
	pca_int_set_pwm;
#IF MCU_48MHZ == 0
	if(PCA0H&0x02) pca_int_exit;
	if(PCA0H&0x01) pca_int_exit;
#ELSE
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
	
pca_int_exit:
	Clear_COVF_Interrupt;
#IF FETON_DELAY == 0
	Clear_CCF_Interrupt;
#ENDIF
	IE_EA = 1;
}





