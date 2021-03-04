#include <p30f4011.h>
//#include <math.h>

_FOSC(FRC_PLL16 & PRI & CSW_FSCM_OFF)
//_FOSC(0xC303)
_FWDT(WDTPSB_16 & WDTPSA_512 & WDT_OFF)
_FBORPOR(PWRT_OFF & BORV27 & PBOR_ON & PWMxL_ACT_HI & PWMxH_ACT_HI & MCLR_EN)
_FGS(GWRP_OFF & CODE_PROT_OFF)

#include<adc10.h>
unsigned int adc[4];
unsigned int adc_n=0;
//static double adc_ant[4];
double adc_vol[4];
double adc_sen[4];
void __attribute__((__interrupt__,no_auto_psv)) _ADCInterrupt(void){//10bits 4, 8bits 6
	ADCON1bits.ADON = 0;//ADC OFF
	
	if(adc_n==0){
		ADCHSbits.CH0SA=1;
		adc[0]=(ADCBUF0+ADCBUF1+ADCBUF2+ADCBUF3+ADCBUF4+ADCBUF5+ADCBUF6+ADCBUF7+ADCBUF8+ADCBUF9+ADCBUFA+ADCBUFB+ADCBUFC+ADCBUFD+ADCBUFE+ADCBUFF)>>4;
		adc_n=1;
	}
	else if(adc_n==1){
		ADCHSbits.CH0SA=2;
		adc[1]=(ADCBUF0+ADCBUF1+ADCBUF2+ADCBUF3+ADCBUF4+ADCBUF5+ADCBUF6+ADCBUF7+ADCBUF8+ADCBUF9+ADCBUFA+ADCBUFB+ADCBUFC+ADCBUFD+ADCBUFE+ADCBUFF)>>4;
		adc_n=2;
	}
	else if(adc_n==2){
		ADCHSbits.CH0SA=3;
		adc[2]=(ADCBUF0+ADCBUF1+ADCBUF2+ADCBUF3+ADCBUF4+ADCBUF5+ADCBUF6+ADCBUF7+ADCBUF8+ADCBUF9+ADCBUFA+ADCBUFB+ADCBUFC+ADCBUFD+ADCBUFE+ADCBUFF)>>4;
		adc_n=3;
	}
	else if(adc_n==3){
		ADCHSbits.CH0SA=0;
		adc[3]=(ADCBUF0+ADCBUF1+ADCBUF2+ADCBUF3+ADCBUF4+ADCBUF5+ADCBUF6+ADCBUF7+ADCBUF8+ADCBUF9+ADCBUFA+ADCBUFB+ADCBUFC+ADCBUFD+ADCBUFE+ADCBUFF)>>4;
		adc_n=0;
	}
	
	IFS0bits.ADIF = 0;
	ADCON1bits.ADON = 1;//ADC ON
}
void vol_val(void){//10bits 0.00489(0.004887), 8bits 0.0196
   adc_vol[0]=5-adc[0]*0.004887;//ECT
   adc_vol[1]=adc[1]*0.004887;//IAT
   adc_vol[2]=5-adc[2]*0.004887;//MAP
   adc_vol[3]=adc[3]*0.004887;//TPS
}

double tiempo_ms=0;
#include<timer.h>
unsigned int tmr1 = 63692;
void __attribute__((__interrupt__,no_auto_psv)) _T1Interrupt(void){
	LATDbits.LATD2 = 0;		// iny OFF */
	
	T1CONbits.TON = 0;		// Timer1 OFF */
	IEC0bits.T1IE = 0;		// Disable TMR1 interrupt */
	
	IFS1bits.INT2IF = 0;	// Clear INT2 interrupt flag */
	
	IFS0bits.T1IF = 0;		// Clear Timer1 interrupt flag */
}
void tiempo_tmr1_iny(float t){
	tmr1=65535-(t*3685);
}

unsigned int tmr2l = 65535-4;
unsigned int tmr2h = 61443+4;
unsigned int adc_m = 0;
void __attribute__((__interrupt__,no_auto_psv)) _T2Interrupt(void){
	T2CONbits.TON = 0; //Start Timer2
	if(LATCbits.LATC14){
		LATCbits.LATC14=0;//bob
		WriteTimer2(tmr2l);
	}
	else{
		LATCbits.LATC14=1;//bob
		WriteTimer2(tmr2h);
	}
	IFS0bits.T2IF = 0;    /* Clear Timer interrupt flag */
	T2CONbits.TON = 1; //Start Timer2
}
void dutty_tmr2(void){
	if(adc[3]>194){//774
		adc_m=11253;
	}else if(adc[3]<37){//149
		adc_m=3;
	}else{
		adc_m=72*adc[3]-2679;
	}
	tmr2l = 65534-adc_m;
	tmr2h = 54281+adc_m;
}

unsigned int tmr3=0;
unsigned int tmr3_100=65535-11516;//100mS
unsigned int ralenti=0;
unsigned int tmr3_enriquesimiento=0;
unsigned int enriquesimiento_i=0;
void __attribute__((__interrupt__,no_auto_psv)) _T3Interrupt(void){
	T3CONbits.TON = 0;		// Timer3 OFF */
	if(ralenti==0){
		tmr3++;
		if(tmr3>=12){
			ralenti=1;
			tmr3=0;
			
			T3CONbits.TON = 0;		// Timer3 OFF */
			IEC0bits.T3IE = 0;		// Disable TMR3 interrupt */
			IFS0bits.T3IF = 0;		// Clear Timer interrupt flag */
		}else{
			T3CONbits.TON = 1;		// Timer3 ON */
			IFS0bits.T3IF = 0;		// Clear Timer interrupt flag */
		}
	}else{
		tmr3++;
		if(tmr3>=tmr3_enriquesimiento){
			tmr3=0;
			enriquesimiento_i=0;
			
			//LATDbits.LATD0=0;
		
			T3CONbits.TON = 0;		// Timer3 OFF */
			IEC0bits.T3IE = 0;		// Disable TMR3 interrupt */
			IFS0bits.T3IF = 0;		// Clear Timer interrupt flag */
		}else{
			WriteTimer3(tmr3_100);	//TMR3=65535-11516;//each 100ms
			T3CONbits.TON = 1;		// Timer3 ON */
			IFS0bits.T3IF = 0;		// Clear Timer interrupt flag */
		}
	}	
}
void tiempo_tmr3_enr(float t){
	tmr3_enriquesimiento=t*10;
}
void empezar_enriquesimiento(void){
	enriquesimiento_i=1;
	tmr3=0;
	
	//LATDbits.LATD0=1;
	
	WriteTimer3(tmr3_100);	//TMR3=65535-11516;//each 100ms
	T3CONbits.TON = 1;		// Timer3 ON */
	IEC0bits.T3IE = 1;		// Enable TMR3 interrupt */
}

void __attribute__((__interrupt__,no_auto_psv)) _T4Interrupt(void){
	//LATDbits.LATD2 = 0;		// iny OFF */
	
	T4CONbits.TON = 0;		// Timer4 OFF */
	IEC1bits.T4IE = 0;		// Disable TMR4 interrupt */
		
	IFS1bits.INT2IF = 0;	// Clear INT2 interrupt flag */
	IEC1bits.INT2IE = 1;	// Enable INT2 interrupt */
	
	IFS1bits.T4IF = 0;		// Clear Timer4 interrupt flag */
}

unsigned int tmr5=0;
double adc_sen3_ant=0;
double taza=0;
double enriquesimiento=1;
void __attribute__((__interrupt__,no_auto_psv)) _T5Interrupt(void){
	T5CONbits.TON = 0;		// TMR5 OFF */
	IEC1bits.T5IE = 0;		// Disable TMR5 interrupt */
	
	taza=adc_sen[3]-adc_sen3_ant;
	adc_sen3_ant=adc_sen[3];
	
	if(enriquesimiento_i==0){
		if((taza>0)&&(taza<5)){
			enriquesimiento=1;
		}else if((taza>=5)&&(taza<20)){
			enriquesimiento=1.7;
			empezar_enriquesimiento();
		}else if((taza>=20)&&(taza<=50)){
			enriquesimiento=1.8;
			empezar_enriquesimiento();
		}else if((taza>=50)&&(taza<=90)){
			enriquesimiento=1.9;
			empezar_enriquesimiento();
		}else
			enriquesimiento=1;
	}
	
	TMR5=tmr5;
	
	IFS1bits.T5IF = 0;		// Clear TMR5 interrupt flag */
	IEC1bits.T5IE = 1;		// Enable TMR5 interrupt */
	T5CONbits.TON = 1;		// TMR5 ON */
}
void tiempo_tmr5_tps(float t){
	tmr5=65535-(t*3685);
}

#include<ports.h>
void __attribute__((__interrupt__,no_auto_psv)) _INT2Interrupt(void){
	LATDbits.LATD2 = 1;		// iny ON */
	
	TMR1 = tmr1;
	IFS0bits.T1IF = 0;		// Clear Timer1 interrupt flag */
	T1CONbits.TON = 1;		// Timer1 ON */
	IEC0bits.T1IE = 1;		// Enable TMR1 interrupt */
	
	WriteTimer4(0);
	IFS1bits.T4IF = 0;		// Clear Timer4 interrupt flag */
	T4CONbits.TON = 1;		// Timer4 ON */
	IEC1bits.T4IE = 1;		// Enable TMR4 interrupt */
	
	IEC1bits.INT2IE = 0;	// Disable INT2 interrupt */
	IFS1bits.INT2IF = 0;	// Clear INT2 interrupt flag */
}

#define Debug_serial
#if defined(Debug_serial)
	#include<uart.h>
	#include<stdio.h>
	
	unsigned char option = 0;
	void __attribute__((__interrupt__,no_auto_psv)) _U1RXInterrupt(void){
		option = getcUART1();
		IFS0bits.U1RXIF = 0;	// Clear INT2 interrupt flag */
	}
	
	void debug_PICkit(void){ 
		if(option=='1')
			printf("1ECT %4u %1.3f %1.3f\n",adc[0],adc_vol[0],adc_sen[0]);
		else if(option=='2')
			printf("2IAT %4u %1.3f %1.3f\n",adc[1],adc_vol[1],adc_sen[1]);
		else if(option=='3')
			printf("3MAP %4u %1.3f %1.3f\n",adc[2],adc_vol[2],adc_sen[2]);
		else if(option=='4')
			printf("4TPS %4u %1.3f %1.3f\n",adc[3],adc_vol[3],adc_sen[3]);
		else if(option=='r'){
			if(ralenti) printf("nr\n");
			else printf("FR\n");
		}
		else if(option=='t')
			printf("mS %1.3f\n",tiempo_ms);
		else if(option=='q'){
			printf("1ECT %4u %1.3f %1.3f\n",adc[0],adc_vol[0],adc_sen[0]);
			printf("2IAT %4u %1.3f %1.3f\n",adc[1],adc_vol[1],adc_sen[1]);
			printf("3MAP %4u %1.3f %1.3f\n",adc[2],adc_vol[2],adc_sen[2]);
			printf("4TPS %4u %1.3f %1.3f\n",adc[3],adc_vol[3],adc_sen[3]);
			printf("mS %1.3f\n",tiempo_ms);
			if(enriquesimiento_i) printf("1\n");
			else printf("0\n");
		}
		else if(option=='e'){
			printf("t %1.3f t %1.3f e %1.3f",tiempo_ms,taza,enriquesimiento);
			if(enriquesimiento_i) printf("e1\n");
			else printf("e0\n");
		}
		option=0;
	}
#endif

int main(void){
	tiempo_ms=0.2;
	LATDbits.LATD2 = 0;		// iny OFF */
	
	TRISB=0xFFFF;
	TRISC=0b1011111111111111;// TRISCbits.TRISC14 = 0;//Bob
	TRISD=0b1111111111111011;// TRISDbits.TRISD1 = 1;//Cruze  TRISDbits.TRISD2 = 0;//Iny
	TRISE=0xFFFF;
	TRISF=0b1111111111110111;// TRISFbits.TRISF2 = 1;//RX     TRISFbits.TRISF3 = 0;//TX
	LATDbits.LATD2 = 0;		// iny OFF */
	
	OpenTimer1(	T1_OFF & T1_IDLE_CON &
				T1_GATE_OFF & T1_PS_1_8 &
				T1_SYNC_EXT_OFF & T1_SOURCE_INT,
				0xFFFF);//match_value
	ConfigIntTimer1(T1_INT_PRIOR_4 & T1_INT_OFF);//t iny
	LATDbits.LATD2 = 0;		// iny OFF */
	
	OpenTimer2(	T2_ON & T2_IDLE_CON &
				T2_GATE_OFF & T2_PS_1_64 &
				T2_32BIT_MODE_OFF & T2_SOURCE_INT,
				0xFFFF);//match_value
	ConfigIntTimer2(T2_INT_PRIOR_4 & T2_INT_ON);//bobina
	LATDbits.LATD2 = 0;		// iny OFF */
	
	OpenTimer3(	T3_ON & T3_IDLE_CON &
				T3_GATE_OFF & T3_PS_1_256 &
				T3_SOURCE_INT, 0xFFFF);//match_value
	ConfigIntTimer3(T3_INT_PRIOR_4 & T3_INT_ON);//arranque frio & enriquesimiento remanente
	LATDbits.LATD2 = 0;		// iny OFF */
	
	OpenTimer4(	T4_OFF & T4_IDLE_CON &
				T4_GATE_OFF & T4_PS_1_1 &
				T4_32BIT_MODE_OFF & T4_SOURCE_INT,
				0xFFFF);//match_value
	ConfigIntTimer4(T4_INT_PRIOR_4 & T4_INT_OFF);//ayuda t iny
	LATDbits.LATD2 = 0;		// iny OFF */
	
	OpenTimer5(	T5_ON & T5_IDLE_CON &
				T5_GATE_OFF & T5_PS_1_8 &
				T5_SOURCE_INT, 0xFFFF);//match_value
	ConfigIntTimer5(T5_INT_PRIOR_4 & T5_INT_ON);//tiempo verificacion tps
	LATDbits.LATD2 = 0;		// iny OFF */
	
	SetChanADC10(	ADC_CH0_POS_SAMPLEA_AN0 & ADC_CH0_NEG_SAMPLEA_VREFN);//Channel
	OpenADC10(	ADC_MODULE_ON & ADC_IDLE_CONTINUE &
				ADC_FORMAT_INTG & ADC_CLK_AUTO &
				ADC_SAMPLE_SIMULTANEOUS & ADC_AUTO_SAMPLING_ON,//Adcon1_reg dont care ADC_SAMPLE_SIMULTANEOUS
				ADC_VREF_AVDD_AVSS & ADC_SCAN_OFF &
				ADC_CONVERT_CH0 & ADC_ALT_BUF_OFF &
				ADC_SAMPLES_PER_INT_16 & ADC_ALT_INPUT_OFF,//Adcon2_reg
				ADC_SAMPLE_TIME_1 & ADC_CONV_CLK_SYSTEM & //ADC_SAMPLE_TIME_1
				ADC_CONV_CLK_2Tcy ,//Adcon3_reg  ADC_CONV_CLK_5Tcy .fast 2Tcy
				ENABLE_AN0_ANA & ENABLE_AN1_ANA &
				ENABLE_AN2_ANA & ENABLE_AN3_ANA,//PinConfig
				SCAN_NONE);//Scanselect
	ConfigIntADC10(ADC_INT_PRI_4 & ADC_INT_ENABLE);
	LATDbits.LATD2 = 0;		// iny OFF */
	
	#if defined(Debug_serial)
		OpenUART1(	UART_EN & UART_IDLE_CON & UART_RX_TX &
					UART_DIS_WAKE & UART_DIS_LOOPBACK  &
					UART_EN_ABAUD & UART_NO_PAR_8BIT  &
					UART_1STOPBIT,//U1MODEvalue
					UART_INT_TX_BUF_EMPTY  &  
					UART_TX_PIN_NORMAL &
					UART_TX_ENABLE & UART_INT_RX_CHAR &
					UART_ADR_DETECT_DIS &
					UART_RX_OVERRUN_CLEAR,//U1STAvalue
					47);//U2BRG = (Fcy/(16*BaudRate))-1 = (29,48M/(16*38400))-1 = 47//3800   //2800=657
		ConfigIntUART1(UART_RX_INT_EN & UART_RX_INT_PR4 &
					UART_TX_INT_DIS & UART_TX_INT_PR4);
		LATDbits.LATD2 = 0;		// iny OFF */
	#endif
	
	ConfigINT2(	RISING_EDGE_INT & EXT_INT_PRI_4 &
				EXT_INT_ENABLE & GLOBAL_INT_ENABLE);
	LATDbits.LATD2 = 0;		// iny OFF */
	
	tiempo_tmr3_enr(5.0);//each S
	
	tiempo_tmr5_tps(20);//each mS
	
	while(1){
		vol_val();
		
		if(ralenti){//programa normal despues de 5s //ADC0 ECT //ADC1 IAT //ADC2 MAP //ADC3 TPS
			
			adc_sen[0]=0.0006419*adc_vol[0]*adc_vol[0]-0.01155*adc_vol[0]+0.0756;//ECT;
			
			//adc_sen[1]=0.06*adc_vol[1];//IAT;
			adc_sen[1]=0.29;
			
			adc_sen[2]=0.613*adc_vol[2]-0.9;//MAP
			//adc_sen[2]=-0.0015*adc_vol[2]+0.1616;//MAP
			
			//adc_sen[2]=0.0875*(5-adc_vol[2])+0.06;
			//adc_sen[2]=0.295+0.09*(5-adc_vol[2]);//MAP
			//if(adc_vol[2]>=3.44)
			//	adc_sen[2]=0.16;
			//else if((adc_vol[2]>=3.4)&&(adc_sen[1]<=2.23))
			//	adc_sen[2]=0.155;
			
			if(adc_vol[3]<1)//TPS
				adc_sen[3]=0;//°
			else if(adc_vol[3]>3)
				adc_sen[3]=90;//°
			else
				adc_sen[3]=45*adc_vol[3]-45;//°
			
			tiempo_ms=(adc_sen[0]+adc_sen[1]+adc_sen[2])*enriquesimiento;
			
			//tiempo_ms=1.2;
			
			tiempo_tmr1_iny(tiempo_ms);	//envio del tiempo calculado
			
			//fprintf(PICkit,"  tmp %f",tiempo_ms);
			//tiempo_ms=tiempo_ms+tiempo_tps(adc_val[3],tiempo_ms);//tiempo enriquecimiento ya incluye TPS
			//fprintf(PICkit,"  tmpEN %f\r\n",tiempo_ms);
		}else{
			//programa para arranque en frio    //ADC0 ECT //ADC1 IAT //ADC2 MAP //ADC3 TPS
			adc_sen[0]=0.0006419*adc_vol[0]*adc_vol[0]-0.01155*adc_vol[0]+0.0756;//ECT
			
			adc_sen[1]=0.5;
			
			adc_sen[2]=-0.0153*adc_vol[2]+0.1769;
			/*
			if((adc_vol[2]>=3)&&(adc_vol[1]<4))//IAT
				//adc_sen[1]=0*adc_vol[3];//TPSa
				adc_sen[1]=0.0455*adc_vol[1];//IAT
			else if ((adc_vol[2]>=2.55)&&(adc_vol[2]<3))
				adc_sen[1]=0.168*adc_vol[1];//IAT
				//adc_sen[1]=1.5;
				//if((adc_vol[2]>=1.5)&&(adc_vol[2]<3.7))
				//adc_sen[2]=(1.25);
				//adc_sen[2]=0.275+0.05*(3-adc_vol[2]);//MAP
				
			if(adc_vol[2]>=3.43)//MAP
				adc_sen[2]=0.45;
			
			if((adc_vol[3]>=0.78)&&(adc_vol[2]<3.44))//TPS
				adc_sen[3]=2;
			else if((adc_vol[3]>=1)&&(adc_vol[2]<3))
				adc_sen[3]=2.5;
				//adc_sen[3]=3*adc_vol[3]+2;//TPS
			else if((adc_vol[3]>=1.5)&&(adc_vol[2]<3.44))
				adc_sen[3]=2.75;
			else if((adc_vol[3]>=2)&&(adc_vol[2]<2))
				adc_sen[3]=3;
				
				//adc_sen[3]=9*adc_vol[3]+5.5;//TPS
				//adc_sen[3]=2.5; 
				//adc_sen[3]=0.345*adc_vol[3];//TPS
				//if adc_vol[3]>=0)&&(adc_vol[2]<0.5))
				//adc_sen[3]=0.5;
				//else if((adc_vol[3]>=0.83)&&(adc_vol[2]<0.9))
				//adc_sen[3]=0.5;
				//else if((adc_vol[3]>=1)&&(adc_vol[2]<2))
				//adc_sen[3]=1.7;
				//else if((adc_vol[3]>=2)&&(adc_vol[2]<3))
				//adc_sen[3]=3.7;
				//else if((adc_vol[3]>=3)&&(adc_vol[2]<4))
				//adc_sen[3]=3.5;
			*/
			//tiempo_ms=adc_sen[0]+adc_sen[1]+adc_sen[2]+adc_sen[3];
			
			tiempo_ms=1.2;
			
			tiempo_tmr1_iny(tiempo_ms);	//envio del tiempo calculado
		}//FIN arranque en frio
		
		dutty_tmr2();
		
		#if defined(Debug_serial)
			debug_PICkit();
		#endif
	}
	return 0;
}
