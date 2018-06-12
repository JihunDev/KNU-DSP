#include <math.h>// math 헤더 파일 선언
#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"// MMR 선언에 해당하는 모든 헤더 파일 포함

#pragma DATA_SECTION (Data_buffer1, "DMARAML5");
#pragma DATA_SECTION (Data_buffer2, "DMARAML6");

#define Buffer_size 64

#define	pi	3.141592654
#define Data_num 2
#define Data_siz Data_num*32
	
#define Beta  0.000000
#define Lognpt 10

#define Signal_A		15 // 수식에 필요한 변수 설정

#define ADC_MODCLK 0x3
#define ADC_CKPS   0x1
#define ADC_SHCLK  0xF // A/D정의

#define SelectWindow	0

volatile float Data_buffer1[Buffer_size]={0,};
volatile float Data_buffer2[Buffer_size]={0,};

float	Window[Data_siz];
float 	facto[]={
 	0,
 	1,
 	2,
	6,
	24,
	120,
	720,
	5040,
	40320,
	362880,
	3628800,
	39916800,
	479001600,
	6.227020800E9,
	8.71782912E10,
	1.307674368E12,
	2.092278989E13,
	3.556874281E14,
	6.402373706E15,
	1.216451004E17,
	2.432902008E18,
	5.109094217E19,
	1.124000728E21,
	2.585201674E22,
	6.204484017E23,
	1.551121004E25,
	4.032914611E26,
	1.088886945E28
};

interrupt void cpu_timer0_isr(void)
{
	static Uint16 i=0;
	float Temp; 

	while(AdcRegs.ADCST.bit.INT_SEQ1== 0);
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;// 인터럽트 0.1ms 설정
	
	Temp = (float)AdcMirror.ADCRESULT0;

	Data_buffer1[i] = Temp-2048;// A/D 변환한 값 을 Data_buffer1에 저장

	Data_buffer2[i] = Window[i%64] * Data_buffer1[i];// Data_buffer1 값과 window에 저장된 값을 곱하여 Data_buffer2 저장

	if(i==Buffer_size-1)
		i=0;
	else
		i++;

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

double Bessel(double x)
{
	double i;
	double Sum=1;

	for(i=1;i<=27;i++){

		Sum += pow( (pow((x/2),i))/facto[(int)i] ,2);
	}

	return Sum;
}

double Kaiser(double beta,double n)
{
	double Ia,Ib,IX;

	Ia = Bessel(beta);
	IX = beta * sqrt( 1- pow( (1 - ((2*(n-1))/(Data_siz-1))),2)  );
	Ib = Bessel(IX);

	IX = Ib/Ia;
	return IX;
}

void MakeWindow(float window[],int SelectW, int beta)
{
	int i;

	if( SelectW == 0){    		// Rectangular Window
		for(i=0;i<Data_siz;i++){
			window[i] = 1;
		}
	}
	else if( SelectW == 1){  		// Hanning Window
		for(i=0;i<Data_siz;i++){
			window[i]= 0.5-0.5*cos((2*pi*i)/(Data_siz-1));
		}
	}
	else if( SelectW == 2){  		// Hamming Window
		for(i=0;i<Data_siz;i++){
			window[i] = 0.54-0.46*cos((2*pi*i)/(Data_siz-1));
		}
	}
	else if( SelectW == 3){  		// Blackman Window
		for(i=0;i<Data_siz;i++){
			window[i] = 0.42-0.5*cos((2*pi*i)/(Data_siz-1))+0.08*cos((4*pi*i)/(Data_siz-1));
		}
	}
	else if( SelectW == 4){  		// Bartlett Window
		for(i=0;i<(Data_siz/2);i++)
			window[i] = (2*i) / (Data_siz-1);
		for(i=(Data_siz/2);i<Data_siz;i++)
			window[i] = 2 - ((2*i) / (Data_siz-1));
	}
	else if( SelectW == 5){		// Kaiser Window
		for(i=0;i<Data_siz;i++)
			window[i] = Kaiser(Beta,i);
		}
	else{}
}// window값을 결정한는 함수 SelectWindow에 값에 따라 결정됨

void main()
{
	MakeWindow(Window,SelectWindow, Beta); //MakeWindow 함수
    
    InitSysCtrl();// Sys Ctrl 레지스터 초기화

	EALLOW; // 쓰기 금지 된 레지스터 보호막 해제
	SysCtrlRegs.HISPCP.all = ADC_MODCLK;// HSPCLK 분주 설정
	EDIS;// 보호막을 설정해 안정적으로 돌아가도록 설정

	DINT;

	InitPieCtrl();

	IER = 0x0000;// Interrupt Enable Register 초기화
	IFR = 0x0000;// Interrupt Flag Register 초기화

	InitPieVectTable();// Pie Vect Table 초기화

	InitAdc();// adc 초기화

	AdcRegs.ADCTRL1.bit.ACQ_PS = ADC_SHCLK;// 샘플 홀드 사이클 설정
	AdcRegs.ADCTRL3.bit.ADCCLKPS = ADC_CKPS;// ADC 클럭분주기 설정
	AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;// 직렬 SEQ 모드
	AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 0;// ADC 채널수 1개 설정
	AdcRegs.ADCCHSELSEQ1.bit.CONV00 = Signal_A;// 변환채널 SIgnal_A설정
	AdcRegs.ADCTRL1.bit.CONT_RUN = 1;// 설정 연속 실행

	EALLOW;
	PieVectTable.TINT0 = &cpu_timer0_isr; // PIE 백터 테이블의 TINT0 을 함수 cpu_timer0_isr와 연결하여 인터럽트  발생시함수가 호출 되도록 설정
	EDIS;

	InitCpuTimers(); //Cpu Timers 초기화
	ConfigCpuTimer(&CpuTimer0, 150, 100); // CpuTimer 0 레지스터의 의 주파수와 주기를 설정

	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;// PIE 활성화
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;//타이머 벡터 테이블 그룹 1에 있는 CPU Timer 0의 인터럽트 신호 TINT0활성화
	IER |= M_INT1; // CPU INT1 레지스터를 CpuTimer 0 와 연동
	EINT; // Enable Global interrupt INTM  인터럽트 발생을 허용

	AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;// ADC 실행


	for(;;);

}
