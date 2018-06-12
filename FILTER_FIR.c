#include <math.h>// math 헤더 파일 선언
#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"// MMR 선언에 해당하는 모든 헤더 파일 포함

#pragma DATA_SECTION (Data_buffer1, "DMARAML5");
#pragma DATA_SECTION (Data_buffer2, "DMARAML6");

#define Buffer_size 128

#define Order 46

#define Signal_A		15// 수식에 필요한 변수 설정

#define ADC_MODCLK 0x3
#define ADC_CKPS   0x1
#define ADC_SHCLK  0xF


volatile float Data_buffer1[Buffer_size]={0,};
volatile float Data_buffer2[Buffer_size]={0,};

float X[Order];
float BCOEF[] = {
	-0.00145272028471,
	-0.00270758394275,
	-0.00245033875536,
	0.00000000000000,
	0.00377611571529,
	0.00648470614096,
	0.00550798963455,
	-0.00000000000000,
	-0.00775654108870,
	-0.01290393439808,
	-0.01069307945820,
	0.00000000000000,
	0.01460903867510,
	0.02416752838723,
	0.02005506870416,
	-0.00000000000000,
	-0.02821875567570,
	-0.04826471394240,
	-0.04220846050610,
	0.00000000000000,
	0.07347074225903,
	0.15796848353036,
	0.22520861391308,
	0.25081568218450,
	0.22520861391308,
	0.15796848353036,
	0.07347074225903,
	0.00000000000000,
	-0.04220846050610,
	-0.04826471394240,
	-0.02821875567570,
	-0.00000000000000,
	0.02005506870416,
	0.02416752838723,
	0.01460903867510,
	0.00000000000000,
	-0.01069307945820,
	-0.01290393439808,
	-0.00775654108870,
	-0.00000000000000,
	0.00550798963455,
	0.00648470614096,
	0.00377611571529,
	0.00000000000000,
	-0.00245033875536,
	-0.00270758394275
};

interrupt void cpu_timer0_isr(void)
{
	static Uint16 i=0;
	float temp_cal=0,Temp;
	Uint16 j; 

	while(AdcRegs.ADCST.bit.INT_SEQ1== 0);
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1; // 인터럽트 0.1ms 설정
	
	Temp = (float)AdcMirror.ADCRESULT0;

	Data_buffer1[i] = Temp-2048; // A/D 변환한 값 을 Data_buffer1에 저장

	X[0] = Data_buffer1[i];

	for(j=0;j<Order;j++)
		temp_cal += (BCOEF[j]*X[j]); // j가 0~45번 반복

	Data_buffer2[i] = temp_cal; // Data_buffer2에 temp_cal값을 저장

	for(j=0;j<Order-1;j++)
		X[Order-j+1] = X[Order-(j+2)];

	if(i==Buffer_size-1)
		i=0;
	else
		i++;

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;// Pie control Register가 다음 인터럽트가 올 것을 대기한다.
}

void main()
{

	InitSysCtrl();// Sys Ctrl 레지스터 초기화

	EALLOW;// 쓰기 금지 된 레지스터 보호막 해제
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
	PieVectTable.TINT0 = &cpu_timer0_isr;// PIE 백터 테이블의 TINT0 을 함수 cpu_timer0_isr와 연결하여 인터럽트  발생시함수가 호출 되도록 설정
	EDIS;

	InitCpuTimers(); //Cpu Timers 초기화
	ConfigCpuTimer(&CpuTimer0, 150, 100); // CpuTimer 0 레지스터의 의 주파수와 주기를 설정

	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;// PIE 활성화
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;//타이머 벡터 테이블 그룹 1에 있는 CPU Timer 0의 인터럽트 신호 TINT0활성화
	IER |= M_INT1; // CPU INT1 레지스터를 CpuTimer 0 와 연동
	EINT;// Enable Global interrupt INTM  인터럽트 발생을 허용

	AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;// ADC 실행
	   
	for(;;);
	
}

