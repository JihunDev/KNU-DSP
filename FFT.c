#include <math.h>  // math 헤더 파일 선언
#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"// MMR 선언에 해당하는 모든 헤더 파일 포함

#define	pi	3.141592654
#define Data_num 2
#define Data_siz Data_num*32
#define Lognpt 10

#define Buffer_size Data_siz

#define Signal_A		15// 수식에 필요한 변수 설정

#define ADC_MODCLK 0x3
#define ADC_CKPS   0x1
#define ADC_SHCLK  0xF// A/D정의


volatile float Data_buffer1[Buffer_size]={0,};
float Data_buffer2[Buffer_size]={0,};
volatile Uint16 FFT_flag=0,write_flag=1;
int	inv = 0;
float imag[Data_siz];

interrupt void cpu_timer0_isr(void)
{
	static Uint16 i=0;
	float Temp;

	while(AdcRegs.ADCST.bit.INT_SEQ1== 0);
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;// 인터럽트 0.1ms 설정
	
	Temp = (float)AdcMirror.ADCRESULT0;
	if(write_flag){
		Data_buffer1[i] = Temp-2048;// A/D 변환한 값 을 Data_buffer1에 저장
		
			Data_buffer2[i] = Data_buffer1[i]; //A/D 변환값을 Data_buffer2에 저장

		if(i==Buffer_size-1){
			i=0;
			write_flag=0; // 저장 종료
			FFT_flag=1; // FFT 실행
		}// Data_buffer1와 Data_buffer2에 64개의 데이터가 저장되면 FFT 함수 실행
		else
			i++;
	}

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;// Pie control Register가 다음 인터럽트가 올 것을 대기한다.
}

void fft(float *Real, float *Imag)
{
	int n,i,j,k,it,xp,xp2,j1,j2,iter;
	double sign, w, wr, wi, dr1, dr2, di1, di2, tr, ti, arg;
	n=Data_siz;
	if (inv==1){
		sign=1.0;  
	}else{
		sign=-1.0;
	}
	// 1 => fourier transform, -1 => inverse Fourier transform
	iter=Lognpt;                     //  Lognpt=log2(Data_siz)
	xp2=n;
	for(it=0;it<iter;it++) {
		xp=xp2;
		xp2/=2;
		w=pi/xp2;
		for(k=0;k<xp2;k++) {
			arg=k*w;
			wr=cos(arg);
			wi=sign*sin(arg);
			i=k-xp;
			for(j=xp;j<=n;j+=xp) {
				j1=j+i;
				j2=j1+xp2;
				dr1=Real[j1];
				dr2=Real[j2];
				di1=Imag[j1];
				di2=Imag[j2];
				tr=dr1-dr2;
				ti=di1-di2;
				Real[j1]=dr1+dr2;
				Imag[j1]=di1+di2;
				Real[j2]=tr*wr-ti*wi;
				Imag[j2]=ti*wr+tr*wi;
			}
		}
	}

	j1=n/2;
	j2=n-1;
	j=1;
	for(i=1;i<=j2;i++) {
		if(i<j) {
			tr=Real[j-1];
			ti=Imag[j-1];
			Real[j-1]=Real[i-1];
			Imag[j-1]=Imag[i-1];
			Real[i-1]=tr;
			Imag[i-1]=ti;
		}
		k=j1;
		while(k<j) {
			j-=k;
			k/=2;
		}
		j+=k;
	}

	if (inv==1) {
		w=n;
		for(i=0;i<n;i++) {
			Real[i]/=w;
			Imag[i]/=w;
		}
	}
}

void Absolute(float *Real, float *Imag)
{
	int i;
	for(i=0; i <Data_siz ; i++){
		Real[i] = sqrt(Real[i]*Real[i] + Imag[i]*Imag[i]);
		Real[i] = Real[i]*2/(Data_siz);
       	}
	Real[0]=Real[0]/2;         
	Real[1]=Real[1]/2;
}

void main()
{
	Uint16 i;

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

	InitCpuTimers();//Cpu Timers 초기화
	ConfigCpuTimer(&CpuTimer0, 150, 100); // CpuTimer 0 레지스터의 의 주파수와 주기를 설정

	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;// PIE 활성화
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1; //타이머 벡터 테이블 그룹 1에 있는 CPU Timer 0의 인터럽트 신호 TINT0활성화
	IER |= M_INT1; // CPU INT1 레지스터를 CpuTimer 0 와 연동
	EINT; // Enable Global interrupt INTM  인터럽트 발생을 허용

	AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;// ADC 실행
	   
	for(;;){
		if(FFT_flag){
			for(i=0;i<Data_siz;i++)
				imag[i] = 0.0;

			for(i=0;i<Data_siz;i++)
				Data_buffer2[i]=Data_buffer2[i+1];
			fft(&Data_buffer2[0], &imag[0]);
			Absolute(&Data_buffer2[0],&imag[0]);

			write_flag=1; // 저장 시작
			FFT_flag=0;// FFT실행 종료
		}
	}
}


