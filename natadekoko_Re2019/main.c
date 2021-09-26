/* 191202 �f�W�^��x3 �A�i���Ox2�@�Z���T�ɉ��ҍς� */
/* 191206 3.2[m/s]�ȏ�o����悤�ύX */

/* 200315 ThunderBird-XE.ver.0 �}�C�R���J�[�d�l */

/*======================================*/
/* �C���N���[�h                         */
/*======================================*/
#include <stdio.h>
#include <stdlib.h>
#include "sfr_r838a.h"                  /* R8C/38A SFR�̒�`�t�@�C��    */
#include "types3_beep.h"                /* �u�U�[�ǉ�                   */
#include "printf_lib.h"

/*======================================*/
/* �V���{����`                         */
/*======================================*/
/* �萔�ݒ� */

// �ύX�֎~:
#define     TRC_MOTOR_CYCLE     20000   /* ���O,�E�O���[�^PWM�̎���     */
/* 50[ns] * 20000 = 1.00[ms]    */
#define     TRD_MOTOR_CYCLE     20000   /* ����,�E��,����Ӱ�PWM�̎���   */
/* 50[ns] * 20000 = 1.00[ms]    */

#define     FREE        1   /* ���[�^���[�h�@�t���[         */
#define     BRAKE       0   /* ���[�^���[�h�@�u���[�L       */\

#define     HIGH        1   /* 5V����           */
#define     LOW         0   /* 0V����           */

#define     ON          1
#define     OFF         0

#define     RIGHT       p1_addr.bit.b6
#define     LEFT        p0_addr.bit.b0
#define 	CENTER 		p0_addr.bit.b3

#define A 3
#define B 4
#define C 5
#define D 6

#define VR  ad2

// �ύXOK:
/* dipsw:�@4-6 */
#define  	KP 	10 	      /* �X�e�A�����O_���  */
#define		KD 	100 	      /* �X�e�A�����O_����  */

#define  	L_KP  	3      /* ��_���  */
#define 	L_KD  	5      /* ��_����  */

#define 	BEEP 				1
#define 	ENC  				1   	/* �G���R�[�_���L���� 1:�L�� 0:����	*/
#define     ENCHU_ENABLE        0       /* �~���W�I�̗L����				*/

#define     RUNNING_TIME        31000L  /* ���s����(ms)                 */
#define		ENC_END				210L*(dipsw_get())/* ���s����(cm) */ 

#define     AD_1DEG             1       /* 1�x�������A/D�l�̑���       */

#define  	SERVO_PWM_MAX   	100    	/* �T�[�{�̍ő�PWM        */
#define		LANCER_PWM_MAX		0		/* ���̍ő�PWM  		*/

#define     CENTER_LNC          512		/* ���̒��S��A/D�l		*/         

#define  	ROLLING  			775 /* �J�[�u���m��臒l */

/* �I�̊p�x */
volatile int     ENCHU_ANGLE_AD = 800;			 /* �~���W�I�ɓ˂����ލۂ̃{�����[���l */ //786
volatile int     ENCHU_ANGLE = 9;			 /* �~���W�I�ɓ˂����ލۂ̃{�����[���l */

#define     UP4BIT /* dipsw2��4bit */ ( dipsw_get2() >> 4 )  /* ���s�W�I�ɓ��Ă�܂ł̋���: */
#define     MIN4BIT/* dipsw2��4bit */ ( dipsw_get2()&0x0f )  /* �n�[�t���C�����o����ʉ߂܂ł̃p���X��(272.5*2.5=25cm) */

// �T�[�{�p
// Arduino nano�Ƃ̐ڑ�:(�o��)
#define     SERVO_ANGLE_PORT    p5_addr.bit.b3
#define     SERVO_ZERO_PORT     p5_addr.bit.b2
#define     SERVO_MODE_PORT     p5_addr.bit.b1

#define     DCM_DIR_PORT		p6_addr.bit.b6
#define     DCM_PWM_PORT        p0.addr.bit.b7

/*======================================*/
/* �v���g�^�C�v�錾                     */
/*======================================*/
void init( void );
void timer( unsigned long timer_set );
unsigned char sensor_inp( void );
unsigned char center_inp( void );
unsigned char dipsw_get( void );
unsigned char dipsw_get2( void );
unsigned char pushsw_get( void );
unsigned char convertBCD( int data );
void led_out( unsigned char led );
void servoSet( int num );
void fullColor_out( unsigned char type );
void traceMain( void );
void motor_r( int accele_l, int accele_r );
void motor2_r( int accele_l, int accele_r );
void motor_f( int accele_l, int accele_r );
void motor2_f( int accele_l, int accele_r );
void motor_mode_r( int mode_l, int mode_r );
void motor_mode_f( int mode_l, int mode_r );
void servoPwmOut( int pwm );
void servoControl( void );
void servoControl2( void );
int check_crossline( void );
int check_rightline( void );
int check_leftline( void );
unsigned char startbar_get( void );
unsigned char cn6_get( void );
int getServoAngle( void );
int getAnalogSensor( void );
int diff( int pwm );
int getLancerAngle( void );
void hyouteki_check( void );
void lancerPwmOut( int pwm );
void lancerPwmOut_ABS( int pwm );
void lancerControl( void );
long map( long x, long in_min, long in_max, long out_min, long out_max );
void newcrank( void );
void sp( int l , int r );
void sakaSyori( void );

/*======================================*/
/* �O���[�o���ϐ��̐錾                 */
/*======================================*/

/* �f�[�^�t���b�V���֘A */

volatile const char      *C_DATE = __DATE__;     /* �R���p�C���������t           */
volatile const char      *C_TIME = __TIME__;     /* �R���p�C����������           */

volatile int             pattern       = 0;      /* �}�C�R���J�[����p�^�[��     */
volatile int             pattern_settings = 0;
volatile int             crank_mode = 0;
volatile int  			 iSensorPattern=0;
volatile unsigned long   cnt0          = 0;
volatile unsigned long   cnt_run       = 0;      /* �^�C�}�p                     */
volatile unsigned long   cnt1          = 0;      /* �^�C�}�p                     */
volatile unsigned long   check_cross_cnt = 0;    /* �^�C�}�p                     */
volatile unsigned long   check_sen_cnt = 0;      /* �^�C�}�p                     */
volatile unsigned long   check_enc_cnt = 0;      /* �^�C�}�p                     */
volatile int             anchi_cross = 0;        /* 1:�O��ɕЕ��̃f�W�^���Z���T������ 0:�f�W�^���̔����Ȃ� */
volatile int             hitcount      = 0;      /* �n�[�t���C����ǂ񂾉�     */
volatile int             hyouteki_flag = 0;      /* �W�I�����������s������������(���s�W�I:0 �����W�I:1)*/
volatile int             heikou  = 0;
volatile int             stair_flag  = 0;        /* 1:�傫���Ȃ����Ă��� 0: ���� */
volatile int	   	     kyori_flug = 0;             	/* �e���C���̒ʉߌ��o�錾  	 �X�^�[�g�O�̏����ݒ�   */
volatile int	   		 kyoritime  = 0;             	/* �e���C���̒ʉߎ��Ԑ錾    �X�^�[�g�O�̏����ݒ�   */
volatile unsigned long	 cross_cnt = 0;			 /*  */


/* �G���R�[�_�֘A     */
volatile int             iTimer10     = 0;       /* 10ms�J�E���g�p               */
volatile int             iEncoder     = 0;       /* 10ms���̍ŐV�l               */
volatile int             iEncoderMax  = 0;       /* ���ݍő�l                   */
volatile long            lEncoderLine = 0;       /* ���C�����o���̐ώZ�l       */
volatile long            lEncoderTotal = 0;      /* �ώZ�l�ۑ��p                 */
volatile unsigned int    uEncoderBuff = 0;       /* �v�Z�p�@���荞�ݓ��Ŏg�p     */

/* �T�[�{�֘A       */
volatile int             iSensorBefore;          /* �O��̃Z���T�l�ۑ�           */
volatile int             iServoPwm;              /* �T�[�{PWM�l                */
volatile int             iAngle0;                /* ���S����A/D�l�ۑ�            */

/* �T�[�{�֘A2          */
volatile int             iSetAngle;
volatile int             iSetAngleAD;
volatile int             iAngleBefore2;
volatile int             iServoPwm2;

/* ��(DC���[�^�ƃ{�����[��ad7) */
volatile int             iLancer0;				 /* ���S����A/D�l�ۑ� 			 */
volatile int			 iSetLancer;			 /* �ڕW��getLancerAngle()�̒l   */
volatile int 			 iSetLancerAD;			 /* �ڕW��AD�l					 */
volatile int			 iLancerPwm;			 
volatile int			 iLancerBefore;

/* TRC���W�X�^�̃o�b�t�@ */
volatile unsigned int    trcgrb_buff;            /* TRCGRB�̃o�b�t�@             */
volatile unsigned int    trcgrd_buff;            /* TRCGRD�̃o�b�t�@             */
volatile unsigned int    trcgrc_buff;

/* ���[�^�h���C�u���TypeS Ver.3���LED�A�f�B�b�v�X�C�b�`���� */
volatile unsigned char   types_led;              /* LED�l�ݒ�                    */
volatile unsigned char   types_dipsw;            /* �f�B�b�v�X�C�b�`�l�ۑ�       */

/* ���֍��l�v�Z�p�@�e�}�C�R���J�[�ɍ��킹�čČv�Z���ĉ����� */
volatile const int revolution_difference[] = {   /* �p�x������ցA�O�։�]���v�Z */			
    100, 98, 97, 95, 94, 			
    92, 91, 89, 88, 86, 			
    85, 83, 82, 81, 79, 			
    78, 76, 75, 74, 72, 			
    71, 70, 68, 67, 66, 			
    64, 63, 62, 60, 59, 			
    58, 56, 55, 54, 52, 			
    51, 50, 48, 47, 45, 		
    44, 42, 41, 40, 38, 			
    37, 36, 35, 34, 33 
};		

/* �⓹���o�p */
unsigned long cnt_saka; /* �⓹���o�p�^�C�} */
int saka_flag; /* 1:�⓹�� 0:�⓹�ł͂Ȃ� */
int saka0_ad; /* ���n�̍⓹�{�����[��A/D�l */

// 0��0*2+25=25(2.3m/s) 8�� 8*2+25=41(3.7m/s)
// 1��1*2+25=27(2.5m/s) 9�� 9*2+25=43(3.9m/s)
// 2��2*2+25=29(2.6m/s) 10��10*2+25=45(4.1m/s)
// 3��3*2+25=31(2.8m/s) 11��11*2+25=47(4.3m/s)
// 4��4*2+25=33(3.0m/s) 12��12*2+25=49(4.5m/s)
// 5��5*2+25=35(3.2m/s) 13��13*2+25=51(4.6m/s)
// 6��6*2+25=37(3.4m/s) 14��14*2+25=53(4.8m/s)
// 7��7*2+25=39(3.5m/s) 15��15*2+25=55(5.0m/s)

/************************************************************************/
/* ���C���v���O����                                                     */
/************************************************************************/
void main( void )
{
  int i, ret;
  unsigned char b;
  
  /* �f�[�^�t���b�V�������p */
  int r;
  unsigned char c;
  /**************************/
	
  /* �}�C�R���@�\�̏����� */
  init();                             /* ������                       */
  _asm(" FSET I ");                   /* �S�̂̊��荞�݋���           */
  initBeepS();                        /* �u�U�[�֘A����               */
  init_uart0_printf( SPEED_9600 );    /* UART0��printf�֘A�̏�����    */
  
  /* �}�C�R���J�[�̏�ԏ����� */
  motor_mode_f( BRAKE, BRAKE );
  motor_mode_r( BRAKE, BRAKE );
  motor2_f( 0, 0 );
  motor2_r( 0, 0 );
  servoPwmOut( 0 );
  led_out(0x00);
  setBeepPatternS( 0x8000 );
  timer(10);
  servoSet( 0 ); 
  ///////////////////////////////

/*
	while(1){  
		// ���E���x�@
		KP = dipsw_get()*3;
		servoPwmOut(iServoPwm);
		if(cnt1 >= 1 ){
			printf("%ld,%d,%d\n",cnt0,KP,ad2);
			cnt1 = 0;
		}	
//  lancerPwmOut(0);  // ��~
//	timer(1000);
  	}
*/
/*
	// �w��p�x�ւ̐���e�X�g
	iAngle0 = getServoAngle();
	while(1){
		iSetAngle = 30;
		servoPwmOut(iServoPwm2);	
	}
*/
	printf("time[ms],pattern,sensor_inp(),center_inp()\n");
	while(1){
		b = sensor_inp();
		printf("%4ld,%4d,%4d,%4d\n",cnt_run,pattern,b,center_inp());
		/*
		b = sensor_inp();
		if( b == 0x0f || b == 0x0d || b == 0x0b ) {
			if( check_sen_cnt >= 100 ){
				pattern = 103;
			}
		} else {
			check_sen_cnt = 0;
		}
		*/
		switch( pattern ){
		case 0:
    	    /* �v�b�V���X�C�b�`�����҂� */
        	servoPwmOut( 0 );
       	 	if( pushsw_get() ) {
            	setBeepPatternS( 0xcc00 );
            	cnt0 = 0;
				cnt1 = 0;
    	        pattern = 1;
        	    break;
	        }
    	    i =  (cnt1/200) % 2 + 1;
        	if( startbar_get() ) {
            	i += (( cnt1/100 ) % 2 + 1) << 2;
        	}
        	led_out( i );                   /* LED�_�ŏ���                  */
			break;

    	case 1:
        	/* �X�^�[�g�o�[�J�҂� */
	        servoPwmOut( iServoPwm / 2 );
    	    if( !startbar_get() ) {
        	    timer(5000);
				led_out( 0x00 );
            	cnt1 = 0;
				cnt0 = 0;
            	check_sen_cnt = 0;
            	check_enc_cnt = 0;
				lEncoderTotal = 0;
				pattern = 11;
				iAngle0 = getServoAngle();
            	break;
        	}
        	led_out( 1 << (cnt1/50) % 4 );
        	break;

    	case 11:
       		/* �ʏ�g���[�X */
        	servoPwmOut( iServoPwm );		// �T�[�{����		
			i = getServoAngle();
			
			if(lEncoderTotal >= ENC_END ){
				pattern = 103;	
			}
/*      
	    if( i > 110 ) {
			motor_mode_f( BRAKE, BRAKE );
			motor_mode_r( BRAKE, BRAKE );
            motor_f( 0, 0 );
            motor_r( 0, 0 );
        } else if( i > 10 ) {
			motor_mode_f( FREE, BRAKE );
			motor_mode_r( FREE, BRAKE );
            motor_f( diff(70), 70 );
            motor_r( diff(50), 50 );
        } else if( i < -110 ) {
            motor_f( 0, 0 );
            motor_r( 0, 0 );
        } else if( i < -10 ) {
			motor_mode_f( BRAKE, FREE );
			motor_mode_r( BRAKE, FREE );
            motor_f( 70, diff(70) );
            motor_r( 50, diff(50) );
        } else {
			motor_mode_f(BRAKE,BRAKE);
			motor_mode_r(BRAKE,BRAKE);
            motor_f( 100, 100 );
            motor_r( 100, 100 );
        }
*/

			if( i > 70 ){
				motor_mode_f( BRAKE, BRAKE );
				motor_mode_r( BRAKE, BRAKE );
				if( iEncoder >= 33 ){
					motor2_f( -20, -20 );
					motor2_r( 0, 0 );	
				}
				else if( iEncoder >= 22 ){
					motor2_f( 0, 0 );
					motor2_r( 0, 0 );
				}else{
					motor2_f( diff(60), 60 );
					motor2_r( diff(30), 30 );
				}
			}else if( i < -70 ){
				motor_mode_f( BRAKE, BRAKE );
				motor_mode_r( BRAKE, BRAKE );
				if( iEncoder >= 33 ){
					motor2_f( -20, -20 );
					motor2_r( 0, 0 );	
				}
				else if( iEncoder >= 22 ){
					motor2_f( 0, 0 );
					motor2_r( 0, 0 );
				}else{
					motor2_f( diff(60), 60 );
					motor2_r( diff(30), 30 );
				}	
			}else if( i > 15 ){
				motor_mode_f( FREE, BRAKE );
				motor_mode_r( FREE, BRAKE );
				if( iEncoder >= 33 ){
					motor2_f( -50, -50 );
					motor2_r( -30, -30 );	
				}else if( iEncoder >= 22 ){
					motor2_f( 0, 70 );
					motor2_r( 0, 50 );	
				}else {
					motor2_f( diff(70), 70 );
					motor2_r( diff(50), 50 );	
				}	
			}else if( i < -15 ){
				motor_mode_f( BRAKE, FREE );
				motor_mode_r( BRAKE, FREE );
				if( iEncoder >= 33 ){
					motor2_f( -50, -50 );
					motor2_r( -30, -30 );	
				}else if( iEncoder >= 22 ){
					motor2_f( 70, 0 );
					motor2_f( 50, 0 );	
				}else {
					motor2_f( 70, diff(70) );
					motor2_r( 50, diff(50) );	
				}		
			}else{
				
				if( iEncoder >= dipsw_get()*2 + 25 ){
					motor_mode_f( BRAKE, BRAKE );
					motor_mode_r( BRAKE, BRAKE );
					motor2_f( 0, 0 );
					motor2_r( 0, 0 );
				}	
				else{
					motor_mode_f( BRAKE, BRAKE );
					motor_mode_r( FREE, FREE );
					motor_f( 100, 100 );
					motor_r( 100, 100 );
				}
			}
	 		if( check_crossline() ) {       /* �N���X���C���`�F�b�N         */
				setBeepPatternS(0xa000);
				led_out(0xff);
				cnt1 = 0;
        		crank_mode = 1;
	            pattern = 21;
				break;
			}
	        if( check_rightline() ) {       /* �E�n�[�t���C���`�F�b�N       */
				setBeepPatternS(0xc000);
        	    led_out(0xcc);
				cnt1 = 0;
            	crank_mode = 1;
	            pattern = 50;
				break;
        	}
			if( check_leftline() ) {		/* ���n�[�t���C���`�F�b�N       */
				setBeepPatternS(0xa000);
				led_out(0xaa);
				cnt1 = 0;
				crank_mode = 1;
				pattern = 60;
				break; 	
			}
        	break;

	    case 21:
    	    /* �N���X���C���ʉߏ��� */
        	servoPwmOut( iServoPwm );
	        led_out( 0xff );
			motor_mode_f( BRAKE, BRAKE );
			motor_mode_r( BRAKE, BRAKE );
        	if( iEncoder >= 11 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
            	motor2_f( -80, -80 );
            	motor2_r( -50, -50 );
	        } else {
    	        motor2_f( 50, 50 );
        		motor2_r( 60, 60 );
        	}
	        if( lEncoderTotal - lEncoderLine >= 109L ) {
    	        cnt1 = 0;
        	    pattern = 22;
        	}	
	        break;

    	case 22:
        	/* �N���X���C����̃g���[�X�A���p���o���� */
	        servoPwmOut( iServoPwm );
    	    motor_mode_f( BRAKE, BRAKE );
			motor_mode_r( BRAKE, BRAKE );
			if( iEncoder >= 11 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
        	    motor2_f( -80, -80 );
    	        motor2_r( -50, -50 );
	        } else {
    	        motor2_f( 50, 50 );
        	    motor2_r( 60, 60 );
        	}
		
    	    if( center_inp() == 1 && (sensor_inp()&0x01) == 0x01 ) { /* �E�N�����N�H             */
        	    led_out( 0x1 );
            	cnt1 = 0;
	            pattern = 31;
    	        break;
        	}
	        if( center_inp() == 1 && (sensor_inp()&0x08) == 0x08 ) {  /* ���N�����N�H            */
    	        led_out( 0x2 );
        	    cnt1 = 0;
            	pattern = 41;
            	break;
        	}
        	break;

	    case 31:
    	    /* �E�N�����N���� */
        	servoPwmOut( 100 );         /* �U�肪�ア�Ƃ��͑傫������       */
			motor_mode_f(BRAKE,FREE);
    	    motor_mode_r(BRAKE,FREE);
			// diff�g�p�֎~!!������
			motor2_f( 100, 33 );          /* ���̕����́u�p�x�v�Z(4WD��).xls�v*/
        	motor2_r( 70, 22 );          /* �Ōv�Z                           */
			if( sensor_inp() == 0x04 ) {    /* �Ȃ��I���`�F�b�N           */
            	cnt1 = 0;
 	            iSensorPattern = 0;
				lEncoderLine = lEncoderTotal;
    	        crank_mode = 0;
         	    pattern = 32;
        	}
        	break;

	    case 32:
    	    /* �������Ԃ��o�܂ő҂� */
        	servoPwmOut( iServoPwm );
			motor_mode_f( BRAKE, BRAKE );
			motor_mode_r( BRAKE, BRAKE );
    	    motor2_r( 80, 80 );
        	motor2_f( 80, 80 );
	        if( lEncoderTotal - lEncoderLine >= 109L ) {
    	        cnt1 = 0;
        	    led_out( 0x0 );
        	    pattern = 11;
        	}	
        	break;

	    case 41:
    	    /* ���N�����N���� */
        	servoPwmOut( -100 );         /* �U�肪�ア�Ƃ��͑傫������       */
	        motor_mode_f(FREE,BRAKE);
    	    motor_mode_r(FREE,BRAKE);
			// diff�g�p�֎~!!������
			motor2_f( 33, 100 );          /* ���̕����́u�p�x�v�Z(4WD��).xls�v*/
        	motor2_r( 22, 70 );           /* �Ōv�Z                           */
        	if( sensor_inp() == 0x02 ) {    /* �Ȃ��I���`�F�b�N           */
            	cnt1 = 0;
            	iSensorPattern = 0;
        	    crank_mode = 0;
				lEncoderLine = lEncoderTotal;
    	        pattern = 42;
	        }
			
        	break;

	    case 42:
    	    /* �������Ԃ��o�܂ő҂� */
        	servoPwmOut( iServoPwm );
			motor_mode_f( BRAKE, BRAKE );
			motor_mode_r( BRAKE, BRAKE );
    	    motor2_f( 30, 30 );
        	motor2_r( 30, 30 );
        	if( lEncoderTotal - lEncoderLine >= 109L ) {
    	        cnt1 = 0;
        	    led_out( 0x0 );
        	    pattern = 11;
        	}	
	        break;
				
		case 50:
			// �E�n�[�t���C�����o��
			servoPwmOut( iServoPwm );
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
			motor2_f( 10, 10 );
			motor2_r( 10, 10 );
			led_out( 0xc0 );
			cnt1 = 0;
			cnt0 = 0;
			lEncoderLine = lEncoderTotal;
			pattern = 51;
			break;
		
		case 51:
			// �E�n�[�t���C���ʉߒ�
			servoPwmOut( iServoPwm );
		
			if( check_crossline() ){
				pattern = 21;
				crank_mode = 1;
				break;
			}
			if( iEncoder >= 11 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
				motor2_f( -30, -30 );
        	    motor2_r( -30, -30 );
	        } else {
			    motor2_f( 80, 80 );
    	        motor2_r( 80, 80 );
        	}
			if(lEncoderTotal - lEncoderLine >= 109L ) {  // ��10cm���������H	
				pattern = 52;	
				lEncoderLine = lEncoderTotal;
				cnt1 = 0;
				break;
			}
			break;
	
		case 52:
			// �E�n�[�t���C���ʉߌ�
			servoPwmOut( iServoPwm );
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
			if( iEncoder >= 16 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
				motor2_f( -0, -0 );
            	motor2_r( -0, -0 );
	        } else {
    	        motor2_f( 50, 50 );
        	    motor2_r( 50, 50 );
        	}
			if( ( center_inp() == 0 ) && ( sensor_inp() == 0x00 ) ) {  // ��������������Ȃ���
				pattern = 53;
				cnt1 = 0;
				lEncoderLine = lEncoderTotal;
				break;
			}
			break;
	
		case 53:
			// �E���[���`�F���W�ʉ�
			iSetAngle = -38;	/*���S����E��12�x�̈ʒu�Ɉړ�	*/
								/* +�ō� -�ŉE�ɋȂ���܂�      */
			servoPwmOut( iServoPwm2 );
			setBeepPatternS(0x8000);
			motor_mode_f( BRAKE , FREE );
			motor_mode_r( BRAKE , FREE );
			motor2_f( 70 , 49 );
			motor2_r( 59 , 48 );
			if((sensor_inp()&0x01) == 0x01 ){
				pattern = 54;	
				lEncoderLine = lEncoderTotal;
				break;	
			}
			if( sensor_inp()&0x04 == 0x04 || (center_inp() == 1)){
				pattern = 55;
				crank_mode = 0;
				lEncoderLine = lEncoderTotal;
				cnt1 = 0;
				break;	
			}
			break;
	
		case 54:
			// �V�����������������Ƃ�
			iSetAngle = 0;		/* �n���h����0�x�ɖ߂�			*/
			servoPwmOut( iServoPwm2 );
			motor_mode_f( FREE , FREE );
			motor_mode_r( FREE , FREE );
			motor2_r( 50, 50 );
			motor2_f( 50, 50 );
			if( sensor_inp()&0x04 == 0x04 || (center_inp() == 1)){
				pattern = 55;
				crank_mode = 0;
				lEncoderLine = lEncoderTotal;
				cnt1 = 0;
				break;	
			}
			break;
	
		case 55:
			// �ʏ한�A�܂�50%�Ńg���[�X
			servoPwmOut( iServoPwm );
			if( lEncoderTotal - lEncoderLine >= 109L ){
				cnt1 = 0;
				led_out(0x00);
				pattern = 11;
				break;
			}
			
			motor2_f( 30, 30 );
			motor2_r( 30, 30 );
			break;
		
		case 60:
			// ���n�[�t���C�����o��
			servoPwmOut( iServoPwm );
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
			motor2_f( 0, 0 );
			motor2_r( 0, 0 );
			led_out( 0xc0 );
			cnt1 = 0;
			cnt0 = 0;
			lEncoderLine = lEncoderTotal;
			pattern = 61;
			break;
		
		case 61:
			// ���n�[�t���C���ʉߒ�
			servoPwmOut( iServoPwm );
			if( check_crossline() ){
				pattern = 21;
				crank_mode = 1;
				break;
			}
			if( iEncoder >= 11 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
				motor_f( -50, -50 );
        	    motor_r( -50, -50 );
        	} else {
				motor2_f( 80, 80 );
    	        motor2_r( 80, 80 );
        	}
			if(lEncoderTotal - lEncoderLine >= 109L ) {  // ��10cm���������H	
				pattern = 62;
				break;
			}
			break;
	
		case 62:
			// ���n�[�t���C���ʉߌ�
			servoPwmOut( iServoPwm );
			motor_mode_f(BRAKE,BRAKE);
			motor_mode_r(BRAKE,BRAKE);
			if( iEncoder >= 11 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
				motor_f( -50, -50 );
	            motor_r( -50, -50 );
    	    } else {
        	    motor2_f( 80, 80 );
            	motor2_r( 80, 80 );
        	}
			if( ( center_inp() == 0 ) && ( sensor_inp() == 0x00 ) ) {  // ��������������Ȃ���
				pattern = 63;			
				break;
			}
			break;
	
		case 63:
			// �����[���`�F���W�ʉ�
			iSetAngle = 38;	/*���S���獶��12�x�̈ʒu�Ɉړ�	*/
							/* +�ō� -�ŉE�ɋȂ���܂�      */
			servoPwmOut( iServoPwm2 );
			motor_mode_f( FREE , BRAKE );
			motor_mode_r( FREE , BRAKE );
			motor2_f( 49 , 60 );
			motor2_r( 48 , 59 );
			if((sensor_inp()&0x08) == 0x08 ){
				pattern = 64;	
				crank_mode = 0;
				break;
			}
			else if((center_inp()==1)||((sensor_inp()&0x02) == 0x02)||((sensor_inp()&0x04)==0x04)){
				pattern = 65;
				crank_mode = 0;
				lEncoderLine = lEncoderTotal;
				cnt1 = 0;
			}
			break;
	
		case 64:
			// �V�����������������Ƃ�
			iSetAngle = 0;		/* �n���h����0�x�ɖ߂�			*/
			servoPwmOut( iServoPwm2 );
			motor_mode_f( BRAKE , BRAKE );
			motor_mode_r( BRAKE , BRAKE );
			motor2_r( 50, 50 );
			motor2_f( 50, 50 );
			if( (center_inp()==1)||((sensor_inp()&0x02) == 0x02)||((sensor_inp()&0x04)==0x04) ){
				pattern = 65;
				crank_mode = 0;
				lEncoderLine = lEncoderTotal;
				cnt1 = 0;
			}
			break;
	
		case 65:
			// �ʏ한�A�܂�50%�Ńg���[�X
			servoPwmOut( iServoPwm );
			if( lEncoderTotal - lEncoderLine >= 109L){
				cnt1 = 0;
				pattern = 11;
				break;
			}
			motor2_f( 30, 30 );
			motor2_r( 30, 30 );
			break;
	
		
	    case 101:
    	    /* ��~���� */
        	servoPwmOut( iServoPwm );
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
	        motor2_f( 0, 0 );
    	    motor2_r( 0, 0 );
			crank_mode = 1;
	        pattern = 102;
    	    cnt1 = 0;
        	break;

    	case 102:
	        servoPwmOut( iServoPwm );
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
	        motor2_f( 0, 0 );
    	    motor2_r( 0, 0 );
    	    if( cnt1 > 300) {
        	    servoPwmOut( 0 );
            	pattern = 103;
    	        cnt1 = 0;
        	    break;
        	}
        	break;

    	case 103: 
			servoPwmOut( iServoPwm );
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
	        motor2_f( 0, 0 );
    	    motor2_r( 0, 0 );
        	if( cnt1 > 500 ) {
	            setBeepPatternS( 0xcc00 );
        	    pattern = 104;
            	cnt1 = 0;
            	break;
       	 	}
        	break;

    	case 104:
        	/* �������Ȃ� */
			servoPwmOut( 0 );
			motor_mode_f( BRAKE, BRAKE );
			motor_mode_r( BRAKE, BRAKE );
	        motor2_f( 0, 0 );
    	    motor2_r( 0, 0 );
			if( cnt1 < 75 ){
				led_out(0x00);
			}
			else if( cnt1 < 150 ){
				led_out(0xff);
			}
			else{
				cnt1 = 0;
			}
        	break;

    	default:
        	break;
		}
	}
	
	/* �ً}��~ */
	motor_mode_f( BRAKE, BRAKE );
	motor_mode_r( BRAKE, BRAKE );
    motor2_f( 0, 0 );
    motor2_r( 0, 0 );
}	


/************************************************************************/
/* R8C/38A �X�y�V�����t�@���N�V�������W�X�^(SFR)�̏�����                */
/************************************************************************/
void init( void )
{
  int i;

  /* �N���b�N��XIN�N���b�N(20MHz)�ɕύX */
  prc0  = 1;                          /* �v���e�N�g����               */
  cm13  = 1;                          /* P4_6,P4_7��XIN-XOUT�[�q�ɂ���*/
  cm05  = 0;                          /* XIN�N���b�N���U              */
  for (i = 0; i < 50; i++ );          /* ���肷��܂ŏ����҂�(��10ms) */
  ocd2  = 0;                          /* �V�X�e���N���b�N��XIN�ɂ���  */
  prc0  = 0;

  /* �|�[�g�̓��o�͐ݒ� */

  /*  PWM(�\��)       ���OM_PMW       �E�OM_PWM       �u�U�[
      �Z���T���[      �Z���T����      �Z���T�E��      �Z���T�E�[  */
  p0   = 0x00;
  prc2 = 1;                           /* PD0�̃v���e�N�g����          */
  pd0  = 0xf0;

  /*  �Z���T���S      �����ް         RxD0            TxD0
      DIPSW3          DIPSW2          DIPSW1          DIPSW0         */
  pur0 |= 0x04;                       /* P1_3?P1_0�̃v���A�b�vON     */
  p1  = 0x00;
  pd1 = 0x10;

  /*  �E�OM_����      �X�e�AM_����    �X�e�AM_PWM     �E��M_PWM
      �E��M_����      ����M_PWM       ����M_����      ���OM_����      */
  p2  = 0x00;
  pd2 = 0xff;

  /* !---�ǉ��E�ύX---! */
  /*  Arduino(ANGLE)  none            none            none
      none            none            none            �G���R�[�_A��   */
  p3  = 0x00;
  pd3 = 0xfa;

  /*  XOUT            XIN             �{�[�h���LED   none
      none            VREF            none            none            */
  p4  = 0x20;                         /* P4_5��LED:�����͓_��         */
  pd4 = 0xb8;

  /*  none            none            none            none
      none            none            none            none            */
  p5  = 0x00;
  pd5 = 0xff;

  /*  none            none            none            none
      none            none            Arduino(ZERO)   Arduino(MODE)   */
  p6  = 0x00;
  pd6 = 0xff;

  /*  DC���[�^��]����1   DC���[�^��]����2       CN6.4����       CN6.5����
      none(��۸ޗ\��) �p�xVR          �Z���T_����۸�  �Z���T_�E��۸�  */
  p7  = 0x00;
  pd7 = 0x00;

  /*  DIPSWorLED      DIPSWorLED      DIPSWorLED      DIPSWorLED
      DIPSWorLED      DIPSWorLED      DIPSWorLED      DIPSWorLED      */
  pur2 |= 0x03;                       /* P8_7?P8_0�̃v���A�b�vON      */
  p8  = 0x00;
  pd8 = 0x00;

  /*  -               -               �߯������       P8����(LEDorSW)
      �E�OM_Free      ���OM_Free      �E��M_Free      ����M_Free      */
  p9  = 0x00;
  pd9 = 0x1f;
  pu23 = 1;   // P9_4,P9_5���v���A�b�v����

  /* �^�C�}RB�̐ݒ� */
  /* ���荞�ݎ��� = 1 / 20[MHz]   * (TRBPRE+1) * (TRBPR+1)
                  = 1 / (20*10^6) * 200        * 100
                  = 0.001[s] = 1[ms]
  */
  trbmr  = 0x00;                      /* ���샂�[�h�A������ݒ�       */
  trbpre = 200 - 1;                   /* �v���X�P�[�����W�X�^         */
  trbpr  = 100 - 1;                   /* �v���C�}�����W�X�^           */
  trbic  = 0x06;                      /* ���荞�ݗD�惌�x���ݒ�       */
  trbcr  = 0x01;                      /* �J�E���g�J�n                 */

  /* A/D�R���o�[�^�̐ݒ� */
  admod   = 0x33;                     /* �J��Ԃ��|�����[�h�ɐݒ�     */
  adinsel = 0xb0;                     /* ���͒[�qP7��8�[�q��I��      */
  adcon1  = 0x30;                     /* A/D����\                  */
  _asm(" NOP ");                      /* ��AD��1�T�C�N���E�G�C�g�����*/
  adcon0  = 0x01;                     /* A/D�ϊ��X�^�[�g              */

  /* �^�C�}RG �^�C�}���[�h(���G�b�W�ŃJ�E���g)�̐ݒ� */
  timsr = 0x40;                       /* TRGCLKA�[�q P3_0�Ɋ��蓖�Ă� */
  trgcr = 0x15;                       /* TRGCLKA�[�q�̗��G�b�W�ŃJ�E���g*/
  trgmr = 0x80;                       /* TRG�̃J�E���g�J�n            */

  /* �^�C�}RC PWM���[�h�ݒ�(���O���[�^�A�E�O���[�^) */
  trcpsr0 = 0x40;                     /* TRCIOA,B�[�q�̐ݒ�           */
  trcpsr1 = 0x33;                     /* TRCIOC,D�[�q�̐ݒ�           */
  trcmr   = 0x0f;                     /* PWM���[�h�I���r�b�g�ݒ�      */
  trccr1  = 0x8e;                     /* �������:f1,�����o�͂̐ݒ�    */
  trccr2  = 0x00;                     /* �o�̓��x���̐ݒ�             */
  trcgra  = TRC_MOTOR_CYCLE - 1;      /* �����ݒ�                     */
  trcgrb  = trcgrb_buff = trcgra;     /* P0_5�[�q��ON��(���O���[�^)   */
  trcgrc  = trcgrc_buff = trcgra;     /* P0_7�[�q��ON��(�\��)         */
  trcgrd  = trcgrd_buff = trcgra;     /* P0_6�[�q��ON��(�E�O���[�^)   */
  trcic   = 0x07;                     /* ���荞�ݗD�惌�x���ݒ�       */
  trcier  = 0x01;                     /* IMIA������                   */
  trcoer  = 0x01;                     /* �o�͒[�q�̑I��               */
  trcmr  |= 0x80;                     /* TRC�J�E���g�J�n              */

  /* �^�C�}RD ���Z�b�g����PWM���[�h�ݒ�(����Ӱ��A�E��Ӱ��A����Ӱ�) */
    trdpsr0 = 0x08;                     /* TRDIOB0,C0,D0�[�q�ݒ�        */
    trdpsr1 = 0x05;                     /* TRDIOA1,B1,C1,D1�[�q�ݒ�     */
    trdmr   = 0xf0;                     /* �o�b�t�@���W�X�^�ݒ�         */
    trdfcr  = 0x01;                     /* ���Z�b�g����PWM���[�h�ɐݒ�  */
    trdcr0  = 0x20;                     /* �\�[�X�J�E���g�̑I��:f1      */
    trdgra0 = trdgrc0 = TRD_MOTOR_CYCLE - 1;    /* �����ݒ�             */
    trdgrb0 = trdgrd0 = 0;              /* P2_2�[�q��ON��(���ヂ�[�^)   */
    trdgra1 = trdgrc1 = 0;              /* P2_4�[�q��ON��(�E�ヂ�[�^)   */
    trdgrb1 = trdgrd1 = 0;              /* P2_5�[�q��ON��(�T�[�{���[�^) */
    trdoer1 = 0xcd;                     /* �o�͒[�q�̑I��               */
    trdstr  = 0x0d;                     /* TRD0�J�E���g�J�n             */
}

/************************************************************************/
/* �^�C�}RB ���荞�ݏ���                                                */
/************************************************************************/
#pragma interrupt /B intTRB(vect=24)
void intTRB( void )
{
  unsigned char b,c,d;
  unsigned int i;
  unsigned int  v;
  _asm(" FSET I ");   /* �^�C�}RB�ȏ�̊��荞�݋���   */

  cnt0++;
  cnt1++;
  cnt_run++;
  check_sen_cnt++;
  check_enc_cnt++;
  check_cross_cnt++;
  cnt_saka++;

  /* �T�[�{���[�^���� */
  servoControl();
  servoControl2();
  //lancerControl();
  
  /* �u�U�[���� */
  beepProcessS();
  if( pattern == 1){
	 servoPwmOut(iServoPwm);
  }
  p4_5=p3_0;
  //c = ~p7_5;
  // �G���R�[�_�̐M����LED�ɕ\��
	
  //led_out((b<<3)|(c<<7)|(d<<2));

  /* 10��1����s���鏈�� */
  iTimer10++;
  switch ( iTimer10 ) {
    case 1:
      /* �G���R�[�_���� */
      i = trg;
      iEncoder       = i - uEncoderBuff;
      lEncoderTotal += iEncoder;
      if ( iEncoder > iEncoderMax ) {
        iEncoderMax = iEncoder;
      }
      uEncoderBuff   = i;
      break;

    case 2:
      /* �X�C�b�`�ǂݍ��ݏ��� */
      p9_4 = 0;                       /* LED�o��OFF                   */
      pd8  = 0x00;
      break;

    case 3:
      /* �X�C�b�`�ǂݍ��݁ALED�o�� */
      types_dipsw = ~p8;              /* ��ײ�ފ��TypeS Ver.3��SW�ǂݍ���*/
      p8  = types_led;                /* ��ײ�ފ��TypeS Ver.3��LED�֏o��*/
      pd8 = 0xff;
      p9_4 = 1;                       /* LED�o��ON                    */
      break;

    case 4:
      break;

    case 5:
      break;

    case 6:
      break;

    case 7:
      break;

    case 8:
      break;

    case 9: 
      break;

    case 10:
      /* iTimer10�ϐ��̏��� */
      iTimer10 = 0;
      break;
  }
}

/************************************************************************/
/* �^�C�}RC ���荞�ݏ���                                                */
/************************************************************************/
#pragma interrupt intTRC(vect=7)
void intTRC( void )
{
  trcsr &= 0xfe;  /* �t���O�N���A */

  /* �^�C�}RC�@�f���[�e�B��̐ݒ� */
  trcgrb = trcgrb_buff;
  trcgrc = trcgrc_buff;
  trcgrd = trcgrd_buff;
}

void timer( unsigned long timer_set ){
	cnt0 = 0;
	while(cnt0 < timer_set );	
}

/************************************************************************/
/* �A�i���O�Z���T���TypeS Ver.2�̃f�W�^���Z���T�l�ǂݍ���              */
/* �����@ �Ȃ�                                                          */
/* �߂�l ���[�A�����A�E���A�E�[�̃f�W�^���Z���T 0:�� 1:��              */
/************************************************************************/
unsigned char sensor_inp( void )
{
    unsigned char sensor;

    sensor = ~p0 & 0x0f;

    return sensor;
}

/************************************************************************/
/* �A�i���O�Z���T���TypeS Ver.2�̒��S�f�W�^���Z���T�ǂݍ���            */
/* �����@ �Ȃ�                                                          */
/* �߂�l ���S�f�W�^���Z���T 0:�� 1:��                                  */
/************************************************************************/
unsigned char center_inp( void )
{
    unsigned char sensor;

    sensor = ~p1_7 & 0x01;

    return sensor;
}

/************************************************************************/
/* �A�i���O�Z���T���TypeS Ver.2�̃X�^�[�g�o�[���o�Z���T�ǂݍ���        */
/* �����@ �Ȃ�                                                          */
/* �߂�l 0:�X�^�[�g�o�[�Ȃ� 1:�X�^�[�g�o�[����                         */
/************************************************************************/
unsigned char startbar_get( void )
{
    unsigned char sensor;

    sensor = ~p1_6 & 0x01;

    return sensor;
}

/************************************************************************/
/* �}�C�R���{�[�h��̃f�B�b�v�X�C�b�`�l�ǂݍ���                         */
/* �����@ �Ȃ�                                                          */
/* �߂�l �X�C�b�`�l 0?15                                              */
/************************************************************************/
unsigned char dipsw_get( void )
{
    unsigned char sw;

    sw = p1 & 0x0f;                     /* P1_3?P1_0�ǂݍ���           */

    return sw;
}

/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.3��̃f�B�b�v�X�C�b�`�l�ǂݍ���          */
/* �����@ �Ȃ�                                                          */
/* �߂�l �X�C�b�`�l 0?255                                             */
/************************************************************************/
unsigned char dipsw_get2( void )
{
    /* ���ۂ̓��͂̓^�C�}RB���荞�ݏ����Ŏ��{ */
    return types_dipsw;
}

/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.3��̃v�b�V���X�C�b�`�l�ǂݍ���          */
/* �����@ �Ȃ�                                                          */
/* �߂�l �X�C�b�`�l 0:OFF 1:ON                                         */
/************************************************************************/
unsigned char pushsw_get( void )
{
    unsigned char sw;

    sw = ~p9_5 & 0x01;

    return sw;
}

/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.3��CN6�̏�ԓǂݍ���                     */
/* �����@ �Ȃ�                                                          */
/* �߂�l 0?15                                                         */
/************************************************************************/
unsigned char cn6_get( void )
{
    unsigned char data;

    data = p7 >> 4;

    return data;
}

/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.3��LED����                               */
/* �����@ 8��LED���� 0:OFF 1:ON                                       */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void led_out( unsigned char led )
{
    /* ���ۂ̏o�͂̓^�C�}RB���荞�ݏ����Ŏ��{ */
    types_led = led;
}

/************************************************************************/
/* ��ւ̑��x����                                                       */
/* �����@ �����[�^:-100?100 , �E���[�^:-100?100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor_r( int accele_l, int accele_r )
{
    int sw_data;

    sw_data  = dipsw_get() + 5;         /* �f�B�b�v�X�C�b�`�ǂݍ���     */
    accele_l = accele_l * sw_data / 20;
    accele_r = accele_r * sw_data / 20;
	
    /* ���ヂ�[�^ */
	accele_l = -accele_l;
    if( accele_l >= 0 ) {
        p2_1 = 0;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
    } else {
        p2_1 = 1;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
    }

    /* �E�ヂ�[�^ */
    if( accele_r >= 0 ) {
        p2_3 = 0;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
    } else {
        p2_3 = 1;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
    }
}

/************************************************************************/
/* ��ւ̑��x����2 �f�B�b�v�X�C�b�`�ɂ͊֌W���Ȃ�motor�֐�              */
/* �����@ �����[�^:-100?100 , �E���[�^:-100?100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor2_r( int accele_l, int accele_r )
{
    /* ���ヂ�[�^ */
	accele_l = -accele_l;
    if( accele_l >= 0 ) {
        p2_1 = 0;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
    } else {
        p2_1 = 1;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
    }

    /* �E�ヂ�[�^ */
    if( accele_r >= 0 ) {
        p2_3 = 0;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
    } else {
        p2_3 = 1;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
    }
}

/************************************************************************/
/* �O�ւ̑��x����                                                       */
/* �����@ �����[�^:-100?100 , �E���[�^:-100?100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor_f( int accele_l, int accele_r )
{
    int sw_data;

    sw_data  = dipsw_get() + 5;         /* �f�B�b�v�X�C�b�`�ǂݍ���     */
    accele_l = accele_l * sw_data / 20;
    accele_r = accele_r * sw_data / 20;
	accele_l = -accele_l;

    /* ���O���[�^ */
    if( accele_l >= 0 ) {
        p2_0 = 0;
    } else {
        p2_0 = 1;
        accele_l = -accele_l;
    }
    if( accele_l <= 5 ) {
        trcgrb = trcgrb_buff = trcgra;
    } else {
        trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_l / 100;
    }

    /* �E�O���[�^ */
    if( accele_r >= 0 ) {
        p2_7 = 0;
    } else {
        p2_7 = 1;
        accele_r = -accele_r;
    }
    if( accele_r <= 5 ) {
        trcgrd = trcgrd_buff = trcgra;
    } else {
        trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_r / 100;
    }
}

/************************************************************************/
/* �O�ւ̑��x����2 �f�B�b�v�X�C�b�`�ɂ͊֌W���Ȃ�motor�֐�              */
/* �����@ �����[�^:-100?100 , �E���[�^:-100?100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor2_f( int accele_l, int accele_r )
{
	/* ���O���[�^ */
    accele_l = -accele_l;
    
	if( accele_l >= 0 ) {
        p2_0 = 0;
    } else {
        p2_0 = 1;
        accele_l = -accele_l;
    }
    if( accele_l <= 5 ) {
        trcgrb = trcgrb_buff = trcgra;
    } else {
        trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_l / 100;
    }

    /* �E�O���[�^ */
    if( accele_r >= 0 ) {
        p2_7 = 0;
    } else {
        p2_7 = 1;
        accele_r = -accele_r;
    }
    if( accele_r <= 5 ) {
        trcgrd = trcgrd_buff = trcgra;
    } else {
        trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_r / 100;
    }
}

/************************************************************************/
/* �ヂ�[�^��~����i�u���[�L�A�t���[�j                                 */
/* �����@ �����[�^:FREE or BRAKE , �E���[�^:FREE or BRAKE               */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor_mode_r( int mode_l, int mode_r )
{
    if( mode_l ) {
        p9_0 = 1;
    } else {
        p9_0 = 0;
    }
    if( mode_r ) {
        p9_1 = 1;
    } else {
        p9_1 = 0;
    }
}

/************************************************************************/
/* �O���[�^��~����i�u���[�L�A�t���[�j                                 */
/* �����@ �����[�^:FREE or BRAKE , �E���[�^:FREE or BRAKE               */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor_mode_f( int mode_l, int mode_r )
{
    if( mode_l ) {
        p9_2 = 1;
    } else {
        p9_2 = 0;
    }
    if( mode_r ) {
        p9_3 = 1;
    } else {
        p9_3 = 0;
    }
}

/************************************************************************/
/* �T�[�{���[�^����                                                     */
/* �����@ �T�[�{���[�^PWM�F-100?100                                    */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void servoPwmOut( int pwm )
{

    /* �{�����[���l�ɂ�荶���~�b�g���� */
//    if( ad2 >= 986 && pattern >= 11 ){
//        pwm = 0;
//    }
    /* �{�����[���l�ɂ��E���~�b�g���� */
//    if( ad2 <= 813 && pattern >= 11 ){
//        pwm = 0;
//    }

    if( pwm >= 0 ) {
        p2_6 = 0;
        trdgrd1 = (long)( TRD_MOTOR_CYCLE - 2 ) * pwm / 100;
    } else {
        p2_6 = 1;
        trdgrd1 = (long)( TRD_MOTOR_CYCLE- 2 ) * ( -pwm ) / 100;
    }
}

/************************************************************************/
/* �N���X���C�����o����                                                 */
/* �����@ �Ȃ�                                                          */
/* �߂�l 0:�N���X���C���Ȃ� 1:����                                     */
/************************************************************************/
int check_crossline( void )
{
    unsigned char b;
    int ret = 0;

    b = sensor_inp();
    if( b==0x0f || b==0x0e || b==0x0d || b==0x0b || b==0x07 ) {
        lEncoderLine = lEncoderTotal;
		ret = 1;
    }
    return ret;
}

/************************************************************************/
/* �E�n�[�t���C�����o���� 												*/
/* ���� �Ȃ� 															*/
/* �߂�l 0:�n�[�t���C���Ȃ� 1:���� 									*/
/************************************************************************/
int check_rightline( void )
{
	int ret = 0;
	if( (sensor_inp() == 0x03 || sensor_inp() == 0x01 || sensor_inp == 0x02) && (center_inp() == 1) ) {
		lEncoderLine = lEncoderTotal;
		ret = 1;
	}
	return ret;
}

/************************************************************************/
/* ���n�[�t���C�����o����                                               */
/* �����@ �Ȃ�                                                          */
/* �߂�l 0:���n�[�t���C���Ȃ� 1:����                                   */
/************************************************************************/
int check_leftline( void )
{
    int ret = 0;
	if( (sensor_inp() == 0x0c || sensor_inp() == 0x08 || sensor_inp() == 0x04) && (center_inp() == 1) ){
		lEncoderLine = lEncoderTotal;
		ret = 1;
	}
	return ret;
}

/************************************************************************/
/* �T�[�{�p�x�擾                                                       */
/* �����@ �Ȃ�                                                          */
/* �߂�l ����ւ���̒l                                                */
/************************************************************************/
int getServoAngle( void )
{
    return( (-ad2) - iAngle0 );
}

/************************************************************************/
/* �A�i���O�Z���T�l�擾                                                 */
/* �����@ �Ȃ�                                                          */
/* �߂�l �Z���T�l                                                      */
/************************************************************************/
int getAnalogSensor( void )
{
    int ret;

    ret = ad1 - ad0;                    /* �A�i���O�Z���T���擾       */

    if( !crank_mode ) {
        /* �N�����N���[�h�łȂ���Ε␳���� */
        switch( iSensorPattern ) {
        case 0:
            if( sensor_inp() == 0x04 ) {
                ret = -650;
                break;
            }
            if( sensor_inp() == 0x02 ) {
                ret = 650;
                break;
            }
            if( sensor_inp() == 0x0c ) {
                ret = -700;
                iSensorPattern = 1;
                break;
            }
            if( sensor_inp() == 0x03 ) {
                ret = 700;
                iSensorPattern = 2;
                break;
            }
            break;

        case 1:
            /* �Z���T�E��� */
            ret = -700;
            if( sensor_inp() == 0x04 ) {
                iSensorPattern = 0;
            }
            break;

        case 2:
            /* �Z���T����� */
            ret = 700;
            if( sensor_inp() == 0x02 ) {
                iSensorPattern = 0;
            }
            break;
        }
	}

    return ret;
}

/************************************************************************/
/* �T�[�{���[�^����                                                     */
/* �����@ �Ȃ�                                                          */
/* �߂�l �O���[�o���ϐ� iServoPwm �ɑ��                               */
/************************************************************************/
void servoControl( void )
{
    int i, iRet, iP, iD;

    i = getAnalogSensor();              /* �Z���T�l�擾                 */

    /* �T�[�{���[�^�pPWM�l�v�Z */
    iP = KP * i;                        /* ���                         */
    iD = KD * (iSensorBefore - i );     /* ����(�ڈ���P��5?10�{)       */
	
    iRet = iP - iD;
    iRet /= 16;

    /* PWM�̏���̐ݒ� */
    if( iRet >  SERVO_PWM_MAX ) iRet =  SERVO_PWM_MAX;/* �}�C�R���J�[�����肵����     */
    if( iRet < -SERVO_PWM_MAX ) iRet = -SERVO_PWM_MAX;/* �����70���炢�ɂ��Ă������� */
    iServoPwm = iRet;

    iSensorBefore = i;                  /* ����͂��̒l��1ms�O�̒l�ƂȂ�*/
}

/************************************************************************/
/* �T�[�{���[�^2����                                                    */
/* �����@ �Ȃ�                                                          */
/* �߂�l �O���[�o���ϐ� iServoPwm2 �ɑ��                              */
/************************************************************************/
void servoControl2( void )
{
    int i, j, iRet, iP, iD;

    i = iSetAngle;
    j = getServoAngle();

    /* �T�[�{���[�^�pPWM�l�v�Z */
    iP = 20 * ( j - i );                /* ���                         */
    iD = 50 * ( iAngleBefore2 - j );    /* ����(�ڈ���P��5?10�{)       */
    iRet = iP - iD;
    iRet /= 2;

    /* PWM�̏���̐ݒ� */
    if( iRet >  SERVO_PWM_MAX ) iRet =  SERVO_PWM_MAX;/* �}�C�R���J�[�����肵����     */
    if( iRet < -SERVO_PWM_MAX ) iRet = -SERVO_PWM_MAX;/* �����70���炢�ɂ��Ă������� */
    iServoPwm2 = iRet;

    iAngleBefore2 = j;                  /* ����͂��̒l��1ms�O�̒l�ƂȂ�*/
}

/************************************************************************/
/* �O�ւ�PWM����A���ւ�PWM������o���@�n���h���p�x�͌��݂̒l���g�p     */
/* �����@ �O��PWM                                                       */
/* �߂�l ����PWM                                                       */
/************************************************************************/
int diff( int pwm )
{
    int i, ret;

    i  = getServoAngle() / 5;    /* 1�x������̑����Ŋ���        */
    if( i <  0 ) i = -i;
    if( i > 45 ) i = 45;
    ret = revolution_difference[i] * pwm / 100;

    return ret;
}

/************************************************************************/
/* �⓹�`�F�b�N(���~�b�g�X�C�b�`)										*/
/* ���� �Ȃ�															*/
/* �߂�l �Ȃ� 															*/
/* ���� �⓹�Ɣ��f�����saka_flag = 1 �⓹�łȂ��Ȃ� 0 					*/
/************************************************************************/
void sakaSyori( void )
{
	static int saka_pattern = 0;
	int saka;
	
	saka = ad4 - saka0_ad;
	
	switch( saka_pattern ){
		case 0:
			// ����A�����̃`�F�b�N
			if( saka <= -10 ) { // ����A/D�l���������Ȃ�
				cnt_saka = 0;
				saka_pattern = 1; // ���⏈��
			}
			break;
		
		case 1:
			// ���� �������Ԃ������āA�ēx�`�F�b�N
			if( cnt_saka >= 10 ) {
				if( saka <= -10 ) { // ��������
					setBeepPatternS( 0xcc00 );
					saka_flag = 1; // ��t���O��ON!
					cnt_saka = 0;
					saka_pattern = 2;
				} else {
					// �����Ȃ��Ȃ�듮��Ɣ��f���Ė߂�
					saka_pattern = 0;
				}
			}
			break;

		case 2:
			// ���� ���_�̒��Ԃ��炢�������I���̃`�F�b�N
			if( cnt_saka >= 100 ) {
				cnt_saka = 0;
				saka_pattern = 3;
			}
			break;
		
		case 3:
			// ���� ���̒��_���`�F�b�N
			if( saka >= 10 ) {
				cnt_saka = 0;
				saka_pattern = 4;
			}
			break;
		
		case 4:
			// ���� �������Ԃ������āA�ēx�`�F�b�N
			if( cnt_saka >= 10 ) {
				if( saka >= 10 ) { // ��������
					setBeepPatternS( 0xff00 );
					saka_flag = 0; // ��t���O��OFF!
					cnt_saka = 0;
					saka_pattern = 5;
				} else {
				// �����Ȃ��Ȃ�듮��Ɣ��f���Ė߂�
				saka_pattern = 3;
			}
		}
		break;

		case 5:
			// ����I��� �����i�܂��Ēʏ푖�s��
			if( cnt_saka >= 200 ) {
				cnt_saka = 0;
				saka_pattern = 0;
			}	
			break;
		}
}

/************************************************************************/
/* ���C���g���[�X�֐��{��                                               */
/* ����   �f���[�e�B��(0?100)                                          */
/* �߂�l �Ȃ�                                                          */
/* ���l �@�̂ɍ��킹�ăp�����[�^�����������Ă�������                    */
/************************************************************************/

void traceMain( void )
{
  const int _3MS = 86;
  const int _2MS = 43;

  int i;
  i = getServoAngle();

  if ( i > 50 ) {
    motor_mode_f( BRAKE, BRAKE );
	
    motor_mode_r( BRAKE, BRAKE );

    if ( iEncoder >= _3MS ) { // 3.0m/s�ȏ�Ȃ�
      motor2_f( -100, -100 );
      motor2_r(   0,   0 );
    } else {
      motor2_f( 40, diff(40) );
      motor2_r( -40, diff(40) );
    }
  } else if ( i < -50 ) {
    motor_mode_f( BRAKE, BRAKE );
    motor_mode_r( BRAKE, BRAKE );

    if ( iEncoder >= _3MS ) { // 3.0m/s�ȏ�Ȃ�
      motor2_f( -100, -100 );
      motor2_r(   0,   0 );
    } else {
      motor2_f( diff(40), 40 );
      motor2_r( -diff(40), 40 );
	} 
  
  }else if ( i > 15 ) {
    if (iEncoder >= _3MS ) { // 3.0m/s�ȏ�Ȃ�
      motor_mode_f( BRAKE, BRAKE );
      motor_mode_r( BRAKE, BRAKE );
      motor2_f( -70, -70 );
      motor2_r( 50, -50 );
    } else if ( iEncoder >= _2MS ) { // 2.0m/s�ȏ�Ȃ�
      motor_mode_f( BRAKE, FREE );
      motor_mode_r( BRAKE, FREE );
      motor2_f( 45, 0 );  // ���ւ�0%�ɂ��āA�J�[�u���Ȃ���₷������
      motor2_r( -45, 0 );
    } else {
      motor_mode_f( BRAKE, FREE );
      motor_mode_r( BRAKE, FREE );
      motor2_f( 50, diff(50) );
      motor2_r( -50, diff(50) );
    }
  } else if ( i < -15 ) {
    if ( iEncoder >= _3MS ) {
      motor_mode_f( BRAKE, BRAKE );
      motor_mode_r( BRAKE, BRAKE );
      motor2_f( -70, -70 );
      motor2_r( 50, -50 );
    } else if ( iEncoder >= _2MS ) { // 2.0m/s�ȏ�Ȃ�
      motor_mode_f( FREE, BRAKE );
      motor_mode_r( FREE, BRAKE );
      motor2_f( 0, 45 );         // ���ւ�0%�ɂ��āA�J�[�u���Ȃ���₷������
      motor2_r( 0, 45 );
    } else {
      motor_mode_f( FREE, BRAKE );
      motor_mode_r( FREE, BRAKE );
      motor2_f( diff(50), 50 );
      motor2_r( -diff(50), 50 );
    }
  } else { 
	
    
	if ( iEncoder >= (dipsw_get() * 2 + 35) ) { // 50(2.0m/s)?138(5.0m/s)  // ����l:15*2+14 = 44
      // dip_sw�̒l��
      // 0��0*2+25=25(2.3m/s) 8�� 8*2+25=41(3.7m/s)
      // 1��1*2+25=27(2.5m/s) 9�� 9*2+25=43(3.9m/s)
      // 2��2*2+25=29(2.6m/s) 10��10*2+25=45(4.1m/s)
      // 3��3*2+25=31(2.8m/s) 11��11*2+25=47(4.3m/s)
      // 4��4*2+25=33(3.0m/s) 12��12*2+25=49(4.5m/s)
      // 5��5*2+25=35(3.2m/s) 13��13*2+25=51(4.6m/s)
      // 6��6*2+25=37(3.4m/s) 14��14*2+25=53(4.8m/s)
      // 7��7*2+25=39(3.5m/s) 15��15*2+25=55(5.0m/s)
      motor_mode_f( BRAKE, BRAKE );
      motor_mode_r( BRAKE, BRAKE );
      motor2_f( 0, 0 );
      motor2_r( 0, 0 );

    } else {
      motor_mode_f( FREE, FREE );
      motor_mode_r( FREE, FREE );
      motor2_f( 100, 100 );
      motor2_r( -100, 100 );
    }
  }
}



/*
void traceMain( void ){
		int i;
		i = getServoAngle();
        if( i > 170 ) {
			motor_mode_f( BRAKE, BRAKE );
      		motor_mode_r( BRAKE, BRAKE );
            motor_f( diff(60), 60 );
            motor_r( -diff(60), 60 );
        } else if( i > 25 ) {
      		motor_mode_f( FREE, BRAKE );
      		motor_mode_r( FREE, BRAKE );
			motor_f( diff(80), 80 );
            motor_r( -diff(80), 80 );
        } else if( i < -170 ) {
			motor_mode_f( BRAKE, BRAKE );
      		motor_mode_r( BRAKE, BRAKE );
            motor_f( 60, diff(60) );
            motor_r( -60, diff(60) );
        } else if( i < -25 ) {
			motor_mode_f( BRAKE, FREE );
      		motor_mode_r( BRAKE, FREE );
			motor_f( 80, diff(80) );
            motor_r( -80, diff(80) );
        } else {	
			// dip_sw�̒l��
      // 0��0*2+25=25(2.3m/s) 8�� 8*2+25=41(3.7m/s)
      // 1��1*2+25=27(2.5m/s) 9�� 9*2+25=43(3.9m/s)
      // 2��2*2+25=29(2.6m/s) 10��10*2+25=45(4.1m/s)
      // 3��3*2+25=31(2.8m/s) 11��11*2+25=47(4.3m/s)
      // 4��4*2+25=33(3.0m/s) 12��12*2+25=49(4.5m/s)
      // 5��5*2+25=35(3.2m/s) 13��13*2+25=51(4.6m/s)
      // 6��6*2+25=37(3.4m/s) 14��14*2+25=53(4.8m/s)
      // 7��7*2+25=39(3.5m/s) 15��15*2+25=55(5.0m/s)
			if( iEncoder <= dipsw_get()*2 +25 ){
      			motor_mode_f( FREE, FREE );
      			motor_mode_r( FREE, FREE );
				if( cnt_run <= 2000 ){
            		motor2_f( 100, 100 );
            		motor2_r( -100, 100 );
				}
				else{
				motor_mode_f( FREE, FREE );
      			motor_mode_r( FREE, FREE );	
					motor_f( 100, 100 );
            		motor_r( -50, 50 );	
				}		
			}else{
				motor_mode_f(BRAKE,BRAKE );
      			motor_mode_r(BRAKE,BRAKE );
				motor2_f( 0, 0 );
            	motor2_r( 0, 0 );
			}
        }	
}
*/


/************************************************************************/
/* �ߋ��̉����̌��o�񐔂ɉ����ĕW�I����������                           */
/* ����   �Ȃ�                                                          */
/* �߂�l �ϐ�hyouteki_flag��(���s�W�I:0 �����W�I:1)������              */
/************************************************************************/
void hyouteki_check( void ) {
  switch ( hitcount ) {
    case 0:
      hyouteki_flag = 0;
      break;

    case 1:
      hyouteki_flag = 1;
      break;

    default:
      break;

  }
}

/************************************************************************/
/* ���l������͈͂���ʂ͈̔͂ɕϊ�(Arduino��map�֐��Ɠ���)             */
/*                                                                      */
/* ����   x: �ϊ����������l                                             */
/*        in_min: ���݂͈̔͂̉���                                      */
/*        int_max: ���݂͈̔͂̏��                                     */
/*        out_min: �ϊ���͈̔͂̉���                                   */
/*        out_max: �ϊ���͈̔͂̏��                                   */
/*                                                                      */
/* �߂�l �ϊ���̐��l (long)                                           */
/************************************************************************/
long map( long x, long in_min, long in_max, long out_min, long out_max ) {

  return ( x - in_min ) * ( out_max - out_min ) / ( in_max - in_min ) + out_min;

}

/************************************************************************/
/* ���p�x�擾	                                                        */
/* �����@ �Ȃ�                                                          */
/* �߂�l ����ւ���̒l                                                */
/************************************************************************/
int getLancerAngle( void )
{
    return( ad4 - iLancer0 );  // TypeS���AN16(p7_4)��R13���O��
}

/************************************************************************/
/* �T�[�{���[�^����(��)	���Έʒu�ł̐���                               */
/* �����@ �T�[�{���[�^PWM�F-100?100                                    */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void lancerPwmOut( int pwm )
{
	int accele;
   	if ( pwm >= 0 ) {
    	p6_6 = 0;
    } else {
    	p6_6 = 1;
  	}
	accele = map(pwm, -100, 100, 0, 100);  // -100����100�� 0�`100�ɕϊ�
	if( pwm == 0 ){
		p6_6 = 0;
		accele = 40;
		trcgrc_buff = (long)( TRD_MOTOR_CYCLE - 2 ) * accele / 100;
	}
	else if ( accele <= 5 ) {
    	trcgrc = trcgrc_buff = trcgra;
  	} else {
		trcgrc_buff = (long)( TRD_MOTOR_CYCLE - 2 ) * accele / 100;
  		
	}
}

/************************************************************************/
/* �T�[�{���[�^����(��) ��Έʒu�ł̐���                               */
/* �����@ �T�[�{���[�^PWM�F-100?100                                    */
/*        50�Œ�~�A100�Ő��]100%�A0�ŋt�]100%v                        */
/* �߂�l �Ȃ�                                                         */
/************************************************************************/
void lancerPwmOut_ABS( int pwm )
{
   	if ( pwm >= 0 ) {
    	DCM_DIR_PORT = 0;
    } else {
    	DCM_DIR_PORT = 1;
		pwm = -pwm;
  	}
	
	if ( pwm <= 5 ) {
    	trcgrc = trcgrc_buff = trcgra;
  	} else {
		trcgrc_buff = (long)( TRD_MOTOR_CYCLE - 2 ) * pwm / 100;
  		
	}
}

/************************************************************************/
/* �T�[�{���[�^���� ��       											*/
/* ���� �Ȃ� 															*/
/* �߂�l �O���[�o���ϐ� iLancerPwm �ɑ�� 								*/
/************************************************************************/
void lancerControl( void )
{
	int i, j, iRet, iP, iD;
	
	/* !!�ǉ��E�ύX!!! */
	// i = iSetAngle; 						/* �ݒ肵�����p�x 	*/
	// j = getServoAngle(); 				/* ���݂̊p�x 		*/
	
	
	i = iSetLancer; 						/* �ݒ肵�����p�x 	*/
	j = ad4;				 			/* ���݂̊p�x 		*/
	 
	/*     P                            D                      */
  	iRet = (L_KP * i) - ( L_KD * (i - iLancerBefore));
    iRet /= 2;
	
	if( iRet >  LANCER_PWM_MAX ) iRet =  LANCER_PWM_MAX;	/* �}�C�R���J�[�����肵���� 	*/
	if( iRet < -LANCER_PWM_MAX ) iRet = -LANCER_PWM_MAX; 	/* �����90���炢�ɂ��Ă������� */
	
	iLancerPwm = iRet;
	iLancerBefore = j;
}

void servoSet( int num ) {
  if ( num != 0 ) {
    if ( num == 1 ) {
    }
    if ( num == 2 ) {
    }
    if ( num == 3 ) {
    }
    if ( num == 4 ) {
    }
    if ( num == 5 ) {
    }
    if ( num == 6 ) {
    }
  }
  if ( (num == 0) || (num >= 7) ) {
	
  }
}

void sp( int l , int r ){
	servoPwmOut( iServoPwm );
	motor_mode_f( BRAKE, BRAKE );
	motor_mode_r( BRAKE, BRAKE );
	
	motor2_f( l, r );
	if(cnt1 < 150){
		motor2_r( -50, -50 );	
	}
	else{
		motor2_r( 30, diff(30) );
	}
}