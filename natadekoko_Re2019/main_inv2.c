/* advance�}�C�R���J�[�p�v���O���� */
// main_inv2.c(���̃t�@�C��)�������p

/* �X�V����: */
// 201220 TypeS���+microSD��ǉ�
// 210326 TypeS���+microSD+LCD(SC1602�n)��ǉ�
// 210704 �X�^�[�g�o�[���m��TypeS��LED(0x04)���_��/����
// 210722 ���TypeD�ɕύX
// 210722 TypeS��LED(0x04)���ƕ�����ɂ��������̂ŃX�^�[�g�o�[���m��TypeD��LED(0x80)���_��/����
// 210723 CSV�̏㕔�ɑ��s���̐ݒ�p�����[�^��\���ł���悤�ɂ���
// 210731 �R�[�i�[���s���̋쓮���[�^�[����֐�corner_run(���PWM)��ǉ�
// 210801 �����f�[�^�t���b�V������O��p�����[�^���N�����ɓǂݍ��ނ悤�ɂ���
// 210828 START�{�^���őO��p�����[�^��ݒ�->���s�ł���悤�ɕύX
 

/*======================================*/
/* �C���N���[�h */
/*======================================*/
#include <stdio.h>
#include <stdlib.h> 		/* abs�֐����g�p */
#include "sfr_r838a.h" 	/* R8C/38A SFR �̒�`�t�@�C�� */
#include "types3_beep.h"	/* �u�U�[�ǉ� */
#include "microsd_lib.h"
#include "switch_lib.h"
#include "lcd_lib.h"
#include "data_flash_lib.h" /* �O��p�����[�^�L���p�Ƀf�[�^�t���b�V�����g�p */

/*======================================*/
/* �V���{����` */
/*======================================*/
/* �萔�ݒ� */
#define TRC_MOTOR_CYCLE 20000 /* ���O,�E�O���[�^ PWM �̎��� */
/* 50[ns] * 20000 = 1.00[ms] */
#define TRD_MOTOR_CYCLE 20000 /* ����,�E��,����Ӱ� PWM �̎��� */
/* 50[ns] * 20000 = 1.00[ms] */
#define SERVO_STEP 3 		/* 1��������̐��l 3 */
#define SERVO_CENTER 254 	/* ���S */
#define FREE  1			/* ���[�^���[�h �t���[ */
#define BRAKE 0 			/* ���[�^���[�h �u���[�L */
#define LEFT  1			/* �N�����N�� */
#define RIGHT 2 			/* �N�����N�E */

/* �f�[�^�t���b�V���֘A  */
#define     DF_ADDR_START   0x3000/* �������݊J�n�A�h���X         */

#define     DF_PARA_SIZE    256  /* DataFlash�p�����[�^��        */
#define     DF_CHECK        0    /* �f�[�^�t���b�V���`�F�b�N     */
#define     DF_DATA         1		/* �f�[�^                       */

#define 	 DF_TESTSPEED    0x01
#define     DF_CURVE_SPEED  0x02
#define     DF_LANE_L       0x03
#define     DF_LANE_R       0x04
#define     DF_CRANK_SPEED  0x05
#define     DF_ENC_END      0x06

/* ���̑� */
#define     BEEP            1           /* ����ON or OFF�̐ݒ�          */
#define     RUNNING_TIME        31000L  /* ���s����(ms)                 */
//#define    ENC_END       1200L*(()+1)
volatile unsigned long ENC_END = 1;/* ���s����[m] 0�ȊO���Ƃ肠��������Ă���:param_Settings�֐��Ŏw��*/
/*======================================*/
/* �v���g�^�C�v�錾 */
/*======================================*/
void init( void );
unsigned char sensor_inp( void );
unsigned char center_inp( void );
unsigned char startbar_get( void );
unsigned char dipsw_get( void );
unsigned char dipsw_get2( void );
unsigned char dipsw_get3( void );
unsigned char dipsw_get4( void );
unsigned char dipsw_getf( void );
unsigned char pushsw_get( void );
unsigned char cn6_get( void );
void led_out( unsigned char led );
void motor_r( int accele_l2, int accele_r2 );
void motor2_r( int accele_l, int accele_r );
void motor_f( int accele_l2, int accele_r2 );
void motor2_f( int accele_l, int accele_r );
void motor_mode_r( int mode_l, int mode_r );
void motor_mode_f( int mode_l, int mode_r );
void servoPwmOut( int pwm );
int check_crossline( void );
int check_zlineR( void );
int check_zlineL( void );
int check_Noline( void );
int getServoAngle( void );
int getAnalogSensor( void );
void servoControl( void );
int diff( int pwm );
void handle( int iSetAngle );
void timer(unsigned long timer_set);
void sakaSyori( void );
int paramSettings( void );
int getSws( void );
long map(long x, long in_min, long in_max, long out_min, long out_max);
void corner_run(int max_power); //�R�[�i�[���s���̋쓮���[�^�[����

/*======================================*/
/* �O���[�o���ϐ��̐錾 */
/*======================================*/
/* �ʏ�ݒ�: */
volatile int pattern = 0; 	/* �}�C�R���J�[����p�^�[�� */
volatile int crank_mode = 0; /* 1:�N�����N���[�h 0:�ʏ� */
volatile unsigned long cnt0 = 0; /* �^�C�}�p */
volatile unsigned long cnt1 = 0; /* �^�C�}�p */
volatile int h;

/* �G���R�[�_�֘A */
volatile int iTimer10; 				 /* 10ms �J�E���g�p */
volatile unsigned long lEncoderTotal; /* �ώZ�l�ۑ��p */
volatile unsigned long lEncoderCorner; /* pattern11~14 100mm���o�p */
volatile unsigned long lEncoderCrank; /*�N�����N���̋��� lEncoderTotal - lEncoderCrank */
volatile int iEncoder; 				 /* 10ms ���̍ŐV�l */
volatile unsigned int uEncoderBuff; 	 /* �v�Z�p ���荞�ݓ��Ŏg�p */
volatile int speed_target; /* �ڕW���x[iEncoder�l] */
volatile int corner_speed; /* �J�[�u�ł̃X�s�[�h[iEncoder�l] */
  
/* �T�[�{�֘A */
volatile int iSensorBefore; 		/* �O��̃Z���T�l�ۑ� */
volatile int iServoPwm; 			/* �T�[�{�o�v�l�l */
volatile int iAngle0; 			/* ���S���� A/D �l�ۑ� */

volatile signed int kakudo;  /* �p�x[��] 		*/
volatile signed int angle;	/* �p�x[AD�l] 	*/

/* �Z���T�֘A */
volatile int iSensorPattern; 		/* �Z���T��ԕێ��p */

/* TRC ���W�X�^�̃o�b�t�@ */
volatile unsigned int trcgrb_buff; 	/* TRCGRB �̃o�b�t�@ */
volatile unsigned int trcgrd_buff; 	/* TRCGRD �̃o�b�t�@ */
volatile unsigned int trcgrc_buff;	/* TRCGRC �̃o�b�t�@ */

/* ���[�^�h���C�u��� TypeS Ver.3 ��� LED�A�f�B�b�v�X�C�b�`���� */
volatile unsigned char types_led; 	/* LED �l�ݒ� */
volatile unsigned char types_dipsw; 	/* �f�B�b�v�X�C�b�`�l�ۑ� */

/* ���[���`�F���W */
volatile unsigned char Lane_Change; /* 1 �� 0 �E */

/*���[�^�֌W */
volatile unsigned char M_FreeMoter;

/* �T�[�{�֌W 2 */
volatile int iSetAngle; 			/* �ݒ肵�����p�x(AD �l) */
volatile int iAngleBefore2; 		/* �O��̊p�x�ۑ� */
volatile int iServoPwm2; 		/* �T�[�{�o�v�l�l */
volatile int gain; 				//��������

/* ���x���� */
volatile unsigned char lanechangespeed;
volatile unsigned char cornerspeed;
volatile unsigned char crankspeed;
volatile unsigned char streetspeed;

/* �R�[�X�A�E�g */
unsigned long causeout;
volatile int acceleFree;
volatile int endFlag = 0; /* ���s�I�����H 1�ő��s�I�� */

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
volatile const int r_in[60]={100,98,97,95,93,92,90,88,87,85,84,82,81,79,78,76,75,73,72,70,69,67,66,65,63,62,60,59,58,56,55,53,52,51,49,48,46,45,43,42,40,39,37,36,34,33,31,30,28,26,25,23,21,20,18,16,14,12,11,9};
volatile const int f_in[60]={100,98,97,96,94,93,92,90,89,88,87,86,85,84,83,82,81,80,79,79,78,77,77,76,75,75,74,74,73,72,72,72,71,71,70,70,70,69,69,69,69,69,68,68,68,68,68,68,68,68,68,68,68,69,69,69,69,70,70,70};
volatile const int f_out[60]={100,99,98,98,97,96,95,95,94,94,93,93,92,92,92,91,91,91,90,90,90,90,90,89,89,89,89,89,89,89,89,89,90,90,90,90,90,91,91,91,92,92,92,93,93,94,94,95,96,96,97,98,99,99,100,100,100,100,100,100};

signed int accel_in;		/* �R�[�i�[�� �O�������[�^����l(accel) */
signed int accel_out;		/* �R�[�i�[�� �O�O�����[�^����l(accel) */
signed int accel_in_b;	/* �R�[�i�[�� ��������[�^����l(accel) */
signed int accel_out_b;	/* �R�[�i�[�� ��������[�^����l(accel) */

/* microSD�֘A�ϐ� */
volatile const char *C_DATE = __DATE__; /* �R���p�C���������t */
volatile const char *C_TIME = __TIME__; /* �R���p�C���������� */
volatile int msdFlag; /* 1:�f�[�^�L�^ 0:�L�^���Ȃ� */
volatile int msdError; /* �G���[�ԍ��L�^ */

/* �f�[�^�t���b�V���֘A */
volatile signed char     data_buff[ DF_PARA_SIZE ];  /* �ꎞ�ۑ��G���A           */

/* �⓹���o�p */
volatile unsigned long cnt_saka; /* �⓹���o�p�^�C�} */
volatile int saka_flag; /* 1:�⓹�� 0:�⓹�ł͂Ȃ� */
volatile int saka0_ad; /* ���n�̍⓹�{�����[��A/D�l */
volatile const int dipsw1_pattern[] = { 9, 10, 11, 12, 13, 14, 15, 16};
volatile const int dipsw2_pattern[] = { 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25};
volatile const int dipsw3_pattern[] = { 10, 11, 12, 13};
volatile const int dipsw4_pattern[] = { 5, 6, 7, 8};
volatile const int dipswf_pattern[] = { 0, 1};

/* getSw�֐��̒l���ω����Ă��邩�m�F�p */
volatile int swnum = 22;
volatile int beforeswnum;
/************************************************************************/
/* ���C���v���O���� */
/************************************************************************/
void main( void ) {
  int i=0, ret, x=0;
  char fileName[ 8+1+3+1 ]; /* ���O�{'.'�{�g���q�{'\0' */
  unsigned char b;
  
  /* �}�C�R���@�\�̏����� */
  init(); /* ������ */
  readDataFlash(DF_ADDR_START, data_buff, DF_PARA_SIZE); /* 0x3000�Ԓn�ȍ~�f�[�^�t���b�V���ǂݍ���  */
  
  //data_buff�z���DF_CHECK�Ԗځi0�Ԗځj�̃f�[�^��0x38���`�F�b�N
  //0x38�Ȃ�edata_buff�ɂ͑O��ۑ������f�[�^���ǂݍ��܂��
  if( data_buff[DF_CHECK] != 0x38 ) {
	  data_buff[DF_CHECK] = 0x38;	// �EDF_CHECK�Ԗځi0�Ԗځj��0x38��
	  data_buff[DF_DATA]  = 0;	// �EDF_DATA�Ԗ�(1�Ԗ�)��0��ݒ�
  }										
  
  setMicroSDLedPort( &p6, &pd6, 0 ); /* microSD���j�^LED�ݒ� */
  _asm(" fset I "); /* �S�̂̊��荞�݋��� */
  initBeepS(); /* �u�U�[�֘A���� */
  initLcd(); /* LCD������ */
  initSwitch(); /* �X�C�b�`������ */
  
  /* �}�C�R���J�[�̏�ԏ����� */
  motor_mode_f( BRAKE, BRAKE ); //BRAKE
  motor_mode_r( BRAKE, BRAKE );
  motor_f( 0, 0 );
  motor_r( 0, 0 );
  servoPwmOut( 0 );
  lcdPosition(0,0 );
  lcdPrintf(C_DATE);
  lcdPosition( 0, 1 );
  lcdPrintf( C_TIME );
  /* microSD������ */
  ret = initMicroSD();
  if( ret != 0x00 ){
	  msdError = 1;
	  lcdPosition(0,0);
	  lcdPrintf("microSD Error!  ");
	  lcdPosition(0,1);
	  lcdPrintf("no=%d CONECT ERR",ret);
  }
  
  /* FAT32�Ń}�E���g */
  if( msdError == 0){
	  ret = mountMicroSD_FAT32();
	  if(ret != 0x00 ){
		  msdError = 2;
		  lcdPosition(0,0);
		  lcdPrintf("microSD Error!  ");
		  lcdPosition(0,1);
		  lcdPrintf("NOT FAT32       ");
	  }
  }
  if(msdError == 0 ){
	/* microSD�ɃG���[���Ȃ����LCD�ɕ\�� */
	lcdPosition(0,0);
	lcdPrintf("microSD OK     ");
	lcdPosition(0,1);
	lcdPrintf("               ");
	timer(1000);
	/* �⓹�̊�ʒu����					*/
	saka0_ad = ad5;
	lcdPosition(0,0);
	lcdPrintf("saka0_ad init. ");
	lcdPosition(0,1);
	lcdPrintf("ad= %d      ",saka0_ad);
	timer(1000);
  }
  
  if( msdError != 0 ){
	/* microSD�����ɃG���[�������3�b��LED�̓_�ŕ��@��ς��� */
	cnt1 = 0;
	while( cnt1 < 3000 ){
		if( cnt1 % 200 < 100 ){
			led_out(0x03);	
		}
		else{
			led_out(0x00);	
		}
	}	  
  }
  
  /* �}�C�R���J�[�̏�ԏ����� */
  motor_mode_f( BRAKE, BRAKE ); //BRAKE
  motor_mode_r( BRAKE, BRAKE );
  motor_f( 0, 0 );
  motor_r( 0, 0 );
  servoPwmOut( 0 );
  setBeepPatternS( 0x8000 );
  saka0_ad = ad5;
  x=0;
  timer(1000);
  
  setBeepPatternS( 0xcc00 );  
  
  /* �������[�v�O��LCD/SW�ő��x�ݒ� */
  while( !x ){
  	x=paramSettings(); // �ݒ�I����1
  }
  
  lcdPosition(0,0);
  lcdPrintf("DATA_FLASH   ");
  lcdPosition(0,1);
  lcdPrintf("Loading Now...");
  blockEraseDataFlash( DF_ADDR_START );
  ret = writeDataFlash( DF_ADDR_START, data_buff, DF_PARA_SIZE );
  
  if(ret == 0){ // �������݃G���[�Ȃ�
	lcdPosition(0,0);
  	lcdPrintf("DATA_FLASH   ");
  	lcdPosition(0,1);
  	lcdPrintf("ERROR!!!     ");
	timer(3000);
  }else{
	lcdPosition(0,0);
  	lcdPrintf("DATA_FLASH  OK  ");
  	lcdPosition(0,1);
  	lcdPrintf("%02d,%02d,%02d,%02d,%02d",
				data_buff[DF_TESTSPEED], data_buff[DF_CURVE_SPEED], data_buff[DF_LANE_L], data_buff[DF_LANE_R], data_buff[DF_CRANK_SPEED]);
	timer(1000);
  }
  //parameter_set();
   
  lcdPosition(0,0);
  lcdPrintf("STAND-BY Now...");
  lcdPosition(0,1);
  lcdPrintf("%02d,%02d,%02d,%02d,%02d",
				data_buff[DF_TESTSPEED], data_buff[DF_CURVE_SPEED], data_buff[DF_LANE_L], data_buff[DF_LANE_R], data_buff[DF_CRANK_SPEED]);
  
  while ( 1 ) {
	
    //���j���[�ȊO
    if (pattern != 1 && pattern != 2 && pattern != 3 && pattern != 4 && pattern != 5 && pattern != 6 && pattern != 7
        && pattern != 8 && pattern != 9 && pattern != 10 && pattern <= 100) {
      //�ʏ�g���[�X�ł̃R�[�X�A�E�g
      if (pattern != 32 && pattern != 33 && pattern != 42 && pattern != 43 && pattern != 61 && pattern != 62 &&
          pattern != 63 && pattern != 71 && pattern != 72 && pattern != 73) {
        if ((lEncoderTotal - causeout) >= 5000) {
          pattern = 101;
		  if (endFlag == 0){
		  	endFlag = 1;	  
		  }
        }
      }
      //�A�N�V�����ł̃R�[�X�A�E�g
      else if ((pattern == 32 || pattern == 33 || pattern == 42 || pattern == 43 || pattern == 61 || pattern == 62 || pattern
               == 63 || pattern == 71 || pattern == 72 || pattern == 73) && (pattern <= 100)) {
        if ((lEncoderTotal - causeout) >= 8500) {
          pattern = 101;
		  if (endFlag == 0){
		  	endFlag = 1;	  
		  }
		}
      }
    }
	if( pattern >= 11 && pattern <= 100 ){
		// �⓹����
		sakaSyori(); // �⓹������main�֐���whileٰ�ߓ��ɓ����
		if((lEncoderTotal >= (data_buff[DF_ENC_END]*1091L)) || endFlag == 1){ // �G���R�[�_��data_buff[DF_ENC_END]�Ȃ��~
			if( pattern == 11 || pattern == 12 || pattern == 13 || pattern == 14){
				pattern = 101;	
			}
		}
	}
	
    switch ( pattern ) {
      case 0:
        /* �v�b�V���X�C�b�`�����҂� */
       gain = 0;
       servoPwmOut( 0 );
       lEncoderTotal = 0;
       lEncoderCrank = 0;
	   lEncoderCorner = 0;
       causeout = 0;
       iAngleBefore2 = 0;
       M_FreeMoter = 0;
	   
//        streetspeed = (dipsw2_pattern[dipsw_get2()] + 5); //����
//        cornerspeed = dipsw2_pattern[dipsw_get2()]; //�Ȑ�
//        lanechangespeed = dipsw3_pattern[dipsw_get3()]; //�Ԑ��ύX
//        crankspeed = dipsw4_pattern[dipsw_get4()];//�N�����N
		
		if ( pushsw_get() && cnt1 >= 100) {
			if( msdError == 0 ){
				/* microSD�̋󂫗̈�֏������� */
				i++;
				if(i >= 10000 ) i = 1;
				ret = writeMicroSDNumber(i);
				if( ret == -1 ){
					msdError = 4;	
				}	
				else{
					/* �t�@�C�����ϊ� */
					sprintf( fileName,"log%04d.csv",i);	
				}
			}
			if( msdError == 0 ){
				/* �t�@�C���̃^�C���X�^���v�Z�b�g */
				setDateStamp( getCompileYear( C_DATE ),
					getCompileMonth( C_DATE ), getCompileDay( C_DATE ) );
				setTimeStamp( getCompileHour( C_TIME ),
					getCompilerMinute( C_TIME ), getCompilerSecond( C_TIME ) );
				
				/* �������݃t�@�C�����쐬 */
				// �������݂���������[ms] : x = 10[ms] : 64�o�C�g
				// 60000ms�Ȃ�Ax = 60000 * 64 / 10 = 384000
				// ���ʂ�512�̔{���ɂȂ�悤�ɌJ��グ����B
				ret = writeFile( fileName, 384000 );
				if( ret != 0x00 ) msdError = 11;
				
				// microSD��������
				msdPrintf("[TB-001]TypeD Log Data\n");
				while( checkMsdPrintf()); // msdPrintf�����҂�
				msdPrintf("Compile Date: ");
				while( checkMsdPrintf()); // msdPrintf�����҂�
				msdPrintf(C_DATE);
				while( checkMsdPrintf()); // msdPrintf�����҂�
				msdPrintf(" Time: ");
				while( checkMsdPrintf()); // msdPrintf�����҂�
				msdPrintf(C_TIME);
				while( checkMsdPrintf()); // msdPrintf�����҂�
				msdPrintf("\nSPEED=%d,CORNER_S=%d,L_LANE=%d,R_LANE=%d,CRANK=%d,ENC_END=%d",data_buff[DF_TESTSPEED], data_buff[DF_CURVE_SPEED], data_buff[DF_LANE_L], data_buff[DF_LANE_R], data_buff[DF_CRANK_SPEED], data_buff[DF_ENC_END]);
				while( checkMsdPrintf()); // msdPrintf�����҂�
				msdPrintf("\n\nLineNo,Pattern,Sensor,Center,Analog,Angle,Encoder,LeftLine,RightLine,CrossLine,saka_pot,saka_flag,lEncoderTotal\n");
				while( checkMsdPrintf()); // msdPrintf�����҂�
				
				/*
				// �����p
				line_no,			// �s�ԍ�
				pattern,			// ����p�^�[��
				sensor_inp(),  		// �f�W�^��(4bit)
				center_inp()+'0', 	// �f�W�^��(���S)
				getAnalogSensor(),  // �A�i���O�Z���T
				getServoAngle(),	// �{�����[��(�X�e�A�����O�p�x)
				iEncoder,          	// �G���R�[�_
				check_zlineL(),  	// �f�W�^����
				check_zlineR(), 	// �f�W�^���E
				check_crossline(), 	// �N���X���C���`�F�b�N
				ad5,				// �⓹���o�p�|�e���V�����[�^ 
				saka_flag			// �⌟�o�t���O
				lEncoderTotal      //  �G���R�[�_�̐ώZ�l
				*/
	
			}
			
        	M_FreeMoter = dipswf_pattern[dipsw_getf()]; //1 �Ńt���[���[�h 0 �Œʏ�
          	setBeepPatternS( 0xcc00 );
          	cnt1 = 0;
          	pattern = 1;
          	break;
		}
  		i = ((cnt1 / 200) % 2 + 1) | (startbar_get()<<7&0x80); // �X�^�[�g�o�[�����m�F
    	led_out( i ); /* LED �_�ŏ��� */
		break;
		
      case 1:
        /* �X�^�[�g�o�[�J�҂� */
        gain = 10;
        servoPwmOut( iServoPwm / 2 );
        causeout = lEncoderTotal;
		 timer(4000);
        if ( !startbar_get() ) {
		  iAngle0 = getServoAngle(); /* 0 �x�̈ʒu�L�� */
		  led_out( 0x0 );
		  if( msdError == 0 ) msdFlag = 1; /* �f�[�^�L�^�J�n */
		  saka0_ad = ad5; /* ���n�̍⓹�{�����[��A/D�l�L��*/
		  cnt1 = 0;
		  pattern = 11; //11 �ʏ푖�s 3 �X�e�A�����O�m�F 4 ���[�^�[���x
		  break;
        }
		
        led_out( 1 << (cnt1 / 50) % 4 | (startbar_get()<<7&0x80)); // �X�^�[�g�o�[�����m�F
        break;
		
      case 2: //��~
        gain = 10;
        servoPwmOut( 0 );
        motor_f( 0, 0 );
        motor_r( 0, 0 );
        led_out( 0xaa);
        break;
      case 3: //�X�e�A�����O
        gain = 10;
        servoPwmOut( iServoPwm );
        if (iEncoder < 5) {
          motor_f( 20, 20 );
          motor_r( 20, 20 );
        }
        else {
          motor_f( 0, 0 );
          motor_r( 0, 0 );
        }
        break;
      case 4: //���[�^���x
        gain = 0;
        servoPwmOut( 0 );
        if (iEncoder < 10) {
          motor_f( -50, -50 );
          motor_r(-50, -50 );
        }
        else {
          motor_f( 0, 0 );
          motor_r( 0, 0 );
        }
        break;
      case 5: //�n���h���p�x
        handle( 0 ); //120 �ȉ��ɂ���
        motor_f( 0, 0 );
        motor_r( 0, 0 );
        break;
      case 6:
        gain = 10;
        servoPwmOut( 0 );
        motor_f( 0, 0 );
        motor_r( 0, 0 );
        led_out( 0xf0 );
        causeout = lEncoderTotal;
        if (cnt1 >= 200) {
          M_FreeMoter = 0;
          cnt1 = 0;
          pattern = 0;
          break;
        }
        break;
		
      case 11:				//�ʏ�g���[�X
		gain = 10;
		servoPwmOut(iServoPwm);
		crank_mode = 0;
		speed_target = data_buff[DF_TESTSPEED];	//�X�s�[�h�ڕW�@�p�����[�^TESTSPEED(�ʏ푖�s�X�s�[�h)	
		/*		�ʏ푖�s�̐���		  				*/
		/*      �����F����   ���ۂ̑��x >=	�ڕW���x */
		if(iEncoder >= speed_target){		
			motor_mode_f(BRAKE,BRAKE);	
			motor_mode_r(BRAKE,BRAKE);
			motor_f(1,1);	//�O �i��,�E�j
			motor_r(1,1);	//��i��,�E�j
		}
		/*		�ʏ푖�s�̐���		  				*/
		/*      �����F����   ���ۂ̑��x <	�ڕW���x*/
		else{
			if(abs(angle)>6){		//�R�[�i�[���s
				corner_run(90);	//�R�[�i�[���s(��O��MAX)�p���w��[%]
			}
			else{
				motor_mode_f(FREE,FREE);	
				motor_mode_r(FREE,FREE);
				motor_f(100,100); 		     //�O �i��,�E�j
				motor_r(100,100); 		 //��i��,�E�j
			}
		}
		if(abs(angle)>10){	//�R�[�i�[���s
			pattern=12;	//�R�[�i�[�̑��x���䏈����
		}
		/* ---------------���[���`�F���W�E�N���X���C������--------------- */
		if ( check_crossline()) { /* �N���X���C���`�F�b�N */
			servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
      	    causeout = lEncoderTotal;
			led_out(0x0f);
			setBeepPatternS( 0xcc00 );
			pattern = 21;
			break;
        }
  		if ( check_zlineR()) { // �E�Ԑ��ύX���C���`�F�b�N
			servoPwmOut( 0 );
         	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
          	Lane_Change = RIGHT; // 1:�� 0:�E
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x33 );
          	pattern = 51;
          	break;
		}
       if ( check_zlineL()) { // ���Ԑ��ύX���C���`�F�b�N
       		servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
         	Lane_Change = LEFT; // 1:�� 0:�E
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x44 );
          	pattern = 51;
          	break;
		}
       if ( !check_Noline() ) {
		   causeout = lEncoderTotal;
		}		
		/* ---------------���[���`�F���W�E�N���X���C�����肱���܂�--------------- */
	  	break;			
	
    case 12:					//�R�[�i�[�̑��x���䏈�� 12 < angle <= 30
		gain = 10;
		servoPwmOut(iServoPwm);
		corner_speed = data_buff[DF_CURVE_SPEED];//�X�s�[�h�ڕW�@�p�����[�^n_s(�ʏ푖�s�X�s�[�h)	
		if(abs(angle)<=12){
			pattern=11;
		}
		corner_run(90);			//�R�[�i�[���s(��O��MAX)�p���w��[%]
		if(corner_speed <= iEncoder){	
		//	kyori=100;	
			pattern=13;
			lEncoderCorner = lEncoderTotal;
		}
		/* ---------------���[���`�F���W�E�N���X���C������--------------- */
		if ( check_crossline()) { /* �N���X���C���`�F�b�N */
			servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
      	    causeout = lEncoderTotal;
			led_out(0x0f);
			setBeepPatternS( 0xcc00 );
			pattern = 21;
			break;
        }
  		if ( check_zlineR()) { // �E�Ԑ��ύX���C���`�F�b�N
			servoPwmOut( 0 );
         	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
          	Lane_Change = RIGHT; // 1:�� 0:�E
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x33 );
          	pattern = 51;
          	break;
		}
       if ( check_zlineL()) { // ���Ԑ��ύX���C���`�F�b�N
       		servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
         	Lane_Change = LEFT; // 1:�� 0:�E
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x44 );
          	pattern = 51;
          	break;
		}
       if ( !check_Noline() ) {
		   causeout = lEncoderTotal;
		}
		break;
		/* ---------------���[���`�F���W�E�N���X���C�����肱���܂�--------------- */

    case 13:	//�R�[�i�[�̃u���[�L�����@�P�i�K�� 100mm
		/*�p�^�[��13�ɂȂ�����100 [mm]�u���[�L���s*/ 
		gain = 10;
		servoPwmOut(iServoPwm);
		motor_mode_f(BRAKE,BRAKE);	
		motor_mode_r(BRAKE,BRAKE);	
		motor_f(-30,-30);	//�O�i��,�E�j
		motor_r(-30,-30); 	//��i��,�E�j
		if(iEncoder < data_buff[DF_CURVE_SPEED] ){	
			pattern = 12;
			lEncoderCorner = lEncoderTotal;
		}
		if( lEncoderTotal - lEncoderCorner >= 1091L){ // 100mm���s
			lEncoderCorner = lEncoderTotal;
			pattern = 14;
			break;
		} 
		if(abs(angle)<=12){
			pattern=11;
		}
		/* ---------------���[���`�F���W�E�N���X���C������--------------- */
		if ( check_crossline()) { /* �N���X���C���`�F�b�N */
			servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
      	    causeout = lEncoderTotal;
			led_out(0x0f);
			setBeepPatternS( 0xcc00 );
			pattern = 21;
			break;
        }
  		if ( check_zlineR()) { // �E�Ԑ��ύX���C���`�F�b�N
			servoPwmOut( 0 );
         	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
          	Lane_Change = RIGHT; // 1:�� 0:�E
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x33 );
          	pattern = 51;
          	break;
		}
       if ( check_zlineL()) { // ���Ԑ��ύX���C���`�F�b�N
       		servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
         	Lane_Change = LEFT; // 1:�� 0:�E
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x44 );
          	pattern = 51;
          	break;
		}
       if ( !check_Noline() ) {
		   causeout = lEncoderTotal;
		}
		/* ---------------���[���`�F���W�E�N���X���C�����肱���܂�--------------- */
	  	break;

    case 14:	//�R�[�i�[�̃u���[�L�����@2�i�K�� 100mm
		servoPwmOut(iServoPwm);
		gain = 10;
		motor_mode_f(BRAKE,BRAKE);	
		motor_mode_r(BRAKE,BRAKE);	
		motor_f(0,0);	//�O�i��,�E�j
		motor_r(-30,-30); 	//��i��,�E�j
		if(iEncoder < data_buff[DF_CURVE_SPEED] ){	
			pattern = 12;
			lEncoderCorner = lEncoderTotal;
		}
		if( lEncoderTotal - lEncoderCorner >= 1091L){ // 100mm���s
			pattern = 12;
			lEncoderCorner = lEncoderTotal;
		}
		if(abs(angle)<=12){
			pattern=11;
		}
		/* ---------------���[���`�F���W�E�N���X���C������--------------- */
		if ( check_crossline()) { /* �N���X���C���`�F�b�N */
			servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
      	    causeout = lEncoderTotal;
			led_out(0x0f);
			setBeepPatternS( 0xcc00 );
			pattern = 21;
			break;
        }
  		if ( check_zlineR()) { // �E�Ԑ��ύX���C���`�F�b�N
			servoPwmOut( 0 );
         	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
          	Lane_Change = RIGHT; // 1:�� 0:�E
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x33 );
          	pattern = 51;
          	break;
		}
       if ( check_zlineL()) { // ���Ԑ��ύX���C���`�F�b�N
       		servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
         	Lane_Change = LEFT; // 1:�� 0:�E
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x44 );
          	pattern = 51;
          	break;
		}
       if ( !check_Noline() ) {
		   causeout = lEncoderTotal;
		}
		/* ---------------���[���`�F���W�E�N���X���C�����肱���܂�--------------- */
	  	break;

     case 21:
        /* �N���X���C���ʉߏ��� */
		gain = 10;
		crank_mode = 1;
        servoPwmOut( iServoPwm );
        led_out( 0xff );
        if ( iEncoder >= data_buff[DF_CRANK_SPEED] ) {         /* �G���R�[�_�ɂ��X�s�[�h���� */
          motor_mode_f( BRAKE, BRAKE );
          motor_mode_r( BRAKE, BRAKE );
          motor_f( -100, -100 );
          motor_r( -100, -100 );
        } else if(iEncoder < data_buff[DF_CRANK_SPEED]){
          motor_mode_f( BRAKE, BRAKE );
          motor_mode_r( BRAKE, BRAKE );
          motor_f( 10, 10 );
          motor_r( 10, 10 );
        } else {
          motor_mode_f( BRAKE, BRAKE );
          motor_mode_r( BRAKE, BRAKE );
		   motor_f( 0, 0 );
		   motor_r( 0, 0 );
        }
		if ( lEncoderTotal - lEncoderCrank >= 109L ) {
          cnt1 = 0;
          pattern = 22;
        }
		
        break;

      case 22:
        /* �N���X���C����̃g���[�X�A���p���o���� */
        gain = 10;
        servoPwmOut( iServoPwm );
        motor_mode_f( BRAKE, BRAKE );
        motor_mode_r( BRAKE, BRAKE );
        if ( iEncoder >= data_buff[DF_CRANK_SPEED] ) {         /* �G���R�[�_�ɂ��X�s�[�h���� */
          motor_f( -100, -100 );
          motor_r( -100, -100 );
        } else if(iEncoder < data_buff[DF_CRANK_SPEED]) {
          motor_f( 50, 50 );
          motor_r( 50, 50 );
        } else {
			motor_f( 0, 0 );
			motor_r( 0, 0 );
		 }

        if ( center_inp() == 1 && ((sensor_inp() & 0x01) == 0x01) ) { /* �E�N�����N�H             */
          led_out( 0x1 );
          cnt1 = 0;
		   lEncoderCrank = lEncoderTotal;
          pattern = 31;
          break;
        }
        if ( center_inp() == 1 && ((sensor_inp() & 0x08) == 0x08) ) { /* ���N�����N�H            */
          led_out( 0x2 );
          cnt1 = 0;
		   lEncoderCrank = lEncoderTotal;
          pattern = 41;
          break;
        }
        break;

      case 31:
        /* �E�N�����N���� */
        servoPwmOut( 100 );         /* �U�肪�ア�Ƃ��͑傫������       */
        motor_mode_f(BRAKE, FREE);
        motor_mode_r(BRAKE, FREE);
        // diff�g�p�֎~!!������
        motor_f( 100, -60 );          /* ���̕����́u�p�x�v�Z(4WD��).xls�v*/
        motor_r( 80, -40 );          /* �Ōv�Z                           */
        if ( sensor_inp() == 0x04) {   /* �Ȃ��I���`�F�b�N           */
          cnt1 = 0;
          iSensorPattern = 0;
          lEncoderCrank = lEncoderTotal;
          crank_mode = 0;
          pattern = 32;
        }
        break;

      case 32:
        /* �������Ԃ��o�܂ő҂� */
        servoPwmOut( iServoPwm );
        motor_mode_f( BRAKE, BRAKE );
        motor_mode_r( BRAKE, BRAKE );
        if (iEncoder <= data_buff[DF_TESTSPEED]) { //43 50
          motor_f( 80, 80 );
          motor_r( 80, 80 );
        }
		else{		
		  motor_f( 0, 0 );
          motor_r( 0, 0 );
        }
        if ( lEncoderTotal - lEncoderCrank >= 109L ) {
          cnt1 = 0;
          led_out( 0x0 );
          pattern = 11;
		  break;
        }
		// ��10cm�i�ފԂɃ��[���`�F���W�E�N���X���C�������Ă��F���ł���悤�ɂɂ���
		
		/* ---------------���[���`�F���W�E�N���X���C������--------------- */
		if ( check_crossline()) { /* �N���X���C���`�F�b�N */
			servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
      	    causeout = lEncoderTotal;
			led_out(0x0f);
			setBeepPatternS( 0xcc00 );
			pattern = 21;
			break;
        }
  		if ( check_zlineR()) { // �E�Ԑ��ύX���C���`�F�b�N
			servoPwmOut( 0 );
         	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
          	Lane_Change = RIGHT; // 1:�� 0:�E
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x33 );
          	pattern = 51;
          	break;
		}
       if ( check_zlineL()) { // ���Ԑ��ύX���C���`�F�b�N
       		servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
         	Lane_Change = LEFT; // 1:�� 0:�E
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x44 );
          	pattern = 51;
          	break;
		}
       if ( !check_Noline() ) {
		   causeout = lEncoderTotal;
		}
		/* ---------------���[���`�F���W�E�N���X���C�����肱���܂�--------------- */
	  	
		
        break;

      case 41:
	  	servoPwmOut( -100 );         /* �U�肪�ア�Ƃ��͑傫������       */
        motor_mode_f(FREE, BRAKE);
        motor_mode_r(FREE, BRAKE);
        // diff�g�p�֎~!!������
        motor_f( -60, 100 );          /* ���̕����́u�p�x�v�Z(4WD��).xls�v*/
        motor_r( -40, 80 );           /* �Ōv�Z                           */
        if ( sensor_inp() == 0x02) {   /* �Ȃ��I���`�F�b�N           */
          cnt1 = 0;
          iSensorPattern = 0;
          crank_mode = 0;
          lEncoderCrank = lEncoderTotal;
          pattern = 42;
        }
        break;
	  
	  case 42:
	  	/* �������Ԃ��o�܂ő҂� */
        servoPwmOut( iServoPwm );
        motor_mode_f( BRAKE, BRAKE );
        motor_mode_r( BRAKE, BRAKE );
        if (iEncoder <= data_buff[DF_TESTSPEED]) { //43 50
          motor_f( 80, 80 );
          motor_r( 80, 80 );
        }
		else{		
		  motor_f( 0, 0 );
          motor_r( 0, 0 );
        }
        if ( lEncoderTotal - lEncoderCrank >= 60L ) {
          cnt1 = 0;
          led_out( 0x0 );
          pattern = 11;
		  break;
        }
		// ��10cm�i�ފԂɃ��[���`�F���W�E�N���X���C�������Ă��F���ł���悤�ɂɂ���
		
		/* ---------------���[���`�F���W�E�N���X���C������--------------- */
		if ( check_crossline()) { /* �N���X���C���`�F�b�N */
			servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
      	    causeout = lEncoderTotal;
			led_out(0x0f);
			setBeepPatternS( 0xcc00 );
			pattern = 21;
			break;
        }
  		if ( check_zlineR()) { // �E�Ԑ��ύX���C���`�F�b�N
			servoPwmOut( 0 );
         	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
          	Lane_Change = RIGHT; // 1:�� 0:�E
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x33 );
          	pattern = 51;
          	break;
		}
       if ( check_zlineL()) { // ���Ԑ��ύX���C���`�F�b�N
       		servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
         	Lane_Change = LEFT; // 1:�� 0:�E
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x44 );
          	pattern = 51;
          	break;
		}
       if ( !check_Noline() ) {
		   causeout = lEncoderTotal;
		}
		/* ---------------���[���`�F���W�E�N���X���C�����肱���܂�--------------- */
	  	break;
		
      case 51: // �N�����N�`�F�b�N
        gain = 10;
        servoPwmOut( 0 );
        if (iEncoder <= data_buff[DF_TESTSPEED]) { //43 50
          motor_f( 80, 80 );
          motor_r( 80, 80 );
        }
        else if (iEncoder > data_buff[DF_TESTSPEED]) {
          motor_f( 0, 0);
          motor_r( 0, 0);
        }
        if (((lEncoderTotal - lEncoderCrank) <= 109L) && check_crossline()) {
          lEncoderCrank = lEncoderTotal;
          causeout = lEncoderTotal;
          pattern = 21;
          break;
        }
        else if ((lEncoderTotal - lEncoderCrank) > 109L) { // 10mm �� 3
          lEncoderCrank = lEncoderTotal;
          causeout = lEncoderTotal;
          pattern = 52;
          break;
        }
        if ( !check_Noline() ) {
          causeout = lEncoderTotal;
        }
        break;
		
      case 52: // ���C���I���T�[�`
        gain = 10;
        servoPwmOut( iServoPwm );
        if (iEncoder <= data_buff[DF_TESTSPEED]) { //45 50
          motor_f( 80, 80 );
          motor_r( 80, 80 );
        }
        else if (iEncoder > data_buff[DF_TESTSPEED]) {
          motor_f( 0, 0); //-100
          motor_r( 0, 0);
        }
        if ( check_Noline() && Lane_Change == RIGHT ) { // �E�Ԑ��ύX?
          crank_mode = 1;
          lEncoderCrank = lEncoderTotal;
          causeout = lEncoderTotal;
          led_out( 0x0f );
          pattern = 61;
          break;
        } else if ( check_Noline() && Lane_Change == LEFT ) { // ���Ԑ��ύX�H
          crank_mode = 1;
          lEncoderCrank = lEncoderTotal;
          causeout = lEncoderTotal;
          led_out( 0x0a );
          pattern = 71;
          break;
        }
		if( check_crossline() ){
			setBeepPatternS(0xaa00);
			crank_mode = 1;
          	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
          	led_out( 0xff );
          	pattern = 22;
			break;
		}
        causeout = lEncoderTotal;
        break;
		
      case 61: // (�E)�Ԑ��ύX���� 1
        handle( 45 );
        if (iEncoder <= data_buff[DF_LANE_R]) { //43 50
		  motor_mode_f(BRAKE,FREE);
          motor_mode_r(BRAKE,FREE);
		  motor_f( 90, 80 );
          motor_r( 90, 80 );
        }
        else if (iEncoder > data_buff[DF_LANE_R]) {
          motor_f( 0, 0);
          motor_r( 0, 0);
        }
        if ( (sensor_inp() == 0x01) && (sensor_inp() != 0x08) ) { // �f�W�^�� 1,2,4 �A�i���O L,R �^�C�������猩��
          iSensorPattern = 0;
          crank_mode = 0;
          lEncoderCrank = lEncoderTotal;
          causeout = lEncoderTotal;
          pattern = 62;
          break;
        }
        break;
		
      case 62: // (�E)�Ԑ��ύX���� 2
        handle( 1 );
        if (iEncoder <= data_buff[DF_LANE_R]) { //43 50
          motor_mode_f(BRAKE,BRAKE);
          motor_mode_r(BRAKE,BRAKE);
		  motor_f( 90, 90 );
          motor_r( 90, 90 );
        }
        else if (iEncoder > data_buff[DF_LANE_R]) {
          motor_f( 0, 0);
          motor_r( 0, 0);
        }
        if ( (sensor_inp() == 0x08 || sensor_inp() == 0x04) ) { //&& (sensor_inp() != 0x01)) {
          lEncoderCrank = lEncoderTotal;
          causeout = lEncoderTotal;
          pattern = 63;
          break;
        }
        break;
		
      case 63: // (�E)�Ԑ��ύX���� 3
        handle( 34 );
        if (iEncoder <= data_buff[DF_LANE_R]) { //43 50
		  motor_mode_f(FREE,BRAKE);
          motor_mode_r(FREE,BRAKE);
          motor_f( 80, 90 );
          motor_r( 80, 90 );
        }
        else if (iEncoder > data_buff[DF_LANE_R]) {
          motor_f( 0, 0);
          motor_r( 0, 0);
        }
        if ( sensor_inp() == 0x02 || sensor_inp() == 0x01 || center_inp() == 0x01 ) {
          lEncoderCrank = lEncoderTotal;
          causeout = lEncoderTotal;
          pattern = 64;
          break;
        }
        break;
		
      case 64: // (�E)�Ԑ��ύX���� 4
        gain = 10;
        servoPwmOut( iServoPwm );
        if (iEncoder <= data_buff[DF_LANE_R]) { //43 50
		  motor_mode_f(BRAKE,BRAKE);
          motor_mode_r(BRAKE,BRAKE);
          motor_f( 90, 90 );
          motor_r( 80, 80 );
        }
        else if (iEncoder > data_buff[DF_LANE_R]) {
          motor_f( 0, 0);
          motor_r( 0, 0);
        }
        if ( (lEncoderTotal - lEncoderCrank) >= (109L) ) { // 10mm �� 3 150mm ��ɒʏ�g���[�X�֑J��
          lEncoderCrank = lEncoderTotal;
          causeout = lEncoderTotal;
          pattern = 11;
          break;
        }
        if ( !check_Noline() ) {
          causeout = lEncoderTotal;
        }
		if( check_crossline()){
			pattern = 21;
			crank_mode = 1;
			lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
          	cnt1 = 0;
		}
		
        break;
		
      case 71: // (��)�Ԑ��ύX���� 1
        handle( -45 );
        if (iEncoder <= data_buff[DF_LANE_L]) { //43 50
		  motor_mode_f(BRAKE,BRAKE);
          motor_mode_r(BRAKE,BRAKE);
          motor_f( diff(70), 70 );
          motor_r( diff(80), 80 );
        }
        else if (iEncoder > data_buff[DF_LANE_L]) {
          motor_f( 0, 0);
          motor_r( 0, 0);
        }
        if ( (sensor_inp()&0x08 == 0x08) && (sensor_inp() != 0x01)) {
          iSensorPattern = 0;
          crank_mode = 0;
          lEncoderCrank = lEncoderTotal;
          causeout = lEncoderTotal;
          pattern = 72;
          break;
        }
        break;
		
      case 72: // (��)�Ԑ��ύX���� 2
        handle( -1 );
        if (iEncoder <= data_buff[DF_LANE_L]) { //43 50
		  motor_mode_f(BRAKE,BRAKE);
          motor_mode_r(BRAKE,BRAKE);
          motor_f( 70, 70 );
          motor_r( 70, 70 );
        }
        else if (iEncoder > data_buff[DF_LANE_L]) {
          motor_f( 0, 0);
          motor_r( 0, 0);
        }
        if ( (sensor_inp()&0x01 == 0x01 || sensor_inp()&0x02 == 0x02)) { // && (sensor_inp() != 0x08)) {
          lEncoderCrank = lEncoderTotal;
          causeout = lEncoderTotal;
          pattern = 73;
          break;
        }
        break;
		
      case 73: // (��)�Ԑ��ύX���� 3
        handle( 80 );
        if (iEncoder <= data_buff[DF_LANE_L]) { //43 50
		  motor_mode_f(BRAKE,BRAKE);
          motor_mode_r(BRAKE,BRAKE);
          motor_f( 70, diff(70) );
          motor_r( 70, diff(70) );
        }
        else if (iEncoder > data_buff[DF_LANE_L]) {
          motor_f( 0, 0);
          motor_r( 0, 0);
        }
        if ( sensor_inp() == 0x04 || sensor_inp() == 0x08 || sensor_inp() == 0x0c || center_inp() == 0x01) {
          lEncoderCrank = lEncoderTotal;
          causeout = lEncoderTotal;
          pattern = 74;
          break;
        }
        break;
		
      case 74: // (��)�Ԑ��ύX���� 4
        gain = 10;
        servoPwmOut( iServoPwm );
        if (iEncoder <= data_buff[DF_LANE_L]) { //43 50
		  motor_mode_f(BRAKE,BRAKE);
          motor_mode_r(BRAKE,BRAKE);
          motor_f( 50, 50 );
          motor_r( 50, 50 );
        }
        else if (iEncoder > data_buff[DF_LANE_L]) {
          motor_f( 0, 0);
          motor_r( 0, 0);
        }
        if ((lEncoderTotal - lEncoderCrank) >= (109L) ) { // 10mm �� 3 150mm ��ɒʏ�g���[�X�֑J��
          lEncoderCrank = lEncoderTotal;
          causeout = lEncoderTotal;
          pattern = 11;
          break;
        }
        if ( !check_Noline() ) {
          causeout = lEncoderTotal;
        }
		if( check_crossline() ){
			pattern = 21;
			lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
          	cnt1 = 0;
		}
        break;

      case 101:
        /* ��~���� */
		gain=10;
        servoPwmOut( iServoPwm );
        motor_mode_f( FREE, FREE );
        motor_mode_r( FREE, FREE );
        motor_f( 0, 0 );
        motor_r( 0, 0 );
        crank_mode = 1;
        pattern = 102;
        cnt1 = 0;
		lcdPosition( 0, 0 );
  		lcdPrintf("FINISH!!!  " );
        break;

      case 102:
        servoPwmOut( iServoPwm );
        motor_mode_f( FREE, FREE );
        motor_mode_r( FREE, FREE );
        motor2_f( 0, 0 );
        motor2_r( 0, 0 );
		msdFlag = 0;
        if ( cnt1 > 300) {
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
        if ( iEncoder <= 1) {
          setBeepPatternS( 0xcc00 );
          pattern = 104;
		  msdFlag = 0;
          cnt1 = 0;
        }
        break;
	 
	  case 104:
	 	servoPwmOut( iServoPwm );
        motor_mode_f( BRAKE, BRAKE );
        motor_mode_r( BRAKE, BRAKE );
        motor2_f( 0, 0 );
        motor2_r( 0, 0 );
	  	if( microSDProcessEnd() == 0 ){
			pattern = 105;
			cnt1 = 0;
		}
		break;
      case 105:
        /* �������Ȃ� */
        servoPwmOut( 0 );
        motor_mode_f( BRAKE, BRAKE );
        motor_mode_r( BRAKE, BRAKE );
        motor2_f( 0, 0 );
        motor2_r( 0, 0 );
        msdFlag = 0;
		
		if ( cnt1 < 75 ) {
          led_out(0x00);
        }
        else if ( cnt1 < 150 ) {
          led_out(0xff);
        }
        else {
          cnt1 = 0;
		  lcdPosition( 0, 1 );
  		  lcdPrintf("ENC= %04ld       ",lEncoderTotal );
        }
        break;

      default:
        break;
    }
  }
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
  pd5 = 0x7f;

  /*  none            none            none            none
      none            none            Arduino(ZERO)   Arduino(MODE)   */
  p6  = 0x00;
  pd6 = 0xef;

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
/* �^�C�} RB ���荞�ݏ��� */
/************************************************************************/
#pragma interrupt /B intTRB(vect=24)
void intTRB( void ) {
  unsigned int i;
  volatile static unsigned int line_no;
  asm(" fset I "); /* �^�C�} RB �ȏ�̊��荞�݋��� */
  cnt0++;
  cnt1++;
  cnt_saka++;
  /* �p�x�langle, kakudo��1ms�����Ɍv�Z */
  angle = getServoAngle();
  kakudo = angle / SERVO_STEP; /* 1 �x������̑����Ŋ��� */
  if ( kakudo < 0 ) kakudo = -kakudo;
  if ( kakudo > 60 ) kakudo = 60; //45
  
  
  /* �T�[�{���[�^���� */
  servoControl();
  if( pattern >= 1 && pattern <= 100 ){
  	handle(0);
  }
  /* �u�U�[���� */
  beepProcessS();
  /* microSD�Ԍ��������ݏ���(1ms���ƂɎ��s) */
  microSDProcess();
  
  	/* lcd���� */
  	lcdShowProcess();
 	 /* �g���X�C�b�`�p�֐�(1ms���ƂɎ��s) */
  	switchProcess();

  
  /* 10 �� 1 ����s���鏈�� */
  /*�s���� 4 ��ɂ��Ă���*/
  iTimer10++;
  switch ( iTimer10 ) {
    case 1:
      /* �G���R�[�_���� */
      i = trg;
      iEncoder = i - uEncoderBuff;
      lEncoderTotal += iEncoder;
      p4_5 = p3_0; // R8C/38A�{�[�h��LED�ɃG���R�[�_�̏�Ԃ��o��
      uEncoderBuff = i;
      break;
    case 2:
      /* �X�C�b�`�ǂݍ��ݏ��� */
      p9_4 = 0; /* LED �o�� OFF */
      pd8 = 0x00;
      break;
    case 3:
      /* �X�C�b�`�ǂݍ��݁ALED �o�� */
      types_dipsw = ~p8; /* ��ײ�ފ�� TypeS Ver.3 �� SW �ǂݍ���*/
      p8 = types_led; /* ��ײ�ފ�� TypeS Ver.3 �� LED �֏o��*/
      pd8 = 0xff;
      p9_4 = 1; /* LED �o�� ON */
      break;
	case 4:
	/* microSD�L�^���� */
	  if( msdFlag == 1 ){
			msdPrintf("%4d,%3d,=\"%4b\",%c,%5d,%4d,%2d,%4d,%4d,%4d,%4d,%1d,%d\r\n",
				line_no,			// �s�ԍ�
				pattern,			// ����p�^�[��
				sensor_inp(),  		// �f�W�^��(4bit)
				center_inp()+'0', 	// �f�W�^��(���S)
				getAnalogSensor(),  // �A�i���O�Z���T
				getServoAngle(),	// �{�����[��(�X�e�A�����O�p�x)
				iEncoder,          	// �G���R�[�_
				check_zlineL(),  	// �f�W�^����
				check_zlineR(), 	// �f�W�^���E
				check_crossline(), 	// �N���X���C���`�F�b�N
				ad5,				// �⓹���o�p�|�e���V�����[�^ 
				saka_flag,			// �⌟�o�t���O
				lEncoderTotal		// �G���R�[�_�ώZ�l
			);
			if(++line_no >= 10000 ) line_no = 0; 
	  }
	  break;
	case 10: //�O��� 4 ������
      /* iTimer10 �ϐ��̏��� */
      iTimer10 = 0;
      break;
  }
  
  if(pattern == 1 ){
	servoPwmOut(iServoPwm);	  
  }
}
/************************************************************************/
/* �^�C�} RC ���荞�ݏ��� */
/************************************************************************/
#pragma interrupt intTRC(vect=7)
void intTRC( void )
{
  trcsr &= 0xfe;
  /* �^�C�} RC �f���[�e�B��̐ݒ� */
  trcgrb = trcgrb_buff;
  trcgrd = trcgrd_buff;
}
void timer(unsigned long timer_set) {
  cnt0 = 0;
  while (cnt0 <= timer_set);
}
/************************************************************************/
/* �A�i���O�Z���T��� TypeS Ver.2 �̃f�W�^���Z���T�l�ǂݍ��� */
/* ���� �Ȃ� */
/* �߂�l ���[�A�����A�E���A�E�[�̃f�W�^���Z���T 0:�� 1:�� */
/************************************************************************/
unsigned char sensor_inp( void ) //0,0,0,0 �@�̂��猩��
{
  unsigned char sensor;
  sensor = ~p0 & 0x0f;
  return sensor;
}
/************************************************************************/
/* �A�i���O�Z���T��� TypeS Ver.2 �̒��S�f�W�^���Z���T�ǂݍ��� */
/* ���� �Ȃ� */
/* �߂�l ���S�f�W�^���Z���T 0:�� 1:�� */
/************************************************************************/
unsigned char center_inp( void )
{
  unsigned char sensor;
  sensor = ~p1_7 & 0x01;
  return sensor;
}
/************************************************************************/
/* �A�i���O�Z���T��� TypeS Ver.2 �̃X�^�[�g�o�[���o�Z���T�ǂݍ��� */
/* ���� �Ȃ� */
/* �߂�l 0:�X�^�[�g�o�[�Ȃ� 1:�X�^�[�g�o�[���� */
/************************************************************************/
unsigned char startbar_get( void )
{
  unsigned char sensor;
  sensor = ~p1_6 & 0x01;
  return sensor;
}
/************************************************************************/
/* �}�C�R���{�[�h��̃f�B�b�v�X�C�b�`�l�ǂݍ��� */
/* ���� �Ȃ� */
/* �߂�l �X�C�b�`�l 0�`15 �����Ă��� */
/************************************************************************/
unsigned char dipsw_get( void )
{
  unsigned char sw;
  sw = p1 & 0x0e; /* P1_3�`P1_1 �ǂݍ��� */
  return sw;
}
unsigned char dipsw_getf( void )
{
  unsigned char sw;
  sw = p1 & 0x01; /*P1_0 �ǂݍ��� */
  return sw;
}
/************************************************************************/
/* ���[�^�h���C�u��� TypeS Ver.3 ��̃f�B�b�v�X�C�b�`�l�ǂݍ��� */
/* ���� �Ȃ� */
/* �߂�l �X�C�b�`�l 0�`255 �Ӗ��Ȃ� */
/************************************************************************/
unsigned char dipsw_get2( void )
{
  /* ���ۂ̓��͂̓^�C�} RB ���荞�ݏ����Ŏ��{ */
  unsigned char sw;
  sw = p8 & 0xf0; /*P8_7�`P8_4 �ǂݍ��� */
  return sw;
  // return types_dipsw;
}
unsigned char dipsw_get3( void )
{
  /* ���ۂ̓��͂̓^�C�} RB ���荞�ݏ����Ŏ��{ */
  unsigned char sw;
  sw = p8 & 0x03; /* P8_1�`P8_0 �ǂݍ��� */
  return sw;
  // return types_dipsw;
}
unsigned char dipsw_get4( void )
{
  /* ���ۂ̓��͂̓^�C�} RB ���荞�ݏ����Ŏ��{ */
  unsigned char sw;
  sw = p8 & 0x0c; /* P8_3�`P8_2 �ǂݍ��� */
  return sw;
  // return types_dipsw;
}
/************************************************************************/
/* ���[�^�h���C�u��� TypeS Ver.3 ��̃v�b�V���X�C�b�`�l�ǂݍ��� */
/* ���� �Ȃ� */
/* �߂�l �X�C�b�`�l 0:OFF 1:ON */
/************************************************************************/
unsigned char pushsw_get( void )
{
  unsigned char sw;
  sw = ~p9_5 & 0x01;
  return sw;
}
/************************************************************************/
/* ���[�^�h���C�u��� TypeS Ver.3 �� CN6 �̏�ԓǂݍ��� */
/* ���� �Ȃ� */
/* �߂�l 0�`15 */
/************************************************************************/
unsigned char cn6_get( void )
{
  unsigned char data;
  data = p7 >> 4;
  return data;
}
/************************************************************************/
/* ���[�^�h���C�u��� TypeS Ver.3 �� LED ���� */
/* ���� 8 �� LED ���� 0:OFF 1:ON */
/* �߂�l �Ȃ� */
/************************************************************************/
void led_out( unsigned char led )
{
  /* ���ۂ̏o�͂̓^�C�} RB ���荞�ݏ����Ŏ��{ */
  types_led = led;
}
/************************************************************************/
/* ��ւ̑��x���� */
/* ���� �����[�^:-100�`100 , �E���[�^:-100�`100 */
/* 0 �Œ�~�A100 �Ő��] 100%�A-100 �ŋt�] 100% */
/* �߂�l �Ȃ� */
/************************************************************************/
void motor2_r( int accele_l, int accele_r ) {
  int sw_data;
  if ( M_FreeMoter == 1 ) {
    accele_l = 1;
    accele_r = 1;
  }
  sw_data = dipsw2_pattern[dipsw_get2()] + 5; /* �f�B�b�v�X�C�b�`�ǂݍ��� */
  accele_l = -accele_l * sw_data / 20;
  accele_r = -accele_r * sw_data / 20;

  accele_r = -accele_r;
  // ���ヂ�[�^
  if ( accele_l >= 0 ) {
    p2_1 = 0;
    trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
  } else {
    p2_1 = 1;
    trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
  }
  // �E�ヂ�[�^
  if ( accele_r >= 0 ) {
    p2_3 = 0;
    trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
  } else {
    p2_3 = 1;
    trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
  }
}
/************************************************************************/
/* ��ւ̑��x���� 2 �f�B�b�v�X�C�b�`�ɂ͊֌W���Ȃ� motor �֐� */
/* ���� �����[�^:-100�`100 , �E���[�^:-100�`100 */
/* 0 �Œ�~�A100 �Ő��] 100%�A-100 �ŋt�] 100% */
/* �߂�l �Ȃ� */
/************************************************************************/
void motor_r( int accele_l2, int accele_r2 )
{
  int accele_l, accele_r;

  if ( M_FreeMoter == 1 ) {
    acceleFree = 1;
    accele_l = acceleFree;
    accele_r = acceleFree;
  }
  else {
    accele_l = accele_l2;
    accele_r = accele_r2;
  }

  accele_r = -accele_r;
  // ���ヂ�[�^
  if ( accele_l >= 0 ) {
    p2_1 = 1;
    trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
  } else {
    p2_1 = 0;
    trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
  }
  // �E�ヂ�[�^
  if ( accele_r >= 0 ) {
    p2_3 = 1;
    trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
  } else {
    p2_3 = 0;
    trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
  }
}
/************************************************************************/
/* �O�ւ̑��x���� */
/* ���� �����[�^:-100�`100 , �E���[�^:-100�`100 */
/* 0 �Œ�~�A100 �Ő��] 100%�A-100 �ŋt�] 100% */
/* �߂�l �Ȃ� */
/************************************************************************/
void motor2_f( int accele_l, int accele_r ) {
  int sw_data;
  if ( M_FreeMoter == 1 ) {
    accele_l = 1;
    accele_r = 1;
  }
  sw_data = dipsw2_pattern[dipsw_get2()] + 5; /* �f�B�b�v�X�C�b�`�ǂݍ��� */
  accele_l = accele_l * sw_data / 20;
  accele_r = accele_r * sw_data / 20;
  accele_l = -accele_l;
  // ���O���[�^
  if ( accele_l >= 0 ) {
    p2_0 = 0;
  } else {
    p2_0 = 1;
    accele_l = -accele_l;
  }
  if ( accele_l <= 5 ) {
    trcgrb = trcgrb_buff = trcgra;
  } else {
    trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE - 2) * accele_l / 100;
  }
  // �E�O���[�^
  if ( accele_r >= 0 ) {
    p2_7 = 0;
  } else {
    p2_7 = 1;
    accele_r = -accele_r;
  }
  if ( accele_r <= 5 ) {
    trcgrd = trcgrd_buff = trcgra;
  } else {
    trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE - 2) * accele_r / 100;
  }
}
/************************************************************************/
/* �O�ւ̑��x���� 2 �f�B�b�v�X�C�b�`�ɂ͊֌W���Ȃ� motor �֐� */
/* ���� �����[�^:-100�`100 , �E���[�^:-100�`100 */
/* 0 �Œ�~�A100 �Ő��] 100%�A-100 �ŋt�] 100% */
/* �߂�l �Ȃ� */
/************************************************************************/
void motor_f( int accele_l2, int accele_r2 )
{
  int accele_l, accele_r;
  if ( M_FreeMoter == 1 ) {
    acceleFree = 1;
    accele_l = acceleFree;
    accele_r = acceleFree;
  }
  else {
    accele_l = accele_l2;
    accele_r = accele_r2;
  }
  accele_l = -accele_l;
  /* ���O���[�^ */
  if ( accele_l >= 0 ) {
    p2_0 = 0;
  } else {
    p2_0 = 1;
    accele_l = -accele_l;
  }
  if ( accele_l <= 5 ) {
    trcgrb = trcgrb_buff = trcgra;
  } else {
    trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE - 2) * accele_l / 100;
  }
  /* �E�O���[�^ */
  if ( accele_r >= 0 ) {
    p2_7 = 0;
  } else {
    p2_7 = 1;
    accele_r = -accele_r;
  }
  if ( accele_r <= 5 ) {
    trcgrd = trcgrd_buff = trcgra;
  } else {
    trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE - 2) * accele_r / 100;
  }
}
/************************************************************************/
/* �ヂ�[�^��~����i�u���[�L�A�t���[�j */
/* ���� �����[�^:FREE or BRAKE , �E���[�^:FREE or BRAKE */
/* �߂�l �Ȃ� */
/************************************************************************/
void motor_mode_r( int mode_l, int mode_r )
{
  if ( mode_l ) {
    p9_0 = 1;
  } else {
    p9_0 = 0;
  }
  if ( mode_r ) {
    p9_1 = 1;
  } else {
    p9_1 = 0;
  }
}
/************************************************************************/
/* �O���[�^��~����i�u���[�L�A�t���[�j */
/* ���� �����[�^:FREE or BRAKE , �E���[�^:FREE or BRAKE */
/* �߂�l �Ȃ� */
/************************************************************************/
void motor_mode_f( int mode_l, int mode_r )
{
  if ( mode_l ) {
    p9_2 = 1;
  } else {
    p9_2 = 0;
  }
  if ( mode_r ) {
    p9_3 = 1;
  } else {
    p9_3 = 0;
  }
}
/************************************************************************/
/* �T�[�{���[�^���� */
/* ���� �T�[�{���[�^ PWM�F-100�`100 */
/* 0 �Œ�~�A100 �Ő��] 100%�A-100 �ŋt�] 100% */
/* �߂�l �Ȃ� */
/************************************************************************/
void servoPwmOut( int pwm )
{
  if ( pwm >= 0 ) {
    p2_6 = 0;
    trdgrd1 = (long)( TRD_MOTOR_CYCLE - 2 ) * pwm / 100;
  } else {
    p2_6 = 1;
    trdgrd1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -pwm ) / 100;
  }
}
/************************************************************************/
/* �N���X���C�����o���� */
/* ���� �Ȃ� */
/* �߂�l 0:�N���X���C���Ȃ� 1:���� */
/************************************************************************/
int check_crossline( void )
{
  int ret;
  ret = 0;
  // if((sensor_inp()==0x0f || sensor_inp()==0x0e || sensor_inp()==0x0d || sensor_inp()==0x0b ||sensor_inp() == 0x07)) {
  //sensor_inp()!=0x03 || sensor_inp()!=0x0c || sensor_inp()!=0x01 || sensor_inp()!=0x08)
  if ((sensor_inp() == 0x09 || sensor_inp() == 0x05 || sensor_inp() == 0x0a || sensor_inp() == 0x0f
       || sensor_inp() == 0x0e || sensor_inp() == 0x0d || sensor_inp() == 0x0b || sensor_inp() == 0x07)
      && center_inp() == 0x01) {
    ret = 1;
  }
  return ret;
}
/************************************************************************/
/* �E�Ԑ��ύX���C�����o���� */
/* ���� �Ȃ� */
/* �߂�l 0:�Ԑ��ύX���C���Ȃ� 1:���� */
/************************************************************************/
int check_zlineR( void )
{
  int ret;
  ret = 0;
  // if((sensor_inp()==0x03) || (sensor_inp()==0x01)) {
  //sensor_inp()!=0x09 || sensor_inp()!=0x05 || sensor_inp()!=0x0a || sensor_inp()!=0x0f)
  if ((sensor_inp() == 0x01 || sensor_inp() == 0x03)
      && center_inp() == 0x01) {
    ret = 1;
  }
  return ret;
}
/************************************************************************/
/* ���Ԑ��ύX���C�����o���� */
/* ���� �Ȃ� */
/* �߂�l 0:�Ԑ��ύX���C���Ȃ� 1:���� */
/************************************************************************/
int check_zlineL( void )
{
  int ret;
  ret = 0;
  // if((sensor_inp()==0x0c) || (sensor_inp()==0x08)) {
  ///sensor_inp()!=0x09 || sensor_inp()!=0x05 || sensor_inp()!=0x0a || sensor_inp()!=0x0f)
  if ((sensor_inp() == 0x08 || sensor_inp() == 0x0c)
      && center_inp() == 0x01) {
    ret = 1;
  }
  return ret;
}
/************************************************************************/
/* ���C���I�����o���� */
/* ���� �Ȃ� */
/* �߂�l 0:�Ԑ��ύX���C���Ȃ� 1:���� */
/************************************************************************/
int check_Noline( void )
{
  int ret;
  ret = 0;
  if ( (sensor_inp() == 0x00) && center_inp() == 0x00) {
    ret = 1;
  }
  return ret;
}
/************************************************************************/
/* �T�[�{�p�x�擾 */
/* ���� �Ȃ� */
/* �߂�l ����ւ���̒l */
/************************************************************************/
int getServoAngle( void )
{
  return ( (-ad2) - iAngle0 );
}
/************************************************************************/
/* �A�i���O�Z���T�l�擾 */
/* ���� �Ȃ� */
/* �߂�l �Z���T�l */
/************************************************************************/
int getAnalogSensor( void )
{
  int ret;
  //ret = ad1 - ad0; /* �A�i���O�Z���T���擾 */
  if ( !crank_mode ) {
    // �N�����N���[�h�łȂ���Ε␳����
    switch ( iSensorPattern ) {
      case 0:
        if ( sensor_inp() == 0x04 && center_inp() == 0x00) {
          ret = -650; //-650
          break;
        }
        else if ( sensor_inp() == 0x02 && center_inp() == 0x00) {
          ret = 650; //650
          break;
        }
        else if ( sensor_inp() == 0x08 && center_inp() == 0x00) {
          ret = -700; //-700
          iSensorPattern = 1;
          break;
        }
        else if ( sensor_inp() == 0x01 && center_inp() == 0x00) {
          ret = 700; //700
          iSensorPattern = 2;
          break;
        }
        else {
			// ��-�E
			ret = (ad1 - 21) - (ad0 + 21);
			//ret = (ad1 - 11) - (ad0 + 11);
        }
        break;
      case 1:
        // �Z���T�E���
        ret = -700;
        if (center_inp() == 0x01 || sensor_inp() == 0x04) {
          iSensorPattern = 0;
        }
        break;
      case 2:
        // �Z���T�����
        ret = 700;
        if (center_inp() == 0x01 || sensor_inp() == 0x02) {
          iSensorPattern = 0;
        }
        break;
    }
  }
  else {
    //ret = (ad1 - 11) - (ad0 + 11);
    ret = (ad1 - 21) - (ad0 + 21);
  }
  return ret;
}
/************************************************************************/
/* �T�[�{���[�^���� */
/* ���� �Ȃ� */
/* �߂�l �O���[�o���ϐ� iServoPwm �ɑ�� */
/************************************************************************/
void servoControl( void )
{
  int i, iRet, iP, iD;
  int kp, kd;
  i = getAnalogSensor(); /* �Z���T�l�擾 */
  kp = dipsw1_pattern[dipsw_get()] ; /* �����ł����� P,D �l�͌Œ�l�� */
  kd = dipsw1_pattern[dipsw_get()] * 10; /* ���Ă������� */
  /* �T�[�{���[�^�p PWM �l�v�Z */
  iP = gain * i; /* ��� */
  //iP = kp * i; /* ��� */
  //10 �� gain �ɂ���
  //iD = 100 * (iSensorBefore - i ); /* ����(�ڈ��� P �� 5�`10 �{) */
  //iD = kd * (iSensorBefore - i ); /* ����(�ڈ��� P �� 5�`10 �{) */
  iD = (gain * 7) * (iSensorBefore - i); /* ����(�ڈ��� P �� 5�`10 �{) */
  iRet = iP - iD;
  iRet = iRet / (130 / 2); //iRet /= 64; //64
  /* PWM �̏���̐ݒ� */
  if ( iRet > 120 ) iRet = 120; /* �}�C�R���J�[�����肵���� */
  if ( iRet < -120 ) iRet = -120; /* ����� 90 ���炢�ɂ��Ă������� */

  iServoPwm = iRet;
  iSensorBefore = i; /* ����͂��̒l�� 1ms �O�̒l�ƂȂ�*/
}
/************************************************************************/
/* ���W���[���� handle */
/* �����T�v �T�[�{���[�^���� �p�x�w��p */
/* ���� �Ȃ� */
/* �߂�l �O���[�o���ϐ� servoPwmOut �ɑ�� */
/************************************************************************/
void handle( int iSetAngle ) {
  int i, j, iRet, iP, iD ;
  i = - iSetAngle; /* �ݒ肵�����p�x */
  //j = ( p7_2 >>2 );
  j = getServoAngle(); /* ���݂̊p�x */
  /* �T�[�{���[�^�p PWM �l�v�Z */
  iP = 20 * (j - i); /* ��� */
  iD = 100 * (iAngleBefore2 - j); /* ���� */
  iRet = iP - iD;
  iRet = iRet / 2; //iRet /= 4; //2
  if ( iRet > 120 ) iRet = 120; /* �}�C�R���J�[�����肵���� */
  if ( iRet < -120 ) iRet = -120; /* ����� 90 ���炢�ɂ��Ă������� */
  servoPwmOut(iRet);
  
  iAngleBefore2 = j;
}
/************************************************************************/
/* �O�ւ� PWM ����A���ւ� PWM ������o�� �n���h���p�x�͌��݂̒l���g�p 	*/
/* ���� �O�� PWM 														*/
/* �߂�l ���� PWM 														*/
/************************************************************************/
int diff( int pwm )
{
  int i, ret;
  i = getServoAngle() / SERVO_STEP; /* 1 �x������̑����Ŋ��� */
  if ( i < 0 ) i = -i;
  if ( i > 40 ) i = 40; //45
  ret = revolution_difference[i] * pwm / 100;
  return ret;
}

/************************************************************************/
/* �⓹�`�F�b�N                   										*/
/* ���� �Ȃ�                              								*/
/* �߂�l �Ȃ�                               							*/
/* ���� �⓹�Ɣ��f�����saka_flag = 1 �⓹�łȂ��Ȃ� 0           		*/
/************************************************************************/
void sakaSyori( void )
{
  volatile static int saka_pattern = 0;
  volatile int saka;

  saka = ad5 - saka0_ad;
  // ��̎��K����led_out�Ńp�^�[����\���A���K���I�������R�����g�ɂ���
#if 0
	led_out( ((saka_pattern/10)<<4) | (saka_pattern%10) );
#endif

  switch ( saka_pattern ) {
    case 0:
      // ����A�����̃`�F�b�N
      if ( saka <= -10 ) { // ����A/D�l���������Ȃ�
        cnt_saka = 0;
        saka_pattern = 1; // ���⏈��
      }
      break;

    case 1:
      // ���� �������Ԃ������āA�ēx�`�F�b�N
      if ( cnt_saka >= 10 ) {
        if ( saka <= -10 ) { // ��������
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
      if ( cnt_saka >= 100 ) {
        cnt_saka = 0;
        saka_pattern = 3;
      }
      break;

    case 3:
      // ���� ���̒��_���`�F�b�N
      if ( saka >= 10 ) {
        cnt_saka = 0;
        saka_pattern = 4;
      }
      break;

    case 4:
      // ���� �������Ԃ������āA�ēx�`�F�b�N
      if ( cnt_saka >= 10 ) {
        if ( saka >= 10 ) { // ��������
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
      if ( cnt_saka >= 200 ) {
        cnt_saka = 0;
        saka_pattern = 0;
      }
      break;
  }
}


/* LCD�ƃX�C�b�`�ő��x�p�����[�^�����߂�֐� 	*/
/* 0:�ݒ�O/�ݒ蒆 1:�ݒ�I��					*/
int paramSettings( void ){
	
	volatile static int param_pattern = 0; // �p�����[�^�ݒ�̃p�^�[��
	volatile static int isGo = 0; // �X�^���o�C��Ԃֈڍs���邩(1:�X�^���o�C��Ԃ� 0:�p�����[�^�ݒ���)
	unsigned int i;
	signed int saka_val;
	
	if( param_pattern < 5 ){	
		i = ((cnt1 / 200) % 2 + 1) | (startbar_get()<<7&0x80); // �X�^�[�g�o�[�����m�F
		led_out( i ); /* LED �_�ŏ��� */
	}
	
	if( pushsw_get() ){
		/* STRAT�X�C�b�`�����őO��p�����[�^�̂܂ܑ��s */
		param_pattern = 6;	
		setBeepPatternS( 0xa800 );
	}
	
	switch( param_pattern ){
		case 0:	
		/* �����̑��s���x:(11��1.0m/s) */
			//TESTSPEED = getSws(); 
			if(getSwNow()&0x0f){    
				data_buff[DF_TESTSPEED] = getSws();
			}
			
			lcdPosition(0,0);
			lcdPrintf("TESTSPEED       ");
			lcdPosition(0,1); 
			lcdPrintf("TESTSPEED= %02d   ",data_buff[DF_TESTSPEED]);
			if( getSwFlag(SW_4)){
  				setBeepPatternS( 0x8000 );
				param_pattern++;	
			}
			break;
		
		case 1:
		/* �J�[�u�̑��s���x(11��1.0m/s) */
			//corner_speed = getSws();
			if(getSwNow()&0x03){    
				data_buff[DF_CURVE_SPEED] = getSws();
			}
			
			lcdPosition(0,0);
			lcdPrintf("corner_speed    ");
			lcdPosition(0,1); 
			lcdPrintf("corner_speed= %02d",data_buff[DF_CURVE_SPEED]);
			if( getSwFlag(SW_4)){
  				setBeepPatternS( 0x8000 );
				param_pattern++;	
			}
			break;
		
		case 2:
		/* �����[���`�F���W�̑��s���x:(11��1.0m/s) */
			//LANE_SPEED_L = getSws();   
			if(getSwNow()&0x03){    
				data_buff[DF_LANE_L] = getSws();
			}
			lcdPosition(0,0);
			lcdPrintf("LANE_SPEED_LEFT ");
			lcdPosition(0,1);
			lcdPrintf("L_LANE= %02d      ",data_buff[DF_LANE_L]);
			if( getSwFlag(SW_4)){
  				setBeepPatternS( 0x8000 );
				param_pattern++;	
			}
			break;
		
		case 3:
		/* �E���[���`�F���W�̑��s���x:(11��1.0m/s) */			
			//LANE_SPEED_R = getSws(); 
			if(getSwNow()&0x03){    
				data_buff[DF_LANE_R] = getSws();
			}
			lcdPosition(0,0);
			lcdPrintf("LANE_SPEED_RIGHT");
			lcdPosition(0,1);
			lcdPrintf("R_LANE= %02d     ",data_buff[DF_LANE_R]);
			if( getSwFlag(SW_4)){
  				setBeepPatternS( 0x8000 );
				param_pattern++;	
			}
			break;
		
		case 4:
		/* �N�����N�̑��s���x:(11��1.0m/s) */
			//CRANK_SPEED = getSws();
			if(getSwNow()&0x03){    
				data_buff[DF_CRANK_SPEED] = getSws();
			}
			lcdPosition(0,0);
			lcdPrintf("CRANK_SPEED     ");
			lcdPosition(0,1);
			lcdPrintf("CRANK_SPEED= %02d ",data_buff[DF_CRANK_SPEED]);
			if( getSwFlag(SW_4)){	
  				setBeepPatternS( 0x8000 );
				param_pattern++;	
			}
			break;
		
		case 5:
		/* ���s����:(�P�ʂ�m�Ŏw��) */
			//ENC_END = getSws();
			if(getSwNow()&0x03){    
				data_buff[DF_ENC_END] = getSws();
			}
			lcdPosition(0,0);
			lcdPrintf("RUN/DISTANCE    ");
			lcdPosition(0,1);
			lcdPrintf("DISTANCE= %03dm ",data_buff[DF_ENC_END]);
			
			if( getSwFlag(SW_4)){
				//data_buff[DF_ENC_END] *= 1091L; // ���������signed char�^�𒴂���̂�NG	
  				setBeepPatternS( 0xcc00 );
				param_pattern++;
				//param_pattern = 5;	
				led_out(0x00);
			}
			break;
			
		case 6:
		/* �X�^���o�C��Ԃ� */
			lcdPosition(0,0);
			lcdPrintf("STAND-BY... ");
			lcdPosition(0,1);
			// param_pattern��1�`4�Őݒ肵���l��\��
			lcdPrintf("%02d,%02d,%02d,%02d,%02d    ",
				data_buff[DF_TESTSPEED], data_buff[DF_CURVE_SPEED], data_buff[DF_LANE_L], data_buff[DF_LANE_R], data_buff[DF_CRANK_SPEED]);
			
			isGo = 1;
			break;
		
	}
	return isGo;
	
}

/* �X�C�b�`�̓��͂��܂Ƃ߂��֐� */
/* SW0�Ԃ�+1 */
/* SW1��-1 */
/* SW2����+10 */
/* SW3�΂�-10 */
/* (SW4���͎�Ɍ��肷��Ƃ��Ɏg��) */
int getSws( void ){
	beforeswnum = swnum;
	/* �ϐ�swnum�͕K���O���[�o���ϐ� */
	if(getSwFlag(SW_0)){
		swnum++;
		if(swnum>=10000) swnum=10000;
	}	
	if(getSwFlag(SW_1)){
		swnum--;
		if(swnum<=-100) swnum=-100;
	}
	if(getSwFlag(SW_2)){
		swnum+=10;
		if(swnum>=10000) swnum=10000;
	}
	if(getSwFlag(SW_3)){
		swnum-=10;
		if(swnum<=-100) swnum=-100;
	}
	return swnum;
		
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/************************************************************************/
/* �R�[�i�[�̑��x���� �@												*/
/* ���� max_power ��ւ̊O�ւ̂o�v�l�̍ő�l 0~100%					*/
/* �߂�l �Ȃ� �@�@�@�@ 												*/
/************************************************************************/
void corner_run(int max_power){
	/*	�R�[�i�[���̐��� */
  
	accel_out_b = max_power;//�O�����	
	accel_in = (long)f_in[kakudo]*max_power/100;		//�����O��
	if (abs(angle) > 60){									
		accel_in_b = 0;	//�������(0%)	
	}
	else{
		accel_in_b = (long)r_in[kakudo]*max_power/100;	//�������
	}
	accel_out = (long)f_out[kakudo]*max_power/100;		//�O���O��
	if(angle < 0){	/* �X�e�A�����O�����ɐ؂�Ƃ� */
		motor_mode_f(BRAKE,FREE);	
		motor_mode_r(BRAKE,FREE);						
		motor_f(accel_in,accel_out); 			//�O �i��,�E�j
		motor_r(accel_in_b,accel_out_b);		//��i��,�E�j
	}
	else if(angle > 0){/* �X�e�A�����O���E�ɐ؂�Ƃ� */
		motor_mode_f(FREE,BRAKE);	
		motor_mode_r(FREE,BRAKE);
		motor_f(accel_out,accel_in);	 		//�O �i��,�E�j
		motor_r(accel_out_b,accel_in_b);		//��i��,�E�j
	}
}

/************************************************************************/
/* end of file */
/************************************************************************/