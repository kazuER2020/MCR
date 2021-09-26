/****************************************************************************/
/* �Ώۃ}�C�R�� RY3048Fone(TypeH) / H8-3048FONE                             */
/* ̧�ٓ��e     �}�C�R���J�[�L�b�gVer.X �ڰ���{��۸���(H8��)���ǔ�		    */
/* �o�[�W����   Ver.1.00                                                    */
/* Date         2020.06.08                                                  */
/****************************************************************************/

/*
���̃v���O�����́A���L��ɑΉ����Ă��܂��B
�ERY3048Fone�{�[�h
�E���[�^�h���C�u���Ver.5
�E�Z���T���Ver.5
*/

/*======================================*/
/* �C���N���[�h                         */
/*======================================*/
#include    <machine.h>
#include    "h8_3048.h"
#include    "beep.h"

/*======================================*/
/* �V���{����`                         */
/*======================================*/

/* �萔�ݒ� */
#define         TIMER_CYCLE     3071    /* �^�C�}�̃T�C�N�� 1ms     */
                                        /* ��/8�Ŏg�p����ꍇ�A     */
                                        /* ��/8 = 325.5[ns]         */
                                        /* ��TIMER_CYCLE =          */
                                        /*      1[ms] / 325.5[ns]   */
                                        /*               = 3072     */
#define         PWM_CYCLE       42152   /* PWM�̃T�C�N�� 16ms       */
                                        /* ��PWM_CYCLE =            */
                                        /*      16[ms] / 325.5[ns]  */
                                        /*               = 49152    */
#define         SERVO_CENTER    4150    /* �T�[�{�̃Z���^�l         */
#define         HANDLE_STEP     26      /* 1�K���̒l                */

#define NIGHT_MODE 1 // ��ԃ��[�h 0:�ʏ� 1:BEEP�I�t
// rgb
#define OFF 0,0,0
#define R 1,0,0
#define G 0,1,0
#define B 0,0,1
#define WHITE 1,1,1

/* �}�X�N�l�ݒ� �~�F�}�X�N����(����)�@���F�}�X�N����(�L��) */
#define MASK2_2         0x66            /* �~�����~�~�����~             */
#define MASK2_0         0x60            /* �~�����~�~�~�~�~             */
#define MASK0_2         0x06            /* �~�~�~�~�~�����~             */
#define MASK3_3         0xe7            /* �������~�~������             */
#define MASK0_3         0x07            /* �~�~�~�~�~������             */
#define MASK3_0         0xe0            /* �������~�~�~�~�~             */
#define MASK4_0         0xf0            /* ���������~�~�~�~             */
#define MASK0_4         0x0f            /* �~�~�~�~��������             */
#define MASK4_4         0xff            /* ����������������             */

/* ��~���̓��� */
#define FREE 			1				/* �t���[ 						*/
#define BRAKE 			0 				/* �u���[�L 					*/

#define HANDLE_MAX		49				/* �N�����N�ł̍ő�؂�p		(��:�ő�49, �E:50)*/
#define R_HF			15				/* �Ԑ��ύX���̊p�x				*/
#define MAGE_SP		90				/* �Ԑ��ύX���̃X�s�[�h			*/
#define YORI			3				/* �n�[�t���C�����o��̍�/�E�񂹑��s���̏����p�x */
#define AR_F			5				/* ���Ԑ��ύX���̃n���h���̖߂��p�x */

#define OUT_THERSHOLD   1500             /* �R�[�X�A�E�g����ɂ����鎞��[ms] */
#define RUNNING_TIME	dipsw_get()*10	/* ���s����[s]					*/


/*======================================*/
/* �v���g�^�C�v�錾                     */
/*======================================*/
void init( void );
void timer( unsigned long timer_set );
void ha( int angle );
void ha_free( void );
void led_out( unsigned char led );
void sp( int accele_l, int accele_r );
void sp_mode( int mode_l, int mode_r );
void newcrank( void );                  /* �N�����N���̌��������錾	 	*/
void shasenhenkou( void );              /* �Ԑ��ύX���̌��������錾  	*/
unsigned char sensor_inp( unsigned char mask );
unsigned char dipsw_get( void );
unsigned char pushsw_get( void );
unsigned char startbar_get( void );
int check_crossline( void );
int check_shasenhenkoum( void );
int check_migikakunin( void );
int check_shasenhenkouh( void );
int check_hidarikakunin( void );
int migikakunin( void );
int hidarikakunin( void );
int diff( int pwm );
void rgb( unsigned char r, unsigned char g, unsigned char b );

/*======================================*/
/* �O���[�o���ϐ��̐錾                 */
/*======================================*/
unsigned long   cnt0 		= 0;                /* �^�C�}�[�p                �X�^�[�g�O�̏����ݒ�   */
unsigned long   cnt1 		= 0;                /* main���Ŏg�p              �X�^�[�g�O�̏����ݒ�   */
unsigned long	cnt_run		= 0;				/* ���s���Ԍv���p			 �X�^�[�g�O�̏����ݒ�   */
unsigned long   check_sensor_cnt = 0;			/* �R�\�X�A�E�g���ً}��~�p	 �X�^�[�g�O�̏����ݒ�	*/
int             pattern 	= 0;                /* �p�^�[���ԍ�              �X�^�[�g�O�̏����ݒ�   */
int				kyori_flug 	= 0;             	/* �e���C���̒ʉߌ��o�錾  	 �X�^�[�g�O�̏����ݒ�   */
int				kyoritime  	= 0;             	/* �e���C���̒ʉߎ��Ԑ錾    �X�^�[�g�O�̏����ݒ�   */
int 			angle_buff	= 0;

const int revolution_difference[] = {       /* �p�x������ցA�O�։�]���v�Z */
    100, 98, 97, 95, 94, 
    93, 91, 90, 88, 87, 
    85, 84, 83, 81, 80, 
    79, 77, 76, 75, 73, 
    72, 71, 70, 68, 67, 
    66, 64, 63, 62, 60, 
    59, 58, 56, 55, 54, 
    52, 51, 50, 48, 47, 
    46, 44, 43, 41, 40, 
    38 
};
						
/************************************************************************/
/* ���C���v���O����                                                     */
/************************************************************************/
void main( void )
{
    
    unsigned char b,c;                    /* �I�ǉ��E�ύX�I               */	
	
	
    /* �}�C�R���@�\�̏����� */
    init(); 		                    /* ������                       */
	set_ccr( 0x00 );                    /* �S�̂̊��荞�݋���           */
	initBeep();
	
if( dipsw_get() & 0x08 == 1 ){
	setBeepPattern(0xa000);
}
	/* �}�C�R���J�[�̏�ԏ����� */
    ha(  0 );                           /* �X�^�[�g�O�̃n���h���p�x     */
	sp_mode( FREE, FREE );				/* ���[�^��Ԃ̓t���[			*/
    sp(    0,    0 );                   /* �X�^�[�g�O�̃��[�^�o�͒l     */
	rgb(OFF);
	
    while( 1 ) {
	    /* �E�֎��̒�~�����i�f�W�^���Z���T�j */
        b = sensor_inp(MASK4_4);
        if( b == 0xff || b == 0x00 || b == 0xfb 
		   || b == 0xf7 || b == 0xfe || b == 0xfd
		   || b == 0xbf || b == 0x7f || b == 0xef
		   || b == 0xdf) {
            if( check_sensor_cnt >= 500 ) {
                pattern = 999;
            }
        } else {
            check_sensor_cnt = 0;
        }

	
	switch( pattern ) {

 case   0:                              /* �X�^�[�g�ʒu�Ɏԑ̂��Z�b�g*/
          if( pushsw_get() ) {          /* ���s���������X�C�b�`ON�@  */
if( dipsw_get() & 0x08){
			  setBeepPattern(0xa000);
}
			  led_out(0x1);  // 3
			  timer(1000);
if( dipsw_get() & 0x08 ){
			  setBeepPattern(0xa000);
}			  led_out(0x2);  // 2
			  timer(1000);
if( dipsw_get() & 0x08 ){
			  setBeepPattern(0xa000);
}		      led_out(0x3);  // 1
			  timer(1000);
              cnt1 = 0;
			  pattern = 1;  // GO!!!
              break;
          }
          if( cnt1 < 100 ) {            /* LED�̒ᑬ�_�ŏ����@�@     */
              led_out( 0x1 ); 
          } else if( cnt1 < 200 ) {
              led_out( 0x2 );
          } else {
              cnt1 = 0;
          }
          break;

 case   1:                              /* �X�^�[�g�̃X�^���o�COK    */
          if( !startbar_get( ) && sensor_inp(MASK4_4)==0x18) {      /* �X�^�[�g�Q�[�g�̊J���m�F  */
              led_out( 0x0 );
              cnt1 = 0;
			  cnt_run = 0;
			  check_sensor_cnt = 0;
              pattern = 20;
              break;
          }
		  else{
          	  if( cnt1 <  35 ) {            /* LED�̍����_�ŏ����@�@     */
              		led_out( 0x1 );
          	  } else if( cnt1 < 75 ) {
              		led_out( 0x2 );
          	  } else {
              		cnt1 = 0;
          	  }
		  }
          break;

 case  20:                 /* �ʏ푖�s����  �����̕ω��ʂŒ������� */
 		  if( cnt1 % 100 < 50 ){
			  rgb(G); 
		  }
		  else{	  
			  rgb(OFF);
		  }
		  
          if( check_crossline() ) {    /* �N���X���C���`�F�b�N      */
              cnt1 = 0;
              pattern = 100;
			  rgb(R);
if( dipsw_get() & 0x08){
			  setBeepPattern(0xcc000);
}
              break;
          }
          if( check_shasenhenkoum() ) {/* �E�Ԑ��ύX���C���`�F�b�N  */
              cnt1 = 0;
              pattern = 200;
              rgb(G);

			  break;
          }
          if( check_shasenhenkouh() ) {/* ���Ԑ��ύX���C���`�F�b�N  */
              cnt1 = 0;
              rgb(B);

			  pattern = 500;
              break;
          }
          switch( sensor_inp( MASK4_4 ) ) {

          case 0x18:ha(   0 );               /*      ��  �i      */
		  			sp_mode(FREE, FREE );
                    sp(  100 ,  100 );
                    break;

          case 0x1c:ha(   3 );               /* �ԑ̂̉E�����␳ */
		  			sp_mode(FREE, FREE );
                    sp(  100 ,  100 );
                    break;

          case 0x0c:ha(   8 );               /* �����x�̉E�Ȃ��� */
		  		  	sp_mode(FREE, FREE );
                    sp(   98 ,   94 );       /* �����ւ̏o�͒��� */
                    break;

          case 0x0e:ha(  13 );               /* �����x�̉E�Ȃ��� */
		  			sp_mode(FREE, FREE );
                    sp(   94 ,   86 );       /* �����ւ̏o�͒��� */
                    break;

          case 0x06:ha(  17 );               /* �����x�̉E�Ȃ��� */
		  			sp_mode(FREE, FREE );
                    sp(   90 ,   78 );       /* �����ւ̏o�͒��� */
                    break;

          case 0x07:ha(  22 );               /* �����x�̉E�Ȃ��� */
		  			sp_mode(FREE, FREE );
                    sp(   88 ,   72 );       /* �����ւ̏o�͒��� */
                    break;
					
          case 0x03:ha(  26 );               /* ����x�̉E�Ȃ��� */
                    sp_mode(BRAKE, FREE );
                    sp(   90 ,   52 );       /* �����ւ̏o�͒��� */
                    cnt1 = 0;
                    pattern = 30;    /* �E�֑�Ȃ��肵�����̑��s���� */
                    break;
          
          case 0x38:ha(  -3 );               /* �ԑ̂̍������␳ */
                    sp_mode(FREE, FREE );
                    sp(  100 ,  100 );
                    break;
				
          case 0x30:ha(  -8 );               /* �����x�̍��Ȃ��� */
                    sp_mode(FREE, FREE );
                    sp(   94 ,   98 );       /* �����ւ̏o�͒��� */
                    break;
				
          case 0x70:ha( -13 );               /* �����x�̍��Ȃ��� */
                    sp_mode(FREE, FREE );
                    sp(   86 ,   94 );       /* �����ւ̏o�͒��� */
                    break;
				
          case 0x60:ha( -17 );               /* �����x�̍��Ȃ��� */
                    sp_mode(FREE, FREE );
                    sp(   78 ,   90 );       /* �����ւ̏o�͒��� */
                    break;

          case 0xe0:ha( -25 );               /* �����x�̍��Ȃ��� */
                    sp_mode(FREE, FREE );
                    sp(   72 ,   88 );       /* �����ւ̏o�͒��� */
                    break;

          case 0xc0:ha( -26 );               /* ����x�̍��Ȃ��� */
                    sp_mode( FREE, BRAKE );
                    sp(   52 ,   90 );       /* �����ւ̏o�͒��� */
                    cnt1 = 0;
                    pattern = 40;    /* ���֑�Ȃ��肵�����̑��s���� */
                    break;
            default:
                    break;
          }
          break;

 case  30:              /* �E�֑�Ȃ��肵�����̃R�[�X�A�E�g�������  */
 		  if( cnt1 % 200 < 50 ){
			  rgb(R); 
		  }
		  else{	  
			  rgb(OFF);
		  }
          switch( sensor_inp( MASK4_4 ) ) {
          
		  case 0x06:
          case 0x07:cnt1 = 0;
					pattern = 20;  /* �E�Ȃ���̒ʏ푖�s�ւ̕��A���� */
                    break;
          
          case 0x03:ha(  28 );
		  			sp_mode(BRAKE, FREE );
                    sp(   100 ,   60 );       /* �����ւ̏o�͒��� */
                    break;
																																												          
          case 0x83:ha(  30 );
                    sp_mode(BRAKE, FREE );
                    sp(   80 ,   40 );       /* �����ւ̏o�͒��� */
                    break;
          
          case 0x81:ha(  33 );         /* �}�u���[�L�̃^�C�~���O */
                    sp_mode(BRAKE, FREE );
                    sp(   40 ,   20 );       /* �����ւ̏o�͒��� */
                    break;
          
          case 0xc1:ha(  36 );
                    sp_mode(BRAKE, FREE );
                    sp(   20 ,   10 );       /* �����ւ̏o�͒��� */
                    break;
          
          case 0xc0:ha(  39 );
                    sp_mode(BRAKE, FREE );
                    sp(   10 ,    5 );       /* �����ւ̏o�͒��� */
                    break;
          
          case 0xe0:ha(  42 );
                    sp_mode(BRAKE, FREE );
                    sp(    5 ,    2 );       /* �����ւ̏o�͒��� */
                    break;
					
          case 0x60:ha(  45 );
                    sp_mode(BRAKE, FREE );
                    sp(    2 ,    1 );       /* �����ւ̏o�͒��� */
                    break;
		  
          case 0x70:
		  case 0x30:
		  case 0x38:
		  case 0x18:
		  case 0x3c:pattern = 31;
                    break;
		    default:
                    break;
          }
          break;
 
 case 31:                          /* �R�[�X�A�E�g�ɂ��ً}��~���� */
          switch( sensor_inp( MASK4_4 ) ) {
          
		          case 0xe0:
		          case 0x60:pattern = 30;
                            break;
                  
				  case 0x00:
		          case 0xff:
		          case 0xf7:          
                  case 0xfd:
		          case 0xfb:
		          case 0xef:
		          case 0xbf:
		          case 0xdf:cnt1 = 0;
		                    pattern = 999;
                            break;
		            default:
                            break;
          }
          break;

 case  40:              /* ���֑�Ȃ��肵�����̃R�[�X�A�E�g�������  */
 		  if( cnt1 % 200 < 50 ){
			  rgb(1,0,1); 
		  }
		  else{	  
			  rgb(OFF);
		  }
          switch( sensor_inp( MASK4_4 ) ) {
          
		  case 0x60:
          case 0xe0:cnt1 = 0;
                    pattern = 20;  /* ���Ȃ���̒ʏ푖�s�ւ̕��A���� */
                    break;
          
          case 0xc0:ha( -28 );
                    sp_mode(FREE, BRAKE );
                    sp(   65 ,   100 );       /* �����ւ̏o�͒��� */
                    break;
																												
          case 0xc1:ha( -30 );
                    sp_mode(FREE, BRAKE );
                    sp(   40 ,   80 );       /* �����ւ̏o�͒��� */
                    break;
					
          case 0x81:ha( -33 );         /* �}�u���[�L�̃^�C�~���O */
                    sp_mode(FREE, BRAKE );
                    sp(   20 ,   40 );       /* �����ւ̏o�͒��� */
                    break;
					
          case 0x83:ha( -36 );
                    sp_mode(FREE, BRAKE );
                    sp(   10 ,   20 );       /* �����ւ̏o�͒��� */
                    break;
					          
          case 0x03:ha( -39 );
                    sp_mode(FREE, BRAKE );
                    sp(    5 ,   10 );       /* �����ւ̏o�͒��� */
                    break;
					          
          case 0x07:ha( -42 );
                    sp_mode(FREE, BRAKE );
                    sp(    2 ,    5 );       /* �����ւ̏o�͒��� */
                    break;
          
          case 0x06:ha( -45 );
                    sp_mode(FREE, BRAKE );
                    sp(    1 ,    2 );       /* �����ւ̏o�͒��� */
                    break;
		  
          case 0x0e:
		  case 0x0c:		  
		  case 0x1c:
		  case 0x18:
		  case 0x3c:pattern = 41;
                    break;
		    default:
                    break;
          }
          break;
		  
 case 41:                          /* �R�[�X�A�E�g�ɂ��ً}��~���� */
          switch( sensor_inp( MASK4_4 ) ) {
          
		          case 0x07:
		          case 0x06:pattern = 40;
                            break;
                  
				  case 0x00:
		          case 0xff:
		          case 0xf7:          
                  case 0xfd:
		          case 0xfb:
		          case 0xef:
		          case 0xbf:
		          case 0xdf:cnt1 = 0;
		                    pattern = 999;
                            break;
		 	        default:
                            break;
          }
          break;

   
 case 100:                              /* �N���X���C�����o���̏���  */
          led_out( 0x3 );
         sp_mode(BRAKE, BRAKE); 
		  ha(  0  );
          kyori_flug = 0;				
		  cnt1 = 0;
          pattern = 101;
          break;

 case 101:                 /* �N���X���C�����o��̒ʉߏ󋵂̊m�F���� */
          if( kyori_flug == 0 && check_crossline( ) == 1 ){
              kyori_flug = 1;
          } else if( kyori_flug == 1 && check_crossline( ) == 0 ){
              kyori_flug = 2;
		    /* �N���X���C���Ԃ̒ʉߎ��Ԍv��[ms] ������O���甒���I���܂� */
              kyoritime = cnt1;
		  }
          if( kyori_flug == 2 ){
              kyori_flug = 0;
              cnt1 = 0;
              pattern = 102;
              break;
          }
          switch( sensor_inp( MASK4_4 ) ) {

                  case 0x18:ha(   0 );   /*   �Z���^ �� ��  �i   */
                            break;                
                  case 0x1c:ha(   2 );   /* ������� �� ���E�Ȃ� */
                            break;                
                  case 0x0c:ha(   4 );   /* ������� �� ���E�Ȃ� */
                            break;                     
                  case 0x0e:ha(   6 );   /* ������� �� ���E�Ȃ� */
                            break;                     
                  case 0x38:ha(  -2 );   /* ���E��� �� �����Ȃ� */
                            break;                     
                  case 0x30:ha(  -4 );   /* ���E��� �� �����Ȃ� */
                            break;                     
                  case 0x70:ha(  -6 );   /* ���E��� �� �����Ȃ� */
                            break;                     
                    default:
                            break;
          }
          break;

 case 102:       /* �N���X���C���ʉߌ�Ƀn���h���������ɋȂ��Ȃ����� */
          kyori_flug = 0;
          newcrank( );
          if( cnt1 > 100 ) {
              pattern = 103;
              break;
          }
          switch( sensor_inp( MASK4_4 ) ) {

                  case 0x18:ha(   0 );   /*   �Z���^ �� ��  �i   */
                            break;                
                  case 0x1c:ha(   2 );   /* ������� �� ���E�Ȃ� */
                            break;                
                  case 0x0c:ha(   4 );   /* ������� �� ���E�Ȃ� */
                            break;                     
                  case 0x0e:ha(   6 );   /* ������� �� ���E�Ȃ� */
                            break;                     
                  case 0x38:ha(  -2 );   /* ���E��� �� �����Ȃ� */
                            break;                     
                  case 0x30:ha(  -4 );   /* ���E��� �� �����Ȃ� */
                            break;                     
                  case 0x70:ha(  -6 );   /* ���E��� �� �����Ȃ� */
                            break;                     
                    default:
                            break;
          }
          break;

 case 103:                           /* �N���X���C���ʉߌ�̑��s���� */
          newcrank( );
          switch( sensor_inp( MASK4_4 ) ) {

                  case 0x18:ha(   0 );   /*   �Z���^ �� ��  �i   */
                            break;                
                  case 0x1c:ha(   2 );   /* ������� �� ���E�Ȃ� */
                            break;                
                  case 0x0c:ha(   4 );   /* ������� �� ���E�Ȃ� */
                            break;                     
                  case 0x0e:ha(   6 );   /* ������� �� ���E�Ȃ� */
                            break;                     
                  case 0x38:ha(  -2 );   /* ���E��� �� �����Ȃ� */
                            break;                     
                  case 0x30:ha(  -4 );   /* ���E��� �� �����Ȃ� */
                            break;                     
                  case 0x70:ha(  -6 );   /* ���E��� �� �����Ȃ� */
                            break;                     
                         /* ���N�����N�Ɣ��f �� ���N�����N�N���A������ */
                  case 0xe0:
                  case 0xf0:
                  case 0xf8:
                  case 0xfc:
				  case 0xfe:led_out( 0x1 );
                            ha( -HANDLE_MAX ); /* �����N�����N�̋Ȃ��p�x */
			  				  sp_mode(BRAKE, BRAKE); 
                            sp(   -5 ,   100 );
                            cnt1 = 0;
                            pattern = 130;
                            break;
				       /* �E�N�����N�Ɣ��f �� �E�N�����N�N���A������ */
                  case 0x07:
                  case 0x0f:
                  case 0x1f:
                  case 0x3f:
				  case 0x7f:led_out( 0x2 );	
                            ha(  HANDLE_MAX ); /* ���E�N�����N�̋Ȃ��p�x */
							  sp_mode(BRAKE,BRAKE);
                            sp(   100 ,   -5 );
                            cnt1 = 0;
                            pattern = 140;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 130:                              /* ���N�����N�̒��p���s����  */
          ha( -HANDLE_MAX );                   /* �����N�����N�̋Ȃ��p�x */
          sp(   -5 ,   100 );
          if( cnt1 >  50 ) {     /* ���n���h���̋Ȃ��p�x�̕ێ����ԗ� */
              cnt1 = 0;
              pattern = 131;
              break;
          }
          switch( sensor_inp(MASK4_4) ) {

                  case 0x07:
                  case 0x03:
                  case 0x05:
                  case 0x02:
                  case 0x01:cnt1 = 0;
                            pattern = 135;
                            break;
                    default:
                            break;
          }
	      break;

 case 131:                              /* ���N�����N�̒��p���s����  */
          ha( -HANDLE_MAX );                   /* �����N�����N�̋Ȃ��p�x */
          sp(   20 ,   80 );
          if( cnt1 >  50 ) {     /* ���n���h���̋Ȃ��p�x�̕ێ����ԗ� */
              cnt1 = 0;
              pattern = 132;
              break;
          }
          switch( sensor_inp(MASK4_4) ) {

                  case 0x07:
                  case 0x03:
                  case 0x05:
                  case 0x02:
                  case 0x01:cnt1 = 0;
                            pattern = 135;
                            break;
                    default:
                            break;
          }
	      break;

 case 132:                              /* ���N�����N�̋Ȃ��I������  */
          ha( -HANDLE_MAX );                   /* �����N�����N�̋Ȃ��p�x */
          sp(   40 ,   80 );
          switch( sensor_inp(MASK4_4) ) {

                  case 0x80:
                  case 0xc0:led_out( 0x0 );
                            cnt1 = 0;
                            pattern = 133;
                            break;
                  case 0x07:
                  case 0x03:
                  case 0x05:
                  case 0x02:
                  case 0x01:cnt1 = 0;
                            pattern = 135;
                            break;
                    default:
                            break;
          }
          break;

 case 133:                              /* ���N�����N�̋Ȃ��I������  */
          ha( -HANDLE_MAX + 25);                   /* �����N�����N�̋Ȃ��p�x */
          sp(   60 ,   90 );
          switch( sensor_inp(MASK4_4) ) {

                  case 0x38:
                  case 0x30:
                  case 0x71:
				  case 0x60:led_out( 0x0 );
                            cnt1 = 0;
                            pattern = 20;
                            break;
                  case 0xe0:
                  case 0xc0:led_out( 0x0 );
                            cnt1 = 0;
                            pattern = 134;
                            break;
                  case 0x07:
                  case 0x03:
                  case 0x05:
                  case 0x02:
                  case 0x01:cnt1 = 0;
                            pattern = 135;
                            break;
                    default:
                            break;
          }
          break;

 case 134:                              /* ���N�����N�̋Ȃ��I������  */
          ha( -HANDLE_MAX + 25);                   /* �����N�����N�̋Ȃ��p�x */
          sp(   80 ,  100 );
          switch( sensor_inp(MASK4_4) ) {

                  case 0x38:
                  case 0x30:
                  case 0x71:
				  case 0x60:led_out( 0x0 );
                            cnt1 = 0;
                            pattern = 20;
                            break;
                  case 0x07:
                  case 0x03:
                  case 0x05:
                  case 0x02:
                  case 0x01:cnt1 = 0;
                            pattern = 135;
                            break;
                    default:
                            break;
          }
          break;

 case 135:
          ha( -HANDLE_MAX );                   /* �����N�����N�̋Ȃ��p�x */
          sp(    5 ,   80 );
          switch( sensor_inp(MASK4_4) ) {

                  case 0x83:
                  case 0x81:
                  case 0xc1:cnt1 = 0;
                            pattern = 132;
                            break;
                    default:
                            break;
          }
          if( cnt1 > 2000 ) {
              cnt1 = 0;
			  pattern = 136;
              break;
		  }
		  break;

 case 136:                         /* �R�[�X�A�E�g�ɂ��ً}��~���� */
          ha( -HANDLE_MAX );                   /* �����N�����N�̋Ȃ��p�x */
          sp(   5 ,   80 );
          switch( sensor_inp(MASK4_4) ) {

                  case 0x83:
                  case 0x81:
                  case 0xc1:cnt1 = 0;
                            pattern = 132;
                            break;
                  case 0xff:
		          case 0xf7:          
                  case 0xfd:
		          case 0xfb:
		          case 0xef:
		          case 0xbf:
		          case 0xdf:cnt1 = 0;
		                    pattern = 999;
				            break;
		            default:
                            break;
          }
          break;

 case 140:                              /* �E�N�����N�̒��p���s����  */
          ha(  HANDLE_MAX );                   /* ���E�N�����N�̋Ȃ��p�x */
          sp(   100 ,   -5 );
          if( cnt1 >  50 ) {     /* ���n���h���̋Ȃ��p�x�̕ێ����ԗ� */
              cnt1 = 0;
              pattern = 141;
              break;
          }
          switch( sensor_inp(MASK4_4) ) {

                  case 0xe0:
                  case 0xc0:
                  case 0xa0:
                  case 0x40:
                  case 0x80:cnt1 = 0;
                            pattern = 145;
                            break;
                    default:
                            break;
          }
	      break;

 case 141:                              /* �E�N�����N�̒��p���s����  */
          ha(  HANDLE_MAX );                   /* ���E�N�����N�̋Ȃ��p�x */
          sp(   70 ,   20 );
          if( cnt1 >  50 ) {     /* ���n���h���̋Ȃ��p�x�̕ێ����ԗ� */
              cnt1 = 0;
              pattern = 142;
              break;
          }
          switch( sensor_inp(MASK4_4) ) {

                  case 0xe0:
                  case 0xc0:
                  case 0xa0:
                  case 0x40:
                  case 0x80:cnt1 = 0;
                            pattern = 145;
                            break;
                    default:
                            break;
          }
	      break;

 case 142:                              /* �E�N�����N�̋Ȃ��I������  */
          ha(  HANDLE_MAX );                   /* ���E�N�����N�̋Ȃ��p�x */
          sp(   70 ,   30 );
          switch( sensor_inp(MASK4_4) ) {
			  	    
					case 0x31:
					case 0x30:
					case 0x60:
					case 0x18:
                  case 0x1c:
                  case 0x1e:led_out( 0x0 );
                            cnt1 = 0;
                            pattern = 20;
                            break;

                  case 0x01:
                  case 0x03:led_out( 0x0 );
                            cnt1 = 0;
                            pattern = 143;
                            break;
                  case 0xe0:
                  case 0xc0:
                  case 0xa0:
                  case 0x40:
                  case 0x80:cnt1 = 0;
                            pattern = 145;
                            break;
                    default:
                            break;
          }
          break;

 case 143:                              /* �E�N�����N�̋Ȃ��I������  */
          ha(  20 );                   /* ���E�N�����N�̋Ȃ��p�x HANDLE_MAX - 25*/
          sp(   70 ,   40 );
          switch( sensor_inp(MASK4_4) ) {
					
					case 0x31:
					case 0x30:
					case 0x60:
					case 0x18:
                  case 0x1c:
                  case 0x0c:
                  case 0x1e:led_out( 0x0 );
                            cnt1 = 0;
                            pattern = 20;
                            break;
                  case 0x07:
                  case 0x06:led_out( 0x0 );
                            cnt1 = 0;
                            pattern = 144;
                            break;
                  case 0xe0:
                  case 0xc0:
                  case 0xa0:
                  case 0x40:
                  case 0x80:cnt1 = 0;
                            pattern = 145;
                            break;
                    default:
                            break;
          }
          break;

 case 144:                              /* �E�N�����N�̋Ȃ��I������  */
          ha(  20 );                   /* ���E�N�����N�̋Ȃ��p�x HANDLE_MAX - 25*/
          sp(  70 ,   60 );
          switch( sensor_inp(MASK4_4) ) {
					
					case 0x06:
					case 0x31:
					case 0x30:
					case 0x60:
					case 0x18:
                  case 0x1c:
                  case 0x0c:
                  case 0x0e:led_out( 0x0 );
                            cnt1 = 0;
                            pattern = 20;
                            break;
                  case 0xe0:
                  case 0xc0:
                  case 0xa0:
                  case 0x40:
                  case 0x80:cnt1 = 0;
                            pattern = 145;
                            break;
                    default:
                            break;
          }
          break;

 case 145:
          ha(  HANDLE_MAX );                   /* ���E�N�����N�̋Ȃ��p�x */
          sp(   70 ,   5 );
          switch( sensor_inp(MASK4_4) ) {

                  case 0xc1:
                  case 0x81:
                  case 0x83:cnt1 = 0;
                            pattern = 142;
                            break;
					case 0x31:
					case 0x30:
					case 0x60:
					case 0x18:
                  case 0x1c:
                  case 0x0c:
				   case 0x06:
                  case 0x0e:led_out( 0x0 );
                            cnt1 = 0;
                            pattern = 20;
                            break;
                    default:
                            break;
          }
          if( cnt1 > 2000 ) {
              cnt1 = 0;
			  pattern = 146;
              break;
		  }
		  break;
   
 case 146:                         /* �R�[�X�A�E�g�ɂ��ً}��~���� */
          ha(  HANDLE_MAX );                   /* ���E�N�����N�̋Ȃ��p�x */
          sp(   70 ,   5 );
          switch( sensor_inp(MASK4_4) ) {

                  case 0xc1:
                  case 0x81:
                  case 0x83:cnt1 = 0;
                            pattern = 142;
                            break;
                  case 0x00:
		          case 0xff:
		          case 0xf7:          
                  case 0xfd:
		          case 0xfb:
		          case 0xef:
		          case 0xbf:
		          case 0xdf:cnt1 = 0;
		                    pattern = 999;
				            break;
		            default:
                            break;
          }
          break;
          
 case 200:                           /* �E�Ԑ��ύX���C�����o���̏��� */
          led_out( 0x1 );
          ha(  0 );
          kyori_flug = 0;
	      cnt1 = 0;
          pattern = 201;
          break;

 case 201:             /* �E�Ԑ��ύX���C�����o��̒ʉߏ󋵂̊m�F���� */
	      if( migikakunin() ) {         /* �N���X���C���`�F�b�N  */
              cnt1 = 0;
	          pattern = 100;
              break;
          }
		  if( kyori_flug == 0 && check_shasenhenkoum() == 1 ){
              kyori_flug = 1;
	   	  } else if( kyori_flug == 1 && check_shasenhenkoum() == 0 ){
              kyori_flug = 2;
		        /* �E�Ԑ��ύX�̒ʉߎ��Ԍv��[ms] ������O���甒���I���܂� */
			  kyoritime = cnt1;
		  }
          if( kyori_flug == 2 ){
	          cnt1 = 0;
	          pattern = 202;
	          break;
		  }
          switch( sensor_inp( MASK4_4 ) ) {

		          case 0x18:ha(   0 );   /*   �Z���^ �� ��  �i   */
                            break;                
                  case 0x1c:ha(   2 );   /* ������� �� ���E�Ȃ� */
                            break;                
                  case 0x0c:ha(   4 );   /* ������� �� ���E�Ȃ� */
                            break;                     
                  case 0x0e:ha(   6 );   /* ������� �� ���E�Ȃ� */
                            break;                     
                  case 0x38:ha(  -2 );   /* ���E��� �� �����Ȃ� */
                            break;                     
                  case 0x30:ha(  -4 );   /* ���E��� �� �����Ȃ� */
                            break;                     
                  case 0x70:ha(  -6 );   /* ���E��� �� �����Ȃ� */
                            break;                     
                    default:
                            break;
          }
		  break;

 case 202:                        /*�E�Ԑ��ύX���C���ʉߌ�̌������� */
		  if( migikakunin() ) {         /* �N���X���C���`�F�b�N  */
              cnt1 = 0;
	          pattern = 100;
              break;
          }
		  kyori_flug = 0;
	      shasenhenkou();
		       if( kyoritime <=  13 ){
		           pattern = 300;
                   break;
		  }
		  else if( kyoritime <=  16 ){
		           pattern = 301;
                   break;
		  }
		  else if( kyoritime <=  19 ){
		           pattern = 302;
                   break;
		  }
		  else if( kyoritime <=  22 ){
		           pattern = 303;
                   break;
		  }
		  else if( kyoritime <=  25 ){
		           pattern = 304;
                   break;
		  }
		  else if( kyoritime <=  28 ){
		           pattern = 305;
                   break;
		  }
		  else if( kyoritime <=  31 ){
		           pattern = 306;
                   break;
		  }
		  else if( kyoritime <=  34 ){
		           pattern = 307;
                   break;
		  }
		  else if( kyoritime <=  37 ){
		           pattern = 308;
                   break;
		  }
		  else if( kyoritime <=  40 ){
		           pattern = 309;
                   break;
		  }
		  else                       {
		           pattern = 310;
                   break;
		  }
		  switch( sensor_inp(MASK4_4) ) {

		      	  case 0x01:
				  case 0x03:
				  case 0x06:
				  case 0x07:
				  case 0x0e:
				  case 0x0c:
		    	  case 0x1c:ha(  15 );
				            break;                
				  case 0x18:ha(  10 );
				            break;                
				  case 0x38:ha(   8 );
				            break;                
				  case 0x30:ha(   6 );
				            break;                
				  case 0x70:ha(   6 );
				            break;                
				  case 0x60:ha(   4 );
				            break;                
				  case 0xe0:ha(   0 );
				            break;                
				  case 0xc0:ha(  -2 );
				            break;                
                  case 0xc1:ha(  -4 );
				            break;
				  case 0x81:ha(  -6 );
				            break;
				  case 0x83:ha(  -8 );
				            break;
				  default:
                            break;
          }
          break;

 case 300:                     /* �E�Ԑ��ύX�O�̎ԑ̂̉E�񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if(c == 0x00){ /* �E�Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x1 );
			  ha( R_HF );	/* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */
              sp_mode(BRAKE, FREE );
          	  sp(   80 ,   55 );
			  cnt1 = 0;
			  pattern = 320;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {

		        
		 		  case 0x01:
				  case 0x03:
				  case 0x06:
				  case 0x07:
				  case 0x0e:
				  case 0x0c:
		    	  case 0x1c:ha(  15 );
				            break;                
				  case 0x18:ha(  10 );
				            break;                
				  case 0x38:ha(   8 );
				            break;                
				  case 0x30:ha(   6 );
				            break;                
				  case 0x70:ha(   6 );
				            break;                
				  case 0x60:ha(   4 );
				            break;                
				  case 0xe0:ha(   0 );
				            break;                
				  case 0xc0:ha(  -2 );
				            break;                
                  case 0xc1:ha(  -4 );
				            break;
				  case 0x81:ha(  -6 );
				            break;
				  case 0x83:ha(  -8 );
				            break;
				  default:
                            break;
          }
          break;

 case 301:                     /* �E�Ԑ��ύX�O�̎ԑ̂̉E�񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if(c == 0x00){ /* �E�Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x1 );
		      ha( R_HF );	/* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */
              sp_mode(BRAKE, FREE );
          	  sp(   80 ,   56 );
			  cnt1 = 0;
			  pattern = 321;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {

		          case 0x01:
				  case 0x03:
				  case 0x06:
				  case 0x07:
				  case 0x0e:
				  case 0x0c:
		    	  case 0x1c:ha(  15 );
				            break;                
				  case 0x18:ha(  10 );
				            break;                
				  case 0x38:ha(   8 );
				            break;                
				  case 0x30:ha(   6 );
				            break;                
				  case 0x70:ha(   6 );
				            break;                
				  case 0x60:ha(   4 );
				            break;                
				  case 0xe0:ha(   0 );
				            break;                
				  case 0xc0:ha(  -2 );
				            break;                
                  case 0xc1:ha(  -4 );
				            break;
				  case 0x81:ha(  -6 );
				            break;
				  case 0x83:ha(  -8 );
				            break;
				  default:
                            break;
          }
          break;

 case 302:                     /* �E�Ԑ��ύX�O�̎ԑ̂̉E�񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if(c == 0x00){ /* �E�Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x1 );
			  ha( R_HF );	/* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */
              sp_mode(BRAKE, FREE );
          	  sp(   81 ,   57 );
			  cnt1 = 0;
			  pattern = 322;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {
				   
		     	  case 0x01:
				  case 0x03:
				  case 0x06:
				  case 0x07:
				  case 0x0e:
				  case 0x0c:
		    	  case 0x1c:ha(  15 );
				            break;                
				  case 0x18:ha(  10 );
				            break;                
				  case 0x38:ha(   8 );
				            break;                
				  case 0x30:ha(   6 );
				            break;                
				  case 0x70:ha(   6 );
				            break;                
				  case 0x60:ha(   4 );
				            break;                
				  case 0xe0:ha(   0 );
				            break;                
				  case 0xc0:ha(  -2 );
				            break;                
                  case 0xc1:ha(  -4 );
				            break;
				  case 0x81:ha(  -6 );
				            break;
				  case 0x83:ha(  -8 );
				            break;
				  default:
                            break;
          }
          break;

 case 303:                     /* �E�Ԑ��ύX�O�̎ԑ̂̉E�񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if(  c == 0x00){ /* �E�Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x1 );
			  ha( R_HF );	/* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */
              sp_mode(BRAKE, FREE );
          	  sp(   81 ,   58 );
			  cnt1 = 0;
			  pattern = 323;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {

		          
		       	  case 0x01:
				  case 0x03:
				  case 0x06:
				  case 0x07:
				  case 0x0e:
				  case 0x0c:
		    	  case 0x1c:ha(  15 );
				            break;                
				  case 0x18:ha(  10 );
				            break;                
				  case 0x38:ha(   8 );
				            break;                
				  case 0x30:ha(   6 );
				            break;                
				  case 0x70:ha(   6 );
				            break;                
				  case 0x60:ha(   4 );
				            break;                
				  case 0xe0:ha(   0 );
				            break;                
				  case 0xc0:ha(  -2 );
				            break;                
                  case 0xc1:ha(  -4 );
				            break;
				  case 0x81:ha(  -6 );
				            break;
				  case 0x83:ha(  -8 );
				            break;
				  default:
                            break;
          }
          break;

 case 304:                     /* �E�Ԑ��ύX�O�̎ԑ̂̉E�񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if(  c == 0x00){ /* �E�Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x1 );
			  ha( R_HF );	/* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */
              sp_mode(BRAKE, FREE );
          	  sp(   82 ,   59 );
			  cnt1 = 0;
			  pattern = 324;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {

		    	  case 0x01:
				  case 0x03:
				  case 0x06:
				  case 0x07:
				  case 0x0e:
				  case 0x0c:
		    	  case 0x1c:ha(  15 );
				            break;                
				  case 0x18:ha(  10 );
				            break;                
				  case 0x38:ha(   8 );
				            break;                
				  case 0x30:ha(   6 );
				            break;                
				  case 0x70:ha(   6 );
				            break;                
				  case 0x60:ha(   4 );
				            break;                
				  case 0xe0:ha(   0 );
				            break;                
				  case 0xc0:ha(  -2 );
				            break;                
                  case 0xc1:ha(  -4 );
				            break;
				  case 0x81:ha(  -6 );
				            break;
				  case 0x83:ha(  -8 );
				            break;
				  default:
                            break;
          }
          break;

 case 305:                     /* �E�Ԑ��ύX�O�̎ԑ̂̉E�񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if(  c == 0x00){ /* �E�Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x1 );
			  ha( R_HF );	/* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */ 
              sp_mode(BRAKE, FREE );
          	  sp(   83 ,   60 );
			  cnt1 = 0;
			  pattern = 325;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {

		      	  case 0x01:
				  case 0x03:
				  case 0x06:
				  case 0x07:
				  case 0x0e:
				  case 0x0c:
		    	  case 0x1c:ha(  15 );
				            break;                
				  case 0x18:ha(  10 );
				            break;                
				  case 0x38:ha(   8 );
				            break;                
				  case 0x30:ha(   6 );
				            break;                
				  case 0x70:ha(   6 );
				            break;                
				  case 0x60:ha(   4 );
				            break;                
				  case 0xe0:ha(   0 );
				            break;                
				  case 0xc0:ha(  -2 );
				            break;                
                  case 0xc1:ha(  -4 );
				            break;
				  case 0x81:ha(  -6 );
				            break;
				  case 0x83:ha(  -8 );
				            break;
				  default:
                            break;
          }
          break;

 case 306:                     /* �E�Ԑ��ύX�O�̎ԑ̂̉E�񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if(  c == 0x00){ /* �E�Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x1 );
			  ha( R_HF );	/* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */
              sp_mode(BRAKE, FREE );
          	  sp(   83 ,   61 );
			  cnt1 = 0;
			  pattern = 326;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {

		          
		        
          }
          break;

 case 307:                     /* �E�Ԑ��ύX�O�̎ԑ̂̉E�񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if(  c == 0x00){ /* �E�Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x1 );
			  ha( R_HF );	/* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */
              sp_mode(BRAKE, FREE );
          	  sp(   83 ,   62 );
			  cnt1 = 0;
			  pattern = 327;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {

		          
		      	  case 0x01:
				  case 0x03:
				  case 0x06:
				  case 0x07:
				  case 0x0e:
				  case 0x0c:
		    	  case 0x1c:ha(  15 );
				            break;                
				  case 0x18:ha(  10 );
				            break;                
				  case 0x38:ha(   8 );
				            break;                
				  case 0x30:ha(   6 );
				            break;                
				  case 0x70:ha(   6 );
				            break;                
				  case 0x60:ha(   4 );
				            break;                
				  case 0xe0:ha(   0 );
				            break;                
				  case 0xc0:ha(  -2 );
				            break;                
                  case 0xc1:ha(  -4 );
				            break;
				  case 0x81:ha(  -6 );
				            break;
				  case 0x83:ha(  -8 );
				            break;
				  default:
                            break;
          }
          break;

 case 308:                     /* �E�Ԑ��ύX�O�̎ԑ̂̉E�񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if(  c == 0x00){ /* �E�Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x1 );
			  ha( R_HF );	/* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */
              sp_mode(BRAKE, FREE );
          	  sp(   84 ,   63 );
			  cnt1 = 0;
			  pattern = 328;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {

		          case 0x01:
				  case 0x03:
				  case 0x06:
				  case 0x07:
				  case 0x0e:
				  case 0x0c:
		    	  case 0x1c:ha(  15 );
				            break;                
				  case 0x18:ha(  10 );
				            break;                
				  case 0x38:ha(   8 );
				            break;                
				  case 0x30:ha(   6 );
				            break;                
				  case 0x70:ha(   6 );
				            break;                
				  case 0x60:ha(   4 );
				            break;                
				  case 0xe0:ha(   0 );
				            break;                
				  case 0xc0:ha(  -2 );
				            break;                
                  case 0xc1:ha(  -4 );
				            break;
				  case 0x81:ha(  -6 );
				            break;
				  case 0x83:ha(  -8 );
				            break;
				  default:
                            break;
          }
          break;

 case 309:                     /* �E�Ԑ��ύX�O�̎ԑ̂̉E�񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if(  c == 0x00){ /* �E�Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x1 );			  
			  ha( R_HF );	/* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */			
              sp_mode(BRAKE, FREE );
          	  sp(   84 ,   64 );
			  cnt1 = 0;
			  pattern = 329;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {

		          
		    	  case 0x01:
				  case 0x03:
				  case 0x06:
				  case 0x07:
				  case 0x0e:
				  case 0x0c:
		    	  case 0x1c:ha(  15 );
				            break;                
				  case 0x18:ha(  10 );
				            break;                
				  case 0x38:ha(   8 );
				            break;                
				  case 0x30:ha(   6 );
				            break;                
				  case 0x70:ha(   6 );
				            break;                
				  case 0x60:ha(   4 );
				            break;                
				  case 0xe0:ha(   0 );
				            break;                
				  case 0xc0:ha(  -2 );
				            break;                
                  case 0xc1:ha(  -4 );
				            break;
				  case 0x81:ha(  -6 );
				            break;
				  case 0x83:ha(  -8 );
				            break;
				  default:
                            break;
          }
          break;

 case 310:                     /* �E�Ԑ��ύX�O�̎ԑ̂̉E�񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00){ /* �E�Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x1 );			  
			  	ha( R_HF );	/* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */			   
              sp_mode(BRAKE, FREE );
          	  sp(   85 ,   65 );
			  cnt1 = 0;
			  pattern = 330;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {

		          case 0x01:
				  case 0x03:
				  case 0x06:
				  case 0x07:
				  case 0x0e:
				  case 0x0c:
		    	  case 0x1c:ha(  15 );
				            break;                
				  case 0x18:ha(  10 );
				            break;                
				  case 0x38:ha(   8 );
				            break;                
				  case 0x30:ha(   6 );
				            break;                
				  case 0x70:ha(   6 );
				            break;                
				  case 0x60:ha(   4 );
				            break;                
				  case 0xe0:ha(   0 );
				            break;                
				  case 0xc0:ha(  -2 );
				            break;                
                  case 0xc1:ha(  -4 );
				            break;
				  case 0x81:ha(  -6 );
				            break;
				  case 0x83:ha(  -8 );
				            break;
				  default:
                            break;
          }
          break;

 case 320: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
         
		  ha( R_HF );	/* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */ 
          sp_mode(BRAKE, FREE );
          sp(   80 ,   55 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 340;
              break;
		  }
          if( sensor_inp(MASK3_3) == ( 0x01 )||( 0x02 )||( 0x03 )||( 0x05 )||( 0x07 ) ) {
              cnt1 = 0;
			  pattern = 400;
              break;
		  }
		  break;

 case 321: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
         
    	  ha( R_HF );	/* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */		   
          sp_mode(BRAKE, FREE );
          sp(   75 ,   51 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 341;
              break;
		  }
          if( sensor_inp(MASK3_3) == ( 0x01 )||( 0x02 )||( 0x03 )||( 0x05 )||( 0x07 ) ) {
              cnt1 = 0;
			  pattern = 401;
              break;
		  }
		  break;

 case 322: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
          
	      ha( R_HF );	/* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */ 
          sp_mode(BRAKE, FREE );
          sp(   81 ,   57 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 342;
              break;
		  }
          if( sensor_inp(MASK3_3) == ( 0x01 )||( 0x02 )||( 0x03 )||( 0x05 )||( 0x07 ) ) {
              cnt1 = 0;
			  pattern = 402;
              break;
		  }
		  break;

 case 323: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
          ha(  R_HF );       /* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */
          sp_mode(BRAKE, FREE );
          sp(   81 ,   58 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 343;
              break;
		  }
          if( sensor_inp(MASK3_3) == ( 0x01 )||( 0x02 )||( 0x03 )||( 0x05 )||( 0x07 ) ) {
              cnt1 = 0;
			  pattern = 403;
              break;
		  }
		  break;

 case 324: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
  
		  ha( R_HF );	/* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */
          sp_mode(BRAKE, FREE );
          sp(   82 ,   59 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 344;
              break;
		  }
          if( sensor_inp(MASK3_3) == ( 0x01 )||( 0x02 )||( 0x03 )||( 0x05 )||( 0x07 ) ) {
              cnt1 = 0;
			  pattern = 404;
              break;
		  }
		  break;

 case 325: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
          
	 	  ha( R_HF );	/* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */		 		 
          sp_mode(BRAKE, FREE );
          sp(   82 ,   60 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 345;
              break;
		  }
          if( sensor_inp(MASK3_3) == ( 0x01 )||( 0x02 )||( 0x03 )||( 0x05 )||( 0x07 ) ) {
              cnt1 = 0;
			  pattern = 405;
              break;
		  }
		  break;

 case 326: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
         
		  ha( R_HF );	/* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */		   
          sp_mode(BRAKE, FREE );
          sp(   83 ,   61 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 346;
              break;
		  }
          if( sensor_inp(MASK3_3) == ( 0x01 )||( 0x02 )||( 0x03 )||( 0x05 )||( 0x07 ) ) {
              cnt1 = 0;
			  pattern = 406;
              break;
		  }
		  break;

 case 327: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
          
		  ha( R_HF );	/* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */ 
          sp_mode(BRAKE, FREE );
          sp(   83 ,   63 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 347;
              break;
		  }
          if( sensor_inp(MASK3_3) == ( 0x01 )||( 0x02 )||( 0x03 )||( 0x05 )||( 0x07 ) ) {
              cnt1 = 0;
			  pattern = 407;
              break;
		  }
		  break;

 case 328: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
         
		  ha( R_HF );	/* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */
          sp_mode(BRAKE, FREE );
          sp(   84 ,   63 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 348;
              break;
		  }
          if( sensor_inp(MASK0_3) == ( 0x01 )||( 0x02 )||( 0x03 )||( 0x05 )||( 0x07 ) ) {
              cnt1 = 0;
			  pattern = 408;
              break;
		  }
		  break;

 case 329: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
         
		  ha( R_HF );	/* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */
          sp_mode(BRAKE, FREE );
          sp(   84 ,   64 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 349;
              break;
		  }
          if( sensor_inp(MASK3_3) == ( 0x01 )||( 0x02 )||( 0x03 )||( 0x05 )||( 0x07 ) ) {
              cnt1 = 0;
			  pattern = 409;
              break;
		  }
		  break;

 case 330: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
        
		  ha( R_HF );	/* ���E�Ԑ��ύX���̃n���h���̋Ȃ��p�x */
          sp_mode(BRAKE, FREE );
          sp(   85 ,   65 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 350;
              break;
		  }
          if( sensor_inp(MASK3_3) == ( 0x01 )||( 0x02 )||( 0x03 )||( 0x05 )||( 0x07 ) ) {
              cnt1 = 0;
			  pattern = 410;
              break;
		  }
		  break;

 case 340: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */  
          ha(   0 );         /* �E�Ԑ��ύX���̃n���h���̖߂��p�x */
          sp_mode(FREE, FREE );
          sp(   75 ,   75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x01:              
				  case 0x02:               
				  case 0x03:               
				  case 0x05:               
				  case 0x07:cnt1 = 0;
                            pattern = 400;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 341: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */  
          ha(   0 );         /* �E�Ԑ��ύX���̃n���h���̖߂��p�x */
          sp_mode(FREE, FREE );
          sp(   75 ,   75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x01:              
				  case 0x02:               
				  case 0x03:               
				  case 0x05:               
				  case 0x07:cnt1 = 0;
                            pattern = 401;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 342: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */  
          ha(   0 );         /* �E�Ԑ��ύX���̃n���h���̖߂��p�x */
          sp_mode(FREE, FREE );
          sp(   75 ,   75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x01:              
				  case 0x02:               
				  case 0x03:               
				  case 0x05:               
				  case 0x07:cnt1 = 0;
                            pattern = 402;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 343: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */ 
          ha(   0 );         /* �E�Ԑ��ύX���̃n���h���̖߂��p�x */
          sp_mode(FREE, FREE );
          sp(   75 ,   75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x01:              
				  case 0x02:               
				  case 0x03:               
				  case 0x05:               
				  case 0x07:cnt1 = 0;
                            pattern = 403;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 344: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */  
          ha(   0 );         /* �E�Ԑ��ύX���̃n���h���̖߂��p�x */
          sp_mode(FREE, FREE );
          sp(   75 ,   75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x01:              
				  case 0x02:               
				  case 0x03:               
				  case 0x05:               
				  case 0x07:cnt1 = 0;
                            pattern = 404;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 345: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */  
          ha(   0 );         /* �E�Ԑ��ύX���̃n���h���̖߂��p�x */
          sp_mode(FREE, FREE );
          sp(   75 ,   75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x01:              
				  case 0x02:               
				  case 0x03:               
				  case 0x05:               
				  case 0x07:cnt1 = 0;
                            pattern = 405;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 346: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */  
          ha(   0 );         /* �E�Ԑ��ύX���̃n���h���̖߂��p�x */
          sp_mode(FREE, FREE );
          sp(   75 ,   75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x01:              
				  case 0x02:               
				  case 0x03:               
				  case 0x05:               
				  case 0x07:cnt1 = 0;
                            pattern = 406;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 347: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */  
          ha(   0 );         /* �E�Ԑ��ύX���̃n���h���̖߂��p�x */
          sp_mode(FREE, FREE );
          sp(   75 ,   75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x01:              
				  case 0x02:               
				  case 0x03:               
				  case 0x05:               
				  case 0x07:cnt1 = 0;
                            pattern = 407;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 348: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */  
          ha(   0 );         /* �E�Ԑ��ύX���̃n���h���̖߂��p�x */
          sp_mode(FREE, FREE );
          sp(   75 ,   75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x01:              
				  case 0x02:               
				  case 0x03:               
				  case 0x05:               
				  case 0x07:cnt1 = 0;
                            pattern = 408;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 349: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */  
          ha(   0 );         /* �E�Ԑ��ύX���̃n���h���̖߂��p�x */
          sp_mode(FREE, FREE );
          sp(  75 ,  75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x01:              
				  case 0x02:               
				  case 0x03:               
				  case 0x05:               
				  case 0x07:cnt1 = 0;
                            pattern = 409;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 350: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */  
          ha(   0 );         /* �E�Ԑ��ύX���̃n���h���̖߂��p�x */
          sp_mode(FREE, FREE );
          sp(   75 ,   75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x01:              
				  case 0x02:               
				  case 0x03:               
				  case 0x05:               
				  case 0x07:cnt1 = 0;
                            pattern = 410;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 400: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
 		  
          switch( sensor_inp(MASK4_4) ) {

                  case 0x1c:ha(  -1 );
				  			sp_mode(FREE, FREE );
                            sp(   73 ,   75 );
		                    break;
			      case 0x18:ha(  -2 );
				  			sp_mode(FREE, FREE );
                            sp(   71 ,   74 );
		                    break;
				  case 0x38:ha(  -4 );
				  			sp_mode(FREE, FREE );
                            sp(   69 ,   73 );
		                    break;
                  case 0x30:ha(  -8 );
				  			sp_mode(FREE, FREE );
				            sp(   65 ,   72 );
		                    break;
				  case 0x70:ha( -12 );
				  			sp_mode(FREE, BRAKE );
				            sp(   60 ,   71 );
		                    cnt1 = 0;
                            pattern = 420;
                            break;
				  case 0x60:ha( -16 );
				  			sp_mode(FREE, BRAKE );
				            sp(   55 ,   70 );
		                    cnt1 = 0;
                            pattern = 420;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 401: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x1c:ha(  -1 );
				  			sp_mode(FREE, FREE );
                            sp(   75 ,   76 );
		                    break;
			      case 0x18:ha(  -2 );
				  			sp_mode(FREE, FREE );
                            sp(   73 ,   75 );
		                    break;
				  case 0x38:ha(  -4 );
				  			sp_mode(FREE, FREE );
                            sp(   70 ,   74 );
		                    break;
				  			sp_mode(FREE, FREE );
                  case 0x30:ha(  -8 );
				  			sp_mode(FREE, FREE );
				            sp(   66 ,   73 );
		                    break;
				  case 0x70:ha( -12 );
				  			sp_mode(FREE, BRAKE);
				            sp(   61 ,   72 );
		                    cnt1 = 0;
                            pattern = 421;
                            break;
				  case 0x60:ha( -16 );
				  			sp_mode(FREE, BRAKE);
				            sp(   56 ,   71 );
		                    cnt1 = 0;
                            pattern = 421;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 402: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x1c:ha(  -1 );
				  			sp_mode(FREE, FREE);
                            sp(   76 ,   77 );
		                    break;
			      case 0x18:ha(  -2 );
				  			sp_mode(FREE, FREE);
                            sp(   74 ,   76 );
		                    break;
				  case 0x38:ha(  -4 );
				  			sp_mode(FREE, FREE);
                            sp(   71 ,   75 );
		                    break;
                  case 0x30:ha(  -8 );
				  			sp_mode(FREE, FREE);
                            sp(   67 ,   74 );
		                    break;
                  case 0x70:ha( -12 );
				  			sp_mode(FREE, BRAKE);
				            sp(   62 ,   73 );
		                    cnt1 = 0;
                            pattern = 422;
                            break;
				  case 0x60:ha( -16 );
				  			sp_mode(FREE, BRAKE);
				            sp(   57 ,   72 );
		                    cnt1 = 0;
                            pattern = 422;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 403: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x1c:ha(  -1 );
				  			sp_mode(FREE, FREE);
                            sp(   77 ,   78 );
		                    break;
			      case 0x18:ha(  -2 );
				  			sp_mode(FREE, FREE);
                            sp(   75 ,   77 );
		                    break;
				  case 0x38:ha(  -4 );
				  			sp_mode(FREE, FREE);
                            sp(   72 ,   76 );
		                    break;
                  case 0x30:ha(  -8 );
				  			sp_mode(FREE, FREE);
                            sp(   68 ,   75 );
		                    break;
                  case 0x70:ha( -12 );
				  			sp_mode(FREE, BRAKE);
				            sp(   63 ,   74 );
		                    cnt1 = 0;
                            pattern = 423;
                            break;
				  case 0x60:ha( -16 );
				  			sp_mode(FREE, BRAKE);
				            sp(   58 ,   73 );
		                    cnt1 = 0;
                            pattern = 423;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 404: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x1c:ha(  -1 );
				  			sp_mode(FREE, FREE);
                            sp(   78 ,   79 );
		                    break;
			      case 0x18:ha(  -2 );
				  			sp_mode(FREE, FREE);
                            sp(   76 ,   78 );
		                    break;
				  case 0x38:ha(  -4 );
				  			sp_mode(FREE, FREE);
                            sp(   73 ,   77 );
		                    break;
                  case 0x30:ha(  -8 );
				  			sp_mode(FREE, FREE);
                            sp(   69 ,   76 );
		                    break;
                  case 0x70:ha( -12 );
				  			sp_mode(FREE, BRAKE);
				            sp(   64 ,   75 );
		                    cnt1 = 0;
                            pattern = 424;
                            break;
				  case 0x60:ha( -16 );
				  			sp_mode(FREE, BRAKE);
				            sp(   59 ,   74 );
		                    cnt1 = 0;
                            pattern = 424;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 405: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x1c:ha(  -1 );
				  			sp_mode(FREE, FREE);
                            sp(   79 ,   80 );
		                    break;
			      case 0x18:ha(  -2 );
				  			sp_mode(FREE, FREE);
                            sp(   77 ,   79 );
		                    break;
				  case 0x38:ha(  -4 );
				  			sp_mode(FREE, FREE);
                            sp(   74 ,   78 );
		                    break;
                  case 0x30:ha(  -8 );
				  			sp_mode(FREE, FREE);
                            sp(   70 ,   77 );
		                    break;
                  case 0x70:ha( -12 );
				  			sp_mode(FREE, BRAKE);
				            sp(   65 ,   76 );
		                    cnt1 = 0;
                            pattern = 425;
                            break;
				  case 0x60:ha( -16 );
				  			sp_mode(FREE, BRAKE);
				            sp(   60 ,   75 );
		                    cnt1 = 0;
                            pattern = 425;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 406: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x1c:ha(  -1 );
				  			sp_mode(FREE, FREE);
                            sp(   80 ,   81 );
		                    break;
			      case 0x18:ha(  -2 );
				  			sp_mode(FREE, FREE);
                            sp(   78 ,   80 );
		                    break;
				  case 0x38:ha(  -4 );
				  			sp_mode(FREE, FREE);
                            sp(   75 ,   79 );
		                    break;
                  case 0x30:ha(  -8 );
				  			sp_mode(FREE, FREE);
                            sp(   71 ,   78 );
		                    break;
                  case 0x70:ha( -12 );
				  			sp_mode(FREE, BRAKE);
				            sp(   66 ,   77 );
		                    cnt1 = 0;
                            pattern = 426;
                            break;
				  case 0x60:ha( -16 );
				  			sp_mode(FREE, BRAKE);
				            sp(   61 ,   76 );
		                    cnt1 = 0;
                            pattern = 426;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 407: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x1c:ha(  -1 );
				  			sp_mode(FREE, FREE);
                            sp(   81 ,   82 );
		                    break;
			      case 0x18:ha(  -2 );
				  			sp_mode(FREE, FREE);
                            sp(   79 ,   81 );
		                    break;
				  case 0x38:ha(  -4 );
				  			sp_mode(FREE, FREE);
                            sp(   76 ,   80 );
		                    break;
                  case 0x30:ha(  -8 );
				  			sp_mode(FREE, FREE);
                            sp(   72 ,   79 );
		                    break;
                  case 0x70:ha( -12 );
				  			sp_mode(FREE, BRAKE);
				            sp(   67 ,   78 );
		                    cnt1 = 0;
                            pattern = 427;
                            break;
				  case 0x60:ha( -16 );
				  			sp_mode(FREE, BRAKE);
				            sp(   62 ,   77 );
		                    cnt1 = 0;
                            pattern = 427;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 408: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x1c:ha(  -1 );
				  			sp_mode(FREE, FREE);
                            sp(   82 ,   83 );
		                    break;
			      case 0x18:ha(  -2 );
				  			sp_mode(FREE, FREE);
                            sp(   80 ,   82 );
		                    break;
				  case 0x38:ha(  -4 );
				  			sp_mode(FREE, FREE);
                            sp(   77 ,   81 );
		                    break;
                  case 0x30:ha(  -8 );
				  			sp_mode(FREE, FREE);
                            sp(   73 ,   80 );
		                    break;
                  case 0x70:ha( -12 );
				  			sp_mode(FREE, BRAKE);
				            sp(   68 ,   79 );
		                    cnt1 = 0;
                            pattern = 428;
                            break;
				  case 0x60:ha( -16 );
				  			sp_mode(FREE, BRAKE);
				            sp(   63 ,   78 );
		                    cnt1 = 0;
                            pattern = 428;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 409: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x1c:ha(  -1 );
				  			sp_mode(FREE, FREE);
                            sp(   83 ,   84 );
		                    break;
			      case 0x18:ha(  -2 );
				  			sp_mode(FREE, FREE);
                            sp(   81 ,   83 );
		                    break;
				  case 0x38:ha(  -4 );
				  			sp_mode(FREE, FREE);
                            sp(   78 ,   82 );
		                    break;
                  case 0x30:ha(  -8 );
				  			sp_mode(FREE, FREE);
                            sp(   74 ,   81 );
		                    break;
                  case 0x70:ha( -12 );
				  			sp_mode(FREE, BRAKE);
				            sp(   69 ,   80 );
		                    cnt1 = 0;
                            pattern = 429;
                            break;
				  case 0x60:ha( -16 );
				  			sp_mode(FREE, BRAKE);
				            sp(   64 ,   79 );
		                    cnt1 = 0;
                            pattern = 429;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 410: /* �E�Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x1c:ha(  -1 );
				  			sp_mode(FREE, FREE);
                            sp(   84 ,   85 );
		                    break;
			      case 0x18:ha(  -2 );
				  			sp_mode(FREE, FREE);
                            sp(   82 ,   84 );
		                    break;
				  case 0x38:ha(  -4 );
				  			sp_mode(FREE, FREE);
                            sp(   79 ,   83 );
		                    break;
                  case 0x30:ha(  -8 );
				  			sp_mode(FREE, FREE);
                            sp(   75 ,   82 );
		                    break;
                  case 0x70:ha( -12 );
				  			sp_mode(FREE, BRAKE);
				            sp(   70 ,   81 );
		                    cnt1 = 0;
                            pattern = 430;
                            break;
				  case 0x60:ha( -16 );
				  			sp_mode(FREE, BRAKE);
				            sp(   65 ,   80 );
		                    cnt1 = 0;
                            pattern = 430;
                            break;		  
                    default:
                            break;
          }
          break;
  
 case 420:             /* �E�Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		          case 0x1c:          
                  case 0x18:
		          case 0x38:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x30:ha(  -8 );
				            sp(   65 ,   72 );
		                    break;
				  case 0x70:ha( -12 );
				            sp(   60 ,   71 );
		                    break;
				  case 0x60:ha( -16 );
				            sp(   55 ,   70 );
		                    break;
                  case 0xe0:ha( -20 );
				            sp(   50 ,   65 );
		                    break;
                  case 0xc0:ha( -24 );
				            sp(   40 ,   60 );
		                    cnt1 = 0;
                            pattern = 440;
                            break;		  
                    default:
                            break;
          }
          break;

 case 421:             /* �E�Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		          case 0x1c:          
                  case 0x18:
		          case 0x38:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x30:ha(  -8 );
				            sp(   66 ,   73 );
		                    break;
				  case 0x70:ha( -12 );
				            sp(   61 ,   72 );
		                    break;
				  case 0x60:ha( -16 );
				            sp(   56 ,   71 );
		                    break;
                  case 0xe0:ha( -20 );
				            sp(   51 ,   66 );
		                    break;
                  case 0xc0:ha( -24 );
				            sp(   41 ,   61 );
		                    cnt1 = 0;
                            pattern = 441;
                            break;		  
                    default:
                            break;
          }
          break;

 case 422:             /* �E�Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		          case 0x1c:          
                  case 0x18:
		          case 0x38:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x30:ha(  -8 );
				            sp(   67 ,   74 );
		                    break;
				  case 0x70:ha( -12 );
				            sp(   62 ,   73 );
		                    break;
				  case 0x60:ha( -16 );
				            sp(   57 ,   72 );
		                    break;
                  case 0xe0:ha( -20 );
				            sp(   52 ,   67 );
		                    break;
                  case 0xc0:ha( -24 );
				            sp(   42 ,   62 );
		                    cnt1 = 0;
                            pattern = 442;
                            break;		  
                    default:
                            break;
          }
          break;

 case 423:             /* �E�Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		          case 0x1c:          
                  case 0x18:
		          case 0x38:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x30:ha(  -8 );
				            sp(   68 ,   75 );
		                    break;
				  case 0x70:ha( -12 );
				            sp(   63 ,   74 );
		                    break;
				  case 0x60:ha( -16 );
				            sp(   58 ,   73 );
		                    break;
                  case 0xe0:ha( -20 );
				            sp(   53 ,   68 );
		                    break;
                  case 0xc0:ha( -24 );
				            sp(   43 ,   63 );
		                    cnt1 = 0;
                            pattern = 443;
                            break;		  
                    default:
                            break;
          }
          break;

 case 424:             /* �E�Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		          case 0x1c:          
                  case 0x18:
		          case 0x38:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x30:ha(  -8 );
				            sp(   69 ,   76 );
		                    break;
				  case 0x70:ha( -12 );
				            sp(   64 ,   75 );
		                    break;
				  case 0x60:ha( -16 );
				            sp(   59 ,   74 );
		                    break;
                  case 0xe0:ha( -20 );
				            sp(   54 ,   69 );
		                    break;
                  case 0xc0:ha( -24 );
				            sp(   44 ,   64 );
		                    cnt1 = 0;
                            pattern = 444;
                            break;		  
                    default:
                            break;
          }
          break;

 case 425:             /* �E�Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		          case 0x1c:          
                  case 0x18:
		          case 0x38:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x30:ha(  -8 );
				            sp(   70 ,   77 );
		                    break;
				  case 0x70:ha( -12 );
				            sp(   65 ,   76 );
		                    break;
				  case 0x60:ha( -16 );
				            sp(   60 ,   75 );
		                    break;
                  case 0xe0:ha( -20 );
				            sp(   55 ,   70 );
		                    break;
                  case 0xc0:ha( -24 );
				            sp(   45 ,   65 );
		                    cnt1 = 0;
                            pattern = 445;
                            break;		  
                    default:
                            break;
          }
          break;

 case 426:             /* �E�Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		          case 0x1c:          
                  case 0x18:
		          case 0x38:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x30:ha(  -8 );
				            sp(   71 ,   78 );
		                    break;
				  case 0x70:ha( -12 );
				            sp(   66 ,   77 );
		                    break;
				  case 0x60:ha( -16 );
				            sp(   61 ,   76 );
		                    break;
                  case 0xe0:ha( -20 );
				            sp(   56 ,   71 );
		                    break;
                  case 0xc0:ha( -24 );
				            sp(   46 ,   66 );
		                    cnt1 = 0;
                            pattern = 446;
                            break;		  
                    default:
                            break;
          }
          break;

 case 427:             /* �E�Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {
			  
                  case 0x3c:
		          case 0x1c:          
                  case 0x18:
		          case 0x38:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x30:ha(  -8 );
				            sp(   72 ,   79 );
		                    break;
				  case 0x70:ha( -12 );
				            sp(   67 ,   78 );
		                    break;
				  case 0x60:ha( -16 );
				            sp(   62 ,   77 );
		                    break;
                  case 0xe0:ha( -20 );
				            sp(   57 ,   72 );
		                    break;
                  case 0xc0:ha( -24 );
				            sp(   47 ,   67 );
		                    cnt1 = 0;
                            pattern = 447;
                            break;		  
                    default:
                            break;
          }
          break;

 case 428:             /* �E�Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		          case 0x1c:          
                  case 0x18:
		          case 0x38:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x30:ha(  -8 );
				            sp(   73 ,   80 );
		                    break;
				  case 0x70:ha( -12 );
				            sp(   68 ,   79 );
		                    break;
				  case 0x60:ha( -16 );
				            sp(   63 ,   78 );
		                    break;
                  case 0xe0:ha( -20 );
				            sp(   58 ,   73 );
		                    break;
                  case 0xc0:ha( -24 );
				            sp(   48 ,   68 );
		                    cnt1 = 0;
                            pattern = 448;
                            break;		  
                    default:
                            break;
          }
          break;

 case 429:             /* �E�Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		          case 0x1c:          
                  case 0x18:
		          case 0x38:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x30:ha(  -8 );
				            sp(   74 ,   81 );
		                    break;
				  case 0x70:ha( -12 );
				            sp(   69 ,   80 );
		                    break;
				  case 0x60:ha( -16 );
				            sp(   64 ,   79 );
		                    break;
                  case 0xe0:ha( -20 );
				            sp(   59 ,   74 );
		                    break;
                  case 0xc0:ha( -24 );
				            sp(   49 ,   69 );
		                    cnt1 = 0;
                            pattern = 449;
                            break;		  
                    default:
                            break;
          }
          break;

 case 430:             /* �E�Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		          case 0x1c:          
                  case 0x18:
		          case 0x38:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x30:ha(  -8 );
				            sp(   75 ,   82 );
		                    break;
				  case 0x70:ha( -12 );
				            sp(   70 ,   81 );
		                    break;
				  case 0x60:ha( -16 );
				            sp(   65 ,   80 );
		                    break;
                  case 0xe0:ha( -20 );
				            sp(   60 ,   75 );
		                    break;
                  case 0xc0:ha( -24 );
				            sp(   50 ,   70 );
		                    cnt1 = 0;
                            pattern = 450;
                            break;		  
                    default:
                            break;
          }
          break;

 case 440: /* �E�Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		          case 0xe0:cnt1 = 0;
                            pattern = 420;
                            break;	  
                  case 0xc0:ha( -24 );
				            sp(   40 ,   60 );
		                    break;
				  case 0xc1:ha( -27 );
				            sp(   30 ,   55 );
		                    break;
		          case 0x81:ha( -30 );
				            sp(   25 ,   50 );
		                    break;
		          case 0x83:ha( -33 );
				            sp(   20 ,   45 );
		                    break;
		          case 0x03:ha( -36 );
				            sp(   15 ,   35 );
		                    break;
		          case 0x07:ha( -39 );
				            sp(   10 ,   25 );
		                    break;
		          case 0x06:ha( -42 );
				            sp(    5 ,   15 );
		                    break;
		            default:
                            break;
          }
          break;

 case 441: /* �E�Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		          case 0xe0:cnt1 = 0;
                            pattern = 421;
                            break;	  
                  case 0xc0:ha( -24 );
				            sp(   41 ,   61 );
		                    break;
				  case 0xc1:ha( -27 );
				            sp(   31 ,   56 );
		                    break;
		          case 0x81:ha( -30 );
				            sp(   26 ,   51 );
		                    break;
		          case 0x83:ha( -33 );
				            sp(   21 ,   46 );
		                    break;
		          case 0x03:ha( -36 );
				            sp(   16 ,   36 );
		                    break;
		          case 0x07:ha( -39 );
				            sp(   11 ,   26 );
		                    break;
		          case 0x06:ha( -42 );
				            sp(    6 ,   16 );
		                    break;
		            default:
                            break;
          }
          break;

 case 442: /* �E�Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		          case 0xe0:cnt1 = 0;
                            pattern = 422;
                            break;	  
                  case 0xc0:ha( -24 );
				            sp(   42 ,   62 );
		                    break;
				  case 0xc1:ha( -27 );
				            sp(   32 ,   57 );
		                    break;
		          case 0x81:ha( -30 );
				            sp(   27 ,   52 );
		                    break;
		          case 0x83:ha( -33 );
				            sp(   22 ,   47 );
		                    break;
		          case 0x03:ha( -36 );
				            sp(   17 ,   37 );
		                    break;
		          case 0x07:ha( -39 );
				            sp(   12 ,   27 );
		                    break;
		          case 0x06:ha( -42 );
				            sp(    7 ,   17 );
		                    break;
		            default:
                            break;
          }
          break;

 case 443: /* �E�Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		          case 0xe0:cnt1 = 0;
                            pattern = 423;
                            break;	  
                  case 0xc0:ha( -24 );
				            sp(   43 ,   63 );
		                    break;
				  case 0xc1:ha( -27 );
				            sp(   33 ,   58 );
		                    break;
		          case 0x81:ha( -30 );
				            sp(   28 ,   53 );
		                    break;
		          case 0x83:ha( -33 );
				            sp(   23 ,   48 );
		                    break;
		          case 0x03:ha( -36 );
				            sp(   18 ,   38 );
		                    break;
		          case 0x07:ha( -39 );
				            sp(   13 ,   28 );
		                    break;
		          case 0x06:ha( -42 );
				            sp(    8 ,   18 );
		                    break;
		            default:
                            break;
          }
          break;

 case 444: /* �E�Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		          case 0xe0:cnt1 = 0;
                            pattern = 424;
                            break;	  
                  case 0xc0:ha( -24 );
				            sp(   44 ,   64 );
		                    break;
				  case 0xc1:ha( -27 );
				            sp(   34 ,   59 );
		                    break;
		          case 0x81:ha( -30 );
				            sp(   29 ,   54 );
		                    break;
		          case 0x83:ha( -33 );
				            sp(   24 ,   49 );
		                    break;
		          case 0x03:ha( -36 );
				            sp(   19 ,   39 );
		                    break;
		          case 0x07:ha( -39 );
				            sp(   14 ,   29 );
		                    break;
		          case 0x06:ha( -42 );
				            sp(    9 ,   19 );
		                    break;
		            default:
                            break;
          }
          break;

 case 445: /* �E�Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		          case 0xe0:cnt1 = 0;
                            pattern = 425;
                            break;	  
                  case 0xc0:ha( -24 );
				            sp(   45 ,   65 );
		                    break;
				  case 0xc1:ha( -27 );
				            sp(   35 ,   60 );
		                    break;
		          case 0x81:ha( -30 );
				            sp(   30 ,   55 );
		                    break;
		          case 0x83:ha( -33 );
				            sp(   25 ,   50 );
		                    break;
		          case 0x03:ha( -36 );
				            sp(   20 ,   40 );
		                    break;
		          case 0x07:ha( -39 );
				            sp(   15 ,   30 );
		                    break;
		          case 0x06:ha( -42 );
				            sp(   10 ,   20 );
		                    break;
		            default:
                            break;
          }
          break;

 case 446: /* �E�Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		          case 0xe0:cnt1 = 0;
                            pattern = 426;
                            break;	  
                  case 0xc0:ha( -24 );
				            sp(   46 ,   66 );
		                    break;
				  case 0xc1:ha( -27 );
				            sp(   36 ,   61 );
		                    break;
		          case 0x81:ha( -30 );
				            sp(   31 ,   56 );
		                    break;
		          case 0x83:ha( -33 );
				            sp(   26 ,   51 );
		                    break;
		          case 0x03:ha( -36 );
				            sp(   21 ,   41 );
		                    break;
		          case 0x07:ha( -39 );
				            sp(   16 ,   31 );
		                    break;
		          case 0x06:ha( -42 );
				            sp(   11 ,   21 );
		                    break;
		            default:
                            break;
          }
          break;

 case 447: /* �E�Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		          case 0xe0:cnt1 = 0;
                            pattern = 427;
                            break;	  
                  case 0xc0:ha( -24 );
				            sp(   47 ,   67 );
		                    break;
				  case 0xc1:ha( -27 );
				            sp(   37 ,   62 );
		                    break;
		          case 0x81:ha( -30 );
				            sp(   32 ,   57 );
		                    break;
		          case 0x83:ha( -33 );
				            sp(   27 ,   52 );
		                    break;
		          case 0x03:ha( -36 );
				            sp(   22 ,   42 );
		                    break;
		          case 0x07:ha( -39 );
				            sp(   17 ,   32 );
		                    break;
		          case 0x06:ha( -42 );
				            sp(   12 ,   22 );
		                    break;
		            default:
                            break;
          }
          break;

 case 448: /* �E�Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		          case 0xe0:cnt1 = 0;
                            pattern = 428;
                            break;	  
                  case 0xc0:ha( -24 );
				            sp(   48 ,   68 );
		                    break;
				  case 0xc1:ha( -27 );
				            sp(   38 ,   63 );
		                    break;
		          case 0x81:ha( -30 );
				            sp(   33 ,   58 );
		                    break;
		          case 0x83:ha( -33 );
				            sp(   28 ,   53 );
		                    break;
		          case 0x03:ha( -36 );
				            sp(   23 ,   43 );
		                    break;
		          case 0x07:ha( -39 );
				            sp(   18 ,   33 );
		                    break;
		          case 0x06:ha( -42 );
				            sp(   13 ,   23 );
		                    break;
		            default:
                            break;
          }
          break;

 case 449: /* �E�Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		          case 0xe0:cnt1 = 0;
                            pattern = 429;
                            break;	  
                  case 0xc0:ha( -24 );
				            sp(   49 ,   69 );
		                    break;
				  case 0xc1:ha( -27 );
				            sp(   39 ,   64 );
		                    break;
		          case 0x81:ha( -30 );
				            sp(   34 ,   59 );
		                    break;
		          case 0x83:ha( -33 );
				            sp(   29 ,   54 );
		                    break;
		          case 0x03:ha( -36 );
				            sp(   24 ,   44 );
		                    break;
		          case 0x07:ha( -39 );
				            sp(   19 ,   34 );
		                    break;
		          case 0x06:ha( -42 );
				            sp(   14 ,   24 );
		                    break;
		            default:
                            break;
          }
          break;

 case 450: /* �E�Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x60:
		          case 0xe0:cnt1 = 0;
                            pattern = 430;
                            break;	  
                  case 0xc0:ha( -24 );
				            sp(   50 ,   70 );
		                    break;
				  case 0xc1:ha( -27 );
				            sp(   40 ,   65 );
		                    break;
		          case 0x81:ha( -30 );
				            sp(   35 ,   60 );
		                    break;
		          case 0x83:ha( -33 );
				            sp(   30 ,   55 );
		                    break;
		          case 0x03:ha( -36 );
				            sp(   25 ,   45 );
		                    break;
		          case 0x07:ha( -39 );
				            sp(   20 ,   35 );
		                    break;
		          case 0x06:ha( -42 );
				            sp(   15 ,   25 );
		                    break;
		            default:
                            break;
          }
          break;

 case 500:                           /* ���Ԑ��ύX���C�����o���̏��� */
          led_out( 0x2 );
          ha(   0 );
          kyori_flug = 0;
	      cnt1 = 0;
		  pattern = 501;
          break;

 case 501:             /* ���Ԑ��ύX���C�����o��̒ʉߏ󋵂̊m�F���� */
	      if( hidarikakunin() ) {          /* �N���X���C���`�F�b�N */
              cnt1 = 0;
	          pattern = 100;
              break;
          }
		  if( kyori_flug == 0 && check_shasenhenkouh() == 1 ){
              kyori_flug = 1;
	   	  } else if( kyori_flug == 1 && check_shasenhenkouh() == 0 ){
              kyori_flug = 2;
		        /* �E�Ԑ��ύX�̒ʉߎ��Ԍv��[ms] ������O���甒���I���܂� */
              kyoritime = cnt1;
		  }
          if( kyori_flug == 2 ){
	          cnt1 = 0;
	          pattern = 502;
	          break;
		  }
          switch( sensor_inp( MASK4_4 ) ) {

		          case 0x18:ha(   0 );   /*   �Z���^ �� ��  �i   */
                            break;                
                  case 0x1c:ha(   2 );   /* ������� �� ���E�Ȃ� */
                            break;                
                  case 0x0c:ha(   4 );   /* ������� �� ���E�Ȃ� */
                            break;                     
                  case 0x0e:ha(   6 );   /* ������� �� ���E�Ȃ� */
                            break;                     
                  case 0x38:ha(  -2 );   /* ���E��� �� �����Ȃ� */
                            break;                     
                  case 0x30:ha(  -4 );   /* ���E��� �� �����Ȃ� */
                            break;                     
                  case 0x70:ha(  -6 );   /* ���E��� �� �����Ȃ� */
                            break;                     
                    default:
                            break;
          }
		  break;

 case 502:                        /*���Ԑ��ύX���C���ʉߌ�̌������� */
		  if( hidarikakunin() ) {          /* �N���X���C���`�F�b�N */
              cnt1 = 0;
	          pattern = 100;
              break;
          }
		  kyori_flug = 0;
	      shasenhenkou();
		       if( kyoritime <=  13 ){
		           pattern = 600;
                   break;
		  }
		  else if( kyoritime <=  16 ){
		           pattern = 601;
                   break;
		  }
		  else if( kyoritime <=  19 ){
		           pattern = 602;
                   break;
		  }
		  else if( kyoritime <=  22 ){
		           pattern = 603;
                   break;
		  }
		  else if( kyoritime <=  25 ){
		           pattern = 604;
                   break;
		  }
		  else if( kyoritime <=  28 ){
		           pattern = 605;
                   break;
		  }
		  else if( kyoritime <=  31 ){
		           pattern = 606;
                   break;
		  }
		  else if( kyoritime <=  34 ){
		           pattern = 607;
                   break;
		  }
		  else if( kyoritime <=  37 ){
		           pattern = 608;
                   break;
		  }
		  else if( kyoritime <=  40 ){
		           pattern = 609;
                   break;
		  }
		  else                       {
		           pattern = 610;
                   break;
		  }
		  switch( sensor_inp(MASK4_4) ) {

		          
		          case 0x70:
				  case 0x60:
				  case 0x30:
		          case 0x38:ha(  -15 );
				            break;                
                  case 0x18:ha(  -10 );
				            break;                    
               	  case 0x1c:ha(  -8 );
				            break;                     
                  case 0x0c:ha(  -6 );
				            break;                      
                  case 0x0e:ha(  -4 );
				            break;                      
                  case 0x06:ha(  -4 );
				            break;                     
                  case 0x07:ha(   0 );
				            break;                     
                  case 0x03:ha(   2 );
				            break;                     
                  case 0x83:ha(   4 );
				            break;
				  case 0x81:ha(   6 );
				            break;
				  case 0xc1:ha(   8 );
				            break;
				  case 0xe0:
				  case 0xc0:ha(   12 );
				            break;
				    default:
                            break;
          }
          break;

 case 600:                     /* ���Ԑ��ύX�O�̎ԑ̂̍��񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){ /* ���Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x2 );
			  ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */	
			  sp_mode(FREE,BRAKE);		  
              sp(   55 ,   80 );
			  cnt1 = 0;
			  pattern = 620;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {

		          case 0x70:
				  case 0x60:
				  case 0x30:
		          case 0x38:ha(  -15 );
				            break;                
                  case 0x18:ha(  -10 );
				            break;                    
               	  case 0x1c:ha(  -8 );
				            break;                     
                  case 0x0c:ha(  -6 );
				            break;                      
                  case 0x0e:ha(  -4 );
				            break;                      
                  case 0x06:ha(  -4 );
				            break;                     
                  case 0x07:ha(   0 );
				            break;                     
                  case 0x03:ha(   2 );
				            break;                     
                  case 0x83:ha(   4 );
				            break;
				  case 0x81:ha(   6 );
				            break;
				  case 0xc1:ha(   8 );
				            break;
				  case 0xe0:
				  case 0xc0:ha(   12 );
				            break;
				    default:
                            break;
          }
          break;
		  
 case 601:                     /* ���Ԑ��ύX�O�̎ԑ̂̍��񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){ /* ���Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x2 );			  
			  ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */
             sp_mode(FREE,BRAKE);		   
			  sp(   56 ,   80 );
			  cnt1 = 0;
			  pattern = 621;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {

		          
		         case 0x70:
				  case 0x60:
				  case 0x30:
		          case 0x38:ha(  -15 );
				            break;                
                  case 0x18:ha(  -10 );
				            break;                    
               	  case 0x1c:ha(  -8 );
				            break;                     
                  case 0x0c:ha(  -6 );
				            break;                      
                  case 0x0e:ha(  -4 );
				            break;                      
                  case 0x06:ha(  -4 );
				            break;                     
                  case 0x07:ha(   0 );
				            break;                     
                  case 0x03:ha(   2 );
				            break;                     
                  case 0x83:ha(   4 );
				            break;
				  case 0x81:ha(   6 );
				            break;
				  case 0xc1:ha(   8 );
				            break;
				  case 0xe0:
				  case 0xc0:ha(   12 );
				            break;
				    default:
                            break;
          }
          break;
		  
 case 602:                     /* ���Ԑ��ύX�O�̎ԑ̂̍��񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){ /* ���Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x2 );			  
			  ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */			  
             sp_mode(FREE,BRAKE);		   
			  sp(   57 ,   81 );
			  cnt1 = 0;
			  pattern = 622;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {

		       	  
		     	  case 0x70:
				  case 0x60:
				  case 0x30:
		          case 0x38:ha(  -15 );
				            break;                
                  case 0x18:ha(  -10 );
				            break;                    
               	  case 0x1c:ha(  -8 );
				            break;                     
                  case 0x0c:ha(  -6 );
				            break;                      
                  case 0x0e:ha(  -4 );
				            break;                      
                  case 0x06:ha(  -4 );
				            break;                     
                  case 0x07:ha(   0 );
				            break;                     
                  case 0x03:ha(   2 );
				            break;                     
                  case 0x83:ha(   4 );
				            break;
				  case 0x81:ha(   6 );
				            break;
				  case 0xc1:ha(   8 );
				            break;
				  case 0xe0:
				  case 0xc0:ha(   12 );
				            break;
				    default:
                            break;
          }
          break;
		  
 case 603:                     /* ���Ԑ��ύX�O�̎ԑ̂̍��񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){ /* ���Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x2 );			 
			  ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */
			  sp_mode(FREE,BRAKE);		  
              sp(   58 ,   81 );
			  cnt1 = 0;
			  pattern = 623;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {

		         
		         case 0x70:
				  case 0x60:
				  case 0x30:
		          case 0x38:ha(  -15 );
				            break;                
                  case 0x18:ha(  -10 );
				            break;                    
               	  case 0x1c:ha(  -8 );
				            break;                     
                  case 0x0c:ha(  -6 );
				            break;                      
                  case 0x0e:ha(  -4 );
				            break;                      
                  case 0x06:ha(  -4 );
				            break;                     
                  case 0x07:ha(   0 );
				            break;                     
                  case 0x03:ha(   2 );
				            break;                     
                  case 0x83:ha(   4 );
				            break;
				  case 0x81:ha(   6 );
				            break;
				  case 0xc1:ha(   8 );
				            break;
				  case 0xe0:
				  case 0xc0:ha(   12 );
				            break;
				    default:
                            break;
          }
          break;
		  
 case 604:                     /* ���Ԑ��ύX�O�̎ԑ̂̍��񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){ /* ���Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x2 );			  
			  	ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */			  
             sp_mode(FREE,BRAKE);		  
             sp(   59 ,   82 );
			  cnt1 = 0;
			  pattern = 624;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {

		          case 0x70:
				  case 0x60:
				  case 0x30:
		          case 0x38:ha(  -15 );
				            break;                
                  case 0x18:ha(  -10 );
				            break;                    
               	  case 0x1c:ha(  -8 );
				            break;                     
                  case 0x0c:ha(  -6 );
				            break;                      
                  case 0x0e:ha(  -4 );
				            break;                      
                  case 0x06:ha(  -4 );
				            break;                     
                  case 0x07:ha(   0 );
				            break;                     
                  case 0x03:ha(   2 );
				            break;                     
                  case 0x83:ha(   4 );
				            break;
				  case 0x81:ha(   6 );
				            break;
				  case 0xc1:ha(   8 );
				            break;
				  case 0xe0:
				  case 0xc0:ha(   12 );
				            break;
				    default:
                            break;
          }
          break;
		  
 case 605:                     /* ���Ԑ��ύX�O�̎ԑ̂̍��񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00){ /* ���Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x2 );
		  	  ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */
             sp_mode(FREE,BRAKE);		  
			  sp(   60 ,   82 );
			  cnt1 = 0;
			  pattern = 625;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {

		         
		          case 0x70:
				  case 0x60:
				  case 0x30:
		          case 0x38:ha(  -15 );
				            break;                
                  case 0x18:ha(  -10 );
				            break;                    
               	  case 0x1c:ha(  -8 );
				            break;                     
                  case 0x0c:ha(  -6 );
				            break;                      
                  case 0x0e:ha(  -4 );
				            break;                      
                  case 0x06:ha(  -4 );
				            break;                     
                  case 0x07:ha(   0 );
				            break;                     
                  case 0x03:ha(   2 );
				            break;                     
                  case 0x83:ha(   4 );
				            break;
				  case 0x81:ha(   6 );
				            break;
				  case 0xc1:ha(   8 );
				            break;
				  case 0xe0:
				  case 0xc0:ha(   12 );
				            break;
				    default:
                            break;
          }
          break;
		  
 case 606:                     /* ���Ԑ��ύX�O�̎ԑ̂̍��񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){ /* ���Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x2 );
			  ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */
             sp_mode(FREE,BRAKE);		   
			  sp(   61 ,   83 );
			  cnt1 = 0;
			  pattern = 626;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {

		    
		          case 0x70:
				  case 0x60:
				  case 0x30:
		          case 0x38:ha(  -15 );
				            break;                
                  case 0x18:ha(  -10 );
				            break;                    
               	  case 0x1c:ha(  -8 );
				            break;                     
                  case 0x0c:ha(  -6 );
				            break;                      
                  case 0x0e:ha(  -4 );
				            break;                      
                  case 0x06:ha(  -4 );
				            break;                     
                  case 0x07:ha(   0 );
				            break;                     
                  case 0x03:ha(   2 );
				            break;                     
                  case 0x83:ha(   4 );
				            break;
				  case 0x81:ha(   6 );
				            break;
				  case 0xc1:ha(   8 );
				            break;
				  case 0xe0:
				  case 0xc0:ha(   12 );
				            break;
				    default:
                            break;
          }
          break;
		  
 case 607:                     /* ���Ԑ��ύX�O�̎ԑ̂̍��񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00){ /* ���Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x2 );
			  ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */
             sp_mode(FREE,BRAKE);		  
             sp(   62 ,   83 );
			  cnt1 = 0;
			  pattern = 627;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {
			  
   				 case 0x70:
				  case 0x60:
				  case 0x30:
		          case 0x38:ha(  -15 );
				            break;                
                  case 0x18:ha(  -10 );
				            break;                    
               	  case 0x1c:ha(  -8 );
				            break;                     
                  case 0x0c:ha(  -6 );
				            break;                      
                  case 0x0e:ha(  -4 );
				            break;                      
                  case 0x06:ha(  -4 );
				            break;                     
                  case 0x07:ha(   0 );
				            break;                     
                  case 0x03:ha(   2 );
				            break;                     
                  case 0x83:ha(   4 );
				            break;
				  case 0x81:ha(   6 );
				            break;
				  case 0xc1:ha(   8 );
				            break;
				  case 0xe0:
				  case 0xc0:ha(   12 );
				            break;
				    default:
                            break;
          }
          break;
		  
 case 608:                     /* ���Ԑ��ύX�O�̎ԑ̂̍��񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){ /* ���Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x2 );		  
			  ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */
             sp_mode(FREE,BRAKE);		  
             sp(   63 ,   84 );
			  cnt1 = 0;
			  pattern = 628;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {

		         
		          case 0x70:
				  case 0x60:
				  case 0x30:
		          case 0x38:ha(  -15 );
				            break;                
                  case 0x18:ha(  -10 );
				            break;                    
               	  case 0x1c:ha(  -8 );
				            break;                     
                  case 0x0c:ha(  -6 );
				            break;                      
                  case 0x0e:ha(  -4 );
				            break;                      
                  case 0x06:ha(  -4 );
				            break;                     
                  case 0x07:ha(   0 );
				            break;                     
                  case 0x03:ha(   2 );
				            break;                     
                  case 0x83:ha(   4 );
				            break;
				  case 0x81:ha(   6 );
				            break;
				  case 0xc1:ha(   8 );
				            break;
				  case 0xe0:
				  case 0xc0:ha(   12 );
				            break;
				    default:
                            break;
          }
          break;
		  
 case 609:                     /* ���Ԑ��ύX�O�̎ԑ̂̍��񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){ /* ���Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x2 );
			  ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */
             sp_mode(FREE,BRAKE);		  
             sp(   64 ,   84 );
			  cnt1 = 0;
			  pattern = 629;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {

		          
		          case 0x70:
				  case 0x60:
				  case 0x30:
		          case 0x38:ha(  -15 );
				            break;                
                  case 0x18:ha(  -10 );
				            break;                    
               	  case 0x1c:ha(  -8 );
				            break;                     
                  case 0x0c:ha(  -6 );
				            break;                      
                  case 0x0e:ha(  -4 );
				            break;                      
                  case 0x06:ha(  -4 );
				            break;                     
                  case 0x07:ha(   0 );
				            break;                     
                  case 0x03:ha(   2 );
				            break;                     
                  case 0x83:ha(   4 );
				            break;
				  case 0x81:ha(   6 );
				            break;
				  case 0xc1:ha(   8 );
				            break;
				  case 0xe0:
				  case 0xc0:ha(   12 );
				            break;
				    default:
                            break;
			 	  }
          break;
		  
 case 610:                     /* ���Ԑ��ύX�O�̎ԑ̂̍��񂹑��s���� */
		  shasenhenkou();
		  if( cnt1 > 50 ) {
          c = sensor_inp( MASK4_4 );
          if( c == 0x00 ){ /* ���Ԑ��ύX�J�n�̔��f �� �Ԑ��ύX�̊J�n */
              kyori_flug = 0;
		      led_out( 0x2 );
			  ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */
             sp_mode(FREE,BRAKE);		  
             sp(   65 ,   85 );
			  cnt1 = 0;
			  pattern = 630;
              break;
		  }
          }
	      switch( sensor_inp(MASK4_4) ) {

		          case 0x70:
				  case 0x60:
				  case 0x30:
		          case 0x38:ha(  -15 );
				            break;                
                  case 0x18:ha(  -10 );
				            break;                    
               	  case 0x1c:ha(  -8 );
				            break;                     
                  case 0x0c:ha(  -6 );
				            break;                      
                  case 0x0e:ha(  -4 );
				            break;                      
                  case 0x06:ha(  -4 );
				            break;                     
                  case 0x07:ha(   0 );
				            break;                     
                  case 0x03:ha(   2 );
				            break;                     
                  case 0x83:ha(   4 );
				            break;
				  case 0x81:ha(   6 );
				            break;
				  case 0xc1:ha(   8 );
				            break;
				  case 0xe0:
				  case 0xc0:ha(   12 );
				            break;
				    default:
                            break;
          }
          break;
		  
 case 620: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
          
		  ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */			  
          sp(   55 ,   80 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 640;
              break;
		  }
          if( sensor_inp(MASK3_3) == ( 0x80 )||( 0x40 )||( 0xc0 )||( 0xa0 )||( 0xe0 ) ) {
              cnt1 = 0;
			  pattern = 700;
              break;
		  }
		  break;

 case 621: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
          
		  ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */			  
          sp(   56 ,   80 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 641;
              break;
		  }
          if( sensor_inp(MASK3_3) == ( 0x80 )||( 0x40 )||( 0xc0 )||( 0xa0 )||( 0xe0 ) ) {
              cnt1 = 0;
			  pattern = 701;
              break;
		  }
		  break;

 case 622: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
          
		  ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */			  
          sp(   57 ,   81 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 642;
              break;
		  }
          if( sensor_inp(MASK3_3) == ( 0x80 )||( 0x40 )||( 0xc0 )||( 0xa0 )||( 0xe0 ) ) {
              cnt1 = 0;
			  pattern = 702;
              break;
		  }
		  break;

 case 623: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
         
		  ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */			  
          sp(   58 ,   81 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 643;
              break;
		  }
          if( sensor_inp(MASK3_3) == ( 0x80 )||( 0x40 )||( 0xc0 )||( 0xa0 )||( 0xe0 ) ) {
              cnt1 = 0;
			  pattern = 703;
              break;
		  }
		  break;

 case 624: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
       
		  ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */			  	  
          sp(   59 ,   82 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 644;
              break;
		  }
          if( sensor_inp(MASK3_3) == ( 0x80 )||( 0x40 )||( 0xc0 )||( 0xa0 )||( 0xe0 ) ) {
              cnt1 = 0;
			  pattern = 704;
              break;
		  }
		  break;

 case 625: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
          
		  ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */			  
          sp(   60 ,   82 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 645;
              break;
		  }
          if( sensor_inp(MASK3_3) == ( 0x80 )||( 0x40 )||( 0xc0 )||( 0xa0 )||( 0xe0 ) ) {
              cnt1 = 0;
			  pattern = 705;
              break;
		  }
		  break;

 case 626: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
         
		  ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */			  
          sp(   61 ,   83 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 646;
              break;
		  }
          if( sensor_inp(MASK3_3) == ( 0x80 )||( 0x40 )||( 0xc0 )||( 0xa0 )||( 0xe0 ) ) {
              cnt1 = 0;
			  pattern = 706;
              break;
		  }
		  break;

 case 627: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
          
		  ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */			  
          sp(   62 ,   83 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 647;
              break;
		  }
          if( sensor_inp(MASK3_3) == ( 0x80 )||( 0x40 )||( 0xc0 )||( 0xa0 )||( 0xe0 ) ) {
              cnt1 = 0;
			  pattern = 707;
              break;
		  }
		  break;

 case 628: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
          
		  ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */			  
          sp(   63 ,   84 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 648;
              break;
		  }
          if( sensor_inp(MASK3_3) == ( 0x80 )||( 0x40 )||( 0xc0 )||( 0xa0 )||( 0xe0 ) ) {
              cnt1 = 0;
			  pattern = 708;
              break;
		  }
		  break;

 case 629: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
         
		  ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */			  
          sp(   64 ,   84 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 649;
              break;
		  }
          if( sensor_inp(MASK3_3) == ( 0x80 )||( 0x40 )||( 0xc0 )||( 0xa0 )||( 0xe0 ) ) {
              cnt1 = 0;
			  pattern = 709;
              break;
		  }
		  break;

 case 630: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
       
		  ha( -R_HF );   /* �����Ԑ��ύX���̃n���h���̋Ȃ��p�x */			  
          sp(   65 ,   85 );
		  if( cnt1 > 160 ) {
              cnt1 = 0;
			  pattern = 650;
              break;
		  }
          if( sensor_inp(MASK3_3) == ( 0x80 )||( 0x40 )||( 0xc0 )||( 0xa0 )||( 0xe0 ) ) {
              cnt1 = 0;
			  pattern = 710;
              break;
		  }
		  break;

 case 640: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
          ha(   0 );         /* ���Ԑ��ύX���̃n���h���̖߂��p�x */
          sp(   75 ,   75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x80:              
				  case 0x40:               
				  case 0xc0:               
				  case 0xa0:               
				  case 0xe0:cnt1 = 0;
                            pattern = 700;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 641: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
          ha(   0 );         /* ���Ԑ��ύX���̃n���h���̖߂��p�x */
          sp(   75 ,   75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x80:              
				  case 0x40:               
				  case 0xc0:               
				  case 0xa0:               
				  case 0xe0:cnt1 = 0;
                            pattern = 701;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 642: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
          ha(   0 );         /* ���Ԑ��ύX���̃n���h���̖߂��p�x */
          sp(   75 ,   75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x80:              
				  case 0x40:               
				  case 0xc0:               
				  case 0xa0:               
				  case 0xe0:cnt1 = 0;
                            pattern = 702;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 643: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
          ha(   0 );         /* ���Ԑ��ύX���̃n���h���̖߂��p�x */
          sp(   75 ,   75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x80:              
				  case 0x40:               
				  case 0xc0:               
				  case 0xa0:               
				  case 0xe0:cnt1 = 0;
                            pattern = 703;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 644: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
          ha(   0 );         /* ���Ԑ��ύX���̃n���h���̖߂��p�x */
          sp(   75 ,   75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x80:              
				  case 0x40:               
				  case 0xc0:               
				  case 0xa0:               
				  case 0xe0:cnt1 = 0;
                            pattern = 704;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 645: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
          ha(   0 );         /* ���Ԑ��ύX���̃n���h���̖߂��p�x */
          sp(   75 ,   75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x80:              
				  case 0x40:               
				  case 0xc0:               
				  case 0xa0:               
				  case 0xe0:cnt1 = 0;
                            pattern = 705;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 646: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
          ha(   0 );         /* ���Ԑ��ύX���̃n���h���̖߂��p�x */
          sp(   75 ,   75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x80:              
				  case 0x40:               
				  case 0xc0:               
				  case 0xa0:               
				  case 0xe0:cnt1 = 0;
                            pattern = 706;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 647: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
          ha(   0 );         /* ���Ԑ��ύX���̃n���h���̖߂��p�x */
          sp(   75 ,   75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x80:              
				  case 0x40:               
				  case 0xc0:               
				  case 0xa0:               
				  case 0xe0:cnt1 = 0;
                            pattern = 707;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 648: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
          ha(   0 );         /* ���Ԑ��ύX���̃n���h���̖߂��p�x */
          sp(   75 ,   75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x80:              
				  case 0x40:               
				  case 0xc0:               
				  case 0xa0:               
				  case 0xe0:cnt1 = 0;
                            pattern = 708;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 649: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
          ha(   0 );         /* ���Ԑ��ύX���̃n���h���̖߂��p�x */
          sp(   75 ,   75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x80:              
				  case 0x40:               
				  case 0xc0:               
				  case 0xa0:               
				  case 0xe0:cnt1 = 0;
                            pattern = 709;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 650: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX���̃��[�^�ݒ� */
          ha(   0 );         /* ���Ԑ��ύX���̃n���h���̖߂��p�x */
          sp(   75 ,   75 );
		  switch( sensor_inp(MASK3_3) ) {

		          case 0x80:              
				  case 0x40:               
				  case 0xc0:               
				  case 0xa0:               
				  case 0xe0:cnt1 = 0;
                            pattern = 710;
                            break;
                    default:
                            break;
          }
          break;
		  
 case 700: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x38:ha(   1 );
				            sp(   75 ,   74 );
		                    break;
				  case 0x18:ha(   2 );
				            sp(   74 ,   72 );
		                    break;
				  case 0x1c:ha(   4 );
                            sp(   73 ,   69 );
		                    break;
				  case 0x0c:ha(   8 );
                            sp(   72 ,   65 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   71 ,   60 );
		                    cnt1 = 0;
                            pattern = 720;
                            break;
				  case 0x06:ha(  16 );
				            sp(   70 ,   55 );
		                    cnt1 = 0;
                            pattern = 720;
                            break;
		            default:
                            break;
          }
          break;
  
 case 701: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x38:ha(   1 );
				            sp(   76 ,   75 );
		                    break;
				  case 0x18:ha(   2 );
				            sp(   75 ,   73 );
		                    break;
				  case 0x1c:ha(   4 );
                            sp(   74 ,   70 );
		                    break;
				  case 0x0c:ha(   8 );
                            sp(   73 ,   66 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   72 ,   61 );
		                    cnt1 = 0;
                            pattern = 721;
                            break;
				  case 0x06:ha(  16 );
				            sp(   71 ,   56 );
		                    cnt1 = 0;
                            pattern = 721;
                            break;
		            default:
                            break;
          }
          break;
  
 case 702: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x38:ha(   1 );
				            sp(   77 ,   76 );
		                    break;
				  case 0x18:ha(   2 );
				            sp(   76 ,   74 );
		                    break;
				  case 0x1c:ha(   4 );
                            sp(   75 ,   71 );
		                    break;
				  case 0x0c:ha(   8 );
                            sp(   74 ,   67 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   73 ,   62 );
		                    cnt1 = 0;
                            pattern = 722;
                            break;
				  case 0x06:ha(  16 );
				            sp(   72 ,   57 );
		                    cnt1 = 0;
                            pattern = 722;
                            break;
		            default:
                            break;
          }
          break;
  
 case 703: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x38:ha(   1 );
				            sp(   78 ,   77 );
		                    break;
				  case 0x18:ha(   2 );
				            sp(   77 ,   75 );
		                    break;
				  case 0x1c:ha(   4 );
                            sp(   76 ,   72 );
		                    break;
				  case 0x0c:ha(   8 );
                            sp(   75 ,   68 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   74 ,   63 );
		                    cnt1 = 0;
                            pattern = 723;
                            break;
				  case 0x06:ha(  16 );
				            sp(   73 ,   58 );
		                    cnt1 = 0;
                            pattern = 723;
                            break;
		            default:
                            break;
          }
          break;
  
 case 704: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x38:ha(   1 );
				            sp(   79 ,   78 );
		                    break;
				  case 0x18:ha(   2 );
				            sp(   78 ,   76 );
		                    break;
				  case 0x1c:ha(   4 );
                            sp(   77 ,   73 );
		                    break;
				  case 0x0c:ha(   8 );
                            sp(   76 ,   69 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   75 ,   64 );
		                    cnt1 = 0;
                            pattern = 724;
                            break;
				  case 0x06:ha(  16 );
				            sp(   74 ,   59 );
		                    cnt1 = 0;
                            pattern = 724;
                            break;
		            default:
                            break;
          }
          break;
  
 case 705: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x38:ha(   1 );
				            sp(   80 ,   79 );
		                    break;
				  case 0x18:ha(   2 );
				            sp(   79 ,   77 );
		                    break;
				  case 0x1c:ha(   4 );
                            sp(   78 ,   74 );
		                    break;
				  case 0x0c:ha(   8 );
                            sp(   77 ,   70 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   76 ,   65 );
		                    cnt1 = 0;
                            pattern = 725;
                            break;
				  case 0x06:ha(  16 );
				            sp(   75 ,   60 );
		                    cnt1 = 0;
                            pattern = 725;
                            break;
		            default:
                            break;
          }
          break;
  
 case 706: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x38:ha(   1 );
				            sp(   81 ,   80 );
		                    break;
				  case 0x18:ha(   2 );
				            sp(   80 ,   78 );
		                    break;
				  case 0x1c:ha(   4 );
                            sp(   79 ,   75 );
		                    break;
				  case 0x0c:ha(   8 );
                            sp(   78 ,   71 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   77 ,   66 );
		                    cnt1 = 0;
                            pattern = 726;
                            break;
				  case 0x06:ha(  16 );
				            sp(   76 ,   61 );
		                    cnt1 = 0;
                            pattern = 726;
                            break;
		            default:
                            break;
          }
          break;
  
 case 707: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x38:ha(   1 );
				            sp(   82 ,   81 );
		                    break;
				  case 0x18:ha(   2 );
				            sp(   81 ,   79 );
		                    break;
				  case 0x1c:ha(   4 );
                            sp(   80 ,   76 );
		                    break;
				  case 0x0c:ha(   8 );
                            sp(   79 ,   72 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   78 ,   67 );
		                    cnt1 = 0;
                            pattern = 727;
                            break;
				  case 0x06:ha(  16 );
				            sp(   77 ,   62 );
		                    cnt1 = 0;
                            pattern = 727;
                            break;
		            default:
                            break;
          }
          break;
  
 case 708: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x38:ha(   1 );
				            sp(   83 ,   82 );
		                    break;
				  case 0x18:ha(   2 );
				            sp(   82 ,   80 );
		                    break;
				  case 0x1c:ha(   4 );
                            sp(   81 ,   77 );
		                    break;
				  case 0x0c:ha(   8 );
                            sp(   80 ,   73 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   79 ,   68 );
		                    cnt1 = 0;
                            pattern = 728;
                            break;
				  case 0x06:ha(  16 );
				            sp(   78 ,   63 );
		                    cnt1 = 0;
                            pattern = 728;
                            break;
		            default:
                            break;
          }
          break;
  
 case 709: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x38:ha(   1 );
				            sp(   84 ,   83 );
		                    break;
				  case 0x18:ha(   2 );
				            sp(   83 ,   81 );
		                    break;
				  case 0x1c:ha(   4 );
                            sp(   82 ,   78 );
		                    break;
				  case 0x0c:ha(   8 );
                            sp(   81 ,   74 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   80 ,   69 );
		                    cnt1 = 0;
                            pattern = 729;
                            break;
				  case 0x06:ha(  16 );
				            sp(   79 ,   64 );
		                    cnt1 = 0;
                            pattern = 729;
                            break;
		            default:
                            break;
          }
          break;
  
 case 710: /* ���Ԑ��ύX���C���̒ʉߎ��Ԃɂ��Ԑ��ύX�̏I���ڍs���� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x38:ha(   1 );
				            sp(   85 ,   84 );
		                    break;
				  case 0x18:ha(   2 );
				            sp(   84 ,   82 );
		                    break;
				  case 0x1c:ha(   4 );
                            sp(   83 ,   79 );
		                    break;
				  case 0x0c:ha(   8 );
                            sp(   82 ,   75 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   81 ,   70 );
		                    cnt1 = 0;
                            pattern = 730;
                            break;
				  case 0x06:ha(  16 );
				            sp(   80 ,   65 );
		                    cnt1 = 0;
                            pattern = 730;
                            break;
		            default:
                            break;
          }
          break;
  
 case 720:             /* ���Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		          case 0x38:          
                  case 0x18:
		          case 0x1c:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x0c:ha(   8 );
				            sp(   72 ,   65 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   71 ,   60 );
		                    break;
				  case 0x06:ha(  16 );
				            sp(   70 ,   55 );
		                    break;
		          case 0x07:ha(  20 );
				            sp(   65 ,   50 );
		                    break;
		          case 0x03:ha(  24 );
				            sp(   60 ,   40 );
		                    cnt1 = 0;
                            pattern = 740;
                            break;
		            default:
                            break;
          }
          break;

 case 721:             /* ���Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		          case 0x38:          
                  case 0x18:
		          case 0x1c:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x0c:ha(   8 );
				            sp(   73 ,   66 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   72 ,   61 );
		                    break;
				  case 0x06:ha(  16 );
				            sp(   71 ,   56 );
		                    break;
		          case 0x07:ha(  20 );
				            sp(   66 ,   51 );
		                    break;
		          case 0x03:ha(  24 );
				            sp(   61 ,   41 );
		                    cnt1 = 0;
                            pattern = 741;
                            break;
		            default:
                            break;
          }
          break;

 case 722:             /* ���Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		          case 0x38:          
                  case 0x18:
		          case 0x1c:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x0c:ha(   8 );
				            sp(   74 ,   67 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   73 ,   62 );
		                    break;
				  case 0x06:ha(  16 );
				            sp(   72 ,   57 );
		                    break;
		          case 0x07:ha(  20 );
				            sp(   67 ,   52 );
		                    break;
		          case 0x03:ha(  24 );
				            sp(   62 ,   42 );
		                    cnt1 = 0;
                            pattern = 742;
                            break;
		            default:
                            break;
          }
          break;

 case 723:             /* ���Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		          case 0x38:          
                  case 0x18:
		          case 0x1c:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x0c:ha(   8 );
				            sp(   75 ,   68 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   74 ,   63 );
		                    break;
				  case 0x06:ha(  16 );
				            sp(   73 ,   58 );
		                    break;
		          case 0x07:ha(  20 );
				            sp(   68 ,   53 );
		                    break;
		          case 0x03:ha(  24 );
				            sp(   63 ,   43 );
		                    cnt1 = 0;
                            pattern = 743;
                            break;
		            default:
                            break;
          }
          break;

 case 724:             /* ���Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		          case 0x38:          
                  case 0x18:
		          case 0x1c:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x0c:ha(   8 );
				            sp(   76 ,   69 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   75 ,   64 );
		                    break;
				  case 0x06:ha(  16 );
				            sp(   74 ,   59 );
		                    break;
		          case 0x07:ha(  20 );
				            sp(   69 ,   54 );
		                    break;
		          case 0x03:ha(  24 );
				            sp(   64 ,   44 );
		                    cnt1 = 0;
                            pattern = 744;
                            break;
		            default:
                            break;
          }
          break;

 case 725:             /* ���Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		          case 0x38:          
                  case 0x18:
		          case 0x1c:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x0c:ha(   8 );
				            sp(   77 ,   70 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   76 ,   65 );
		                    break;
				  case 0x06:ha(  16 );
				            sp(   75 ,   60 );
		                    break;
		          case 0x07:ha(  20 );
				            sp(   70 ,   55 );
		                    break;
		          case 0x03:ha(  24 );
				            sp(   65 ,   45 );
		                    cnt1 = 0;
                            pattern = 745;
                            break;
		            default:
                            break;
          }
          break;

 case 726:             /* ���Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		          case 0x38:          
                  case 0x18:
		          case 0x1c:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x0c:ha(   8 );
				            sp(   78 ,   71 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   77 ,   66 );
		                    break;
				  case 0x06:ha(  16 );
				            sp(   76 ,   61 );
		                    break;
		          case 0x07:ha(  20 );
				            sp(   71 ,   56 );
		                    break;
		          case 0x03:ha(  24 );
				            sp(   66 ,   46 );
		                    cnt1 = 0;
                            pattern = 746;
                            break;
		            default:
                            break;
          }
          break;

 case 727:             /* ���Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		          case 0x38:          
                  case 0x18:
		          case 0x1c:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x0c:ha(   8 );
				            sp(   79 ,   72 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   78 ,   67 );
		                    break;
				  case 0x06:ha(  16 );
				            sp(   77 ,   62 );
		                    break;
		          case 0x07:ha(  20 );
				            sp(   72 ,   57 );
		                    break;
		          case 0x03:ha(  24 );
				            sp(   67 ,   47 );
		                    cnt1 = 0;
                            pattern = 747;
                            break;
		            default:
                            break;
          }
          break;

 case 728:             /* ���Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		          case 0x38:          
                  case 0x18:
		          case 0x1c:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x0c:ha(   8 );
				            sp(   80 ,   73 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   79 ,   68 );
		                    break;
				  case 0x06:ha(  16 );
				            sp(   78 ,   63 );
		                    break;
		          case 0x07:ha(  20 );
				            sp(   73 ,   58 );
		                    break;
		          case 0x03:ha(  24 );
				            sp(   68 ,   48 );
		                    cnt1 = 0;
                            pattern = 748;
                            break;
		            default:
                            break;
          }
          break;

 case 729:             /* ���Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		          case 0x38:          
                  case 0x18:
		          case 0x1c:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x0c:ha(   8 );
				            sp(   81 ,   74 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   80 ,   69 );
		                    break;
				  case 0x06:ha(  16 );
				            sp(   79 ,   64 );
		                    break;
		          case 0x07:ha(  20 );
				            sp(   74 ,   59 );
		                    break;
		          case 0x03:ha(  24 );
				            sp(   69 ,   49 );
		                    cnt1 = 0;
                            pattern = 749;
                            break;
		            default:
                            break;
          }
          break;

 case 730:             /* ���Ԑ��ύX�̏I�������ƃR�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x3c:
		          case 0x38:          
                  case 0x18:
		          case 0x1c:led_out( 0x0 );
		                    cnt1 = 0;
                            pattern = 20;
                            break;
			      case 0x0c:ha(   8 );
				            sp(   82 ,   75 );
		                    break;
				  case 0x0e:ha(  12 );
				            sp(   81 ,   70 );
		                    break;
				  case 0x06:ha(  16 );
				            sp(   80 ,   65 );
		                    break;
		          case 0x07:ha(  20 );
				            sp(   75 ,   60 );
		                    break;
		          case 0x03:ha(  24 );
				            sp(   70 ,   50 );
		                    cnt1 = 0;
                            pattern = 750;
                            break;
		            default:
                            break;
          }
          break;

 case 740: /* ���Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		          case 0x07:cnt1 = 0;
                            pattern = 720;
                            break;	  
                  case 0x03:ha(  24 );
				            sp(   60 ,   40 );
		                    break;
				  case 0x83:ha(  27 );
				            sp(   55 ,   30 );
		                    break;
		          case 0x81:ha(  30 );
				            sp(   50 ,   25 );
		                    break;
		          case 0xc1:ha(  33 );
				            sp(   45 ,   20 );
		                    break;
		          case 0xc0:ha(  36 );
				            sp(   35 ,   15 );
		                    break;
		          case 0xe0:ha(  39 );
				            sp(   25 ,   10 );
		                    break;
		          case 0x60:ha(  42 );
				            sp(   15 ,    5 );
		                    break;
		            default:
                            break;
          }
          break;

 case 741: /* ���Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		          case 0x07:cnt1 = 0;
                            pattern = 721;
                            break;	  
                  case 0x03:ha(  24 );
				            sp(   61 ,   41 );
		                    break;
				  case 0x83:ha(  27 );
				            sp(   56 ,   31 );
		                    break;
		          case 0x81:ha(  30 );
				            sp(   51 ,   26 );
		                    break;
		          case 0xc1:ha(  33 );
				            sp(   46 ,   21 );
		                    break;
		          case 0xc0:ha(  36 );
				            sp(   36 ,   16 );
		                    break;
		          case 0xe0:ha(  39 );
				            sp(   26 ,   11 );
		                    break;
		          case 0x60:ha(  42 );
				            sp(   16 ,    6 );
		                    break;
		            default:
                            break;
          }
          break;

 case 742: /* ���Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		          case 0x07:cnt1 = 0;
                            pattern = 722;
                            break;	  
                  case 0x03:ha(  24 );
				            sp(   62 ,   42 );
		                    break;
				  case 0x83:ha(  27 );
				            sp(   57 ,   32 );
		                    break;
		          case 0x81:ha(  30 );
				            sp(   52 ,   27 );
		                    break;
		          case 0xc1:ha(  33 );
				            sp(   47 ,   22 );
		                    break;
		          case 0xc0:ha(  36 );
				            sp(   37 ,   17 );
		                    break;
		          case 0xe0:ha(  39 );
				            sp(   27 ,   12 );
		                    break;
		          case 0x60:ha(  42 );
				            sp(   17 ,    7 );
		                    break;
		            default:
                            break;
          }
          break;

 case 743: /* ���Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		          case 0x07:cnt1 = 0;
                            pattern = 723;
                            break;	  
                  case 0x03:ha(  24 );
				            sp(   63 ,   43 );
		                    break;
				  case 0x83:ha(  27 );
				            sp(   58 ,   33 );
		                    break;
		          case 0x81:ha(  30 );
				            sp(   53 ,   28 );
		                    break;
		          case 0xc1:ha(  33 );
				            sp(   48 ,   23 );
		                    break;
		          case 0xc0:ha(  36 );
				            sp(   38 ,   18 );
		                    break;
		          case 0xe0:ha(  39 );
				            sp(   28 ,   13 );
		                    break;
		          case 0x60:ha(  42 );
				            sp(   18 ,    8 );
		                    break;
		            default:
                            break;
          }
          break;

 case 744: /* ���Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		          case 0x07:cnt1 = 0;
                            pattern = 724;
                            break;	  
                  case 0x03:ha(  24 );
				            sp(   64 ,   44 );
		                    break;
				  case 0x83:ha(  27 );
				            sp(   59 ,   34 );
		                    break;
		          case 0x81:ha(  30 );
				            sp(   54 ,   29 );
		                    break;
		          case 0xc1:ha(  33 );
				            sp(   49 ,   24 );
		                    break;
		          case 0xc0:ha(  36 );
				            sp(   39 ,   19 );
		                    break;
		          case 0xe0:ha(  39 );
				            sp(   29 ,   14 );
		                    break;
		          case 0x60:ha(  42 );
				            sp(   19 ,    9 );
		                    break;
		            default:
                            break;
          }
          break;

 case 745: /* ���Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		          case 0x07:cnt1 = 0;
                            pattern = 725;
                            break;	  
                  case 0x03:ha(  24 );
				            sp(   65 ,   45 );
		                    break;
				  case 0x83:ha(  27 );
				            sp(   60 ,   35 );
		                    break;
		          case 0x81:ha(  30 );
				            sp(   55 ,   30 );
		                    break;
		          case 0xc1:ha(  33 );
				            sp(   50 ,   25 );
		                    break;
		          case 0xc0:ha(  36 );
				            sp(   40 ,   20 );
		                    break;
		          case 0xe0:ha(  39 );
				            sp(   30 ,   15 );
		                    break;
		          case 0x60:ha(  42 );
				            sp(   20 ,   10 );
		                    break;
		            default:
                            break;
          }
          break;

 case 746: /* ���Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		          case 0x07:cnt1 = 0;
                            pattern = 726;
                            break;	  
                  case 0x03:ha(  24 );
				            sp(   66 ,   46 );
		                    break;
				  case 0x83:ha(  27 );
				            sp(   61 ,   36 );
		                    break;
		          case 0x81:ha(  30 );
				            sp(   56 ,   31 );
		                    break;
		          case 0xc1:ha(  33 );
				            sp(   51 ,   26 );
		                    break;
		          case 0xc0:ha(  36 );
				            sp(   41 ,   21 );
		                    break;
		          case 0xe0:ha(  39 );
				            sp(   31 ,   16 );
		                    break;
		          case 0x60:ha(  42 );
				            sp(   21 ,   11 );
		                    break;
		            default:
                            break;
          }
          break;

 case 747: /* ���Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		          case 0x07:cnt1 = 0;
                            pattern = 727;
                            break;	  
                  case 0x03:ha(  24 );
				            sp(   67 ,   47 );
		                    break;
				  case 0x83:ha(  27 );
				            sp(   62 ,   37 );
		                    break;
		          case 0x81:ha(  30 );
				            sp(   57 ,   32 );
		                    break;
		          case 0xc1:ha(  33 );
				            sp(   52 ,   27 );
		                    break;
		          case 0xc0:ha(  36 );
				            sp(   42 ,   22 );
		                    break;
		          case 0xe0:ha(  39 );
				            sp(   32 ,   17 );
		                    break;
		          case 0x60:ha(  42 );
				            sp(   22 ,   12 );
		                    break;
		            default:
                            break;
          }
          break;

 case 748: /* ���Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		          case 0x07:cnt1 = 0;
                            pattern = 728;
                            break;	  
                  case 0x03:ha(  24 );
				            sp(   68 ,   48 );
		                    break;
				  case 0x83:ha(  27 );
				            sp(   63 ,   38 );
		                    break;
		          case 0x81:ha(  30 );
				            sp(   58 ,   33 );
		                    break;
		          case 0xc1:ha(  33 );
				            sp(   53 ,   28 );
		                    break;
		          case 0xc0:ha(  36 );
				            sp(   43 ,   23 );
		                    break;
		          case 0xe0:ha(  39 );
				            sp(   33 ,   18 );
		                    break;
		          case 0x60:ha(  42 );
				            sp(   23 ,   13 );
		                    break;
		            default:
                            break;
          }
          break;

 case 749: /* ���Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		          case 0x07:cnt1 = 0;
                            pattern = 729;
                            break;	  
                  case 0x03:ha(  24 );
				            sp(   69 ,   49 );
		                    break;
				  case 0x83:ha(  27 );
				            sp(   64 ,   39 );
		                    break;
		          case 0x81:ha(  30 );
				            sp(   59 ,   34 );
		                    break;
		          case 0xc1:ha(  33 );
				            sp(   54 ,   29 );
		                    break;
		          case 0xc0:ha(  36 );
				            sp(   44 ,   24 );
		                    break;
		          case 0xe0:ha(  39 );
				            sp(   34 ,   19 );
		                    break;
		          case 0x60:ha(  42 );
				            sp(   24 ,   14 );
		                    break;
		            default:
                            break;
          }
          break;

 case 750: /* ���Ԑ��ύX�I�����ɊO�֖c��񂾎��̃R�[�X�A�E�g������� */
          switch( sensor_inp(MASK4_4) ) {

                  case 0x06:
		          case 0x07:cnt1 = 0;
                            pattern = 730;
                            break;	  
                  case 0x03:ha(  24 );
				            sp(   70 ,   50 );
		                    break;
				  case 0x83:ha(  27 );
				            sp(   65 ,   40 );
		                    break;
		          case 0x81:ha(  30 );
				            sp(   60 ,   35 );
		                    break;
		          case 0xc1:ha(  33 );
				            sp(   55 ,   30 );
		                    break;
		          case 0xc0:ha(  36 );
				            sp(   45 ,   25 );
		                    break;
		          case 0xe0:ha(  39 );
				            sp(   35 ,   20 );
		                    break;
		          case 0x60:ha(  42 );
				            sp(   25 ,   15 );
		                    break;
		            default:
                            break;
          }
          break;
 case 999:
 		/* �R�[�X�A�E�g�ɂ��ً}��~ */
		   sp_mode( FREE, FREE );
		   if(dipsw_get() & 0x08 ){
	      		setBeepPattern(0xe0cc);
		   }
		   sp(    0 ,    0 );
		   pattern = 1000;
		   cnt1 = 0;
           break;
 case 1000:                             /* �R�[�X�A�E�g�ɂ��ً}��~ */
 		  sp(    0 ,    0 ); 
          if( cnt1 <  35 ) {            /* LED�̍����_�ŏ����@�@     */
              led_out( 0x0 );
			  rgb(R);
          } else if( cnt1 < 75 ) {
              led_out( 0x3 );
			  rgb(OFF);
          } else {
              cnt1 = 0;
          }
		  break;
		  

    
  default:
                                 /* �ǂ�ł��Ȃ��ꍇ�͑ҋ@��Ԃɖ߂� */
          pattern = 0;
          break;
          }
  }
}

/************************************************************************/
/* H8/3048F-ONE �������Ӌ@�\�@������                                    */
/************************************************************************/
void init( void )
{
    /* I/O�|�[�g�̓��o�͐ݒ� */
    P1DDR = 0xff;
    P2DDR = 0xff;
    P3DDR = 0xff;
    P4DDR = 0xff;
    P5DDR = 0xff;
    P6DDR = 0xf0;                       /* CPU����DIP SW        */
    P8DDR = 0xff;
    P9DDR = 0xf7;                       /* �ʐM�|�[�g               */
    PADDR = 0xff;
    PBDR  = 0xc0;
    PBDDR = 0xfe;                       /* ���[�^�h���C�u���Vol.3  */
    /* ���Z���T���P7�́A���͐�p�Ȃ̂œ��o�͐ݒ�͂���܂���     */

    /* ITU0 1ms���Ƃ̊��荞�� */
    ITU0_TCR = 0x23;
    ITU0_GRA = TIMER_CYCLE;
    ITU0_IER = 0x01;

    /* ITU3,4 ���Z�b�g����PWM���[�h ���E���[�^�A�T�[�{�p */
    ITU3_TCR = 0x23;
    ITU_FCR  = 0x3e;
    ITU3_GRA = PWM_CYCLE;               /* �����̐ݒ�               */
    ITU3_GRB = ITU3_BRB = 0;            /* �����[�^��PWM�ݒ�        */
    ITU4_GRA = ITU4_BRA = 0;            /* �E���[�^��PWM�ݒ�        */
    ITU4_GRB = ITU4_BRB = SERVO_CENTER; /* �T�[�{��PWM�ݒ�          */
    ITU_TOER = 0x38;

    /* ITU�̃J�E���g�X�^�[�g */
    ITU_STR = 0x09;
}

/************************************************************************/
/* ITU0 ���荞�ݏ���                                                    */
/************************************************************************/
#pragma interrupt( interrupt_timer0 )
void interrupt_timer0( void )
{
    ITU0_TSR &= 0xfe;                   /* �t���O�N���A             */
	
	cnt0++;
    cnt1++;
	cnt_run++;
	check_sensor_cnt++;
	beepProcess();
}

/************************************************************************/
/* �^�C�}�{��                                                           */
/* �����@ �^�C�}�l 1=1ms                                                */
/************************************************************************/
void timer( unsigned long timer_set )
{
    cnt0 = 0;
    while( cnt0 < timer_set );
}

/************************************************************************/
/* �T�[�{�n���h������                                                   */
/* �����@ �T�[�{����p�x�F-90�`90                                       */
/*        -90�ō���90�x�A0�ł܂������A90�ŉE��90�x��]                  */
/************************************************************************/
void ha( int angle )
{
	angle_buff = angle;
	/* �T�[�{�����E�t�ɓ����ꍇ�́A�u-�v���u+�v�ɑւ��Ă������� */
    ITU4_BRB = SERVO_CENTER - angle * HANDLE_STEP;
}

/************************************************************************/
/* �T�[�{�n���h������                                                   */
/* �����@ �T�[�{����p�x�F-90�`90                                       */
/*        -90�ō���90�x�A0�ł܂������A90�ŉE��90�x��]                  */
/************************************************************************/
void ha_free( void )
{
	ITU4_BRB = 0;
}

/************************************************************************/
/* �Z���T��Ԍ��o                                                       */
/* �����@ �}�X�N�l                                                      */
/* �߂�l �Z���T�l                                                      */
/************************************************************************/
unsigned char sensor_inp( unsigned char mask )
{
    unsigned char sensor;

    sensor  = ~P7DR;

    sensor &= mask;

    return sensor;
}


/************************************************************************/
/* �N���X���C�����o����                                                 */
/* �߂�l 0:�N���X���C���Ȃ� 1:����                                     */
/************************************************************************/
int check_crossline( void )
{
    unsigned char b;
    int ret;

    ret = 0;
    b = sensor_inp( MASK4_4 );
 
	if( b==0xff || b==0xfe || b==0xf7 || b==0xfd || b==0xfb ||
        b==0xdf || b==0xde || b==0xd7 || b==0xdd || b==0xdb ||
        b==0xef || b==0xee || b==0xe7 || b==0xed || b==0xeb ||
        b==0xbf || b==0xbe || b==0xb7 || b==0xbd || b==0xbb ||
        b==0x7f || b==0x7b ) {
        ret = 1;
		
    }
	
    return ret;
}

/************************************************************************/
/* �E�n�[�t���C�����o����                                               */
/* �߂�l 0:�Ȃ� 1:����                                                 */
/************************************************************************/
int check_shasenhenkoum( void )
{
    unsigned char m;
    int ret;

    ret = 0;
    m = sensor_inp( MASK4_4 );
    if( m==0x0f || m==0x1f || m==0x1d || m==0x1b || m==0x3f || m==0x3d || m==0x3b || m==0x1e) {
        ret = 1;
    }
    return ret;
}

/************************************************************************/
/* �E�n�[�t���C�����o����                                               */
/* �߂�l 0:�Ȃ� 1:����                                                 */
/************************************************************************/
int migikakunin( void )
{
    unsigned char m;
    int ret;

    ret = 0;
    m = sensor_inp( MASK3_0 );
    if( m==0xe0 || m==0xc0 || m==0x60 || m==0xa0 || m==0x80 || m==0x40 ) {
        ret = 1;
    }
    return ret;
}

/************************************************************************/
/* ���n�[�t���C�����o����                                               */
/* �߂�l 0:�Ȃ� 1:����                                                 */
/************************************************************************/
int check_shasenhenkouh( void )
{
    unsigned char h;
    int ret;

    ret = 0;
    h = sensor_inp( MASK4_4 );
    if( h==0xf0 ||h==0xf8 || h==0xb8 || h==0xd8 || h==0xfc || h==0xbc || h==0xdc || h==0x78) {
        ret = 1;
    }
    return ret;
}

/************************************************************************/
/* ���n�[�t���C�����o����                                               */
/* �߂�l 0:�Ȃ� 1:����                                                 */
/************************************************************************/
int hidarikakunin( void )
{
    unsigned char h;
    int ret;

    ret = 0;

    h = sensor_inp( MASK0_3 );
    if( h==0x07 || h==0x03 || h==0x06 || h==0x05 || h==0x01 || h==0x02 ) {
        ret = 1;
    }
    return ret;
}

/************************************************************************/
/* �f�B�b�v�X�C�b�`�l�ǂݍ���                                           */
/* �߂�l �X�C�b�`�l 0�`15                                              */
/************************************************************************/
unsigned char dipsw_get( void )
{
    unsigned char sw;

    sw  = ~P6DR;                        /* �f�B�b�v�X�C�b�`�ǂݍ��� */
    sw &= 0x0f;
	sw = 0x0f-sw;

    return  sw;
}

/************************************************************************/
/* �v�b�V���X�C�b�`�l�ǂݍ���                                           */
/* �߂�l �X�C�b�`�l ON:1 OFF:0                                         */
/************************************************************************/
unsigned char pushsw_get( void )
{
    unsigned char sw;

    sw  = ~PBDR;                        /* �X�C�b�`�̂���|�[�g�ǂݍ��� */
    sw &= 0x01;

    return  sw;
}

/************************************************************************/
/* �X�^�[�g�o�[���o�Z���T�ǂݍ���                                       */
/* �߂�l �Z���T�l ON(�o�[����):1 OFF(�Ȃ�):0                           */
/************************************************************************/
unsigned char startbar_get( void )
{
    unsigned char b;

    b  = ~P7DR;                        /* �X�^�[�g�o�[�M���ǂݍ���  */
    b &= 0x80;
    b >>= 4;

    return  b;
}

/************************************************************************/
/* LED����                                                              */
/* �����@�X�C�b�`�l LED0:bit0 LED1:bit1  "0":���� "1":�_��              */
/* ��)0x3��LED1:ON LED0:ON  0x2��LED1:ON LED0:OFF                       */
/************************************************************************/
void led_out( unsigned char led )
{
    unsigned char data;

    led = ~led;
    led <<= 6;
    data = PBDR & 0x3f;
    PBDR = data | led;
}

/************************************************************************/
/* ��ւ̑��x����2 �f�B�b�v�X�C�b�`�͊֌W�Ȃ�                           */
/* �����@ �����[�^:-100�`100 , �E���[�^:-100�`100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/************************************************************************/
void sp( int accele_l, int accele_r )
{
    unsigned long   speed_max;

    speed_max = PWM_CYCLE - 1;
	
	if( cnt_run > (RUNNING_TIME*1000)){
		accele_l = 0;
		accele_r = 0;	
	}
	accele_l = -accele_l;
	
	
	/* �����[�^ */
    if( accele_l >= 0 ) {
        PBDR &= 0xfb;
        ITU3_BRB = speed_max * accele_l / 100;
    } else {
        PBDR |= 0x04;
        accele_l = -accele_l;
        ITU3_BRB = speed_max * accele_l / 100;
    }

    /* �E���[�^ */
    if( accele_r >= 0 ) {
        PBDR &= 0xf7;
        ITU4_BRA = speed_max * accele_r / 100;
    } else {
        PBDR |= 0x08;
        accele_r = -accele_r;
        ITU4_BRA = speed_max * accele_r / 100;
    }

}

/************************************************************************/
/* ���[�^��~����i�u���[�L�A�t���[�j                                   */
/* �����@ �����[�^:FREE or BRAKE , �E���[�^:FREE or BRAKE               */
/************************************************************************/
void sp_mode( int mode_l, int mode_r )
{
    if( mode_l ) {
        PADR |= 0x01;
    } else {
        PADR &= 0xfe;
    }
    if( mode_r ) {
        PADR |= 0x02;
    } else {
        PADR &= 0xfd;
    }
}



/*********************************************************************/
/* �e�����̃N�����N�����̏W��								         */
/* �����p�^�[���ɂ��Ă͐擪�̃O���[�o���ϐ��̐錾�ɂčs��          */
/*********************************************************************/
void newcrank( void )
{
	sp_mode(BRAKE , BRAKE);
         if( kyoritime <=   5 ){     /* �N���X���C���ʉߌ�̌������� */
         if(cnt1 <  50 ){sp(  -50 ,  -50 );}
    else if(cnt1 <  75 ){sp(  -35 ,  -35 );}
    else if(cnt1 < 100 ){sp(  -30 ,  -30 );}
    else if(cnt1 < 125 ){sp(  -25 ,  -25 );}
    else if(cnt1 < 150 ){sp(  -20 ,  -20 );}
    else if(cnt1 < 175 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 200 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 225 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 250 ){sp(    0 ,    0 );}
    else if(cnt1 < 275 ){sp(   10 ,   10 );}
    else if(cnt1 < 300 ){sp(   20 ,   20 );}
    else if(cnt1 < 325 ){sp(   30 ,   30 );}
    else if(cnt1 < 350 ){sp(   40 ,   40 );}
    else if(cnt1 < 375 ){sp(   50 ,   50 );}
    else if(cnt1 < 400 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=   6 ){     /* �N���X���C���ʉߌ�̌������� */
         if(cnt1 <  40 ){sp(  -50 ,  -50 );}
    else if(cnt1 <  65 ){sp(  -35 ,  -35 );}
    else if(cnt1 <  90 ){sp(  -30 ,  -30 );}
    else if(cnt1 < 115 ){sp(  -25 ,  -25 );}
    else if(cnt1 < 140 ){sp(  -20 ,  -20 );}
    else if(cnt1 < 165 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 190 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 215 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 240 ){sp(    0 ,    0 );}
    else if(cnt1 < 265 ){sp(   10 ,   10 );}
    else if(cnt1 < 290 ){sp(   20 ,   20 );}
    else if(cnt1 < 315 ){sp(   30 ,   30 );}
    else if(cnt1 < 340 ){sp(   40 ,   40 );}
    else if(cnt1 < 365 ){sp(   50 ,   50 );}
    else if(cnt1 < 390 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=   7 ){     /* �N���X���C���ʉߌ�̌������� */
         if(cnt1 <  30 ){sp(  -50 ,  -50 );}
    else if(cnt1 <  55 ){sp(  -35 ,  -35 );}
    else if(cnt1 <  80 ){sp(  -30 ,  -30 );}
    else if(cnt1 < 105 ){sp(  -25 ,  -25 );}
    else if(cnt1 < 130 ){sp(  -20 ,  -20 );}
    else if(cnt1 < 155 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 180 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 205 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 230 ){sp(    0 ,    0 );}
    else if(cnt1 < 255 ){sp(   10 ,   10 );}
    else if(cnt1 < 280 ){sp(   20 ,   20 );}
    else if(cnt1 < 305 ){sp(   30 ,   30 );}
    else if(cnt1 < 330 ){sp(   40 ,   40 );}
    else if(cnt1 < 355 ){sp(   50 ,   50 );}
    else if(cnt1 < 380 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=   8 ){     /* �N���X���C���ʉߌ�̌������� */
         if(cnt1 <  20 ){sp(  -50 ,  -50 );}
    else if(cnt1 <  45 ){sp(  -35 ,  -35 );}
    else if(cnt1 <  70 ){sp(  -30 ,  -30 );}
    else if(cnt1 <  95 ){sp(  -25 ,  -25 );}
    else if(cnt1 < 120 ){sp(  -20 ,  -20 );}
    else if(cnt1 < 145 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 170 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 195 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 220 ){sp(    0 ,    0 );}
    else if(cnt1 < 245 ){sp(   10 ,   10 );}
    else if(cnt1 < 270 ){sp(   20 ,   20 );}
    else if(cnt1 < 295 ){sp(   30 ,   30 );}
    else if(cnt1 < 320 ){sp(   40 ,   40 );}
    else if(cnt1 < 345 ){sp(   50 ,   50 );}
    else if(cnt1 < 370 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=   9 ){     /* �N���X���C���ʉߌ�̌������� */
         if(cnt1 <  10 ){sp(  -50 ,  -50 );}
    else if(cnt1 <  35 ){sp(  -35 ,  -35 );}
    else if(cnt1 <  60 ){sp(  -30 ,  -30 );}
    else if(cnt1 <  85 ){sp(  -25 ,  -25 );}
    else if(cnt1 < 110 ){sp(  -20 ,  -20 );}
    else if(cnt1 < 135 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 160 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 185 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 210 ){sp(    0 ,    0 );}
    else if(cnt1 < 235 ){sp(   10 ,   10 );}
    else if(cnt1 < 260 ){sp(   20 ,   20 );}
    else if(cnt1 < 285 ){sp(   30 ,   30 );}
    else if(cnt1 < 310 ){sp(   40 ,   40 );}
    else if(cnt1 < 335 ){sp(   50 ,   50 );}
    else if(cnt1 < 360 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  10 ){     /* �N���X���C���ʉߌ�̌������� */
//         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    if(cnt1 <  25 ){sp(  -35 ,  -35 );}
    else if(cnt1 <  50 ){sp(  -30 ,  -30 );}
    else if(cnt1 <  75 ){sp(  -25 ,  -25 );}
    else if(cnt1 < 100 ){sp(  -20 ,  -20 );}
    else if(cnt1 < 125 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 150 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 175 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 200 ){sp(    0 ,    0 );}
    else if(cnt1 < 225 ){sp(   10 ,   10 );}
    else if(cnt1 < 250 ){sp(   20 ,   20 );}
    else if(cnt1 < 275 ){sp(   30 ,   30 );}
    else if(cnt1 < 300 ){sp(   40 ,   40 );}
    else if(cnt1 < 325 ){sp(   50 ,   50 );}
    else if(cnt1 < 350 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  11 ){     /* �N���X���C���ʉߌ�̌������� */
//         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    if(cnt1 <  15 ){sp(  -35 ,  -35 );}
    else if(cnt1 <  40 ){sp(  -30 ,  -30 );}
    else if(cnt1 <  65 ){sp(  -25 ,  -25 );}
    else if(cnt1 <  90 ){sp(  -20 ,  -20 );}
    else if(cnt1 < 115 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 140 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 165 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 190 ){sp(    0 ,    0 );}
    else if(cnt1 < 215 ){sp(   10 ,   10 );}
    else if(cnt1 < 240 ){sp(   20 ,   20 );}
    else if(cnt1 < 265 ){sp(   30 ,   30 );}
    else if(cnt1 < 290 ){sp(   40 ,   40 );}
    else if(cnt1 < 315 ){sp(   50 ,   50 );}
    else if(cnt1 < 340 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  12 ){     /* �N���X���C���ʉߌ�̌������� */
//         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    if(cnt1 <   5 ){sp(  -35 ,  -35 );}
    else if(cnt1 <  30 ){sp(  -30 ,  -30 );}
    else if(cnt1 <  55 ){sp(  -25 ,  -25 );}
    else if(cnt1 <  80 ){sp(  -20 ,  -20 );}
    else if(cnt1 < 105 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 130 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 155 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 180 ){sp(    0 ,    0 );}
    else if(cnt1 < 205 ){sp(   10 ,   10 );}
    else if(cnt1 < 230 ){sp(   20 ,   20 );}
    else if(cnt1 < 255 ){sp(   30 ,   30 );}
    else if(cnt1 < 280 ){sp(   40 ,   40 );}
    else if(cnt1 < 305 ){sp(   50 ,   50 );}
    else if(cnt1 < 330 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  13 ){     /* �N���X���C���ʉߌ�̌������� */
/*         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}	*/
    if(cnt1 <  20 ){sp(  -30 ,  -30 );}
    else if(cnt1 <  45 ){sp(  -25 ,  -25 );}
    else if(cnt1 <  70 ){sp(  -20 ,  -20 );}
    else if(cnt1 <  95 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 120 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 145 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 170 ){sp(    0 ,    0 );}
    else if(cnt1 < 195 ){sp(   10 ,   10 );}
    else if(cnt1 < 220 ){sp(   20 ,   20 );}
    else if(cnt1 < 245 ){sp(   30 ,   30 );}
    else if(cnt1 < 270 ){sp(   40 ,   40 );}
    else if(cnt1 < 295 ){sp(   50 ,   50 );}
    else if(cnt1 < 320 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  14 ){     /* �N���X���C���ʉߌ�̌������� */
/*         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}	*/
    if(cnt1 <  10 ){sp(  -30 ,  -30 );}
    else if(cnt1 <  35 ){sp(  -25 ,  -25 );}
    else if(cnt1 <  60 ){sp(  -20 ,  -20 );}
    else if(cnt1 <  85 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 110 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 135 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 160 ){sp(    0 ,    0 );}
    else if(cnt1 < 185 ){sp(   10 ,   10 );}
    else if(cnt1 < 210 ){sp(   20 ,   20 );}
    else if(cnt1 < 235 ){sp(   30 ,   30 );}
    else if(cnt1 < 260 ){sp(   40 ,   40 );}
    else if(cnt1 < 285 ){sp(   50 ,   50 );}
    else if(cnt1 < 310 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  15 ){     /* �N���X���C���ʉߌ�̌������� */
/*         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}	*/
    if(cnt1 <  25 ){sp(  -25 ,  -25 );}
    else if(cnt1 <  50 ){sp(  -20 ,  -20 );}
    else if(cnt1 <  75 ){sp(  -15 ,  -15 );}
    else if(cnt1 < 100 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 125 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 150 ){sp(    0 ,    0 );}
    else if(cnt1 < 175 ){sp(   10 ,   10 );}
    else if(cnt1 < 200 ){sp(   20 ,   20 );}
    else if(cnt1 < 225 ){sp(   30 ,   30 );}
    else if(cnt1 < 250 ){sp(   40 ,   40 );}
    else if(cnt1 < 275 ){sp(   50 ,   50 );}
    else if(cnt1 < 300 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  16 ){     /* �N���X���C���ʉߌ�̌������� */
/*         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}	*/
    if(cnt1 <  15 ){sp(  -25 ,  -25 );}
    else if(cnt1 <  40 ){sp(  -20 ,  -20 );}
    else if(cnt1 <  65 ){sp(  -15 ,  -15 );}
    else if(cnt1 <  90 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 115 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 140 ){sp(    0 ,    0 );}
    else if(cnt1 < 165 ){sp(   10 ,   10 );}
    else if(cnt1 < 190 ){sp(   20 ,   20 );}
    else if(cnt1 < 215 ){sp(   30 ,   30 );}
    else if(cnt1 < 240 ){sp(   40 ,   40 );}
    else if(cnt1 < 265 ){sp(   50 ,   50 );}
    else if(cnt1 < 290 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  17 ){     /* �N���X���C���ʉߌ�̌������� */
/*         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}	*/
    if(cnt1 <   5 ){sp(  -25 ,  -25 );}
    else if(cnt1 <  30 ){sp(  -20 ,  -20 );}
    else if(cnt1 <  55 ){sp(  -15 ,  -15 );}
    else if(cnt1 <  80 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 105 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 130 ){sp(    0 ,    0 );}
    else if(cnt1 < 155 ){sp(   10 ,   10 );}
    else if(cnt1 < 180 ){sp(   20 ,   20 );}
    else if(cnt1 < 205 ){sp(   30 ,   30 );}
    else if(cnt1 < 230 ){sp(   40 ,   40 );}
    else if(cnt1 < 255 ){sp(   50 ,   50 );}
    else if(cnt1 < 280 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  18 ){     /* �N���X���C���ʉߌ�̌������� */
/*         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}
    else if(cnt1 <   0 ){sp(  -25 ,  -25 );}	*/
    if(cnt1 <  20 ){sp(  -20 ,  -20 );}
    else if(cnt1 <  45 ){sp(  -15 ,  -15 );}
    else if(cnt1 <  70 ){sp(  -10 ,  -10 );}
    else if(cnt1 <  95 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 120 ){sp(    0 ,    0 );}
    else if(cnt1 < 145 ){sp(   10 ,   10 );}
    else if(cnt1 < 170 ){sp(   20 ,   20 );}
    else if(cnt1 < 195 ){sp(   30 ,   30 );}
    else if(cnt1 < 220 ){sp(   40 ,   40 );}
    else if(cnt1 < 245 ){sp(   50 ,   50 );}
    else if(cnt1 < 270 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  19 ){     /* �N���X���C���ʉߌ�̌������� */
/*         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}
    else if(cnt1 <   0 ){sp(  -25 ,  -25 );}	*/
    if(cnt1 <  10 ){sp(  -20 ,  -20 );}
    else if(cnt1 <  35 ){sp(  -15 ,  -15 );}
    else if(cnt1 <  60 ){sp(  -10 ,  -10 );}
    else if(cnt1 <  85 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 110 ){sp(    0 ,    0 );}
    else if(cnt1 < 135 ){sp(   10 ,   10 );}
    else if(cnt1 < 160 ){sp(   20 ,   20 );}
    else if(cnt1 < 185 ){sp(   30 ,   30 );}
    else if(cnt1 < 210 ){sp(   40 ,   40 );}
    else if(cnt1 < 235 ){sp(   50 ,   50 );}
    else if(cnt1 < 260 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  20 ){     /* �N���X���C���ʉߌ�̌������� */
/*         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}
    else if(cnt1 <   0 ){sp(  -25 ,  -25 );}
    else if(cnt1 <   0 ){sp(  -20 ,  -20 );}	*/
    if(cnt1 <  25 ){sp(  -15 ,  -15 );}
    else if(cnt1 <  50 ){sp(  -10 ,  -10 );}
    else if(cnt1 <  75 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 100 ){sp(    0 ,    0 );}
    else if(cnt1 < 125 ){sp(   10 ,   10 );}
    else if(cnt1 < 150 ){sp(   20 ,   20 );}
    else if(cnt1 < 175 ){sp(   30 ,   30 );}
    else if(cnt1 < 200 ){sp(   40 ,   40 );}
    else if(cnt1 < 225 ){sp(   50 ,   50 );}
    else if(cnt1 < 250 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  21 ){     /* �N���X���C���ʉߌ�̌������� */
/*         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}
    else if(cnt1 <   0 ){sp(  -25 ,  -25 );}
    else if(cnt1 <   0 ){sp(  -20 ,  -20 );}	*/
    if(cnt1 <  15 ){sp(  -15 ,  -15 );}
    else if(cnt1 <  40 ){sp(  -10 ,  -10 );}
    else if(cnt1 <  65 ){sp(   -5 ,   -5 );}
    else if(cnt1 <  90 ){sp(    0 ,    0 );}
    else if(cnt1 < 115 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 165 ){sp(   30 ,   30 );}
    else if(cnt1 < 190 ){sp(   40 ,   40 );}
    else if(cnt1 < 215 ){sp(   50 ,   50 );}
    else if(cnt1 < 240 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  22 ){     /* �N���X���C���ʉߌ�̌������� */
 /*        if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}
    else if(cnt1 <   0 ){sp(  -25 ,  -25 );}
    else if(cnt1 <   0 ){sp(  -20 ,  -20 );}	*/
    if(cnt1 <   5 ){sp(  -15 ,  -15 );}
    else if(cnt1 <  30 ){sp(  -10 ,  -10 );}
    else if(cnt1 <  55 ){sp(   -5 ,   -5 );}
    else if(cnt1 <  80 ){sp(    0 ,    0 );}
    else if(cnt1 < 105 ){sp(   10 ,   10 );}
    else if(cnt1 < 130 ){sp(   20 ,   20 );}
    else if(cnt1 < 155 ){sp(   30 ,   30 );}
    else if(cnt1 < 180 ){sp(   40 ,   40 );}
    else if(cnt1 < 205 ){sp(   50 ,   50 );}
    else if(cnt1 < 230 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  23 ){     /* �N���X���C���ʉߌ�̌������� */
 /*        if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}
    else if(cnt1 <   0 ){sp(  -25 ,  -25 );}
    else if(cnt1 <   0 ){sp(  -20 ,  -20 );}
    else if(cnt1 <   0 ){sp(  -15 ,  -15 );}	*/
    if(cnt1 <  20 ){sp(  -10 ,  -10 );}
    else if(cnt1 <  45 ){sp(   -5 ,   -5 );}
    else if(cnt1 <  70 ){sp(    0 ,    0 );}
    else if(cnt1 <  95 ){sp(   10 ,   10 );}
    else if(cnt1 < 120 ){sp(   20 ,   20 );}
    else if(cnt1 < 145 ){sp(   30 ,   30 );}
    else if(cnt1 < 170 ){sp(   40 ,   40 );}
    else if(cnt1 < 195 ){sp(   50 ,   50 );}
    else if(cnt1 < 220 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  24 ){     /* �N���X���C���ʉߌ�̌������� */
/*         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}
    else if(cnt1 <   0 ){sp(  -25 ,  -25 );}
    else if(cnt1 <   0 ){sp(  -20 ,  -20 );}
    else if(cnt1 <   0 ){sp(  -15 ,  -15 );}	*/
    if(cnt1 <  10 ){sp(  -10 ,  -10 );}
    else if(cnt1 <  35 ){sp(   -5 ,   -5 );}
    else if(cnt1 <  60 ){sp(    0 ,    0 );}
    else if(cnt1 <  85 ){sp(   10 ,   10 );}
    else if(cnt1 < 110 ){sp(   20 ,   20 );}
    else if(cnt1 < 135 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 185 ){sp(   50 ,   50 );}
    else if(cnt1 < 210 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  25 ){     /* �N���X���C���ʉߌ�̌������� */
 /*        if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}
    else if(cnt1 <   0 ){sp(  -25 ,  -25 );}
    else if(cnt1 <   0 ){sp(  -20 ,  -20 );}
    else if(cnt1 <   0 ){sp(  -15 ,  -15 );}
    else if(cnt1 <   0 ){sp(  -10 ,  -10 );}	*/
    if(cnt1 <  25 ){sp(   -5 ,   -5 );}
    else if(cnt1 <  50 ){sp(    0 ,    0 );}
    else if(cnt1 <  75 ){sp(   10 ,   10 );}
    else if(cnt1 < 100 ){sp(   20 ,   20 );}
    else if(cnt1 < 125 ){sp(   30 ,   30 );}
    else if(cnt1 < 150 ){sp(   40 ,   40 );}
    else if(cnt1 < 175 ){sp(   50 ,   50 );}
    else if(cnt1 < 200 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  26 ){     /* �N���X���C���ʉߌ�̌������� */
  /*       if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}
    else if(cnt1 <   0 ){sp(  -25 ,  -25 );}
    else if(cnt1 <   0 ){sp(  -20 ,  -20 );}
    else if(cnt1 <   0 ){sp(  -15 ,  -15 );}
    else if(cnt1 <   0 ){sp(  -10 ,  -10 );}	*/
    if(cnt1 <  15 ){sp(   -5 ,   -5 );}
    else if(cnt1 <  40 ){sp(    0 ,    0 );}
    else if(cnt1 <  65 ){sp(   10 ,   10 );}
    else if(cnt1 <  90 ){sp(   20 ,   20 );}
    else if(cnt1 < 115 ){sp(   30 ,   30 );}
    else if(cnt1 < 140 ){sp(   40 ,   40 );}
    else if(cnt1 < 165 ){sp(   50 ,   50 );}
    else if(cnt1 < 190 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  27 ){     /* �N���X���C���ʉߌ�̌������� */
/*         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}
    else if(cnt1 <   0 ){sp(  -25 ,  -25 );}
    else if(cnt1 <   0 ){sp(  -20 ,  -20 );}
    else if(cnt1 <   0 ){sp(  -15 ,  -15 );}
    else if(cnt1 <   0 ){sp(  -10 ,  -10 );}	*/
    if(cnt1 <   5 ){sp(   -5 ,   -5 );}
    else if(cnt1 <  30 ){sp(    0 ,    0 );}
    else if(cnt1 <  55 ){sp(   10 ,   10 );}
    else if(cnt1 <  80 ){sp(   20 ,   20 );}
    else if(cnt1 < 105 ){sp(   30 ,   30 );}
    else if(cnt1 < 130 ){sp(   40 ,   40 );}
    else if(cnt1 < 155 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  28 ){     /* �N���X���C���ʉߌ�̌������� */
/*         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}
    else if(cnt1 <   0 ){sp(  -25 ,  -25 );}
    else if(cnt1 <   0 ){sp(  -20 ,  -20 );}
    else if(cnt1 <   0 ){sp(  -15 ,  -15 );}
    else if(cnt1 <   0 ){sp(  -10 ,  -10 );}
    else if(cnt1 <   0 ){sp(   -5 ,   -5 );}	*/
    if(cnt1 <  20 ){sp(    0 ,    0 );}
    else if(cnt1 <  45 ){sp(   10 ,   10 );}
    else if(cnt1 <  70 ){sp(   20 ,   20 );}
    else if(cnt1 <  95 ){sp(   30 ,   30 );}
    else if(cnt1 < 120 ){sp(   40 ,   40 );}
    else if(cnt1 < 145 ){sp(   50 ,   50 );}
    else if(cnt1 < 170 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  29 ){     /* �N���X���C���ʉߌ�̌������� */
/*         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}
    else if(cnt1 <   0 ){sp(  -25 ,  -25 );}
    else if(cnt1 <   0 ){sp(  -20 ,  -20 );}
    else if(cnt1 <   0 ){sp(  -15 ,  -15 );}
    else if(cnt1 <   0 ){sp(  -10 ,  -10 );}
    else if(cnt1 <   0 ){sp(   -5 ,   -5 );}	*/
    if(cnt1 <  10 ){sp(    0 ,    0 );}
    else if(cnt1 <  35 ){sp(   10 ,   10 );}
    else if(cnt1 <  60 ){sp(   20 ,   20 );}
    else if(cnt1 <  85 ){sp(   30 ,   30 );}
    else if(cnt1 < 110 ){sp(   40 ,   40 );}
    else if(cnt1 < 135 ){sp(   50 ,   50 );}
    else if(cnt1 < 160 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  30 ){     /* �N���X���C���ʉߌ�̌������� */
/*         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}
    else if(cnt1 <   0 ){sp(  -25 ,  -25 );}
    else if(cnt1 <   0 ){sp(  -20 ,  -20 );}
    else if(cnt1 <   0 ){sp(  -15 ,  -15 );}
    else if(cnt1 <   0 ){sp(  -10 ,  -10 );}
    else if(cnt1 <   0 ){sp(   -5 ,   -5 );}
    else if(cnt1 <   0 ){sp(    0 ,    0 );}	*/
    if(cnt1 <  25 ){sp(   10 ,   10 );}
    else if(cnt1 <  50 ){sp(   20 ,   20 );}
    else if(cnt1 <  75 ){sp(   30 ,   30 );}
    else if(cnt1 < 100 ){sp(   40 ,   40 );}
    else if(cnt1 < 125 ){sp(   50 ,   50 );}
    else if(cnt1 < 150 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  31 ){     /* �N���X���C���ʉߌ�̌������� */
/*         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}
    else if(cnt1 <   0 ){sp(  -25 ,  -25 );}
    else if(cnt1 <   0 ){sp(  -20 ,  -20 );}
    else if(cnt1 <   0 ){sp(  -15 ,  -15 );}
    else if(cnt1 <   0 ){sp(  -10 ,  -10 );}
    else if(cnt1 <   0 ){sp(   -5 ,   -5 );}
    else if(cnt1 <   0 ){sp(    0 ,    0 );}	*/
    if(cnt1 <  15 ){sp(   10 ,   10 );}
    else if(cnt1 <  40 ){sp(   20 ,   20 );}
    else if(cnt1 <  65 ){sp(   30 ,   30 );}
    else if(cnt1 <  90 ){sp(   40 ,   40 );}
    else if(cnt1 < 115 ){sp(   50 ,   50 );}
    else if(cnt1 < 140 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  32 ){     /* �N���X���C���ʉߌ�̌������� */
/*         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}
    else if(cnt1 <   0 ){sp(  -25 ,  -25 );}
    else if(cnt1 <   0 ){sp(  -20 ,  -20 );}
    else if(cnt1 <   0 ){sp(  -15 ,  -15 );}
    else if(cnt1 <   0 ){sp(  -10 ,  -10 );}
    else if(cnt1 <   0 ){sp(   -5 ,   -5 );}
    else if(cnt1 <   0 ){sp(    0 ,    0 );}	*/
    if(cnt1 <   5 ){sp(   10 ,   10 );}
    else if(cnt1 <  30 ){sp(   20 ,   20 );}
    else if(cnt1 <  55 ){sp(   30 ,   30 );}
    else if(cnt1 <  80 ){sp(   40 ,   40 );}
    else if(cnt1 < 105 ){sp(   50 ,   50 );}
    else if(cnt1 < 130 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  33 ){     /* �N���X���C���ʉߌ�̌������� */
/*         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}
    else if(cnt1 <   0 ){sp(  -25 ,  -25 );}
    else if(cnt1 <   0 ){sp(  -20 ,  -20 );}
    else if(cnt1 <   0 ){sp(  -15 ,  -15 );}
    else if(cnt1 <   0 ){sp(  -10 ,  -10 );}
    else if(cnt1 <   0 ){sp(   -5 ,   -5 );}
    else if(cnt1 <   0 ){sp(    0 ,    0 );}	
    else if(cnt1 <   0 ){sp(   10 ,   10 );}	*/
    if(cnt1 <  20 ){sp(   20 ,   20 );}
    else if(cnt1 <  45 ){sp(   30 ,   30 );}
    else if(cnt1 <  70 ){sp(   40 ,   40 );}
    else if(cnt1 <  95 ){sp(   50 ,   50 );}
    else if(cnt1 < 120 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  34 ){     /* �N���X���C���ʉߌ�̌������� */

    if(cnt1 <  10 ){sp(   20 ,   20 );}
    else if(cnt1 <  35 ){sp(   30 ,   30 );}
    else if(cnt1 <  60 ){sp(   40 ,   40 );}
    else if(cnt1 <  85 ){sp(   50 ,   50 );}
    else if(cnt1 < 110 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  35 ){     /* �N���X���C���ʉߌ�̌������� */

    if(cnt1 <  25 ){sp(   30 ,   30 );}
    else if(cnt1 <  50 ){sp(   40 ,   40 );}
    else if(cnt1 <  75 ){sp(   50 ,   50 );}
    else if(cnt1 < 100 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  36 ){     /* �N���X���C���ʉߌ�̌������� */
	
    if(cnt1 <  15 ){sp(   30 ,   30 );}
    else if(cnt1 <  40 ){sp(   40 ,   40 );}
    else if(cnt1 <  65 ){sp(   50 ,   50 );}
    else if(cnt1 <  90 ){sp(   60 ,   60 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  37 ){     /* �N���X���C���ʉߌ�̌������� */

    if(cnt1 <   5 ){sp(   30 ,   30 );}
    else if(cnt1 <  30 ){sp(   40 ,   40 );}
    else if(cnt1 <  55 ){sp(   50 ,   50 );}
    else if(cnt1 <  80 ){sp(   70 ,   70 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  38 ){     /* �N���X���C���ʉߌ�̌������� */

    if(cnt1 <  20 ){sp(   50 ,   50 );}
    else if(cnt1 <  45 ){sp(   60 ,   60 );}
    else if(cnt1 <  70 ){sp(   70 ,   70 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  39 ){     /* �N���X���C���ʉߌ�̌������� */
 /*        if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}
    else if(cnt1 <   0 ){sp(  -25 ,  -25 );}
    else if(cnt1 <   0 ){sp(  -20 ,  -20 );}
    else if(cnt1 <   0 ){sp(  -15 ,  -15 );}
    else if(cnt1 <   0 ){sp(  -10 ,  -10 );}
    else if(cnt1 <   0 ){sp(   -5 ,   -5 );}
    else if(cnt1 <   0 ){sp(    0 ,    0 );}
    else if(cnt1 <   0 ){sp(   10 ,   10 );}
    else if(cnt1 <   0 ){sp(   20 ,   20 );}
    else if(cnt1 <   0 ){sp(   30 ,   30 );}	*/
    if(cnt1 <  10 ){sp(   50 ,   50 );}
    else if(cnt1 <  35 ){sp(   60 ,   60 );}
    else if(cnt1 <  60 ){sp(   70 ,   70 );}
                   else {sp(   80 ,   80 );}
    }
    else if( kyoritime <=  40 ){     /* �N���X���C���ʉߌ�̌������� */
/*         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}
    else if(cnt1 <   0 ){sp(  -25 ,  -25 );}
    else if(cnt1 <   0 ){sp(  -20 ,  -20 );}
    else if(cnt1 <   0 ){sp(  -15 ,  -15 );}
    else if(cnt1 <   0 ){sp(  -10 ,  -10 );}
    else if(cnt1 <   0 ){sp(   -5 ,   -5 );}
    else if(cnt1 <   0 ){sp(    0 ,    0 );}
    else if(cnt1 <   0 ){sp(   10 ,   10 );}
    else if(cnt1 <   0 ){sp(   20 ,   20 );}
    else if(cnt1 <   0 ){sp(   30 ,   30 );}
    else if(cnt1 <   0 ){sp(   40 ,   40 );}	*/
    if(cnt1 <  25 ){sp(   60 ,   60 );}
    else if(cnt1 <  50 ){sp(   70 ,   70 );}
                   else {sp(   80 ,   80 );}
    }
    else                       {     /* �N���X���C���ʉߌ�̌������� */
/*         if(cnt1 <   0 ){sp(  -40 ,  -40 );}
    else if(cnt1 <   0 ){sp(  -35 ,  -35 );}
    else if(cnt1 <   0 ){sp(  -30 ,  -30 );}
    else if(cnt1 <   0 ){sp(  -25 ,  -25 );}
    else if(cnt1 <   0 ){sp(  -20 ,  -20 );}
    else if(cnt1 <   0 ){sp(  -15 ,  -15 );}
    else if(cnt1 <   0 ){sp(  -10 ,  -10 );}
    else if(cnt1 <   0 ){sp(   -5 ,   -5 );}
    else if(cnt1 <   0 ){sp(    0 ,    0 );}
    else if(cnt1 <   0 ){sp(   10 ,   10 );}
    else if(cnt1 <   0 ){sp(   20 ,   20 );}
    else if(cnt1 <   0 ){sp(   30 ,   30 );}
    else if(cnt1 <   0 ){sp(   40 ,   40 );}	*/
    if(cnt1 <  15 ){sp(   60 ,   60 );}
    else if(cnt1 <  40 ){sp(   70 ,   70 );}
                   else {sp(   80 ,   80 );}
    }
}

/********************************************************************/
/* �e�����̃��[���`�F���W�����̏W��									*/
/* �����p�^�[���ɂ��Ă͐擪�̃O���[�o���ϐ��̐錾�ɂčs��			*/
/********************************************************************/
void shasenhenkou( void )
{
	unsigned char b, c;
	b = sensor_inp(MASK4_0);
	c = sensor_inp(MASK0_4);
	sp_mode( FREE , FREE );
	
	if( b == 0xf0 ){
		pattern = 130;	
	}
	if( b == 0x0f ){
		pattern = 140;	
	}
	
         if( kyoritime <=   3 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 <  30 ){sp(  100 ,  100 );}
    else if(cnt1 < 100 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 110 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 120 ){sp(    0 ,    0 );}
    else if(cnt1 < 130 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=   4 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 <  35 ){sp(  100 ,  100 );}
    else if(cnt1 < 100 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 110 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 120 ){sp(    0 ,    0 );}
    else if(cnt1 < 130 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=   5 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 <  40 ){sp(  100 ,  100 );}
    else if(cnt1 < 100 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 110 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 120 ){sp(    0 ,    0 );}
    else if(cnt1 < 130 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=   6 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 <  45 ){sp(  100 ,  100 );}
    else if(cnt1 < 100 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 110 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 120 ){sp(    0 ,    0 );}
    else if(cnt1 < 130 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=   7 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 <  50 ){sp(  100 ,  100 );}
    else if(cnt1 < 100 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 110 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 120 ){sp(    0 ,    0 );}
    else if(cnt1 < 130 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=   8 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 <  55 ){sp(  100 ,  100 );}
    else if(cnt1 < 100 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 110 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 120 ){sp(    0 ,    0 );}
    else if(cnt1 < 130 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=   9 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 <  60 ){sp(  100 ,  100 );}
    else if(cnt1 < 100 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 110 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 120 ){sp(    0 ,    0 );}
    else if(cnt1 < 130 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  10 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 <  65 ){sp(  100 ,  100 );}
    else if(cnt1 < 100 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 110 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 120 ){sp(    0 ,    0 );}
    else if(cnt1 < 130 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  11 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 <  70 ){sp(  100 ,  100 );}
    else if(cnt1 < 100 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 110 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 120 ){sp(    0 ,    0 );}
    else if(cnt1 < 130 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  12 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 <  75 ){sp(  100 ,  100 );}
    else if(cnt1 < 100 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 110 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 120 ){sp(    0 ,    0 );}
    else if(cnt1 < 130 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  13 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 <  80 ){sp(  100 ,  100 );}
    else if(cnt1 < 100 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 110 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 120 ){sp(    0 ,    0 );}
    else if(cnt1 < 130 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  14 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 <  85 ){sp(  100 ,  100 );}
    else if(cnt1 < 100 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 110 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 120 ){sp(    0 ,    0 );}
    else if(cnt1 < 130 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  15 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 <  90 ){sp(  100 ,  100 );}
    else if(cnt1 < 100 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 110 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 120 ){sp(    0 ,    0 );}
    else if(cnt1 < 130 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  16 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 <  95 ){sp(  100 ,  100 );}
    else if(cnt1 < 100 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 110 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 120 ){sp(    0 ,    0 );}
    else if(cnt1 < 130 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  17){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 < 100 ){sp(  100 ,  100 );}
    else if(cnt1 < 100 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 110 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 120 ){sp(    0 ,    0 );}
    else if(cnt1 < 130 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  18 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 < 105 ){sp(  100 ,  100 );}
    else if(cnt1 < 105 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 110 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 120 ){sp(    0 ,    0 );}
    else if(cnt1 < 130 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  19 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 < 110 ){sp(  100 ,  100 );}
    else if(cnt1 < 110 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 110 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 120 ){sp(    0 ,    0 );}
    else if(cnt1 < 130 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  20 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 < 115 ){sp(  100 ,  100 );}
    else if(cnt1 < 115 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 115 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 120 ){sp(    0 ,    0 );}
    else if(cnt1 < 130 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  21 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 < 120 ){sp(  100 ,  100 );}
    else if(cnt1 < 120 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 120 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 120 ){sp(    0 ,    0 );}
    else if(cnt1 < 130 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 , 100 );}
    }
    else if( kyoritime <=  22 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 < 125 ){sp(  100 ,  100 );}
    else if(cnt1 < 125 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 125 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 125 ){sp(    0 ,    0 );}
    else if(cnt1 < 130 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  23 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 < 130 ){sp(  100 ,  100 );}
    else if(cnt1 < 130 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 130 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 130 ){sp(    0 ,    0 );}
    else if(cnt1 < 130 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  24 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 < 135 ){sp(  100 ,  100 );}
    else if(cnt1 < 135 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 135 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 135 ){sp(    0 ,    0 );}
    else if(cnt1 < 135 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  25 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 < 140 ){sp(  100 ,  100 );}
    else if(cnt1 < 140 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 140 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 140 ){sp(    0 ,    0 );}
    else if(cnt1 < 140 ){sp(   10 ,   10 );}
    else if(cnt1 < 140 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  26 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 < 145 ){sp(  100 ,  100 );}
    else if(cnt1 < 145 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 145 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 145 ){sp(    0 ,    0 );}
    else if(cnt1 < 145 ){sp(   10 ,   10 );}
    else if(cnt1 < 145 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  27 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 < 150 ){sp(  100 ,  100 );}
    else if(cnt1 < 150 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 150 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 150 ){sp(    0 ,    0 );}
    else if(cnt1 < 150 ){sp(   10 ,   10 );}
    else if(cnt1 < 150 ){sp(   20 ,   20 );}
    else if(cnt1 < 150 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  28 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 < 155 ){sp(  100 ,  100 );}
    else if(cnt1 < 155 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 155 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 155 ){sp(    0 ,    0 );}
    else if(cnt1 < 155 ){sp(   10 ,   10 );}
    else if(cnt1 < 155 ){sp(   20 ,   20 );}
    else if(cnt1 < 155 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  29 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 < 160 ){sp(  100 ,  100 );}
    else if(cnt1 < 160 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 160 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 160 ){sp(    0 ,    0 );}
    else if(cnt1 < 160 ){sp(   10 ,   10 );}
    else if(cnt1 < 160 ){sp(   20 ,   20 );}
    else if(cnt1 < 160 ){sp(   30 ,   30 );}
    else if(cnt1 < 160 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  30 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 < 165 ){sp(  100 ,  100 );}
    else if(cnt1 < 165 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 165 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 165 ){sp(    0 ,    0 );}
    else if(cnt1 < 165 ){sp(   10 ,   10 );}
    else if(cnt1 < 165 ){sp(   20 ,   20 );}
    else if(cnt1 < 165 ){sp(   30 ,   30 );}
    else if(cnt1 < 165 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  31 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 < 170 ){sp(  100 ,  100 );}
    else if(cnt1 < 170 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 170 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 170 ){sp(    0 ,    0 );}
    else if(cnt1 < 170 ){sp(   10 ,   10 );}
    else if(cnt1 < 170 ){sp(   20 ,   20 );}
    else if(cnt1 < 170 ){sp(   30 ,   30 );}
    else if(cnt1 < 170 ){sp(   40 ,   40 );}
    else if(cnt1 < 170 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  32 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 < 175 ){sp(  100 ,  100 );}
    else if(cnt1 < 175 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 175 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 175 ){sp(    0 ,    0 );}
    else if(cnt1 < 175 ){sp(   10 ,   10 );}
    else if(cnt1 < 175 ){sp(   20 ,   20 );}
    else if(cnt1 < 175 ){sp(   30 ,   30 );}
    else if(cnt1 < 175 ){sp(   40 ,   40 );}
    else if(cnt1 < 175 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  33 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 < 180 ){sp(  100 ,  100 );}
    else if(cnt1 < 180 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 180 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 180 ){sp(    0 ,    0 );}
    else if(cnt1 < 180 ){sp(   10 ,   10 );}
    else if(cnt1 < 180 ){sp(   20 ,   20 );}
    else if(cnt1 < 180 ){sp(   30 ,   30 );}
    else if(cnt1 < 180 ){sp(   40 ,   40 );}
    else if(cnt1 < 180 ){sp(   50 ,   50 );}
    else if(cnt1 < 180 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  34 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 < 185 ){sp(  100 ,  100 );}
    else if(cnt1 < 185 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 185 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 185 ){sp(    0 ,    0 );}
    else if(cnt1 < 185 ){sp(   10 ,   10 );}
    else if(cnt1 < 185 ){sp(   20 ,   20 );}
    else if(cnt1 < 185 ){sp(   30 ,   30 );}
    else if(cnt1 < 185 ){sp(   40 ,   40 );}
    else if(cnt1 < 185 ){sp(   50 ,   50 );}
    else if(cnt1 < 185 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else if( kyoritime <=  35 ){  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 < 190 ){sp(  100 ,  100 );}
    else if(cnt1 < 190 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 190 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 190 ){sp(    0 ,    0 );}
    else if(cnt1 < 190 ){sp(   10 ,   10 );}
    else if(cnt1 < 190 ){sp(   20 ,   20 );}
    else if(cnt1 < 190 ){sp(   30 ,   30 );}
    else if(cnt1 < 190 ){sp(   40 ,   40 );}
    else if(cnt1 < 190 ){sp(   50 ,   50 );}
    else if(cnt1 < 190 ){sp(   60 ,   60 );}
    else if(cnt1 < 190 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
    else                       {  /* �Ԑ��ύX���C���ʉߌ�̌������� */
         if(cnt1 < 195 ){sp(  100 ,  100 );}
    else if(cnt1 < 195 ){sp(  -10 ,  -10 );}
    else if(cnt1 < 195 ){sp(   -5 ,   -5 );}
    else if(cnt1 < 195 ){sp(    0 ,    0 );}
    else if(cnt1 < 195 ){sp(   10 ,   10 );}
    else if(cnt1 < 195 ){sp(   20 ,   20 );}
    else if(cnt1 < 195 ){sp(   30 ,   30 );}
    else if(cnt1 < 195 ){sp(   40 ,   40 );}
    else if(cnt1 < 195 ){sp(   50 ,   50 );}
    else if(cnt1 < 195 ){sp(   60 ,   60 );}
    else if(cnt1 < 195 ){sp(   70 ,   70 );}
    else if(cnt1 < 200 ){sp(   80 ,   80 );}
                   else {sp(   100 ,   100 );}
    }
}


/************************************************************************/
/* �O�ւ�PWM����A���ւ�PWM������o�� �n���h���p�x�͌��݂̒l���g�p 		*/
/* ���� �O��PWM 														*/
/* �߂�l ����PWM 														*/
/************************************************************************/
int diff( int pwm )
{
	int ret;
	
	if( pwm >= 0 ) {
	/* PWM�l�����̐��Ȃ� */
		if( angle_buff < 0 ) {
			angle_buff = -angle_buff;
	}
	ret = revolution_difference[angle_buff] * pwm / 100;
	
	} else {
	/* PWM�l�����̐��Ȃ� */
		ret = pwm; /* ���̂܂ܕԂ� */
	}
	return ret;
}

void rgb( unsigned char r, unsigned char g, unsigned char b ){
	if( r == 0 ){
		P3DR |= 0x02;
	} else{
		P3DR &= 0xfd;	
	}
	
	if( g == 0 ){
		P4DR |= 0x08;	
	}
	else{
		P4DR &= 0xf7; 	
	}
	
	if( b == 0 ){
		P4DR |= 0x20;
	}
	else{
		P4DR &= 0xdf;	
	}
	
}

/********************************************************************/
/* end of file                                                      */
/********************************************************************/