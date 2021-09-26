/*======================================*/
/* �C���N���[�h                         */
/*======================================*/
#include 	<no_float.h>
#include 	<stdio.h>
#include    <machine.h>
#include    "h8_3048.h"

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
int         SERVO_CENTER  = 4138;    /* �T�[�{�̃Z���^�l         */
#define         HANDLE_STEP     26      /* 1�K���̒l                */

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

/*======================================*/
/* �v���g�^�C�v�錾                     */
/*======================================*/
void init( void );
unsigned char sensor_inp( unsigned char mask );
unsigned char dipsw_get( void );
unsigned char pushsw_get( void );
void led_out( unsigned char led );
void speed( int accele_l, int accele_r );
void handle( int angle );
void sp_mode( int mode_l, int mode_r );
void timer( unsigned long timer_set );

/*======================================*/
/* �O���[�o���ϐ��̐錾                 */
/*======================================*/
unsigned long   cnt0;                   /* timer�֐��p              */
unsigned long   cnt1;                   /* main���Ŏg�p             */

int pattern = 0;

/************************************************************************/
/* ���C���v���O����                                                     */
/************************************************************************/
void main( void )
{
    unsigned char   now_sw;             /* ���݃f�B�b�v�X�C�b�`�L�� */
    unsigned char   before_sw;          /* �O��f�B�b�v�X�C�b�`�L�� */
    unsigned char   c = 0;              /* ��Ɨp                   */
    int             in = 0;             /* ��Ɨp                   */

    /* �}�C�R���@�\�̏����� */
    init();                             /* ������                   */
    init_sci1( 0x00, 79 ); 				/* SCI1������ 				*/ 
	set_ccr( 0x00 );                    /* �S�̊��荞�݋���         */

    /* �ϐ������� */
    before_sw = dipsw_get();
    cnt1 = 0;

    /* �}�C�R���J�[�̏�ԏ����� */
    handle( 0 );
    speed( 0, 0 );
    led_out( 0x0 );
	
	pattern = 0;
    while( 1 ) {
		switch( pattern ){
			case 0:
				handle( 0 );
    			speed( 0, 0 );
    			led_out( 0x0 );
				c = 0;
				in = 0;
				
				/* ���j���[ */
            	printf( "\n\n" );
            	printf( "MCR_kazoom Ver.1.x"
                	    "Test Program(H8-3048F / RY3048Fone Ver.) Ver1.00\n" );
            	printf( "\n" );
            	printf( "1 : LED�̃e�X�g\n" );
        	    printf( "2 : �X�C�b�`�̃e�X�g\n" );
    	        printf( "3 : �T�[�{���[�^�̃e�X�g\n" );
    	        printf( "4 : �T�[�{�Z���^�[�l�̒���\n" ); 
				printf( "5 : �f�W�^���Z���T��̃e�X�g\n" );
	            printf( "6 : ���[�^�̃e�X�g\n" );
        	    printf( "\n" );
    	        printf( "1-6�̐�������͂��Ă������� " );
				pattern = 1;
				break;
			
			case 1:
				scanf("%d", &in );
				if(in > 6 || in < 0 ){
					printf("���͂��s���ł�\n\n");
					pattern = 0;
				}
				else{
					pattern = in * 10;	
					cnt1 = 0;
				}
				scanf( "%*[^\n]" );
				break;
			
			case 10:
				/* LED�̃e�X�g */		
				printf("\nLED��0.5�b�����ɓ_�����܂��B\n");
				pattern = 11;
				cnt1 = 0;
				break;
			
			case 11:
				if( cnt1 >= 2000 ){
					cnt1 = 0;
					c++;
					if( c >= 2 ) pattern = 0;	
				}
				if( cnt1 < 500 ){
					led_out(0x1);
				}
				else if( cnt1 < 1000 ){
					led_out(0x2);	
				}
				else if( cnt1 < 1500 ){
					led_out(0x3);	
				}
				else if( cnt1 < 2000 ){
					led_out(0x0);				
				}
				break;
			
			case 20:
				/* �X�C�b�`�̃e�X�g */
				printf("\n�X�C�b�`���e�X�g���܂�\n" );
				pattern = 21;
				cnt1 = 0;
				break;
			
			case 21:
				if(!pushsw_get()){
					led_out(0x1);	
				}
				else{
					led_out(0x2);	
				}
				if( cnt1 > 5000 ){
					pattern = 0;
					cnt1 = 0;
				}
				break;
			
			case 30:
				/* �T�[�{���[�^�̃e�X�g */ // �x�[�V�b�N�}�V���ł͍ő�49?
				printf("�T�[�{��1�b������ 0 -> 45 -> -45 �̏��ɓ������܂��B\n");
				printf("�ő�p�x: ");
				scanf("%d",&in);
				pattern = 31;
				cnt1 = 0;
				break;
				
			case 31:
				printf("0\n");
				handle(0);
				timer(3000);
				printf("%d\n",in);
				handle(in);
				timer(3000);
				printf("-%d\n",in);
				handle(-1*in);
				timer(3000);
				pattern = 0;
				break;
			
			case 40:
				/* �T�[�{�Z���^�[�l�̒��� */
				printf("�T�[�{�Z���^�[�l�̒��������܂��B\n");
				printf("-1����͂ŏI���ł��B\n");
				printf("���݂̒l�� %d �ł�\n\n",SERVO_CENTER);
				pattern = 41;
				cnt1 = 0;
				break;
			
			case 41:
				handle(0);
				printf("SERVO_CENTER= ");
				scanf("%d", &in);
				if( in == -1 ){
					pattern = 0;	
				}
				else{
					SERVO_CENTER = in;
				}
				break;
			
			case 50:
				/* �f�W�^���Z���T��̃e�X�g */
				printf("�f�W�^���Z���T���(Ver.5)�̃e�X�g�ł�\n");
				pattern = 51;
				cnt1 = 0;
				break;
			
			case 51:
				printf("sensor= 0x%x\n", sensor_inp(MASK4_4));
				if( cnt1 > 5000 ) pattern = 0;
				break;
			
			
			case 60:
				/* ���[�^�̃e�X�g */
				printf("���[�^�̃e�X�g\n");
				printf("1:��  2:�E  0:�I��\n");
				scanf("%d",&in);
				if(in == 0 ) pattern = 0;
				else{
					pattern = 60+in;
				}
				cnt1 = 0;
				break;
			
			case 61:
				/* �� */
				printf("��: ��~ �u���[�L\n");
				sp_mode(0, 0 );
				speed(0, 0);
				timer(1000);
				
				printf("��: ���]50% �u���[�L\n");
				sp_mode(0, 0 );
				speed(50, 0);
				timer(1000);
				
				printf("��: ���]50% �t���[\n");
				sp_mode(1, 0 );
				speed(50, 0);
				timer(1000);
				
				printf("��: �t�]50% �u���[�L\n");
				sp_mode(0, 0 );
				speed(-50, 0);
				timer(1000);
				
				printf("��: �t�]50% �t���[\n");
				sp_mode(1, 0 );
				speed(-50, 0);
				timer(1000);
				
				printf("��: ���]100% �u���[�L\n");
				sp_mode(0, 0 );
				speed(100, 0);
				timer(1000);
			
				printf("��: ���]100% �t���[\n");
				sp_mode(1, 0 );
				speed(100, 0);
				timer(1000);
				
				printf("��: �t�]100% �u���[�L\n");
				sp_mode(0, 0 );
				speed(-100, 0);
				timer(1000);
			
				printf("��: �t�]100% �t���[\n");
				sp_mode(1, 0 );
				speed(-100, 0);
				timer(1000);
				speed(0, 0);
				pattern = 0;
				break;
			
			case 62:
				/* �E */
				printf("�E: ��~ �u���[�L\n");
				sp_mode(0, 0 );
				speed(0, 0);
				timer(1000);
				
				printf("�E: ���]50% �u���[�L\n");
				sp_mode(0, 0 );
				speed(0, 50);
				timer(1000);
				
				printf("�E: ���]50% �t���[\n");
				sp_mode(0, 1 );
				speed(0, 50);
				timer(1000);
				
				printf("�E: �t�]50% �u���[�L\n");
				sp_mode(0, 0 );
				speed(0, -50);
				timer(1000);
				
				printf("�E: �t�]50% �t���[\n");
				sp_mode(0, 1 );
				speed(0, -50);
				timer(1000);
				
				printf("�E: ���]100% �u���[�L\n");
				sp_mode(0, 0 );
				speed(0, 100);
				timer(1000);
			
				printf("�E: ���]100% �t���[\n");
				sp_mode(0 ,1 );
				speed(0, 100);
				timer(1000);
				
				printf("�E: �t�]100% �u���[�L\n");
				sp_mode(0, 0 );
				speed(0, -100);
				timer(1000);
			
				printf("�E: �t�]100% �t���[\n");
				sp_mode(0, 1 );
				speed(0, -100);
				timer(1000);
				speed(0, 0);
				pattern = 0;
				break;
		}
    
    }
}

/************************************************************************/
/* H8/3048F�������W���[���@������                                       */
/************************************************************************/
void init( void )
{
    /* �|�[�g�̓��o�͐ݒ� */
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
}

void timer( unsigned long timer_set ){
	cnt0 = 0;
	while(cnt0 < timer_set);	
}
/************************************************************************/
/* �Z���T��Ԍ��o(�e�X�g���[�h�p)                                       */
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
/* ���x����                                                             */
/* �����@ �����[�^:-100�`100 , �E���[�^:-100�`100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/************************************************************************/
void speed( int accele_l, int accele_r )
{
    unsigned char   sw_data;
    unsigned long   speed_max;

    sw_data  = dipsw_get() + 5;         /* �f�B�b�v�X�C�b�`�ǂݍ��� */
    speed_max = (unsigned long)(PWM_CYCLE-1) * sw_data / 20;

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
/* �T�[�{�n���h������                                                   */
/* �����@ �T�[�{����p�x�F-90�`90                                       */
/*        -90�ō���90�x�A0�ł܂������A90�ŉE��90�x��]                  */
/************************************************************************/
void handle( int angle )
{
    ITU4_BRB = SERVO_CENTER - angle * HANDLE_STEP;
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

/************************************************************************/
/* end of file                                                          */
/************************************************************************/
