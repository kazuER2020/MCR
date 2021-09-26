/****************************************************************************/
/* �}�C�R���J�[�e�X�g�v���O���� "kit07test.c"                               */
/*                          2007.05 �W���p���}�C�R���J�[�����[���s�ψ���    */
/****************************************************************************/

/*
�L�b�g�p�Z���T���Ver.4�A���[�^�h���C�u���Vol.3��
�e�X�g���s���܂��B
CPU��̃f�B�b�v�X�C�b�`�ɂ��e�X�g���e��ύX���܂��B
   DipSW
bit3 2 1 0
   0 0 0 0 LED�̃e�X�g      LED��0.5�b�Ԋu�Ō��݂ɓ_��
   0 0 0 1 �v�b�V���X�C�b�`�̃e�X�g OFF���FLED0�_�� ON���FLED1�_��
   0 0 1 0 �T�[�{�̃e�X�g   0�����E30������30���̌J��Ԃ�
   0 0 1 1 ���얳��
   0 1 0 0 �E���[�^�̃e�X�g ���]���u���[�L�̌J��Ԃ�
   0 1 0 1                  �t�]���u���[�L�̌J��Ԃ�
   0 1 1 0 �����[�^�̃e�X�g ���]���u���[�L�̌J��Ԃ�
   0 1 1 1                  �t�]���u���[�L�̌J��Ԃ�

   1 0 0 0 �Z���T�e�X�g     �Z���Tbit1,0��LED1,0�ɏo��
   1 0 0 1                  �Z���Tbit3,2��LED1,0�ɏo��
   1 0 1 0                  �Z���Tbit5,4��LED1,0�ɏo��
   1 0 1 1                  �Z���Tbit7,6��LED1,0�ɏo��

   1 1 0 0 ���i�e�X�g       PWM  50%�őO�i�A 2�b��X�g�b�v
   1 1 0 1 ���i�e�X�g       PWM  50%�őO�i�A 5�b��X�g�b�v
   1 1 1 0 ���i�e�X�g       PWM 100%�őO�i�A 2�b��X�g�b�v
   1 1 1 1 ���i�e�X�g       PWM 100%�őO�i�A 5�b��X�g�b�v
*/

/*======================================*/
/* �C���N���[�h                         */
/*======================================*/
#include    <no_float.h>
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

/*======================================*/
/* �v���g�^�C�v�錾                     */
/*======================================*/
void init( void );
unsigned char sensor_inp_test( unsigned char mask );
unsigned char dipsw_get( void );
unsigned char pushsw_get( void );
void led_out( unsigned char led );
void speed( int accele_l, int accele_r );
void handle( int angle );

/*======================================*/
/* �O���[�o���ϐ��̐錾                 */
/*======================================*/
unsigned long   cnt0;                   /* timer�֐��p              */
unsigned long   cnt1;                   /* main���Ŏg�p             */

/************************************************************************/
/* ���C���v���O����                                                     */
/************************************************************************/
void main( void )
{
    unsigned char   now_sw;             /* ���݃f�B�b�v�X�C�b�`�L�� */
    unsigned char   before_sw;          /* �O��f�B�b�v�X�C�b�`�L�� */
    unsigned char   c;                  /* ��Ɨp                   */
    int             i;                  /* ��Ɨp                   */

    /* �}�C�R���@�\�̏����� */
    init();                             /* ������                   */
    set_ccr( 0x00 );                    /* �S�̊��荞�݋���         */

    /* �ϐ������� */
    before_sw = dipsw_get();
    cnt1 = 0;

    /* �}�C�R���J�[�̏�ԏ����� */
    handle( 0 );
    speed( 0, 0 );
    led_out( 0x0 );

    while( 1 ) {
    /* �f�B�b�v�X�C�b�`�ǂݍ��� */
    now_sw = dipsw_get();

    /* �O��̃X�C�b�`�l�Ɣ�r */
    if( before_sw != now_sw ) {
        /* �s��v�Ȃ�O��l�X�V�A�^�C�}�l�̃N���A */
        before_sw = now_sw;
        cnt1 = 0;
    }

    /* �f�B�b�v�X�C�b�`�̒l�ɂ��e�X�g���[�h�̑I�� */
    switch( now_sw ) {

        /* LED�̃e�X�g LED��0.5�b�Ԋu�Ō��݂ɓ_�� */
        case 0:
            if( cnt1 < 500 ) {
                led_out( 0x1 );
            } else if( cnt1 < 1000 ) {
                led_out( 0x2 );
            } else {
                cnt1 = 0;
            }
            break;

        /* �v�b�V���X�C�b�`�̃e�X�g OFF���FLED0�_�� ON���FLED1�_�� */
        case 1:
            led_out( pushsw_get() + 1 );
            break;

        /* �T�[�{�̃e�X�g 0�����E45������45���̌J��Ԃ� */
        case 2:
            if( cnt1 < 1000 ) {
                handle( 0 );
            } else if( cnt1 < 2000 ) {
                handle( 45 );
            } else if( cnt1 < 3000 ) {
                handle( -45 );
            } else {
                cnt1 = 0;
            }
            break;

        /* �������Ȃ� */
        case 3:
            break;

        /* �E���[�^�̃e�X�g ���]���u���[�L�̌J��Ԃ� */
        case 4:
            if( cnt1 < 1000 ) {
                speed( 0, 100 );
            } else if( cnt1 < 2000 ) {
                speed( 0, 0 );
            } else {
                cnt1 = 0;
            }
            break;

        /* �E���[�^�̃e�X�g �t�]���u���[�L�̌J��Ԃ� */
        case 5:
            if( cnt1 < 1000 ) {
                speed( 0, -100 );
            } else if( cnt1 < 2000 ) {
                speed( 0, 0 );
            } else {
                cnt1 = 0;
            }
            break;

        /* �����[�^�̃e�X�g ���]���u���[�L�̌J��Ԃ� */
        case 6:
            if( cnt1 < 1000 ) {
                speed( 100, 0 );
            } else if( cnt1 < 2000 ) {
                speed( 0, 0 );
            } else {
                cnt1 = 0;
            }
            break;

        /* �����[�^�̃e�X�g �t�]���u���[�L�̌J��Ԃ� */
        case 7:
            if( cnt1 < 1000 ) {
                speed( -100, 0 );
            } else if( cnt1 < 2000 ) {
                speed( 0, 0 );
            } else {
                cnt1 = 0;
            }
            break;

        /* �Z���T�e�X�g �Z���Tbit1,0��LED1,0�ɏo�� */
        case 8:
            c = sensor_inp_test( 0x03 );
            led_out( c );
            break;

        /* �Z���T�e�X�g �Z���Tbit3,2��LED1,0�ɏo�� */
        case 9:
            c = sensor_inp_test( 0x0c );
            c = c >> 2;
            led_out( c );
            break;

        /* �Z���T�e�X�g �Z���Tbit5,4��LED1,0�ɏo�� */
        case 10:
            c = sensor_inp_test( 0x30 );
            c = c >> 4;
            led_out( c );
            break;

        /* �Z���T�e�X�g �Z���Tbit7,6��LED1,0�ɏo�� */
        case 11:
            c = sensor_inp_test( 0xc0 );
            c = c >> 6;
            led_out( c );
            break;

        /* ���i�e�X�g PWM  50%�őO�i�A 2�b��X�g�b�v */
        case 12:
            if( cnt1 < 2000 ) {
                speed( 0, 0 );
            } else if( cnt1 < 4000 ) {
                speed( 50, 50 );
            } else {
                speed( 0, 0 );
            }
            break;

        /* ���i�e�X�g PWM  50%�őO�i�A 5�b��X�g�b�v */
        case 13:
            if( cnt1 < 2000 ) {
                speed( 0, 0 );
            } else if( cnt1 < 7000 ) {
                speed( 50, 50 );
            } else {
                speed( 0, 0 );
            }
            break;

        /* ���i�e�X�g PWM 100%�őO�i�A 2�b��X�g�b�v */
        case 14:
            if( cnt1 < 2000 ) {
                speed( 0, 0 );
            } else if( cnt1 < 4000 ) {
                speed( 100, 100 );
            } else {
                speed( 0, 0 );
            }
            break;

        /* ���i�e�X�g PWM 100%�őO�i�A 5�b��X�g�b�v */
        case 15:
            if( cnt1 < 2000 ) {
                speed( 0, 0 );
            } else if( cnt1 < 7000 ) {
                speed( 100, 100 );
            } else {
                speed( 0, 0 );
            }
            break;

        /* �ǂ�ł��Ȃ��Ȃ� */
        default:
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

/************************************************************************/
/* �Z���T��Ԍ��o(�e�X�g���[�h�p)                                       */
/* �����@ �}�X�N�l                                                      */
/* �߂�l �Z���T�l                                                      */
/************************************************************************/
unsigned char sensor_inp_test( unsigned char mask )
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
    ITU4_BRB = SERVO_CENTER + angle * HANDLE_STEP;
}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/
