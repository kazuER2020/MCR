/****************************************************************************/
/* マイコンカーテストプログラム "kit07test.c"                               */
/*                          2007.05 ジャパンマイコンカーラリー実行委員会    */
/****************************************************************************/

/*
キット用センサ基板Ver.4、モータドライブ基板Vol.3の
テストを行います。
CPU基板のディップスイッチによりテスト内容を変更します。
   DipSW
bit3 2 1 0
   0 0 0 0 LEDのテスト      LEDが0.5秒間隔で交互に点灯
   0 0 0 1 プッシュスイッチのテスト OFF時：LED0点灯 ON時：LED1点灯
   0 0 1 0 サーボのテスト   0°→右30°→左30°の繰り返し
   0 0 1 1 動作無し
   0 1 0 0 右モータのテスト 正転→ブレーキの繰り返し
   0 1 0 1                  逆転→ブレーキの繰り返し
   0 1 1 0 左モータのテスト 正転→ブレーキの繰り返し
   0 1 1 1                  逆転→ブレーキの繰り返し

   1 0 0 0 センサテスト     センサbit1,0をLED1,0に出力
   1 0 0 1                  センサbit3,2をLED1,0に出力
   1 0 1 0                  センサbit5,4をLED1,0に出力
   1 0 1 1                  センサbit7,6をLED1,0に出力

   1 1 0 0 直進テスト       PWM  50%で前進、 2秒後ストップ
   1 1 0 1 直進テスト       PWM  50%で前進、 5秒後ストップ
   1 1 1 0 直進テスト       PWM 100%で前進、 2秒後ストップ
   1 1 1 1 直進テスト       PWM 100%で前進、 5秒後ストップ
*/

/*======================================*/
/* インクルード                         */
/*======================================*/
#include    <no_float.h>
#include    <machine.h>
#include    "h8_3048.h"
#include    "beep.h"

/*======================================*/
/* シンボル定義                         */
/*======================================*/

/* 定数設定 */
#define         TIMER_CYCLE     3071    /* タイマのサイクル 1ms     */
                                        /* φ/8で使用する場合、     */
                                        /* φ/8 = 325.5[ns]         */
                                        /* ∴TIMER_CYCLE =          */
                                        /*      1[ms] / 325.5[ns]   */
                                        /*               = 3072     */
#define         PWM_CYCLE       42152   /* PWMのサイクル 16ms       */
                                        /* ∴PWM_CYCLE =            */
                                        /*      16[ms] / 325.5[ns]  */
                                        /*               = 49152    */
#define         SERVO_CENTER    4150    /* サーボのセンタ値         */
#define         HANDLE_STEP     26      /* 1゜分の値                */

/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/
void init( void );
unsigned char sensor_inp_test( unsigned char mask );
unsigned char dipsw_get( void );
unsigned char pushsw_get( void );
void led_out( unsigned char led );
void speed( int accele_l, int accele_r );
void handle( int angle );

/*======================================*/
/* グローバル変数の宣言                 */
/*======================================*/
unsigned long   cnt0;                   /* timer関数用              */
unsigned long   cnt1;                   /* main内で使用             */

/************************************************************************/
/* メインプログラム                                                     */
/************************************************************************/
void main( void )
{
    unsigned char   now_sw;             /* 現在ディップスイッチ記憶 */
    unsigned char   before_sw;          /* 前回ディップスイッチ記憶 */
    unsigned char   c;                  /* 作業用                   */
    int             i;                  /* 作業用                   */

    /* マイコン機能の初期化 */
    init();                             /* 初期化                   */
    set_ccr( 0x00 );                    /* 全体割り込み許可         */

    /* 変数初期化 */
    before_sw = dipsw_get();
    cnt1 = 0;

    /* マイコンカーの状態初期化 */
    handle( 0 );
    speed( 0, 0 );
    led_out( 0x0 );

    while( 1 ) {
    /* ディップスイッチ読み込み */
    now_sw = dipsw_get();

    /* 前回のスイッチ値と比較 */
    if( before_sw != now_sw ) {
        /* 不一致なら前回値更新、タイマ値のクリア */
        before_sw = now_sw;
        cnt1 = 0;
    }

    /* ディップスイッチの値によりテストモードの選択 */
    switch( now_sw ) {

        /* LEDのテスト LEDが0.5秒間隔で交互に点灯 */
        case 0:
            if( cnt1 < 500 ) {
                led_out( 0x1 );
            } else if( cnt1 < 1000 ) {
                led_out( 0x2 );
            } else {
                cnt1 = 0;
            }
            break;

        /* プッシュスイッチのテスト OFF時：LED0点灯 ON時：LED1点灯 */
        case 1:
            led_out( pushsw_get() + 1 );
            break;

        /* サーボのテスト 0°→右45°→左45°の繰り返し */
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

        /* 何もしない */
        case 3:
            break;

        /* 右モータのテスト 正転→ブレーキの繰り返し */
        case 4:
            if( cnt1 < 1000 ) {
                speed( 0, 100 );
            } else if( cnt1 < 2000 ) {
                speed( 0, 0 );
            } else {
                cnt1 = 0;
            }
            break;

        /* 右モータのテスト 逆転→ブレーキの繰り返し */
        case 5:
            if( cnt1 < 1000 ) {
                speed( 0, -100 );
            } else if( cnt1 < 2000 ) {
                speed( 0, 0 );
            } else {
                cnt1 = 0;
            }
            break;

        /* 左モータのテスト 正転→ブレーキの繰り返し */
        case 6:
            if( cnt1 < 1000 ) {
                speed( 100, 0 );
            } else if( cnt1 < 2000 ) {
                speed( 0, 0 );
            } else {
                cnt1 = 0;
            }
            break;

        /* 左モータのテスト 逆転→ブレーキの繰り返し */
        case 7:
            if( cnt1 < 1000 ) {
                speed( -100, 0 );
            } else if( cnt1 < 2000 ) {
                speed( 0, 0 );
            } else {
                cnt1 = 0;
            }
            break;

        /* センサテスト センサbit1,0をLED1,0に出力 */
        case 8:
            c = sensor_inp_test( 0x03 );
            led_out( c );
            break;

        /* センサテスト センサbit3,2をLED1,0に出力 */
        case 9:
            c = sensor_inp_test( 0x0c );
            c = c >> 2;
            led_out( c );
            break;

        /* センサテスト センサbit5,4をLED1,0に出力 */
        case 10:
            c = sensor_inp_test( 0x30 );
            c = c >> 4;
            led_out( c );
            break;

        /* センサテスト センサbit7,6をLED1,0に出力 */
        case 11:
            c = sensor_inp_test( 0xc0 );
            c = c >> 6;
            led_out( c );
            break;

        /* 直進テスト PWM  50%で前進、 2秒後ストップ */
        case 12:
            if( cnt1 < 2000 ) {
                speed( 0, 0 );
            } else if( cnt1 < 4000 ) {
                speed( 50, 50 );
            } else {
                speed( 0, 0 );
            }
            break;

        /* 直進テスト PWM  50%で前進、 5秒後ストップ */
        case 13:
            if( cnt1 < 2000 ) {
                speed( 0, 0 );
            } else if( cnt1 < 7000 ) {
                speed( 50, 50 );
            } else {
                speed( 0, 0 );
            }
            break;

        /* 直進テスト PWM 100%で前進、 2秒後ストップ */
        case 14:
            if( cnt1 < 2000 ) {
                speed( 0, 0 );
            } else if( cnt1 < 4000 ) {
                speed( 100, 100 );
            } else {
                speed( 0, 0 );
            }
            break;

        /* 直進テスト PWM 100%で前進、 5秒後ストップ */
        case 15:
            if( cnt1 < 2000 ) {
                speed( 0, 0 );
            } else if( cnt1 < 7000 ) {
                speed( 100, 100 );
            } else {
                speed( 0, 0 );
            }
            break;

        /* どれでもないなら */
        default:
            break;
    }
    }
}

/************************************************************************/
/* H8/3048F内蔵モジュール　初期化                                       */
/************************************************************************/
void init( void )
{
    /* ポートの入出力設定 */
    P1DDR = 0xff;
    P2DDR = 0xff;
    P3DDR = 0xff;
    P4DDR = 0xff;
    P5DDR = 0xff;
    P6DDR = 0xf0;                       /* CPU基板上のDIP SW        */
    P8DDR = 0xff;
    P9DDR = 0xf7;                       /* 通信ポート               */
    PADDR = 0xff;
    PBDR  = 0xc0;
    PBDDR = 0xfe;                       /* モータドライブ基板Vol.3  */
    /* ※センサ基板のP7は、入力専用なので入出力設定はありません     */

    /* ITU0 1msごとの割り込み */
    ITU0_TCR = 0x23;
    ITU0_GRA = TIMER_CYCLE;
    ITU0_IER = 0x01;

    /* ITU3,4 リセット同期PWMモード 左右モータ、サーボ用 */
    ITU3_TCR = 0x23;
    ITU_FCR  = 0x3e;
    ITU3_GRA = PWM_CYCLE;               /* 周期の設定               */
    ITU3_GRB = ITU3_BRB = 0;            /* 左モータのPWM設定        */
    ITU4_GRA = ITU4_BRA = 0;            /* 右モータのPWM設定        */
    ITU4_GRB = ITU4_BRB = SERVO_CENTER; /* サーボのPWM設定          */
    ITU_TOER = 0x38;

    /* ITUのカウントスタート */
    ITU_STR = 0x09;
}

/************************************************************************/
/* ITU0 割り込み処理                                                    */
/************************************************************************/
#pragma interrupt( interrupt_timer0 )
void interrupt_timer0( void )
{
    ITU0_TSR &= 0xfe;                   /* フラグクリア             */
    cnt0++;
    cnt1++;
}

/************************************************************************/
/* センサ状態検出(テストモード用)                                       */
/* 引数　 マスク値                                                      */
/* 戻り値 センサ値                                                      */
/************************************************************************/
unsigned char sensor_inp_test( unsigned char mask )
{
    unsigned char sensor;

    sensor  = ~P7DR;
    sensor &= mask;

    return sensor;
}

/************************************************************************/
/* ディップスイッチ値読み込み                                           */
/* 戻り値 スイッチ値 0～15                                              */
/************************************************************************/
unsigned char dipsw_get( void )
{
    unsigned char sw;

    sw  = ~P6DR;                        /* ディップスイッチ読み込み */
    sw &= 0x0f;
	sw = 0x0f-sw;

    return  sw;
}

/************************************************************************/
/* プッシュスイッチ値読み込み                                           */
/* 戻り値 スイッチ値 ON:1 OFF:0                                         */
/************************************************************************/
unsigned char pushsw_get( void )
{
    unsigned char sw;

    sw  = ~PBDR;                        /* スイッチのあるポート読み込み */
    sw &= 0x01;

    return  sw;
}

/************************************************************************/
/* LED制御                                                              */
/* 引数　スイッチ値 LED0:bit0 LED1:bit1  "0":消灯 "1":点灯              */
/* 例)0x3→LED1:ON LED0:ON  0x2→LED1:ON LED0:OFF                       */
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
/* 速度制御                                                             */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/************************************************************************/
void speed( int accele_l, int accele_r )
{
    unsigned char   sw_data;
    unsigned long   speed_max;

    sw_data  = dipsw_get() + 5;         /* ディップスイッチ読み込み */
    speed_max = (unsigned long)(PWM_CYCLE-1) * sw_data / 20;

    /* 左モータ */
    if( accele_l >= 0 ) {
        PBDR &= 0xfb;
        ITU3_BRB = speed_max * accele_l / 100;
    } else {
        PBDR |= 0x04;
        accele_l = -accele_l;
        ITU3_BRB = speed_max * accele_l / 100;
    }

    /* 右モータ */
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
/* サーボハンドル操作                                                   */
/* 引数　 サーボ操作角度：-90～90                                       */
/*        -90で左へ90度、0でまっすぐ、90で右へ90度回転                  */
/************************************************************************/
void handle( int angle )
{
    ITU4_BRB = SERVO_CENTER + angle * HANDLE_STEP;
}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/
