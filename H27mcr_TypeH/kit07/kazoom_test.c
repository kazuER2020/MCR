/*======================================*/
/* インクルード                         */
/*======================================*/
#include 	<no_float.h>
#include 	<stdio.h>
#include    <machine.h>
#include    "h8_3048.h"

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
int         SERVO_CENTER  = 4138;    /* サーボのセンタ値         */
#define         HANDLE_STEP     26      /* 1゜分の値                */

/* マスク値設定 ×：マスクあり(無効)　○：マスク無し(有効) */
#define MASK2_2         0x66            /* ×○○××○○×             */
#define MASK2_0         0x60            /* ×○○×××××             */
#define MASK0_2         0x06            /* ×××××○○×             */
#define MASK3_3         0xe7            /* ○○○××○○○             */
#define MASK0_3         0x07            /* ×××××○○○             */
#define MASK3_0         0xe0            /* ○○○×××××             */
#define MASK4_0         0xf0            /* ○○○○××××             */
#define MASK0_4         0x0f            /* ××××○○○○             */
#define MASK4_4         0xff            /* ○○○○○○○○             */

/*======================================*/
/* プロトタイプ宣言                     */
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
/* グローバル変数の宣言                 */
/*======================================*/
unsigned long   cnt0;                   /* timer関数用              */
unsigned long   cnt1;                   /* main内で使用             */

int pattern = 0;

/************************************************************************/
/* メインプログラム                                                     */
/************************************************************************/
void main( void )
{
    unsigned char   now_sw;             /* 現在ディップスイッチ記憶 */
    unsigned char   before_sw;          /* 前回ディップスイッチ記憶 */
    unsigned char   c = 0;              /* 作業用                   */
    int             in = 0;             /* 作業用                   */

    /* マイコン機能の初期化 */
    init();                             /* 初期化                   */
    init_sci1( 0x00, 79 ); 				/* SCI1初期化 				*/ 
	set_ccr( 0x00 );                    /* 全体割り込み許可         */

    /* 変数初期化 */
    before_sw = dipsw_get();
    cnt1 = 0;

    /* マイコンカーの状態初期化 */
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
				
				/* メニュー */
            	printf( "\n\n" );
            	printf( "MCR_kazoom Ver.1.x"
                	    "Test Program(H8-3048F / RY3048Fone Ver.) Ver1.00\n" );
            	printf( "\n" );
            	printf( "1 : LEDのテスト\n" );
        	    printf( "2 : スイッチのテスト\n" );
    	        printf( "3 : サーボモータのテスト\n" );
    	        printf( "4 : サーボセンター値の調整\n" ); 
				printf( "5 : デジタルセンサ基板のテスト\n" );
	            printf( "6 : モータのテスト\n" );
        	    printf( "\n" );
    	        printf( "1-6の数字を入力してください " );
				pattern = 1;
				break;
			
			case 1:
				scanf("%d", &in );
				if(in > 6 || in < 0 ){
					printf("入力が不正です\n\n");
					pattern = 0;
				}
				else{
					pattern = in * 10;	
					cnt1 = 0;
				}
				scanf( "%*[^\n]" );
				break;
			
			case 10:
				/* LEDのテスト */		
				printf("\nLEDが0.5秒おきに点灯します。\n");
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
				/* スイッチのテスト */
				printf("\nスイッチをテストします\n" );
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
				/* サーボモータのテスト */ // ベーシックマシンでは最大49?
				printf("サーボを1秒おきに 0 -> 45 -> -45 の順に動かします。\n");
				printf("最大角度: ");
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
				/* サーボセンター値の調整 */
				printf("サーボセンター値の調整をします。\n");
				printf("-1を入力で終了です。\n");
				printf("現在の値は %d です\n\n",SERVO_CENTER);
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
				/* デジタルセンサ基板のテスト */
				printf("デジタルセンサ基板(Ver.5)のテストです\n");
				pattern = 51;
				cnt1 = 0;
				break;
			
			case 51:
				printf("sensor= 0x%x\n", sensor_inp(MASK4_4));
				if( cnt1 > 5000 ) pattern = 0;
				break;
			
			
			case 60:
				/* モータのテスト */
				printf("モータのテスト\n");
				printf("1:左  2:右  0:終了\n");
				scanf("%d",&in);
				if(in == 0 ) pattern = 0;
				else{
					pattern = 60+in;
				}
				cnt1 = 0;
				break;
			
			case 61:
				/* 左 */
				printf("左: 停止 ブレーキ\n");
				sp_mode(0, 0 );
				speed(0, 0);
				timer(1000);
				
				printf("左: 正転50% ブレーキ\n");
				sp_mode(0, 0 );
				speed(50, 0);
				timer(1000);
				
				printf("左: 正転50% フリー\n");
				sp_mode(1, 0 );
				speed(50, 0);
				timer(1000);
				
				printf("左: 逆転50% ブレーキ\n");
				sp_mode(0, 0 );
				speed(-50, 0);
				timer(1000);
				
				printf("左: 逆転50% フリー\n");
				sp_mode(1, 0 );
				speed(-50, 0);
				timer(1000);
				
				printf("左: 正転100% ブレーキ\n");
				sp_mode(0, 0 );
				speed(100, 0);
				timer(1000);
			
				printf("左: 正転100% フリー\n");
				sp_mode(1, 0 );
				speed(100, 0);
				timer(1000);
				
				printf("左: 逆転100% ブレーキ\n");
				sp_mode(0, 0 );
				speed(-100, 0);
				timer(1000);
			
				printf("左: 逆転100% フリー\n");
				sp_mode(1, 0 );
				speed(-100, 0);
				timer(1000);
				speed(0, 0);
				pattern = 0;
				break;
			
			case 62:
				/* 右 */
				printf("右: 停止 ブレーキ\n");
				sp_mode(0, 0 );
				speed(0, 0);
				timer(1000);
				
				printf("右: 正転50% ブレーキ\n");
				sp_mode(0, 0 );
				speed(0, 50);
				timer(1000);
				
				printf("右: 正転50% フリー\n");
				sp_mode(0, 1 );
				speed(0, 50);
				timer(1000);
				
				printf("右: 逆転50% ブレーキ\n");
				sp_mode(0, 0 );
				speed(0, -50);
				timer(1000);
				
				printf("右: 逆転50% フリー\n");
				sp_mode(0, 1 );
				speed(0, -50);
				timer(1000);
				
				printf("右: 正転100% ブレーキ\n");
				sp_mode(0, 0 );
				speed(0, 100);
				timer(1000);
			
				printf("右: 正転100% フリー\n");
				sp_mode(0 ,1 );
				speed(0, 100);
				timer(1000);
				
				printf("右: 逆転100% ブレーキ\n");
				sp_mode(0, 0 );
				speed(0, -100);
				timer(1000);
			
				printf("右: 逆転100% フリー\n");
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

void timer( unsigned long timer_set ){
	cnt0 = 0;
	while(cnt0 < timer_set);	
}
/************************************************************************/
/* センサ状態検出(テストモード用)                                       */
/* 引数　 マスク値                                                      */
/* 戻り値 センサ値                                                      */
/************************************************************************/
unsigned char sensor_inp( unsigned char mask )
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
    ITU4_BRB = SERVO_CENTER - angle * HANDLE_STEP;
}

/************************************************************************/
/* モータ停止動作（ブレーキ、フリー）                                   */
/* 引数　 左モータ:FREE or BRAKE , 右モータ:FREE or BRAKE               */
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
