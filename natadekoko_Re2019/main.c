/* 191202 デジタルx3 アナログx2　センサに改編済み */
/* 191206 3.2[m/s]以上出せるよう変更 */

/* 200315 ThunderBird-XE.ver.0 マイコンカー仕様 */

/*======================================*/
/* インクルード                         */
/*======================================*/
#include <stdio.h>
#include <stdlib.h>
#include "sfr_r838a.h"                  /* R8C/38A SFRの定義ファイル    */
#include "types3_beep.h"                /* ブザー追加                   */
#include "printf_lib.h"

/*======================================*/
/* シンボル定義                         */
/*======================================*/
/* 定数設定 */

// 変更禁止:
#define     TRC_MOTOR_CYCLE     20000   /* 左前,右前モータPWMの周期     */
/* 50[ns] * 20000 = 1.00[ms]    */
#define     TRD_MOTOR_CYCLE     20000   /* 左後,右後,ｻｰﾎﾞﾓｰﾀPWMの周期   */
/* 50[ns] * 20000 = 1.00[ms]    */

#define     FREE        1   /* モータモード　フリー         */
#define     BRAKE       0   /* モータモード　ブレーキ       */\

#define     HIGH        1   /* 5V入力           */
#define     LOW         0   /* 0V入力           */

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

// 変更OK:
/* dipsw:　4-6 */
#define  	KP 	10 	      /* ステアリング_比例  */
#define		KD 	100 	      /* ステアリング_微分  */

#define  	L_KP  	3      /* 槍_比例  */
#define 	L_KD  	5      /* 槍_微分  */

#define 	BEEP 				1
#define 	ENC  				1   	/* エンコーダが有効か 1:有効 0:無効	*/
#define     ENCHU_ENABLE        0       /* 円柱標的の有効化				*/

#define     RUNNING_TIME        31000L  /* 走行時間(ms)                 */
#define		ENC_END				210L*(dipsw_get())/* 走行距離(cm) */ 

#define     AD_1DEG             1       /* 1度あたりのA/D値の増分       */

#define  	SERVO_PWM_MAX   	100    	/* サーボの最大PWM        */
#define		LANCER_PWM_MAX		0		/* 槍の最大PWM  		*/

#define     CENTER_LNC          512		/* 槍の中心のA/D値		*/         

#define  	ROLLING  			775 /* カーブ検知の閾値 */

/* 的の角度 */
volatile int     ENCHU_ANGLE_AD = 800;			 /* 円柱標的に突っ込む際のボリューム値 */ //786
volatile int     ENCHU_ANGLE = 9;			 /* 円柱標的に突っ込む際のボリューム値 */

#define     UP4BIT /* dipsw2上4bit */ ( dipsw_get2() >> 4 )  /* 平行標的に当てるまでの距離: */
#define     MIN4BIT/* dipsw2下4bit */ ( dipsw_get2()&0x0f )  /* ハーフライン検出から通過までのパルス数(272.5*2.5=25cm) */

// サーボ用
// Arduino nanoとの接続:(出力)
#define     SERVO_ANGLE_PORT    p5_addr.bit.b3
#define     SERVO_ZERO_PORT     p5_addr.bit.b2
#define     SERVO_MODE_PORT     p5_addr.bit.b1

#define     DCM_DIR_PORT		p6_addr.bit.b6
#define     DCM_PWM_PORT        p0.addr.bit.b7

/*======================================*/
/* プロトタイプ宣言                     */
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
/* グローバル変数の宣言                 */
/*======================================*/

/* データフラッシュ関連 */

volatile const char      *C_DATE = __DATE__;     /* コンパイルした日付           */
volatile const char      *C_TIME = __TIME__;     /* コンパイルした時間           */

volatile int             pattern       = 0;      /* マイコンカー動作パターン     */
volatile int             pattern_settings = 0;
volatile int             crank_mode = 0;
volatile int  			 iSensorPattern=0;
volatile unsigned long   cnt0          = 0;
volatile unsigned long   cnt_run       = 0;      /* タイマ用                     */
volatile unsigned long   cnt1          = 0;      /* タイマ用                     */
volatile unsigned long   check_cross_cnt = 0;    /* タイマ用                     */
volatile unsigned long   check_sen_cnt = 0;      /* タイマ用                     */
volatile unsigned long   check_enc_cnt = 0;      /* タイマ用                     */
volatile int             anchi_cross = 0;        /* 1:前回に片方のデジタルセンサが反応 0:デジタルの反応なし */
volatile int             hitcount      = 0;      /* ハーフラインを読んだ回数     */
volatile int             hyouteki_flag = 0;      /* 標的が垂直か平行かを見分ける(平行標的:0 垂直標的:1)*/
volatile int             heikou  = 0;
volatile int             stair_flag  = 0;        /* 1:大きく曲がっている 0: 直線 */
volatile int	   	     kyori_flug = 0;             	/* 各ラインの通過検出宣言  	 スタート前の初期設定   */
volatile int	   		 kyoritime  = 0;             	/* 各ラインの通過時間宣言    スタート前の初期設定   */
volatile unsigned long	 cross_cnt = 0;			 /*  */


/* エンコーダ関連     */
volatile int             iTimer10     = 0;       /* 10msカウント用               */
volatile int             iEncoder     = 0;       /* 10ms毎の最新値               */
volatile int             iEncoderMax  = 0;       /* 現在最大値                   */
volatile long            lEncoderLine = 0;       /* ライン検出時の積算値       */
volatile long            lEncoderTotal = 0;      /* 積算値保存用                 */
volatile unsigned int    uEncoderBuff = 0;       /* 計算用　割り込み内で使用     */

/* サーボ関連       */
volatile int             iSensorBefore;          /* 前回のセンサ値保存           */
volatile int             iServoPwm;              /* サーボPWM値                */
volatile int             iAngle0;                /* 中心時のA/D値保存            */

/* サーボ関連2          */
volatile int             iSetAngle;
volatile int             iSetAngleAD;
volatile int             iAngleBefore2;
volatile int             iServoPwm2;

/* 槍(DCモータとボリュームad7) */
volatile int             iLancer0;				 /* 中心時のA/D値保存 			 */
volatile int			 iSetLancer;			 /* 目標のgetLancerAngle()の値   */
volatile int 			 iSetLancerAD;			 /* 目標のAD値					 */
volatile int			 iLancerPwm;			 
volatile int			 iLancerBefore;

/* TRCレジスタのバッファ */
volatile unsigned int    trcgrb_buff;            /* TRCGRBのバッファ             */
volatile unsigned int    trcgrd_buff;            /* TRCGRDのバッファ             */
volatile unsigned int    trcgrc_buff;

/* モータドライブ基板TypeS Ver.3上のLED、ディップスイッチ制御 */
volatile unsigned char   types_led;              /* LED値設定                    */
volatile unsigned char   types_dipsw;            /* ディップスイッチ値保存       */

/* 内輪差値計算用　各マイコンカーに合わせて再計算して下さい */
volatile const int revolution_difference[] = {   /* 角度から内輪、外輪回転差計算 */			
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

/* 坂道検出用 */
unsigned long cnt_saka; /* 坂道検出用タイマ */
int saka_flag; /* 1:坂道中 0:坂道ではない */
int saka0_ad; /* 平地の坂道ボリュームA/D値 */

// 0→0*2+25=25(2.3m/s) 8→ 8*2+25=41(3.7m/s)
// 1→1*2+25=27(2.5m/s) 9→ 9*2+25=43(3.9m/s)
// 2→2*2+25=29(2.6m/s) 10→10*2+25=45(4.1m/s)
// 3→3*2+25=31(2.8m/s) 11→11*2+25=47(4.3m/s)
// 4→4*2+25=33(3.0m/s) 12→12*2+25=49(4.5m/s)
// 5→5*2+25=35(3.2m/s) 13→13*2+25=51(4.6m/s)
// 6→6*2+25=37(3.4m/s) 14→14*2+25=53(4.8m/s)
// 7→7*2+25=39(3.5m/s) 15→15*2+25=55(5.0m/s)

/************************************************************************/
/* メインプログラム                                                     */
/************************************************************************/
void main( void )
{
  int i, ret;
  unsigned char b;
  
  /* データフラッシュ処理用 */
  int r;
  unsigned char c;
  /**************************/
	
  /* マイコン機能の初期化 */
  init();                             /* 初期化                       */
  _asm(" FSET I ");                   /* 全体の割り込み許可           */
  initBeepS();                        /* ブザー関連処理               */
  init_uart0_printf( SPEED_9600 );    /* UART0とprintf関連の初期化    */
  
  /* マイコンカーの状態初期化 */
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
		// 限界感度法
		KP = dipsw_get()*3;
		servoPwmOut(iServoPwm);
		if(cnt1 >= 1 ){
			printf("%ld,%d,%d\n",cnt0,KP,ad2);
			cnt1 = 0;
		}	
//  lancerPwmOut(0);  // 停止
//	timer(1000);
  	}
*/
/*
	// 指定角度への制御テスト
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
    	    /* プッシュスイッチ押下待ち */
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
        	led_out( i );                   /* LED点滅処理                  */
			break;

    	case 1:
        	/* スタートバー開待ち */
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
       		/* 通常トレース */
        	servoPwmOut( iServoPwm );		// サーボ制御		
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
	 		if( check_crossline() ) {       /* クロスラインチェック         */
				setBeepPatternS(0xa000);
				led_out(0xff);
				cnt1 = 0;
        		crank_mode = 1;
	            pattern = 21;
				break;
			}
	        if( check_rightline() ) {       /* 右ハーフラインチェック       */
				setBeepPatternS(0xc000);
        	    led_out(0xcc);
				cnt1 = 0;
            	crank_mode = 1;
	            pattern = 50;
				break;
        	}
			if( check_leftline() ) {		/* 左ハーフラインチェック       */
				setBeepPatternS(0xa000);
				led_out(0xaa);
				cnt1 = 0;
				crank_mode = 1;
				pattern = 60;
				break; 	
			}
        	break;

	    case 21:
    	    /* クロスライン通過処理 */
        	servoPwmOut( iServoPwm );
	        led_out( 0xff );
			motor_mode_f( BRAKE, BRAKE );
			motor_mode_r( BRAKE, BRAKE );
        	if( iEncoder >= 11 ) {          /* エンコーダによりスピード制御 */
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
        	/* クロスライン後のトレース、直角検出処理 */
	        servoPwmOut( iServoPwm );
    	    motor_mode_f( BRAKE, BRAKE );
			motor_mode_r( BRAKE, BRAKE );
			if( iEncoder >= 11 ) {          /* エンコーダによりスピード制御 */
        	    motor2_f( -80, -80 );
    	        motor2_r( -50, -50 );
	        } else {
    	        motor2_f( 50, 50 );
        	    motor2_r( 60, 60 );
        	}
		
    	    if( center_inp() == 1 && (sensor_inp()&0x01) == 0x01 ) { /* 右クランク？             */
        	    led_out( 0x1 );
            	cnt1 = 0;
	            pattern = 31;
    	        break;
        	}
	        if( center_inp() == 1 && (sensor_inp()&0x08) == 0x08 ) {  /* 左クランク？            */
    	        led_out( 0x2 );
        	    cnt1 = 0;
            	pattern = 41;
            	break;
        	}
        	break;

	    case 31:
    	    /* 右クランク処理 */
        	servoPwmOut( 100 );         /* 振りが弱いときは大きくする       */
			motor_mode_f(BRAKE,FREE);
    	    motor_mode_r(BRAKE,FREE);
			// diff使用禁止!!↓↓↓
			motor2_f( 100, 33 );          /* この部分は「角度計算(4WD時).xls」*/
        	motor2_r( 70, 22 );          /* で計算                           */
			if( sensor_inp() == 0x04 ) {    /* 曲げ終わりチェック           */
            	cnt1 = 0;
 	            iSensorPattern = 0;
				lEncoderLine = lEncoderTotal;
    	        crank_mode = 0;
         	    pattern = 32;
        	}
        	break;

	    case 32:
    	    /* 少し時間が経つまで待つ */
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
    	    /* 左クランク処理 */
        	servoPwmOut( -100 );         /* 振りが弱いときは大きくする       */
	        motor_mode_f(FREE,BRAKE);
    	    motor_mode_r(FREE,BRAKE);
			// diff使用禁止!!↓↓↓
			motor2_f( 33, 100 );          /* この部分は「角度計算(4WD時).xls」*/
        	motor2_r( 22, 70 );           /* で計算                           */
        	if( sensor_inp() == 0x02 ) {    /* 曲げ終わりチェック           */
            	cnt1 = 0;
            	iSensorPattern = 0;
        	    crank_mode = 0;
				lEncoderLine = lEncoderTotal;
    	        pattern = 42;
	        }
			
        	break;

	    case 42:
    	    /* 少し時間が経つまで待つ */
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
			// 右ハーフライン検出時
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
			// 右ハーフライン通過中
			servoPwmOut( iServoPwm );
		
			if( check_crossline() ){
				pattern = 21;
				crank_mode = 1;
				break;
			}
			if( iEncoder >= 11 ) {          /* エンコーダによりスピード制御 */
				motor2_f( -30, -30 );
        	    motor2_r( -30, -30 );
	        } else {
			    motor2_f( 80, 80 );
    	        motor2_r( 80, 80 );
        	}
			if(lEncoderTotal - lEncoderLine >= 109L ) {  // 約10cmすぎたか？	
				pattern = 52;	
				lEncoderLine = lEncoderTotal;
				cnt1 = 0;
				break;
			}
			break;
	
		case 52:
			// 右ハーフライン通過後
			servoPwmOut( iServoPwm );
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
			if( iEncoder >= 16 ) {          /* エンコーダによりスピード制御 */
				motor2_f( -0, -0 );
            	motor2_r( -0, -0 );
	        } else {
    	        motor2_f( 50, 50 );
        	    motor2_r( 50, 50 );
        	}
			if( ( center_inp() == 0 ) && ( sensor_inp() == 0x00 ) ) {  // 白線が消えたら曲がる
				pattern = 53;
				cnt1 = 0;
				lEncoderLine = lEncoderTotal;
				break;
			}
			break;
	
		case 53:
			// 右レーンチェンジ通過
			iSetAngle = -38;	/*中心から右に12度の位置に移動	*/
								/* +で左 -で右に曲がります      */
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
			// 新しい白線を見つけたとき
			iSetAngle = 0;		/* ハンドルを0度に戻す			*/
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
			// 通常復帰まで50%でトレース
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
			// 左ハーフライン検出時
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
			// 左ハーフライン通過中
			servoPwmOut( iServoPwm );
			if( check_crossline() ){
				pattern = 21;
				crank_mode = 1;
				break;
			}
			if( iEncoder >= 11 ) {          /* エンコーダによりスピード制御 */
				motor_f( -50, -50 );
        	    motor_r( -50, -50 );
        	} else {
				motor2_f( 80, 80 );
    	        motor2_r( 80, 80 );
        	}
			if(lEncoderTotal - lEncoderLine >= 109L ) {  // 約10cmすぎたか？	
				pattern = 62;
				break;
			}
			break;
	
		case 62:
			// 左ハーフライン通過後
			servoPwmOut( iServoPwm );
			motor_mode_f(BRAKE,BRAKE);
			motor_mode_r(BRAKE,BRAKE);
			if( iEncoder >= 11 ) {          /* エンコーダによりスピード制御 */
				motor_f( -50, -50 );
	            motor_r( -50, -50 );
    	    } else {
        	    motor2_f( 80, 80 );
            	motor2_r( 80, 80 );
        	}
			if( ( center_inp() == 0 ) && ( sensor_inp() == 0x00 ) ) {  // 白線が消えたら曲がる
				pattern = 63;			
				break;
			}
			break;
	
		case 63:
			// 左レーンチェンジ通過
			iSetAngle = 38;	/*中心から左に12度の位置に移動	*/
							/* +で左 -で右に曲がります      */
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
			// 新しい白線を見つけたとき
			iSetAngle = 0;		/* ハンドルを0度に戻す			*/
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
			// 通常復帰まで50%でトレース
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
    	    /* 停止処理 */
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
        	/* 何もしない */
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
	
	/* 緊急停止 */
	motor_mode_f( BRAKE, BRAKE );
	motor_mode_r( BRAKE, BRAKE );
    motor2_f( 0, 0 );
    motor2_r( 0, 0 );
}	


/************************************************************************/
/* R8C/38A スペシャルファンクションレジスタ(SFR)の初期化                */
/************************************************************************/
void init( void )
{
  int i;

  /* クロックをXINクロック(20MHz)に変更 */
  prc0  = 1;                          /* プロテクト解除               */
  cm13  = 1;                          /* P4_6,P4_7をXIN-XOUT端子にする*/
  cm05  = 0;                          /* XINクロック発振              */
  for (i = 0; i < 50; i++ );          /* 安定するまで少し待つ(約10ms) */
  ocd2  = 0;                          /* システムクロックをXINにする  */
  prc0  = 0;

  /* ポートの入出力設定 */

  /*  PWM(予備)       左前M_PMW       右前M_PWM       ブザー
      センサ左端      センサ左中      センサ右中      センサ右端  */
  p0   = 0x00;
  prc2 = 1;                           /* PD0のプロテクト解除          */
  pd0  = 0xf0;

  /*  センサ中心      ｽﾀｰﾄﾊﾞｰ         RxD0            TxD0
      DIPSW3          DIPSW2          DIPSW1          DIPSW0         */
  pur0 |= 0x04;                       /* P1_3?P1_0のプルアップON     */
  p1  = 0x00;
  pd1 = 0x10;

  /*  右前M_方向      ステアM_方向    ステアM_PWM     右後M_PWM
      右後M_方向      左後M_PWM       左後M_方向      左前M_方向      */
  p2  = 0x00;
  pd2 = 0xff;

  /* !---追加・変更---! */
  /*  Arduino(ANGLE)  none            none            none
      none            none            none            エンコーダA相   */
  p3  = 0x00;
  pd3 = 0xfa;

  /*  XOUT            XIN             ボード上のLED   none
      none            VREF            none            none            */
  p4  = 0x20;                         /* P4_5のLED:初期は点灯         */
  pd4 = 0xb8;

  /*  none            none            none            none
      none            none            none            none            */
  p5  = 0x00;
  pd5 = 0xff;

  /*  none            none            none            none
      none            none            Arduino(ZERO)   Arduino(MODE)   */
  p6  = 0x00;
  pd6 = 0xff;

  /*  DCモータ回転方向1   DCモータ回転方向2       CN6.4入力       CN6.5入力
      none(ｱﾅﾛｸﾞ予備) 角度VR          センサ_左ｱﾅﾛｸﾞ  センサ_右ｱﾅﾛｸﾞ  */
  p7  = 0x00;
  pd7 = 0x00;

  /*  DIPSWorLED      DIPSWorLED      DIPSWorLED      DIPSWorLED
      DIPSWorLED      DIPSWorLED      DIPSWorLED      DIPSWorLED      */
  pur2 |= 0x03;                       /* P8_7?P8_0のプルアップON      */
  p8  = 0x00;
  pd8 = 0x00;

  /*  -               -               ﾌﾟｯｼｭｽｲｯﾁ       P8制御(LEDorSW)
      右前M_Free      左前M_Free      右後M_Free      左後M_Free      */
  p9  = 0x00;
  pd9 = 0x1f;
  pu23 = 1;   // P9_4,P9_5をプルアップする

  /* タイマRBの設定 */
  /* 割り込み周期 = 1 / 20[MHz]   * (TRBPRE+1) * (TRBPR+1)
                  = 1 / (20*10^6) * 200        * 100
                  = 0.001[s] = 1[ms]
  */
  trbmr  = 0x00;                      /* 動作モード、分周比設定       */
  trbpre = 200 - 1;                   /* プリスケーラレジスタ         */
  trbpr  = 100 - 1;                   /* プライマリレジスタ           */
  trbic  = 0x06;                      /* 割り込み優先レベル設定       */
  trbcr  = 0x01;                      /* カウント開始                 */

  /* A/Dコンバータの設定 */
  admod   = 0x33;                     /* 繰り返し掃引モードに設定     */
  adinsel = 0xb0;                     /* 入力端子P7の8端子を選択      */
  adcon1  = 0x30;                     /* A/D動作可能                  */
  _asm(" NOP ");                      /* φADの1サイクルウエイト入れる*/
  adcon0  = 0x01;                     /* A/D変換スタート              */

  /* タイマRG タイマモード(両エッジでカウント)の設定 */
  timsr = 0x40;                       /* TRGCLKA端子 P3_0に割り当てる */
  trgcr = 0x15;                       /* TRGCLKA端子の両エッジでカウント*/
  trgmr = 0x80;                       /* TRGのカウント開始            */

  /* タイマRC PWMモード設定(左前モータ、右前モータ) */
  trcpsr0 = 0x40;                     /* TRCIOA,B端子の設定           */
  trcpsr1 = 0x33;                     /* TRCIOC,D端子の設定           */
  trcmr   = 0x0f;                     /* PWMモード選択ビット設定      */
  trccr1  = 0x8e;                     /* ｿｰｽｶｳﾝﾄ:f1,初期出力の設定    */
  trccr2  = 0x00;                     /* 出力レベルの設定             */
  trcgra  = TRC_MOTOR_CYCLE - 1;      /* 周期設定                     */
  trcgrb  = trcgrb_buff = trcgra;     /* P0_5端子のON幅(左前モータ)   */
  trcgrc  = trcgrc_buff = trcgra;     /* P0_7端子のON幅(予備)         */
  trcgrd  = trcgrd_buff = trcgra;     /* P0_6端子のON幅(右前モータ)   */
  trcic   = 0x07;                     /* 割り込み優先レベル設定       */
  trcier  = 0x01;                     /* IMIAを許可                   */
  trcoer  = 0x01;                     /* 出力端子の選択               */
  trcmr  |= 0x80;                     /* TRCカウント開始              */

  /* タイマRD リセット同期PWMモード設定(左後ﾓｰﾀ、右後ﾓｰﾀ、ｻｰﾎﾞﾓｰﾀ) */
    trdpsr0 = 0x08;                     /* TRDIOB0,C0,D0端子設定        */
    trdpsr1 = 0x05;                     /* TRDIOA1,B1,C1,D1端子設定     */
    trdmr   = 0xf0;                     /* バッファレジスタ設定         */
    trdfcr  = 0x01;                     /* リセット同期PWMモードに設定  */
    trdcr0  = 0x20;                     /* ソースカウントの選択:f1      */
    trdgra0 = trdgrc0 = TRD_MOTOR_CYCLE - 1;    /* 周期設定             */
    trdgrb0 = trdgrd0 = 0;              /* P2_2端子のON幅(左後モータ)   */
    trdgra1 = trdgrc1 = 0;              /* P2_4端子のON幅(右後モータ)   */
    trdgrb1 = trdgrd1 = 0;              /* P2_5端子のON幅(サーボモータ) */
    trdoer1 = 0xcd;                     /* 出力端子の選択               */
    trdstr  = 0x0d;                     /* TRD0カウント開始             */
}

/************************************************************************/
/* タイマRB 割り込み処理                                                */
/************************************************************************/
#pragma interrupt /B intTRB(vect=24)
void intTRB( void )
{
  unsigned char b,c,d;
  unsigned int i;
  unsigned int  v;
  _asm(" FSET I ");   /* タイマRB以上の割り込み許可   */

  cnt0++;
  cnt1++;
  cnt_run++;
  check_sen_cnt++;
  check_enc_cnt++;
  check_cross_cnt++;
  cnt_saka++;

  /* サーボモータ制御 */
  servoControl();
  servoControl2();
  //lancerControl();
  
  /* ブザー処理 */
  beepProcessS();
  if( pattern == 1){
	 servoPwmOut(iServoPwm);
  }
  p4_5=p3_0;
  //c = ~p7_5;
  // エンコーダの信号をLEDに表示
	
  //led_out((b<<3)|(c<<7)|(d<<2));

  /* 10回中1回実行する処理 */
  iTimer10++;
  switch ( iTimer10 ) {
    case 1:
      /* エンコーダ制御 */
      i = trg;
      iEncoder       = i - uEncoderBuff;
      lEncoderTotal += iEncoder;
      if ( iEncoder > iEncoderMax ) {
        iEncoderMax = iEncoder;
      }
      uEncoderBuff   = i;
      break;

    case 2:
      /* スイッチ読み込み準備 */
      p9_4 = 0;                       /* LED出力OFF                   */
      pd8  = 0x00;
      break;

    case 3:
      /* スイッチ読み込み、LED出力 */
      types_dipsw = ~p8;              /* ﾄﾞﾗｲﾌﾞ基板TypeS Ver.3のSW読み込み*/
      p8  = types_led;                /* ﾄﾞﾗｲﾌﾞ基板TypeS Ver.3のLEDへ出力*/
      pd8 = 0xff;
      p9_4 = 1;                       /* LED出力ON                    */
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
      /* iTimer10変数の処理 */
      iTimer10 = 0;
      break;
  }
}

/************************************************************************/
/* タイマRC 割り込み処理                                                */
/************************************************************************/
#pragma interrupt intTRC(vect=7)
void intTRC( void )
{
  trcsr &= 0xfe;  /* フラグクリア */

  /* タイマRC　デューティ比の設定 */
  trcgrb = trcgrb_buff;
  trcgrc = trcgrc_buff;
  trcgrd = trcgrd_buff;
}

void timer( unsigned long timer_set ){
	cnt0 = 0;
	while(cnt0 < timer_set );	
}

/************************************************************************/
/* アナログセンサ基板TypeS Ver.2のデジタルセンサ値読み込み              */
/* 引数　 なし                                                          */
/* 戻り値 左端、左中、右中、右端のデジタルセンサ 0:黒 1:白              */
/************************************************************************/
unsigned char sensor_inp( void )
{
    unsigned char sensor;

    sensor = ~p0 & 0x0f;

    return sensor;
}

/************************************************************************/
/* アナログセンサ基板TypeS Ver.2の中心デジタルセンサ読み込み            */
/* 引数　 なし                                                          */
/* 戻り値 中心デジタルセンサ 0:黒 1:白                                  */
/************************************************************************/
unsigned char center_inp( void )
{
    unsigned char sensor;

    sensor = ~p1_7 & 0x01;

    return sensor;
}

/************************************************************************/
/* アナログセンサ基板TypeS Ver.2のスタートバー検出センサ読み込み        */
/* 引数　 なし                                                          */
/* 戻り値 0:スタートバーなし 1:スタートバーあり                         */
/************************************************************************/
unsigned char startbar_get( void )
{
    unsigned char sensor;

    sensor = ~p1_6 & 0x01;

    return sensor;
}

/************************************************************************/
/* マイコンボード上のディップスイッチ値読み込み                         */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0?15                                              */
/************************************************************************/
unsigned char dipsw_get( void )
{
    unsigned char sw;

    sw = p1 & 0x0f;                     /* P1_3?P1_0読み込み           */

    return sw;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3上のディップスイッチ値読み込み          */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0?255                                             */
/************************************************************************/
unsigned char dipsw_get2( void )
{
    /* 実際の入力はタイマRB割り込み処理で実施 */
    return types_dipsw;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3上のプッシュスイッチ値読み込み          */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0:OFF 1:ON                                         */
/************************************************************************/
unsigned char pushsw_get( void )
{
    unsigned char sw;

    sw = ~p9_5 & 0x01;

    return sw;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3のCN6の状態読み込み                     */
/* 引数　 なし                                                          */
/* 戻り値 0?15                                                         */
/************************************************************************/
unsigned char cn6_get( void )
{
    unsigned char data;

    data = p7 >> 4;

    return data;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3のLED制御                               */
/* 引数　 8個のLED制御 0:OFF 1:ON                                       */
/* 戻り値 なし                                                          */
/************************************************************************/
void led_out( unsigned char led )
{
    /* 実際の出力はタイマRB割り込み処理で実施 */
    types_led = led;
}

/************************************************************************/
/* 後輪の速度制御                                                       */
/* 引数　 左モータ:-100?100 , 右モータ:-100?100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_r( int accele_l, int accele_r )
{
    int sw_data;

    sw_data  = dipsw_get() + 5;         /* ディップスイッチ読み込み     */
    accele_l = accele_l * sw_data / 20;
    accele_r = accele_r * sw_data / 20;
	
    /* 左後モータ */
	accele_l = -accele_l;
    if( accele_l >= 0 ) {
        p2_1 = 0;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
    } else {
        p2_1 = 1;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
    }

    /* 右後モータ */
    if( accele_r >= 0 ) {
        p2_3 = 0;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
    } else {
        p2_3 = 1;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
    }
}

/************************************************************************/
/* 後輪の速度制御2 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100?100 , 右モータ:-100?100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor2_r( int accele_l, int accele_r )
{
    /* 左後モータ */
	accele_l = -accele_l;
    if( accele_l >= 0 ) {
        p2_1 = 0;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
    } else {
        p2_1 = 1;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
    }

    /* 右後モータ */
    if( accele_r >= 0 ) {
        p2_3 = 0;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
    } else {
        p2_3 = 1;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
    }
}

/************************************************************************/
/* 前輪の速度制御                                                       */
/* 引数　 左モータ:-100?100 , 右モータ:-100?100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_f( int accele_l, int accele_r )
{
    int sw_data;

    sw_data  = dipsw_get() + 5;         /* ディップスイッチ読み込み     */
    accele_l = accele_l * sw_data / 20;
    accele_r = accele_r * sw_data / 20;
	accele_l = -accele_l;

    /* 左前モータ */
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

    /* 右前モータ */
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
/* 前輪の速度制御2 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100?100 , 右モータ:-100?100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor2_f( int accele_l, int accele_r )
{
	/* 左前モータ */
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

    /* 右前モータ */
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
/* 後モータ停止動作（ブレーキ、フリー）                                 */
/* 引数　 左モータ:FREE or BRAKE , 右モータ:FREE or BRAKE               */
/* 戻り値 なし                                                          */
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
/* 前モータ停止動作（ブレーキ、フリー）                                 */
/* 引数　 左モータ:FREE or BRAKE , 右モータ:FREE or BRAKE               */
/* 戻り値 なし                                                          */
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
/* サーボモータ制御                                                     */
/* 引数　 サーボモータPWM：-100?100                                    */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void servoPwmOut( int pwm )
{

    /* ボリューム値により左リミット制御 */
//    if( ad2 >= 986 && pattern >= 11 ){
//        pwm = 0;
//    }
    /* ボリューム値により右リミット制御 */
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
/* クロスライン検出処理                                                 */
/* 引数　 なし                                                          */
/* 戻り値 0:クロスラインなし 1:あり                                     */
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
/* 右ハーフライン検出処理 												*/
/* 引数 なし 															*/
/* 戻り値 0:ハーフラインなし 1:あり 									*/
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
/* 左ハーフライン検出処理                                               */
/* 引数　 なし                                                          */
/* 戻り値 0:左ハーフラインなし 1:あり                                   */
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
/* サーボ角度取得                                                       */
/* 引数　 なし                                                          */
/* 戻り値 入れ替え後の値                                                */
/************************************************************************/
int getServoAngle( void )
{
    return( (-ad2) - iAngle0 );
}

/************************************************************************/
/* アナログセンサ値取得                                                 */
/* 引数　 なし                                                          */
/* 戻り値 センサ値                                                      */
/************************************************************************/
int getAnalogSensor( void )
{
    int ret;

    ret = ad1 - ad0;                    /* アナログセンサ情報取得       */

    if( !crank_mode ) {
        /* クランクモードでなければ補正処理 */
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
            /* センサ右寄り */
            ret = -700;
            if( sensor_inp() == 0x04 ) {
                iSensorPattern = 0;
            }
            break;

        case 2:
            /* センサ左寄り */
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
/* サーボモータ制御                                                     */
/* 引数　 なし                                                          */
/* 戻り値 グローバル変数 iServoPwm に代入                               */
/************************************************************************/
void servoControl( void )
{
    int i, iRet, iP, iD;

    i = getAnalogSensor();              /* センサ値取得                 */

    /* サーボモータ用PWM値計算 */
    iP = KP * i;                        /* 比例                         */
    iD = KD * (iSensorBefore - i );     /* 微分(目安はPの5?10倍)       */
	
    iRet = iP - iD;
    iRet /= 16;

    /* PWMの上限の設定 */
    if( iRet >  SERVO_PWM_MAX ) iRet =  SERVO_PWM_MAX;/* マイコンカーが安定したら     */
    if( iRet < -SERVO_PWM_MAX ) iRet = -SERVO_PWM_MAX;/* 上限を70くらいにしてください */
    iServoPwm = iRet;

    iSensorBefore = i;                  /* 次回はこの値が1ms前の値となる*/
}

/************************************************************************/
/* サーボモータ2制御                                                    */
/* 引数　 なし                                                          */
/* 戻り値 グローバル変数 iServoPwm2 に代入                              */
/************************************************************************/
void servoControl2( void )
{
    int i, j, iRet, iP, iD;

    i = iSetAngle;
    j = getServoAngle();

    /* サーボモータ用PWM値計算 */
    iP = 20 * ( j - i );                /* 比例                         */
    iD = 50 * ( iAngleBefore2 - j );    /* 微分(目安はPの5?10倍)       */
    iRet = iP - iD;
    iRet /= 2;

    /* PWMの上限の設定 */
    if( iRet >  SERVO_PWM_MAX ) iRet =  SERVO_PWM_MAX;/* マイコンカーが安定したら     */
    if( iRet < -SERVO_PWM_MAX ) iRet = -SERVO_PWM_MAX;/* 上限を70くらいにしてください */
    iServoPwm2 = iRet;

    iAngleBefore2 = j;                  /* 次回はこの値が1ms前の値となる*/
}

/************************************************************************/
/* 外輪のPWMから、内輪のPWMを割り出す　ハンドル角度は現在の値を使用     */
/* 引数　 外輪PWM                                                       */
/* 戻り値 内輪PWM                                                       */
/************************************************************************/
int diff( int pwm )
{
    int i, ret;

    i  = getServoAngle() / 5;    /* 1度あたりの増分で割る        */
    if( i <  0 ) i = -i;
    if( i > 45 ) i = 45;
    ret = revolution_difference[i] * pwm / 100;

    return ret;
}

/************************************************************************/
/* 坂道チェック(リミットスイッチ)										*/
/* 引数 なし															*/
/* 戻り値 なし 															*/
/* メモ 坂道と判断するとsaka_flag = 1 坂道でないなら 0 					*/
/************************************************************************/
void sakaSyori( void )
{
	static int saka_pattern = 0;
	int saka;
	
	saka = ad4 - saka0_ad;
	
	switch( saka_pattern ){
		case 0:
			// 上り坂、下り坂のチェック
			if( saka <= -10 ) { // 上りはA/D値が小さくなる
				cnt_saka = 0;
				saka_pattern = 1; // 上り坂処理
			}
			break;
		
		case 1:
			// 上り坂 少し時間をおいて、再度チェック
			if( cnt_saka >= 10 ) {
				if( saka <= -10 ) { // 反応あり
					setBeepPatternS( 0xcc00 );
					saka_flag = 1; // 坂フラグをON!
					cnt_saka = 0;
					saka_pattern = 2;
				} else {
					// 反応なしなら誤動作と判断して戻る
					saka_pattern = 0;
				}
			}
			break;

		case 2:
			// 上り坂 頂点の中間くらいから上り坂終わりのチェック
			if( cnt_saka >= 100 ) {
				cnt_saka = 0;
				saka_pattern = 3;
			}
			break;
		
		case 3:
			// 上り坂 上りの頂点をチェック
			if( saka >= 10 ) {
				cnt_saka = 0;
				saka_pattern = 4;
			}
			break;
		
		case 4:
			// 上り坂 少し時間をおいて、再度チェック
			if( cnt_saka >= 10 ) {
				if( saka >= 10 ) { // 反応あり
					setBeepPatternS( 0xff00 );
					saka_flag = 0; // 坂フラグをOFF!
					cnt_saka = 0;
					saka_pattern = 5;
				} else {
				// 反応なしなら誤動作と判断して戻る
				saka_pattern = 3;
			}
		}
		break;

		case 5:
			// 上り坂終わり 少し進ませて通常走行へ
			if( cnt_saka >= 200 ) {
				cnt_saka = 0;
				saka_pattern = 0;
			}	
			break;
		}
}

/************************************************************************/
/* ライントレース関数本体                                               */
/* 引数   デューティ比(0?100)                                          */
/* 戻り値 なし                                                          */
/* 備考 機体に合わせてパラメータを書き換えてください                    */
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

    if ( iEncoder >= _3MS ) { // 3.0m/s以上なら
      motor2_f( -100, -100 );
      motor2_r(   0,   0 );
    } else {
      motor2_f( 40, diff(40) );
      motor2_r( -40, diff(40) );
    }
  } else if ( i < -50 ) {
    motor_mode_f( BRAKE, BRAKE );
    motor_mode_r( BRAKE, BRAKE );

    if ( iEncoder >= _3MS ) { // 3.0m/s以上なら
      motor2_f( -100, -100 );
      motor2_r(   0,   0 );
    } else {
      motor2_f( diff(40), 40 );
      motor2_r( -diff(40), 40 );
	} 
  
  }else if ( i > 15 ) {
    if (iEncoder >= _3MS ) { // 3.0m/s以上なら
      motor_mode_f( BRAKE, BRAKE );
      motor_mode_r( BRAKE, BRAKE );
      motor2_f( -70, -70 );
      motor2_r( 50, -50 );
    } else if ( iEncoder >= _2MS ) { // 2.0m/s以上なら
      motor_mode_f( BRAKE, FREE );
      motor_mode_r( BRAKE, FREE );
      motor2_f( 45, 0 );  // 内輪を0%にして、カーブを曲がりやすくする
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
    } else if ( iEncoder >= _2MS ) { // 2.0m/s以上なら
      motor_mode_f( FREE, BRAKE );
      motor_mode_r( FREE, BRAKE );
      motor2_f( 0, 45 );         // 内輪を0%にして、カーブを曲がりやすくする
      motor2_r( 0, 45 );
    } else {
      motor_mode_f( FREE, BRAKE );
      motor_mode_r( FREE, BRAKE );
      motor2_f( diff(50), 50 );
      motor2_r( -diff(50), 50 );
    }
  } else { 
	
    
	if ( iEncoder >= (dipsw_get() * 2 + 35) ) { // 50(2.0m/s)?138(5.0m/s)  // 安定値:15*2+14 = 44
      // dip_swの値↓
      // 0→0*2+25=25(2.3m/s) 8→ 8*2+25=41(3.7m/s)
      // 1→1*2+25=27(2.5m/s) 9→ 9*2+25=43(3.9m/s)
      // 2→2*2+25=29(2.6m/s) 10→10*2+25=45(4.1m/s)
      // 3→3*2+25=31(2.8m/s) 11→11*2+25=47(4.3m/s)
      // 4→4*2+25=33(3.0m/s) 12→12*2+25=49(4.5m/s)
      // 5→5*2+25=35(3.2m/s) 13→13*2+25=51(4.6m/s)
      // 6→6*2+25=37(3.4m/s) 14→14*2+25=53(4.8m/s)
      // 7→7*2+25=39(3.5m/s) 15→15*2+25=55(5.0m/s)
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
			// dip_swの値↓
      // 0→0*2+25=25(2.3m/s) 8→ 8*2+25=41(3.7m/s)
      // 1→1*2+25=27(2.5m/s) 9→ 9*2+25=43(3.9m/s)
      // 2→2*2+25=29(2.6m/s) 10→10*2+25=45(4.1m/s)
      // 3→3*2+25=31(2.8m/s) 11→11*2+25=47(4.3m/s)
      // 4→4*2+25=33(3.0m/s) 12→12*2+25=49(4.5m/s)
      // 5→5*2+25=35(3.2m/s) 13→13*2+25=51(4.6m/s)
      // 6→6*2+25=37(3.4m/s) 14→14*2+25=53(4.8m/s)
      // 7→7*2+25=39(3.5m/s) 15→15*2+25=55(5.0m/s)
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
/* 過去の横線の検出回数に応じて標的を見分ける                           */
/* 引数   なし                                                          */
/* 戻り値 変数hyouteki_flagに(平行標的:0 垂直標的:1)が入る              */
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
/* 数値をある範囲から別の範囲に変換(Arduinoのmap関数と同等)             */
/*                                                                      */
/* 引数   x: 変換したい数値                                             */
/*        in_min: 現在の範囲の下限                                      */
/*        int_max: 現在の範囲の上限                                     */
/*        out_min: 変換後の範囲の下限                                   */
/*        out_max: 変換後の範囲の上限                                   */
/*                                                                      */
/* 戻り値 変換後の数値 (long)                                           */
/************************************************************************/
long map( long x, long in_min, long in_max, long out_min, long out_max ) {

  return ( x - in_min ) * ( out_max - out_min ) / ( in_max - in_min ) + out_min;

}

/************************************************************************/
/* 槍角度取得	                                                        */
/* 引数　 なし                                                          */
/* 戻り値 入れ替え後の値                                                */
/************************************************************************/
int getLancerAngle( void )
{
    return( ad4 - iLancer0 );  // TypeS基板AN16(p7_4)のR13を外す
}

/************************************************************************/
/* サーボモータ制御(槍)	相対位置での制御                               */
/* 引数　 サーボモータPWM：-100?100                                    */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void lancerPwmOut( int pwm )
{
	int accele;
   	if ( pwm >= 0 ) {
    	p6_6 = 0;
    } else {
    	p6_6 = 1;
  	}
	accele = map(pwm, -100, 100, 0, 100);  // -100から100を 0～100に変換
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
/* サーボモータ制御(槍) 絶対位置での制御                               */
/* 引数　 サーボモータPWM：-100?100                                    */
/*        50で停止、100で正転100%、0で逆転100%v                        */
/* 戻り値 なし                                                         */
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
/* サーボモータ制御 槍       											*/
/* 引数 なし 															*/
/* 戻り値 グローバル変数 iLancerPwm に代入 								*/
/************************************************************************/
void lancerControl( void )
{
	int i, j, iRet, iP, iD;
	
	/* !!追加・変更!!! */
	// i = iSetAngle; 						/* 設定したい角度 	*/
	// j = getServoAngle(); 				/* 現在の角度 		*/
	
	
	i = iSetLancer; 						/* 設定したい角度 	*/
	j = ad4;				 			/* 現在の角度 		*/
	 
	/*     P                            D                      */
  	iRet = (L_KP * i) - ( L_KD * (i - iLancerBefore));
    iRet /= 2;
	
	if( iRet >  LANCER_PWM_MAX ) iRet =  LANCER_PWM_MAX;	/* マイコンカーが安定したら 	*/
	if( iRet < -LANCER_PWM_MAX ) iRet = -LANCER_PWM_MAX; 	/* 上限を90くらいにしてください */
	
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