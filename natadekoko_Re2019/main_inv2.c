/* advanceマイコンカー用プログラム */
// main_inv2.c(このファイル)が完走用

/* 更新履歴: */
// 201220 TypeS基板+microSDを追加
// 210326 TypeS基板+microSD+LCD(SC1602系)を追加
// 210704 スタートバー検知でTypeSのLED(0x04)が点灯/消灯
// 210722 基板をTypeDに変更
// 210722 TypeSのLED(0x04)だと分かりにくかったのでスタートバー検知でTypeDのLED(0x80)が点灯/消灯
// 210723 CSVの上部に走行時の設定パラメータを表示できるようにした
// 210731 コーナー走行時の駆動モーター制御関数corner_run(後輪PWM)を追加
// 210801 内蔵データフラッシュから前回パラメータを起動時に読み込むようにする
// 210828 STARTボタンで前回パラメータを設定->走行できるように変更
 

/*======================================*/
/* インクルード */
/*======================================*/
#include <stdio.h>
#include <stdlib.h> 		/* abs関数を使用 */
#include "sfr_r838a.h" 	/* R8C/38A SFR の定義ファイル */
#include "types3_beep.h"	/* ブザー追加 */
#include "microsd_lib.h"
#include "switch_lib.h"
#include "lcd_lib.h"
#include "data_flash_lib.h" /* 前回パラメータ記憶用にデータフラッシュを使用 */

/*======================================*/
/* シンボル定義 */
/*======================================*/
/* 定数設定 */
#define TRC_MOTOR_CYCLE 20000 /* 左前,右前モータ PWM の周期 */
/* 50[ns] * 20000 = 1.00[ms] */
#define TRD_MOTOR_CYCLE 20000 /* 左後,右後,ｻｰﾎﾞﾓｰﾀ PWM の周期 */
/* 50[ns] * 20000 = 1.00[ms] */
#define SERVO_STEP 3 		/* 1°あたりの数値 3 */
#define SERVO_CENTER 254 	/* 中心 */
#define FREE  1			/* モータモード フリー */
#define BRAKE 0 			/* モータモード ブレーキ */
#define LEFT  1			/* クランク左 */
#define RIGHT 2 			/* クランク右 */

/* データフラッシュ関連  */
#define     DF_ADDR_START   0x3000/* 書き込み開始アドレス         */

#define     DF_PARA_SIZE    256  /* DataFlashパラメータ数        */
#define     DF_CHECK        0    /* データフラッシュチェック     */
#define     DF_DATA         1		/* データ                       */

#define 	 DF_TESTSPEED    0x01
#define     DF_CURVE_SPEED  0x02
#define     DF_LANE_L       0x03
#define     DF_LANE_R       0x04
#define     DF_CRANK_SPEED  0x05
#define     DF_ENC_END      0x06

/* その他 */
#define     BEEP            1           /* 音のON or OFFの設定          */
#define     RUNNING_TIME        31000L  /* 走行時間(ms)                 */
//#define    ENC_END       1200L*(()+1)
volatile unsigned long ENC_END = 1;/* 走行距離[m] 0以外をとりあえず入れておく:param_Settings関数で指定*/
/*======================================*/
/* プロトタイプ宣言 */
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
void corner_run(int max_power); //コーナー走行時の駆動モーター制御

/*======================================*/
/* グローバル変数の宣言 */
/*======================================*/
/* 通常設定: */
volatile int pattern = 0; 	/* マイコンカー動作パターン */
volatile int crank_mode = 0; /* 1:クランクモード 0:通常 */
volatile unsigned long cnt0 = 0; /* タイマ用 */
volatile unsigned long cnt1 = 0; /* タイマ用 */
volatile int h;

/* エンコーダ関連 */
volatile int iTimer10; 				 /* 10ms カウント用 */
volatile unsigned long lEncoderTotal; /* 積算値保存用 */
volatile unsigned long lEncoderCorner; /* pattern11~14 100mm検出用 */
volatile unsigned long lEncoderCrank; /*クランク時の距離 lEncoderTotal - lEncoderCrank */
volatile int iEncoder; 				 /* 10ms 毎の最新値 */
volatile unsigned int uEncoderBuff; 	 /* 計算用 割り込み内で使用 */
volatile int speed_target; /* 目標速度[iEncoder値] */
volatile int corner_speed; /* カーブでのスピード[iEncoder値] */
  
/* サーボ関連 */
volatile int iSensorBefore; 		/* 前回のセンサ値保存 */
volatile int iServoPwm; 			/* サーボＰＷＭ値 */
volatile int iAngle0; 			/* 中心時の A/D 値保存 */

volatile signed int kakudo;  /* 角度[°] 		*/
volatile signed int angle;	/* 角度[AD値] 	*/

/* センサ関連 */
volatile int iSensorPattern; 		/* センサ状態保持用 */

/* TRC レジスタのバッファ */
volatile unsigned int trcgrb_buff; 	/* TRCGRB のバッファ */
volatile unsigned int trcgrd_buff; 	/* TRCGRD のバッファ */
volatile unsigned int trcgrc_buff;	/* TRCGRC のバッファ */

/* モータドライブ基板 TypeS Ver.3 上の LED、ディップスイッチ制御 */
volatile unsigned char types_led; 	/* LED 値設定 */
volatile unsigned char types_dipsw; 	/* ディップスイッチ値保存 */

/* レーンチェンジ */
volatile unsigned char Lane_Change; /* 1 左 0 右 */

/*モータ関係 */
volatile unsigned char M_FreeMoter;

/* サーボ関係 2 */
volatile int iSetAngle; 			/* 設定したい角度(AD 値) */
volatile int iAngleBefore2; 		/* 前回の角度保存 */
volatile int iServoPwm2; 		/* サーボＰＷＭ値 */
volatile int gain; 				//強さ調整

/* 速度調整 */
volatile unsigned char lanechangespeed;
volatile unsigned char cornerspeed;
volatile unsigned char crankspeed;
volatile unsigned char streetspeed;

/* コースアウト */
unsigned long causeout;
volatile int acceleFree;
volatile int endFlag = 0; /* 走行終了か？ 1で走行終了 */

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
volatile const int r_in[60]={100,98,97,95,93,92,90,88,87,85,84,82,81,79,78,76,75,73,72,70,69,67,66,65,63,62,60,59,58,56,55,53,52,51,49,48,46,45,43,42,40,39,37,36,34,33,31,30,28,26,25,23,21,20,18,16,14,12,11,9};
volatile const int f_in[60]={100,98,97,96,94,93,92,90,89,88,87,86,85,84,83,82,81,80,79,79,78,77,77,76,75,75,74,74,73,72,72,72,71,71,70,70,70,69,69,69,69,69,68,68,68,68,68,68,68,68,68,68,68,69,69,69,69,70,70,70};
volatile const int f_out[60]={100,99,98,98,97,96,95,95,94,94,93,93,92,92,92,91,91,91,90,90,90,90,90,89,89,89,89,89,89,89,89,89,90,90,90,90,90,91,91,91,92,92,92,93,93,94,94,95,96,96,97,98,99,99,100,100,100,100,100,100};

signed int accel_in;		/* コーナー時 前内側モータ制御値(accel) */
signed int accel_out;		/* コーナー時 前外側モータ制御値(accel) */
signed int accel_in_b;	/* コーナー時 後内側モータ制御値(accel) */
signed int accel_out_b;	/* コーナー時 後内側モータ制御値(accel) */

/* microSD関連変数 */
volatile const char *C_DATE = __DATE__; /* コンパイルした日付 */
volatile const char *C_TIME = __TIME__; /* コンパイルした時間 */
volatile int msdFlag; /* 1:データ記録 0:記録しない */
volatile int msdError; /* エラー番号記録 */

/* データフラッシュ関連 */
volatile signed char     data_buff[ DF_PARA_SIZE ];  /* 一時保存エリア           */

/* 坂道検出用 */
volatile unsigned long cnt_saka; /* 坂道検出用タイマ */
volatile int saka_flag; /* 1:坂道中 0:坂道ではない */
volatile int saka0_ad; /* 平地の坂道ボリュームA/D値 */
volatile const int dipsw1_pattern[] = { 9, 10, 11, 12, 13, 14, 15, 16};
volatile const int dipsw2_pattern[] = { 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25};
volatile const int dipsw3_pattern[] = { 10, 11, 12, 13};
volatile const int dipsw4_pattern[] = { 5, 6, 7, 8};
volatile const int dipswf_pattern[] = { 0, 1};

/* getSw関数の値が変化しているか確認用 */
volatile int swnum = 22;
volatile int beforeswnum;
/************************************************************************/
/* メインプログラム */
/************************************************************************/
void main( void ) {
  int i=0, ret, x=0;
  char fileName[ 8+1+3+1 ]; /* 名前＋'.'＋拡張子＋'\0' */
  unsigned char b;
  
  /* マイコン機能の初期化 */
  init(); /* 初期化 */
  readDataFlash(DF_ADDR_START, data_buff, DF_PARA_SIZE); /* 0x3000番地以降データフラッシュ読み込み  */
  
  //data_buff配列のDF_CHECK番目（0番目）のデータが0x38かチェック
  //0x38なら各data_buffには前回保存したデータが読み込まれる
  if( data_buff[DF_CHECK] != 0x38 ) {
	  data_buff[DF_CHECK] = 0x38;	// ・DF_CHECK番目（0番目）に0x38を
	  data_buff[DF_DATA]  = 0;	// ・DF_DATA番目(1番目)に0を設定
  }										
  
  setMicroSDLedPort( &p6, &pd6, 0 ); /* microSDモニタLED設定 */
  _asm(" fset I "); /* 全体の割り込み許可 */
  initBeepS(); /* ブザー関連処理 */
  initLcd(); /* LCD初期化 */
  initSwitch(); /* スイッチ初期化 */
  
  /* マイコンカーの状態初期化 */
  motor_mode_f( BRAKE, BRAKE ); //BRAKE
  motor_mode_r( BRAKE, BRAKE );
  motor_f( 0, 0 );
  motor_r( 0, 0 );
  servoPwmOut( 0 );
  lcdPosition(0,0 );
  lcdPrintf(C_DATE);
  lcdPosition( 0, 1 );
  lcdPrintf( C_TIME );
  /* microSD初期化 */
  ret = initMicroSD();
  if( ret != 0x00 ){
	  msdError = 1;
	  lcdPosition(0,0);
	  lcdPrintf("microSD Error!  ");
	  lcdPosition(0,1);
	  lcdPrintf("no=%d CONECT ERR",ret);
  }
  
  /* FAT32でマウント */
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
	/* microSDにエラーがなければLCDに表示 */
	lcdPosition(0,0);
	lcdPrintf("microSD OK     ");
	lcdPosition(0,1);
	lcdPrintf("               ");
	timer(1000);
	/* 坂道の基準位置を代入					*/
	saka0_ad = ad5;
	lcdPosition(0,0);
	lcdPrintf("saka0_ad init. ");
	lcdPosition(0,1);
	lcdPrintf("ad= %d      ",saka0_ad);
	timer(1000);
  }
  
  if( msdError != 0 ){
	/* microSD処理にエラーがあれば3秒間LEDの点滅方法を変える */
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
  
  /* マイコンカーの状態初期化 */
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
  
  /* 無限ループ前にLCD/SWで速度設定 */
  while( !x ){
  	x=paramSettings(); // 設定終了で1
  }
  
  lcdPosition(0,0);
  lcdPrintf("DATA_FLASH   ");
  lcdPosition(0,1);
  lcdPrintf("Loading Now...");
  blockEraseDataFlash( DF_ADDR_START );
  ret = writeDataFlash( DF_ADDR_START, data_buff, DF_PARA_SIZE );
  
  if(ret == 0){ // 書き込みエラーなら
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
	
    //メニュー以外
    if (pattern != 1 && pattern != 2 && pattern != 3 && pattern != 4 && pattern != 5 && pattern != 6 && pattern != 7
        && pattern != 8 && pattern != 9 && pattern != 10 && pattern <= 100) {
      //通常トレースでのコースアウト
      if (pattern != 32 && pattern != 33 && pattern != 42 && pattern != 43 && pattern != 61 && pattern != 62 &&
          pattern != 63 && pattern != 71 && pattern != 72 && pattern != 73) {
        if ((lEncoderTotal - causeout) >= 5000) {
          pattern = 101;
		  if (endFlag == 0){
		  	endFlag = 1;	  
		  }
        }
      }
      //アクションでのコースアウト
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
		// 坂道処理
		sakaSyori(); // 坂道処理をmain関数のwhileﾙｰﾌﾟ内に入れる
		if((lEncoderTotal >= (data_buff[DF_ENC_END]*1091L)) || endFlag == 1){ // エンコーダがdata_buff[DF_ENC_END]なら停止
			if( pattern == 11 || pattern == 12 || pattern == 13 || pattern == 14){
				pattern = 101;	
			}
		}
	}
	
    switch ( pattern ) {
      case 0:
        /* プッシュスイッチ押下待ち */
       gain = 0;
       servoPwmOut( 0 );
       lEncoderTotal = 0;
       lEncoderCrank = 0;
	   lEncoderCorner = 0;
       causeout = 0;
       iAngleBefore2 = 0;
       M_FreeMoter = 0;
	   
//        streetspeed = (dipsw2_pattern[dipsw_get2()] + 5); //直線
//        cornerspeed = dipsw2_pattern[dipsw_get2()]; //曲線
//        lanechangespeed = dipsw3_pattern[dipsw_get3()]; //車線変更
//        crankspeed = dipsw4_pattern[dipsw_get4()];//クランク
		
		if ( pushsw_get() && cnt1 >= 100) {
			if( msdError == 0 ){
				/* microSDの空き領域へ書き込み */
				i++;
				if(i >= 10000 ) i = 1;
				ret = writeMicroSDNumber(i);
				if( ret == -1 ){
					msdError = 4;	
				}	
				else{
					/* ファイル名変換 */
					sprintf( fileName,"log%04d.csv",i);	
				}
			}
			if( msdError == 0 ){
				/* ファイルのタイムスタンプセット */
				setDateStamp( getCompileYear( C_DATE ),
					getCompileMonth( C_DATE ), getCompileDay( C_DATE ) );
				setTimeStamp( getCompileHour( C_TIME ),
					getCompilerMinute( C_TIME ), getCompilerSecond( C_TIME ) );
				
				/* 書き込みファイル名作成 */
				// 書き込みしたい時間[ms] : x = 10[ms] : 64バイト
				// 60000msなら、x = 60000 * 64 / 10 = 384000
				// 結果は512の倍数になるように繰り上げする。
				ret = writeFile( fileName, 384000 );
				if( ret != 0x00 ) msdError = 11;
				
				// microSD書き込み
				msdPrintf("[TB-001]TypeD Log Data\n");
				while( checkMsdPrintf()); // msdPrintf完了待ち
				msdPrintf("Compile Date: ");
				while( checkMsdPrintf()); // msdPrintf完了待ち
				msdPrintf(C_DATE);
				while( checkMsdPrintf()); // msdPrintf完了待ち
				msdPrintf(" Time: ");
				while( checkMsdPrintf()); // msdPrintf完了待ち
				msdPrintf(C_TIME);
				while( checkMsdPrintf()); // msdPrintf完了待ち
				msdPrintf("\nSPEED=%d,CORNER_S=%d,L_LANE=%d,R_LANE=%d,CRANK=%d,ENC_END=%d",data_buff[DF_TESTSPEED], data_buff[DF_CURVE_SPEED], data_buff[DF_LANE_L], data_buff[DF_LANE_R], data_buff[DF_CRANK_SPEED], data_buff[DF_ENC_END]);
				while( checkMsdPrintf()); // msdPrintf完了待ち
				msdPrintf("\n\nLineNo,Pattern,Sensor,Center,Analog,Angle,Encoder,LeftLine,RightLine,CrossLine,saka_pot,saka_flag,lEncoderTotal\n");
				while( checkMsdPrintf()); // msdPrintf完了待ち
				
				/*
				// メモ用
				line_no,			// 行番号
				pattern,			// 動作パターン
				sensor_inp(),  		// デジタル(4bit)
				center_inp()+'0', 	// デジタル(中心)
				getAnalogSensor(),  // アナログセンサ
				getServoAngle(),	// ボリューム(ステアリング角度)
				iEncoder,          	// エンコーダ
				check_zlineL(),  	// デジタル左
				check_zlineR(), 	// デジタル右
				check_crossline(), 	// クロスラインチェック
				ad5,				// 坂道検出用ポテンショメータ 
				saka_flag			// 坂検出フラグ
				lEncoderTotal      //  エンコーダの積算値
				*/
	
			}
			
        	M_FreeMoter = dipswf_pattern[dipsw_getf()]; //1 でフリーモード 0 で通常
          	setBeepPatternS( 0xcc00 );
          	cnt1 = 0;
          	pattern = 1;
          	break;
		}
  		i = ((cnt1 / 200) % 2 + 1) | (startbar_get()<<7&0x80); // スタートバー反応確認
    	led_out( i ); /* LED 点滅処理 */
		break;
		
      case 1:
        /* スタートバー開待ち */
        gain = 10;
        servoPwmOut( iServoPwm / 2 );
        causeout = lEncoderTotal;
		 timer(4000);
        if ( !startbar_get() ) {
		  iAngle0 = getServoAngle(); /* 0 度の位置記憶 */
		  led_out( 0x0 );
		  if( msdError == 0 ) msdFlag = 1; /* データ記録開始 */
		  saka0_ad = ad5; /* 平地の坂道ボリュームA/D値記憶*/
		  cnt1 = 0;
		  pattern = 11; //11 通常走行 3 ステアリング確認 4 モーター速度
		  break;
        }
		
        led_out( 1 << (cnt1 / 50) % 4 | (startbar_get()<<7&0x80)); // スタートバー反応確認
        break;
		
      case 2: //停止
        gain = 10;
        servoPwmOut( 0 );
        motor_f( 0, 0 );
        motor_r( 0, 0 );
        led_out( 0xaa);
        break;
      case 3: //ステアリング
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
      case 4: //モータ速度
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
      case 5: //ハンドル角度
        handle( 0 ); //120 以下にする
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
		
      case 11:				//通常トレース
		gain = 10;
		servoPwmOut(iServoPwm);
		crank_mode = 0;
		speed_target = data_buff[DF_TESTSPEED];	//スピード目標　パラメータTESTSPEED(通常走行スピード)	
		/*		通常走行の制御		  				*/
		/*      条件：直線   実際の速度 >=	目標速度 */
		if(iEncoder >= speed_target){		
			motor_mode_f(BRAKE,BRAKE);	
			motor_mode_r(BRAKE,BRAKE);
			motor_f(1,1);	//前 （左,右）
			motor_r(1,1);	//後（左,右）
		}
		/*		通常走行の制御		  				*/
		/*      条件：直線   実際の速度 <	目標速度*/
		else{
			if(abs(angle)>6){		//コーナー走行
				corner_run(90);	//コーナー走行(後外輪MAX)パラ指定[%]
			}
			else{
				motor_mode_f(FREE,FREE);	
				motor_mode_r(FREE,FREE);
				motor_f(100,100); 		     //前 （左,右）
				motor_r(100,100); 		 //後（左,右）
			}
		}
		if(abs(angle)>10){	//コーナー走行
			pattern=12;	//コーナーの速度制御処理へ
		}
		/* ---------------レーンチェンジ・クロスライン判定--------------- */
		if ( check_crossline()) { /* クロスラインチェック */
			servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
      	    causeout = lEncoderTotal;
			led_out(0x0f);
			setBeepPatternS( 0xcc00 );
			pattern = 21;
			break;
        }
  		if ( check_zlineR()) { // 右車線変更ラインチェック
			servoPwmOut( 0 );
         	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
          	Lane_Change = RIGHT; // 1:左 0:右
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x33 );
          	pattern = 51;
          	break;
		}
       if ( check_zlineL()) { // 左車線変更ラインチェック
       		servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
         	Lane_Change = LEFT; // 1:左 0:右
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x44 );
          	pattern = 51;
          	break;
		}
       if ( !check_Noline() ) {
		   causeout = lEncoderTotal;
		}		
		/* ---------------レーンチェンジ・クロスライン判定ここまで--------------- */
	  	break;			
	
    case 12:					//コーナーの速度制御処理 12 < angle <= 30
		gain = 10;
		servoPwmOut(iServoPwm);
		corner_speed = data_buff[DF_CURVE_SPEED];//スピード目標　パラメータn_s(通常走行スピード)	
		if(abs(angle)<=12){
			pattern=11;
		}
		corner_run(90);			//コーナー走行(後外輪MAX)パラ指定[%]
		if(corner_speed <= iEncoder){	
		//	kyori=100;	
			pattern=13;
			lEncoderCorner = lEncoderTotal;
		}
		/* ---------------レーンチェンジ・クロスライン判定--------------- */
		if ( check_crossline()) { /* クロスラインチェック */
			servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
      	    causeout = lEncoderTotal;
			led_out(0x0f);
			setBeepPatternS( 0xcc00 );
			pattern = 21;
			break;
        }
  		if ( check_zlineR()) { // 右車線変更ラインチェック
			servoPwmOut( 0 );
         	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
          	Lane_Change = RIGHT; // 1:左 0:右
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x33 );
          	pattern = 51;
          	break;
		}
       if ( check_zlineL()) { // 左車線変更ラインチェック
       		servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
         	Lane_Change = LEFT; // 1:左 0:右
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x44 );
          	pattern = 51;
          	break;
		}
       if ( !check_Noline() ) {
		   causeout = lEncoderTotal;
		}
		break;
		/* ---------------レーンチェンジ・クロスライン判定ここまで--------------- */

    case 13:	//コーナーのブレーキ処理　１段階目 100mm
		/*パターン13になったら100 [mm]ブレーキ走行*/ 
		gain = 10;
		servoPwmOut(iServoPwm);
		motor_mode_f(BRAKE,BRAKE);	
		motor_mode_r(BRAKE,BRAKE);	
		motor_f(-30,-30);	//前（左,右）
		motor_r(-30,-30); 	//後（左,右）
		if(iEncoder < data_buff[DF_CURVE_SPEED] ){	
			pattern = 12;
			lEncoderCorner = lEncoderTotal;
		}
		if( lEncoderTotal - lEncoderCorner >= 1091L){ // 100mm走行
			lEncoderCorner = lEncoderTotal;
			pattern = 14;
			break;
		} 
		if(abs(angle)<=12){
			pattern=11;
		}
		/* ---------------レーンチェンジ・クロスライン判定--------------- */
		if ( check_crossline()) { /* クロスラインチェック */
			servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
      	    causeout = lEncoderTotal;
			led_out(0x0f);
			setBeepPatternS( 0xcc00 );
			pattern = 21;
			break;
        }
  		if ( check_zlineR()) { // 右車線変更ラインチェック
			servoPwmOut( 0 );
         	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
          	Lane_Change = RIGHT; // 1:左 0:右
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x33 );
          	pattern = 51;
          	break;
		}
       if ( check_zlineL()) { // 左車線変更ラインチェック
       		servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
         	Lane_Change = LEFT; // 1:左 0:右
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x44 );
          	pattern = 51;
          	break;
		}
       if ( !check_Noline() ) {
		   causeout = lEncoderTotal;
		}
		/* ---------------レーンチェンジ・クロスライン判定ここまで--------------- */
	  	break;

    case 14:	//コーナーのブレーキ処理　2段階目 100mm
		servoPwmOut(iServoPwm);
		gain = 10;
		motor_mode_f(BRAKE,BRAKE);	
		motor_mode_r(BRAKE,BRAKE);	
		motor_f(0,0);	//前（左,右）
		motor_r(-30,-30); 	//後（左,右）
		if(iEncoder < data_buff[DF_CURVE_SPEED] ){	
			pattern = 12;
			lEncoderCorner = lEncoderTotal;
		}
		if( lEncoderTotal - lEncoderCorner >= 1091L){ // 100mm走行
			pattern = 12;
			lEncoderCorner = lEncoderTotal;
		}
		if(abs(angle)<=12){
			pattern=11;
		}
		/* ---------------レーンチェンジ・クロスライン判定--------------- */
		if ( check_crossline()) { /* クロスラインチェック */
			servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
      	    causeout = lEncoderTotal;
			led_out(0x0f);
			setBeepPatternS( 0xcc00 );
			pattern = 21;
			break;
        }
  		if ( check_zlineR()) { // 右車線変更ラインチェック
			servoPwmOut( 0 );
         	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
          	Lane_Change = RIGHT; // 1:左 0:右
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x33 );
          	pattern = 51;
          	break;
		}
       if ( check_zlineL()) { // 左車線変更ラインチェック
       		servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
         	Lane_Change = LEFT; // 1:左 0:右
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x44 );
          	pattern = 51;
          	break;
		}
       if ( !check_Noline() ) {
		   causeout = lEncoderTotal;
		}
		/* ---------------レーンチェンジ・クロスライン判定ここまで--------------- */
	  	break;

     case 21:
        /* クロスライン通過処理 */
		gain = 10;
		crank_mode = 1;
        servoPwmOut( iServoPwm );
        led_out( 0xff );
        if ( iEncoder >= data_buff[DF_CRANK_SPEED] ) {         /* エンコーダによりスピード制御 */
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
        /* クロスライン後のトレース、直角検出処理 */
        gain = 10;
        servoPwmOut( iServoPwm );
        motor_mode_f( BRAKE, BRAKE );
        motor_mode_r( BRAKE, BRAKE );
        if ( iEncoder >= data_buff[DF_CRANK_SPEED] ) {         /* エンコーダによりスピード制御 */
          motor_f( -100, -100 );
          motor_r( -100, -100 );
        } else if(iEncoder < data_buff[DF_CRANK_SPEED]) {
          motor_f( 50, 50 );
          motor_r( 50, 50 );
        } else {
			motor_f( 0, 0 );
			motor_r( 0, 0 );
		 }

        if ( center_inp() == 1 && ((sensor_inp() & 0x01) == 0x01) ) { /* 右クランク？             */
          led_out( 0x1 );
          cnt1 = 0;
		   lEncoderCrank = lEncoderTotal;
          pattern = 31;
          break;
        }
        if ( center_inp() == 1 && ((sensor_inp() & 0x08) == 0x08) ) { /* 左クランク？            */
          led_out( 0x2 );
          cnt1 = 0;
		   lEncoderCrank = lEncoderTotal;
          pattern = 41;
          break;
        }
        break;

      case 31:
        /* 右クランク処理 */
        servoPwmOut( 100 );         /* 振りが弱いときは大きくする       */
        motor_mode_f(BRAKE, FREE);
        motor_mode_r(BRAKE, FREE);
        // diff使用禁止!!↓↓↓
        motor_f( 100, -60 );          /* この部分は「角度計算(4WD時).xls」*/
        motor_r( 80, -40 );          /* で計算                           */
        if ( sensor_inp() == 0x04) {   /* 曲げ終わりチェック           */
          cnt1 = 0;
          iSensorPattern = 0;
          lEncoderCrank = lEncoderTotal;
          crank_mode = 0;
          pattern = 32;
        }
        break;

      case 32:
        /* 少し時間が経つまで待つ */
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
		// ↓10cm進む間にレーンチェンジ・クロスラインが来ても認識できるようににする
		
		/* ---------------レーンチェンジ・クロスライン判定--------------- */
		if ( check_crossline()) { /* クロスラインチェック */
			servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
      	    causeout = lEncoderTotal;
			led_out(0x0f);
			setBeepPatternS( 0xcc00 );
			pattern = 21;
			break;
        }
  		if ( check_zlineR()) { // 右車線変更ラインチェック
			servoPwmOut( 0 );
         	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
          	Lane_Change = RIGHT; // 1:左 0:右
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x33 );
          	pattern = 51;
          	break;
		}
       if ( check_zlineL()) { // 左車線変更ラインチェック
       		servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
         	Lane_Change = LEFT; // 1:左 0:右
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x44 );
          	pattern = 51;
          	break;
		}
       if ( !check_Noline() ) {
		   causeout = lEncoderTotal;
		}
		/* ---------------レーンチェンジ・クロスライン判定ここまで--------------- */
	  	
		
        break;

      case 41:
	  	servoPwmOut( -100 );         /* 振りが弱いときは大きくする       */
        motor_mode_f(FREE, BRAKE);
        motor_mode_r(FREE, BRAKE);
        // diff使用禁止!!↓↓↓
        motor_f( -60, 100 );          /* この部分は「角度計算(4WD時).xls」*/
        motor_r( -40, 80 );           /* で計算                           */
        if ( sensor_inp() == 0x02) {   /* 曲げ終わりチェック           */
          cnt1 = 0;
          iSensorPattern = 0;
          crank_mode = 0;
          lEncoderCrank = lEncoderTotal;
          pattern = 42;
        }
        break;
	  
	  case 42:
	  	/* 少し時間が経つまで待つ */
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
		// ↓10cm進む間にレーンチェンジ・クロスラインが来ても認識できるようににする
		
		/* ---------------レーンチェンジ・クロスライン判定--------------- */
		if ( check_crossline()) { /* クロスラインチェック */
			servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
      	    causeout = lEncoderTotal;
			led_out(0x0f);
			setBeepPatternS( 0xcc00 );
			pattern = 21;
			break;
        }
  		if ( check_zlineR()) { // 右車線変更ラインチェック
			servoPwmOut( 0 );
         	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
          	Lane_Change = RIGHT; // 1:左 0:右
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x33 );
          	pattern = 51;
          	break;
		}
       if ( check_zlineL()) { // 左車線変更ラインチェック
       		servoPwmOut( 0 );
          	lEncoderCrank = lEncoderTotal;
          	causeout = lEncoderTotal;
         	Lane_Change = LEFT; // 1:左 0:右
          	setBeepPatternS( 0xc8000 );
          	led_out( 0x44 );
          	pattern = 51;
          	break;
		}
       if ( !check_Noline() ) {
		   causeout = lEncoderTotal;
		}
		/* ---------------レーンチェンジ・クロスライン判定ここまで--------------- */
	  	break;
		
      case 51: // クランクチェック
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
        else if ((lEncoderTotal - lEncoderCrank) > 109L) { // 10mm で 3
          lEncoderCrank = lEncoderTotal;
          causeout = lEncoderTotal;
          pattern = 52;
          break;
        }
        if ( !check_Noline() ) {
          causeout = lEncoderTotal;
        }
        break;
		
      case 52: // ライン終了サーチ
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
        if ( check_Noline() && Lane_Change == RIGHT ) { // 右車線変更?
          crank_mode = 1;
          lEncoderCrank = lEncoderTotal;
          causeout = lEncoderTotal;
          led_out( 0x0f );
          pattern = 61;
          break;
        } else if ( check_Noline() && Lane_Change == LEFT ) { // 左車線変更？
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
		
      case 61: // (右)車線変更処理 1
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
        if ( (sensor_inp() == 0x01) && (sensor_inp() != 0x08) ) { // デジタル 1,2,4 アナログ L,R タイヤ側から見て
          iSensorPattern = 0;
          crank_mode = 0;
          lEncoderCrank = lEncoderTotal;
          causeout = lEncoderTotal;
          pattern = 62;
          break;
        }
        break;
		
      case 62: // (右)車線変更処理 2
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
		
      case 63: // (右)車線変更処理 3
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
		
      case 64: // (右)車線変更処理 4
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
        if ( (lEncoderTotal - lEncoderCrank) >= (109L) ) { // 10mm で 3 150mm 後に通常トレースへ遷移
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
		
      case 71: // (左)車線変更処理 1
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
		
      case 72: // (左)車線変更処理 2
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
		
      case 73: // (左)車線変更処理 3
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
		
      case 74: // (左)車線変更処理 4
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
        if ((lEncoderTotal - lEncoderCrank) >= (109L) ) { // 10mm で 3 150mm 後に通常トレースへ遷移
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
        /* 停止処理 */
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
        /* 何もしない */
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
  pd5 = 0x7f;

  /*  none            none            none            none
      none            none            Arduino(ZERO)   Arduino(MODE)   */
  p6  = 0x00;
  pd6 = 0xef;

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
/* タイマ RB 割り込み処理 */
/************************************************************************/
#pragma interrupt /B intTRB(vect=24)
void intTRB( void ) {
  unsigned int i;
  volatile static unsigned int line_no;
  asm(" fset I "); /* タイマ RB 以上の割り込み許可 */
  cnt0++;
  cnt1++;
  cnt_saka++;
  /* 角度値angle, kakudoを1msおきに計算 */
  angle = getServoAngle();
  kakudo = angle / SERVO_STEP; /* 1 度あたりの増分で割る */
  if ( kakudo < 0 ) kakudo = -kakudo;
  if ( kakudo > 60 ) kakudo = 60; //45
  
  
  /* サーボモータ制御 */
  servoControl();
  if( pattern >= 1 && pattern <= 100 ){
  	handle(0);
  }
  /* ブザー処理 */
  beepProcessS();
  /* microSD間欠書き込み処理(1msごとに実行) */
  microSDProcess();
  
  	/* lcd処理 */
  	lcdShowProcess();
 	 /* 拡張スイッチ用関数(1msごとに実行) */
  	switchProcess();

  
  /* 10 回中 1 回実行する処理 */
  /*都合上 4 回にしている*/
  iTimer10++;
  switch ( iTimer10 ) {
    case 1:
      /* エンコーダ制御 */
      i = trg;
      iEncoder = i - uEncoderBuff;
      lEncoderTotal += iEncoder;
      p4_5 = p3_0; // R8C/38AボードのLEDにエンコーダの状態を出力
      uEncoderBuff = i;
      break;
    case 2:
      /* スイッチ読み込み準備 */
      p9_4 = 0; /* LED 出力 OFF */
      pd8 = 0x00;
      break;
    case 3:
      /* スイッチ読み込み、LED 出力 */
      types_dipsw = ~p8; /* ﾄﾞﾗｲﾌﾞ基板 TypeS Ver.3 の SW 読み込み*/
      p8 = types_led; /* ﾄﾞﾗｲﾌﾞ基板 TypeS Ver.3 の LED へ出力*/
      pd8 = 0xff;
      p9_4 = 1; /* LED 出力 ON */
      break;
	case 4:
	/* microSD記録処理 */
	  if( msdFlag == 1 ){
			msdPrintf("%4d,%3d,=\"%4b\",%c,%5d,%4d,%2d,%4d,%4d,%4d,%4d,%1d,%d\r\n",
				line_no,			// 行番号
				pattern,			// 動作パターン
				sensor_inp(),  		// デジタル(4bit)
				center_inp()+'0', 	// デジタル(中心)
				getAnalogSensor(),  // アナログセンサ
				getServoAngle(),	// ボリューム(ステアリング角度)
				iEncoder,          	// エンコーダ
				check_zlineL(),  	// デジタル左
				check_zlineR(), 	// デジタル右
				check_crossline(), 	// クロスラインチェック
				ad5,				// 坂道検出用ポテンショメータ 
				saka_flag,			// 坂検出フラグ
				lEncoderTotal		// エンコーダ積算値
			);
			if(++line_no >= 10000 ) line_no = 0; 
	  }
	  break;
	case 10: //前回は 4 だった
      /* iTimer10 変数の処理 */
      iTimer10 = 0;
      break;
  }
  
  if(pattern == 1 ){
	servoPwmOut(iServoPwm);	  
  }
}
/************************************************************************/
/* タイマ RC 割り込み処理 */
/************************************************************************/
#pragma interrupt intTRC(vect=7)
void intTRC( void )
{
  trcsr &= 0xfe;
  /* タイマ RC デューティ比の設定 */
  trcgrb = trcgrb_buff;
  trcgrd = trcgrd_buff;
}
void timer(unsigned long timer_set) {
  cnt0 = 0;
  while (cnt0 <= timer_set);
}
/************************************************************************/
/* アナログセンサ基板 TypeS Ver.2 のデジタルセンサ値読み込み */
/* 引数 なし */
/* 戻り値 左端、左中、右中、右端のデジタルセンサ 0:黒 1:白 */
/************************************************************************/
unsigned char sensor_inp( void ) //0,0,0,0 機体から見て
{
  unsigned char sensor;
  sensor = ~p0 & 0x0f;
  return sensor;
}
/************************************************************************/
/* アナログセンサ基板 TypeS Ver.2 の中心デジタルセンサ読み込み */
/* 引数 なし */
/* 戻り値 中心デジタルセンサ 0:黒 1:白 */
/************************************************************************/
unsigned char center_inp( void )
{
  unsigned char sensor;
  sensor = ~p1_7 & 0x01;
  return sensor;
}
/************************************************************************/
/* アナログセンサ基板 TypeS Ver.2 のスタートバー検出センサ読み込み */
/* 引数 なし */
/* 戻り値 0:スタートバーなし 1:スタートバーあり */
/************************************************************************/
unsigned char startbar_get( void )
{
  unsigned char sensor;
  sensor = ~p1_6 & 0x01;
  return sensor;
}
/************************************************************************/
/* マイコンボード上のディップスイッチ値読み込み */
/* 引数 なし */
/* 戻り値 スイッチ値 0～15 分けている */
/************************************************************************/
unsigned char dipsw_get( void )
{
  unsigned char sw;
  sw = p1 & 0x0e; /* P1_3～P1_1 読み込み */
  return sw;
}
unsigned char dipsw_getf( void )
{
  unsigned char sw;
  sw = p1 & 0x01; /*P1_0 読み込み */
  return sw;
}
/************************************************************************/
/* モータドライブ基板 TypeS Ver.3 上のディップスイッチ値読み込み */
/* 引数 なし */
/* 戻り値 スイッチ値 0～255 意味なし */
/************************************************************************/
unsigned char dipsw_get2( void )
{
  /* 実際の入力はタイマ RB 割り込み処理で実施 */
  unsigned char sw;
  sw = p8 & 0xf0; /*P8_7～P8_4 読み込み */
  return sw;
  // return types_dipsw;
}
unsigned char dipsw_get3( void )
{
  /* 実際の入力はタイマ RB 割り込み処理で実施 */
  unsigned char sw;
  sw = p8 & 0x03; /* P8_1～P8_0 読み込み */
  return sw;
  // return types_dipsw;
}
unsigned char dipsw_get4( void )
{
  /* 実際の入力はタイマ RB 割り込み処理で実施 */
  unsigned char sw;
  sw = p8 & 0x0c; /* P8_3～P8_2 読み込み */
  return sw;
  // return types_dipsw;
}
/************************************************************************/
/* モータドライブ基板 TypeS Ver.3 上のプッシュスイッチ値読み込み */
/* 引数 なし */
/* 戻り値 スイッチ値 0:OFF 1:ON */
/************************************************************************/
unsigned char pushsw_get( void )
{
  unsigned char sw;
  sw = ~p9_5 & 0x01;
  return sw;
}
/************************************************************************/
/* モータドライブ基板 TypeS Ver.3 の CN6 の状態読み込み */
/* 引数 なし */
/* 戻り値 0～15 */
/************************************************************************/
unsigned char cn6_get( void )
{
  unsigned char data;
  data = p7 >> 4;
  return data;
}
/************************************************************************/
/* モータドライブ基板 TypeS Ver.3 の LED 制御 */
/* 引数 8 個の LED 制御 0:OFF 1:ON */
/* 戻り値 なし */
/************************************************************************/
void led_out( unsigned char led )
{
  /* 実際の出力はタイマ RB 割り込み処理で実施 */
  types_led = led;
}
/************************************************************************/
/* 後輪の速度制御 */
/* 引数 左モータ:-100～100 , 右モータ:-100～100 */
/* 0 で停止、100 で正転 100%、-100 で逆転 100% */
/* 戻り値 なし */
/************************************************************************/
void motor2_r( int accele_l, int accele_r ) {
  int sw_data;
  if ( M_FreeMoter == 1 ) {
    accele_l = 1;
    accele_r = 1;
  }
  sw_data = dipsw2_pattern[dipsw_get2()] + 5; /* ディップスイッチ読み込み */
  accele_l = -accele_l * sw_data / 20;
  accele_r = -accele_r * sw_data / 20;

  accele_r = -accele_r;
  // 左後モータ
  if ( accele_l >= 0 ) {
    p2_1 = 0;
    trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
  } else {
    p2_1 = 1;
    trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
  }
  // 右後モータ
  if ( accele_r >= 0 ) {
    p2_3 = 0;
    trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
  } else {
    p2_3 = 1;
    trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
  }
}
/************************************************************************/
/* 後輪の速度制御 2 ディップスイッチには関係しない motor 関数 */
/* 引数 左モータ:-100～100 , 右モータ:-100～100 */
/* 0 で停止、100 で正転 100%、-100 で逆転 100% */
/* 戻り値 なし */
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
  // 左後モータ
  if ( accele_l >= 0 ) {
    p2_1 = 1;
    trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
  } else {
    p2_1 = 0;
    trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
  }
  // 右後モータ
  if ( accele_r >= 0 ) {
    p2_3 = 1;
    trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
  } else {
    p2_3 = 0;
    trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
  }
}
/************************************************************************/
/* 前輪の速度制御 */
/* 引数 左モータ:-100～100 , 右モータ:-100～100 */
/* 0 で停止、100 で正転 100%、-100 で逆転 100% */
/* 戻り値 なし */
/************************************************************************/
void motor2_f( int accele_l, int accele_r ) {
  int sw_data;
  if ( M_FreeMoter == 1 ) {
    accele_l = 1;
    accele_r = 1;
  }
  sw_data = dipsw2_pattern[dipsw_get2()] + 5; /* ディップスイッチ読み込み */
  accele_l = accele_l * sw_data / 20;
  accele_r = accele_r * sw_data / 20;
  accele_l = -accele_l;
  // 左前モータ
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
  // 右前モータ
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
/* 前輪の速度制御 2 ディップスイッチには関係しない motor 関数 */
/* 引数 左モータ:-100～100 , 右モータ:-100～100 */
/* 0 で停止、100 で正転 100%、-100 で逆転 100% */
/* 戻り値 なし */
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
  /* 左前モータ */
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
  /* 右前モータ */
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
/* 後モータ停止動作（ブレーキ、フリー） */
/* 引数 左モータ:FREE or BRAKE , 右モータ:FREE or BRAKE */
/* 戻り値 なし */
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
/* 前モータ停止動作（ブレーキ、フリー） */
/* 引数 左モータ:FREE or BRAKE , 右モータ:FREE or BRAKE */
/* 戻り値 なし */
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
/* サーボモータ制御 */
/* 引数 サーボモータ PWM：-100～100 */
/* 0 で停止、100 で正転 100%、-100 で逆転 100% */
/* 戻り値 なし */
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
/* クロスライン検出処理 */
/* 引数 なし */
/* 戻り値 0:クロスラインなし 1:あり */
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
/* 右車線変更ライン検出処理 */
/* 引数 なし */
/* 戻り値 0:車線変更ラインなし 1:あり */
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
/* 左車線変更ライン検出処理 */
/* 引数 なし */
/* 戻り値 0:車線変更ラインなし 1:あり */
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
/* ライン終了検出処理 */
/* 引数 なし */
/* 戻り値 0:車線変更ラインなし 1:あり */
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
/* サーボ角度取得 */
/* 引数 なし */
/* 戻り値 入れ替え後の値 */
/************************************************************************/
int getServoAngle( void )
{
  return ( (-ad2) - iAngle0 );
}
/************************************************************************/
/* アナログセンサ値取得 */
/* 引数 なし */
/* 戻り値 センサ値 */
/************************************************************************/
int getAnalogSensor( void )
{
  int ret;
  //ret = ad1 - ad0; /* アナログセンサ情報取得 */
  if ( !crank_mode ) {
    // クランクモードでなければ補正処理
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
			// 左-右
			ret = (ad1 - 21) - (ad0 + 21);
			//ret = (ad1 - 11) - (ad0 + 11);
        }
        break;
      case 1:
        // センサ右寄り
        ret = -700;
        if (center_inp() == 0x01 || sensor_inp() == 0x04) {
          iSensorPattern = 0;
        }
        break;
      case 2:
        // センサ左寄り
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
/* サーボモータ制御 */
/* 引数 なし */
/* 戻り値 グローバル変数 iServoPwm に代入 */
/************************************************************************/
void servoControl( void )
{
  int i, iRet, iP, iD;
  int kp, kd;
  i = getAnalogSensor(); /* センサ値取得 */
  kp = dipsw1_pattern[dipsw_get()] ; /* 調整できたら P,D 値は固定値に */
  kd = dipsw1_pattern[dipsw_get()] * 10; /* してください */
  /* サーボモータ用 PWM 値計算 */
  iP = gain * i; /* 比例 */
  //iP = kp * i; /* 比例 */
  //10 を gain にする
  //iD = 100 * (iSensorBefore - i ); /* 微分(目安は P の 5～10 倍) */
  //iD = kd * (iSensorBefore - i ); /* 微分(目安は P の 5～10 倍) */
  iD = (gain * 7) * (iSensorBefore - i); /* 微分(目安は P の 5～10 倍) */
  iRet = iP - iD;
  iRet = iRet / (130 / 2); //iRet /= 64; //64
  /* PWM の上限の設定 */
  if ( iRet > 120 ) iRet = 120; /* マイコンカーが安定したら */
  if ( iRet < -120 ) iRet = -120; /* 上限を 90 くらいにしてください */

  iServoPwm = iRet;
  iSensorBefore = i; /* 次回はこの値が 1ms 前の値となる*/
}
/************************************************************************/
/* モジュール名 handle */
/* 処理概要 サーボモータ制御 角度指定用 */
/* 引数 なし */
/* 戻り値 グローバル変数 servoPwmOut に代入 */
/************************************************************************/
void handle( int iSetAngle ) {
  int i, j, iRet, iP, iD ;
  i = - iSetAngle; /* 設定したい角度 */
  //j = ( p7_2 >>2 );
  j = getServoAngle(); /* 現在の角度 */
  /* サーボモータ用 PWM 値計算 */
  iP = 20 * (j - i); /* 比例 */
  iD = 100 * (iAngleBefore2 - j); /* 微分 */
  iRet = iP - iD;
  iRet = iRet / 2; //iRet /= 4; //2
  if ( iRet > 120 ) iRet = 120; /* マイコンカーが安定したら */
  if ( iRet < -120 ) iRet = -120; /* 上限を 90 くらいにしてください */
  servoPwmOut(iRet);
  
  iAngleBefore2 = j;
}
/************************************************************************/
/* 外輪の PWM から、内輪の PWM を割り出す ハンドル角度は現在の値を使用 	*/
/* 引数 外輪 PWM 														*/
/* 戻り値 内輪 PWM 														*/
/************************************************************************/
int diff( int pwm )
{
  int i, ret;
  i = getServoAngle() / SERVO_STEP; /* 1 度あたりの増分で割る */
  if ( i < 0 ) i = -i;
  if ( i > 40 ) i = 40; //45
  ret = revolution_difference[i] * pwm / 100;
  return ret;
}

/************************************************************************/
/* 坂道チェック                   										*/
/* 引数 なし                              								*/
/* 戻り値 なし                               							*/
/* メモ 坂道と判断するとsaka_flag = 1 坂道でないなら 0           		*/
/************************************************************************/
void sakaSyori( void )
{
  volatile static int saka_pattern = 0;
  volatile int saka;

  saka = ad5 - saka0_ad;
  // 坂の実習中はled_outでパターンを表示、実習が終わったらコメントにする
#if 0
	led_out( ((saka_pattern/10)<<4) | (saka_pattern%10) );
#endif

  switch ( saka_pattern ) {
    case 0:
      // 上り坂、下り坂のチェック
      if ( saka <= -10 ) { // 上りはA/D値が小さくなる
        cnt_saka = 0;
        saka_pattern = 1; // 上り坂処理
      }
      break;

    case 1:
      // 上り坂 少し時間をおいて、再度チェック
      if ( cnt_saka >= 10 ) {
        if ( saka <= -10 ) { // 反応あり
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
      if ( cnt_saka >= 100 ) {
        cnt_saka = 0;
        saka_pattern = 3;
      }
      break;

    case 3:
      // 上り坂 上りの頂点をチェック
      if ( saka >= 10 ) {
        cnt_saka = 0;
        saka_pattern = 4;
      }
      break;

    case 4:
      // 上り坂 少し時間をおいて、再度チェック
      if ( cnt_saka >= 10 ) {
        if ( saka >= 10 ) { // 反応あり
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
      if ( cnt_saka >= 200 ) {
        cnt_saka = 0;
        saka_pattern = 0;
      }
      break;
  }
}


/* LCDとスイッチで速度パラメータを決める関数 	*/
/* 0:設定前/設定中 1:設定終了					*/
int paramSettings( void ){
	
	volatile static int param_pattern = 0; // パラメータ設定のパターン
	volatile static int isGo = 0; // スタンバイ状態へ移行するか(1:スタンバイ状態へ 0:パラメータ設定状態)
	unsigned int i;
	signed int saka_val;
	
	if( param_pattern < 5 ){	
		i = ((cnt1 / 200) % 2 + 1) | (startbar_get()<<7&0x80); // スタートバー反応確認
		led_out( i ); /* LED 点滅処理 */
	}
	
	if( pushsw_get() ){
		/* STRATスイッチ押下で前回パラメータのまま続行 */
		param_pattern = 6;	
		setBeepPatternS( 0xa800 );
	}
	
	switch( param_pattern ){
		case 0:	
		/* 直線の走行速度:(11で1.0m/s) */
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
		/* カーブの走行速度(11で1.0m/s) */
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
		/* 左レーンチェンジの走行速度:(11で1.0m/s) */
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
		/* 右レーンチェンジの走行速度:(11で1.0m/s) */			
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
		/* クランクの走行速度:(11で1.0m/s) */
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
		/* 走行距離:(単位はmで指定) */
			//ENC_END = getSws();
			if(getSwNow()&0x03){    
				data_buff[DF_ENC_END] = getSws();
			}
			lcdPosition(0,0);
			lcdPrintf("RUN/DISTANCE    ");
			lcdPosition(0,1);
			lcdPrintf("DISTANCE= %03dm ",data_buff[DF_ENC_END]);
			
			if( getSwFlag(SW_4)){
				//data_buff[DF_ENC_END] *= 1091L; // これをやるとsigned char型を超えるのでNG	
  				setBeepPatternS( 0xcc00 );
				param_pattern++;
				//param_pattern = 5;	
				led_out(0x00);
			}
			break;
			
		case 6:
		/* スタンバイ状態へ */
			lcdPosition(0,0);
			lcdPrintf("STAND-BY... ");
			lcdPosition(0,1);
			// param_patternの1～4で設定した値を表示
			lcdPrintf("%02d,%02d,%02d,%02d,%02d    ",
				data_buff[DF_TESTSPEED], data_buff[DF_CURVE_SPEED], data_buff[DF_LANE_L], data_buff[DF_LANE_R], data_buff[DF_CRANK_SPEED]);
			
			isGo = 1;
			break;
		
	}
	return isGo;
	
}

/* スイッチの入力をまとめた関数 */
/* SW0赤は+1 */
/* SW1青は-1 */
/* SW2黄は+10 */
/* SW3緑は-10 */
/* (SW4黒は主に決定するときに使う) */
int getSws( void ){
	beforeswnum = swnum;
	/* 変数swnumは必ずグローバル変数 */
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
/* コーナーの速度制御 　												*/
/* 引数 max_power 後輪の外輪のＰＷＭの最大値 0~100%					*/
/* 戻り値 なし 　　　　 												*/
/************************************************************************/
void corner_run(int max_power){
	/*	コーナー時の制御 */
  
	accel_out_b = max_power;//外側後輪	
	accel_in = (long)f_in[kakudo]*max_power/100;		//内側前輪
	if (abs(angle) > 60){									
		accel_in_b = 0;	//内側後輪(0%)	
	}
	else{
		accel_in_b = (long)r_in[kakudo]*max_power/100;	//内側後輪
	}
	accel_out = (long)f_out[kakudo]*max_power/100;		//外側前輪
	if(angle < 0){	/* ステアリングを左に切るとき */
		motor_mode_f(BRAKE,FREE);	
		motor_mode_r(BRAKE,FREE);						
		motor_f(accel_in,accel_out); 			//前 （左,右）
		motor_r(accel_in_b,accel_out_b);		//後（左,右）
	}
	else if(angle > 0){/* ステアリングを右に切るとき */
		motor_mode_f(FREE,BRAKE);	
		motor_mode_r(FREE,BRAKE);
		motor_f(accel_out,accel_in);	 		//前 （左,右）
		motor_r(accel_out_b,accel_in_b);		//後（左,右）
	}
}

/************************************************************************/
/* end of file */
/************************************************************************/