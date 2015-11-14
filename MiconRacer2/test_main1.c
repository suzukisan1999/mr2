//------------------------------------------------------------------------------
// 対象マイコン R8C/34C
// ﾌｧｲﾙ内容     Micon Racer Advance
// バージョン   Ver.1.00
// Date         2014.12.19
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// インクルード
//------------------------------------------------------------------------------
#include <stdlib.h>
#include "sfr_r834c.h"

#define	motor	motor_fix

//------------------------------------------------------------------------------
// シンボル定義
//------------------------------------------------------------------------------
#define TIMER_CYCLE     155             // 1ms:0.001/(1/(20000000/128))-1

#define PWM_CYCLE       999             // 0.4ms:0.001/(1/(20000000/8))-1
//#define PWM_CYCLE       19999         // 8ms  :0.008/(1/(20000000/8))-1
//#define PWM_CYCLE       3999          // 1.6ms:0.0016/(1/(20000000/8))-1
//#define PWM_CYCLE       1999          // 0.8ms:0.0008/(1/(20000000/8))-1
//#define PWM_CYCLE       999           // 0.4ms:0.0004/(1/(20000000/8))-1
//#define PWM_CYCLE       499           // 0.2ms:0.0002/(1/(20000000/8))-1

#define Def_500Hz       4999            // 500Hz:(1/500)/(1/(20000000/8))-1
#define Def_1000Hz      2499            // 1000Hz:(1/1000)/(1/(20000000/8))-1

#define Def_C3          19083           // ド:(1/131)/(1/(20000000/8))-1
#define Def_D3          17006           // レ:(1/147)/(1/(20000000/8))-1
#define Def_E3          15151           // ミ:(1/165)/(1/(20000000/8))-1
#define Def_F3          14285           // ファ:(1/175)/(1/(20000000/8))-1
#define Def_G3          12754           // ソ:(1/196)/(1/(20000000/8))-1
#define Def_A3          11362           // ラ:(1/220)/(1/(20000000/8))-1
#define Def_B3          10120           // シ:(1/247)/(1/(20000000/8))-1
#define Def_C4          9541            // ド:(1/262)/(1/(20000000/8))-1

#define DI()            asm("FCLR I")   // 割り込み禁止
#define EI()            asm("FSET I")   // 割り込み許可

#define WHITE           (1)
#define BLACK           (0)

//------------------------------------------------------------------------------
// オプション設定
//------------------------------------------------------------------------------

#define CENTER_LINE     BLACK						// 中央ラインの色の設定
//#define CENTER_LINE     WHITE						// 中央ラインの色の設定

#define MOTOR_LIMIT		(50)						// モータ最大出力 : 60%
#define OPTION_L        (MOTOR_LIMIT)				// 最大回転数設定
#define OPTION_R        (MOTOR_LIMIT)

// LEDセンサー定義  ○：消灯   ●：点灯
#define SENSOR_LR0		(       0x04 | 0x02       )	// 中央   : ○●●○
#define SENSOR_L1		(       0x04              )	// 左(小) : ○●○○
#define SENSOR_R1		(              0x02       )	// 右(小) : ○○●○
#define SENSOR_L2		(0x08 | 0x04              )	// 左(中) : ●●○○
#define SENSOR_R2		(              0x02 | 0x01)	// 右(中) : ○○●●
#define SENSOR_L3		(0x08                     )	// 左(大) : ●○○○
#define SENSOR_R3		(                     0x01)	// 右(大) : ○○○●
#define SENSOR_AB		(0x08 | 0x04 | 0x02 | 0x01)	// 全点灯 : ●●●● (AB:All Black)
#define SENSOR_AW		(0x00)						// 全消灯 : ○○○○ (AW:All White)

//------------------------------------------------------------------------------
// 関数プロトタイプの宣言
//------------------------------------------------------------------------------
void init( void );
unsigned char sensor( void );
unsigned char sensor_check( void );
void motor( int data1, int data2 );
void timer( unsigned long timer_set );
void beep( int data1 );
unsigned char pushsw( void );
void ctrl_servo( int degree );
void set_servo( int degree );
unsigned short get_adc(void);

//------------------------------------------------------------------------------
// グローバル変数の宣言
//------------------------------------------------------------------------------
unsigned long           cnt0 = 0;       // timer関数用
unsigned long           cnt1 = 0;       // main内で使用
unsigned long           cnt2 = 0;       // 自由
unsigned long           cnt3 = 0;       // 自由
int                     pattern = 0;    // パターン番号
volatile static int		deg = 0;		// 目標角度
volatile static unsigned char	line_data = 0;	// 最新ラインパターン

//------------------------------------------------------------------------------
// メインプログラム
//------------------------------------------------------------------------------

void car_ctrl(	int vector,	int speed )
{
	int motor_r;
 	int motor_l;
	int	vol = abs( vector );

	if( vector > 0 ){
		motor_l = speed;
	}else{
		// minus, zero
		motor_l = speed - vol;
	}
	
	if( vector < 0 ){
		motor_r = speed;
	}else{
		// plus, zero
		motor_r = speed - vol;
	}
	//あとちゃんと計算する

#if 0
	if( motor_l == motor_r ){
		beep( Def_C3);
	}else{
		beep( 0 );
	}
#endif

	motor( motor_l, motor_r );
	
	return;
}


void main(void)
{
	unsigned char read_line = 0;
	
    init();		// 周辺機能初期化

#if 1
	while( pushsw() == 0 );	// スタートボタン押下待ち
    while(1){		
		
		read_line = sensor_check();		// ラインセンサーから最新情報を取得
		switch(read_line){				// ラインセンサーからの最新情報に応じた処理の分岐
			case SENSOR_AW:				// ライン：なし (コースアウト時)
				car_ctrl( 0, 0 );
				break;
			case SENSOR_LR0:			// ライン：中央 (車体:中央)
				car_ctrl( 0, 100 );
				break;
			case SENSOR_L1:				// ライン：少し左 (車体:ラインより少し右に位置)
				car_ctrl( -10, 100 );
				break;
			case SENSOR_R1:				// ライン：少し右 (車体:中央より少し左に位置)
				car_ctrl(  10, 100 );
				break;
			case SENSOR_L2:				// ライン：左 (車体:ラインより右に位置)
				car_ctrl( -30, 100 );
				break;
			case SENSOR_R2:				// ライン：右 (車体:ラインより左に位置)
				car_ctrl(  30, 100 );
				break;
			case SENSOR_L3:				// ライン：大きく左 (車体:ラインより大きく右に位置)
				car_ctrl( -60, 60 );
				break;
			case SENSOR_R3:				// ライン：大きく右 (車体:ラインより大きく左に位置)
				car_ctrl(  60, 60 );
				break;
			default:					// LEDセンサー定義に存在しない入力信号
				break;					// なにもしない
			}
	}
#else
	while( pushsw() == 0 );	// スタートボタン押下待ち
    while(1){		
		
		read_line = sensor_check();		// ラインセンサーから最新情報を取得
		switch(read_line){				// ラインセンサーからの最新情報に応じた処理の分岐
			case SENSOR_AW:				// ライン：なし (コースアウト時)
				motor(0,0);				// モータ停止
				break;
			case SENSOR_LR0:			// ライン：中央 (車体:中央)
				set_servo(0);			// 目標角度：±0
				motor(100, 100);		// 後輪駆動モータ：左100% | 右100% (直進)
				break;
			case SENSOR_L1:				// ライン：少し左 (車体:ラインより少し右に位置)
				set_servo(-10);			// 目標角度：-10°(左に少し曲げる)
				motor(100, 100);		// 後輪駆動モータ：左100% | 右100% (直進)
				break;
			case SENSOR_R1:				// ライン：少し右 (車体:中央より少し左に位置)
				set_servo(10);			// 目標角度： 10°(右に少し曲げる)
				motor(100, 100);		// 後輪駆動モータ：左100% | 右100% (直進)
				break;
			case SENSOR_L2:				// ライン：左 (車体:ラインより右に位置)
				set_servo(-25);			// 目標角度：-25°(左に中くらい曲げる)
				motor( 60,  100);		// 後輪駆動モータ：左 60% | 右100% (左に少し曲げる)
				break;
			case SENSOR_R2:				// ライン：右 (車体:ラインより左に位置)
				set_servo(25);			// 目標角度： 25°(右に中くらい曲げる)
				motor( 100,  60);		// 後輪駆動モータ：左100% | 右 60% (右に少し曲げる)
				break;
			case SENSOR_L3:				// ライン：大きく左 (車体:ラインより大きく右に位置)
				set_servo(-40);			// 目標角度：-40°(左に大きく曲げる)
				motor( 33,  100);		// 後輪駆動モータ：左 33% | 右100% (左に大きく曲げる)
				while( sensor_check() != SENSOR_L2);	// センターラインから外れても、復帰するまで現状維持し駆動モータは停止しない
				break;
			case SENSOR_R3:				// ライン：大きく右 (車体:ラインより大きく左に位置)
				set_servo(40);			// 目標角度： 40°(右に大きく曲げる)
				motor( 100,  33);		// 後輪駆動モータ：左右100% | 右 33% (右に大きく曲げる)
				while( sensor_check() != SENSOR_R2);	// センターラインから外れても、復帰するまで現状維持し駆動モータは停止しない
				break;
			default:					// LEDセンサー定義に存在しない入力信号
				break;					// なにもしない
			}
	}
#endif

}

//------------------------------------------------------------------------------
// R8C/34Cの内蔵周辺機能の初期化
//------------------------------------------------------------------------------
void init( void )
{
        volatile unsigned char i = 0;

        // 割り込み禁止
        DI();

        // 高速オンチップオシレータの設定
        prc0 = 1;

        fra1 = fra4;
        fra3 = fra5;
        fra2 = 0x00;
        fra00 = 1;
        while(i <= 50) i++;
        fra01 = 1;
        cm16 = 0;
        cm17 = 0;
        cm06 = 0;
        prc0 = 0;

        // I/Oポートの入出力設定
        prc2 = 1;                       // pd0レジスタへの書き込み許可
        pd0 = 0xe0;                     // P0_0～P0_3:センサ
										// P0_4(AN3) ロータリーエンコーダ
                                        // P0_5～P0_7:LED
        prc2 = 0;                       // pd0レジスタへの書き込み禁止

        pd1 = 0xdf;                     // P1_0～P1_3:LED
                                        // P1_4:TXD0
                                        // P1_5:RXD0
										// P1_7:Servo 1:Active
		p1 = 0x00;
        pu04 = 1;
        pd2 = 0xfe;                     // P2_0:スイッチ
                                        // P2_1:AIN1
                                        // P2_2:PWMA
                                        // P2_3:BIN1
                                        // P2_4:PWMB
                                        // P2_5:
                                        // P2_6:AIN2
                                        // P2_7:

        pd3 = 0xff;                     // P3_4:ブザー

        pd4 = 0xfb;                     // P4_2:VREF
                                        // P4_5:BIN2

        pd6 = 0xff;                     // none

        mstcr = 0x00;                   // モジュールストップ解除

        // タイマRAの1ms割り込み設定
        tramr = 0x00;                   // カウントソースはf1
        trapre = 128 - 1;               // プリスケーラ
        tra = TIMER_CYCLE;              // プライマリカウンタ
        traic = 0x01;                   // タイマRAの割り込みレベル設定
        tracr = 0x01;                   // カウントを開始

        // タイマRBの1ms割り込み設定
        trbmr = 0x00;                   // カウントソースはf1
        trbpre = 128 - 1;               // プリスケーラ
        trbpr = TIMER_CYCLE;            // プライマリカウンタ
        trbic = 0x01;                   // タイマRBの割り込みレベル設定
        trbcr = 0x01;                   // カウントを開始

        // タイマRCのPWMモード
        trccr1 = 0xb0;                  // カウントソースはf8
        trcgra = 0;                     // 圧電サウンダの周期
        trcgrc = 0;                     // 圧電サウンダのデューティ比
        trccr2 = 0x02;                  // TRCIOC端子はアクティブレベルH
        trcoer = 0x0b;                  // TRCIOC端子の出力許可
        trcpsr1 = 0x02;                 // TRCIOC端子をP3_4に割り当て
        trcmr = 0x8a;                   // カウントを開始

        // タイマRDのリセット同期PWMモード
        trdpsr0 = 0x08;                 // TRDIOB0端子をP2_2に割り当て
#if 0
        trdpsr1 = 0x45;                 // TRDIOB1端子をP2_5に割り当て（SERVO_IN2）
                                        // TRDIOA1端子をP2_4に割り当て
										// TRDIOD1端子をP2_7に割り当て（SERVO_IN1）
#else
        trdpsr1 = 0x01;                 // TRDIOA1端子をP2_4に割り当て
#endif
        trdmr = 0xf0;                   // レジスタをバッファ動作にする
        trdfcr = 0x01;                  // リセット同期PWMモードに設定
        trdoer1 = 0x4d;                 // TRDIOB1の出力許可
                                        // TRDIOA1の出力許可
                                        // TRDIOB0の出力許可
										// TRDIOD1の出力許可
        trdcr0 = 0x20;                  // カウントソースはf1
        trdgra0 = trdgrc0 = PWM_CYCLE;  // 周期
        trdgrb0 = trdgrd0 = 0;          // TRDIOB0端子（左モータ）
        trdgra1 = trdgrc1 = 0;          // TRDIOA1端子（右モータ）
        trdgrb1 = trdgrd1 = PWM_CYCLE/2;          // TRDIOB1端子（サーボ）
        trdstr = 0x0d;                  // カウントを開始

		admod   = 0x33;					// 繰り返し掃引モード
		adinsel = 0x30;                 // 8端子を使用
		adcon1  = 0x30;                 // 10ビットモード、AD動作可能
		adic 	= 0x01;					// A/D割り込み 優先レベル：1
		adst    = 1;                    // A/D変換スタート
		
        // 割り込み許可
        EI();
}




//------------------------------------------------------------------------------
// 割り込み
//------------------------------------------------------------------------------

volatile static unsigned short ad_result;

// encoder
// priority : 1 (Low)
#pragma interrupt intAN0 (vect=14)
void intAN0( void )
{
	static int i;
	static unsigned short ad_min = 0x03ff;	// A/D変換ピーク値 (最大値)
	static unsigned short ad_max = 0;		// A/D変換ピーク値 (最小値)
	static unsigned short ad_work = 0;		// A/D変換結果 平均値計算用変数
	unsigned short ad_data;					// A/D変換結果 現在値
	
	if(i == 0){								// １回目のデータ取得時
		ad_work = ad_max = 0;				// 変数(再)初期化
		ad_min = 0x3ff;						// 変数(再)初期化
	}

	ad_data = ad3 & 0x03ff;					// A/D変換結果 取得

	ad_work += ad_data;						// 平均値計算用変数への加算
	if(ad_data > ad_max){ ad_max = ad_data; }	// 最大値更新
	if(ad_data < ad_min){ ad_min = ad_data;	}	// 最小値更新
	i++;
	if(i >= 4){
		ad_result = (ad_work - ad_max - ad_min) >> 1;  // (4回の加算結果 - 最大値 - 最小値)/2
		// ここまでで「ピーク値除去済み平均値(2)」が得られる
	}

	i &= 3;		// カウンタ i ループ用処理  ( 0→1→2→3→0→1→… ) 

}

// line censor
// cyclic handler
// priority : 2 (middle)
#pragma interrupt intTRAIC (vect=22)
void intTRAIC( void )
{
        p0_7 = ~p0_7;

        if( p0_7 == 0 ){
#if CENTER_LINE
            //p0_1、p0_3のモニタが可能
            p0_5 = ~p0_1;
            p0_6 = ~p0_3;
#else
            p0_5 = p0_1;
            p0_6 = p0_3;
#endif
        }else{
#if CENTER_LINE
            //p0_0、p0_2のモニタが可能
            p0_5 = p0_0;
            p0_6 = p0_2;
#else
            p0_5 = ~p0_0;
            p0_6 = ~p0_2;
#endif
        }
		line_data = sensor();
        cnt0++;
        cnt1++;
}

// servo
// priority : 3 (High)
#pragma interrupt intTRBIC (vect=24)
void intTRBIC( void )
{
	ctrl_servo(deg);	// サーボ制御関数呼び出し
}

//------------------------------------------------------------------------------
// サーボ用A/D変換関数
// 引数         なし
// 戻り値       A/D変換結果 ( ピーク値除去済み平均値(2) )
//------------------------------------------------------------------------------
unsigned short get_adc(void){
	return ad_result;	// A/D変換結果 ( ピーク値除去済み平均値(2) ) を返す
}

//------------------------------------------------------------------------------
// センサー状態検出
// 引数         なし
// 戻り値       センサ値 + 情報更新フラグ
//------------------------------------------------------------------------------
unsigned char sensor( void )
{
        volatile unsigned char  data1;
#if CENTER_LINE
        data1 = ~p0;                   // ラインの色は白
#else
        data1 = p0;                    // ラインの色は黒
#endif
        data1 = data1 & 0x0f;

        data1 = data1 | 0x80;			// update flag set

        return( data1 );
}

//------------------------------------------------------------------------------
// 最新センサー状態検出
// 引数         なし
// 戻り値       センサ値 (情報更新フラグはクリアされている)
//------------------------------------------------------------------------------
unsigned char sensor_check( void ){
	unsigned char data;
	while( (line_data & 0x80) == 0x00 );	// update flag check
	line_data &= 0x7f;						// update flag clear
	data = line_data;			
	return data;
}

//------------------------------------------------------------------------------
// モーター速度制御
// 引数         左モータ:-100～100、右モータ:-100～100
//              0で停止、100で正転100%、-100で逆転100%
// 戻り値       なし
//------------------------------------------------------------------------------



void motor_org( int data1, int data2 )
{
        volatile int    motor_r;
        volatile int    motor_l;

        motor_l = (long)data1 * OPTION_L / 100;
        motor_r = (long)data2 * OPTION_R / 100;

        if( motor_l >= 0 ) {
			// CCW
            p2_1 = 0;
            p2_6 = 1;
            trdgrd0 = (long)( PWM_CYCLE - 1 ) * motor_l / 100;
        } else {
			// CW
            p2_1 = 1;
            p2_6 = 0;
            trdgrd0 = (long)( PWM_CYCLE - 1 ) * ( -motor_l ) / 100;
        }

        if( motor_r >= 0 ) {
			// Stop or CCW
            p2_3 = 0;
            p4_5 = 1;
            trdgrc1 = (long)( PWM_CYCLE - 1 ) * motor_r / 100;
        } else {
			// CW or Brake
            p2_3 = 1;
            p4_5 = 0;
            trdgrc1 = (long)( PWM_CYCLE - 1 ) * ( -motor_r ) / 100;
        }
}

void motor_fix( int data1, int data2 )
{
        volatile int    motor_r;
        volatile int    motor_l;

        motor_l = (long)data1 * OPTION_L / 100;
        motor_r = (long)data2 * OPTION_R / 100;

        if( motor_l >= 0 ) {
			// CCW
            p2_1 = 0;
            p2_6 = 1;
            trdgrd0 = (long)( PWM_CYCLE - 1 ) * motor_l / 100;
        } else {
			// CW
            p2_1 = 1;
            p2_6 = 0;
            trdgrd0 = (long)( PWM_CYCLE - 1 ) * ( -motor_l ) / 100;
        }

        if( motor_r >= 0 ) {
			// Stop or CCW
			p2_3 = 0;
            p2_7 = 1;
            trdgrc1 = (long)( PWM_CYCLE - 1 ) * motor_r / 100;
        } else {
			// CW or Brake
            p2_3 = 1;
            p2_7 = 0;
            trdgrc1 = (long)( PWM_CYCLE - 1 ) * ( -motor_r ) / 100;
        }
}

//------------------------------------------------------------------------------
// 時間稼ぎ
// 引数         タイマ値 1=1ms
// 戻り値       なし
//------------------------------------------------------------------------------
void timer( unsigned long data1 )
{
        cnt0 = 0;
        while( cnt0 < data1 );
}

//------------------------------------------------------------------------------
// 音を鳴らす
// 引数         (1/音の周波数)/(1/(クロック周波数/8))-1
// 戻り値       なし
//------------------------------------------------------------------------------
void beep( int data1 )
{
        trcgra = data1;                 // 周期の設定
        trcgrc = data1 / 2;             // デューティ50%のため周期の半分の値
}

//------------------------------------------------------------------------------
// プッシュスイッチ状態検出
// 引数         なし
// 戻り値       スイッチが押されていない場合:0、押された場合:1
//------------------------------------------------------------------------------
unsigned char pushsw( void )
{
        unsigned char data1;

        data1 = ~p2;
        data1 &= 0x01;

        return( data1 );
}

//------------------------------------------------------------------------------
// サーボ角度制御
// 引数         目標角度
// 戻り値       なし
//------------------------------------------------------------------------------
void ctrl_servo(int degree)
{
#define RIGHT_MAX  		(405)						// +40度のときのA/D変換結果
#define LEFT_MAX   		(615)						// -40度のときのA/D変換結果
#define CENTER_VAL 		((LEFT_MAX+RIGHT_MAX)/2)	// ±0度のときの値
#define GAIN       		(10)							// ゲイン
#define ABS(x)     		(((x)>=0)?(x):-(x))

	static int ad_data = 9999;
	static int ad_data_old;
	int pos = 0;	// 現在の角度(±40度)
	int diff = 0;	// 目標の角度と現在の角度の差
	int duty = 0;	// duty比指定用
	
	ad_data_old = ad_data;
	ad_data = get_adc();	// サーボの出力値(電圧)取得
	pos = -((ad_data - CENTER_VAL) * 40 / ((LEFT_MAX-RIGHT_MAX)/2));	// サーボの出力値(電圧)を「現在角度」に変換
	diff = deg - pos;		// 目標角度と現在角度の差分を計算
	duty = ABS(diff) * GAIN + ((PWM_CYCLE)>>4);		// 差に応じて回転速度(duty比)を決める
	if(ABS(duty) >= (PWM_CYCLE/2)){
		duty = PWM_CYCLE/2 -1;
	}

	if(diff < 0){						// 目標の角度より現在の角度が右の場合
		trdgrd1 = PWM_CYCLE/2 - duty;	// duty比の変更(左に回す)
	}
	else{								// 目標の角度より現在の角度が左の場合
		trdgrd1 = PWM_CYCLE/2 + duty;	// duty比の変更(右に回す)
	}
	p1_7 = 1;							// Servo 動作開始

	if( (ABS(diff) == 0) && ( ABS(ad_data - ad_data_old)<= 1) ){
		trdgrd1 = PWM_CYCLE/2;
		p1_7 = 0;						// Servo 動作停止
	}
}


//------------------------------------------------------------------------------
// サーボ角度設定
// 引数         目標角度
// 戻り値       なし
//------------------------------------------------------------------------------
void set_servo(int degree){
	deg = degree;		// 目標角度の更新
}


//------------------------------------------------------------------------------
// End of file
//------------------------------------------------------------------------------
