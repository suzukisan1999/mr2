
#include "mr2_lib.h"

//------------------------------------------------------------------------------
// グローバル変数
//------------------------------------------------------------------------------


unsigned long	mr2_timer_count = 0;       // timer関数用
volatile char	mr2_line_data = 0;	// 最新ラインパターン

// position log
unsigned char	mr2_pos_log[POS_MAX] ={
	SENSOR_LR0,
	SENSOR_LR0,
	SENSOR_LR0,
	SENSOR_LR0,
	SENSOR_LR0,
	SENSOR_LR0,
	SENSOR_LR0,
	SENSOR_LR0,
	SENSOR_LR0,
	SENSOR_LR0	};
	
int	mr2_pos_ptr = (POS_MAX-1);


//------------------------------------------------------------------------------
// センサー状態検出
// 引数         なし
// 戻り値       センサ値 + 情報更新フラグ
//------------------------------------------------------------------------------
unsigned char mr2_sensor( void )
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
unsigned char mr2_sensor_check( void ){
	unsigned char data;
	while( (mr2_line_data & 0x80) == 0x00 );	// update flag check
	mr2_line_data &= 0x7f;						// update flag clear
	data = mr2_line_data;			
	return data;
}

//------------------------------------------------------------------------------
// モーター速度制御
// 引数         左モータ:-100〜100、右モータ:-100〜100
//              0で停止、100で正転100%、-100で逆転100%
// 戻り値       なし
//------------------------------------------------------------------------------
void mr2_motor( int data1, int data2 )
{
        volatile int    motor_r;
        volatile int    motor_l;

        motor_l = (long)data1 * OPTION_L / 100;
        motor_r = (long)data2 * OPTION_R / 100;

        if( motor_l >= 0 ) {
            p2_1 = 0;
            p2_6 = 1;
            trdgrd0 = (long)( PWM_CYCLE - 1 ) * motor_l / 100;
        } else {
            p2_1 = 1;
            p2_6 = 0;
            trdgrd0 = (long)( PWM_CYCLE - 1 ) * ( -motor_l ) / 100;
        }

        if( motor_r >= 0 ) {
            p2_3 = 0;
            p2_7 = 1;
            trdgrc1 = (long)( PWM_CYCLE - 1 ) * motor_r / 100;
        } else {
            p2_3 = 1;
            p2_7 = 0;
            trdgrc1 = (long)( PWM_CYCLE - 1 ) * ( -motor_r ) / 100;
        }
}

//------------------------------------------------------------------------------
// R8C/34Cの内蔵周辺機能の初期化
//------------------------------------------------------------------------------
void mr2_clock_init( void )
{
        volatile unsigned char i = 0;

        // 高速オンチップオシレータの設定
        prc0 = 1;

        fra1 = fra4;
        fra3 = fra5;
        fra2 = 0x00;
		
        fra00 = 1;
        while(i <= 50) i++;
        fra01 = 1;

        cm06 = 0;
        cm16 = 0;
        cm17 = 0;
        prc0 = 0;
		
}
void mr2_peri_init( void )
{
		
        // I/Oポートの入出力設定
        prc2 = 1;                       // pd0レジスタへの書き込み許可
        pd0 = 0xe0;                     // P0_0〜P0_3:センサ
										// P0_4(AN3) ロータリーエンコーダ
                                        // P0_5〜P0_7:LED
        prc2 = 0;                       // pd0レジスタへの書き込み禁止

        pd1 = 0xdf;                     // P1_0〜P1_3:LED
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
        trdpsr1 = 0x01;                 // TRDIOA1端子をP2_4に割り当て
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
		
}

//------------------------------------------------------------------------------
// 時間稼ぎ
// 引数         タイマ値 1=1ms
// 戻り値       なし
//------------------------------------------------------------------------------
void mr2_timer( unsigned long data1 )
{
        mr2_timer_count = 0;
        while( mr2_timer_count < data1 );
}

//------------------------------------------------------------------------------
// 音を鳴らす
// 引数         (1/音の周波数)/(1/(クロック周波数/8))-1
;// 戻り値       なし
//------------------------------------------------------------------------------
void mr2_beep( int data1 )
{
        trcgra = data1;                 // 周期の設定
        trcgrc = data1 / 2;             // デューティ50%のため周期の半分の値
}

//------------------------------------------------------------------------------
// プッシュスイッチ状態検出
// 引数         なし
// 戻り値       スイッチが押されていない場合:0、押された場合:1
//------------------------------------------------------------------------------
unsigned char mr2_pushsw( void )
{
        unsigned char data1;

        data1 = ~p2;
        data1 &= 0x01;

        return( data1 );
}


void	mr2_pos_push( unsigned char data )
{
	DI();
	mr2_pos_log[mr2_pos_ptr--] = data;
	if( mr2_pos_ptr<0 ){
		mr2_pos_ptr=POS_MAX-1;
	}
	EI();
	return;
}

// 0: 最新
// 9: 一番古い
unsigned char mr2_pos( unsigned char pos )
{
	pos = (mr2_pos_ptr+1+pos)%POS_MAX;
	return( mr2_pos_log[pos] );
}

void mr2_pos_all( unsigned char *pos )
{
	int i;
	
	pos += POS_MAX-1;
	
	for( i=0; i<POS_MAX; i++ ){
		*pos-- = mr2_pos_log[ (mr2_pos_ptr+1+i)%POS_MAX ];
	}
	return;
}

