#ifndef	__MR2_LIB__

#define __MR2_LIB__

#include "sfr_r834c.h"

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

#define MOTOR_LIMIT		(80)						// モータ最大出力 : 60%
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

#define	OOC_MAX			(500)

//------------------------------------------------------------------------------
// グローバル変数の宣言
//------------------------------------------------------------------------------

extern	unsigned long           mr2_timer_count;	// タイマーカウンタ
extern	volatile unsigned char	mr2_line_data;		// 最新ラインパターン

#define	POS_MAX		(10)
//extern	volatile unsigned char	mr2_pos_log[POS_MAX];
//extern	volatile unsigned char	mr2_pos_ptr;


//------------------------------------------------------------------------------
// プロトタイプ宣言
//------------------------------------------------------------------------------

extern	void mr2_clock_init( void );
extern	void mr2_peri_init( void );
extern	unsigned char mr2_sensor( void );
extern	unsigned char mr2_sensor_check( void );
extern	void mr2_motor( int, int );
extern	void mr2_timer( unsigned long );
extern	void mr2_beep( int );
extern	unsigned char mr2_pushsw( void );

extern	void	mr2_pos_push( unsigned char );
extern	unsigned char mr2_pos( unsigned char );
extern	void mr2_pos_all( unsigned char * );

#endif	// ifndef __MR2_LIB__