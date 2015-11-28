#ifndef	__MR2_LIB__

#define __MR2_LIB__

#include "sfr_r834c.h"

//------------------------------------------------------------------------------
// シンボル定義
//------------------------------------------------------------------------------
#define TIMER_CYCLE     (155)             // 1ms:0.001/(1/(20000000/128))-1

#define PWM_CYCLE       999             // 0.4ms:0.001/(1/(20000000/8))-1
//#define PWM_CYCLE       19999         // 8ms  :0.008/(1/(20000000/8))-1
//#define PWM_CYCLE       3999          // 1.6ms:0.0016/(1/(20000000/8))-1
//#define PWM_CYCLE       1999          // 0.8ms:0.0008/(1/(20000000/8))-1
//#define PWM_CYCLE       999           // 0.4ms:0.0004/(1/(20000000/8))-1
//#define PWM_CYCLE       499           // 0.2ms:0.0002/(1/(20000000/8))-1

#define Def_500Hz       4999            // 500Hz:(1/500)/(1/(20000000/8))-1
#define Def_1000Hz      2499            // 1000Hz:(1/1000)/(1/(20000000/8))-1

#define Def_C3          (19083)           // ド:(1/131)/(1/(20000000/8))-1
#define Def_D3          (17006)           // レ:(1/147)/(1/(20000000/8))-1
#define Def_E3          (15151)           // ミ:(1/165)/(1/(20000000/8))-1
#define Def_F3          (14285)           // ファ:(1/175)/(1/(20000000/8))-1
#define Def_G3          (12754)           // ソ:(1/196)/(1/(20000000/8))-1
#define Def_A3          (11362)           // ラ:(1/220)/(1/(20000000/8))-1
#define Def_B3          (10120)           // シ:(1/247)/(1/(20000000/8))-1
#define Def_C4          (9541)            // ド:(1/262)/(1/(20000000/8))-1
#define Def_C4          (9541)            // ド:(1/262)/(1/(20000000/8))-1
#define OFF				(0)

#define DI()            asm("FCLR I")   // 割り込み禁止
#define EI()            asm("FSET I")   // 割り込み許可

#define WHITE           (1)
#define BLACK           (0)

//------------------------------------------------------------------------------
// オプション設定
//------------------------------------------------------------------------------

#define CENTER_LINE     BLACK						// 中央ラインの色の設定

#define MOTOR_LIMIT		(100)						// モータ最大出力 : 60%
#define OPTION_L        (MOTOR_LIMIT)				// 最大回転数設定
#define OPTION_R        (MOTOR_LIMIT)

// LEDセンサー定義  ○：消灯   ●：点灯
#define SENSOR_AB		(0x08 | 0x04 | 0x02 | 0x01)	// 全点灯 : ●●●● (AB:All Black)
#define SENSOR_AW		(0x00)						// 全消灯 : ○○○○ (AW:All White)
#define SENSOR_LR0		(       0x04 | 0x02       )	// 中央   : ○●●○

#define SENSOR_L1		(       0x04              )	// 左(小) : ○●○○
#define SENSOR_L2		(0x08 | 0x04              )	// 左(中) : ●●○○
#define SENSOR_L3		(0x08                     )	// 左(大) : ●○○○
#define SENSOR_LC		(0x08 | 0x04 | 0x02       ) // 左曲   : ●●●○

#define SENSOR_R1		(              0x02       )	// 右(小) : ○○●○
#define SENSOR_R2		(              0x02 | 0x01)	// 右(中) : ○○●●
#define SENSOR_R3		(                     0x01)	// 右(大) : ○○○●
#define SENSOR_RC		(       0x04 | 0x02 | 0x01)	// 右曲   : ○●●●


#define	OOC_MAX			(500)

//------------------------------------------------------------------------------
// グローバル変数の宣言
//------------------------------------------------------------------------------

#define	TIMER_NUM		(4)
extern	unsigned long           timer_count[TIMER_NUM];	// タイマーカウンタ
extern	volatile unsigned char	line_data;		// 最新ラインパターン
extern  int distance_def[9];

#define TIMER_PERIOD        (1)
#define CONTROL_PERIOD      (10)
#define OPERATION_PERIOD    (100)
#define VEHICLE_PERIOD      (1000)

// Synchronize flag
extern  int timer_flag;
extern  int cont_flag;
extern  int opr_flag;
extern  int veh_flag;

// Communication data inter Multi-rate
extern  int resp_intg;      // RESPONSE_PERIOD区間の距離の積分値


//------------------------------------------------------------------------------
// プロトタイプ宣言
//------------------------------------------------------------------------------
extern	void clock_init( void );
extern	void peri_init( void );
extern	unsigned char sensor( void );
extern	unsigned char sensor_check( void );
extern	void motor( int, int );
extern	void timer( int, unsigned long );
extern	void beep( int );
extern	unsigned char pushsw( void );
extern  void senser_calibration_w ( void );
extern  void senser_calibration_b ( void );
extern  int get_position(void);
extern  void ledoff ( void );
extern  void ledmon (unsigned int );

#endif	// ifndef __MR2__LIB__
