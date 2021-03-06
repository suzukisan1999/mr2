#ifndef	__MR2_LIB__

#define __MR2_LIB__

#include "sfr_r834c.h"

//------------------------------------------------------------------------------
// V{θ`
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

#define Def_C3          (19083)           // h:(1/131)/(1/(20000000/8))-1
#define Def_D3          (17006)           // :(1/147)/(1/(20000000/8))-1
#define Def_E3          (15151)           // ~:(1/165)/(1/(20000000/8))-1
#define Def_F3          (14285)           // t@:(1/175)/(1/(20000000/8))-1
#define Def_G3          (12754)           // \:(1/196)/(1/(20000000/8))-1
#define Def_A3          (11362)           // :(1/220)/(1/(20000000/8))-1
#define Def_B3          (10120)           // V:(1/247)/(1/(20000000/8))-1
#define Def_C4          (9541)            // h:(1/262)/(1/(20000000/8))-1
#define OFF				(0)

#define DI()            asm("FCLR I")   // θέΦ~
#define EI()            asm("FSET I")   // θέΒ

#define WHITE           (1)
#define BLACK           (0)

//------------------------------------------------------------------------------
// IvVέθ
//------------------------------------------------------------------------------

#define CENTER_LINE     BLACK						// CΜFΜέθ

#define MOTOR_LIMIT		(100)						// [^ΕεoΝ : 60%
#define OPTION_L        (MOTOR_LIMIT*0.857)				// Εερ]έθ
#define OPTION_R        (MOTOR_LIMIT)

// LEDZT[θ`  FΑ   F_
#define SENSOR_AB		(0x08 | 0x04 | 0x02 | 0x01)	// S_ :  (AB:All Black)
#define SENSOR_AW		(0x00)						// SΑ :  (AW:All White)
#define SENSOR_LR0		(       0x04 | 0x02       )	//    : 

#define SENSOR_L1		(       0x04              )	// Ά(¬) : 
#define SENSOR_L2		(0x08 | 0x04              )	// Ά() : 
#define SENSOR_L3		(0x08                     )	// Ά(ε) : 
#define SENSOR_LC		(0x08 | 0x04 | 0x02       ) // ΆΘ   : 

#define SENSOR_R1		(              0x02       )	// E(¬) : 
#define SENSOR_R2		(              0x02 | 0x01)	// E() : 
#define SENSOR_R3		(                     0x01)	// E(ε) : 
#define SENSOR_RC		(       0x04 | 0x02 | 0x01)	// EΘ   : 


#define	OOC_MAX			(500)

//------------------------------------------------------------------------------
// O[oΟΜιΎ
//------------------------------------------------------------------------------

#define	TIMER_NUM		(4)
extern	unsigned long           timer_count[TIMER_NUM];	// ^C}[JE^
extern	volatile unsigned char	line_data;		// ΕVCp^[
extern  int distance_def[9];

#define TIMER_PERIOD        (1)
#define CONTROL_PERIOD      (10)
#define OPERATION_PERIOD    (100)
#define VEHICLE_PERIOD      (500)

#define	C_T_RATIO	(CONTROL_PERIOD   / TIMER_PERIOD    )
#define	O_C_RATIO	(OPERATION_PERIOD / CONTROL_PERIOD  )
#define	V_O_RATIO	(VEHICLE_PERIOD   / OPERATION_PERIOD)

// Synchronize flag
extern  int timer_flag;
extern  int cont_flag;
extern  int opr_flag;
extern  int veh_flag;

// Communication data inter Multi-rate
extern  int resp_intg;      // RESPONSE_PERIODζΤΜ£ΜΟͺl


//------------------------------------------------------------------------------
// vg^CvιΎ
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
