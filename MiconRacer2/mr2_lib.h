#ifndef	__LIB__

#define __LIB__

#include "sfr_r834c.h"

//------------------------------------------------------------------------------
// V{θ`
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

#define Def_C3          (19083)           // h:(1/131)/(1/(20000000/8))-1
#define Def_D3          (17006)           // :(1/147)/(1/(20000000/8))-1
#define Def_E3          (15151)           // ~:(1/165)/(1/(20000000/8))-1
#define Def_F3          (14285)           // t@:(1/175)/(1/(20000000/8))-1
#define Def_G3          (12754)           // \:(1/196)/(1/(20000000/8))-1
#define Def_A3          (11362)           // :(1/220)/(1/(20000000/8))-1
#define Def_B3          (10120)           // V:(1/247)/(1/(20000000/8))-1
#define Def_C4          (9541)            // h:(1/262)/(1/(20000000/8))-1
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
#define OPTION_L        (MOTOR_LIMIT)				// Εερ]έθ
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

extern	unsigned long           timer_count;	// ^C}[JE^
extern	volatile unsigned char	line_data;		// ΕVCp^[

//------------------------------------------------------------------------------
// vg^CvιΎ
//------------------------------------------------------------------------------

extern	void clock_init( void );
extern	void peri_init( void );
extern	unsigned char sensor( void );
extern	unsigned char sensor_check( void );
extern	void motor( int, int );
extern	void timer( unsigned long );
extern	void beep( int );
extern	unsigned char pushsw( void );

#endif	// ifndef __LIB__