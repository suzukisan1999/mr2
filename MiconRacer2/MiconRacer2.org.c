/***********************************************************************/
/*                                                                     */
/*  FILE        :MiconRacer2.c                                         */
/*  DATE        :Sat, Nov 14, 2015                                     */
/*  DESCRIPTION :main program file.                                    */
/*  CPU GROUP   :34C                                                   */
/*                                                                     */
/*  This file is generated by Renesas Project Generator (Ver.4.19).    */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/***********************************************************************/

//------------------------------------------------------------------------------
// インクルード
//------------------------------------------------------------------------------
#include <stdlib.h>
#include "mr2_lib.h"

#pragma CREG	_flg_	flg
unsigned int	_flg_;

#define BEEP(x)		mr2_beep(x)
//#define BEEP(x)		do{}while(0)

	
//------------------------------------------------------------------------------
// メインプログラム
//------------------------------------------------------------------------------
#define	SCALING		(10)
long  coeff_fixed[3] = { 10240, 0, 15360};	
float coeff_float[3] = { 0.1, 1.0, 0.0001 };	
float dt = 0.001;
int pos_log[POS_MAX];

int pid_fixed( int dist, int dist_p )
{

    int diff;
    static int intg;
    long p, i, d;
    int v, m;

    diff = dist-dist_p;
    intg = 0;

    p = (coeff_fixed[0]*dist)>>SCALING;
	i = (coeff_fixed[1]*intg)>>SCALING;
    d = (coeff_fixed[2]*diff)>>SCALING;

    v = p + i + d;
    m = v>>(SCALING-2);

    return( m );
}

int pid_float( int dist )
{
	int a;
	
    float dist_p, diff;
    static float intg = 0;
    float p, i, d;

	dist_p = (float)pos_log[1];
    diff   = ((float)dist-dist_p)*dt;
/*
	for( a=0; a<POS_MAX; a++ ){
 		intg  += ((float)pos_log[a]+(float)pos_log[a+1])/2*dt;
	}
*/
    p = coeff_float[0] * (float)dist;
	i = coeff_float[1] * intg;
    d = coeff_float[2] * diff;

    return( (int)( p + i + d ));
}


void run_main( void )
{
	unsigned char read_line = 0;
	int i;
	int	pos_log[POS_MAX];
	int	ooc;
	int	oop;
	int	pos;

#define CENTER	(4)
	int distance_def[9] = { -1024, -672, -416, -160, 0, 160, 416, 672, 1024 };
	int vr, vl;
	int dist, dist_p, dist_sum, dist_op, r_factor;
	static int dv = 0;
	
	dist=0;
	dist_p=0;	
	ooc = 0;
    while(1){		
		read_line = mr2_sensor_check();		// ラインセンサーから最新情報を取得

		// position log
		for( i=POS_MAX-1; i>0; i-- ){
			pos_log[i] = pos_log[i-1];
		}

		switch( read_line ){
			case SENSOR_LR0:	pos = distance_def[CENTER  ]; ooc = 0;	break;
			case SENSOR_L1 :	pos = distance_def[CENTER-1]; ooc = 0;	break;
			case SENSOR_L2 :	pos = distance_def[CENTER-2]; ooc = 0;	break;
			case SENSOR_L3 :	pos = distance_def[CENTER-3]; ooc = 0;	break;
			case SENSOR_R1 :	pos = distance_def[CENTER+1]; ooc = 0;	break;
			case SENSOR_R2 :	pos = distance_def[CENTER+2]; ooc = 0;	break;
			case SENSOR_R3 :	pos = distance_def[CENTER+3]; ooc = 0;	break;
			case SENSOR_AW :
				if( pos_log[1] > 0 ){
					pos = distance_def[CENTER+4];
				} else if( pos_log[1] < 0 ){
					pos = distance_def[CENTER-4];
				} else {
					pos = distance_def[CENTER];
				}
				ooc++;
				break;
			default :
				pos = 0;
				ooc++;
				break;
		}
		pos_log[0] = pos;
		
		// cource rounding factor		
		r_factor = (pos_log[0]+pos_log[1]+pos_log[2])/3;

		// PID control
		dist     = pos_log[0] + r_factor;
		dist_op  = pid_float( dist );

		// output motor		
		dv = dist_op;
		oop=0;
		if( dv > 100 ){ vr = 100; oop=1; BEEP( Def_C4 ); }
		if( dv <   0 ){ vr =   0; oop=1; BEEP( Def_C3 ); }
		if( oop == 0 ){ BEEP( 0 ); }

		vr = (100-abs(dv))-dv;
		vl = (100-abs(dv))+dv;

		if( abs(pos_log[0])>600 ){
			vr = (int)( (float)vr * 0.8 );
			vl = (int)( (float)vl * 0.8 );
		}else if ( abs(pos_log[0]>1000 )){
			vr = (int)( (float)vr * 0.6 );
			vl = (int)( (float)vl * 0.6 );
		}
		
/*
		vr = (100-abs(dv))+dv;
		vl = (100-abs(dv))-dv;

		oop=0;
		if( vr > 100 ){ vr = 100; oop=1; BEEP( Def_C4 ); }
		if( vr <   0 ){ vr =   0; oop=1; BEEP( Def_C3 ); }
		if( vl > 100 ){ vl = 100; oop=1; BEEP( Def_C4 ); }		
		if( vl <   0 ){ vl =   0; oop=1; BEEP( Def_C3 ); }
		if( oop == 0 ){ BEEP( 0 ); }
*/

		if( ooc>300 ){
			vr = 0;
			vl = 0;
		}
		
		mr2_motor( vr, vl );

	}	// while(1)

	return;
}

void main(void)
{
	DI();
	if( (_flg_ & 0x02) == 0 ){
		mr2_clock_init();
	}
	mr2_peri_init();
	EI();

	//while( mr2_pushsw() == 0 );	// スタートボタン押下待ち

	run_main();
}


//------------------------------------------------------------------------------
// End of file
//------------------------------------------------------------------------------
