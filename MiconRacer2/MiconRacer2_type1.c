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

#define BEEP_MAX	(100)
#define BEEP(x)		beep(x)
//#define BEEP(x)		do{}while(0)


#define	BEEP_LOGERR		OFF
#define	BEEP_TOBI		OFF
#define	BEEP_DAKOU		OFF
#define	BEEP_POWLOW		OFF
#define BEEP_POWMAX		OFF

#define CENTER	(4)
#define	DISTANCE(x)	distance_def[CENTER+pos_log[x]]
#define	SATURATE( in, upper, lower )	(in > upper) ? upper : (in < lower) ? lower : in

#define	POWER_HALF (int)((float)500 * (50.0F/(float)MOTOR_LIMIT))
#define	POWER_SLOW (int)((float)750 * (50.0F/(float)MOTOR_LIMIT))

	
//------------------------------------------------------------------------------
// メインプログラム
//------------------------------------------------------------------------------
int pos_log[POS_MAX];

void run_main( void )
{
	unsigned char read_line = 0;
	int i;
	int	ooc;
	int	oop;
	int	pos;

	int distance_def[9] = { -1024, -672, -416, -160, 0, 160, 416, 672, 1024 };
	int dv, vr, vl;
	int dist, diff, intg;
	int	p_factor, i_factor, d_factor, r_factor;
	float coeff[3];
	volatile float tmp1, tmp2, tmp3;

	int tobi, dakou, instab;
	int side_l, side_r;
	
	int beep_count;
	int error;
	int power_down, power_up, power_off;
	int	power;

	ooc = 0;
	beep_count = 0;
	power = 100;
    while(1){		
		read_line = sensor_check();		// ラインセンサーから最新情報を取得

		// position log
		error = 0;
		for( i=POS_MAX-1; i>0; i-- ){
			pos_log[i] = pos_log[i-1];
			     if( pos_log[i] >  1024 ){ error = 1; }
			else if( pos_log[i] < -1024 ){ error = 1; }
		}
#if BEEP_LOGERR > 0
		if( error ){
			BEEP( BEEP_LOGERR );
			beep_count = BEEP_MAX;
		}
#endif

		switch( read_line ){
			case SENSOR_LR0:	pos =  0; ooc = 0;	break;
			case SENSOR_L1 :	pos = -1; ooc = 0;	break;
			case SENSOR_L2 :	pos = -2; ooc = 0;	break;
			case SENSOR_L3 :	pos = -3; ooc = 0;	break;
			case SENSOR_LC :	pos = -4; ooc = 0;	break;
			case SENSOR_R1 :	pos = +1; ooc = 0;	break;
			case SENSOR_R2 :	pos = +2; ooc = 0;	break;
			case SENSOR_R3 :	pos = +3; ooc = 0;	break;
			case SENSOR_RC :	pos = +4; ooc = 0;	break;
			default :
				if( pos_log[1] > 0 ){
					pos = +4;
				} else if( pos_log[1] < 0 ){
					pos = -4;
				} else {
					pos = 0;
				}
				ooc++;
				break;
		}
		pos_log[0] = pos;

		// 安定性の評価
		
		// センサ飛び検出
		tobi = (abs(pos_log[0]-pos_log[1])>1);
#if BEEP_TOBI > 0
		if( tobi > 0 ){
			BEEP( BEEP_TOBI );
			beep_count = BEEP_MAX;
		}
#endif
		
		// 蛇行検出
		side_l = 0;
		side_r = 0;
		for (i=0; i<POS_MAX-1; i++ ){
			if( pos_log[i] < -2 ){
				side_l = 1;
			}
			if( pos_log[i] > 2 ){
				side_r = 1;
			}
		}
		dakou = 0;
		if( (side_l==1) && (side_r==1) ){
			dakou = 1;
		}
#if BEEP_DAKOU > 0
		if( dakou > 0 ){
			BEEP( BEEP_DAKOU );
			beep_count = BEEP_MAX;
		}
#endif
		
		//tobi=0;
		//dakou=0;
		// 速度制限
		if( (tobi==0) && (dakou==0) ){
			instab = 0;
		}else{
			instab++;
		}
	
		// 距離での制御
		dist = DISTANCE(0);
		
		// 接近速度での制御（微分）
		diff = DISTANCE(0)-DISTANCE(1);

		// カーブ追従（積分）
		intg =	((DISTANCE(0)+DISTANCE(1))>>1) +
				((DISTANCE(1)+DISTANCE(2))>>1) +
				((DISTANCE(2)+DISTANCE(3))>>1) +
				((DISTANCE(3)+DISTANCE(4))>>1) +
				((DISTANCE(4)+DISTANCE(5))>>1);

		// PID
		coeff[0] = 0.0002F;
		coeff[1] = 0.03F;
		coeff[2] = 0.005;
//		coeff[2] = 0;

		p_factor = (int)( coeff[0] * (float)dist * (float)abs(dist) );
		d_factor = (int)( coeff[1] * (float)diff );
		i_factor = (int)( coeff[2] * (float)intg );
		
		// output motor		
		dv = p_factor + d_factor + i_factor;
		SATURATE( dv, 100, -100 );
		

		// パワーダウン指示
		power_down = (instab>10) || (abs(intg)>3000);
		power_up   = (abs(intg)<1500);
		
		// 出力調整
		if( power_down ){
			power -= 4;
		}else if( power_up ){
			power += 1;
		}
		if( abs(dist) > 700 ){
			power = SATURATE( power, 1000, POWER_HALF );
		}else{
			power = SATURATE( power, 1000, POWER_SLOW );
		}
		
#if BEEP_POWLOW > 0
		if( power < POWER_SLOW ){
			BEEP( BEEP_POWLOW );
			beep_count = BEEP_MAX;
		}
#endif
#if BEEP_POWMAX > 0
		if( power >= 1000 ){
			BEEP( BEEP_POWMAX );
			beep_count = BEEP_MAX;
		}
#endif
		
		vr = power/10;
		vl = power/10;

		// balancing
		if( dv > 0 ){
			vr = vr - dv;
		}else if( dv < 0 ){
			vl = vl + dv;
		}
		vr = SATURATE( vr, 100, 0 );
		vl = SATURATE( vl, 100, 0 );

		// パワーオフ指示
		power_off = ooc > 500;
		if( power_off ){
			vr = 0;
			vl = 0;
		}

		motor( vl, vr );
		
		// Beep 停止
		beep_count--;
		if( beep_count <= 0 ){
			BEEP( 0 );
		}
			

	}	// while(1)

	return;
}

void main(void)
{
	DI();
	if( (_flg_ & 0x02) == 0 ){
		clock_init();
	}
	peri_init();
	EI();

	while( pushsw() == 0 );	// スタートボタン押下待ち
	run_main();
}


//------------------------------------------------------------------------------
// End of file
//------------------------------------------------------------------------------
