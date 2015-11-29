/***********************************************************************/
/*                                                                     */
/*  FILE        :MiconRacer2.c                                         */
/*                                                                     */
/***********************************************************************/

//------------------------------------------------------------------------------
// インクルード
//------------------------------------------------------------------------------
#include <stdlib.h>
#include "mr2_lib.h"


//====================================================
//  レーンチェンジ／クランク切り替え
//		コメントアウトでクランクモード
//		共通化できなかった理由は、予告サインの検出を区別できなかったから。
//		レーンチェンジの方は、頭が少しゆれただけで安定しないため、
//		片側2本と、両側2本の区別が困難
//====================================================
//#define	LANECHANGE

//====================================================
//  汎用定義
//====================================================

#define	SATURATE( in, upper, lower )	(in > upper) ? upper : (in < lower) ? lower : in
#define	PUSH_LOG( data, array, max )	do{ for( ll=max-1; ll>0; ll-- ){ array[ll] = array[ll-1]; } array[0]=data; }while(0)
#define myabs(x)	((x) >= 0 ? (x) : -(x))


//====================================================
//  Timer周期で使うもの
//====================================================

// 周期起動フラグ
int timer_flag;		// センサデータ加工など		1ms周期
int cont_flag;		// 制御値の算出 			10ms周期
int opr_flag;		// 指令値（速度など）の算出	100ms周期
int veh_flag;		// 車両挙動などの算出 		500ms周期

// センサデータ
unsigned char sensor_val;

// センサ位置
#define	POSLOG_MAX		(2)
int pos_log[POSLOG_MAX] = {};

// センサ位置から距離を作る
#define DIS_CENTER	(4)
#define	DISTANCE(x)	distance_def[DIS_CENTER+x]
int distance_def[9] = { -1024, -672, -416, -160, 0, 160, 416, 672, 1024 };

// 蛇行検出用の左右ゆれ回数のカウンタ
int	left_side = 0;
int right_side = 0;


// 横ライン検出用のログ
//   シフトタイミングが独特。
//   横列状の黒白が切り替わるか、一定時間(Thline)でシフトする。
//   つまり、ログ10個分の時間は一定ではない。

#define	THhline			(5)		// 横ライン検出のライン認定時間(ms)
#define Thline			(500)	// 横ライン検出のログ送り時間

#define HLINE_MAX	(10)
int	hline_log[HLINE_MAX] = {};
int	hline_count = -1;


//====================================================
//  Controller周期で使うもの
//====================================================

// 出力制限
#define	POWER_MIN	(20)
#define	POWER_MAX	(80)


// 出力パワー履歴 （擬似的な速度算出用）
#define	POWLOG_MAX		(O_C_RATIO)
int power_log[POWLOG_MAX] = {};

// Controller周期での区間積分（距離）
int timer_intg;

// Contoroller周期での区間積分のログ（10回分)
#define	INTG_MAX		(O_C_RATIO)
int intg_log[INTG_MAX] = {};

// ブレーキ（挙動が激しいので使っていない）
#define	BRAKE_TIME		(3)		// n * CONTROL_PERIOD
int	brake    = 0;

// レーンチェンジ用
int	lean_change = 0;		// カウンタ
int scenario_leanchange[10]	// シナリオ
		= { 20, 20, 0, 0, 0,	-20, -20, 0, 0, 0 };

// PID制御パラメータ
float	Kl[] = { 0.06F,  0.2F, 0.003F };	// 横方向制御用

float	fKp = 0;	// これらはPIDを何箇所かで使うための代入係数
float	fKd = 0;	// 結局1箇所でしか使っていない
float	fKi = 0;
int Pr, Di, In;

// モータ制御量 （デバッグのためになんでもGlobalにおく）
int Speed   = 0;			// 現在速度
int	Speed_p = 0;			// ↑の1個前の値
int Dist    = 0;			// 現在のラインからの距離
int Dist_p  = 0;			// ↑の1個前の値

int Power   = POWER_MIN;	// 現在の出力値
int dv, dl;					// 速度、距離の制御値
int vr, vl;					// 最終的なモータへの入力

int	power_off = 0;			// 強制パワーオフ（使っていない）


//====================================================
//  Operation周期で使うもの
//====================================================

// 状態検出結果、ステート変更の入力
int Detect_dakou = 0;	// 蛇行検出
int Detect_edge  = 0;	// ラインの端っこにいる
int Detect_sign1 = 0;	// 直角サイン
int Detect_sign2 = 0;	// レーンチェンジサイン

// 速度指令
#define SPEED_HI	(800)
#define SPEED_DRIVE	(600)
#define SPEED_LO	(300)
#define SPEED_SLOW	(150)
int Target_Speed = SPEED_DRIVE;

// 時間指令
#define	COOLDOWN_TIME	(3)		// n * OPERATION_PERIOD
#define	HISPEED_TIME	(4)		// n * OPERATION_PERIOD
#define	SIGN1_TIME		(8)		// n * OPERATION_PERIOD
#define	SIGN2_TIME		(50)	// n * OPERATION_PERIOD

// カウンタ
int sign1    = 0;
int sign2    = 0;
int hispeed  = 0;
int cooldown = COOLDOWN_TIME;

// ステートマシン
#define		STATE_NORMAL	(0)
#define		STATE_DAKOU		(1)
#define		STATE_EDGE		(2)
#define		STATE_HISPEED	(3)
#define		STATE_SIGN1		(4)
#define		STATE_SIGN2		(5)
int state   = STATE_NORMAL;
int state_p = STATE_NORMAL;


//====================================================
//  Vehicle周期で使うもの
//====================================================

// 蛇行検出関連
#define	THdakou	(400)	// 蛇行検出用の値(積分値が一定異常で横にふれていると判断)
#define THldet	(5)
#define	DAKOU_MAX	(V_O_RATIO)
int	dakou_left_log[DAKOU_MAX] = {};
int	dakou_right_log[DAKOU_MAX] = {};
int dakou_ptr = 0;




//====================================================
//  雑多なおまじない
//====================================================
#pragma CREG	_flg_	flg
unsigned int	_flg_;


//====================================================
//  BEEP制御用
//====================================================
int beep_time = 0;
#define BEEP_PERIOD()			do{ if( beep_time > 0 ){ beep_time--; }else{ beep( OFF ); } }while(0)
#define BEEP(note, time)		do{ beep(note); beep_time = time; }while(0)
//#define BEEP(note, time)		do{}while(0)


/* 複数音を使うやつ */
#define	BEEP_TARGET		(0)
#define	BEEP_INTEG		(0)
#define	BEEP_LEAN_SCENARIO	(0)

/* 単音 */
#define BEEP_DAKOU			(OFF)
#define BEEP_EDGE			(OFF)
#define BEEP_NORMAL			(OFF)
#define BEEP_HISPEED		(OFF)
#define BEEP_SIGN1			(Def_C3)
#define BEEP_SIGN2			(Def_C3)

#define BEEP_SIGN1_START	(Def_C4)
#define BEEP_SIGN1_END		(OFF)
#define BEEP_SIGN2_START	(Def_C4)
#define BEEP_SIGN2_END		(OFF)
#define BEEP_DAKOU_START	(OFF)
#define BEEP_EDGE_START		(OFF)
#define BEEP_HISPEED_START	(OFF)
#define BEEP_HISPEED_END	(OFF)






//############################################################
//##  PID制御
//############################################################
int pid_float( int dist, int diff, int intg )
{
	int p, i, d;

	p = (int)( fKp * (float)dist );
	d = (int)( fKd * (float)diff );
	i = (int)( fKi * (float)intg );
		
	// output motor		
	return( p+d+i );
}




//############################################################
//##  センサ値の加工など
//############################################################
void   timer_main( void )
{
    static int dist_p = 0;
    static int intg   = 0;
    int dist;
	int i, j, ll;


	sensor_val = sensor_check();		// ラインセンサーから最新情報を取得

	// ポジション値アップデート
	for( i=POSLOG_MAX-1; i>0; i-- ){
        pos_log[i] = pos_log[i-1];
	}
    switch( sensor_val ){
        case SENSOR_LR0:	pos_log[0] =  0; break;
        case SENSOR_L1 :	pos_log[0] = -1; break;
        case SENSOR_L2 :	pos_log[0] = -2; break;
        case SENSOR_L3 :	pos_log[0] = -3; break;
        case SENSOR_LC :	pos_log[0] = -4; break;
        case SENSOR_R1 :	pos_log[0] = +1; break;
        case SENSOR_R2 :	pos_log[0] = +2; break;
        case SENSOR_R3 :	pos_log[0] = +3; break;
        case SENSOR_RC :	pos_log[0] = +4; break;
        case SENSOR_AB :	pos_log[0] = pos_log[1]; break;
        default :
			// 直角で、線の左側から進入すると左に曲がりながら、
			// ●○○●になって、さらに左に曲がるのを無理やり補正
			if( state == STATE_SIGN1 ){
                pos_log[0] = +4;
			}else{
                if( pos_log[1] > 0 ){
                    pos_log[0] = +4;
                } else if( pos_log[1] < 0 ){
                    pos_log[0] = -4;
                } else {
                    pos_log[0] = 0;
                }
			}
            break;
    }


    // 制御周期だったら、積分値を送信する
    if( cont_flag > 0 ){
        timer_intg = intg;
        intg = 0;
    }

    // 区間積分値
    dist   = DISTANCE(pos_log[0]);
    intg  += (( dist + dist_p )>>1);
	
    dist_p = dist;

	if( dist > THdakou ){
		right_side++;
	}else if( dist < -THdakou ){
		left_side++;
	}


	// レーンチェンジの終了を検出
	if( (lean_change>0) && (sensor_val != SENSOR_AW) ){
		sign2 = 0;
		lean_change = 0;
	}

	// レーンチェンジ中は無視
	if( (sign2==0) && (lean_change==0) ){

		// L3,R3だったら緊急減速
		if( (pos_log[0]==3) || (pos_log[0]==-3) ){
			Detect_edge = 1;
			Target_Speed = SPEED_LO;
		}
		// L3,R3より外だったら徐行
		if( (pos_log[0]>3) || (pos_log[0]<-3) ){
			Detect_edge = 1;
			Target_Speed = SPEED_SLOW;
		}
	}
	
	// 横ラインっぽいものをチェック
	if( hline_count>0 ){
		if(( sensor_val==SENSOR_AB)||(sensor_val==SENSOR_RC)||(sensor_val==SENSOR_LC)){
			if( hline_count < Thline ){
				hline_count++;
			}else{
				PUSH_LOG( hline_count, hline_log, HLINE_MAX );
			}
		}else{
			PUSH_LOG( hline_count, hline_log, HLINE_MAX );
			hline_count = -1;
		}
	}else{
		if(( sensor_val==SENSOR_AB)||(sensor_val==SENSOR_RC)||(sensor_val==SENSOR_LC)){
			PUSH_LOG( hline_count, hline_log, HLINE_MAX );
			hline_count = 1;
		}else{
			if( hline_count > -Thline ){
				hline_count--;
			}else{
				PUSH_LOG( hline_count, hline_log, HLINE_MAX );
			}
		}
	}

	return;
}


//############################################################
//##  制御量算出
//############################################################
void    control_main( void )
{
	int i, j;
	int	Kpow;
	int max;

	//====================================================
	//  縦方向の制御	
	//====================================================
	// 過去の出力Powerから擬似的な速度を算出
	// Target_Speed指示は0-1000の値で与えるので、
	// 算出Speedも最大値が1000になるように補正する
	
	Speed = 0;
	for( i=0; i<O_C_RATIO; i++ ){
		Speed = Speed + power_log[i];
	}
#define	SPEED_RATIO		((100*O_C_RATIO)/1000)
#if SPEED_RATIO != 1
	Speed = Speed / SPEED_RATIO;
#endif
	Speed = SATURATE( Speed, 1000, 0 );

	// 今回のSpeed制御値
	Pr = Target_Speed - Speed;
	if( myabs( Pr ) == 0 ){
		dv = 0;
	}else if( myabs( Pr )<200 ){
		dv = 1;
	}else if( myabs( Pr )<400 ){
		dv = 2;
	}else{
		dv = 3;
	}
	if( Pr < 0 ){
		dv = -2*dv;
	}

	//====================================================
	//  横方向の制御
	//====================================================
	Dist = DISTANCE(pos_log[0]);
	Pr = Dist;

	// 接近速度での制御（微分）
	Di = Dist - Dist_p;

	// カーブ追従（積分）
	for( i=O_C_RATIO-1; i>0; i-- ){
		intg_log[i] = intg_log[i-1];
	}
	intg_log[0] = timer_intg / C_T_RATIO;	// 値が大きくなりすぎるので、スケーリング

	

	In = 0;
	for( i=1; i<O_C_RATIO; i++ ){
		In = In + ((intg_log[i]+intg_log[i-1])>>1);
	}

#if BEEP_INTEG > 0
	        if( myabs(In) < 1000 ){
		BEEP( Def_C3, 10 );
	}else 	if( myabs(In) < 2000 ){
		BEEP( Def_D3, 10 );
	}else 	if( myabs(In) < 3000 ){
		BEEP( Def_E3, 10 );
	}else 	if( myabs(In) < 4000 ){
		BEEP( Def_F3, 10 );
	}else{
		BEEP( Def_G3, 10 );
	}		
#endif


	// PID
	fKp = Kl[0];
	fKd = Kl[1];
	fKi = Kl[2];	
	dl = pid_float( Pr, Di, In );
	dl = SATURATE( dl, 100, -100 );


	// レーンチェンジ	
	if((lean_change==-1)&&(sensor_val==SENSOR_AW)){
		lean_change = 199;
		BEEP(Def_C4, 100);
	}

	if( lean_change > 0 ){
		i = ( 200-lean_change )/20;
		dl = scenario_leanchange[i];
		lean_change--;
#if BEEP_LEAN_SCENARIO > 0
		switch( i ){
			case 0 : BEEP( Def_C3, 100 ); break;
			case 1 : BEEP( Def_D3, 100 ); break;
			case 2 : BEEP( Def_E3, 100 ); break;
			case 3 : BEEP( Def_F3, 100 ); break;
			case 4 : BEEP( Def_G3, 100 ); break;
			case 5 : BEEP( Def_A3, 100 ); break;
			case 6 : BEEP( Def_B3, 100 ); break;
			case 7 : BEEP( Def_C3, 100 ); break;
			case 8 : BEEP( Def_D3, 100 ); break;
			case 9 : BEEP( Def_E3, 100 ); break;
			default: BEEP( Def_C4, 100 ); break;
		}
#endif
	}

	//====================================================
	//  左右のバランス
	//====================================================
	vl = 100;
	vr = 100;

	// balancing
	if( dl > 0 ){
		vr = vr - myabs( dl );
	}else if( dl < 0 ){
		vl = vl - myabs( dl );
	}
	vl = SATURATE( vl, 100, 0 );
	vr = SATURATE( vr, 100, 0 );
	
	//====================================================
	//  車速の調整
	//====================================================
	Power = Power + dv;
	Power = SATURATE( Power, POWER_MAX, POWER_MIN );
	vl = ( Power * vl ) / 100;
	vr = ( Power * vr ) / 100;

	// ブレーキ指示
	if( brake>0 ){
		vl = 0;
		vr = 0;
	}

	// パワーオフ指示
	if( power_off>0 ){
		vl = 0;
		vr = 0;
	}

	//====================================================
    //  モータ出力
	//====================================================
	vl = SATURATE( vl, 100, 0 );
	vr = SATURATE( vr, 100, 0 );
	motor( vl, vr );

	//====================================================
    //  次の周期へのデータ保存
	//====================================================
    Dist_p  = Dist;
	Speed_p = Speed;

	if( brake > 0 ){
		brake--;
	}

	// Powerの累積
	for( i=O_C_RATIO-1; i>0; i-- ){
		power_log[i] = power_log[i-1];
	}
	power_log[0] = Power;

	return;
}


//############################################################
//##  指令値算出
//############################################################
void    operation_main( void )
{
	int	i, j;
	int detect_sign = 0;

	//====================================================
	//  ステート系ビープ
	//====================================================
#if BEEP_NORMAL > 0
	if(state==STATE_NORMAL){
		BEEP( BEEP_NORMAL, 20 );
	}		
#endif
#if BEEP_DAKOU > 0
	if(state==STATE_DAKOU){
		BEEP( BEEP_DAKOU, 20 );
	}		
#endif
#if BEEP_EDGE > 0
	if(state==STATE_EDGE){
		BEEP( BEEP_EDGE, 20 );
	}		
#endif
#if BEEP_HISPEED > 0
	if(state==STATE_HISPEED){
		BEEP( BEEP_HISPEED, 20 );
	}		
#endif
#if BEEP_SIGN1 > 0
	if(state==STATE_SIGN1){
		BEEP( BEEP_SIGN1, 20 );
	}		
#endif
#if BEEP_SIGN2 > 0
	if(state==STATE_SIGN2){
		BEEP( BEEP_SIGN2, 20 );
	}		
#endif

	//====================================================
	//  サイン検出
	//====================================================
	i = 0;
	detect_sign = 0;
	while( i<HLINE_MAX ){
		switch( detect_sign ){
			case 0 :	if( hline_log[i] >  THhline ){	detect_sign = 1;	}	break;
			case 1 :	if( hline_log[i] < -THhline ){	detect_sign = 2;	}	break;
			case 2 :	if( hline_log[i] >  THhline ){	detect_sign = 3;	}	break;
			default:	break;
		}
		i++;
	}
	if( detect_sign == 3 ){
#ifdef	LANECHANGE
		if( state != STATE_SIGN2 ){
			Detect_sign2 = 1;
		}
#else
		if( state != STATE_SIGN1 ){
			Detect_sign1 = 1;
		}
#endif
	}

	//====================================================
	//  蛇行検出用ログ
	//====================================================
	dakou_left_log[dakou_ptr]  = left_side;
	dakou_right_log[dakou_ptr] = right_side;
	dakou_ptr ++;
	dakou_ptr = dakou_ptr % DAKOU_MAX;
	left_side  = 0;
	right_side = 0;

	//====================================================
	//  走行ステート
	//====================================================
	if( (sign1==0) && (sign2==0) ){
		if( Detect_sign2>0 ){
			state = STATE_SIGN2;
			Detect_sign2 = 0;
		}else if( Detect_sign1>0 ){
			state = STATE_SIGN1;
			Detect_sign1 = 0;
		}else  if( Detect_dakou ){
			state = STATE_DAKOU;
			Detect_dakou = 0;
		}else if( Detect_edge ){
			state = STATE_EDGE;
			Detect_edge = 0;
		}else if( hispeed==0 ){
			if( cooldown == 0 ){
				state = STATE_HISPEED;
			}else{
				state = STATE_NORMAL;
			}
		}
	}

	//====================================================
	// State変化に伴う処理
	//====================================================

	// クランクモード 開始
	if( (state_p!=STATE_SIGN1)&&(state==STATE_SIGN1) ){
		sign1    = SIGN1_TIME;
		hispeed  = 0;
#if BEEP_SIGN1_START > 0
	BEEP( BEEP_SIGN1_START, 50 );
#endif
	}

	// クランクモード 終了
	if( (state_p==STATE_SIGN1)&&(state!=STATE_SIGN1) ){
#if BEEP_SIGN1_END > 0
	BEEP( BEEP_SIGN1_END, 50 );
#endif
	}

	// レーンチェンジモード 開始
	if( (state_p!=STATE_SIGN2)&&(state==STATE_SIGN2) ){
		sign2       = SIGN2_TIME;
		lean_change = -1;
		hispeed     = 0;
#if BEEP_SIGN2_START > 0
	BEEP( BEEP_SIGN2_START, 50 );
#endif
	}

	// レーンチェンジモード 終了
	if( (state_p==STATE_SIGN2)&&(state!=STATE_SIGN2) ){
		lean_change = 0;
#if BEEP_SIGN2_END > 0
	BEEP( BEEP_SIGN2_END, 50 );
#endif
	}

	// 蛇行検出モード 開始
	if( (state_p!=STATE_DAKOU)&&(state==STATE_DAKOU) ){
		hispeed  = 0;
#if BEEP_DAKOU_START > 0
	BEEP( BEEP_DAKOU_START, 50 );
#endif
	}

	// ラインエッジ検出モード 開始
	if( (state_p!=STATE_EDGE)&&(state==STATE_EDGE) ){
		hispeed  = 0;
#if BEEP_EDGE_START > 0
	BEEP( BEEP_EDGE_START, 50 );
#endif
	}

	// ハイスピードモード 開始
	if( (state_p!=STATE_HISPEED)&&(state==STATE_HISPEED) ){
		hispeed = HISPEED_TIME;
		cooldown = COOLDOWN_TIME + HISPEED_TIME;
#if BEEP_HISPEED_START > 0
	BEEP( BEEP_HISPEED_START, 50 );
#endif
	}

	// ハイスピードモード 終了
	if( (state_p==STATE_HISPEED)&&(state!=STATE_HISPEED) ){
#if BEEP_HISPEED_END > 0
	BEEP( BEEP_HISPEED_END, 50 );
#endif
	}


	//====================================================
	// 速度指令 生成
	//====================================================
	switch ( state ) {
		case STATE_SIGN1   :	Target_Speed = SPEED_SLOW;	break;
		case STATE_SIGN2   :	Target_Speed = SPEED_DRIVE;	break;
		case STATE_DAKOU   :	Target_Speed = SPEED_LO;	break;
		case STATE_EDGE    :	Target_Speed = SPEED_LO;	break;
		case STATE_HISPEED :	Target_Speed = SPEED_HI;	break;
		default            :	Target_Speed = SPEED_DRIVE;	break;
	}
	
#if BEEP_TARGET > 0
	switch ( Target_Speed ) {
		case SPEED_SLOW  :	BEEP( Def_C3, 20 ); break;
		case SPEED_LO    :	BEEP( Def_E3, 20 ); break;
		case SPEED_DRIVE :	BEEP( Def_G3, 20 ); break;
		case SPEED_HI    :	BEEP( Def_B3, 20 ); break;
	}
#endif

	//====================================================
    //  次の周期へのデータ保存／更新
	//====================================================
	if( hispeed > 0 ){
		hispeed--;
	}
	if( cooldown > 0 ){
		cooldown--;
	}
	if( sign1 > 0 ){
		sign1--;
	}
	if( sign2 > 0 ){
		sign2--;
	}
	state_p = state;

	return;
}


//############################################################
//##  車両挙動 （蛇行ぐらいしかない）
//############################################################
void	vehicle_main( void )
{
	int i;
	int left, right;
	
	right = 0;
	left  = 0;
	for( i=0; i<DAKOU_MAX; i++ ){
		right += dakou_right_log[i];
		left  += dakou_left_log[i];
	}
	if( (right>0) && (left>0) ){
		Detect_dakou = 1;
	}else{
		Detect_dakou = 0;
	}
	
}

//############################################################
//##  メイン関数
//############################################################
void main(void)
{
	int i;
	
	// 初期化
	DI();
	if( (_flg_ & 0x02) == 0 ){
		clock_init();
	}
	peri_init();
	EI();

#if 1
	while( pushsw() == 0 );	// スタートボタン押下待ち
    timer( 0, 100 );
	while( pushsw() != 0 );	// スタートボタン開放待ち
#endif

	// メインループ
    while(1){

        if( timer_flag > 0 ){
			timer_main();
            timer_flag = 0;
        }
        if( cont_flag > 0 ){
            control_main();
            cont_flag = 0;
        }
        if( opr_flag > 0 ){
			operation_main();
            opr_flag = 0;
        }
		if( veh_flag > 0 ){
			vehicle_main();
			veh_flag = 0;
		}
		
		// ビープ出力の停止
		BEEP_PERIOD();
	}

}


//------------------------------------------------------------------------------
// End of file
//------------------------------------------------------------------------------
