/***********************************************************************/
/*                                                                     */
/*  FILE        :MiconRacer2.c                                         */
/*                                                                     */
/***********************************************************************/

//------------------------------------------------------------------------------
// �C���N���[�h
//------------------------------------------------------------------------------
#include <stdlib.h>
#include "mr2_lib.h"


//====================================================
//  ���[���`�F���W�^�N�����N�؂�ւ�
//		�R�����g�A�E�g�ŃN�����N���[�h
//		���ʉ��ł��Ȃ��������R�́A�\���T�C���̌��o����ʂł��Ȃ���������B
//		���[���`�F���W�̕��́A����������ꂽ�����ň��肵�Ȃ����߁A
//		�Б�2�{�ƁA����2�{�̋�ʂ�����
//====================================================
//#define	LANECHANGE

//====================================================
//  �ėp��`
//====================================================

#define	SATURATE( in, upper, lower )	(in > upper) ? upper : (in < lower) ? lower : in
#define	PUSH_LOG( data, array, max )	do{ for( ll=max-1; ll>0; ll-- ){ array[ll] = array[ll-1]; } array[0]=data; }while(0)
#define myabs(x)	((x) >= 0 ? (x) : -(x))


//====================================================
//  Timer�����Ŏg������
//====================================================

// �����N���t���O
int timer_flag;		// �Z���T�f�[�^���H�Ȃ�		1ms����
int cont_flag;		// ����l�̎Z�o 			10ms����
int opr_flag;		// �w�ߒl�i���x�Ȃǁj�̎Z�o	100ms����
int veh_flag;		// �ԗ������Ȃǂ̎Z�o 		500ms����

// �Z���T�f�[�^
unsigned char sensor_val;

// �Z���T�ʒu
#define	POSLOG_MAX		(2)
int pos_log[POSLOG_MAX] = {};

// �Z���T�ʒu���狗�������
#define DIS_CENTER	(4)
#define	DISTANCE(x)	distance_def[DIS_CENTER+x]
int distance_def[9] = { -1024, -672, -416, -160, 0, 160, 416, 672, 1024 };

// �֍s���o�p�̍��E���񐔂̃J�E���^
int	left_side = 0;
int right_side = 0;


// �����C�����o�p�̃��O
//   �V�t�g�^�C�~���O���Ɠ��B
//   �����̍������؂�ւ�邩�A��莞��(Thline)�ŃV�t�g����B
//   �܂�A���O10���̎��Ԃ͈��ł͂Ȃ��B

#define	THhline			(5)		// �����C�����o�̃��C���F�莞��(ms)
#define Thline			(500)	// �����C�����o�̃��O���莞��

#define HLINE_MAX	(10)
int	hline_log[HLINE_MAX] = {};
int	hline_count = -1;


//====================================================
//  Controller�����Ŏg������
//====================================================

// �o�͐���
#define	POWER_MIN	(20)
#define	POWER_MAX	(80)


// �o�̓p���[���� �i�[���I�ȑ��x�Z�o�p�j
#define	POWLOG_MAX		(O_C_RATIO)
int power_log[POWLOG_MAX] = {};

// Controller�����ł̋�Ԑϕ��i�����j
int timer_intg;

// Contoroller�����ł̋�Ԑϕ��̃��O�i10��)
#define	INTG_MAX		(O_C_RATIO)
int intg_log[INTG_MAX] = {};

// �u���[�L�i�������������̂Ŏg���Ă��Ȃ��j
#define	BRAKE_TIME		(3)		// n * CONTROL_PERIOD
int	brake    = 0;

// ���[���`�F���W�p
int	lean_change = 0;		// �J�E���^
int scenario_leanchange[10]	// �V�i���I
		= { 20, 20, 0, 0, 0,	-20, -20, 0, 0, 0 };

// PID����p�����[�^
float	Kl[] = { 0.06F,  0.2F, 0.003F };	// ����������p

float	fKp = 0;	// ������PID�����ӏ����Ŏg�����߂̑���W��
float	fKd = 0;	// ����1�ӏ��ł����g���Ă��Ȃ�
float	fKi = 0;
int Pr, Di, In;

// ���[�^����� �i�f�o�b�O�̂��߂ɂȂ�ł�Global�ɂ����j
int Speed   = 0;			// ���ݑ��x
int	Speed_p = 0;			// ����1�O�̒l
int Dist    = 0;			// ���݂̃��C������̋���
int Dist_p  = 0;			// ����1�O�̒l

int Power   = POWER_MIN;	// ���݂̏o�͒l
int dv, dl;					// ���x�A�����̐���l
int vr, vl;					// �ŏI�I�ȃ��[�^�ւ̓���

int	power_off = 0;			// �����p���[�I�t�i�g���Ă��Ȃ��j


//====================================================
//  Operation�����Ŏg������
//====================================================

// ��Ԍ��o���ʁA�X�e�[�g�ύX�̓���
int Detect_dakou = 0;	// �֍s���o
int Detect_edge  = 0;	// ���C���̒[�����ɂ���
int Detect_sign1 = 0;	// ���p�T�C��
int Detect_sign2 = 0;	// ���[���`�F���W�T�C��

// ���x�w��
#define SPEED_HI	(800)
#define SPEED_DRIVE	(600)
#define SPEED_LO	(300)
#define SPEED_SLOW	(150)
int Target_Speed = SPEED_DRIVE;

// ���Ԏw��
#define	COOLDOWN_TIME	(3)		// n * OPERATION_PERIOD
#define	HISPEED_TIME	(4)		// n * OPERATION_PERIOD
#define	SIGN1_TIME		(8)		// n * OPERATION_PERIOD
#define	SIGN2_TIME		(50)	// n * OPERATION_PERIOD

// �J�E���^
int sign1    = 0;
int sign2    = 0;
int hispeed  = 0;
int cooldown = COOLDOWN_TIME;

// �X�e�[�g�}�V��
#define		STATE_NORMAL	(0)
#define		STATE_DAKOU		(1)
#define		STATE_EDGE		(2)
#define		STATE_HISPEED	(3)
#define		STATE_SIGN1		(4)
#define		STATE_SIGN2		(5)
int state   = STATE_NORMAL;
int state_p = STATE_NORMAL;


//====================================================
//  Vehicle�����Ŏg������
//====================================================

// �֍s���o�֘A
#define	THdakou	(400)	// �֍s���o�p�̒l(�ϕ��l�����ُ�ŉ��ɂӂ�Ă���Ɣ��f)
#define THldet	(5)
#define	DAKOU_MAX	(V_O_RATIO)
int	dakou_left_log[DAKOU_MAX] = {};
int	dakou_right_log[DAKOU_MAX] = {};
int dakou_ptr = 0;




//====================================================
//  �G���Ȃ��܂��Ȃ�
//====================================================
#pragma CREG	_flg_	flg
unsigned int	_flg_;


//====================================================
//  BEEP����p
//====================================================
int beep_time = 0;
#define BEEP_PERIOD()			do{ if( beep_time > 0 ){ beep_time--; }else{ beep( OFF ); } }while(0)
#define BEEP(note, time)		do{ beep(note); beep_time = time; }while(0)
//#define BEEP(note, time)		do{}while(0)


/* ���������g����� */
#define	BEEP_TARGET		(0)
#define	BEEP_INTEG		(0)
#define	BEEP_LEAN_SCENARIO	(0)

/* �P�� */
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
//##  PID����
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
//##  �Z���T�l�̉��H�Ȃ�
//############################################################
void   timer_main( void )
{
    static int dist_p = 0;
    static int intg   = 0;
    int dist;
	int i, j, ll;


	sensor_val = sensor_check();		// ���C���Z���T�[����ŐV�����擾

	// �|�W�V�����l�A�b�v�f�[�g
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
			// ���p�ŁA���̍�������i������ƍ��ɋȂ���Ȃ���A
			// ���������ɂȂ��āA����ɍ��ɋȂ���̂𖳗����␳
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


    // ���������������A�ϕ��l�𑗐M����
    if( cont_flag > 0 ){
        timer_intg = intg;
        intg = 0;
    }

    // ��Ԑϕ��l
    dist   = DISTANCE(pos_log[0]);
    intg  += (( dist + dist_p )>>1);
	
    dist_p = dist;

	if( dist > THdakou ){
		right_side++;
	}else if( dist < -THdakou ){
		left_side++;
	}


	// ���[���`�F���W�̏I�������o
	if( (lean_change>0) && (sensor_val != SENSOR_AW) ){
		sign2 = 0;
		lean_change = 0;
	}

	// ���[���`�F���W���͖���
	if( (sign2==0) && (lean_change==0) ){

		// L3,R3��������ً}����
		if( (pos_log[0]==3) || (pos_log[0]==-3) ){
			Detect_edge = 1;
			Target_Speed = SPEED_LO;
		}
		// L3,R3���O�������珙�s
		if( (pos_log[0]>3) || (pos_log[0]<-3) ){
			Detect_edge = 1;
			Target_Speed = SPEED_SLOW;
		}
	}
	
	// �����C�����ۂ����̂��`�F�b�N
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
//##  ����ʎZ�o
//############################################################
void    control_main( void )
{
	int i, j;
	int	Kpow;
	int max;

	//====================================================
	//  �c�����̐���	
	//====================================================
	// �ߋ��̏o��Power����[���I�ȑ��x���Z�o
	// Target_Speed�w����0-1000�̒l�ŗ^����̂ŁA
	// �Z�oSpeed���ő�l��1000�ɂȂ�悤�ɕ␳����
	
	Speed = 0;
	for( i=0; i<O_C_RATIO; i++ ){
		Speed = Speed + power_log[i];
	}
#define	SPEED_RATIO		((100*O_C_RATIO)/1000)
#if SPEED_RATIO != 1
	Speed = Speed / SPEED_RATIO;
#endif
	Speed = SATURATE( Speed, 1000, 0 );

	// �����Speed����l
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
	//  �������̐���
	//====================================================
	Dist = DISTANCE(pos_log[0]);
	Pr = Dist;

	// �ڋߑ��x�ł̐���i�����j
	Di = Dist - Dist_p;

	// �J�[�u�Ǐ]�i�ϕ��j
	for( i=O_C_RATIO-1; i>0; i-- ){
		intg_log[i] = intg_log[i-1];
	}
	intg_log[0] = timer_intg / C_T_RATIO;	// �l���傫���Ȃ肷����̂ŁA�X�P�[�����O

	

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


	// ���[���`�F���W	
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
	//  ���E�̃o�����X
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
	//  �ԑ��̒���
	//====================================================
	Power = Power + dv;
	Power = SATURATE( Power, POWER_MAX, POWER_MIN );
	vl = ( Power * vl ) / 100;
	vr = ( Power * vr ) / 100;

	// �u���[�L�w��
	if( brake>0 ){
		vl = 0;
		vr = 0;
	}

	// �p���[�I�t�w��
	if( power_off>0 ){
		vl = 0;
		vr = 0;
	}

	//====================================================
    //  ���[�^�o��
	//====================================================
	vl = SATURATE( vl, 100, 0 );
	vr = SATURATE( vr, 100, 0 );
	motor( vl, vr );

	//====================================================
    //  ���̎����ւ̃f�[�^�ۑ�
	//====================================================
    Dist_p  = Dist;
	Speed_p = Speed;

	if( brake > 0 ){
		brake--;
	}

	// Power�̗ݐ�
	for( i=O_C_RATIO-1; i>0; i-- ){
		power_log[i] = power_log[i-1];
	}
	power_log[0] = Power;

	return;
}


//############################################################
//##  �w�ߒl�Z�o
//############################################################
void    operation_main( void )
{
	int	i, j;
	int detect_sign = 0;

	//====================================================
	//  �X�e�[�g�n�r�[�v
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
	//  �T�C�����o
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
	//  �֍s���o�p���O
	//====================================================
	dakou_left_log[dakou_ptr]  = left_side;
	dakou_right_log[dakou_ptr] = right_side;
	dakou_ptr ++;
	dakou_ptr = dakou_ptr % DAKOU_MAX;
	left_side  = 0;
	right_side = 0;

	//====================================================
	//  ���s�X�e�[�g
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
	// State�ω��ɔ�������
	//====================================================

	// �N�����N���[�h �J�n
	if( (state_p!=STATE_SIGN1)&&(state==STATE_SIGN1) ){
		sign1    = SIGN1_TIME;
		hispeed  = 0;
#if BEEP_SIGN1_START > 0
	BEEP( BEEP_SIGN1_START, 50 );
#endif
	}

	// �N�����N���[�h �I��
	if( (state_p==STATE_SIGN1)&&(state!=STATE_SIGN1) ){
#if BEEP_SIGN1_END > 0
	BEEP( BEEP_SIGN1_END, 50 );
#endif
	}

	// ���[���`�F���W���[�h �J�n
	if( (state_p!=STATE_SIGN2)&&(state==STATE_SIGN2) ){
		sign2       = SIGN2_TIME;
		lean_change = -1;
		hispeed     = 0;
#if BEEP_SIGN2_START > 0
	BEEP( BEEP_SIGN2_START, 50 );
#endif
	}

	// ���[���`�F���W���[�h �I��
	if( (state_p==STATE_SIGN2)&&(state!=STATE_SIGN2) ){
		lean_change = 0;
#if BEEP_SIGN2_END > 0
	BEEP( BEEP_SIGN2_END, 50 );
#endif
	}

	// �֍s���o���[�h �J�n
	if( (state_p!=STATE_DAKOU)&&(state==STATE_DAKOU) ){
		hispeed  = 0;
#if BEEP_DAKOU_START > 0
	BEEP( BEEP_DAKOU_START, 50 );
#endif
	}

	// ���C���G�b�W���o���[�h �J�n
	if( (state_p!=STATE_EDGE)&&(state==STATE_EDGE) ){
		hispeed  = 0;
#if BEEP_EDGE_START > 0
	BEEP( BEEP_EDGE_START, 50 );
#endif
	}

	// �n�C�X�s�[�h���[�h �J�n
	if( (state_p!=STATE_HISPEED)&&(state==STATE_HISPEED) ){
		hispeed = HISPEED_TIME;
		cooldown = COOLDOWN_TIME + HISPEED_TIME;
#if BEEP_HISPEED_START > 0
	BEEP( BEEP_HISPEED_START, 50 );
#endif
	}

	// �n�C�X�s�[�h���[�h �I��
	if( (state_p==STATE_HISPEED)&&(state!=STATE_HISPEED) ){
#if BEEP_HISPEED_END > 0
	BEEP( BEEP_HISPEED_END, 50 );
#endif
	}


	//====================================================
	// ���x�w�� ����
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
    //  ���̎����ւ̃f�[�^�ۑ��^�X�V
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
//##  �ԗ����� �i�֍s���炢�����Ȃ��j
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
//##  ���C���֐�
//############################################################
void main(void)
{
	int i;
	
	// ������
	DI();
	if( (_flg_ & 0x02) == 0 ){
		clock_init();
	}
	peri_init();
	EI();

#if 1
	while( pushsw() == 0 );	// �X�^�[�g�{�^�������҂�
    timer( 0, 100 );
	while( pushsw() != 0 );	// �X�^�[�g�{�^���J���҂�
#endif

	// ���C�����[�v
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
		
		// �r�[�v�o�͂̒�~
		BEEP_PERIOD();
	}

}


//------------------------------------------------------------------------------
// End of file
//------------------------------------------------------------------------------
