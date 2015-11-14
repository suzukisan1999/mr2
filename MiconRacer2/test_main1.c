//------------------------------------------------------------------------------
// �Ώۃ}�C�R�� R8C/34C
// ̧�ٓ��e     Micon Racer Advance
// �o�[�W����   Ver.1.00
// Date         2014.12.19
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// �C���N���[�h
//------------------------------------------------------------------------------
#include <stdlib.h>
#include "sfr_r834c.h"

#define	motor	motor_fix

//------------------------------------------------------------------------------
// �V���{����`
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

#define Def_C3          19083           // �h:(1/131)/(1/(20000000/8))-1
#define Def_D3          17006           // ��:(1/147)/(1/(20000000/8))-1
#define Def_E3          15151           // �~:(1/165)/(1/(20000000/8))-1
#define Def_F3          14285           // �t�@:(1/175)/(1/(20000000/8))-1
#define Def_G3          12754           // �\:(1/196)/(1/(20000000/8))-1
#define Def_A3          11362           // ��:(1/220)/(1/(20000000/8))-1
#define Def_B3          10120           // �V:(1/247)/(1/(20000000/8))-1
#define Def_C4          9541            // �h:(1/262)/(1/(20000000/8))-1

#define DI()            asm("FCLR I")   // ���荞�݋֎~
#define EI()            asm("FSET I")   // ���荞�݋���

#define WHITE           (1)
#define BLACK           (0)

//------------------------------------------------------------------------------
// �I�v�V�����ݒ�
//------------------------------------------------------------------------------

#define CENTER_LINE     BLACK						// �������C���̐F�̐ݒ�
//#define CENTER_LINE     WHITE						// �������C���̐F�̐ݒ�

#define MOTOR_LIMIT		(50)						// ���[�^�ő�o�� : 60%
#define OPTION_L        (MOTOR_LIMIT)				// �ő��]���ݒ�
#define OPTION_R        (MOTOR_LIMIT)

// LED�Z���T�[��`  ���F����   ���F�_��
#define SENSOR_LR0		(       0x04 | 0x02       )	// ����   : ��������
#define SENSOR_L1		(       0x04              )	// ��(��) : ��������
#define SENSOR_R1		(              0x02       )	// �E(��) : ��������
#define SENSOR_L2		(0x08 | 0x04              )	// ��(��) : ��������
#define SENSOR_R2		(              0x02 | 0x01)	// �E(��) : ��������
#define SENSOR_L3		(0x08                     )	// ��(��) : ��������
#define SENSOR_R3		(                     0x01)	// �E(��) : ��������
#define SENSOR_AB		(0x08 | 0x04 | 0x02 | 0x01)	// �S�_�� : �������� (AB:All Black)
#define SENSOR_AW		(0x00)						// �S���� : �������� (AW:All White)

//------------------------------------------------------------------------------
// �֐��v���g�^�C�v�̐錾
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
// �O���[�o���ϐ��̐錾
//------------------------------------------------------------------------------
unsigned long           cnt0 = 0;       // timer�֐��p
unsigned long           cnt1 = 0;       // main���Ŏg�p
unsigned long           cnt2 = 0;       // ���R
unsigned long           cnt3 = 0;       // ���R
int                     pattern = 0;    // �p�^�[���ԍ�
volatile static int		deg = 0;		// �ڕW�p�x
volatile static unsigned char	line_data = 0;	// �ŐV���C���p�^�[��

//------------------------------------------------------------------------------
// ���C���v���O����
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
	//���Ƃ����ƌv�Z����

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
	
    init();		// ���Ӌ@�\������

#if 1
	while( pushsw() == 0 );	// �X�^�[�g�{�^�������҂�
    while(1){		
		
		read_line = sensor_check();		// ���C���Z���T�[����ŐV�����擾
		switch(read_line){				// ���C���Z���T�[����̍ŐV���ɉ����������̕���
			case SENSOR_AW:				// ���C���F�Ȃ� (�R�[�X�A�E�g��)
				car_ctrl( 0, 0 );
				break;
			case SENSOR_LR0:			// ���C���F���� (�ԑ�:����)
				car_ctrl( 0, 100 );
				break;
			case SENSOR_L1:				// ���C���F������ (�ԑ�:���C����菭���E�Ɉʒu)
				car_ctrl( -10, 100 );
				break;
			case SENSOR_R1:				// ���C���F�����E (�ԑ�:������菭�����Ɉʒu)
				car_ctrl(  10, 100 );
				break;
			case SENSOR_L2:				// ���C���F�� (�ԑ�:���C�����E�Ɉʒu)
				car_ctrl( -30, 100 );
				break;
			case SENSOR_R2:				// ���C���F�E (�ԑ�:���C����荶�Ɉʒu)
				car_ctrl(  30, 100 );
				break;
			case SENSOR_L3:				// ���C���F�傫���� (�ԑ�:���C�����傫���E�Ɉʒu)
				car_ctrl( -60, 60 );
				break;
			case SENSOR_R3:				// ���C���F�傫���E (�ԑ�:���C�����傫�����Ɉʒu)
				car_ctrl(  60, 60 );
				break;
			default:					// LED�Z���T�[��`�ɑ��݂��Ȃ����͐M��
				break;					// �Ȃɂ����Ȃ�
			}
	}
#else
	while( pushsw() == 0 );	// �X�^�[�g�{�^�������҂�
    while(1){		
		
		read_line = sensor_check();		// ���C���Z���T�[����ŐV�����擾
		switch(read_line){				// ���C���Z���T�[����̍ŐV���ɉ����������̕���
			case SENSOR_AW:				// ���C���F�Ȃ� (�R�[�X�A�E�g��)
				motor(0,0);				// ���[�^��~
				break;
			case SENSOR_LR0:			// ���C���F���� (�ԑ�:����)
				set_servo(0);			// �ڕW�p�x�F�}0
				motor(100, 100);		// ��֋쓮���[�^�F��100% | �E100% (���i)
				break;
			case SENSOR_L1:				// ���C���F������ (�ԑ�:���C����菭���E�Ɉʒu)
				set_servo(-10);			// �ڕW�p�x�F-10��(���ɏ����Ȃ���)
				motor(100, 100);		// ��֋쓮���[�^�F��100% | �E100% (���i)
				break;
			case SENSOR_R1:				// ���C���F�����E (�ԑ�:������菭�����Ɉʒu)
				set_servo(10);			// �ڕW�p�x�F 10��(�E�ɏ����Ȃ���)
				motor(100, 100);		// ��֋쓮���[�^�F��100% | �E100% (���i)
				break;
			case SENSOR_L2:				// ���C���F�� (�ԑ�:���C�����E�Ɉʒu)
				set_servo(-25);			// �ڕW�p�x�F-25��(���ɒ����炢�Ȃ���)
				motor( 60,  100);		// ��֋쓮���[�^�F�� 60% | �E100% (���ɏ����Ȃ���)
				break;
			case SENSOR_R2:				// ���C���F�E (�ԑ�:���C����荶�Ɉʒu)
				set_servo(25);			// �ڕW�p�x�F 25��(�E�ɒ����炢�Ȃ���)
				motor( 100,  60);		// ��֋쓮���[�^�F��100% | �E 60% (�E�ɏ����Ȃ���)
				break;
			case SENSOR_L3:				// ���C���F�傫���� (�ԑ�:���C�����傫���E�Ɉʒu)
				set_servo(-40);			// �ڕW�p�x�F-40��(���ɑ傫���Ȃ���)
				motor( 33,  100);		// ��֋쓮���[�^�F�� 33% | �E100% (���ɑ傫���Ȃ���)
				while( sensor_check() != SENSOR_L2);	// �Z���^�[���C������O��Ă��A���A����܂Ō���ێ����쓮���[�^�͒�~���Ȃ�
				break;
			case SENSOR_R3:				// ���C���F�傫���E (�ԑ�:���C�����傫�����Ɉʒu)
				set_servo(40);			// �ڕW�p�x�F 40��(�E�ɑ傫���Ȃ���)
				motor( 100,  33);		// ��֋쓮���[�^�F���E100% | �E 33% (�E�ɑ傫���Ȃ���)
				while( sensor_check() != SENSOR_R2);	// �Z���^�[���C������O��Ă��A���A����܂Ō���ێ����쓮���[�^�͒�~���Ȃ�
				break;
			default:					// LED�Z���T�[��`�ɑ��݂��Ȃ����͐M��
				break;					// �Ȃɂ����Ȃ�
			}
	}
#endif

}

//------------------------------------------------------------------------------
// R8C/34C�̓������Ӌ@�\�̏�����
//------------------------------------------------------------------------------
void init( void )
{
        volatile unsigned char i = 0;

        // ���荞�݋֎~
        DI();

        // �����I���`�b�v�I�V���[�^�̐ݒ�
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

        // I/O�|�[�g�̓��o�͐ݒ�
        prc2 = 1;                       // pd0���W�X�^�ւ̏������݋���
        pd0 = 0xe0;                     // P0_0�`P0_3:�Z���T
										// P0_4(AN3) ���[�^���[�G���R�[�_
                                        // P0_5�`P0_7:LED
        prc2 = 0;                       // pd0���W�X�^�ւ̏������݋֎~

        pd1 = 0xdf;                     // P1_0�`P1_3:LED
                                        // P1_4:TXD0
                                        // P1_5:RXD0
										// P1_7:Servo 1:Active
		p1 = 0x00;
        pu04 = 1;
        pd2 = 0xfe;                     // P2_0:�X�C�b�`
                                        // P2_1:AIN1
                                        // P2_2:PWMA
                                        // P2_3:BIN1
                                        // P2_4:PWMB
                                        // P2_5:
                                        // P2_6:AIN2
                                        // P2_7:

        pd3 = 0xff;                     // P3_4:�u�U�[

        pd4 = 0xfb;                     // P4_2:VREF
                                        // P4_5:BIN2

        pd6 = 0xff;                     // none

        mstcr = 0x00;                   // ���W���[���X�g�b�v����

        // �^�C�}RA��1ms���荞�ݐݒ�
        tramr = 0x00;                   // �J�E���g�\�[�X��f1
        trapre = 128 - 1;               // �v���X�P�[��
        tra = TIMER_CYCLE;              // �v���C�}���J�E���^
        traic = 0x01;                   // �^�C�}RA�̊��荞�݃��x���ݒ�
        tracr = 0x01;                   // �J�E���g���J�n

        // �^�C�}RB��1ms���荞�ݐݒ�
        trbmr = 0x00;                   // �J�E���g�\�[�X��f1
        trbpre = 128 - 1;               // �v���X�P�[��
        trbpr = TIMER_CYCLE;            // �v���C�}���J�E���^
        trbic = 0x01;                   // �^�C�}RB�̊��荞�݃��x���ݒ�
        trbcr = 0x01;                   // �J�E���g���J�n

        // �^�C�}RC��PWM���[�h
        trccr1 = 0xb0;                  // �J�E���g�\�[�X��f8
        trcgra = 0;                     // ���d�T�E���_�̎���
        trcgrc = 0;                     // ���d�T�E���_�̃f���[�e�B��
        trccr2 = 0x02;                  // TRCIOC�[�q�̓A�N�e�B�u���x��H
        trcoer = 0x0b;                  // TRCIOC�[�q�̏o�͋���
        trcpsr1 = 0x02;                 // TRCIOC�[�q��P3_4�Ɋ��蓖��
        trcmr = 0x8a;                   // �J�E���g���J�n

        // �^�C�}RD�̃��Z�b�g����PWM���[�h
        trdpsr0 = 0x08;                 // TRDIOB0�[�q��P2_2�Ɋ��蓖��
#if 0
        trdpsr1 = 0x45;                 // TRDIOB1�[�q��P2_5�Ɋ��蓖�āiSERVO_IN2�j
                                        // TRDIOA1�[�q��P2_4�Ɋ��蓖��
										// TRDIOD1�[�q��P2_7�Ɋ��蓖�āiSERVO_IN1�j
#else
        trdpsr1 = 0x01;                 // TRDIOA1�[�q��P2_4�Ɋ��蓖��
#endif
        trdmr = 0xf0;                   // ���W�X�^���o�b�t�@����ɂ���
        trdfcr = 0x01;                  // ���Z�b�g����PWM���[�h�ɐݒ�
        trdoer1 = 0x4d;                 // TRDIOB1�̏o�͋���
                                        // TRDIOA1�̏o�͋���
                                        // TRDIOB0�̏o�͋���
										// TRDIOD1�̏o�͋���
        trdcr0 = 0x20;                  // �J�E���g�\�[�X��f1
        trdgra0 = trdgrc0 = PWM_CYCLE;  // ����
        trdgrb0 = trdgrd0 = 0;          // TRDIOB0�[�q�i�����[�^�j
        trdgra1 = trdgrc1 = 0;          // TRDIOA1�[�q�i�E���[�^�j
        trdgrb1 = trdgrd1 = PWM_CYCLE/2;          // TRDIOB1�[�q�i�T�[�{�j
        trdstr = 0x0d;                  // �J�E���g���J�n

		admod   = 0x33;					// �J��Ԃ��|�����[�h
		adinsel = 0x30;                 // 8�[�q���g�p
		adcon1  = 0x30;                 // 10�r�b�g���[�h�AAD����\
		adic 	= 0x01;					// A/D���荞�� �D�惌�x���F1
		adst    = 1;                    // A/D�ϊ��X�^�[�g
		
        // ���荞�݋���
        EI();
}




//------------------------------------------------------------------------------
// ���荞��
//------------------------------------------------------------------------------

volatile static unsigned short ad_result;

// encoder
// priority : 1 (Low)
#pragma interrupt intAN0 (vect=14)
void intAN0( void )
{
	static int i;
	static unsigned short ad_min = 0x03ff;	// A/D�ϊ��s�[�N�l (�ő�l)
	static unsigned short ad_max = 0;		// A/D�ϊ��s�[�N�l (�ŏ��l)
	static unsigned short ad_work = 0;		// A/D�ϊ����� ���ϒl�v�Z�p�ϐ�
	unsigned short ad_data;					// A/D�ϊ����� ���ݒl
	
	if(i == 0){								// �P��ڂ̃f�[�^�擾��
		ad_work = ad_max = 0;				// �ϐ�(��)������
		ad_min = 0x3ff;						// �ϐ�(��)������
	}

	ad_data = ad3 & 0x03ff;					// A/D�ϊ����� �擾

	ad_work += ad_data;						// ���ϒl�v�Z�p�ϐ��ւ̉��Z
	if(ad_data > ad_max){ ad_max = ad_data; }	// �ő�l�X�V
	if(ad_data < ad_min){ ad_min = ad_data;	}	// �ŏ��l�X�V
	i++;
	if(i >= 4){
		ad_result = (ad_work - ad_max - ad_min) >> 1;  // (4��̉��Z���� - �ő�l - �ŏ��l)/2
		// �����܂łŁu�s�[�N�l�����ςݕ��ϒl(2)�v��������
	}

	i &= 3;		// �J�E���^ i ���[�v�p����  ( 0��1��2��3��0��1���c ) 

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
            //p0_1�Ap0_3�̃��j�^���\
            p0_5 = ~p0_1;
            p0_6 = ~p0_3;
#else
            p0_5 = p0_1;
            p0_6 = p0_3;
#endif
        }else{
#if CENTER_LINE
            //p0_0�Ap0_2�̃��j�^���\
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
	ctrl_servo(deg);	// �T�[�{����֐��Ăяo��
}

//------------------------------------------------------------------------------
// �T�[�{�pA/D�ϊ��֐�
// ����         �Ȃ�
// �߂�l       A/D�ϊ����� ( �s�[�N�l�����ςݕ��ϒl(2) )
//------------------------------------------------------------------------------
unsigned short get_adc(void){
	return ad_result;	// A/D�ϊ����� ( �s�[�N�l�����ςݕ��ϒl(2) ) ��Ԃ�
}

//------------------------------------------------------------------------------
// �Z���T�[��Ԍ��o
// ����         �Ȃ�
// �߂�l       �Z���T�l + ���X�V�t���O
//------------------------------------------------------------------------------
unsigned char sensor( void )
{
        volatile unsigned char  data1;
#if CENTER_LINE
        data1 = ~p0;                   // ���C���̐F�͔�
#else
        data1 = p0;                    // ���C���̐F�͍�
#endif
        data1 = data1 & 0x0f;

        data1 = data1 | 0x80;			// update flag set

        return( data1 );
}

//------------------------------------------------------------------------------
// �ŐV�Z���T�[��Ԍ��o
// ����         �Ȃ�
// �߂�l       �Z���T�l (���X�V�t���O�̓N���A����Ă���)
//------------------------------------------------------------------------------
unsigned char sensor_check( void ){
	unsigned char data;
	while( (line_data & 0x80) == 0x00 );	// update flag check
	line_data &= 0x7f;						// update flag clear
	data = line_data;			
	return data;
}

//------------------------------------------------------------------------------
// ���[�^�[���x����
// ����         �����[�^:-100�`100�A�E���[�^:-100�`100
//              0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%
// �߂�l       �Ȃ�
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
// ���ԉ҂�
// ����         �^�C�}�l 1=1ms
// �߂�l       �Ȃ�
//------------------------------------------------------------------------------
void timer( unsigned long data1 )
{
        cnt0 = 0;
        while( cnt0 < data1 );
}

//------------------------------------------------------------------------------
// ����炷
// ����         (1/���̎��g��)/(1/(�N���b�N���g��/8))-1
// �߂�l       �Ȃ�
//------------------------------------------------------------------------------
void beep( int data1 )
{
        trcgra = data1;                 // �����̐ݒ�
        trcgrc = data1 / 2;             // �f���[�e�B50%�̂��ߎ����̔����̒l
}

//------------------------------------------------------------------------------
// �v�b�V���X�C�b�`��Ԍ��o
// ����         �Ȃ�
// �߂�l       �X�C�b�`��������Ă��Ȃ��ꍇ:0�A�����ꂽ�ꍇ:1
//------------------------------------------------------------------------------
unsigned char pushsw( void )
{
        unsigned char data1;

        data1 = ~p2;
        data1 &= 0x01;

        return( data1 );
}

//------------------------------------------------------------------------------
// �T�[�{�p�x����
// ����         �ڕW�p�x
// �߂�l       �Ȃ�
//------------------------------------------------------------------------------
void ctrl_servo(int degree)
{
#define RIGHT_MAX  		(405)						// +40�x�̂Ƃ���A/D�ϊ�����
#define LEFT_MAX   		(615)						// -40�x�̂Ƃ���A/D�ϊ�����
#define CENTER_VAL 		((LEFT_MAX+RIGHT_MAX)/2)	// �}0�x�̂Ƃ��̒l
#define GAIN       		(10)							// �Q�C��
#define ABS(x)     		(((x)>=0)?(x):-(x))

	static int ad_data = 9999;
	static int ad_data_old;
	int pos = 0;	// ���݂̊p�x(�}40�x)
	int diff = 0;	// �ڕW�̊p�x�ƌ��݂̊p�x�̍�
	int duty = 0;	// duty��w��p
	
	ad_data_old = ad_data;
	ad_data = get_adc();	// �T�[�{�̏o�͒l(�d��)�擾
	pos = -((ad_data - CENTER_VAL) * 40 / ((LEFT_MAX-RIGHT_MAX)/2));	// �T�[�{�̏o�͒l(�d��)���u���݊p�x�v�ɕϊ�
	diff = deg - pos;		// �ڕW�p�x�ƌ��݊p�x�̍������v�Z
	duty = ABS(diff) * GAIN + ((PWM_CYCLE)>>4);		// ���ɉ����ĉ�]���x(duty��)�����߂�
	if(ABS(duty) >= (PWM_CYCLE/2)){
		duty = PWM_CYCLE/2 -1;
	}

	if(diff < 0){						// �ڕW�̊p�x��茻�݂̊p�x���E�̏ꍇ
		trdgrd1 = PWM_CYCLE/2 - duty;	// duty��̕ύX(���ɉ�)
	}
	else{								// �ڕW�̊p�x��茻�݂̊p�x�����̏ꍇ
		trdgrd1 = PWM_CYCLE/2 + duty;	// duty��̕ύX(�E�ɉ�)
	}
	p1_7 = 1;							// Servo ����J�n

	if( (ABS(diff) == 0) && ( ABS(ad_data - ad_data_old)<= 1) ){
		trdgrd1 = PWM_CYCLE/2;
		p1_7 = 0;						// Servo �����~
	}
}


//------------------------------------------------------------------------------
// �T�[�{�p�x�ݒ�
// ����         �ڕW�p�x
// �߂�l       �Ȃ�
//------------------------------------------------------------------------------
void set_servo(int degree){
	deg = degree;		// �ڕW�p�x�̍X�V
}


//------------------------------------------------------------------------------
// End of file
//------------------------------------------------------------------------------
