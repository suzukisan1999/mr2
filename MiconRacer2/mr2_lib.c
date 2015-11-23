
#include "mr2_lib.h"

//------------------------------------------------------------------------------
// �O���[�o���ϐ�
//------------------------------------------------------------------------------


unsigned long	timer_count = 0;       // timer�֐��p
volatile char	line_data = 0;	// �ŐV���C���p�^�[��

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
void motor( int data1, int data2 )
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
// R8C/34C�̓������Ӌ@�\�̏�����
//------------------------------------------------------------------------------
void clock_init( void )
{
        volatile unsigned char i = 0;

        // �����I���`�b�v�I�V���[�^�̐ݒ�
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
void peri_init( void )
{
		
        // I/O�|�[�g�̓��o�͐ݒ�
        prc2 = 1;                       // pd0���W�X�^�ւ̏������݋���
        pd0 = 0xe0;                     // P0_0�`P0_3:�Z���T
										// P0_4(AN3) ���[�^���[�G���R�[�_
                                        // P0_5�`P0_7:LED
        prc2 = 0;                       // pd0���W�X�^�ւ̏������݋֎~

        pd1 = pd1 | 0xf;                     // P1_0�`P1_3:LED
//        pd1 = 0xdf;                     // P1_0�`P1_3:LED
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
        trdpsr1 = 0x01;                 // TRDIOA1�[�q��P2_4�Ɋ��蓖��
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
		
}

//------------------------------------------------------------------------------
// ���ԉ҂�
// ����         �^�C�}�l 1=1ms
// �߂�l       �Ȃ�
//------------------------------------------------------------------------------
void timer( unsigned long data1 )
{
        timer_count = 0;
        while( timer_count < data1 );
}

//------------------------------------------------------------------------------
// ����炷
// ����         (1/���̎��g��)/(1/(�N���b�N���g��/8))-1
;// �߂�l       �Ȃ�
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

