
#include "mr2_lib.h"

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



// line sensor
// cyclic handler
// priority : 2 (middle)
#pragma interrupt intTRAIC (vect=22)
void intTRAIC( void )
{
		int i;
        static unsigned int count = 0;

#if 0
		static beep_count=0;
		if( line_data & 0x80){
			beep(Def_C4);
			beep_count = 100;
		}
		if( --beep_count <= 0 ){
			beep( OFF );
		}
#endif
		
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

		for( i=0; i<TIMER_NUM; i++ ){
	        timer_count[i]++;
		}

        if( (count % TIMER_PERIOD) == 0 ){
            timer_flag++;
        }
        if( (count % CONTROL_PERIOD) == 0 ){
            cont_flag++;
        }
        if( (count % OPERATION_PERIOD) == 0 ){
            opr_flag++;
        }
        if( (count % VEHICLE_PERIOD) == 0 ){
            veh_flag++;
        }
        
        count++;
}






