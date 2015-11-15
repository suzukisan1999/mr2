
#include "mr2_lib.h"

//------------------------------------------------------------------------------
// 割り込み
//------------------------------------------------------------------------------

volatile static unsigned short ad_result;

// encoder
// priority : 1 (Low)
#pragma interrupt intAN0 (vect=14)
void intAN0( void )
{
	static int i;
	static unsigned short ad_min = 0x03ff;	// A/D変換ピーク値 (最大値)
	static unsigned short ad_max = 0;		// A/D変換ピーク値 (最小値)
	static unsigned short ad_work = 0;		// A/D変換結果 平均値計算用変数
	unsigned short ad_data;					// A/D変換結果 現在値
	
	if(i == 0){								// １回目のデータ取得時
		ad_work = ad_max = 0;				// 変数(再)初期化
		ad_min = 0x3ff;						// 変数(再)初期化
	}

	ad_data = ad3 & 0x03ff;					// A/D変換結果 取得

	ad_work += ad_data;						// 平均値計算用変数への加算
	if(ad_data > ad_max){ ad_max = ad_data; }	// 最大値更新
	if(ad_data < ad_min){ ad_min = ad_data;	}	// 最小値更新
	i++;
	if(i >= 4){
		ad_result = (ad_work - ad_max - ad_min) >> 1;  // (4回の加算結果 - 最大値 - 最小値)/2
		// ここまでで「ピーク値除去済み平均値(2)」が得られる
	}

	i &= 3;		// カウンタ i ループ用処理  ( 0→1→2→3→0→1→… ) 

}



// line censor
// cyclic handler
// priority : 2 (middle)
#pragma interrupt intTRAIC (vect=22)
void intTRAIC( void )
{
	int i;
	unsigned char pos[POS_MAX];
	
        p0_7 = ~p0_7;

        if( p0_7 == 0 ){
#if CENTER_LINE
            //p0_1、p0_3のモニタが可能
            p0_5 = ~p0_1;
            p0_6 = ~p0_3;
#else
            p0_5 = p0_1;
            p0_6 = p0_3;
#endif
        }else{
#if CENTER_LINE
            //p0_0、p0_2のモニタが可能
            p0_5 = p0_0;
            p0_6 = p0_2;
#else
            p0_5 = ~p0_0;
            p0_6 = ~p0_2;
#endif
        }
		mr2_line_data = mr2_sensor();
        mr2_timer_count++;

		// position log
		mr2_pos_push( mr2_line_data & 0x0f );
		
		// 姿勢推定
		if( mr2_pos(0) > 0x0f ){
			mr2_beep( Def_C3 );
		}
			
#if 0		
		mr2_pos_all( &pos[0] );
		for( i=0; i<POS_MAX; i++ ){
			if( pos[i] > 16 ){
				mr2_beep( Def_C3 );
			}
		}
#endif
}






