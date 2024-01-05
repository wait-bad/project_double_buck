#ifndef __batter_h__
#define __batter_h__

#include "stm32f10x.h"
#include "12864.h"
void batter_show_channel();
void batter_charge_show_state(uint8_t flag);
void batter_charge_show_state_null();

void batter_show_channel()
{
    uint8_t o;
    for ( o = 0; o < 4; o++)
    {
        lcd12864_show_n(0,0,22+o*25,1+o);
    }
    for ( o = 0; o < 4; o++)
    {
        lcd12864_show_n(0,3,22+o*25,5+o);
    }   
}

/**
 * @brief 状态显示
 * 
 * @param flag 使能显示 从高位起 代表八个通道
 */
void batter_charge_show_state(uint8_t flag)
{
    uint8_t i;
    for ( i = 0; i < 4; i++)
    {
        lcd12864_show_square(0,2,22+i*25,flag&0x80>>i);

    }

    for ( i = 0; i < 4; i++)
    {

        lcd12864_show_square(0,5,22+i*25,flag&0x08>>i);
    }
    
    
}
void batter_charge_show_state_null()
{
    uint8_t i;
    for ( i = 0; i < 4; i++)
    {
        lcd12864_show_square(0,2,22+i*25,0);

    }

    for ( i = 0; i < 4; i++)
    {

        lcd12864_show_square(0,5,22+i*25,0);
    }
    
    
}



#endif // !__batter_h__
