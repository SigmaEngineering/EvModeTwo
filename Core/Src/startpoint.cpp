#include "startpoint.h"


void start()
{
	//static UartClass g(&huart2);
	//g.sendUart("Ahmed");
    //HAL_Delay(1000);

    //---------------------------MainCode_Start----------------------------------------------------
    static MgmtLink link(&huart2);

    link.write(NULL);
    HAL_Delay(2000);
}

