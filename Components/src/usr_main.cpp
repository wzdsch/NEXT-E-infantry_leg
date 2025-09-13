/*
 * @Author: wzdsch 1919524828@qq.com
 * @Date: 2025-09-12 15:00:02
 * @LastEditors: wzdsch 1919524828@qq.com
 * @LastEditTime: 2025-09-13 10:43:38
 * @FilePath: /leg/Components/src/usr_main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "main.h"
#include "usr_main.hpp"
#include "bsp_can.hpp"

bsp_can_rx_instance bsp_can_rx(&hcan1, 0x201);

void main_configs() {
    bsp_can_init_all();
    bsp_can_rx.enable();
}
void main_loop()  {
    
}

