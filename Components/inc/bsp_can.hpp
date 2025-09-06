/*
 * @Author: wzdsch 1919524828@qq.com
 * @Date: 2025-09-02 22:18:23
 * @LastEditors: wzdsch 1919524828@qq.com
 * @LastEditTime: 2025-09-06 11:26:14
 * @FilePath: /leg/Components/inc/bsp_can.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef BSP_CAN_HPP
#define BSP_CAN_HPP

#include "can.h"
#include "stm32f4xx_hal_can.h"
#include <cstdint>

#define BSP_CAN2_FILTER_START 14

enum bsp_can_tx_mode_e {
    BSP_CAN_TX_DISABLE = 0,
    BSP_CAN_TX_ENABLE,
    BSP_CAN_TX_ERROR
};
enum bsp_can_rx_mode_e {
    BSP_CAN_RX_DISABLE = 0,
    BSP_CAN_RX_ENABLE,
    BSP_CAN_RX_ERROR
};
enum status_e {
    MY_ERROR = 0,
    MY_OK
};


class bsp_can_tx_instance {
private:
    bsp_can_tx_mode_e m_mode; // 默认关闭
    CAN_HandleTypeDef* m_hcan;
    CAN_TxHeaderTypeDef m_tx_header;
    const uint8_t* m_ptxd;
    uint32_t m_mailbox;
public:
    bsp_can_tx_instance(CAN_HandleTypeDef* hcan, uint32_t id, uint32_t ide, uint32_t rtr, uint32_t dlc);
    bsp_can_tx_instance(CAN_HandleTypeDef* hcan, uint32_t id, uint32_t ide, uint32_t rtr, uint32_t dlc, const uint8_t* const ptxd);

    // setters
    void disable();
    void enable();
    void set_ptxd(const uint8_t* const ptxd);
    void set_dlc(uint32_t dlc);

    // getters
    CAN_HandleTypeDef* get_can_handle() const;
    uint32_t get_id() const;
    uint32_t get_ide() const;
    uint32_t get_dlc() const;
    uint32_t get_mailbox() const;
    bsp_can_tx_mode_e get_mode() const;

    // methods
    status_e transmit();
};


/// can receive instace
/// 接收数据方法:
///     1. 使用构造函数的prxd参数，或调用set_prxd方法，将接收到的数据存储到该地址(m_prxd变量)
///     2. 调用get_prxd方法，获取接收到的数据(m_arxd变量)
class bsp_can_rx_instance {
private:
    bsp_can_rx_mode_e m_mode; // 默认关闭
    CAN_HandleTypeDef* m_hcan;
    CAN_RxHeaderTypeDef m_rx_header;
    uint8_t* m_prxd; // ptr rxd
    uint8_t m_arxd[8]; // array rxd
    uint32_t m_fifo;
    CAN_FilterTypeDef m_filter;
    static uint8_t sm_can1_filter_index;
    static uint8_t sm_can2_filter_index;
public:
    bsp_can_rx_instance(CAN_HandleTypeDef* hcan, uint32_t id, uint32_t ide);
    bsp_can_rx_instance(CAN_HandleTypeDef* hcan, uint32_t id, uint32_t ide, uint8_t* prxd);

    // setters
    void disable();
    void enable();
    void set_prxd(uint8_t* const prxd);

    // getters
    bsp_can_rx_mode_e get_mode() const;
    uint32_t get_id() const;
    uint32_t get_ide() const;
    uint32_t get_dlc() const;
    uint32_t get_fifo() const;
    const uint8_t* const get_prxd() const;
};

status_e bsp_can_init_all();

#endif
