/*
 * @Author: wzdsch 1919524828@qq.com
 * @Date: 2025-09-02 22:18:23
 * @LastEditors: wzdsch 1919524828@qq.com
 * @LastEditTime: 2025-09-06 15:41:54
 * @FilePath: /leg/Components/inc/bsp_can.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef BSP_CAN_HPP
#define BSP_CAN_HPP

#include "can.h"
#include "stm32f4xx_hal_can.h"
#include <cstdint>
#include <vector>

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
enum bsp_can_status_e {
    BSP_CAN_ERROR = 0,
    BSP_CAN_OK
};


class bsp_can_tx_instance {
private:
    bsp_can_tx_mode_e m_mode; // 默认关闭
    CAN_HandleTypeDef* m_hcan;
    CAN_TxHeaderTypeDef m_tx_header;
    const uint8_t* m_ptxd;
    uint32_t m_mailbox;
public:
    // constructors
    bsp_can_tx_instance(CAN_HandleTypeDef* const hcan, const uint32_t id, const uint32_t ide, const uint32_t rtr, \
                        const uint32_t dlc);
    bsp_can_tx_instance(CAN_HandleTypeDef* const hcan, const uint32_t id, const uint32_t ide, const uint32_t rtr, \
                        const uint32_t dlc, const uint8_t* const ptxd);

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
///        注意：(1) prxd空间 >= 8
///             (2) prxd失效时必须 set_prxd(nullptr)
///     2. 调用get_prxd方法，获取接收到的数据(m_arxd变量)
class bsp_can_rx_instance {
private:
    bsp_can_rx_mode_e m_mode; // 默认关闭
    CAN_HandleTypeDef* m_hcan;
    CAN_RxHeaderTypeDef m_rx_header;
    uint32_t m_id;
    uint32_t m_ide;
    uint8_t* m_prxd; // ptr rxd(外部缓存)
    uint8_t m_arxd[8]; // array rxd(内部缓存)
    uint32_t m_rxfifo; // 接收要读取的FIFO (CAN_RX_FIFO0/CAN_RX_FIFO1)
    CAN_FilterTypeDef m_filter;

    static uint8_t sm_can1_filter_index;
    static uint8_t sm_can2_filter_index;

    // 接收实例地址映射表，用于自动在接收回调中接收各实例的数据
    static std::vector<bsp_can_rx_instance*> spm_rx_instances;
public:
    // constructors
    bsp_can_rx_instance(CAN_HandleTypeDef* const hcan, const uint32_t id, const uint32_t ide);
    bsp_can_rx_instance(CAN_HandleTypeDef* const hcan, const uint32_t id, const uint32_t ide, uint8_t* const prxd);

    // destructors
    ~bsp_can_rx_instance();

    // setters
    void disable();
    void enable();
    void set_prxd(uint8_t* const prxd); // 设置外部缓存地址 注意：地址失效时需将此参数设为nullptr，否则可能造成非法写内存！

    // getters
    bsp_can_rx_mode_e get_mode() const;
    uint32_t get_id() const;
    uint32_t get_ide() const;
    uint32_t get_dlc() const;
    uint32_t get_fifo() const;
    const uint8_t* const get_arxd() const; // 获取内部缓存

    // methods
    static void bsp_can_get_msg_to_instances(const CAN_HandleTypeDef* const hcan, const uint32_t fifo, \
                                             const CAN_RxHeaderTypeDef * const header, const uint8_t* const data);
};

status_e bsp_can_init_all();

#endif
