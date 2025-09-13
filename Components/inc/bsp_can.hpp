/*
 * @Author: wzdsch 1919524828@qq.com
 * @Date: 2025-09-02 22:18:23
 * @LastEditors: wzdsch 1919524828@qq.com
 * @LastEditTime: 2025-09-13 11:36:01
 * @FilePath: /leg/Components/inc/bsp_can.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef BSP_CAN_HPP
#define BSP_CAN_HPP

#include "can.h"
#include "stm32f4xx_hal_can.h"
#include <cstdint>
#include <vector>
#include <functional>

constexpr uint8_t BSP_CAN2_FILTER_START = 14; // 0 ~ 27

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
    BSP_CAN_OK,
    BSP_CAN_BUSY
};


class bsp_can_tx_instance {
private:
    bsp_can_tx_mode_e m_mode; // 默认关闭
    CAN_HandleTypeDef* m_hcan;
    CAN_TxHeaderTypeDef m_tx_header;
    uint32_t m_mailbox;
public:
    // constructors
    bsp_can_tx_instance(CAN_HandleTypeDef* const hcan, const uint32_t id, const uint32_t ide = CAN_ID_STD, const uint32_t rtr = CAN_RTR_DATA, \
                        const uint32_t dlc = 0x08U);

    // setters
    void disable();
    void enable();
    bsp_can_status_e set_dlc(const uint32_t dlc);

    // getters
    CAN_HandleTypeDef* get_can_handle() const;
    uint32_t get_id() const;
    uint32_t get_ide() const;
    uint32_t get_dlc() const;
    uint32_t get_mailbox() const;
    bsp_can_tx_mode_e get_mode() const;

    // methods
    bsp_can_status_e transmit(const uint8_t* const ptxd);
    bsp_can_status_e transmit(const uint8_t* const ptxd, uint32_t dlc); // 调用此方法会覆盖之前设置的dlc!
};


/// can receive instace
/// 接收数据方法:
///     1. 使用构造函数的prxd参数，或调用set_prxd方法，将接收到的数据存储到该地址(m_prxd变量)
///        注意：(1) prxd空间 >= 8
///             (2) prxd失效时必须 set_prxd(nullptr)
///     2. 调用get_prxd方法，获取接收到的数据(m_arxd变量)
class bsp_can_rx_instance {
private:
    // members
    bsp_can_rx_mode_e m_mode; // 默认关闭

    CAN_HandleTypeDef* m_hcan;
    CAN_RxHeaderTypeDef mp_rx_header;

    uint32_t m_id;
    uint32_t m_ide;

    // array rxd(内部缓存) 双缓冲区
    uint8_t ma_rxd1[8];
    uint8_t ma_rxd2[8];
    uint8_t* mp_last_received_data; // 上一次接收完成的数据

    uint32_t m_rxfifo; // 接收要读取的FIFO (CAN_RX_FIFO0/CAN_RX_FIFO1)
    CAN_FilterTypeDef m_filter;

    std::function<void()> m_callback; // 外部回调函数
    
    static uint8_t sm_can1_filter_index;
    static uint8_t sm_can2_filter_index;

    // 接收实例地址映射表，用于自动在接收回调中接收各实例的数据
    static bsp_can_rx_instance* spm_rx_instances[28];

    // methods
    volatile static void bsp_can_get_msg_to_instances(const CAN_HandleTypeDef* const hcan, const uint32_t fifo, \
                                             const CAN_RxHeaderTypeDef * const header, const uint8_t* const data);

    // frend functions
    friend void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
    friend void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
public:
    // constructors
    // 注意: 由于不确定何时实例化, 因此构造函数中不能出现任何hal库对can的配置, 
    //      只能进行一些类内的数据初始化, 防止实例化全局实例在main函数之前运行!
    bsp_can_rx_instance(CAN_HandleTypeDef* const hcan, const uint32_t id, const uint32_t ide = CAN_ID_STD, const std::function<void()> callback = nullptr);

    // destructors
    ~bsp_can_rx_instance();

    // setters
    bsp_can_status_e disable();
    bsp_can_status_e enable();
    void set_callback(const std::function<void()> callback);
    void clear_callback();

    // getters
    bsp_can_rx_mode_e get_mode() const;
    uint32_t get_id() const;
    uint32_t get_ide() const;
    uint32_t get_dlc() const;
    uint32_t get_fifo() const;
    bsp_can_status_e get_arxd(uint8_t* const prxd) const; // 获取内部缓存

    // methods
    bsp_can_status_e filter_cfg(); // 配置滤波器 (过滤器配置必须在CAN初始化之后，由用户自己决定配置过滤器的时机)
};

bsp_can_status_e bsp_can_init_all();

#endif
