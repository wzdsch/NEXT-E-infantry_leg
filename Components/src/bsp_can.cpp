/*
 * @Author: wzdsch 1919524828@qq.com
 * @Date: 2025-09-02 23:09:59
 * @LastEditors: wzdsch 1919524828@qq.com
 * @LastEditTime: 2025-09-06 11:14:48
 * @FilePath: /leg/Components/src/bsp_can.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "bsp_can.hpp"
#include "can.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_def.h"


status_e bsp_can_init_all() {
    if (HAL_CAN_Start(&hcan1) == HAL_OK && \
        HAL_CAN_Start(&hcan2) == HAL_OK && \
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK && \
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING) == HAL_OK && \
        HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK && \
        HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) == HAL_OK ) {
            return MY_OK;
    }
    return MY_ERROR;
}

//////////////////////////////////// CAN_TX BEGIN ///////////////////////////////////////////////////

bsp_can_tx_instance::bsp_can_tx_instance(CAN_HandleTypeDef *hcan, uint32_t id, uint32_t ide, uint32_t rtr, \
    uint32_t dlc) : m_hcan(hcan), m_ptxd(nullptr) {
    m_mode = BSP_CAN_TX_DISABLE; // 默认关闭

    m_tx_header.IDE = ide;
    if (ide == CAN_ID_STD) {
        m_tx_header.StdId = id;
    }
    else if (ide == CAN_ID_EXT) {
        m_tx_header.ExtId = id;
    }
    m_tx_header.RTR = rtr;
    m_tx_header.DLC = dlc;
}

bsp_can_tx_instance::bsp_can_tx_instance(CAN_HandleTypeDef *hcan, uint32_t id, uint32_t ide, uint32_t rtr, \
    uint32_t dlc, const uint8_t* const txd) : bsp_can_tx_instance(hcan, id, ide, rtr, dlc) {
    m_ptxd = txd;
}

// setters begin //

void bsp_can_tx_instance::disable() {
    if (m_mode != BSP_CAN_TX_ERROR) {
        m_mode = BSP_CAN_TX_DISABLE;
    }
}

void bsp_can_tx_instance::enable() {
    if (m_mode != BSP_CAN_TX_ERROR) {
        m_mode = BSP_CAN_TX_ENABLE;
    }
}

void bsp_can_tx_instance::set_ptxd(const uint8_t* const txd) {
    m_ptxd = txd;
}

void bsp_can_tx_instance::set_dlc(uint32_t dlc) {
    m_tx_header.DLC = dlc;
}

// setters end //

// getters begin //

CAN_HandleTypeDef* bsp_can_tx_instance::get_can_handle() const {
    return m_hcan;
}

bsp_can_tx_mode_e bsp_can_tx_instance::get_mode() const {
    return m_mode;
}

uint32_t bsp_can_tx_instance::get_id() const {
    if (m_tx_header.IDE == CAN_ID_STD) {
        return m_tx_header.StdId;
    } else if (m_tx_header.IDE == CAN_ID_EXT) {
        return m_tx_header.ExtId;
    } else {
        return MY_ERROR;
    }
}

uint32_t bsp_can_tx_instance::get_ide() const {
    return m_tx_header.IDE;
}

uint32_t bsp_can_tx_instance::get_dlc() const {
    return m_tx_header.DLC;
}

uint32_t bsp_can_tx_instance::get_mailbox() const {
    return m_mailbox;
}

// getters end //

// methods begin //

status_e bsp_can_tx_instance::transmit() {
    if (m_mode == BSP_CAN_TX_ENABLE && m_ptxd != nullptr) {
        if(HAL_CAN_AddTxMessage(m_hcan, &m_tx_header, m_ptxd, &m_mailbox) == HAL_OK){
            return MY_OK;
        }
    }
    return MY_ERROR;
}

// methods end //

//////////////////////////////////// CAN_TX END ///////////////////////////////////////////////////

//////////////////////////////////// CAN_RX START /////////////////////////////////////////////////

// static members begin //

uint8_t bsp_can_rx_instance::sm_can1_filter_index = 0; // can1起始过滤器默认为0
uint8_t bsp_can_rx_instance::sm_can2_filter_index = BSP_CAN2_FILTER_START; // can2起始过滤器默认为14(可修改)

// static members end //

bsp_can_rx_instance::bsp_can_rx_instance(CAN_HandleTypeDef* hcan, uint32_t id, uint32_t ide) : \
     m_hcan(hcan), m_prxd(nullptr) {
    m_mode = BSP_CAN_RX_DISABLE; // 默认关闭
    m_hcan = hcan;
    m_rx_header.IDE = ide;
    if(ide == CAN_ID_STD){
        m_rx_header.StdId = id;
    } else if (ide == CAN_ID_EXT) {
        m_rx_header.ExtId = id;
    }
    else {
        // 谁他妈教你乱填参数的
        m_mode = BSP_CAN_RX_ERROR;
    }

    m_arxd[0] = 0;
    m_arxd[1] = 0;
    m_arxd[2] = 0;
    m_arxd[3] = 0;
    m_arxd[4] = 0;
    m_arxd[5] = 0;
    m_arxd[6] = 0;
    m_arxd[7] = 0;

    if (hcan == &hcan1) {
        m_filter.FilterBank = sm_can1_filter_index++;
    } else if (hcan == &hcan2) {
        m_filter.FilterBank = sm_can2_filter_index++;
    } else {
        // 谁他妈教你乱填参数的
        m_mode = BSP_CAN_RX_ERROR;
    }
    m_filter.FilterMode = CAN_FILTERMODE_IDLIST;
    m_filter.FilterActivation = ENABLE;

    m_filter.FilterScale = CAN_FILTERSCALE_16BIT;
    m_filter.FilterIdHigh = id << 5;
    if (ide == CAN_ID_STD) {
        m_filter.FilterScale = CAN_FILTERSCALE_16BIT;
        m_filter.FilterIdHigh = (id << 5) & 0xFFFF;  // 正确位布局，加掩码
        m_filter.FilterIdLow = 0x0000;
    }
    else if (ide == CAN_ID_EXT) {
        m_filter.FilterScale = CAN_FILTERSCALE_32BIT;
        m_filter.FilterIdHigh = (id >> 13) & 0xFFFF; // 29位ID的高16位
        m_filter.FilterIdLow = (id & 0x1FFF) << 3;
    }
    else {
        // 谁他妈教你乱填参数的
        m_mode = BSP_CAN_RX_ERROR;
    }
    m_filter.FilterFIFOAssignment = (m_filter.FilterBank % 2) ? CAN_RX_FIFO0 : CAN_RX_FIFO1; // 交替使用FIFO
    m_filter.SlaveStartFilterBank = BSP_CAN2_FILTER_START;
    if (HAL_CAN_ConfigFilter(hcan, &m_filter) != HAL_OK) {
        m_mode = BSP_CAN_RX_ERROR;
    }
}

bsp_can_rx_instance::bsp_can_rx_instance(CAN_HandleTypeDef* hcan, uint32_t id, uint32_t ide, uint8_t* prxd) :
    bsp_can_rx_instance(hcan, id, ide) {
    m_prxd = prxd;
}

// setters begin //

void bsp_can_rx_instance::disable() {
    if (m_mode != BSP_CAN_RX_ERROR) {
        m_mode = BSP_CAN_RX_DISABLE;
    }
}

void bsp_can_rx_instance::enable() {
    if (m_mode != BSP_CAN_RX_ERROR) {
        m_mode = BSP_CAN_RX_ENABLE;
    }
}

void bsp_can_rx_instance::set_prxd(uint8_t* prxd) {
    m_prxd = prxd;
}

// setters end //

// getters begin //

bsp_can_rx_mode_e bsp_can_rx_instance::get_mode() const {
    return m_mode;
}

uint32_t bsp_can_rx_instance::get_id() const {
    if (m_rx_header.IDE == CAN_ID_STD) {
        return m_rx_header.StdId;
    } else if (m_rx_header.IDE == CAN_ID_EXT) {
        return m_rx_header.ExtId;
    } else {
        // 谁他妈教你乱填参数的
        return MY_ERROR;
    }
}

uint32_t bsp_can_rx_instance::get_ide() const {
    return m_rx_header.IDE;
}

uint32_t bsp_can_rx_instance::get_dlc() const {
    return m_rx_header.DLC;
}

uint32_t bsp_can_rx_instance::get_fifo() const {
    return m_fifo;
}

const uint8_t* const bsp_can_rx_instance::get_prxd() const {
    return m_prxd;
}

// getters end //

//////////////////////////////////// CAN_RX END ///////////////////////////////////////////////////
