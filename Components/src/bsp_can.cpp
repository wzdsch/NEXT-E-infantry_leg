/*
 * @Author: wzdsch 1919524828@qq.com
 * @Date: 2025-09-02 23:09:59
 * @LastEditors: wzdsch 1919524828@qq.com
 * @LastEditTime: 2025-09-06 17:00:48
 * @FilePath: /leg/Components/src/bsp_can.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "bsp_can.hpp"
#include "can.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_def.h"

#include <vector>
#include <cstring>

/////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// CAN_TX BEGIN ///////////////////////////////////////////////////

//  constructors begin //

bsp_can_tx_instance::bsp_can_tx_instance(CAN_HandleTypeDef* const hcan, const uint32_t id, const uint32_t ide, \
                                         const uint32_t rtr, const uint32_t dlc) : \
                                         m_hcan(hcan), m_ptxd(nullptr) {
    // 验证参数正确性
    if (hcan != &hcan1 && hcan != &hcan2) {
        m_mode = BSP_CAN_TX_ERROR;
        return;
    }
    if (ide != CAN_ID_STD && ide != CAN_ID_EXT) {
        m_mode = BSP_CAN_TX_ERROR;
        return;
    }
    if (rtr != CAN_RTR_DATA && rtr != CAN_RTR_REMOTE) {
        m_mode = BSP_CAN_TX_ERROR;
        return;
    }
    if (dlc < 0x00U || dlc > 0x08U) {
        m_mode = BSP_CAN_TX_ERROR;
        return;
    }

    // 初始化参数
    m_mode = BSP_CAN_TX_DISABLE; // 发送默认关闭

    m_tx_header.IDE = ide;
    if (ide == CAN_ID_STD) {
        m_tx_header.StdId = id;
    }
    else {
        m_tx_header.ExtId = id;
    }
    m_tx_header.RTR = rtr;
    m_tx_header.DLC = dlc;
}

bsp_can_tx_instance::bsp_can_tx_instance(CAN_HandleTypeDef* const hcan, const uint32_t id, const uint32_t ide,
                                         const uint32_t rtr, const uint32_t dlc, const uint8_t* const txd) : \
                                         bsp_can_tx_instance(hcan, id, ide, rtr, dlc) {
    m_ptxd = txd;
}

// constructors end //

// destructors begin //



// destructors end //

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
        return BSP_CAN_ERROR;
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
    if (m_mode == BSP_CAN_TX_ENABLE && m_ptxd != nullptr && HAL_CAN_GetState(m_hcan) == HAL_CAN_STATE_READY) {
        if(HAL_CAN_AddTxMessage(m_hcan, &m_tx_header, m_ptxd, &m_mailbox) == HAL_OK){
            return BSP_CAN_OK;
        }
    }

    // 未使能 / 数据为空指针 / CAN状态异常 / 未发送成功
    return BSP_CAN_ERROR;
}

// methods end //

//////////////////////////////////// CAN_TX END ///////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// CAN_RX START /////////////////////////////////////////////////

// static members begin //

uint8_t bsp_can_rx_instance::sm_can1_filter_index = 0; // can1起始过滤器默认为0
uint8_t bsp_can_rx_instance::sm_can2_filter_index = BSP_CAN2_FILTER_START; // can2起始过滤器默认为14(可修改)

std::vector<bsp_can_rx_instance*> bsp_can_rx_instance::spm_rx_instances;

// static members end //

// constuctors begin //

bsp_can_rx_instance::bsp_can_rx_instance(CAN_HandleTypeDef* const hcan, const uint32_t id, const uint32_t ide) : \
                                         m_hcan(hcan), m_prxd(nullptr) {
    if (hcan != &hcan1 && hcan != &hcan2) {
        m_mode = BSP_CAN_RX_ERROR;
        return;
    }
    if (ide != CAN_ID_STD && ide != CAN_ID_EXT) {
        m_mode = BSP_CAN_RX_ERROR;
        return;
    }

    m_mode = BSP_CAN_RX_DISABLE; // 接收默认关闭，需与filterActivation同步
    m_hcan = hcan;
    m_id = id;
    m_ide = ide;

    m_arxd[0] = 0;
    m_arxd[1] = 0;
    m_arxd[2] = 0;
    m_arxd[3] = 0;
    m_arxd[4] = 0;
    m_arxd[5] = 0;
    m_arxd[6] = 0;
    m_arxd[7] = 0;

    if (hcan == &hcan1) {
        // 检查过滤器索引是否超出
        if (sm_can1_filter_index >= BSP_CAN2_FILTER_START) {
            m_mode = BSP_CAN_RX_ERROR;
            return;
        }
        m_filter.FilterBank = sm_can1_filter_index++;
    } else {
        if (sm_can2_filter_index > 27) {
            m_mode = BSP_CAN_RX_ERROR;
            return;
        }
        m_filter.FilterBank = sm_can2_filter_index++;
    }
    m_filter.FilterMode = CAN_FILTERMODE_IDLIST;
    m_filter.FilterActivation = DISABLE; // 接收默认关闭，需与m_mode同步

    m_filter.FilterScale = CAN_FILTERSCALE_16BIT;
    m_filter.FilterIdHigh = id << 5;
    if (ide == CAN_ID_STD) {
        m_filter.FilterScale = CAN_FILTERSCALE_16BIT;
        m_filter.FilterIdHigh = (id << 5) & 0xFFFF;
        m_filter.FilterIdLow = 0x0000;
    } else {
        m_filter.FilterScale = CAN_FILTERSCALE_32BIT;
        m_filter.FilterIdHigh = (id >> 13) & 0xFFFF; // 29位ID的高16位
        m_filter.FilterIdLow = (id & 0x1FFF) << 3; // 29位ID的低13位
    }

    // 交替使用FIFO0和FIFO1
    // 注意: HAL库中分别定义了 接收数据所需要读取的FIFO：CAN_RX_FIFO 和 与过滤器关联的FIFO：CAN_FILTER_FIFO
    //      虽然他们的值对应的FIFO是相同的，但是含义不同，最好分开使用
    if (m_filter.FilterBank % 2) {
        m_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
        m_rxfifo = CAN_RX_FIFO0;
    }
    else {
        m_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
        m_rxfifo = CAN_RX_FIFO1;
    }

    m_filter.SlaveStartFilterBank = BSP_CAN2_FILTER_START;
    if (HAL_CAN_ConfigFilter(hcan, &m_filter) != HAL_OK) {
        m_mode = BSP_CAN_RX_ERROR;
    }

    // 为映射表分配至少容纳28个实例的空间
    static bool is_reserved = false;
    if (!is_reserved) {
        is_reserved = true;
        spm_rx_instances.reserve(28);
    }

    spm_rx_instances.push_back(this);
}

bsp_can_rx_instance::bsp_can_rx_instance(CAN_HandleTypeDef* const hcan, const uint32_t id, \
                                         const uint32_t ide, uint8_t* const prxd) :\
                                         bsp_can_rx_instance(hcan, id, ide) {
    m_prxd = prxd;
}

// constructors end //

// destructors begin //

bsp_can_rx_instance::~bsp_can_rx_instance() {
    // 迭代器遍历接收映射表，找到并删除自身
    for (auto it = spm_rx_instances.begin(); it != spm_rx_instances.end(); it++) {
        if (*it == this) {
            spm_rx_instances.erase(it);
            break;
        }
    }
}

// destructors end //

// setters begin //

void bsp_can_rx_instance::disable() {
    if (m_mode != BSP_CAN_RX_ERROR) {
        m_mode = BSP_CAN_RX_DISABLE;
        m_filter.FilterActivation = DISABLE;
        HAL_CAN_ConfigFilter(m_hcan, &m_filter);
    }
}

void bsp_can_rx_instance::enable() {
    if (m_mode != BSP_CAN_RX_ERROR) {
        m_mode = BSP_CAN_RX_ENABLE;
        m_filter.FilterActivation = ENABLE;
        HAL_CAN_ConfigFilter(m_hcan, &m_filter);
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
    return m_id;
}

uint32_t bsp_can_rx_instance::get_ide() const {
    return m_ide;
}

uint32_t bsp_can_rx_instance::get_dlc() const {
    return m_rx_header.DLC;
}

uint32_t bsp_can_rx_instance::get_fifo() const {
    return m_filter.FilterFIFOAssignment;
}

const uint8_t* const bsp_can_rx_instance::get_arxd() const {
    return m_arxd;
}

// getters end //

// methods begin //

void bsp_can_rx_instance::bsp_can_get_msg_to_instances(const CAN_HandleTypeDef *const hcan, const uint32_t fifo, \
                                  const CAN_RxHeaderTypeDef *const rx_header, const uint8_t *const rx_data) {
        // 遍历映射表，查找匹配的实例
        for (auto it = spm_rx_instances.begin(); it != spm_rx_instances.end(); it++) {
        if ((*it)->m_hcan == hcan &&
            (*it)->m_rxfifo == fifo &&
            (*it)->m_ide == rx_header->IDE &&
            (*it)->m_id == ((rx_header->IDE == CAN_ID_STD) ? rx_header->StdId : rx_header->ExtId)) {
            // 如果参数匹配，则对此实例赋值
            // 听ai说memcpy有点危险，改用直接赋值
            (*it)->m_rx_header = *rx_header; // 对结构体的直接赋值？

            uint32_t copy_len = (rx_header->DLC > 8) ? 8 : rx_header->DLC;
            memcpy((*it)->m_arxd, rx_data, copy_len);
            if ((*it)->m_prxd != nullptr) {
                memcpy((*it)->m_prxd, rx_data, copy_len);
            }
            break;
        }
    }
}

// methods end //

//////////////////////////////////// CAN_RX END ///////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// OTHER FUNCTIONS BEGIN /////////////////////////////////////////////

status_e bsp_can_init_all() {
    if (HAL_CAN_Start(&hcan1) == HAL_OK && \
        HAL_CAN_Start(&hcan2) == HAL_OK && \
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK && \
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING) == HAL_OK && \
        HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK && \
        HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) == HAL_OK ) {
            return BSP_CAN_OK;
    }
    return BSP_CAN_ERROR;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// RECEIVE CALLBACKS BEGIN////////////////////////////////////////////

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef p_rx_header;
    uint8_t rx_data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &p_rx_header, rx_data) == HAL_OK) {
        bsp_can_rx_instance::bsp_can_get_msg_to_instances(hcan, CAN_RX_FIFO0, &p_rx_header, rx_data);
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef p_rx_header;
    uint8_t rx_data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &p_rx_header, rx_data) == HAL_OK) {
        bsp_can_rx_instance::bsp_can_get_msg_to_instances(hcan, CAN_RX_FIFO1, &p_rx_header, rx_data);
    }
}

/////////////////////////////// RECEIVE CALLBACKS END /////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
