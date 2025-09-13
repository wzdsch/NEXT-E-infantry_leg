/*
 * @Author: wzdsch 1919524828@qq.com
 * @Date: 2025-09-02 23:09:59
 * @LastEditors: wzdsch 1919524828@qq.com
 * @LastEditTime: 2025-09-13 11:41:06
 * @FilePath: /leg/Components/src/bsp_can.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "bsp_can.hpp"
#include "can.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_def.h"

#include <cstddef>
#include <cstdint>
#include <vector>
#include <cstring>
#include <iterator>

/////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// CAN_TX BEGIN ///////////////////////////////////////////////////

//  constructors begin //

bsp_can_tx_instance::bsp_can_tx_instance(CAN_HandleTypeDef* const hcan, const uint32_t id, const uint32_t ide, \
                                         const uint32_t rtr, const uint32_t dlc) : \
                                         m_hcan(hcan) {
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
    if (dlc > 0x08U) {
        m_mode = BSP_CAN_TX_ERROR;
        return;
    }
    if (ide == CAN_ID_STD && id > 0x7FF) {  // 标准ID：11位，0~0x7FF
        m_mode = BSP_CAN_TX_ERROR;
        return;
    }
    if (ide == CAN_ID_EXT && id > 0x1FFFFFFF) {  // 扩展ID：29位，0~0x1FFFFFFF
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

bsp_can_status_e bsp_can_tx_instance::set_dlc(const uint32_t dlc) {
    if (dlc <= 0x08U) {
        m_tx_header.DLC = dlc;
        return BSP_CAN_OK;
    }
    return BSP_CAN_ERROR;
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
    } else {
        return m_tx_header.ExtId;
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

bsp_can_status_e bsp_can_tx_instance::transmit(const uint8_t* const ptxd) {
    if (m_mode == BSP_CAN_TX_ENABLE && ptxd != nullptr && HAL_CAN_GetState(m_hcan) == HAL_CAN_STATE_READY) {
        HAL_StatusTypeDef hal_tx_status = HAL_CAN_AddTxMessage(m_hcan, &m_tx_header, ptxd, &m_mailbox);
        if (hal_tx_status == HAL_OK) {
            return BSP_CAN_OK;
        } else if (hal_tx_status == HAL_BUSY) {
            return BSP_CAN_BUSY;
        }
    }

    // 未使能 / 数据为空指针 / CAN状态异常 / 未发送成功 / dlc参数错误
    return BSP_CAN_ERROR;
}


bsp_can_status_e bsp_can_tx_instance::transmit(const uint8_t* const ptxd, uint32_t dlc) {
    if (dlc > 0x08U) {
        return BSP_CAN_ERROR;
    }

    m_tx_header.DLC = dlc;
    return transmit(ptxd);
}

// methods end //

//////////////////////////////////// CAN_TX END ///////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// CAN_RX START /////////////////////////////////////////////////

// static members begin //

uint8_t bsp_can_rx_instance::sm_can1_filter_index = 0; // can1起始过滤器默认为0
uint8_t bsp_can_rx_instance::sm_can2_filter_index = BSP_CAN2_FILTER_START; // can2起始过滤器默认为14(可修改)

bsp_can_rx_instance* bsp_can_rx_instance::spm_rx_instances[28] = {nullptr};

// static members end //

// constuctors begin //

bsp_can_rx_instance::bsp_can_rx_instance(CAN_HandleTypeDef* const hcan, const uint32_t id, const uint32_t ide, const std::function<void()> callback) : \
                                         m_hcan(hcan), m_id(id), m_ide(ide), m_callback(callback) {
    if (hcan != &hcan1 && hcan != &hcan2) {
        m_mode = BSP_CAN_RX_ERROR;
        return;
    }
    if (ide != CAN_ID_STD && ide != CAN_ID_EXT) {
        m_mode = BSP_CAN_RX_ERROR;
        return;
    }
    if (ide == CAN_ID_STD && id > 0x7FF) {
        m_mode = BSP_CAN_RX_ERROR;
        return;
    } else if (id > 0x1FFFFFFF) {
        m_mode = BSP_CAN_RX_ERROR;
        return;
    }

    m_mode = BSP_CAN_RX_DISABLE; // 接收默认关闭，需与filterActivation同步

    // 初始化双缓冲区
    for (int i = 0; i < 8; i++) {
        ma_rxd1[i] = 0;
        ma_rxd2[i] = 0;
    }
    mp_last_received_data = ma_rxd1;

    // 配置过滤器结构体
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

    if (ide == CAN_ID_STD) { // stdID
        m_filter.FilterScale = CAN_FILTERSCALE_16BIT;
        m_filter.FilterIdHigh = (id << 5) & 0xFFFF;
        m_filter.FilterIdLow = 0x0000;
    } else { // extID
        m_filter.FilterScale = CAN_FILTERSCALE_32BIT;
        m_filter.FilterIdHigh = (id >> 13) & 0xFFFF; // 29位ID的高16位
        m_filter.FilterIdLow = ((id  << 3) & 0xFFF8) | CAN_ID_EXT; // 29位ID的低13位
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

    // 后续提供了配置过滤器的方法, 以上只初始化数据, 配置要调用方法

    // // 为映射表分配至少容纳28个实例的空间
    // static bool is_reserved = false;
    // if (!is_reserved) {
    //     is_reserved = true;
    //     spm_rx_instances.reserve(28);
    // }

    spm_rx_instances[m_filter.FilterBank] = this;
}

// constructors end //

// destructors begin //

bsp_can_rx_instance::~bsp_can_rx_instance() {
    // 迭代器遍历接收映射表，找到并删除自身
    for (auto instance : spm_rx_instances) {
        if (instance == this) {
            spm_rx_instances[this->m_filter.FilterBank] = nullptr;
            break;
        }
    }
}

// destructors end //

// setters begin //

bsp_can_status_e bsp_can_rx_instance::disable() {
    if (m_mode != BSP_CAN_RX_ERROR) {
        m_filter.FilterActivation = DISABLE;
        if (HAL_CAN_ConfigFilter(m_hcan, &m_filter) == HAL_OK) {
            m_mode = BSP_CAN_RX_DISABLE;
            return BSP_CAN_OK;
        }
    }
    return BSP_CAN_ERROR;
}

bsp_can_status_e bsp_can_rx_instance::enable() {
    if (m_mode != BSP_CAN_RX_ERROR) {
        m_filter.FilterActivation = ENABLE;
        if (HAL_CAN_ConfigFilter(m_hcan, &m_filter) == HAL_OK) {
            m_mode = BSP_CAN_RX_ENABLE;
            return BSP_CAN_OK;
        }
    }
    return BSP_CAN_ERROR;
}

void bsp_can_rx_instance::set_callback(const std::function<void()> callback) {
    m_callback = callback;
}

void bsp_can_rx_instance::clear_callback() {
    m_callback = nullptr;
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
    return mp_rx_header.DLC;
}

uint32_t bsp_can_rx_instance::get_fifo() const {
    return m_filter.FilterFIFOAssignment;
}

bsp_can_status_e bsp_can_rx_instance::get_arxd(uint8_t* const prxd) const {
    if (prxd == nullptr || mp_last_received_data == nullptr || m_mode == BSP_CAN_RX_ERROR) {
        return BSP_CAN_ERROR;
    }

    uint8_t copy_len = (mp_rx_header.DLC > 0x08U) ? 8 : mp_rx_header.DLC;
    memcpy(prxd, mp_last_received_data, copy_len);
    return BSP_CAN_OK;
}

// getters end //

// methods begin //

volatile void bsp_can_rx_instance::bsp_can_get_msg_to_instances(const CAN_HandleTypeDef *const hcan, const uint32_t fifo, \
                                  const CAN_RxHeaderTypeDef *const rx_header, const uint8_t *const rx_data) {
    // 遍历映射表，查找匹配的实例
     for (bsp_can_rx_instance* instance : spm_rx_instances) {
        if (instance->m_hcan == hcan && \
            instance->m_rxfifo == fifo && \
            instance->m_ide == rx_header->IDE && \
            instance->m_id == ((rx_header->IDE == CAN_ID_STD) ? rx_header->StdId : rx_header->ExtId) && \
            instance->m_mode == BSP_CAN_RX_ENABLE) {
            // 如果参数匹配且使能接收，则对此实例赋值

            // 听ai说memcpy有点危险，改用直接赋值
            instance->mp_rx_header = *rx_header; // 对结构体的直接赋值？

            uint32_t copy_len = (rx_header->DLC > 8) ? 8 : rx_header->DLC;
            // 双缓冲区
            if (instance->mp_last_received_data == instance->ma_rxd1) {
                memcpy(instance->ma_rxd2, rx_data, copy_len);
                instance->mp_last_received_data = instance->ma_rxd2;
            } else if (instance->mp_last_received_data == instance->ma_rxd2 || instance->mp_last_received_data == nullptr) {
                memcpy(instance->ma_rxd1, rx_data, copy_len);
                instance->mp_last_received_data = instance->ma_rxd1;
            }

            if (instance->m_callback) {
                instance->m_callback();
            }
            break;
        }
    }
}

bsp_can_status_e bsp_can_rx_instance::filter_cfg() {
    if (m_mode != BSP_CAN_RX_ERROR) {
        if (HAL_CAN_ConfigFilter(m_hcan, &m_filter) == HAL_OK) {
            return BSP_CAN_OK;
        }
    }
    m_mode = BSP_CAN_RX_ERROR;
    return BSP_CAN_ERROR;
}

// methods end //

//////////////////////////////////// CAN_RX END ///////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// OTHER FUNCTIONS BEGIN /////////////////////////////////////////////

bsp_can_status_e bsp_can_init_all() {
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

 extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef p_rx_header;
    uint8_t rx_data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &p_rx_header, rx_data) == HAL_OK) {
        bsp_can_rx_instance::bsp_can_get_msg_to_instances(hcan, CAN_RX_FIFO0, &p_rx_header, rx_data);
    }
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef p_rx_header;
    uint8_t rx_data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &p_rx_header, rx_data) == HAL_OK) {
        bsp_can_rx_instance::bsp_can_get_msg_to_instances(hcan, CAN_RX_FIFO1, &p_rx_header, rx_data);
    }
}

/////////////////////////////// RECEIVE CALLBACKS END /////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
