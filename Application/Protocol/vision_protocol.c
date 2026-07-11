/**
 * @file    vision_protocol.c
 * @brief   高复用视觉协议 V4.0 实现 (普通单字节中断版)
 */

#include "vision_protocol.h"
#include <string.h>

/*============================ 全局变量定义 ============================*/

volatile VisionPacket_t g_vision = {0};                 ///< 全局数据包（零初始化）
uint8_t g_vision_rx_buf[VISION_RX_BUF_SIZE] = {0};      ///< 接收缓冲区

static UART_HandleTypeDef *s_huart = NULL;              ///< 串口句柄（内部保存）
static const FrameConfig_t *s_current_cfg = NULL;       ///< 当前使用的配置

// 【新增】单字节中断接收缓存
static uint8_t s_rx_byte = 0;
// 记录当前缓冲区已经存了多少个字节
static uint16_t s_rx_len = 0;

/*============================ 预定义配置表 ============================*/

const FrameConfig_t g_config_QD4310 = {
    .name = "QD4310",
    .sof = {0xAA, 0x55},
    .field_num = 5,
    .fields = {
        {FIELD_VALID,     1, 0, 1.0f},
        {FIELD_DX,        2, 1, 1.0f},
        {FIELD_DY,        2, 1, 1.0f},
        {FIELD_PHASE,     2, 0, 65535.0f / 360.0f},
        {FIELD_RPM,       2, 1, 100.0f},
    }
};

const FrameConfig_t g_config_linefollow = {
    .name = "LineFollow",
    .sof = {0xAA, 0x55},
    .field_num = 2,
    .fields = {
        {FIELD_VALID,     1, 0, 1.0f},
        {FIELD_DX,        2, 1, 1.0f},
    }
};

const FrameConfig_t g_config_multitarget = {
    .name = "MultiTarget",
    .sof = {0xAA, 0x55},
    .field_num = 5,
    .fields = {
        {FIELD_VALID,     1, 0, 1.0f},
        {FIELD_TYPE,      1, 0, 1.0f},
        {FIELD_DX,        2, 1, 1.0f},
        {FIELD_DY,        2, 1, 1.0f},
        {FIELD_CONFIDENCE,1, 0, 1.0f},
        {FIELD_CUSTOM0,   1, 0, 1.0f},
    }
};

/*============================ 内部工具函数 ============================*/

static uint8_t CalcFrameLen(const FrameConfig_t *cfg)
{
    if (!cfg) return 0;
    uint8_t len = 2;
    for (uint8_t i = 0; i < cfg->field_num; i++) {
        len += cfg->fields[i].size;
    }
    len += 1;
    return len;
}

static uint8_t CalcChecksum(uint8_t *data, uint8_t len)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

static uint8_t PackFrame(const FrameConfig_t *cfg, const VisionPacket_t *pkt, uint8_t *out)
{
    uint8_t pos = 0;
    out[pos++] = cfg->sof[0];
    out[pos++] = cfg->sof[1];

    for (uint8_t i = 0; i < cfg->field_num; i++) {
        const FieldDef_t *f = &cfg->fields[i];
        int32_t val = 0;

        switch (f->type) {
            case FIELD_VALID:     val = (int32_t)(pkt->valid / f->scale); break;
            case FIELD_DX:        val = (int32_t)(pkt->dx / f->scale); break;
            case FIELD_DY:        val = (int32_t)(pkt->dy / f->scale); break;
            case FIELD_TYPE:      val = (int32_t)(pkt->type / f->scale); break;
            case FIELD_PHASE:     val = (int32_t)(pkt->phase / f->scale); break;
            case FIELD_RPM:       val = (int32_t)(pkt->rpm / f->scale); break;
            case FIELD_CONFIDENCE:val = (int32_t)(pkt->confidence / f->scale); break;
            case FIELD_TIMESTAMP: val = (int32_t)(pkt->timestamp / f->scale); break;
            case FIELD_CUSTOM0:   val = (int32_t)(pkt->custom[0] / f->scale); break;
            case FIELD_CUSTOM1:   val = (int32_t)(pkt->custom[1] / f->scale); break;
            case FIELD_CUSTOM2:   val = (int32_t)(pkt->custom[2] / f->scale); break;
            case FIELD_CUSTOM3:   val = (int32_t)(pkt->custom[3] / f->scale); break;
            default:              val = 0; break;
        }

        for (uint8_t b = 0; b < f->size; b++) {
            out[pos++] = (uint8_t)((val >> (b * 8)) & 0xFF);
        }
    }

    out[pos] = CalcChecksum(out, pos);
    pos++;
    return pos;
}

static bool UnpackFrame(const FrameConfig_t *cfg, uint8_t *data, uint16_t len, VisionPacket_t *out)
{
    if (!cfg || !data || !out) return false;

    uint8_t frame_len = CalcFrameLen(cfg);
    if (len < frame_len) return false;

    for (uint16_t i = 0; i <= len - frame_len; i++) {
        if (data[i] != cfg->sof[0] || data[i+1] != cfg->sof[1]) {
            continue;
        }

        uint8_t calc_chk = CalcChecksum(&data[i], frame_len - 1);
        if (calc_chk != data[i + frame_len - 1]) {
            continue;
        }

        memset(out, 0, sizeof(VisionPacket_t));
        uint8_t pos = i + 2;

        for (uint8_t f = 0; f < cfg->field_num; f++) {
            const FieldDef_t *fd = &cfg->fields[f];
            int32_t raw_val = 0;
            for (uint8_t b = 0; b < fd->size; b++) {
                raw_val |= (int32_t)(data[pos + b]) << (b * 8);
            }

            if (fd->is_signed && fd->size < 4) {
                int32_t sign_bit = 1 << (fd->size * 8 - 1);
                if (raw_val & sign_bit) {
                    raw_val |= ~((1 << (fd->size * 8)) - 1);
                }
            }

            float real_val = (float)raw_val * fd->scale;
            switch (fd->type) {
                case FIELD_VALID:     out->valid = (uint8_t)real_val; break;
                case FIELD_DX:        out->dx = (int16_t)real_val; break;
                case FIELD_DY:        out->dy = (int16_t)real_val; break;
                case FIELD_TYPE:      out->type = (uint8_t)real_val; break;
                case FIELD_PHASE:     out->phase = (uint16_t)real_val; break;
                case FIELD_RPM:       out->rpm = (int16_t)real_val; break;
                case FIELD_CONFIDENCE:out->confidence = (uint8_t)real_val; break;
                case FIELD_TIMESTAMP: out->timestamp = (uint32_t)real_val; break;
                case FIELD_CUSTOM0:   out->custom[0] = (int16_t)real_val; break;
                case FIELD_CUSTOM1:   out->custom[1] = (int16_t)real_val; break;
                case FIELD_CUSTOM2:   out->custom[2] = (int16_t)real_val; break;
                case FIELD_CUSTOM3:   out->custom[3] = (int16_t)real_val; break;
                default: break;
            }

            pos += fd->size;
        }

        out->recv_time = HAL_GetTick();
        out->data_valid = 1;
        return true;
    }

    return false;
}

/*============================ 接口函数实现 ============================*/

void Vision_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_current_cfg = &g_config_QD4310;

    memset((void*)&g_vision, 0, sizeof(g_vision));
    memset(g_vision_rx_buf, 0, VISION_RX_BUF_SIZE);
    s_rx_len = 0;

    // 【修改点】启动单字节普通中断接收
    if (huart) {
        HAL_UART_Receive_IT(huart, &s_rx_byte, 1);
    }
}

bool Vision_SelectConfig(const char *name)
{
    const FrameConfig_t *new_cfg = NULL;

    if (strcmp(name, "QD4310") == 0) {
        new_cfg = &g_config_QD4310;
    } else if (strcmp(name, "LineFollow") == 0) {
        new_cfg = &g_config_linefollow;
    } else if (strcmp(name, "MultiTarget") == 0) {
        new_cfg = &g_config_multitarget;
    } else {
        return false;
    }

    s_current_cfg = new_cfg;

    // 重置接收状态
    s_rx_len = 0;
    memset(g_vision_rx_buf, 0, VISION_RX_BUF_SIZE);
    if (s_huart) {
        HAL_UART_AbortReceive(s_huart);
        // 【修改点】重启单字节普通中断接收
        HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
    }

    return true;
}

// 【重点修改】函数名变了！参数也变了！因为普通中断不需要传Size
void Vision_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != s_huart || !s_current_cfg) return;

    // 1. 将刚收到的 1 个字节存入大数组
    g_vision_rx_buf[s_rx_len++] = s_rx_byte;

    // 2. 判断目前数组里的数据够不够一帧的长度
    uint8_t target_len = CalcFrameLen(s_current_cfg);

    if (s_rx_len >= target_len) {
        VisionPacket_t temp;
        // 3. 尝试解析
        if (UnpackFrame(s_current_cfg, g_vision_rx_buf, s_rx_len, &temp)) {
            // 解析成功！保存数据
            __disable_irq();
            memcpy((void*)&g_vision, &temp, sizeof(VisionPacket_t));
            __enable_irq();

            // 解析成功后，清空整个数组，从头开始存下一个新帧
            s_rx_len = 0;
            // memset(g_vision_rx_buf, 0, VISION_RX_BUF_SIZE); // (可选)
        }
        else {
            // 解析失败：说明里面有错位的数据或者垃圾字节
            // 防溢出保护：如果数组塞满了都没找到正确的帧头，就丢弃最老的一个字节，整体往前挪
            if (s_rx_len >= VISION_RX_BUF_SIZE) {
                for (int i = 1; i < VISION_RX_BUF_SIZE; i++) {
                    g_vision_rx_buf[i - 1] = g_vision_rx_buf[i];
                }
                s_rx_len--; // 数组长度减1，让出最后一个位置给下次中断
            }
        }
    }

    // 4. 再次开启单字节接收中断
    HAL_UART_Receive_IT(huart, &s_rx_byte, 1);
}

bool Vision_SendCommand(uint8_t cmd, int16_t param)
{
    if (!s_huart) return false;

    uint8_t tx_buf[9];
    tx_buf[0] = 0xAA;
    tx_buf[1] = 0x55;
    tx_buf[2] = cmd & 0xFF;
    tx_buf[3] = 0;
    tx_buf[4] = param & 0xFF;
    tx_buf[5] = (param >> 8) & 0xFF;
    tx_buf[6] = 0;
    tx_buf[7] = 0;
    tx_buf[8] = CalcChecksum(tx_buf, 8);

    return (HAL_UART_Transmit(s_huart, tx_buf, 9, 100) == HAL_OK);
}

const FrameConfig_t* Vision_GetCurrentConfig(void)
{
    return s_current_cfg;
}

uint8_t Vision_GetFrameLen(void)
{
    return s_current_cfg ? CalcFrameLen(s_current_cfg) : 0;
}

bool Vision_ParseManual(uint8_t *raw_data, uint16_t len, VisionPacket_t *out)
{
    if (!s_current_cfg) return false;
    return UnpackFrame(s_current_cfg, raw_data, len, out);
}

bool Vision_SendPacket(const VisionPacket_t* pkt)
{
    if (!s_huart || !s_current_cfg) return false;

    uint8_t tx_buf[VISION_MAX_FRAME_LEN];
    uint8_t len = PackFrame(s_current_cfg, pkt, tx_buf);

    return (HAL_UART_Transmit(s_huart, tx_buf, len, 100) == HAL_OK);
}