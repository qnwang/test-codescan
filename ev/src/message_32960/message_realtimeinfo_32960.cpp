/******************************************************************************
*
*  Copyright (C) 2021 SmartLink
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/

#include <cstring>
#include <smlk_log.h>

#include "common_32960.h"
#include "message_realtimeinfo_32960.h"

using namespace smartlink;

char MessageRealTimeInfo32960::buf[M_GB_32960_SZ_DATA_MAX] = {0};

uint MessageRealTimeInfo32960::bufLength = 0;

EV_Timer *MessageRealTimeInfo32960::RTInfoUpTimer = nullptr;

MessageRealTimeInfo32960 *MessageRealTimeInfo32960::RTInfoUpPtr = nullptr;

/**
 * @brief 大小端转换
 * @param pData 待转换数据
 * @param startIndex 起始位置
 * @param length 转换字节数
 */
void EndianSwap(uint8_t *pData, int startIndex, int length) {
    int i, cnt, end, start;
    if (length == 1) return;
    cnt = length / 2;
    start = startIndex;
    end = startIndex + length - 1;
    uint8_t tmp;
    for (i = 0; i < cnt; i++) {
        tmp = pData[start + i];
        pData[start + i] = pData[end - i];
        pData[end - i] = tmp;
    }
}

MessageRealTimeInfo32960::MessageRealTimeInfo32960()
{
    memset(&m_dataUpload, 0, sizeof(m_dataUpload));
    memset(&m_dataLastReport, 0, sizeof(m_dataLastReport));

    memset(&m_last_volt_info, 0, sizeof(m_last_volt_info));
    memset(&m_last_temp_info, 0, sizeof(m_last_temp_info));

    countSendTime = 0;   //采集的索引
    m_rt_send_num = 0;
    m_is_send_num = 0;
    m_last_common_warn = 0;

    m_dataLastReport.CarInfo.car_speed = 0xFFFF;
    m_dataLastReport.CarInfo.mile = VALUE_INVALID_DWORD;
    m_dataLastReport.CarInfo.InsRes = 0;                    // 无 0xFFFE / 0xFFFF

    m_dataLastReport.MotorInfo.motor_speed = 0xFFFF;

    m_dataLastReport.FuelcellInfo.probe_temp[0] = 0;            // 无 0xFE / 0xFF
    m_dataLastReport.FuelcellInfo.probe_temp[1] = 0;            // 无 0xFE / 0xFF
    m_dataLastReport.FuelcellInfo.h_sys_high_press = 0;
    m_dataLastReport.FuelcellInfo.high_dcdc_state = 0xFF;

    m_dataLastReport.EngInfo.fuel_rate = 0xFFFF;

    m_dataLastReport.LimiInfo.max_volbat_subsys_seq = 0xFF;
    m_dataLastReport.LimiInfo.max_volbat_sigle_seq = 0xFF;
    m_dataLastReport.LimiInfo.max_sigle_bat_vol = 0xFFFF;
    m_dataLastReport.LimiInfo.min_volbat_subsys_seq = 0xFF;
    m_dataLastReport.LimiInfo.min_volbat_sigle_seq = 0xFF;
    m_dataLastReport.LimiInfo.min_sigle_bat_vol = 0xFFFF;

    m_dataLastReport.LimiInfo.max_temp_subsys_seq = 0xFF;
    m_dataLastReport.LimiInfo.max_temp_probe_seq = 0xFF;
    m_dataLastReport.LimiInfo.max_temp = 0xFF;
    m_dataLastReport.LimiInfo.min_temp_subsys_seq = 0xFF;
    m_dataLastReport.LimiInfo.min_temp_probe_seq = 0xFF;
    m_dataLastReport.LimiInfo.min_temp = 0xFF;

    m_last_volt_info.energy_device_volt = 0xFFFF;
    m_last_volt_info.energy_device_curr = 0xFFFF;
    m_last_volt_info.cell_num = 0x1;

    m_last_temp_info.energy_device_temp_probe_num = 0x01;

}

MessageRealTimeInfo32960::~MessageRealTimeInfo32960()
{
}

void MessageRealTimeInfo32960::decodeMsg(char *rawData, uint inlength)
{
    SMLK_LOGD("enter");
    if (rawData == NULL)
    {
        return;
    }
    return;
}

void MessageRealTimeInfo32960::encodeMsg(GB_32960_MsgEntity entity, uint length)
{
    int ret = -1;

    // 如果收到IGNOFF不再发送实时数据
    if (0 == common32960::getInstance()->getIgnState())
    {
        return;
    }

    ret = MessageObserver32960::GetInstance()->encodeMsg(entity, length, buf, bufLength);
    // SMLK_LOGD("ret=%d", ret);
    if (ret <= 0 && length > 0)
    {
        // 如果上传失败，则将数据保存至数据库
        SMLK_LOGD("WriteBlindData");
        common32960::getInstance()->m_SqlLiteBase32960->WriteBlindData(buf, bufLength);
    }
}

void MessageRealTimeInfo32960::encodeMsg(GB_32960_MsgEntity entity, uint length, char *dest, uint &msgLength)
{
}

void MessageRealTimeInfo32960::broadcastMsgAck(uint msgId, uint rspFlag)
{
    SMLK_LOGD("msgId %d, rspFlag %d ",msgId,rspFlag);
    if (msgId == static_cast<uint8_t>(msgID_32960::M_GB_32960_CMD_REALTIME_REPORT) && rspFlag == static_cast<uint8_t>(rspFlag_32960::M_GB_32960_RSP_FLAG_SUCCESS))
    {
        SMLK_LOGD("Send realtime info ok ");
    }
}

void MessageRealTimeInfo32960::DoRealTimeInfoUp()
{
    SMLK_UINT8 buf[1024] = {0};
    int body_len = 0;
    int last_max_warn = 0;
    int last_common_warn = 0;
    g_rtinfo_vlot_cache.clear();
    CacheRTInfoReissue rt_info_reissue;

    rt_info_reissue.rtinfo_vlot_cache.clear();
    memset(&rt_info_reissue.cachertinfo, 0, sizeof(CacheRTInfo));

    memset(&m_MessageEntity32960, 0, sizeof(m_MessageEntity32960));
    m_MessageEntity32960.msgId = static_cast<SMLK_UINT8>(msgID_32960::M_GB_32960_CMD_REALTIME_REPORT);
    m_MessageEntity32960.rspFlag = static_cast<SMLK_UINT8>(rspFlag_32960::M_GB_32960_RSP_FLAG_CMD);
    m_MessageEntity32960.encryptFlag = static_cast<int>(encryType_32960::M_GB_32960_ENCRYPTION_NONE);

    memset(m_dataUpload.sendTime, 0, sizeof(m_dataUpload.sendTime));
    common32960::getInstance()->getTimestamp(m_dataUpload.sendTime);
    SMLK_LOGD("%d-%d-%d %d:%d:%d", m_dataUpload.sendTime[0], m_dataUpload.sendTime[1], m_dataUpload.sendTime[2], m_dataUpload.sendTime[3], m_dataUpload.sendTime[4], m_dataUpload.sendTime[5]);

    memcpy(buf, m_dataUpload.sendTime, sizeof(m_dataUpload.sendTime));
    body_len += sizeof(m_dataUpload.sendTime);

//采集数据
//整车数据 0x01
// if(0)
{
    vehicle::SignalVariant m_vehicle_state          = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVVehicleStatus)->second;
    vehicle::SignalVariant m_charge_state           = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVPowerBatteryChargeStatus)->second;
    vehicle::SignalVariant m_vehicle_speed          = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVVehicleSpeed)->second;
    vehicle::SignalVariant m_total_vehicle_distance = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::TotalVehicleDistance)->second;
    vehicle::SignalVariant m_vol_total              = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVPowerBatteryVoltage)->second;
    vehicle::SignalVariant m_cur_total              = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVPowerBatteryCurrent)->second;
    vehicle::SignalVariant m_soc                    = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVPowerBatterySOC)->second;
    vehicle::SignalVariant m_dcdc                   = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVDCDCWorkStatus)->second;
    vehicle::SignalVariant m_gear                   = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVCurrentGear)->second;
    vehicle::SignalVariant m_insulation_resistance  = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVInsulationResistance)->second;
    vehicle::SignalVariant m_acc_pedal_value        = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVAcceleratorPedalPosition)->second;
    vehicle::SignalVariant m_brake_pedal_state      = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVBrakeSwitch)->second;

    m_dataUpload.CarInfo.msgType                    = static_cast<SMLK_UINT8>(rtinfoID_32960::M_GB_32960_MESSAGE_VEHICLE_INFO);
    // m_dataUpload.CarInfo.car_state
    // 0x01 启动：运行准备就绪（HCU_BMS）= 11
    // 0x02 熄火：模块采集IG OFF信号
    // 0x03 其他：运行准备就绪报文（HCU_BMS）= 00,01,10
    if (common32960::getInstance()->getIgnState() == 0 || common32960::getInstance()->getIgnState() == 2) {
        m_dataUpload.CarInfo.car_state              = 0x02;
    } else {
        if (m_vehicle_state.Valid()) {
            if (m_vehicle_state.Encoded() == 3){
                m_dataUpload.CarInfo.car_state      = 0x01;
            } else {
                m_dataUpload.CarInfo.car_state      = 0x03;
            }
        } else {
            m_dataUpload.CarInfo.car_state          = 0xFF;
        }
    }
    // m_dataUpload.CarInfo.charge_state 
    // 0x01 停车充电：Power Battery Charging Status（BMS_1）= 01
    // 0x02 行驶充电：无该状态
    // 0x03 未充电状态：Power Battery Charging Status（BMS_1）= 00
    // 0x04 充电完成：Power Battery Charging Status（BMS_1）= 10
    // 0xFE 异常：Power Battery Charging Status（BMS_1）= 11
    if (m_charge_state.Valid()) {
        if (m_charge_state.Encoded() == 1) {
            m_dataUpload.CarInfo.charge_state       = 0x01;
        } else if (m_charge_state.Encoded() == 0) {
            m_dataUpload.CarInfo.charge_state       = 0x03;
        } else if (m_charge_state.Encoded() == 2) {
            m_dataUpload.CarInfo.charge_state       = 0x04;
        } else if (m_charge_state.Encoded() == 3) {
            m_dataUpload.CarInfo.charge_state       = 0xFE;
        }
    } else {
        m_dataUpload.CarInfo.charge_state           = 0xFF;
    }   
  
    if (common32960::getInstance()->m_run_mode == 2 || common32960::getInstance()->m_run_mode == 1) {
        m_dataUpload.CarInfo.run_mode = 1;
    } else if (common32960::getInstance()->m_run_mode == 3) {
        m_dataUpload.CarInfo.run_mode = 2;
    }

    if (m_vehicle_speed.Valid() && m_vehicle_speed.Val() >= 0 && m_vehicle_speed.Val() <= 2200)
    {
        m_dataLastReport.CarInfo.car_speed = m_vehicle_speed.Encoded();
    }
    m_dataUpload.CarInfo.car_speed = m_vehicle_speed.Valid() ? m_dataLastReport.CarInfo.car_speed : 0xFFFF;

    if (m_total_vehicle_distance.Valid() && m_total_vehicle_distance.Val() >= 0 && m_total_vehicle_distance.Val() <= 999999.9)
    {
        m_dataLastReport.CarInfo.mile = m_total_vehicle_distance.Encoded();
    }
    m_dataUpload.CarInfo.mile = m_total_vehicle_distance.Valid() ? m_dataLastReport.CarInfo.mile : 0xFFFFFFFF;

    m_dataUpload.CarInfo.vol_total = m_vol_total.Valid() ? m_vol_total.Encoded() : 0xFFFF;
    m_dataUpload.CarInfo.cur_total = m_cur_total.Valid() ? m_cur_total.Encoded() : 0xFFFF;
    m_dataUpload.CarInfo.soc = m_soc.Valid() ? m_soc.Encoded() : 0xFF;
    // m_dataUpload.CarInfo.dcdc
    // 0x01 工作：DCDC 系统状态 = 01,10
    // 0x02 断开：DCDC 系统状态 = 00
    // 0xFE 异常：DCDC 系统状态 = 11
    if (m_dcdc.Valid()) {
        if ((m_dcdc.Encoded() == 1) || m_dcdc.Encoded() == 2) {
            m_dataUpload.CarInfo.dcdc               = 0x01;
        } else if (m_dcdc.Encoded() == 0) {
            m_dataUpload.CarInfo.dcdc               = 0x02;
        } else if (m_dcdc.Encoded() == 3) {
            m_dataUpload.CarInfo.dcdc               = 0xFE;
        }
    } else {
        m_dataUpload.CarInfo.dcdc                   = 0xFF;
    } 

    // m_dataUpload.CarInfo.gear 
    // Bit7 Bit6：预留，预留位用0表示
    // Bit5：1：有驱动力：加速踏板开度 ＞ 0; 0：无驱动力：加速踏板开度 = 0
    m_dataUpload.CarInfo.gear = 0;
    if (m_acc_pedal_value.Valid()) {
        if (m_acc_pedal_value.Encoded() > 0) {
            m_dataUpload.CarInfo.gear |= 1;
        }
    }
    // Bit4：1：有制动力: Brake switch = 01; 0：无制动力: Brake switch = 00
    m_dataUpload.CarInfo.gear = m_dataUpload.CarInfo.gear << 1;
    if (m_brake_pedal_state.Valid()) {
        if (m_brake_pedal_state.Encoded() == 1) {
            m_dataUpload.CarInfo.gear |= 1;
        }
    }
    // Bit3-0：挡位：=0000 空挡; =0001 1挡; =0010 2挡; =0011 3挡; =0100 4挡; =0101 5挡; =0110 6挡; =1101 倒挡; =1110 自动D挡; =1111 停车P挡;
    m_dataUpload.CarInfo.gear = m_dataUpload.CarInfo.gear << 4;
    if (m_gear.Valid()) {
        if ((m_gear.Encoded() > 0x7D)) {
            // =1110 自动D挡: Current Gear > 0x7D
            m_dataUpload.CarInfo.gear |= 0x0E;
        } else if (m_gear.Encoded() < 0x7D) {
            // =1101倒挡: Current Gear < 0x7D
            m_dataUpload.CarInfo.gear |= 0x0D;
        } else if (m_gear.Encoded() == 0x7D) {
            // =0000 空挡: Current Gear = 0x7D
            m_dataUpload.CarInfo.gear |= 0x00;
        }
    }

    if (m_insulation_resistance.Valid() && m_insulation_resistance.Val() >= 0 && m_insulation_resistance.Val() <= 60000) {
        m_dataLastReport.CarInfo.InsRes = m_insulation_resistance.Encoded();
    }
    m_dataUpload.CarInfo.InsRes = m_dataLastReport.CarInfo.InsRes;      // 无 0xFFFF

    m_dataUpload.CarInfo.acc_pedal                  = m_acc_pedal_value.Valid() ? m_acc_pedal_value.Encoded() : 0xFF;
    // m_dataUpload.CarInfo.brake_pedal
    // 0x00 制动关: Brake switch = 00 
    // 0x65 制动有效: 在无具体行程值情况下用“0x65”表示制动有效状态: Brake switch = 01
    if (m_brake_pedal_state.Valid()) {
        if ((m_brake_pedal_state.Encoded() == 0)) {
            m_dataUpload.CarInfo.brake_pedal        = 0x00;
        } else if (m_brake_pedal_state.Encoded() == 1) {
            m_dataUpload.CarInfo.brake_pedal        = 0x65;
        }
    } else {
        m_dataUpload.CarInfo.brake_pedal            = 0xFF;
    }   

    // SMLK_LOGD("msgType=0x%02X", m_dataUpload.CarInfo.msgType);
    // SMLK_LOGD("car_state=%02d", m_dataUpload.CarInfo.car_state);
    // SMLK_LOGD("charge_state=%02d", m_dataUpload.CarInfo.charge_state);
    // SMLK_LOGD("run_mode=%02d", m_dataUpload.CarInfo.run_mode);
    // SMLK_LOGD("car_speed=%02d", m_dataUpload.CarInfo.car_speed);
    // SMLK_LOGD("mile=%02d", m_dataUpload.CarInfo.mile);
    // SMLK_LOGD(" vol_total=%02d", m_dataUpload.CarInfo.vol_total);
    // SMLK_LOGD(" cur_total=%02d", m_dataUpload.CarInfo.cur_total);
    // SMLK_LOGD("soc=%02d", m_dataUpload.CarInfo.soc);
    // SMLK_LOGD("dcdc=%02d", m_dataUpload.CarInfo.dcdc);
    // SMLK_LOGD("gear=%02d", m_dataUpload.CarInfo.gear);
    // SMLK_LOGD("InsRes=%d", m_dataUpload.CarInfo.InsRes);
    // SMLK_LOGD("acc_pedal=%02d", m_dataUpload.CarInfo.acc_pedal);
    // SMLK_LOGD("brake_pedal=%02d", m_dataUpload.CarInfo.brake_pedal);

    buf[body_len++] = m_dataUpload.CarInfo.msgType;
    buf[body_len++] = m_dataUpload.CarInfo.car_state;
    buf[body_len++] = m_dataUpload.CarInfo.charge_state;
    buf[body_len++] = m_dataUpload.CarInfo.run_mode;
    EndianSwap((uint8_t *)&m_dataUpload.CarInfo.car_speed, 0, sizeof(m_dataUpload.CarInfo.car_speed));
    memcpy(&buf[body_len], &m_dataUpload.CarInfo.car_speed, sizeof(m_dataUpload.CarInfo.car_speed));
    body_len += sizeof(m_dataUpload.CarInfo.car_speed);
    EndianSwap((uint8_t *)&m_dataUpload.CarInfo.mile, 0, sizeof(m_dataUpload.CarInfo.mile));
    memcpy(&buf[body_len], &m_dataUpload.CarInfo.mile, sizeof(m_dataUpload.CarInfo.mile));
    body_len += sizeof(m_dataUpload.CarInfo.mile);
    EndianSwap((uint8_t *)&m_dataUpload.CarInfo.vol_total, 0, sizeof(m_dataUpload.CarInfo.vol_total));
    memcpy(&buf[body_len], &m_dataUpload.CarInfo.vol_total, sizeof(m_dataUpload.CarInfo.vol_total));
    body_len += sizeof(m_dataUpload.CarInfo.vol_total);
    EndianSwap((uint8_t *)&m_dataUpload.CarInfo.cur_total, 0, sizeof(m_dataUpload.CarInfo.cur_total));
    memcpy(&buf[body_len], &m_dataUpload.CarInfo.cur_total, sizeof(m_dataUpload.CarInfo.cur_total));
    body_len += sizeof(m_dataUpload.CarInfo.cur_total);
    buf[body_len++] = m_dataUpload.CarInfo.soc;
    buf[body_len++] = m_dataUpload.CarInfo.dcdc;
    buf[body_len++] = m_dataUpload.CarInfo.gear;
    EndianSwap((uint8_t *)&m_dataUpload.CarInfo.InsRes, 0, sizeof(m_dataUpload.CarInfo.InsRes));
    memcpy(&buf[body_len], &m_dataUpload.CarInfo.InsRes, sizeof(m_dataUpload.CarInfo.InsRes));
    body_len += sizeof(m_dataUpload.CarInfo.InsRes);
    buf[body_len++] = m_dataUpload.CarInfo.acc_pedal;
    buf[body_len++] = m_dataUpload.CarInfo.brake_pedal;
}

//电机数据 0x02
// if (0)
if (m_dataUpload.CarInfo.charge_state != 0x01) 
{
    vehicle::SignalVariant m_motor_state            = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVMotorSysStatus)->second;
    vehicle::SignalVariant m_motor_ctl_temp         = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVMotorCtrlTemp)->second;
    vehicle::SignalVariant m_motor_speed            = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVMotorSpeed)->second;
    vehicle::SignalVariant m_motor_high_acc_torque  = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVMotorHighAccuracyTorque)->second;
    vehicle::SignalVariant m_motor_torque           = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVMotorTorque)->second;
    vehicle::SignalVariant m_motor_temp             = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVMotorTemp)->second;
    vehicle::SignalVariant m_motor_vol              = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVMotorInputVolt)->second;
    vehicle::SignalVariant m_motor_cur              = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVMotorCurr)->second;
    vehicle::SignalVariant m_motor_direction        = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVMotorDirection)->second;

    m_dataUpload.MotorInfo.msgType                  = static_cast<SMLK_UINT8>(rtinfoID_32960::M_GB_32960_MESSAGE_MOTOR_INFO);
    m_dataUpload.MotorInfo.motor_num                = 1;
    m_dataUpload.MotorInfo.motor_seq                = 1;
    // m_dataUpload.MotorInfo.motor_state
    // 0x01 耗电：扭矩>0，电机系统状态（MCU_1）= 0x2,0x3,0x4
    // 0x02 发电：扭矩<0，电机系统状态（MCU_1）= 0x2,0x3,0x4
    // 0x03 关闭状态：电机系统状态（MCU_1）= 0x6
    // 0x04 准备状态：电机系统状态（MCU_1）= 0x0,0x1
    // 0xFE 表示异常：电机系统状态（MCU_1）= 0x5,0xf

    if (m_motor_state.Valid()) {
        if (m_motor_state.Encoded() == 0x2 || m_motor_state.Encoded() == 0x3 || m_motor_state.Encoded() == 0x4) {
            if (m_motor_torque.Valid()) {
                if (m_motor_torque.Encoded() > 125) {
                    m_dataUpload.MotorInfo.motor_state      = 0x01;
                } else if (m_motor_torque.Encoded() < 125) {
                    m_dataUpload.MotorInfo.motor_state      = 0x02;
                } else {
                    m_dataUpload.MotorInfo.motor_state      = 0xFF;
                }
            } else {
                m_dataUpload.MotorInfo.motor_state          = 0xFF;
            }
        } else if (m_motor_state.Encoded() == 0x6) {
                m_dataUpload.MotorInfo.motor_state          = 0x03;
        } else if (m_motor_state.Encoded() == 0x0 || m_motor_state.Encoded() == 0x1) {
                m_dataUpload.MotorInfo.motor_state          = 0x04;
        // } else if (m_motor_state.Encoded() == 0x5 || m_motor_state.Encoded() == 0xf) {
        } else {
                m_dataUpload.MotorInfo.motor_state          = 0xFE;
        }
    } else {
            m_dataUpload.MotorInfo.motor_state              = 0xFF;
    }       

    m_dataUpload.MotorInfo.motor_ctl_temp           = m_motor_ctl_temp.Valid() ? m_motor_ctl_temp.Encoded() : 0xFF;

    if (m_motor_direction.Valid() && m_motor_direction.Encoded() == 2) { //反转
        if (m_motor_speed.Valid()) {                
            if (m_motor_speed.Val() > 20000) {
                m_dataUpload.MotorInfo.motor_speed = m_dataLastReport.MotorInfo.motor_speed;
            } else {
                m_dataLastReport.MotorInfo.motor_speed = 20000 - m_motor_speed.Val();
                m_dataUpload.MotorInfo.motor_speed = m_dataLastReport.MotorInfo.motor_speed;
            }
        } else {
            m_dataUpload.MotorInfo.motor_speed = 0xFFFF;
        }
    } else {
        if (m_motor_speed.Valid()) {
            m_dataLastReport.MotorInfo.motor_speed = 20000 + m_motor_speed.Val();
            m_dataUpload.MotorInfo.motor_speed = m_dataLastReport.MotorInfo.motor_speed;
        } else {
                m_dataUpload.MotorInfo.motor_speed = 0xFFFF;
        }
    }

    if (m_motor_torque.Valid()) {

        int m_motor_torque_pct = 0;
        if (m_motor_torque.Encoded() >= 125) {
            m_motor_torque_pct = m_motor_torque.Encoded() - 125;
        } else {
            m_motor_torque_pct = 125 - m_motor_torque.Encoded();
        }

        m_dataUpload.MotorInfo.motor_torque = m_motor_torque_pct * common32960::getInstance()->m_refer_torque * 0.01 * 10;

        if (m_motor_high_acc_torque.Valid())
        {
            if (m_motor_high_acc_torque.Encoded() == 1)
                m_dataUpload.MotorInfo.motor_torque += common32960::getInstance()->m_refer_torque * 0.125 * 0.01 * 10;
            if (m_motor_high_acc_torque.Encoded() == 2)
                m_dataUpload.MotorInfo.motor_torque += common32960::getInstance()->m_refer_torque * 0.250 * 0.01 * 10;
            if (m_motor_high_acc_torque.Encoded() == 3)
                m_dataUpload.MotorInfo.motor_torque += common32960::getInstance()->m_refer_torque * 0.375 * 0.01 * 10;
            if (m_motor_high_acc_torque.Encoded() == 4)
                m_dataUpload.MotorInfo.motor_torque += common32960::getInstance()->m_refer_torque * 0.500 * 0.01 * 10;
            if (m_motor_high_acc_torque.Encoded() == 5)
                m_dataUpload.MotorInfo.motor_torque += common32960::getInstance()->m_refer_torque * 0.625 * 0.01 * 10;
            if (m_motor_high_acc_torque.Encoded() == 6)
                m_dataUpload.MotorInfo.motor_torque += common32960::getInstance()->m_refer_torque * 0.750 * 0.01 * 10;
            if (m_motor_high_acc_torque.Encoded() == 7)
                m_dataUpload.MotorInfo.motor_torque += common32960::getInstance()->m_refer_torque * 0.875 * 0.01 * 10;
        }

        if (m_motor_torque.Encoded() >= 125) {
            m_dataUpload.MotorInfo.motor_torque = 20000 + m_dataUpload.MotorInfo.motor_torque;
        } else  {
            if (m_dataUpload.MotorInfo.motor_torque > 20000) {
                m_dataUpload.MotorInfo.motor_torque = 0xFFFF;
            } else {
                m_dataUpload.MotorInfo.motor_torque = 20000 - m_dataUpload.MotorInfo.motor_torque;
            }
        } 
    } else {
        m_dataUpload.MotorInfo.motor_torque = 0xFFFF;
    }

    m_dataUpload.MotorInfo.motor_temp               = m_motor_temp.Valid() ? m_motor_temp.Encoded() : 0xFF;
    m_dataUpload.MotorInfo.motor_vol                = m_motor_vol.Valid() ? m_motor_vol.Encoded() : 0xFFFF;
    m_dataUpload.MotorInfo.motor_cur                = m_motor_cur.Valid() ? m_motor_cur.Encoded() : 0xFFFF;

    // SMLK_LOGD("msgType=0x%02X", m_dataUpload.MotorInfo.msgType);
    // SMLK_LOGD("motor_num=%02d", m_dataUpload.MotorInfo.motor_num);
    // SMLK_LOGD("motor_seq=%02d", m_dataUpload.MotorInfo.motor_seq);
    // SMLK_LOGD("motor_state=%02d", m_dataUpload.MotorInfo.motor_state);
    // SMLK_LOGD("motor_ctl_temp=%02d", m_dataUpload.MotorInfo.motor_ctl_temp);
    // SMLK_LOGD("motor_speed=%04X", m_dataUpload.MotorInfo.motor_speed);
    // SMLK_LOGD("motor_torque=%d", m_dataUpload.MotorInfo.motor_torque);
    // SMLK_LOGD("motor_temp=%02d", m_dataUpload.MotorInfo.motor_temp);
    // SMLK_LOGD("motor_vol=%02d", m_dataUpload.MotorInfo.motor_vol);
    // SMLK_LOGD("motor_cur=%02d", m_dataUpload.MotorInfo.motor_cur);

    buf[body_len++] = m_dataUpload.MotorInfo.msgType;
    buf[body_len++] = m_dataUpload.MotorInfo.motor_num;
    buf[body_len++] = m_dataUpload.MotorInfo.motor_seq;
    buf[body_len++] = m_dataUpload.MotorInfo.motor_state;
    buf[body_len++] = m_dataUpload.MotorInfo.motor_ctl_temp;

    EndianSwap((uint8_t *)&m_dataUpload.MotorInfo.motor_speed, 0, sizeof(m_dataUpload.MotorInfo.motor_speed));
    memcpy(&buf[body_len], &m_dataUpload.MotorInfo.motor_speed, sizeof(m_dataUpload.MotorInfo.motor_speed));
    body_len += sizeof(m_dataUpload.MotorInfo.motor_speed);

    EndianSwap((uint8_t *)&m_dataUpload.MotorInfo.motor_torque, 0, sizeof(m_dataUpload.MotorInfo.motor_torque));
    memcpy(&buf[body_len], &m_dataUpload.MotorInfo.motor_torque, sizeof(m_dataUpload.MotorInfo.motor_torque));
    body_len += sizeof(m_dataUpload.MotorInfo.motor_torque);

    buf[body_len++] = m_dataUpload.MotorInfo.motor_temp;

    EndianSwap((uint8_t *)&m_dataUpload.MotorInfo.motor_vol, 0, sizeof(m_dataUpload.MotorInfo.motor_vol));
    memcpy(&buf[body_len], &m_dataUpload.MotorInfo.motor_vol, sizeof(m_dataUpload.MotorInfo.motor_vol));
    body_len += sizeof(m_dataUpload.MotorInfo.motor_vol);

    EndianSwap((uint8_t *)&m_dataUpload.MotorInfo.motor_cur, 0, sizeof(m_dataUpload.MotorInfo.motor_cur));
    memcpy(&buf[body_len], &m_dataUpload.MotorInfo.motor_cur, sizeof(m_dataUpload.MotorInfo.motor_cur));
    body_len += sizeof(m_dataUpload.MotorInfo.motor_cur);
}

// 燃料电池数据 0x03 
// if (0)
if (common32960::getInstance()->m_run_mode == 0x02)
{
    vehicle::SignalVariant m_fuel_cell_vol                  = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVFuelCellVolt)->second;
    vehicle::SignalVariant m_fuel_cell_cur                  = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVFuelCellCurr)->second;
    vehicle::SignalVariant m_fuel_rate                      = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVFuelCellRate)->second;
    vehicle::SignalVariant m_fuel_probe_cnt                 = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVFuelCellProbeCnt)->second;
    vehicle::SignalVariant m_h_sys_high_temp                = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVHSysHighTemp)->second;
    vehicle::SignalVariant m_h_sys_high_temp_num            = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVHSysHighTempNum)->second;
    vehicle::SignalVariant m_h_sys_high_concentration       = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVHSysHighConcentration)->second;
    vehicle::SignalVariant m_h_sys_high_concentration_num   = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVHSysHighConcentrationNum)->second;
    vehicle::SignalVariant m_h_sys_high_press               = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVHSysHighPress)->second;
    vehicle::SignalVariant m_h_sys_high_press_num           = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVHSysHighPressNum)->second;
    vehicle::SignalVariant m_high_dcdc_state                = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVHighDcdcState)->second;
    vehicle::SignalVariant m_water_in_temp                  = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVWaterInTemp)->second;
    vehicle::SignalVariant m_water_out_temp                 = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVWaterOutTemp)->second;
    
    m_dataUpload.FuelcellInfo.msgType                       = static_cast<SMLK_UINT8>(rtinfoID_32960::M_GB_32960_MESSAGE_FUELCELL_INFO);
    m_dataUpload.FuelcellInfo.fuel_cell_vol                 = m_fuel_cell_vol.Valid() ? m_fuel_cell_vol.Encoded() : 0xFFFF;
    m_dataUpload.FuelcellInfo.fuel_cell_cur                 = m_fuel_cell_cur.Valid() ? m_fuel_cell_cur.Encoded() : 0xFFFF;
    m_dataUpload.FuelcellInfo.fuel_rate                     = m_fuel_rate.Valid() ? m_fuel_rate.Encoded() : 0xFFFF;
    //modify 2021.08.27 客户确认: 燃料电池里面2个温度探针, 一个叫入水温度, 一个叫出水温度
    m_dataUpload.FuelcellInfo.fuel_probe_cnt                = M_GB_32960_SZ_PROBE_CNT;
    if (m_water_in_temp.Valid() && m_water_in_temp.Val() <= 200) {
        m_dataLastReport.FuelcellInfo.probe_temp[0] = m_water_in_temp.Encoded();
    }
    m_dataUpload.FuelcellInfo.probe_temp[0] = m_dataLastReport.FuelcellInfo.probe_temp[0];      // 无 0xFF
    if (m_water_out_temp.Valid() && m_water_out_temp.Val() <= 200) {
        m_dataLastReport.FuelcellInfo.probe_temp[1] = m_water_out_temp.Encoded();
    }
    m_dataUpload.FuelcellInfo.probe_temp[1] = m_dataLastReport.FuelcellInfo.probe_temp[1];      // 无 0xFF

    m_dataUpload.FuelcellInfo.h_sys_high_temp               = m_h_sys_high_temp.Valid() ? m_h_sys_high_temp.Encoded() : 0xFFFF;
    m_dataUpload.FuelcellInfo.h_sys_high_temp_num           = m_h_sys_high_temp_num.Valid() ? m_h_sys_high_temp_num.Encoded() : 0xFF;
    m_dataUpload.FuelcellInfo.h_sys_high_concentration      = m_h_sys_high_concentration.Valid() ? m_h_sys_high_concentration.Encoded() : 0xFFFF;
    m_dataUpload.FuelcellInfo.h_sys_high_concentration_num  = m_h_sys_high_concentration_num.Valid() ? m_h_sys_high_concentration_num.Encoded() : 0xFF;

    if (m_h_sys_high_press.Valid() && (m_h_sys_high_press.Val() * 0.1) <= 100) {
        m_dataLastReport.FuelcellInfo.h_sys_high_press = m_h_sys_high_press.Val() * 0.1 * 10;
    }
    m_dataUpload.FuelcellInfo.h_sys_high_press = m_dataLastReport.FuelcellInfo.h_sys_high_press;    // 无 0xFFFF

    m_dataUpload.FuelcellInfo.h_sys_high_press_num          = m_h_sys_high_press_num.Valid() ? m_h_sys_high_press_num.Encoded() : 0xFF;

    // m_dataUpload.FuelcellInfo.high_dcdc_state 2021.08.27 客户确认: DCF系统状态与协议对应关系
    // 0x01 工作: DCF系统状态（FCS_3）= 0011, 0000, 0001, 0010
    // 0x02 断开: DCF系统状态（FCS_3）= 0100
    // 0xFE 异常: DCF系统状态（FCS_3）= 0101
    // 0xFF 无效：DCF系统状态（FCS_3）= 1111
    if (m_high_dcdc_state.Valid()) {
        if ((m_high_dcdc_state.Encoded() == 0) || (m_high_dcdc_state.Encoded() == 1) || (m_high_dcdc_state.Encoded() == 2) || (m_high_dcdc_state.Encoded() == 3)) {
            m_dataLastReport.FuelcellInfo.high_dcdc_state = 0x01;
        } else if (m_high_dcdc_state.Encoded() == 4) {
            m_dataLastReport.FuelcellInfo.high_dcdc_state = 0x02;
        } 
    } 
    m_dataUpload.FuelcellInfo.high_dcdc_state               = m_high_dcdc_state.Valid() ? m_dataLastReport.FuelcellInfo.high_dcdc_state : 0xFF;

    // SMLK_LOGD("msgType=0x%02X", m_dataUpload.FuelcellInfo.msgType);

    // SMLK_LOGD("fuel_cell_vol=%02d", m_dataUpload.FuelcellInfo.fuel_cell_vol);
    // SMLK_LOGD("fuel_cell_cur=%02d", m_dataUpload.FuelcellInfo.fuel_cell_cur);
    // SMLK_LOGD("0x03 fuel_rate=%02d", m_dataUpload.FuelcellInfo.fuel_rate);
    // SMLK_LOGD("fuel_probe_cnt=%02d", m_dataUpload.FuelcellInfo.fuel_probe_cnt);
    // SMLK_LOGD("h_sys_high_temp=%02d", m_dataUpload.FuelcellInfo.h_sys_high_temp);
    // SMLK_LOGD("h_sys_high_temp_num=%02x", m_dataUpload.FuelcellInfo.h_sys_high_temp_num);
    // SMLK_LOGD("h_sys_high_concentration=%02x", m_dataUpload.FuelcellInfo.h_sys_high_concentration);
    // SMLK_LOGD("h_sys_high_concentration_num=%02x", m_dataUpload.FuelcellInfo.h_sys_high_concentration_num);
    // SMLK_LOGD("h_sys_high_press=%d", m_dataUpload.FuelcellInfo.h_sys_high_press);
    // SMLK_LOGD("h_sys_high_press_num=%02x", m_dataUpload.FuelcellInfo.h_sys_high_press_num);
    // SMLK_LOGD("high_dcdc_state=%02d", m_dataUpload.FuelcellInfo.high_dcdc_state);
    // SMLK_LOGD("m_water_in_temp=%d", m_dataUpload.FuelcellInfo.probe_temp[0]);
    // SMLK_LOGD("m_water_out_temp=%d", m_dataUpload.FuelcellInfo.probe_temp[1]);

    buf[body_len++] = m_dataUpload.FuelcellInfo.msgType;
    EndianSwap((uint8_t *)&m_dataUpload.FuelcellInfo.fuel_cell_vol, 0, sizeof(m_dataUpload.FuelcellInfo.fuel_cell_vol));
    memcpy(&buf[body_len], &m_dataUpload.FuelcellInfo.fuel_cell_vol, sizeof(m_dataUpload.FuelcellInfo.fuel_cell_vol));
    body_len += sizeof(m_dataUpload.FuelcellInfo.fuel_cell_vol);
    EndianSwap((uint8_t *)&m_dataUpload.FuelcellInfo.fuel_cell_cur, 0, sizeof(m_dataUpload.FuelcellInfo.fuel_cell_cur));
    memcpy(&buf[body_len], &m_dataUpload.FuelcellInfo.fuel_cell_cur, sizeof(m_dataUpload.FuelcellInfo.fuel_cell_cur));
    body_len += sizeof(m_dataUpload.FuelcellInfo.fuel_cell_cur);
    EndianSwap((uint8_t *)&m_dataUpload.FuelcellInfo.fuel_rate, 0, sizeof(m_dataUpload.FuelcellInfo.fuel_rate));
    memcpy(&buf[body_len], &m_dataUpload.FuelcellInfo.fuel_rate, sizeof(m_dataUpload.FuelcellInfo.fuel_rate));
    body_len += sizeof(m_dataUpload.FuelcellInfo.fuel_rate);
    EndianSwap((uint8_t *)&m_dataUpload.FuelcellInfo.fuel_probe_cnt, 0, sizeof(m_dataUpload.FuelcellInfo.fuel_probe_cnt));
    memcpy(&buf[body_len], &m_dataUpload.FuelcellInfo.fuel_probe_cnt, sizeof(m_dataUpload.FuelcellInfo.fuel_probe_cnt));
    body_len += sizeof(m_dataUpload.FuelcellInfo.fuel_probe_cnt);
    buf[body_len++] = m_dataUpload.FuelcellInfo.probe_temp[0];
    buf[body_len++] = m_dataUpload.FuelcellInfo.probe_temp[1];
    EndianSwap((uint8_t *)&m_dataUpload.FuelcellInfo.h_sys_high_temp, 0, sizeof(m_dataUpload.FuelcellInfo.h_sys_high_temp));
    memcpy(&buf[body_len], &m_dataUpload.FuelcellInfo.h_sys_high_temp, sizeof(m_dataUpload.FuelcellInfo.h_sys_high_temp));
    body_len += sizeof(m_dataUpload.FuelcellInfo.h_sys_high_temp);
    buf[body_len++] = m_dataUpload.FuelcellInfo.h_sys_high_temp_num;
    EndianSwap((uint8_t *)&m_dataUpload.FuelcellInfo.h_sys_high_concentration, 0, sizeof(m_dataUpload.FuelcellInfo.h_sys_high_concentration));
    memcpy(&buf[body_len], &m_dataUpload.FuelcellInfo.h_sys_high_concentration, sizeof(m_dataUpload.FuelcellInfo.h_sys_high_concentration));
    body_len += sizeof(m_dataUpload.FuelcellInfo.h_sys_high_concentration);
    buf[body_len++] = m_dataUpload.FuelcellInfo.h_sys_high_concentration_num;
    EndianSwap((uint8_t *)&m_dataUpload.FuelcellInfo.h_sys_high_press, 0, sizeof(m_dataUpload.FuelcellInfo.h_sys_high_press));
    memcpy(&buf[body_len], &m_dataUpload.FuelcellInfo.h_sys_high_press, sizeof(m_dataUpload.FuelcellInfo.h_sys_high_press));
    body_len += sizeof(m_dataUpload.FuelcellInfo.h_sys_high_press);
    buf[body_len++] = m_dataUpload.FuelcellInfo.h_sys_high_press_num;
    buf[body_len++] = m_dataUpload.FuelcellInfo.high_dcdc_state;
}

// 发动机数据 0x04
// if (0)
if ((common32960::getInstance()->m_run_mode == 0x03) && (m_dataUpload.CarInfo.charge_state != 0x01)) 
{
    vehicle::SignalVariant m_crankshaft_speed       = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EngineSpeed)->second;
    vehicle::SignalVariant m_inst_fuel_rate         = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EngineInstantaneousFuelEconomy)->second;

    m_dataUpload.EngInfo.msgType                    = static_cast<SMLK_UINT8>(rtinfoID_32960::M_GB_32960_MESSAGE_ENGINE_INFO);

    // m_dataUpload.EngInfo.engine_state
    // 0x01 启动: Engine speed超过300r/min
    // 0x02 关闭: 判断IGOFF信号，则认为关闭状态
    if (common32960::getInstance()->getIgnState() == 0 || common32960::getInstance()->getIgnState() == 2) {
        m_dataUpload.EngInfo.engine_state = 0x02;
    } else {
        if (m_crankshaft_speed.Valid()) {
            if (m_crankshaft_speed.Encoded() > 300 ) {
                m_dataUpload.EngInfo.engine_state = 0x01;
            } else {
                m_dataUpload.EngInfo.engine_state = 0x02;
            }
        } else {
            m_dataUpload.EngInfo.engine_state = 0xFF;
        }
    }
    // m_dataUpload.EngInfo.crankshaft_speed = m_crankshaft_speed.Valid() ? (m_crankshaft_speed.Encoded() * 512) : 0xFFFF;
    m_dataUpload.EngInfo.crankshaft_speed = m_crankshaft_speed.Valid() ? m_crankshaft_speed.Encoded() : 0xFFFF;

    // EMS 发送 Instantaneous Fuel Economy 信号（单位 km/L），T-box 对其进行取倒数操作可得 L/km 单位的信号，再进行相应的倍数和精度转换
    int tmp = 0;
    if (m_inst_fuel_rate.Valid()) {
        if (m_inst_fuel_rate.Val() != 0) {
            int tmp_fuel_rate = 1.00 * 100 * 100 / m_inst_fuel_rate.Val();
            if (tmp_fuel_rate >= 0 && tmp_fuel_rate <= 60000) {
                m_dataLastReport.EngInfo.fuel_rate = tmp_fuel_rate;
            }
        } else {
            m_dataLastReport.EngInfo.fuel_rate = 0;
        }
        m_dataUpload.EngInfo.fuel_rate = m_dataLastReport.EngInfo.fuel_rate;
    } else {
        m_dataUpload.EngInfo.fuel_rate = 0xFFFF;
    }

    // SMLK_LOGD("msgType=0x%02X", m_dataUpload.EngInfo.msgType);
    // SMLK_LOGD("engine_state=%02d", m_dataUpload.EngInfo.engine_state);
    // SMLK_LOGD("crankshaft_speed=%02d", m_dataUpload.EngInfo.crankshaft_speed);
    // SMLK_LOGD("0x04 fuel_rate=%02d", m_dataUpload.EngInfo.fuel_rate);

    buf[body_len++] = m_dataUpload.EngInfo.msgType;
    buf[body_len++] = m_dataUpload.EngInfo.engine_state;
    EndianSwap((uint8_t *)&m_dataUpload.EngInfo.crankshaft_speed, 0, sizeof(m_dataUpload.EngInfo.crankshaft_speed));
    memcpy(&buf[body_len], &m_dataUpload.EngInfo.crankshaft_speed, sizeof(m_dataUpload.EngInfo.crankshaft_speed));
    body_len += sizeof(m_dataUpload.EngInfo.crankshaft_speed);
    EndianSwap((uint8_t *)&m_dataUpload.EngInfo.fuel_rate, 0, sizeof(m_dataUpload.EngInfo.fuel_rate));
    memcpy(&buf[body_len], &m_dataUpload.EngInfo.fuel_rate, sizeof(m_dataUpload.EngInfo.fuel_rate));
    body_len += sizeof(m_dataUpload.EngInfo.fuel_rate);
}

//位置信息 0x05
// if (0)
{
    vehicle::SignalVariant m_located_status         = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::LocatedStatus)->second;
    vehicle::SignalVariant m_longitude              = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::Longitude)->second;
    vehicle::SignalVariant m_latitude               = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::Latitude)->second;

    m_dataUpload.PosInfo.msgType                    = static_cast<SMLK_UINT8>(rtinfoID_32960::M_GB_32960_MESSAGE_LOCATION_INFO);
    m_dataUpload.PosInfo.pos_state                  = m_located_status.Valid() ? m_located_status.Encoded() : 0xFF;
    m_dataUpload.PosInfo.longitude                  = m_longitude.Valid() ? m_longitude.Encoded() : 0xFFFFFFFF;
    m_dataUpload.PosInfo.latitude                   = m_latitude.Valid() ? m_latitude.Encoded() : 0xFFFFFFFF;

    // SMLK_LOGD("msgType=0x%02X", m_dataUpload.PosInfo.msgType);

    buf[body_len++] = m_dataUpload.PosInfo.msgType;
    buf[body_len++] = m_dataUpload.PosInfo.pos_state;
    EndianSwap((uint8_t *)&m_dataUpload.PosInfo.longitude, 0, sizeof(m_dataUpload.PosInfo.longitude));
    memcpy(&buf[body_len], &m_dataUpload.PosInfo.longitude, sizeof(m_dataUpload.PosInfo.longitude));
    body_len += sizeof(m_dataUpload.PosInfo.longitude);
    EndianSwap((uint8_t *)&m_dataUpload.PosInfo.latitude, 0, sizeof(m_dataUpload.PosInfo.latitude));
    memcpy(&buf[body_len], &m_dataUpload.PosInfo.latitude, sizeof(m_dataUpload.PosInfo.latitude));
    body_len += sizeof(m_dataUpload.PosInfo.latitude);
}

// 极值数据 告警数据 可充电存能装置电压数据
// if (0)
{
    vehicle::SignalVariant m_max_volt_cell_subsys_seq = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVMaxVoltCellSubsysSeq)->second;
    vehicle::SignalVariant m_max_volt_sigle_cell_seq = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVMaxVoltSigleCellSeq)->second;
    vehicle::SignalVariant m_max_volt_sigle_cell_value = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVMaxVoltSigleCellValue)->second;
    vehicle::SignalVariant m_min_volt_cell_subsys_seq = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVMinVoltCellSubsysSeq)->second;
    vehicle::SignalVariant m_min_volt_sigle_cell_seq = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVMinVoltSigleCellSeq)->second;
    vehicle::SignalVariant m_min_volt_sigle_cell_value = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVMinVoltSigleCellValue)->second;
    vehicle::SignalVariant m_max_temp_subsys_seq = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVMaxTempSubsysSeq)->second;
    vehicle::SignalVariant m_max_temp_probe_seq = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVMaxTempProbeSeq)->second;
    vehicle::SignalVariant m_max_temp_value = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVMaxTempValue)->second;
    vehicle::SignalVariant m_min_temp_subsys_seq = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVMinTempSubsysSeq)->second;
    vehicle::SignalVariant m_min_temp_probe_seq = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVMinTempProbeSeq)->second;
    vehicle::SignalVariant m_min_temp_value = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVMinTempValue)->second;

    vehicle::SignalVariant m_energy_store_subsys_volt = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVPowerBatteryVoltage)->second;
    vehicle::SignalVariant m_energy_store_subsys_curr = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVPowerBatteryCurrent)->second;
    vehicle::SignalVariant m_energy_store_subsys_cell_num = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVEnergyStorageCellNum)->second;

    GB_32960_RechageDeviceVoltInfo m_volt_info;
    memset(&m_volt_info, 0, sizeof(m_volt_info));
    m_volt_info.energy_device_seq = M_GB_32960_SZ_RECHARGE_DEVICE_SUBSYS_CNT;

    if (m_energy_store_subsys_volt.Valid()) {
        m_last_volt_info.energy_device_volt = m_energy_store_subsys_volt.Encoded();
    }
    m_volt_info.energy_device_volt = m_last_volt_info.energy_device_volt;

    if (m_energy_store_subsys_curr.Valid()) {
        m_last_volt_info.energy_device_curr = m_energy_store_subsys_curr.Encoded();
    }
    m_volt_info.energy_device_curr = m_last_volt_info.energy_device_curr;

    if (m_energy_store_subsys_cell_num.Valid()) {
        m_last_volt_info.cell_num = m_energy_store_subsys_cell_num.Encoded();
    }
    m_volt_info.cell_num = m_last_volt_info.cell_num;
    int m_cell_num = m_volt_info.cell_num; // 单体电池个数

    m_volt_info.begin_seq = 0x01;
    m_volt_info.sigle_cell_list_num = m_volt_info.cell_num > M_GB_32960_SZ_SIGLE_CELL_CNT_MAX ? M_GB_32960_SZ_SIGLE_CELL_CNT_MAX : m_volt_info.cell_num;

    // SMLK_LOGD("m_volt_info.cell_num = %d", m_volt_info.cell_num);
    // SMLK_LOGD("m_volt_info.begin_seq = %d", m_volt_info.begin_seq);
    // SMLK_LOGD("m_volt_info.sigle_cell_list_num = %d", m_volt_info.sigle_cell_list_num);

    // EndianSwap((uint8_t *)&(m_volt_info.energy_device_volt), 0, sizeof(m_volt_info.energy_device_volt));
    // EndianSwap((uint8_t *)&(m_volt_info.energy_device_curr), 0, sizeof(m_volt_info.energy_device_curr));
    // EndianSwap((uint8_t *)&(m_volt_info.cell_num), 0, sizeof(m_volt_info.cell_num));

    //补齐电压值
    std::vector<SMLK_UINT8> m_sigle_bat_vol(2 * m_cell_num, 0xFF);
    if (m_sigle_bat_vol.size() > common32960::getInstance()->m_voltinfo.mdata.size()) {
        for (int m = 0; m < common32960::getInstance()->m_voltinfo.mdata.size(); m++) {
            m_sigle_bat_vol[m] = common32960::getInstance()->m_voltinfo.mdata[m];
        }
    } else {
        for (int m = 0; m < m_sigle_bat_vol.size(); m++) {
            m_sigle_bat_vol[m] = common32960::getInstance()->m_voltinfo.mdata[m];
        }
    }

    // for (int x = 0; x < m_sigle_bat_vol.size(); x++) {
    //         printf("0x%02x, ", m_sigle_bat_vol[x]);
    // }
    // printf("33===== \n");

    // 01帧极值
    // SMLK_LOGD("volbat_subsys_seq = %d ~ %d", m_volt_info.begin_seq, m_volt_info.begin_seq + m_volt_info.sigle_cell_list_num - 1);

    m_dataUpload.LimiInfo.msgType = static_cast<SMLK_UINT8>(rtinfoID_32960::M_GB_32960_MESSAGE_EXTREME_VALUES);
    
    if (m_max_volt_cell_subsys_seq.Valid()) {
        m_dataLastReport.LimiInfo.max_volbat_subsys_seq = m_max_volt_cell_subsys_seq.Encoded();
    }
    m_dataUpload.LimiInfo.max_volbat_subsys_seq = m_dataLastReport.LimiInfo.max_volbat_subsys_seq;

    if (m_max_volt_sigle_cell_seq.Valid()) {
        m_dataLastReport.LimiInfo.max_volbat_sigle_seq = m_max_volt_sigle_cell_seq.Encoded();
    }
    m_dataUpload.LimiInfo.max_volbat_sigle_seq = m_dataLastReport.LimiInfo.max_volbat_sigle_seq;

    if (m_max_volt_sigle_cell_value.Valid()) {
        m_dataLastReport.LimiInfo.max_sigle_bat_vol = m_max_volt_sigle_cell_value.Encoded();
    }
    m_dataUpload.LimiInfo.max_sigle_bat_vol = m_dataLastReport.LimiInfo.max_sigle_bat_vol;

    if (m_dataUpload.LimiInfo.max_volbat_sigle_seq != 0xFF) {
        if (m_dataUpload.LimiInfo.max_volbat_sigle_seq <= m_volt_info.begin_seq || m_dataUpload.LimiInfo.max_volbat_sigle_seq >= (m_volt_info.begin_seq + m_volt_info.sigle_cell_list_num - 1)) {
            m_dataUpload.LimiInfo.max_volbat_sigle_seq = 0xFF;
            m_dataUpload.LimiInfo.max_sigle_bat_vol = 0xFFFF;
        }
    }

    if (m_min_volt_cell_subsys_seq.Valid()) {
        m_dataLastReport.LimiInfo.min_volbat_subsys_seq = m_min_volt_cell_subsys_seq.Encoded();
    }
    m_dataUpload.LimiInfo.min_volbat_subsys_seq = m_dataLastReport.LimiInfo.min_volbat_subsys_seq;

    if (m_min_volt_sigle_cell_seq.Valid()) {
        m_dataLastReport.LimiInfo.min_volbat_sigle_seq = m_min_volt_sigle_cell_seq.Encoded();
    }
    m_dataUpload.LimiInfo.min_volbat_sigle_seq = m_dataLastReport.LimiInfo.min_volbat_sigle_seq;

    if (m_min_volt_sigle_cell_value.Valid()) {
        m_dataLastReport.LimiInfo.min_sigle_bat_vol = m_min_volt_sigle_cell_value.Encoded();
    }
    m_dataUpload.LimiInfo.min_sigle_bat_vol = m_dataLastReport.LimiInfo.min_sigle_bat_vol;

    if (m_dataUpload.LimiInfo.min_volbat_sigle_seq != 0xFF) {
        if (m_dataUpload.LimiInfo.min_volbat_sigle_seq <= m_volt_info.begin_seq || m_dataUpload.LimiInfo.min_volbat_sigle_seq >= (m_volt_info.begin_seq + m_volt_info.sigle_cell_list_num - 1)) {
            m_dataUpload.LimiInfo.min_volbat_sigle_seq = 0xFF;
            m_dataUpload.LimiInfo.min_sigle_bat_vol = 0xFFFF;
        }
    }
    
    if (m_max_temp_subsys_seq.Valid()) {
        m_dataLastReport.LimiInfo.max_temp_subsys_seq = m_max_temp_subsys_seq.Encoded();
    }
    m_dataUpload.LimiInfo.max_temp_subsys_seq = m_dataLastReport.LimiInfo.max_temp_subsys_seq;
    if (m_max_temp_probe_seq.Valid()) {
        m_dataLastReport.LimiInfo.max_temp_probe_seq =m_max_temp_probe_seq.Encoded();
    }
    m_dataUpload.LimiInfo.max_temp_probe_seq = m_dataLastReport.LimiInfo.max_temp_probe_seq;
    if (m_max_temp_value.Valid()) {
        m_dataLastReport.LimiInfo.max_temp = m_max_temp_value.Encoded();
    }
    m_dataUpload.LimiInfo.max_temp = m_dataLastReport.LimiInfo.max_temp;
    if (m_min_temp_subsys_seq.Valid()) {
        m_dataLastReport.LimiInfo.min_temp_subsys_seq =m_min_temp_subsys_seq.Encoded();
    }
    m_dataUpload.LimiInfo.min_temp_subsys_seq = m_dataLastReport.LimiInfo.min_temp_subsys_seq;
    if (m_min_temp_probe_seq.Valid()) {
        m_dataLastReport.LimiInfo.min_temp_probe_seq =m_min_temp_probe_seq.Encoded();
    }
    m_dataUpload.LimiInfo.min_temp_probe_seq = m_dataLastReport.LimiInfo.min_temp_probe_seq;
    if (m_min_temp_value.Valid()) {
        m_dataLastReport.LimiInfo.min_temp = m_min_temp_value.Encoded();
    }
    m_dataUpload.LimiInfo.min_temp = m_dataLastReport.LimiInfo.min_temp;

    // SMLK_LOGD("msgType=0x%02X", m_dataUpload.LimiInfo.msgType);
    // SMLK_LOGD("max_volbat_subsys_seq=%d", m_dataUpload.LimiInfo.max_volbat_subsys_seq);
    // SMLK_LOGD("max_volbat_sigle_seq=%d", m_dataUpload.LimiInfo.max_volbat_sigle_seq);
    // SMLK_LOGD("max_sigle_bat_vol=%d", m_dataUpload.LimiInfo.max_sigle_bat_vol);
    // SMLK_LOGD("min_volbat_subsys_seq=%d", m_dataUpload.LimiInfo.min_volbat_subsys_seq);
    // SMLK_LOGD("min_volbat_sigle_seq=%d", m_dataUpload.LimiInfo.min_volbat_sigle_seq);
    // SMLK_LOGD("min_sigle_bat_vol=%d", m_dataUpload.LimiInfo.min_sigle_bat_vol);
    // SMLK_LOGD("max_temp_subsys_seq=%02d", m_dataUpload.LimiInfo.max_temp_subsys_seq);
    // SMLK_LOGD("max_temp_probe_seq=%02d", m_dataUpload.LimiInfo.max_temp_probe_seq);
    // SMLK_LOGD("max_temp=%02d", m_dataUpload.LimiInfo.max_temp);
    // SMLK_LOGD("min_temp_subsys_seq=%02d", m_dataUpload.LimiInfo.min_temp_subsys_seq);
    // SMLK_LOGD("min_temp_probe_seq=%02d", m_dataUpload.LimiInfo.min_temp_probe_seq);
    // SMLK_LOGD("min_temp=%02d", m_dataUpload.LimiInfo.min_temp);

    buf[body_len++] = m_dataUpload.LimiInfo.msgType;
    buf[body_len++] = m_dataUpload.LimiInfo.max_volbat_subsys_seq;
    buf[body_len++] = m_dataUpload.LimiInfo.max_volbat_sigle_seq;

    EndianSwap((uint8_t *)&m_dataUpload.LimiInfo.max_sigle_bat_vol, 0, sizeof(m_dataUpload.LimiInfo.max_sigle_bat_vol));
    memcpy(&buf[body_len], &m_dataUpload.LimiInfo.max_sigle_bat_vol, sizeof(m_dataUpload.LimiInfo.max_sigle_bat_vol));
    body_len += sizeof(m_dataUpload.LimiInfo.max_sigle_bat_vol);

    buf[body_len++] = m_dataUpload.LimiInfo.min_volbat_subsys_seq;
    buf[body_len++] = m_dataUpload.LimiInfo.min_volbat_sigle_seq;

    EndianSwap((uint8_t *)&m_dataUpload.LimiInfo.min_sigle_bat_vol, 0, sizeof(m_dataUpload.LimiInfo.min_sigle_bat_vol));
    memcpy(&buf[body_len], &m_dataUpload.LimiInfo.min_sigle_bat_vol, sizeof(m_dataUpload.LimiInfo.min_sigle_bat_vol));
    body_len += sizeof(m_dataUpload.LimiInfo.min_sigle_bat_vol);

    buf[body_len++] = m_dataUpload.LimiInfo.max_temp_subsys_seq;
    buf[body_len++] = (SMLK_UINT8)m_dataUpload.LimiInfo.max_temp_probe_seq;
    buf[body_len++] = (SMLK_UINT8)m_dataUpload.LimiInfo.max_temp;
    buf[body_len++] = m_dataUpload.LimiInfo.min_temp_subsys_seq;
    buf[body_len++] = (SMLK_UINT8)m_dataUpload.LimiInfo.min_temp_probe_seq;
    buf[body_len++] = (SMLK_UINT8)m_dataUpload.LimiInfo.min_temp;

    // 报警
    SMLK_UINT8 max_warn;
    SMLK_UINT32 common_warn;
    m_dataUpload.warnInfo.msgType = static_cast<SMLK_UINT8>(rtinfoID_32960::M_GB_32960_MESSAGE_ALERT_INFO);
    max_warn = common32960::getInstance()->m_dm1info.max_warn;
    common_warn = common32960::getInstance()->m_dm1info.common_warn;
    m_dataUpload.warnInfo.batt_warn_num = common32960::getInstance()->m_dm1info.warn_num;   //全部当成电池故障处理
    m_dataUpload.warnInfo.divmotor_warn_num = 0;
    m_dataUpload.warnInfo.engmotor_warn_num = 0;
    m_dataUpload.warnInfo.other_warn_num = 0;

    // SMLK_LOGD("msgType=0x%02X", m_dataUpload.warnInfo.msgType);
    // SMLK_LOGD("max_warn=%02d, common_warn=%02d, batt_warn_num=%02d", max_warn, common_warn, m_dataUpload.warnInfo.batt_warn_num);
    // SMLK_LOGD("divmotor_warn=%02d, engmotor_warn_num=%02d, other_warn_num=%02d"
    //         , m_dataUpload.warnInfo.divmotor_warn_num, m_dataUpload.warnInfo.engmotor_warn_num, m_dataUpload.warnInfo.other_warn_num);

    m_dataUpload.warnInfo.common_warn = m_last_common_warn;

    if (m_dataUpload.warnInfo.max_warn != max_warn) {
        SMLK_LOGD("m_dataUpload.warnInfo.max_warn last =%d, max_warn now =%d", m_dataUpload.warnInfo.max_warn, max_warn);
        m_dataUpload.warnInfo.max_warn = max_warn;
        m_dataUpload.warnInfo.common_warn = common_warn;
        if (m_dataUpload.warnInfo.max_warn == 0x03) {
            if (!common32960::getInstance()->getThreeWarnFlag()) {
                common32960::getInstance()->setThreeWarnFlag(true);
                common32960::getInstance()->setInfoUpTime32960(1); //实时上报每秒1次
            }
        }
    } else { //一直重复发送同一级别告警，同一3级警告只处理一次
        if (m_dataUpload.warnInfo.max_warn == 0x03) {
            SMLK_LOGD("m_dataUpload.warnInfo.common_warn last =%d, common_warn now =%d", m_dataUpload.warnInfo.common_warn, common_warn);
            if (m_dataUpload.warnInfo.common_warn != common_warn) { //触发了不同的3级告警
                m_dataUpload.warnInfo.common_warn = common_warn;
                if (!common32960::getInstance()->getThreeWarnFlag()) {
                    common32960::getInstance()->setThreeWarnFlag(true);
                    common32960::getInstance()->setInfoUpTime32960(1); //实时上报每秒1次
                }
            }
        } else if (m_dataUpload.warnInfo.common_warn != common_warn) {
            m_dataUpload.warnInfo.common_warn = common_warn;
        }
    }

    m_last_common_warn = m_dataUpload.warnInfo.common_warn;

    // SMLK_LOGD("msgType=0x%02X", m_dataUpload.warnInfo.msgType);

    buf[body_len++] = m_dataUpload.warnInfo.msgType;
    buf[body_len++] = m_dataUpload.warnInfo.max_warn;

    EndianSwap((uint8_t *)&m_dataUpload.warnInfo.common_warn, 0, sizeof(m_dataUpload.warnInfo.common_warn));
    memcpy(&buf[body_len], &m_dataUpload.warnInfo.common_warn, sizeof(m_dataUpload.warnInfo.common_warn));
    body_len += sizeof(m_dataUpload.warnInfo.common_warn);

    buf[body_len++] = m_dataUpload.warnInfo.batt_warn_num;

    SMLK_UINT32 error_code;
    for (int i = 0; i < 19; i++)
    {
        error_code = 0x00000000;
        if (common32960::getInstance()->m_dm1info.dm1_event[i].errcode != 0)
        {
            error_code = common32960::getInstance()->m_dm1info.dm1_event[i].errvalue;
            // if (error_code != 0x00000000) {
            //     SMLK_LOGD("1.error_code=%02d", error_code);
            // }
            EndianSwap((uint8_t *)&error_code, 0, sizeof(error_code));
            memcpy(&buf[body_len], &error_code, sizeof(error_code));
            body_len += sizeof(error_code);
        }
    }

    buf[body_len++] = m_dataUpload.warnInfo.divmotor_warn_num;
    buf[body_len++] = m_dataUpload.warnInfo.engmotor_warn_num;
    buf[body_len++] = m_dataUpload.warnInfo.other_warn_num;

    // 单体电池电压
    m_dataUpload.VolInfo.msgType = static_cast<SMLK_UINT8>(rtinfoID_32960::M_GB_32960_MESSAGE_RECHARGE_DEVICE_VOLT_INFO);
    m_dataUpload.VolInfo.energy_store_subsys_num = M_GB_32960_SZ_RECHARGE_DEVICE_SUBSYS_CNT;
    buf[body_len++] = m_dataUpload.VolInfo.msgType;
    buf[body_len++] = m_dataUpload.VolInfo.energy_store_subsys_num;

    // SMLK_LOGD("msgType=0x%02X", m_dataUpload.VolInfo.msgType);
    // SMLK_LOGD("m_dataUpload.VolInfo.energy_store_subsys_num=%02d", m_dataUpload.VolInfo.energy_store_subsys_num);

    // 01帧单体电池电压
    int m_begin_seq = m_volt_info.begin_seq;
    int m_sigle_cell_list_num = m_volt_info.sigle_cell_list_num;

    //电池子系统号
    buf[body_len++] = m_volt_info.energy_device_seq;
    //总电压
    EndianSwap((uint8_t *)&(m_volt_info.energy_device_volt), 0, sizeof(m_volt_info.energy_device_volt));
    memcpy(&buf[body_len], &(m_volt_info.energy_device_volt), sizeof(m_volt_info.energy_device_volt));
    body_len += sizeof(m_volt_info.energy_device_volt);
    //总电流
    EndianSwap((uint8_t *)&(m_volt_info.energy_device_curr), 0, sizeof(m_volt_info.energy_device_curr));
    memcpy(&buf[body_len], &(m_volt_info.energy_device_curr), sizeof(m_volt_info.energy_device_curr));
    body_len += sizeof(m_volt_info.energy_device_curr);
    //单体电池总数
    EndianSwap((uint8_t *)&(m_volt_info.cell_num), 0, sizeof(m_volt_info.cell_num));
    memcpy(&buf[body_len], &(m_volt_info.cell_num), sizeof(m_volt_info.cell_num));
    body_len += sizeof(m_volt_info.cell_num);
    //本帧单体电池起始序号
    // SMLK_LOGD("begin_seq=%02d", m_volt_info.begin_seq);
    EndianSwap((uint8_t *)&(m_volt_info.begin_seq), 0, sizeof(m_volt_info.begin_seq));
    memcpy(&buf[body_len], &(m_volt_info.begin_seq), sizeof(m_volt_info.begin_seq));
    body_len += sizeof(m_volt_info.begin_seq);
    //本帧单体电池总数
    // SMLK_LOGD("sigle_cell_list_num=%02d", m_volt_info.sigle_cell_list_num);
    buf[body_len++] = m_volt_info.sigle_cell_list_num;

    for (int i = 0; i < m_sigle_cell_list_num; i++) {
        // SMLK_LOGD("m_begin_seq=%02x", m_begin_seq);
        // SMLK_LOGD("m_sigle_bat_vol[%d]=%02x", 2 * i + 2 * (m_begin_seq - 1), m_sigle_bat_vol[2 * i + 2 * (m_begin_seq - 1)]);
        // SMLK_LOGD("m_sigle_bat_vol[%d]=%02x", 2 * i + 1 + 2 * (m_begin_seq - 1), m_sigle_bat_vol[2 * i + 1 + 2 * (m_begin_seq - 1)]);
        buf[body_len++] = m_sigle_bat_vol[2 * i + 1 + 2 * (m_begin_seq - 1)];
        buf[body_len++] = m_sigle_bat_vol[2 * i + 2 * (m_begin_seq - 1)];
    }

    // 多帧 编码
    int m_cell_frame_num = 0; // 分包个数

    if (m_cell_num > M_GB_32960_SZ_SIGLE_CELL_CNT_MAX) {
        if (m_cell_num % M_GB_32960_SZ_SIGLE_CELL_CNT_MAX) {
            m_cell_frame_num = m_cell_num / M_GB_32960_SZ_SIGLE_CELL_CNT_MAX;
            m_cell_frame_num++;
        } else {
            m_cell_frame_num = m_cell_num / M_GB_32960_SZ_SIGLE_CELL_CNT_MAX;
        }

        m_cell_frame_num = m_cell_frame_num - 1;

        if (m_cell_frame_num > 0) {
            for (int i = 0; i < m_cell_frame_num; i++) {
                SMLK_UINT8 buf_vlot_cache[1024] = {0};
                int body_vlot_cache_len = 0;

                GB_32960_RechageDeviceVoltInfo m_volt_info_cache;
                memset(&m_volt_info_cache, 0, sizeof(m_volt_info_cache));

                if (i + 1 < m_cell_frame_num) {
                    m_volt_info_cache.begin_seq = i * M_GB_32960_SZ_SIGLE_CELL_CNT_MAX + M_GB_32960_SZ_SIGLE_CELL_CNT_MAX + 1;
                    m_volt_info_cache.sigle_cell_list_num = M_GB_32960_SZ_SIGLE_CELL_CNT_MAX;
                } else if (i + 1 == m_cell_frame_num) {
                    m_volt_info_cache.begin_seq = i * M_GB_32960_SZ_SIGLE_CELL_CNT_MAX + M_GB_32960_SZ_SIGLE_CELL_CNT_MAX + 1;
                    m_volt_info_cache.sigle_cell_list_num = m_cell_num - M_GB_32960_SZ_SIGLE_CELL_CNT_MAX - i * M_GB_32960_SZ_SIGLE_CELL_CNT_MAX;
                }

                memcpy(buf_vlot_cache, m_dataUpload.sendTime, sizeof(m_dataUpload.sendTime));
                body_vlot_cache_len += sizeof(m_dataUpload.sendTime);

                GB_32960_ExtremeInfoStream m_extreme_info_cache;
                memset(&m_extreme_info_cache, 0, sizeof(m_extreme_info_cache));

                // SMLK_LOGD("volbat_subsys_seq = %d ~ %d", m_volt_info_cache.begin_seq, m_volt_info_cache.begin_seq + m_volt_info_cache.sigle_cell_list_num - 1);
                m_extreme_info_cache.msgType = static_cast<SMLK_UINT8>(rtinfoID_32960::M_GB_32960_MESSAGE_EXTREME_VALUES);

                m_extreme_info_cache.max_volbat_subsys_seq = m_dataLastReport.LimiInfo.max_volbat_subsys_seq;
                m_extreme_info_cache.max_volbat_sigle_seq = m_dataLastReport.LimiInfo.max_volbat_sigle_seq;
                m_extreme_info_cache.max_sigle_bat_vol = m_dataLastReport.LimiInfo.max_sigle_bat_vol;

                if (m_extreme_info_cache.max_volbat_sigle_seq != 0xFF) {
                    if (m_extreme_info_cache.max_volbat_sigle_seq <= m_volt_info_cache.begin_seq || m_extreme_info_cache.max_volbat_sigle_seq >= (m_volt_info_cache.begin_seq + m_volt_info_cache.sigle_cell_list_num - 1)) {
                        m_extreme_info_cache.max_volbat_sigle_seq = 0xFF;
                        m_extreme_info_cache.max_sigle_bat_vol = 0xFFFF;
                    }
                    else {
                        m_extreme_info_cache.max_volbat_sigle_seq -= (i+1) * M_GB_32960_SZ_SIGLE_CELL_CNT_MAX;
                    }
                }

                m_extreme_info_cache.min_volbat_subsys_seq = m_dataLastReport.LimiInfo.min_volbat_subsys_seq;
                m_extreme_info_cache.min_volbat_sigle_seq = m_dataLastReport.LimiInfo.min_volbat_sigle_seq;
                m_extreme_info_cache.min_sigle_bat_vol = m_dataLastReport.LimiInfo.min_sigle_bat_vol;

                if (m_extreme_info_cache.min_volbat_sigle_seq != 0xFF) {
                    if (m_extreme_info_cache.min_volbat_sigle_seq <= m_volt_info_cache.begin_seq || m_extreme_info_cache.min_volbat_sigle_seq >= (m_volt_info_cache.begin_seq + m_volt_info_cache.sigle_cell_list_num - 1)) {
                        m_extreme_info_cache.min_volbat_sigle_seq = 0xFF;
                        m_extreme_info_cache.min_sigle_bat_vol = 0xFFFF;
                    }
                    else {
                        m_extreme_info_cache.min_volbat_sigle_seq -= (i+1) * M_GB_32960_SZ_SIGLE_CELL_CNT_MAX;
                    }
                }

                m_extreme_info_cache.max_temp_subsys_seq = m_dataLastReport.LimiInfo.max_temp_subsys_seq;
                m_extreme_info_cache.max_temp_probe_seq = m_dataLastReport.LimiInfo.max_temp_probe_seq;
                m_extreme_info_cache.max_temp = m_dataLastReport.LimiInfo.max_temp;
                m_extreme_info_cache.min_temp_subsys_seq = m_dataLastReport.LimiInfo.min_temp_subsys_seq;
                m_extreme_info_cache.min_temp_probe_seq = m_dataLastReport.LimiInfo.min_temp_probe_seq;
                m_extreme_info_cache.min_temp = m_dataLastReport.LimiInfo.min_temp;

                // SMLK_LOGD("msgType=0x%02X", m_extreme_info_cache.msgType);
                // SMLK_LOGD("max_volbat_subsys_seq=%d", m_extreme_info_cache.max_volbat_subsys_seq);
                // SMLK_LOGD("max_volbat_sigle_seq=%d", m_extreme_info_cache.max_volbat_sigle_seq);
                // SMLK_LOGD("max_sigle_bat_vol=%d", m_extreme_info_cache.max_sigle_bat_vol);
                // SMLK_LOGD("min_volbat_subsys_seq=%d", m_extreme_info_cache.min_volbat_subsys_seq);
                // SMLK_LOGD("min_volbat_sigle_seq=%d", m_extreme_info_cache.min_volbat_sigle_seq);
                // SMLK_LOGD("min_sigle_bat_vol=%d", m_extreme_info_cache.min_sigle_bat_vol);
                // SMLK_LOGD("max_temp_subsys_seq=%02d", m_extreme_info_cache.max_temp_subsys_seq);
                // SMLK_LOGD("max_temp_probe_seq=%02d", m_extreme_info_cache.max_temp_probe_seq);
                // SMLK_LOGD("max_temp=%02d", m_extreme_info_cache.max_temp);
                // SMLK_LOGD("min_temp_subsys_seq=%02d", m_extreme_info_cache.min_temp_subsys_seq);
                // SMLK_LOGD("min_temp_probe_seq=%02d", m_extreme_info_cache.min_temp_probe_seq);
                // SMLK_LOGD("min_temp=%02d", m_extreme_info_cache.min_temp);

                buf_vlot_cache[body_vlot_cache_len++] = m_extreme_info_cache.msgType;
                buf_vlot_cache[body_vlot_cache_len++] = m_extreme_info_cache.max_volbat_subsys_seq;
                buf_vlot_cache[body_vlot_cache_len++] = m_extreme_info_cache.max_volbat_sigle_seq;

                EndianSwap((uint8_t *)&m_extreme_info_cache.max_sigle_bat_vol, 0, sizeof(m_extreme_info_cache.max_sigle_bat_vol));
                memcpy(&buf_vlot_cache[body_vlot_cache_len], &m_extreme_info_cache.max_sigle_bat_vol, sizeof(m_extreme_info_cache.max_sigle_bat_vol));
                body_vlot_cache_len += sizeof(m_extreme_info_cache.max_sigle_bat_vol);

                buf_vlot_cache[body_vlot_cache_len++] = m_extreme_info_cache.min_volbat_subsys_seq;
                buf_vlot_cache[body_vlot_cache_len++] = m_extreme_info_cache.min_volbat_sigle_seq;

                EndianSwap((uint8_t *)&m_extreme_info_cache.min_sigle_bat_vol, 0, sizeof(m_extreme_info_cache.min_sigle_bat_vol));
                memcpy(&buf_vlot_cache[body_vlot_cache_len], &m_extreme_info_cache.min_sigle_bat_vol, sizeof(m_extreme_info_cache.min_sigle_bat_vol));
                body_vlot_cache_len += sizeof(m_extreme_info_cache.min_sigle_bat_vol);

                buf_vlot_cache[body_vlot_cache_len++] = m_extreme_info_cache.max_temp_subsys_seq;
                buf_vlot_cache[body_vlot_cache_len++] = (SMLK_UINT8)m_extreme_info_cache.max_temp_probe_seq;
                buf_vlot_cache[body_vlot_cache_len++] = (SMLK_UINT8)m_extreme_info_cache.max_temp;
                buf_vlot_cache[body_vlot_cache_len++] = m_extreme_info_cache.min_temp_subsys_seq;
                buf_vlot_cache[body_vlot_cache_len++] = (SMLK_UINT8)m_extreme_info_cache.min_temp_probe_seq;
                buf_vlot_cache[body_vlot_cache_len++] = (SMLK_UINT8)m_extreme_info_cache.min_temp;

                GB_32960_RechageDeviceVoltInfoStream m_volt_info_stream_cache;

                m_volt_info_stream_cache.msgType = static_cast<SMLK_UINT8>(rtinfoID_32960::M_GB_32960_MESSAGE_RECHARGE_DEVICE_VOLT_INFO);
                m_volt_info_stream_cache.energy_store_subsys_num = M_GB_32960_SZ_RECHARGE_DEVICE_SUBSYS_CNT;
                buf_vlot_cache[body_vlot_cache_len++] = m_volt_info_stream_cache.msgType;
                buf_vlot_cache[body_vlot_cache_len++] = m_volt_info_stream_cache.energy_store_subsys_num;

                // SMLK_LOGD("msgType=0x%02X", m_volt_info_stream_cache.msgType);
                // SMLK_LOGD("m_volt_info_stream_cache.energy_store_subsys_num=%02d", m_volt_info_stream_cache.energy_store_subsys_num);

                m_volt_info_cache.energy_device_volt = m_last_volt_info.energy_device_volt;
                m_volt_info_cache.energy_device_curr = m_last_volt_info.energy_device_curr;
                m_volt_info_cache.cell_num = m_last_volt_info.cell_num;

                int m_begin_seq_cache = m_volt_info_cache.begin_seq;
                int m_sigle_cell_list_num_cache = m_volt_info_cache.sigle_cell_list_num;

                //电池子系统号
                buf_vlot_cache[body_vlot_cache_len++] = m_volt_info.energy_device_seq;
                //总电压
                EndianSwap((uint8_t *)&(m_volt_info_cache.energy_device_volt), 0, sizeof(m_volt_info_cache.energy_device_volt));
                memcpy(&buf_vlot_cache[body_vlot_cache_len], &(m_volt_info_cache.energy_device_volt), sizeof(m_volt_info_cache.energy_device_volt));
                body_vlot_cache_len += sizeof(m_volt_info_cache.energy_device_volt);
                //总电流
                EndianSwap((uint8_t *)&(m_volt_info_cache.energy_device_curr), 0, sizeof(m_volt_info_cache.energy_device_curr));
                memcpy(&buf_vlot_cache[body_vlot_cache_len], &(m_volt_info_cache.energy_device_curr), sizeof(m_volt_info_cache.energy_device_curr));
                body_vlot_cache_len += sizeof(m_volt_info_cache.energy_device_curr);
                //单体电池总数
                EndianSwap((uint8_t *)&(m_volt_info_cache.cell_num), 0, sizeof(m_volt_info_cache.cell_num));
                memcpy(&buf_vlot_cache[body_vlot_cache_len], &(m_volt_info_cache.cell_num), sizeof(m_volt_info_cache.cell_num));
                body_vlot_cache_len += sizeof(m_volt_info_cache.cell_num);
                //本帧单体电池起始序号
                // SMLK_LOGD("begin_seq=%02d", m_volt_info_cache.begin_seq);
                EndianSwap((uint8_t *)&(m_volt_info_cache.begin_seq), 0, sizeof(m_volt_info_cache.begin_seq));
                memcpy(&buf_vlot_cache[body_vlot_cache_len], &(m_volt_info_cache.begin_seq), sizeof(m_volt_info_cache.begin_seq));
                body_vlot_cache_len += sizeof(m_volt_info_cache.begin_seq);
                //本帧单体电池总数
                // SMLK_LOGD("sigle_cell_list_num=%02d", m_volt_info_cache.sigle_cell_list_num);
                buf_vlot_cache[body_vlot_cache_len++] = m_volt_info_cache.sigle_cell_list_num;

                for (int i = 0; i < m_sigle_cell_list_num_cache; i++) {
                        buf_vlot_cache[body_vlot_cache_len++] = m_sigle_bat_vol[2 * i + 1 + 2 * (m_begin_seq_cache - 1)];
                        buf_vlot_cache[body_vlot_cache_len++] = m_sigle_bat_vol[2 * i + 2 * (m_begin_seq_cache - 1)];
                }

                CacheRTInfo rt_vlot_cache;
                memset(&rt_vlot_cache, 0, sizeof(rt_vlot_cache));
                rt_vlot_cache.info.msgId = static_cast<SMLK_UINT8>(msgID_32960::M_GB_32960_CMD_REALTIME_REPORT);
                rt_vlot_cache.info.rspFlag = static_cast<SMLK_UINT8>(rspFlag_32960::M_GB_32960_RSP_FLAG_CMD);
                rt_vlot_cache.info.encryptFlag = static_cast<int>(encryType_32960::M_GB_32960_ENCRYPTION_NONE);

                body_vlot_cache_len = (body_vlot_cache_len <= 1024 ? body_vlot_cache_len : 1024);
                rt_vlot_cache.info.dataBody = (SMLK_UINT8 *)malloc(body_vlot_cache_len * sizeof(SMLK_UINT8));
                if (rt_vlot_cache.info.dataBody != NULL) {
                    memcpy(rt_vlot_cache.info.dataBody, buf_vlot_cache, body_vlot_cache_len);
                    rt_vlot_cache.body_len = body_vlot_cache_len;
                }

                g_rtinfo_vlot_cache.push_back(rt_vlot_cache);
                SMLK_UINT8 *pdate = rt_vlot_cache.info.dataBody;
                if(pdate != NULL)
                {
                    SMLK_LOGD("DoRealTimeInfoUp save vlot cache, size=%d,time=%04d-%02d-%02d_%02d:%02d:%02d", g_rtinfo_vlot_cache.size(), pdate[0] + 2000, pdate[1], pdate[2], pdate[3], pdate[4], pdate[5]);
                }

                //以补发形式存储电池多包数据
                rt_vlot_cache.info.msgId = static_cast<SMLK_UINT8>(msgID_32960::M_GB_32960_CMD_REISSUE_REPORT);
                rt_info_reissue.rtinfo_vlot_cache.push_back(rt_vlot_cache);
            }
        }
    }
}

// 可充电存能装置温度数据
// if (0)
{
    vehicle::SignalVariant m_energy_device_temp_probe_num = common32960::getInstance()->m_cached_signals.find(vehicle::SignalID::EVEnergyStorageTempProbeNum)->second;

    m_dataUpload.TempInfo.msgType = static_cast<SMLK_UINT8>(rtinfoID_32960::M_GB_32960_MESSAGE_RECHARGE_DEVICE_TEMP_INFO);
    m_dataUpload.TempInfo.energy_store_subsys_num = M_GB_32960_SZ_RECHARGE_DEVICE_SUBSYS_CNT;
    buf[body_len++] = m_dataUpload.TempInfo.msgType;
    buf[body_len++] = m_dataUpload.TempInfo.energy_store_subsys_num;

    // SMLK_LOGD("msgType=0x%02X", m_dataUpload.TempInfo.msgType);
    // SMLK_LOGD("energy_store_subsys_num=%02d", m_dataUpload.TempInfo.energy_store_subsys_num);

    GB_32960_RechageDeviceTempInfo m_temp_info;
    m_temp_info.energy_device_seq = M_GB_32960_SZ_RECHARGE_DEVICE_SUBSYS_CNT;

    if (m_energy_device_temp_probe_num.Valid()) {
        m_last_temp_info.energy_device_temp_probe_num = m_energy_device_temp_probe_num.Encoded();
    }
    m_temp_info.energy_device_temp_probe_num = m_last_temp_info.energy_device_temp_probe_num;

    // SMLK_LOGD("tmp_sub_sys_seq=0x%02x", m_temp_info.energy_device_seq);
    // SMLK_LOGD("tmp_probe_num=0x%04x", m_temp_info.energy_device_temp_probe_num);

    buf[body_len++] = m_temp_info.energy_device_seq;

    int m_temp_probe_num = m_temp_info.energy_device_temp_probe_num;

    EndianSwap((uint8_t *)&m_temp_info.energy_device_temp_probe_num, 0, sizeof(m_temp_info.energy_device_temp_probe_num));
    memcpy(&buf[body_len], &m_temp_info.energy_device_temp_probe_num, sizeof(m_temp_info.energy_device_temp_probe_num));
    body_len += sizeof(m_temp_info.energy_device_temp_probe_num);

    for (int i = 0; i < m_temp_probe_num; i++) {
        if ((i + 1) > common32960::getInstance()->m_tempinfo.mdata.size()) {
            buf[body_len++] = 0xFF;
        } else {
            if (common32960::getInstance()->m_tempinfo.mdata[i] > 250 || common32960::getInstance()->m_tempinfo.mdata[i] < 0) {
                buf[body_len++] = 0xFF;
            } else {
                buf[body_len++] = common32960::getInstance()->m_tempinfo.mdata[i];
            }
        }
        // SMLK_LOGD("tmp_probe_num=%02d", common32960::getInstance()->m_tempinfo.mdata[i]);
    }
}

    if (!common32960::getInstance()->getThreeWarnFlag())
    {
        //以补发形式存储
        CacheRTInfo rt_info; 
        memset(&rt_info, 0, sizeof(rt_info));
        rt_info.info.msgId = static_cast<SMLK_UINT8>(msgID_32960::M_GB_32960_CMD_REISSUE_REPORT);
        rt_info.info.rspFlag = static_cast<SMLK_UINT8>(rspFlag_32960::M_GB_32960_RSP_FLAG_CMD);
        rt_info.info.encryptFlag = static_cast<int>(encryType_32960::M_GB_32960_ENCRYPTION_NONE);
        body_len = (body_len <= 1024 ? body_len : 1024);
        rt_info.info.dataBody = (SMLK_UINT8 *)malloc(body_len * sizeof(SMLK_UINT8));
        if (rt_info.info.dataBody != NULL)
        {
            memcpy(rt_info.info.dataBody, buf, body_len);
            rt_info.body_len = body_len;
        }

        

        memcpy(&rt_info_reissue.cachertinfo, &rt_info, sizeof(CacheRTInfo));
        g_rtinfo_cache_reissue.push_back(rt_info_reissue);
        if(g_rtinfo_cache_reissue.size() > CACHE_PKG_NUM)
        {
            CacheRTInfoReissue &rt_info = g_rtinfo_cache_reissue.front();
            if (rt_info.cachertinfo.info.dataBody != NULL)
            {
                free(rt_info.cachertinfo.info.dataBody);
            }
            g_rtinfo_cache_reissue.pop_front();
        }

        SMLK_UINT8 *pdate = rt_info.info.dataBody;
        if(pdate != NULL)
        {
            SMLK_LOGD("DoRealTimeInfoUp save cache,size=%d,time=%04d-%02d-%02d_%02d:%02d:%02d", g_rtinfo_cache.size(), pdate[0] + 2000, pdate[1], pdate[2], pdate[3], pdate[4], pdate[5]);
        }
    }
    else if (common32960::getInstance()->getThreeWarnFlag() && m_rt_send_num >= CACHE_PKG_NUM) //如果已经发了30个实时数据包
    {
        //开始发送补发数据
        if (m_rt_send_num == CACHE_PKG_NUM) //恢复实时数据的正常发送时间
        {
            countSendTime = 0;
            m_rt_send_num++;
            SMLK_LOGD("reconver realdata normal time");
            common32960::getInstance()->setInfoUpTime32960(common32960::getInstance()->getConfigParam().upload_period_bak);
        }
        //发送补发数据，按秒发，有可能补发期间突然断电未补发完
        //if (g_rtinfo_cache.size() > 0)
        if (g_rtinfo_cache_reissue.size() > 0)
        {
            if (m_is_send_num < g_rtinfo_cache_reissue.size())
            {
                CacheRTInfoReissue &rt_info = g_rtinfo_cache_reissue.at(m_is_send_num++);
                encodeMsg(rt_info.cachertinfo.info, rt_info.cachertinfo.body_len);

                while (rt_info.rtinfo_vlot_cache.size() > 0) {

                    CacheRTInfo &rt_vlot_info = rt_info.rtinfo_vlot_cache.front();
                    encodeMsg(rt_vlot_info.info, rt_vlot_info.body_len);
                    SMLK_UINT8 *pdate = rt_vlot_info.info.dataBody;
                    SMLK_LOGD("m_is_send_num=%02d", m_is_send_num);
                    if(pdate != NULL)
                    {
                        SMLK_LOGD("====== rsdata time: %04d-%02d-%02d_%02d:%02d:%02d", pdate[0] + 2000, pdate[1], pdate[2], pdate[3], pdate[4], pdate[5]);
                    }

                    rt_info.rtinfo_vlot_cache.pop_front();
                }
            }
            //SMLK_LOGD("g_rtinfo_cache.size = %d", g_rtinfo_cache.size());
        }
        else
        {
            m_is_send_num = CACHE_PKG_NUM;
        }

        if (m_is_send_num >= CACHE_PKG_NUM) //发完补发数据s
        {
            m_is_send_num = 0;
            m_rt_send_num = 0;
            common32960::getInstance()->setThreeWarnFlag(false);
            SMLK_LOGD("send warning rissue info end\n");
        }
    }

    countSendTime++;

    m_MessageEntity32960.dataBody = (SMLK_UINT8 *)buf;

    if (common32960::getInstance()->getInfoUpTime32960() <= countSendTime) {
        if (common32960::getInstance()->getIgnState()) {
            encodeMsg(m_MessageEntity32960, body_len);
            
            while (g_rtinfo_vlot_cache.size() > 0) {

                CacheRTInfo &rt_vlot_info = g_rtinfo_vlot_cache.front();
                encodeMsg(rt_vlot_info.info, rt_vlot_info.body_len);
                SMLK_UINT8 *pdate = rt_vlot_info.info.dataBody;
                g_rtinfo_vlot_cache.pop_front();
            }
        }
        countSendTime = 0;

        if (common32960::getInstance()->getThreeWarnFlag()) {
            m_rt_send_num++;
            SMLK_LOGD("m_rt_send_num=%d", m_rt_send_num);
        }
        SMLK_LOGD("====== rtdata time: %04d-%02d-%02d_%02d:%02d:%02d", m_dataUpload.sendTime[0] + 2000, m_dataUpload.sendTime[1], m_dataUpload.sendTime[2], m_dataUpload.sendTime[3], m_dataUpload.sendTime[4], m_dataUpload.sendTime[5]);
    }
}

void MessageRealTimeInfo32960::run(MessageRealTimeInfo32960 *obj)
{
    RTInfoUpPtr = obj;

    if (RTInfoUpTimer == nullptr)
    {
        RTInfoUpTimer = new EV_Timer(1, 1, dataUpLoop);
    }
    if (RTInfoUpTimer != nullptr)
    {
        if (TimerClient::getInstance()->chekExist_timer(*RTInfoUpTimer) == 0)
        {
            SMLK_LOGD("MessageRealTimeInfo32960::add RTInfoUpTimer....");
            TimerClient::getInstance()->add_timer(*RTInfoUpTimer);
        }
    }
}

void MessageRealTimeInfo32960::deleInfoUpTimer()
{
    if (RTInfoUpTimer != nullptr)
    {
        if (TimerClient::getInstance()->chekExist_timer(*RTInfoUpTimer) != 0)
        {
            SMLK_LOGD("MessageRealTimeInfo32960::del RTInfoUpTimer....");
            TimerClient::getInstance()->del_timer(*RTInfoUpTimer);
            delete RTInfoUpTimer;
            RTInfoUpTimer = nullptr;
        }
    }
}

/**
 * @brief dataUpLoop, 将数据按照规定的时间间隔上传
 */
void MessageRealTimeInfo32960::dataUpLoop()
{
    RTInfoUpPtr->DoRealTimeInfoUp();
}
