#ifndef _SIGNAL_FORMAT_H_
#define _SIGNAL_FORMAT_H_
#include <smlk_types.h>
#include <iostream>
#include <sstream>

//#include <message_inf.h>
//#include <smlk_log.h>

enum ByteOrderType
{
    BYTE_ORDER_LITTLE_ENDIAN = 0,
    BYTE_ORDER_BIG_ENDIAN = 1

};
enum ValueType
{
    VALUE_TYPE_UNSIGNED = 0,
    VALUE_TYPE_SIGNED = 1
};

#pragma once

#include <bitset>
#include <iomanip>

#include "vehicle_data_service_def.h"
#include "smartlink_sdk_mcu.h"
#include "vehicle_data_def.h"
#include "smlk_types.h"
#include "smlk_log.h"

#define ROUND_UP_INTERGER(a, b)  (((a) + (b) - 1) / (b))

//目前解放全部为大端摩托罗拉序
using CAN_ID       = SMLK_UINT32;
using Index        = SMLK_UINT32;
using Cycle        = SMLK_UINT32;
using PhysicalData = SMLK_DOUBLE;
using CAN_DATA     = SMLK_UINT64;
using SPN_INDEX    = SMLK_UINT32;
using DBC_INDEX    = SMLK_UINT32;

namespace CanFormatData{

    struct CanFrame
    {
        SMLK_UINT32  can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
        SMLK_UINT16  can_dlc; /* frame payload length in byte (0 .. VEC_CAN_MAX_DATA_LEN) */
        SMLK_UINT8   data[0];
    };

    struct DataInfo
    {
        Index   index;
        Cycle   cycle;    //周期: ms 最大值 0xFFFFFF
    };

    // use for check timeout
    struct DM1DataInfo
    {
        SMLK_BOOL     data_valid;
        SMLK_UINT32   cycle;                          // can raw data cycle ms
        std::chrono::steady_clock::time_point tp;     // timepoint for data received
    };

    struct dtc_code{
        SMLK_UINT32 SPN:19;
        SMLK_UINT32 FMI:5;
        SMLK_UINT32 SRC:8;
    };
    typedef union {
        SMLK_UINT32 id;
        struct {
            SMLK_UINT32 SPN:19;
            SMLK_UINT32 FMI:5;
            SMLK_UINT32 SRC:8;
        }           dtc;
    } InternalDTC;

    struct CanMessageInfo
    {
        int           cycle;
        int     byte_lenth;
        int      can_channel;
    };

    struct CanPhysicalDataInfo
    {
        SMLK_UINT64 decoded_can_data;  // can raw data  after decode
        SMLK_DOUBLE physical_data;     // physical data after decode
        SMLK_BOOL   data_valid;
        Cycle       cycle;             //can raw data cycle ms
        std::chrono::steady_clock::time_point tp;     // timepoint for data received
        SMLK_BOOL   signal_timeout_out;
    };

    struct VehicleData
    {
        VehicleData() {}
        VehicleData(Index u32_index, SMLK_DOUBLE  dou_value)
        {
            index = u32_index;
            value = dou_value;
        }

        VehicleData(Index u32_index)
        {
            index = u32_index;
        }

        VehicleData(const VehicleData& other)
        {
            index = other.index;
            value = other.value;
            valid = other.valid;
        }

        VehicleData& operator=(const VehicleData& other)
        {
            index = other.index;
            value = other.value;
            valid = other.valid;
            return *this;
        }

        Index         index;         // 数据索引
        PhysicalData  value;         // 物理值
        SMLK_BOOL     valid = true;  // 数据是否有效
    };

}

class SignalFormat
{
public:
    SignalFormat() = delete;

	SignalFormat(  Index       	index,
	         	   SMLK_UINT8  	start_bit,
		     	   SMLK_UINT8  	bit_lenth,
		     	   SMLK_DOUBLE 	min = 0,
		     	   SMLK_DOUBLE 	max = 0,
		     	   SMLK_DOUBLE 	offset = 0.0,
		     	   SMLK_DOUBLE 	factor = 1.0,
				   ByteOrderType order_type = ByteOrderType::BYTE_ORDER_BIG_ENDIAN,
                   SMLK_UINT8   byte_lenth = 0,
                   SMLK_UINT8   start_byte = 0)
		     	   : m_index(index)
		     	   , m_start_bit(start_bit)
		     	   , m_bit_lenth(bit_lenth < 0 ? -bit_lenth : bit_lenth)
		     	   , m_min(min)
		     	   , m_max(max > 0 ? max : (-1ULL >> (sizeof(std::uint64_t) * 8 - bit_lenth)) * factor)
		     	   , m_offset(offset)
		     	   , m_factor(factor)
				   , m_order_type(order_type)
        		   , m_signed(bit_lenth < 0 ? true : false)
        		   , m_valid(false)
                   , m_val(0.0)
                   , m_byte_lenth(byte_lenth)
                   , m_start_byte(start_byte)
	{
	}

	SignalFormat(const SignalFormat& other)
				   : m_index(other.m_index)
				   , m_bit_lenth(other.m_bit_lenth)
        		   , m_signed(other.m_signed)
        		   , m_valid(other.m_valid)
				   , m_factor(other.m_factor)
				   , m_offset(other.m_offset)
				   , m_min(other.m_min)
				   , m_max(other.m_max)
				   , m_start_bit(other.m_start_bit)
				   , m_order_type(other.m_order_type)
        		   , m_val(other.m_val)
                   , m_byte_lenth(other.m_byte_lenth)
                   , m_start_byte(other.m_start_byte)
	{
	}

	virtual ~SignalFormat() {}

    SMLK_UINT32 getMsgId();
    SMLK_UINT32 getSignalId();
    SMLK_UINT8 getStartBit();
    SMLK_UINT8 getBitLength();
    ByteOrderType getByteOrder();
    ValueType getValueType();
    const char *getName();
    SMLK_UINT32 getTimeInterval();
    void genSimpleFormatString(char *pBuffer);
	SMLK_DOUBLE getSignalMaxPhyValue();
	SMLK_DOUBLE getSignalMinPhyValue();
	SMLK_DOUBLE getResoultion();
	SMLK_DOUBLE getOffSet();
	std::string getSignalName();

	const SMLK_UINT64 GetStartBit() const
	{
		return m_start_bit;
	}

	const SMLK_UINT64 Getlenth() const
	{
		return m_bit_lenth;
	}

	const Index GetIndex() const;

    std::uint32_t ID(void) const {
        return m_index;
    }
    std::uint32_t   CalculateRawData() const
    {
        double  v = m_val; // (double)(encoded & (0xFFFFFFFF >> (32 - m_bit_lenth)));

        v -= m_offset;
        v /= m_factor;

        return (std::uint32_t)v;
    }
    bool Valid() const {
       // 物理值六位精度比较
       // SMLK_LOGD("valuetest m_id = %d, m_min = %lf, m_val = %lf", m_index,  m_min, m_val);
       // SMLK_LOGD("  m_max = %lf, m_valid = %d \n", m_max, m_valid ? ((m_min <= m_val) && (m_val <= m_max)) : false);
       return m_valid ? ((m_min <= m_val) && (m_val <= m_max)) : false;
    }

    void Invalid() {
        m_valid = false;
    }

    double Val() const {
        return m_val;
    }

    double Min() const {
        return m_min;
    }

    double Max() const {
        return m_max;
    }

    int Bits() const {
        return m_bit_lenth;
    }

    bool IsInteger() const {
        return ((int)(m_factor)) == m_factor;
    }

    SignalFormat & operator=(double val)
    {
        m_val = val;
        // SMLK_LOGD("valuetest SignalFormat is operator= get value %f", m_val);
        m_valid = true;
        return *this;
    }

    SignalFormat & operator=(const SignalFormat &other)
    {
        // SMLK_LOGD("valuetest SignalFormat is operator= get value %f", m_val);
        m_val = other.m_val;
        m_valid = other.m_valid;
        return *this;
    }

    operator double() const { return m_val; }
    operator int() const { return (int)m_val; }
    operator bool() const { return (bool)m_val; }

#define ROUND_DIV(n,d) ((n)+((d)/2))/(d)
#if 1
    std::uint32_t Encoded() const
    {
        if ( !Valid() ) {
            return 0xFFFFFFFF;
        }
        std::uint64_t   v = ROUND_DIV(m_val - m_offset, m_factor);
        v &= -1ULL >> (64 - m_bit_lenth);
        return (std::uint32_t)v;
    }
#endif


	// const SMLK_INT32 GetCycle() const
	// {
	// 	return m_cycle;
	// }
	bool Decode(const CanFormatData::CanFrame* data, SMLK_DOUBLE& physics_data, SMLK_UINT64& decode_can_data) const
	{
		SMLK_BOOL decode_ret = true;

		//判断实际长度是否大于解码长度
		decode_ret = CheckDataLenthValid(data);

		if (decode_ret == false)
		{
			physics_data = 0;
			decode_can_data = 0;
			SMLK_LOGE("CAN RAW DATA lenth error, id[0x%x], byte_lenth[%d]", data->can_id, data->can_dlc);
			return false;
		}

		SMLK_UINT64 raw_data = 0;

 		for (SMLK_UINT8 i = 0; i < m_byte_lenth; ++i)
 		{
 			raw_data = raw_data | (data->data[m_start_byte + i - 1] << (8 * i));
 		}

		SMLK_UINT8 bit_head = m_start_bit % 8;

		raw_data = raw_data >> (bit_head - 1);
		// SMLK_LOGD("valuetest Caculate 00000, canid:0x%x, index = %d, decode_can_data = %d", data->can_id, m_index, decode_can_data);

		// Caculate计算正确，返回CheckDataLenthValid比特位置校验结果
		if (Caculate(raw_data, physics_data, decode_can_data))
		{
			// SMLK_LOGD("Caculate result, canid:0x%08x, index = %d, decode_can_data = %d", data->can_id, m_index, decode_can_data);
			return decode_ret;
		}

		// 计算错误，返回false
		SMLK_LOGE("[Error] : Caculate error, canid:0x%08x, index = %d, decode_can_data = %08x", data->can_id, m_index, decode_can_data);
		return false;
	}
private:
	bool CheckDataLenthValid(const CanFormatData::CanFrame* data) const
	{

		SMLK_UINT32 total_bit_lenth = (data->can_dlc) * 8;

		if (m_start_bit + m_bit_lenth - 1 > total_bit_lenth)
		{
			SMLK_LOGW("[Error] : canid:0x%x  can_dlc = %d, need total lenth = %d", data->can_id, data->can_dlc, m_start_bit + m_bit_lenth - 1);
			return false;
		}
		return true;
	}

	bool Caculate(const SMLK_UINT64& raw_data, SMLK_DOUBLE& physics_data, SMLK_UINT64& can_data) const
	{
		SMLK_BOOL ret = true;

		std::bitset<64> data;

		if (ByteOrderType::BYTE_ORDER_BIG_ENDIAN == m_order_type)
		{
			for (SMLK_UINT8 i = 0; i < m_bit_lenth; ++i)
			{
				data.set(i, (raw_data >> i) & 1);
			}
		}
		else if (ByteOrderType::BYTE_ORDER_LITTLE_ENDIAN == m_order_type)
		{
			for (SMLK_UINT8 i = m_bit_lenth - 1; i >= 0; ++i)
			{
				data.set(i, (raw_data >> i) & 1);
			}
		}
/* this check can signal range
		if (data.to_ullong() < m_min || data.to_ullong() > m_max)
		{
			SMLK_UINT64 t = data.to_ullong();
			SMLK_LOGW("[Decode]out of range : can data[%llu], max[%llu], min[%llu]", t, m_max, m_min);
			ret = false;
		}
*/
		can_data = data.to_ullong();
#if 0
		SMLK_LOGD("testdata data to string is [%s]", data.to_string().c_str());
		SMLK_LOGD("testdata can_data is [%lf]", can_data);
		SMLK_LOGD("testdata m_factor is [%lf]", m_factor);
		SMLK_LOGD("testdata m_offset is [%lf]", m_offset);
#endif
		physics_data = can_data * m_factor + m_offset;
		// SMLK_LOGW("testdata physics_data is [%lf]", physics_data);
        // 判断物理意义的最大最小
		// SMLK_LOGD(" physics_data is [%lf]", physics_data);

#if 0
        std::stringstream ss;
        ss << std::fixed << std::setprecision(6) << physics_data;
        ss >> physics_data;
        ss.clear();
		// SMLK_LOGD(" after presion physics_data is [%lf]", physics_data);
#endif
		if (physics_data < (SMLK_DOUBLE)m_min || physics_data > (SMLK_DOUBLE)m_max)
		{
			SMLK_UINT64 t = data.to_ullong();
			SMLK_LOGE("testdata!!!!!!!!!!!!out of physics range : physics_data[%lf], m_min[%lf], m_max[%lf]", physics_data, m_max, m_min);
			ret = false;
		}

		return ret;
	}
private:
	Index              m_index;               // spn索引
	SMLK_UINT8         m_start_bit;           // start bit
	SMLK_UINT8         m_bit_lenth;           // bit lenth
    bool               m_signed;

	SMLK_DOUBLE        m_min;                 // dbc提供的物理最小
	SMLK_DOUBLE        m_max;                 // dbc提供的物理最大值
	SMLK_DOUBLE        m_offset;              // offset = physical data - can data
	SMLK_DOUBLE        m_factor;              // Resolution
    bool               m_valid;

    SMLK_DOUBLE        m_val;
	ByteOrderType      m_order_type;          // bit order
	ValueType          m_value_type;
	SMLK_UINT8         m_start_byte;          // signal 在can fram中的start byte
	SMLK_UINT8         m_byte_lenth;          // signal byte lenth
	SMLK_UINT32        m_interval;
	CAN_ID             m_msg_id;
};
// typedef std::map<std::string, SignalFormat *> TSignalMap;

#endif