
#include "SignalFormat.h"

#define MAX_NAME_LENGTH 129
/*************************************************************
Summary: SignalFormat,constrcutor
Parameters:
   None
Return: no
**************************************************************/
const Index SignalFormat::GetIndex() const
{
    return m_index;
}

SMLK_UINT32 SignalFormat::getMsgId()
{
    return m_msg_id;
}

SMLK_UINT32 SignalFormat::getSignalId()
{
    return m_index;
}

ByteOrderType SignalFormat::getByteOrder()
{
    return m_order_type;
}

ValueType SignalFormat::getValueType()
{
    return m_value_type;
}

SMLK_UINT8 SignalFormat::getStartBit()
{
    return m_start_bit;
}


SMLK_UINT8 SignalFormat::getBitLength()
{
    return m_bit_lenth;
}

SMLK_UINT32 SignalFormat::getTimeInterval()
{
    return m_interval;
}

SMLK_DOUBLE SignalFormat::getSignalMaxPhyValue()
{
    return m_max;
}

SMLK_DOUBLE SignalFormat::getSignalMinPhyValue()
{
    return m_min;
}

SMLK_DOUBLE SignalFormat::getOffSet()
{
    return m_offset;
}

SMLK_DOUBLE SignalFormat::getResoultion()
{
    return m_factor;
}








