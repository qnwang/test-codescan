
#pragma once
#include "smlk_types.h"

typedef struct
{
        SMLK_UINT16 data_len;// type +data[]
        SMLK_UINT8  type;// 1:ble
        SMLK_UINT8  data[0]; // reserved
}__attribute__((__packed__))TspCmdData;