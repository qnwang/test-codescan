/*****************************************************************************/
/**
 * \file       vehctrl_inf.h
 * \author     huangxin
 * \date       2020/11/04
 * \version    Tbox2.0 V1
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
#ifndef _VEHICLE_CTRL_INF_H_
#define _VEHICLE_CTRL_INF_H_
#include <iostream>
#include <vector>
#include "vehctrl_general_definition.h"
#include "vehctrl_queue.h"
#include "smlk_error.h"
#include "smlk_types.h"

namespace smartlink
{
  class IDissector
  {
  public:
    virtual SMLK_RC OnChanged(IN SMLK_UINT8 &, IN SMLK_UINT16 &, IN SMLK_DOUBLE &) = 0; /*三个参数分别为:数据来源-见RctrlMsgType,数据ID,数据内容*/
    virtual SMLK_RC ResultEncode(IN RctrlResultQueue &) = 0;
    virtual SMLK_RC Decode(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT RctrlCmdQueue &, OUT SMLK_UINT8 &is_encode) = 0;
    using VecCtrlInnerCallBack = std::function<SMLK_BOOL(VecCtrlInnerEventID, void *, size_t)>;
    virtual void RegisterCB(IN VecCtrlInnerCallBack &cb)
    {
      if (cb)
        m_vec_cb = cb;
    };

  protected:
    VecCtrlInnerCallBack m_vec_cb;
  };
}; // namespace smartlink
#endif
