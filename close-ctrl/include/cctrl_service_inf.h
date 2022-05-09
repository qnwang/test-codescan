/*****************************************************************************/
/**
* \file       remote_ctrl_service.h
* \author     huangxin
* \date       2020/10/30
* \version    Tbox2.0 V1
* \brief      service common interface
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    修改日志
******************************************************************************/
#ifndef _REMOTE_CTRL_SERVICE_H_
#define _REMOTE_CTRL_SERVICE_H_
/*****************************************************************************
*                                头文件引用                                  *
*****************************************************************************/

#include <time.h>
#include <list>
#include "smlk_error.h"
#include "smlk_types.h"
#include "smlk_types.h"

namespace smartlink {

class IService {
public:
    virtual ~IService(){}
    virtual SMLK_RC     Init() = 0;
    virtual SMLK_RC     Start() = 0;
    virtual void            Stop() = 0;
    virtual bool            IsRunning() const = 0;

protected:
    static const SMLK_UINT32    Uninitialized   = 0x00000000;
    static const SMLK_UINT32    Initialized     = 0x00000001;
    static const SMLK_UINT32    Running         = 0x00000002;
    static const SMLK_UINT32    Stopping        = 0x00000004;
};

};  // namespace smartlink




#endif
