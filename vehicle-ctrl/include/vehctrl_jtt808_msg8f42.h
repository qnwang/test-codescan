/*****************************************************************************/
/**
 * \file       vehctrl_jtt808_msg8f42.h
 * \author     huangxin
 * \date       2020/11/30
 * \version    Tbox2.0 V1
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
#ifndef _VEHICLE_JTT808_MSG8F42_H_
#define _VEHICLE_JTT808_MSG8F42_H_
/*****************************************************************************
 *                                头文件引用                                  *
 *****************************************************************************/
#include <iostream>
#include <functional>
#include "future"
#include "smlk_tools.h"
#include <smlk_timer_new.h>

namespace smartlink
{
#define LICENSE_LENGTH 512	/*预见性巡航license长度*/
#define CONFIG_MODIFY_LEN 5 /*0x11 整车参数配置消息长度*/
#define CONFIG_QUERY_LEN 3	/*0x12 整车参数查询消息长度*/

	/*设置查询tbox信息的消息通用格式*/
	typedef struct
	{
		SMLK_UINT16 seq_id;
		SMLK_UINT8 cmd_id;
		SMLK_UINT16 length;
	} __attribute__((__packed__)) Msg0F42GetCommon;

	/*设置查询tbox信息的消息通用格式*/
	typedef struct
	{
		SMLK_UINT16 seq_id;
		SMLK_UINT8 cmd_id;
		SMLK_UINT16 length;
		SMLK_UINT8 result;
	} __attribute__((__packed__)) Msg0F42SetCommon;

	typedef struct
	{
		SMLK_UINT8 p1;							  //包含高4位保养要求,低4位保养总成类型
		SMLK_UINT16 p2;							  //保留字节,全f
		SMLK_UINT8 p3;							  //保留字节,全f
		SMLK_UINT32 p4;							  //保养到期剩余里程
	} __attribute__((__packed__)) MsgCan18FC174A; /*仪表盘保养里程CAN下发组包结构体*/

	enum CommonSetRes_0F42 : SMLK_UINT8
	{
		COM_SET_FAIL = 0,
		COM_SET_SUCC = 1,
	};

	/*设置查询tbox信息的消息通用格式*/
	typedef struct
	{
		SMLK_UINT8 cmd_id;
		SMLK_UINT16 length;
	} __attribute__((__packed__)) Msg8F42Common;

	enum SubCmdID_8F42 : SMLK_UINT8
	{
		CMD_VIN_GET = 0X03,				 /*获取VIN码*/
		CMD_VIN_SET = 0X04,				 /*设置VIN码*/
		CMD_SSID_PASWD_SET = 0X05,		 /*设置SSID和密码*/
		CMD_PROT_VERSION_GET = 0X06,	 /*获取终端平台通讯协议版本*/
		CMD_WIFI_SSID_PASSWD_GET = 0X07, /*获取wifi和ssid密码*/
		CMD_VEHICLE_CONFIG_SET = 0X0A,	 /*车型配置下发*/
		CMD_LICENSE_SET = 0X0B,			 /*预见性驾驶license配置*/
		CMD_DISCHAR_STD_GW_SET = 0X0D,	 /*排放标准网关设置*/
		CMD_GB_REC_STATUS_SET = 0X0F,	 /*国6TBOX是否备案*/
		CMD_ENC_CHIP_ID_SET = 0X10,		 /*设置加密芯片ID*/
		CMD_CAR_CONFIG_MODIFY = 0X11,	 /*整车配置修改*/
		CMD_CAR_CONFIG_QUERY = 0X12,	 /*整车配置查询*/
		CMD_ENGINE_VIN_GET = 0X13,		 /*获取发动机VIN码*/
		CMD_STORAGE_MODE_GET = 0X14,	 /*获取仓储模式*/
		CMD_STORAGE_MODE_SET = 0X15,	 /*设置仓储模式*/
		CMD_DB_MTN_RMD_SET = 0X19,		 /*仪表盘保养提醒信息设置*/
	};

	enum GB_REC_STATUS : SMLK_UINT8
	{
		GB_REC_CLOSE = 0,		 /*已收到8F42 0x0F消息:关闭备案逻辑*/
		GB_REC_OPEN = 1,		 /*已收到8F42 0x0F消息:打开备案逻辑*/
		GB_REC_CLOSE_FINISH = 2, /*HJ完成关闭备案逻辑*/
		GB_REC_OPEN_FINISH = 3,	 /*HJ完成打开备案逻辑*/
	};

	enum GB_REC_RESULT : SMLK_UINT8
	{
		GB_REC_RES_RESERVE = 0,	 /*保留*/
		GB_REC_RES_SUCCESS = 1,	 /*备案成功*/
		GB_REC_RES_ALDY_REC = 2, /*已备案*/
		GB_REC_RES_VIN_ERR = 3,	 /*VIN码错误备案失败*/
		GB_REC_RES_OTHER_ERR = 4 /*其他失败*/
	};

	enum VEHCLE_CONFIG_QUERY_SUPPORT_DID : SMLK_UINT16
	{
		VEHCLE_SUPPORT_DID_0100 = 0x0100,
		VEHCLE_SUPPORT_DID_0110 = 0x0110,
		VEHCLE_SUPPORT_DID_102a = 0x102a,
		VEHCLE_SUPPORT_DID_102b = 0x102b,
		VEHCLE_SUPPORT_DID_102c = 0x102c,
	};

	class DissectorMsg8F42 : public IDissector
	{
	public:
		DissectorMsg8F42();
		virtual ~DissectorMsg8F42();

	public:
		virtual SMLK_RC OnChanged(IN SMLK_UINT8 &, IN SMLK_UINT16 &, IN SMLK_DOUBLE &) { return SMLK_RC::RC_OK; };
		virtual SMLK_RC ResultEncode(IN RctrlResultQueue &) { return SMLK_RC::RC_OK; };
		virtual SMLK_RC Decode(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT RctrlCmdQueue &, OUT SMLK_UINT8 &is_encode);

	private:
		SMLK_RC SendMsg0F42(IN RctrlHead &head, IN SMLK_UINT8 &cmd_id, IN SMLK_UINT8 &result);
		SMLK_RC SendMsg0F42(IN RctrlHead &head, IN SMLK_UINT8 &cmd_id, IN std::string &str);
		SMLK_RC SendMsg0F42(IN RctrlHead &head, IN std::vector<SMLK_UINT8> &msg);
		/*仪表盘保养信息设置*/
		void Process_8F42_0x19(IN SMLK_UINT8 *indata, IN size_t length, INOUT SMLK_UINT8 &result);
	};
};

#endif
