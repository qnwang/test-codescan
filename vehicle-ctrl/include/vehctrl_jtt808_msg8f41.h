/*****************************************************************************/
/**
 * \file       remote_ctrl_jtt808_msg8f41.h
 * \author     huangxin
 * \date       2020/11/25
 * \version    Tbox2.0 V1
 * \brief      文件描述
 * \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
 * \remarks    修改日志
 ******************************************************************************/
#ifndef _VEHICLE_JTT808_MSG8F41_H_
#define _VEHICLE_JTT808_MSG8F41_H_
/*****************************************************************************
 *                                头文件引用                                  *
 *****************************************************************************/
namespace smartlink
{
/*车门控制*/
#define JTT808_SUBCMD_DOOR_UNLOCK 0X01 /*功能指令:车门解锁 -> 8F41 子指令:0x01 功能指令:0x01 功能参数:0x00*/
#define JTT808_SUBCMD_DOOR_LOCK 0X02   /*功能指令:车门上锁 -> 8F41 子指令:0x01 功能指令:0x01 功能参数:0x00*/
/*发动机控制*/
#define JTT808_SUBCMD_ENGINE_START 0X01 /*功能指令:启动发动机 -> 8F41 子指令:0x06 功能指令:0x01 功能参数:0x00*/
#define JTT808_SUBCMD_ENGINE_STOP 0X02  /*功能指令:关闭发动机 -> 8F41 子指令:0x06 功能指令:0x01 功能参数:0x00*/
/*原车空调*/
#define JTT808_SUBCMD_AC_OPEN 0X01           /*请求打开原车空调*/
#define JTT808_SUBCMD_AC_CLOSE 0X02          /*请求关闭原车空调*/
#define JTT808_SUBCMD_AC_AIROUTLET_MODE 0X03 /*原车空调出风模式*/
#define JTT808_SUBCMD_AC_TEMPER 0X04         /*原车空调温度控制*/
#define JTT808_SUBCMD_AC_WIND 0X05           /*原车空调出风量设置*/
#define JTT808_SUBCMD_AC_LOOP_MOOD 0X0F      /*原车空调循环模式*/
#define JTT808_SUBCMD_AC_COMPRESSOR 0X10     /*原车空调压缩机*/
#define JTT808_SUBCMD_AC_AUTO_SWITCH 0X11    /*原车空调AUTO开关*/
#define JTT808_SUBCMD_AC_VENTILATION 0X12    /*原车空调一键通风开关*/
#define JTT808_SUBCMD_AC_DEFROST 0X13        /*原车空调强制除霜开关*/
/*独立暖风*/
#define JTT808_SUBCMD_INDEPT_WARM_AIR_SWITCH 0X08 /*独立暖风开关*/
#define JTT808_SUBCMD_INDEPT_WARM_AIR_TEMPER 0X09 /*独立暖风温度*/
/*驻车空调*/
#define JTT808_SUBCMD_PARKING_AC_SWITCH 0X0D /*驻车空调开关:ON OF*/
#define JTT808_SUBCMD_PARKING_AC_AUTO 0X0E   /*驻车空调AUTO开关:ON OF*/
#define JTT808_SUBCMD_PARKING_AC_TEMPER 0X0B /*驻车空调温度*/
#define JTT808_SUBCMD_PARKING_AC_WIND 0X0C   /*驻车空调风量*/
/*原车空调出风模式*/
#define JTT808_PARAM_BLOW_FACE 0         /*吹面*/
#define JTT808_PARAM_BLOW_FACE_FEET 1    /*吹面+吹脚*/
#define JTT808_PARAM_BLOW_FEET 2         /*吹脚*/
#define JTT808_PARAM_BLOW_FEET_DEFROST 3 /*吹脚+除霜*/
/*原车空调温度控制*/
#define JTT808_PARAM_AIRCONDITON_TEMPER_MIN 0  /*n最小值,原车空调最低设置温度17,算法0.5*n+17*/
#define JTT808_PARAM_AIRCONDITON_TEMPER_MAX 32 /*n最小值,原车空调最高设置温度33,算法0.5*n+17*/
/*原车空调出风量设置*/
#define JTT808_PARAM_AIRCONDITON_WIND_MIN 0  /*原车空调风量最小值*/
#define JTT808_PARAM_AIRCONDITON_WIND_MAX 13 /*原车空调风量最大值*/
/*原车空调循环模式*/
#define JTT808_PARAM_AC_OUTER_LOOP 1 /*外循环*/
#define JTT808_PARAM_AC_INNER_LOOP 0 /*内循环*/
/*原车空调压缩机开关*/
#define JTT808_PARAM_AC_COMPRESSOR_OPEN 1  /*压缩机打开*/
#define JTT808_PARAM_AC_COMPRESSOR_CLOSE 0 /*压缩机关闭*/
/*原车空调AUTO开关*/
#define JTT808_PARAM_AC_AUTO_ON 1  /*AUTO打开*/
#define JTT808_PARAM_AC_AUTO_OFF 0 /*AUTO关闭*/
/*原车空调一键通风开关*/
#define JTT808_PARAM_AC_VENTILATION_ON 1  /*一键通风打开*/
#define JTT808_PARAM_AC_VENTILATION_OFF 0 /*一键通风关闭*/
/*原车空调强制除霜*/
#define JTT808_PARAM_AC_DEFROST_ON 1  /*强制除霜打开*/
#define JTT808_PARAM_AC_DEFROST_OFF 0 /*强制除霜关闭*/
/*独立暖风开关设置*/
#define JTT808_PARAM_WARM_AIR_ON 1  /*打开独立暖风*/
#define JTT808_PARAM_WARM_AIR_OFF 0 /*关闭独立暖风*/
/*独立暖风温度设置*/
#define JTT808_PARAM_WARM_AIR_MIN 1 /*独立暖风最小值*/
#define JTT808_PARAM_WARM_AIR_MAX 7 /*独立暖风最大值*/
/*驻车空调开关设置*/
#define JTT808_PARAM_PARKING_AIR_ON 1  /*驻车空调打开*/
#define JTT808_PARAM_PARKING_AIR_OFF 0 /*驻车空调关闭*/
/*驻车空调温度设置*/
#define JTT808_PARAM_PARKING_AIRCONDITON_TEMPER_MIN 0  /*驻车空调最低设置温度,17,算法0.5*n+17*/
#define JTT808_PARAM_PARKING_AIRCONDITON_TEMPER_MAX 32 /*驻车空调最高设置温度,33,算法0.5*n+17*/
/*驻车空调风量设置*/
#define JTT808_PARAM_PARKING_AC_WIND_MIN 0  /*驻车空调最低设置温度*/
#define JTT808_PARAM_PARKING_AC_WIND_MAX 13 /*驻车空调最高设置温度*/
/*驻车空调AUTO开关设置*/
#define JTT808_PARAM_PARKING_AIR_AUTO_ON 1  /*驻车空调auto打开*/
#define JTT808_PARAM_PARKING_AIR_AUTO_OFF 0 /*驻车空调auto关闭*/
/*后视镜控制项*/
#define JTT808_SUBCMD_DRIVER_PASSAGER_MIRROR 1    /*驾驶侧,乘客侧后视镜*/
#define JTT808_PARAM_REARVIEW_MIRROR_FOLD 0       /*后视镜折叠*/
#define JTT808_PARAM_REARVIEW_MIRROR_UNFOLD 1     /*后视镜展开*/
#define JTT808_PARAM_REARVIEW_MIRROR_HORI_MOVE 2  /*后视镜水平移动*/
#define JTT808_PARAM_REARVIEW_MIRROR_VERT_MOVE 3  /*后视镜垂直移动*/
#define JTT808_PARAM_REARVIEW_MIRROR_HEAT 4       /*后视镜加热*/
#define JTT808_PARAM_REARVIEW_MIRROR_HEAT_CLOSE 5 /*后视镜加热关闭*/
/*油箱防盗开关控制项*/
#define JTT808_SUBCMD_MAILBOX_SECURITY_OPEN 1  /*打开*/
#define JTT808_SUBCMD_MAILBOX_SECURITY_CLOSE 2 /*关闭*/
/*怠速暖机开关控制项*/
#define JTT808_SUBCMD_IDLING_WARM_UP_CLOSE 1   /*关闭*/
#define JTT808_SUBCMD_IDLING_WARM_UP_LEVEL_1 2 /*1挡*/
#define JTT808_SUBCMD_IDLING_WARM_UP_LEVEL_2 3 /*2挡*/
/*熄火上报动作*/
#define JTT808_PARAM_ENGINE_STOP_ON_TIME 3   /*定时熄火*/
#define JTT808_PARAM_ENGINE_STOP_EMERGENCY 4 /*紧急熄火*/
/*智能冷机控制*/
#define JTT808_SUBCMD_RUC_SWITCH 0X01         /*智能冷机开关控制*/
#define JTT808_SUBCMD_RUC_TEMPER 0X02         /*智能冷机温度设置*/
#define JTT808_SUBCMD_RUC_REMOTE_DEFROST 0X03 /*智能冷机远程除霜控制*/
/*一键温暖&一键清凉设置*/
#define JTT808_SUBCMD_AUTO_TEMPER_HEAT 0x01 /*一键温暖*/
#define JTT808_SUBCMD_AUTO_TEMPER_COOL 0x02 /*一键清凉*/

    class DissectorMsg8F41 : public IDissector
    {
    public:
        DissectorMsg8F41();
        virtual ~DissectorMsg8F41();

    public:
        virtual SMLK_RC OnChanged(IN SMLK_UINT8 &, IN SMLK_UINT16 &, IN SMLK_DOUBLE &) { return SMLK_RC::RC_OK; };
        virtual SMLK_RC ResultEncode(IN RctrlResultQueue &);
        virtual SMLK_RC Decode(IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length, OUT RctrlCmdQueue &, OUT SMLK_UINT8 &is_encode);

    private:
        void Msg8F41Door(IN SMLK_UINT8 subcmd_id, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag);
        void Msg8F41Engine(IN SMLK_UINT8 subcmd_id, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag);
        void Msg8F41AC(IN SMLK_UINT8 subcmd_id, IN SMLK_UINT8 param, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag);
        void Msg8f41OriginalAC(IN SMLK_UINT8 subcmd_id, IN SMLK_UINT8 param, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag);
        void Msg8f41IndptWarmAC(IN SMLK_UINT8 subcmd_id, IN SMLK_UINT8 param, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag);
        void Msg8f41ParkingAC(IN SMLK_UINT8 subcmd_id, IN SMLK_UINT8 param, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag);
        void Msg8F41Mirror(IN SMLK_UINT8 subcmd_id, IN SMLK_UINT8 param, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag);
        void Msg8F41IdlingWarmUp(IN SMLK_UINT8 subcmd_id, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag);
        void Msg8F41OilTankAntiSheftSwitch(IN SMLK_UINT8 subcmd_id, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag);
        SMLK_UINT8 Msg8F41FinancialLckCar(IN SMLK_UINT8 subcmd_id, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag);
        void Msg8F41Ruc(IN SMLK_UINT8 subcmd_id, IN SMLK_UINT8 param, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag);
        void Msg8F41AutoTemper(IN SMLK_UINT8 subcmd_id, IN SMLK_UINT8 param, OUT SmlkMcuCmd &mcu_cmd, OUT bool &param_err_flag);
        SMLK_RC SendMsg0F41(IN RctrlHead &head, IN SMLK_UINT8 *indata, IN SMLK_UINT16 &length);
        SMLK_RC SendMsg0F41(IN RctrlHead &head, IN SMLK_UINT8 *indata, IN SMLK_UINT16 &, IN SMLK_UINT8 loan_enable);
    };
};
#endif
