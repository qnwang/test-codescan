/******************************************************************************
*
*  Copyright (C) 2020 SmartLink
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
#ifndef __MESSAGE_DEF_32960_H__
#define __MESSAGE_DEF_32960_H__
#include <stdint.h>
#include <string>
#include <vector>
#include <smlk_types.h>

#define SL_SUCCESS      0
#define SL_EFAILED      1
#define SL_ESUPPORT     2
#define SL_OVERFLOW     3
#define SL_ELIBINIT     4
#define SL_EEXISTS      5
#define SL_EFORMAT      6

#define M_GB_32960_SZ_FLAGS                 2               // GB32960 head flags size
#define M_GB_32960_SZ_VIN                   17              // GB32960 head vin size
#define M_GB_32960_SZ_TIMESTAMP             6
#define M_GB_32960_SZ_ICCID                 20
#define M_GB_32960_SZ_BATY_SYS_CNT_MAX      250
#define M_GB_32960_SZ_BATY_CODE_LEN_MAX     50
#define M_GB_32960_SZ_MOTOR_CNT_MAX         253
#define M_GB_32960_SZ_SIGLE_CELL_CNT_MAX    200
// #define M_GB_32960_SZ_SIGLE_CELL_CNT_MAX    6
#define M_GB_32960_SZ_PROBE_CNT             2
#define M_GB_32960_SZ_PROBE_CNT_MAX         65531

#define M_GB_32960_SZ_RECHARGE_DEVICE_SUBSYS_CNT                1
#define M_GB_32960_SZ_RECHARGE_DEVICE_SUBSYS_CNT_MAX            250
#define M_GB_32960_SZ_RECHARGE_SIGLE_CELL_LIST_CNT_MAX          200
#define M_GB_32960_SZ_RECHARGE_DEVICE_TEMP_PROBE_LIST_CNT_MAX   65531
#define M_GB_32960_SZ_RECHARGE_DEVICE_ERR_CNT_MAX               19

#define M_GB_32960_SZ_DATA_MAX              2048
#define M_GB_32960_SZ_HEAD_INFO             25              // 2 x '#' + 1 x ${cmd} + 17 x ${vin} + 1 x ${reversion} + 1 x ${encryption} + 2 x ${length} + 1 x ${BCC}


#define M_GB_32960_REPORT_PERIOD_DEF        10               // default report report period 10 s
#define M_GB_32960_REPORT_WARN_PERIOD_DEF   1


// GB32960 Flag define
#define M_GB_32960_MSG_FLAG                 0x23            // '#'

#define M_GB_32960_DATABASE_DIR             "/userdata/smartlink/record/ev/"
#define M_GB_32960_DATABASE_FILE_DIR        "/userdata/smartlink/record/ev/record_gb32960.db"
#define M_GB_32960_DATABASE_SAVEDAY         7
#define M_GB_32960_RAW_DATA_SAVE_DIR        "/userdata/smartlink/record/ev/record_gb32960.log"

#define M_GB_32960_CONFIG_DIR               "/userdata/smartlink/config/ev/"
#define M_GB_32960_CONFIG_FILE_DIR          "/userdata/smartlink/config/ev/ev_config.ini"
#define M_GB_32960_SEQUEMCE_LOGINHEAD       "loginSeq"

#pragma pack(1)

// GB32960 command class
enum class msgID_32960 {
    M_GB_32960_CMD_VEHICLE_LOGIN        = 0x01,
    M_GB_32960_CMD_REALTIME_REPORT      = 0x02,
    M_GB_32960_CMD_REISSUE_REPORT       = 0x03,
    M_GB_32960_CMD_VEHICLE_LOGOUT       = 0x04,

    M_GB_32960_CMD_HEARTBEAT            = 0x07,
};

// GB32960 response flag class
enum class rspFlag_32960 {
    M_GB_32960_RSP_FLAG_SUCCESS         = 0x01,
    M_GB_32960_RSP_FLAG_FAULT           = 0x02,
    M_GB_32960_RSP_FLAG_VIN_ERROR       = 0x03,
    M_GB_32960_RSP_FLAG_CMD             = 0xFE,
};

// GB32960 encryption method define
enum class encryType_32960 {
    M_GB_32960_ENCRYPTION_NONE          = 0x01,
    M_GB_32960_ENCRYPTION_RSA           = 0x02,
    M_GB_32960_ENCRYPTION_AES128        = 0x03,
    M_GB_32960_ENCRYPTION_ABNORMAL      = 0xFE,
    M_GB_32960_ENCRYPTION_INVALID       = 0xFF,
};

// GB32960 realtime message type class
enum class rtinfoID_32960 {
    M_GB_32960_MESSAGE_VEHICLE_INFO     = 0x01,             // 整车数据
    M_GB_32960_MESSAGE_MOTOR_INFO       = 0x02,             // 驱动电机数据
    M_GB_32960_MESSAGE_FUELCELL_INFO    = 0x03,             // 燃料电池数据
    M_GB_32960_MESSAGE_ENGINE_INFO      = 0x04,             // 发动机数据
    M_GB_32960_MESSAGE_LOCATION_INFO    = 0x05,             // 车辆位置数据
    M_GB_32960_MESSAGE_EXTREME_VALUES   = 0x06,             // 极值数据
    M_GB_32960_MESSAGE_ALERT_INFO       = 0x07,             // 报警数据
    M_GB_32960_MESSAGE_RECHARGE_DEVICE_VOLT_INFO = 0x08,    // 可充电储能装置电压数据
    M_GB_32960_MESSAGE_RECHARGE_DEVICE_TEMP_INFO = 0x09,    // 可充电储能装温度数据
};


typedef struct {
    SMLK_UINT8 errcode;
    SMLK_UINT32 errvalue;
} DM1Evnet;

typedef struct {
    //告警错误码
    SMLK_UINT8 max_warn;
    SMLK_UINT8 warn_num;
    SMLK_UINT32 common_warn;
    DM1Evnet  dm1_event[M_GB_32960_SZ_RECHARGE_DEVICE_ERR_CNT_MAX];
} DM1EventInfo;

typedef struct {
    SMLK_BOOL   valid;
    std::vector<SMLK_UINT8>  mdata;
} MDATAInfo;

// GB32960 message entity define
typedef struct
{
    SMLK_UINT16     startFlag;
    SMLK_UINT8      msgId;
    SMLK_UINT8      rspFlag;
    SMLK_UINT8      vin[M_GB_32960_SZ_VIN];
    SMLK_UINT8      encryptFlag;
    SMLK_UINT16     dataLength;
    SMLK_UINT8      *dataBody;
    SMLK_UINT8      checkCode;
} GB_32960_MsgEntity;

// GB32960 vehicle information stream define
typedef struct {
    SMLK_UINT8 msgType;
    SMLK_UINT8 car_state;
    SMLK_UINT8 charge_state;
    SMLK_UINT8 run_mode;
    SMLK_UINT16 car_speed;
    SMLK_UINT32 mile;
    SMLK_UINT16 vol_total;
    SMLK_UINT16 cur_total;
    SMLK_UINT8 soc;
    SMLK_UINT8 dcdc;
    SMLK_UINT8 gear;
    SMLK_UINT16 InsRes;
    SMLK_UINT8 resv[2];
    SMLK_UINT8 brake_pedal;
    SMLK_UINT8 bms_cnt;
    SMLK_UINT8 bms_encode_len;
    SMLK_UINT8 acc_pedal;
} GB_32960_VehicleInfoStream;

// GB32960 motor information stream define
typedef struct {
    SMLK_UINT8 msgType;
    SMLK_UINT8 motor_num;
    SMLK_UINT8 motor_seq;
    SMLK_UINT8 motor_state;
    SMLK_UINT16 motor_torque;
    SMLK_UINT16 motor_speed;
    SMLK_UINT16 motor_vol;
    SMLK_UINT16 motor_cur;
    SMLK_UINT8 motor_temp;
    SMLK_UINT8 motor_ctl_temp;
} GB_32960_MotorInfoStream;

// GB32960 fuel cell information stream define
typedef struct {
    SMLK_UINT8 msgType;
    SMLK_UINT16 fuel_cell_vol;
    SMLK_UINT16 fuel_cell_cur;
    SMLK_UINT16 fuel_rate;
    SMLK_UINT16 fuel_probe_cnt;
    SMLK_UINT8 probe_temp[M_GB_32960_SZ_PROBE_CNT];
    SMLK_UINT16 h_sys_high_temp;
    SMLK_UINT8 h_sys_high_temp_num;
    SMLK_UINT16 h_sys_high_concentration;
    SMLK_UINT8 h_sys_high_concentration_num;
    SMLK_UINT16 h_sys_high_press;
    SMLK_UINT8 h_sys_high_press_num;
    SMLK_UINT8 high_dcdc_state;
} GB_32960_FuelCellInfoStream;

// GB32960 engine information stream define
typedef struct {
    SMLK_UINT8 msgType;
    SMLK_UINT8 engine_state;
    SMLK_UINT16 crankshaft_speed;
    SMLK_UINT16 fuel_rate;
} GB_32960_EngineInfoStream;

// GB32960 location information stream define
typedef struct {
    SMLK_UINT8 msgType;
    SMLK_UINT8 pos_state;
    SMLK_UINT32 longitude; /* 经度 */
    SMLK_UINT32 latitude;  /* 纬度 */
} GB_32960_LocationInfoStream;


// GB32960 extreme information stream define
typedef struct {
    SMLK_UINT8 msgType;
    SMLK_UINT8 max_volbat_subsys_seq;
    SMLK_UINT8 min_volbat_subsys_seq;
    SMLK_UINT8 max_temp_subsys_seq;
    SMLK_UINT8 min_temp_subsys_seq;
    SMLK_UINT16 max_sigle_bat_vol;
    SMLK_UINT16 min_sigle_bat_vol;
    SMLK_UINT8 max_volbat_sigle_seq;
    SMLK_UINT8 min_volbat_sigle_seq;
    SMLK_UINT16 max_temp_probe_seq;
    SMLK_UINT16 min_temp_probe_seq;
    SMLK_UINT16 max_temp;
    SMLK_UINT16 min_temp;
} GB_32960_ExtremeInfoStream;

// GB32960 alarm information stream define
typedef struct {
    SMLK_UINT8 msgType;
    SMLK_UINT8 max_warn;
    SMLK_UINT32 common_warn;
    SMLK_UINT8 batt_warn_num;
    SMLK_UINT8 divmotor_warn_num;
    SMLK_UINT8 engmotor_warn_num;
    SMLK_UINT8 other_warn_num;
} GB_32960_AlarmInfoStream;

// GB32960 rechargeable energy device voltage information stream define
typedef struct {
    SMLK_UINT8 energy_device_seq;   // 可充电储能子系统号 1 有效值范围：1—250
    SMLK_UINT16 energy_device_volt; // 可充电储能装置电压 2 有效值范围：0～10000(表示Ov～iooov)，最小计最单元： o.iV，“OxFF，OxFE”表示异常，“OxFF，OxFF"表示无效
    SMLK_UINT16 energy_device_curr; // 可充电储能装置电流 2 有效值范围；0～20000(数值偏移量1000A，表示 -1000A～+1000A)，最小计量单元：0.1A，“OxFF, OxFE”表示异常，“OxFF，OxFF"表示无效
    SMLK_UINT16 cell_num;           // 荤体电池总数 2 N个电池单体，有效值范围：1—65531，“OxFF，OxFE"表示异 常，“OxFF，OxFF"表示无效
    SMLK_UINT16 begin_seq;          // 奉帧起始电池序号 2 当本帧单体个数超过200时，应拆分成多帧数据进行传输，有 效值范围：1—65531
    SMLK_UINT8 sigle_cell_list_num; // 本帧单体电池总数 1 本帧单体总数m；有效值范围：1—200
    // SMLK_UINT16     cell_volt_list[M_GB_32960_SZ_RECHARGE_SIGLE_CELL_LIST_CNT_MAX] ;    // 单体电池电压 2×m 有效值范围：0—60000(表示OV—60.000V)，最小计量单 元：0.001V，单体电池电压个数等于本帧单体电池总数优， “OxFF，OxFE"表示异常，“OxFF，OxFF"表示无效
} GB_32960_RechageDeviceVoltInfo;
typedef struct {
    SMLK_UINT8 msgType;
    SMLK_UINT8 energy_store_subsys_num;
} GB_32960_RechageDeviceVoltInfoStream;

// GB32960 rechargeable energy device temperature information stream define
typedef struct {
    SMLK_UINT8 energy_device_seq;             // 可充电储能子系统号 l 有效值范围：1～250
    SMLK_UINT16 energy_device_temp_probe_num; // 可充电储能 温凌探针个数 2 N个温度探针，有效值范围：1～65531，“OxFF，OxFE”表示异 常，“OxFF，OxFF”表示无效
    // SMLK_UINT8      subsys_sigle_probe_temp_list[M_GB_32960_SZ_RECHARGE_DEVICE_TEMP_PROBE_LIST_CNT_MAX];   // 可充电储能子系统各温度探针检 测到的温度值 1×N 有效值范围：0—250(数值偏移量40℃，表示-40℃～ +Zl0℃)，最小计量单元：1℃，“OxFE”表示异常，“OxFF”表 示无效
} GB_32960_RechageDeviceTempInfo;
typedef struct {
    SMLK_UINT8 msgType;
    SMLK_UINT8 energy_store_subsys_num;
} GB_32960_RechageDeviceTempInfoStream;

// GB32960 realtime report define
typedef struct {
    SMLK_UINT8 sendTime[6];                           //数据发送时间
    //SMLK_UINT8 msgFlag;                             //信息类型标志
    GB_32960_VehicleInfoStream              CarInfo;    //整车信息
    GB_32960_MotorInfoStream                MotorInfo;
    GB_32960_FuelCellInfoStream             FuelcellInfo;
    GB_32960_EngineInfoStream               EngInfo;
    GB_32960_LocationInfoStream             PosInfo;
    GB_32960_ExtremeInfoStream              LimiInfo;
    GB_32960_AlarmInfoStream                warnInfo;
    GB_32960_RechageDeviceVoltInfoStream    VolInfo;
    GB_32960_RechageDeviceTempInfoStream    TempInfo;
} GB_32960_RealTimeReport;


// GB32960 vehicle login define
typedef struct {
    SMLK_BCD        timestamp[M_GB_32960_SZ_TIMESTAMP];
    SMLK_UINT16     sequence;
    SMLK_UINT8      iccid[M_GB_32960_SZ_ICCID];
    SMLK_UINT8      baty_sys_cnt;
    SMLK_UINT8      baty_code_len;
//    SMLK_UINT8  baty_code[M_GB_32960_SZ_BATY_SYS_CNT_MAX][M_GB_32960_SZ_BATY_CODE_LEN_MAX];
} GB_32960_VehicleLogin;

// GB32960 vehicle logout define
typedef struct {
    SMLK_BCD        timestamp[M_GB_32960_SZ_TIMESTAMP];
    SMLK_UINT16     sequence;
} GB_32960_VehicleLogout;


#pragma pack()

#endif  // #ifndef __MESSAGE_DEF_32960_H__