/*****************************************************************************/
/**
* \file       smlk_loan_VCU_ECONTROL.cpp
* \date       2022/1/5
* \author     weixuefeng
* \version    Tbox2.0 V2
* \brief      文件描述
* \note       Copyright (c) 2010-2030  SmartLink Co., Ltd.
* \remarks    loan ctrl
******************************************************************************/
/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/


#include "smlk_loan_VCU_ECONTROL.h"
#include <dlfcn.h>
#include <mutex>
using namespace smartlink::CAN;
using namespace std;
typedef unsigned char byte;
namespace smartlink {
namespace LOAN {

SMLK_BOOL SMLK_LOAN_VCU_ECONTROL::EncryptSeedKey(std::vector<SMLK_UINT8>& vec /*IN OUT*/)
{
    SMLK_LOGI("Encrypt libpath -> [%s]", m_encrypt_lib_path.c_str());
    SMLK_LOGI("Encrypt funName -> [%s]", m_encrypt_func_name.c_str());

    SMLK_LOGI(" seed : %02X %02X %02X %02X", vec[0], vec[1], vec[2], vec[3]);

    std::vector<SMLK_UINT8> m_seed(4);
    m_seed[0] = vec[3];
    m_seed[1] = vec[2];
    m_seed[2] = vec[1];
    m_seed[3] = vec[0];
    SMLK_LOGI(" m_seed : %02X %02X %02X %02X", m_seed[0], m_seed[1], m_seed[2], m_seed[3]);

    if (m_encrypt_lib_path.empty()) return SMLK_FALSE;
    void* dl_handle = dlopen(m_encrypt_lib_path.c_str(), RTLD_NOW);
    SMLK_LOGD("Load Gaslib success!\r\n");
    if(dl_handle == NULL)
    {
        char *error = NULL;
        error = dlerror();
        char tp[1024] = {0};
        snprintf(tp, 256, "Load module \"%s\" error: %s\n", m_encrypt_lib_path.c_str(), error);
        SMLK_LOGW("%s",tp);
        return SMLK_FALSE;
    }
    SMLK_LOGD("Load module[%s] success!\r\n", m_encrypt_lib_path.c_str());
    SMLK_UINT8 key[16] = {0};
    SMLK_UINT16 key_len = m_keylen;

    unsigned int (*myseedToKey) (byte *seed, unsigned int length, byte *key,unsigned int *retLen);
    myseedToKey = (unsigned int (*)(byte *seed, unsigned int length, byte *key,unsigned int *retLen))dlsym(dl_handle, m_encrypt_func_name.c_str());
    myseedToKey(m_seed.data(), m_seed.size(), key, (unsigned int *)&key_len);
    SMLK_LOGI(" key :%02X %02X %02X %02X, key_len:%d ", key[0], key[1], key[2], key[3],key_len);
    string name_str = SYS_PRO_NAME_ENGINE_TYPE;

    if(dl_handle) {
        dlclose(dl_handle);
        dl_handle = NULL;
    }
    vec.clear();
    vec.insert(vec.end(), key, key + key_len);
    SMLK_LOGI(" vec :%02X %02X %02X %02X", vec[0], vec[1], vec[2], vec[3]);
    return SMLK_TRUE;
}


}
}
