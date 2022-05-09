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
#pragma once
#include <tuple>
#include <vector>
#include <string>
#include <smlk_types.h>
#include <sl_errors.h>

namespace smartlink {

class Recorder {
public:
    enum Flags : SMLK_UINT8 {
        login       = 0x01,
        realtime    = 0x02,
        reissue     = 0x03,
        logout      = 0x04,

        ignore      = 0xFF,
    };
public:
    Recorder();
    Recorder(IN std::string &dir, IN std::string &db="record_gb17691.sqlite3");
    Recorder(IN Recorder &) = delete;
    Recorder(Recorder &&) = delete;
    ~Recorder();

    Recorder & operator=(IN Recorder &) = delete;

    SMLK_UINT32 Init();
    SMLK_UINT32 Open();
    void        Close();

    using Record = std::tuple<SMLK_UINT32, SMLK_UINT8, std::vector<SMLK_UINT8>>;

    SMLK_UINT32 Insert(IN SMLK_UINT32 id, IN SMLK_UINT8 flag, IN SMLK_UINT8 data[], IN std::size_t sz);
    SMLK_UINT32 Insert(IN SMLK_UINT32 id, IN SMLK_UINT8 flag, IN std::vector<SMLK_UINT8> &data);
    SMLK_UINT32 Update(IN SMLK_UINT32 id, IN SMLK_UINT8 flag);
    SMLK_UINT32 Update(IN std::vector<SMLK_UINT32> &ids, IN SMLK_UINT8 flag);
    SMLK_UINT32 Query(IN SMLK_UINT32 id, SMLK_UINT8 &flag,  SMLK_UINT8 data[], std::size_t &sz);
    SMLK_UINT32 Query(IN SMLK_UINT32 id, SMLK_UINT8 &flag, std::vector<SMLK_UINT8> &data);
    SMLK_UINT32 Delete(IN SMLK_UINT32 id);
    SMLK_UINT32 Delete(IN std::vector<SMLK_UINT32> &ids);
    SMLK_UINT32 QueryTotal(SMLK_UINT32 &total);
    SMLK_UINT32 DeleteLessThan(IN SMLK_UINT32 id);
    SMLK_UINT32 DeleteAhead(IN SMLK_UINT32 n);
    SMLK_UINT32 QueryOneRecord(std::vector<std::tuple<SMLK_UINT32, std::vector<SMLK_UINT8>>> &records, SMLK_UINT32 id_from, SMLK_UINT32 id_to, std::size_t limit);
    SMLK_UINT32 QueryWhereFlag(IN SMLK_UINT8 flag, std::vector<std::tuple<SMLK_UINT32, std::vector<SMLK_UINT8>>> &records, SMLK_UINT32 id_from = 0x00000000, SMLK_UINT32 id_to=0xFFFFFFFF, std::size_t limit=16);

    const std::string & LastError() const;

private:
    void LastError(IN int rc, IN char *err);

private:
    std::string     m_dir;
    std::string     m_db;
    void *          m_dbh;

    struct {
        int         rc;
        std::string desc;
    }               m_last_error;
};

};  // namespace smartlink