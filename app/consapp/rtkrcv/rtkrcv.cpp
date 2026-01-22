#include "rtkrcv.hpp"

#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <csignal>
#include <exception>
#include <format>
#include <iostream>
#include <stdexcept>
#include <thread>

#include <time.h>

#include "rtklib.h"

constexpr size_t MAXSTR = 1024; /* max length of a stream */

/* receiver options table ----------------------------------------------------*/
#define TIMOPT "0:gpst,1:utc,2:jst,3:tow"
#define CONOPT "0:dms,1:deg,2:xyz,3:enu,4:pyl"
#define FLGOPT "0:off,1:std+2:age/ratio/ns"
#define ISTOPT "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,6:ntripcli,7:ftp,8:http"
#define OSTOPT "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,5:ntripsvr,9:ntripcas,11:udpcli"
#define FMTOPT                                                                 \
    "0:rtcm2,1:rtcm3,2:oem4,4:ubx,5:swift,6:hemis,7:skytraq,8:javad,9:nvs,10:" \
    "binex,11:rt17,12:sbf,14:unicore,15:rinex,16:sp3,17:clk"
#define NMEOPT "0:off,1:latlon,2:single"
#define SOLOPT "0:llh,1:xyz,2:enu,3:nmea,4:stat"
#define MSGOPT "0:all,1:rover,2:base,3:corr"

/* global variables ----------------------------------------------------------*/
static rtksvr_t svr; /* rtk server struct */

static int    timetype           = 0; /* time format (0:gpst,1:utc,2:jst,3:tow) */
static int    soltype            = 0; /* solution format (0:dms,1:deg,2:xyz,3:enu,4:pyl) */
static int    solflag            = 2; /* solution flag (1:std+2:age/ratio/ns) */
static int    strtype[]          = {STR_SERIAL, STR_NONE, STR_NONE, STR_NONE,
                                    STR_NONE,   STR_NONE, STR_NONE, STR_NONE}; /* stream types */
static char   strpath[8][MAXSTR] = {"", "", "", "", "", "", "", ""};           /* stream paths */
static int    strfmt[]           = {STRFMT_UBX, STRFMT_RTCM3, STRFMT_SP3, SOLF_LLH,
                                    SOLF_NMEA}; /* stream formats */
static int    svrcycle           = 10;          /* server cycle (ms) */
static int    timeout            = 10000;       /* timeout time (ms) */
static int    reconnect          = 10000;       /* reconnect interval (ms) */
static int    nmeacycle          = 5000;        /* nmea request cycle (ms) */
static int    buffsize           = 32768;       /* input buffer size (bytes) */
static int    navmsgsel          = 0;           /* navigation mesaage select */
static char   proxyaddr[256]     = "";          /* http/ntrip proxy */
static int    nmeareq            = 0;           /* nmea request type (0:off,1:lat/lon,2:single) */
static double nmeapos[]          = {0, 0, 0};   /* nmea position (lat/lon/height) (deg,m) */
static char   rcvcmds[3][MAXSTR] = {""};        /* receiver commands files */
static char   startcmd[MAXSTR]   = "";          /* start command */
static char   stopcmd[MAXSTR]    = "";          /* stop command */
static int    fswapmargin        = 30;          /* file swap margin (s) */
static char   sta_name[256]      = "";          /* station name */

static prcopt_t prcopt;            /* processing options */
static solopt_t solopt[2] = {{0}}; /* solution options */
static filopt_t filopt    = {""};  /* file options */

static char error_msg[2048] = ""; /* error message buffer */

static opt_t rcvopts[] = {{"console-timetype", 3, (void*)&timetype, TIMOPT},
                          {"console-soltype", 3, (void*)&soltype, CONOPT},
                          {"console-solflag", 0, (void*)&solflag, FLGOPT},

                          {"inpstr1-type", 3, (void*)&strtype[0], ISTOPT},
                          {"inpstr2-type", 3, (void*)&strtype[1], ISTOPT},
                          {"inpstr3-type", 3, (void*)&strtype[2], ISTOPT},
                          {"inpstr1-path", 2, (void*)strpath[0], ""},
                          {"inpstr2-path", 2, (void*)strpath[1], ""},
                          {"inpstr3-path", 2, (void*)strpath[2], ""},
                          {"inpstr1-format", 3, (void*)&strfmt[0], FMTOPT},
                          {"inpstr2-format", 3, (void*)&strfmt[1], FMTOPT},
                          {"inpstr3-format", 3, (void*)&strfmt[2], FMTOPT},
                          {"inpstr2-nmeareq", 3, (void*)&nmeareq, NMEOPT},
                          {"inpstr2-nmealat", 1, (void*)&nmeapos[0], "deg"},
                          {"inpstr2-nmealon", 1, (void*)&nmeapos[1], "deg"},
                          {"inpstr2-nmeahgt", 1, (void*)&nmeapos[2], "m"},
                          {"outstr1-type", 3, (void*)&strtype[3], OSTOPT},
                          {"outstr2-type", 3, (void*)&strtype[4], OSTOPT},
                          {"outstr1-path", 2, (void*)strpath[3], ""},
                          {"outstr2-path", 2, (void*)strpath[4], ""},
                          {"outstr1-format", 3, (void*)&strfmt[3], SOLOPT},
                          {"outstr2-format", 3, (void*)&strfmt[4], SOLOPT},
                          {"logstr1-type", 3, (void*)&strtype[5], OSTOPT},
                          {"logstr2-type", 3, (void*)&strtype[6], OSTOPT},
                          {"logstr3-type", 3, (void*)&strtype[7], OSTOPT},
                          {"logstr1-path", 2, (void*)strpath[5], ""},
                          {"logstr2-path", 2, (void*)strpath[6], ""},
                          {"logstr3-path", 2, (void*)strpath[7], ""},

                          {"misc-svrcycle", 0, (void*)&svrcycle, "ms"},
                          {"misc-timeout", 0, (void*)&timeout, "ms"},
                          {"misc-reconnect", 0, (void*)&reconnect, "ms"},
                          {"misc-nmeacycle", 0, (void*)&nmeacycle, "ms"},
                          {"misc-buffsize", 0, (void*)&buffsize, "bytes"},
                          {"misc-navmsgsel", 3, (void*)&navmsgsel, MSGOPT},
                          {"misc-proxyaddr", 2, (void*)proxyaddr, ""},
                          {"misc-fswapmargin", 0, (void*)&fswapmargin, "s"},

                          {"misc-startcmd", 2, (void*)startcmd, ""},
                          {"misc-stopcmd", 2, (void*)stopcmd, ""},

                          {"file-cmdfile1", 2, (void*)rcvcmds[0], ""},
                          {"file-cmdfile2", 2, (void*)rcvcmds[1], ""},
                          {"file-cmdfile3", 2, (void*)rcvcmds[2], ""},

                          {"", 0, nullptr, ""}};

/* read antenna file ---------------------------------------------------------*/
static void readant(prcopt_t* opt, nav_t* nav) {
    const pcv_t pcv0 = {0};
    pcvs_t      pcvr = {0}, pcvs = {0};
    pcv_t*      pcv;
    gtime_t     time = timeget();
    int         i;

    trace(3, "readant:\n");

    opt->pcvr[0] = opt->pcvr[1] = pcv0;
    if (!*filopt.rcvantp) return;

    if (readpcv(filopt.rcvantp, &pcvr)) {
        for (i = 0; i < 2; i++) {
            if (!*opt->anttype[i]) continue;
            if (!(pcv = searchpcv(0, opt->anttype[i], time, &pcvr))) {
                trace(2, "no antenna %s in %s", opt->anttype[i], filopt.rcvantp);
                continue;
            }
            opt->pcvr[i] = *pcv;
        }
    } else
        trace(2, "antenna file open error %s", filopt.rcvantp);

    if (readpcv(filopt.satantp, &pcvs)) {
        for (i = 0; i < MAXSAT; i++) {
            if (!(pcv = searchpcv(i + 1, "", time, &pcvs))) continue;
            nav->pcvs[i] = *pcv;
        }
    } else
        trace(2, "antenna file open error %s", filopt.satantp);

    free(pcvr.pcv);
    free(pcvs.pcv);
}

static Solution decode_solution(const sol_t& sol) {
    double bl[3]  = {0};
    double enu[3] = {0}, pitch = 0.0, yaw = 0.0, len = 0.0;
    int    i;

    Solution ret{};

    using XYZ = Solution::XYZ;
    using LLH = Solution::LLH;

    /* Convert GPS time to UTC calendar time */
    double epoch[6];
    time2epoch(sol.time, epoch);

    ret.time = {
        .year  = static_cast<uint32_t>(epoch[0]),
        .month = static_cast<uint32_t>(epoch[1]),
        .day   = static_cast<uint32_t>(epoch[2]),
        .hour  = static_cast<uint32_t>(epoch[3]),
        .min   = static_cast<uint32_t>(epoch[4]),
        .sec   = epoch[5],
    };

    ret.status = static_cast<Solution::Status>(sol.stat);

    ret.xyz = XYZ{.x = sol.rr[0], .y = sol.rr[1], .z = sol.rr[2]};

    ret.std = XYZ{.x = std::sqrt(sol.qr[0]), .y = std::sqrt(sol.qr[1]), .z = std::sqrt(sol.qr[2])};

    if (norm(sol.rr, 3) > 0.0) {
        double pos[3]{};
        ecef2pos(sol.rr, pos);

        ret.llh = LLH{.lat = pos[0], .lon = pos[1], .height = pos[2]};
    }

    ret.age   = sol.age;
    ret.ratio = sol.ratio;
    ret.ns    = sol.ns;

    return ret;
}

using json = nlohmann::json;

static void to_json(json& j, const Solution::Time& time) {
    j = json{{"year", time.year}, {"month", time.month}, {"day", time.day},
             {"hour", time.hour}, {"min", time.min},     {"sec", time.sec}};
}

static void to_json(json& j, const Solution::LLH& llh) {
    j = json{
        {"lat", llh.lat},
        {"lon", llh.lon},
        {"height", llh.height},
    };
}

static void to_json(json& j, const Solution::XYZ& xyz) {
    j = json{
        {"x", xyz.x},
        {"y", xyz.y},
        {"z", xyz.z},
    };
}

void to_json(json& j, const Solution& solution) {
    j = json{{"time", solution.time},   {"status", solution.status}, {"llh", solution.llh},
             {"xyz", solution.xyz},     {"std", solution.std},       {"age", solution.age},
             {"ratio", solution.ratio}, {"ns", solution.ns}};
}

Rtkrcv::Rtkrcv(const char* conf_file, const char* trace_file, int trace_level) {
    trace(3, "rtkrcv_init:\n");

    /* initialize trace */
    if (trace_file && trace_level > 0) {
        traceopen(trace_file);
        tracelevel(trace_level);
    }

    /* initialize rtk server and monitor port */
    rtksvrinit(&svr);

    /* load options file */
    assert(conf_file && *conf_file);
    resetsysopts();
    if (!loadopts(conf_file, rcvopts) || !loadopts(conf_file, sysopts)) {
        throw std::runtime_error("Failed to load options file");
    }

    getsysopts(&prcopt, solopt, &filopt);
    trace(1, "rtkrcv initialized with config: %s\n", conf_file);
}

Rtkrcv::~Rtkrcv() {
    trace(3, "rtkrcv_cleanup:\n");

    if (svr.state) stop();
    traceclose();
}

void Rtkrcv::start() {
    static sta_t sta[MAXRCV] = {{""}};
    const char*  cmds[]      = {"", "", ""};
    double       pos[3], npos[3];
    const char*  ropts[]      = {"", "", ""};
    const char*  paths[]      = {strpath[0], strpath[1], strpath[2], strpath[3],
                                 strpath[4], strpath[5], strpath[6], strpath[7]};
    char         errmsg[2048] = "";
    int          i, stropt[8] = {0};

    trace(3, "rtkrcv_start:\n");

    error_msg[0] = '\0';

    if (prcopt.refpos == 4) { /* rtcm */
        for (i = 0; i < 3; i++) prcopt.rb[i] = 0.0;
    }
    pos[0] = nmeapos[0] * D2R;
    pos[1] = nmeapos[1] * D2R;
    pos[2] = nmeapos[2];
    pos2ecef(pos, npos);

    /* read antenna file */
    readant(&prcopt, &svr.nav);

    /* read dcb file */
    if (*filopt.dcb) {
        strcpy(sta[0].name, sta_name);
        readdcb(filopt.dcb, &svr.nav, sta);
    }
    /* open geoid data file */
    if (solopt[0].geoid > 0 && !opengeoid(solopt[0].geoid, filopt.geoid)) {
        trace(2, "geoid data open error: %s\n", filopt.geoid);
    }

    /* set stream options */
    stropt[0] = timeout;
    stropt[1] = reconnect;
    stropt[2] = 1000;
    stropt[3] = buffsize;
    stropt[4] = fswapmargin;
    strsetopt(stropt);

    if (strfmt[2] == 8) strfmt[2] = STRFMT_SP3;

    /* set ftp/http directory and proxy */
    strsetdir(filopt.tempdir);
    strsetproxy(proxyaddr);

    solopt[0].posf = strfmt[3];
    solopt[1].posf = strfmt[4];

    /* start rtk server */
    if (!rtksvrstart(&svr, svrcycle, buffsize, strtype, paths, strfmt, navmsgsel, cmds, cmds, ropts,
                     nmeacycle, nmeareq, npos, &prcopt, solopt, nullptr, errmsg)) {
        throw std::runtime_error(std::format("RTK server start error: {}", errmsg));
    }

    trace(1, "RTK server started\n");
}

void Rtkrcv::stop() {
    const char* cmds[] = {nullptr, nullptr, nullptr};

    trace(3, "rtkrcv_stop:\n");

    if (!svr.state) return;

    /* stop rtk server */
    rtksvrstop(&svr, cmds);

    if (solopt[0].geoid > 0) closegeoid();

    trace(1, "RTK server stopped\n");
}

std::string Rtkrcv::get_error() {
    rtksvrlock(&svr);
    std::string ret;
    if (svr.rtk.neb > 0) {
        svr.rtk.errbuf[svr.rtk.neb] = '\0';

        ret         = svr.rtk.errbuf;
        svr.rtk.neb = 0;
    }
    rtksvrunlock(&svr);
    return ret;
}

Solution Rtkrcv::get_sol() {
    sol_t sol;
    rtksvrlock(&svr);
    sol = svr.rtk.sol;
    rtksvrunlock(&svr);
    return decode_solution(sol);
}

bool Rtkrcv::is_running() const { return svr.state; }

/**
 * @brief 安全加载星历数据到 Server
 * * 采用“中转法”：
 * 1. 先读入临时 nav_t (允许动态内存分配，避免 readrnxt 破坏 server 内存布局)
 * 2. 在临时内存中排序去重 (uniqnav)
 * 3. 将有效数据精准填入 server_nav 的固定槽位中
 * * @param file 星历文件路径
 * @param server_nav 目标 Server 的导航数据结构 (引用传递)
 */
static void load_empharis(const char* file, nav_t& server_nav) {
    assert(server_nav.eph);

    // 1. 创建一个临时的 nav，用于读取文件
    // 初始化为 0，readrnxt 会根据文件内容自动 malloc 内存
    nav_t temp_nav = {0};

    // 2. 读取文件到临时 nav (这里会作为动态列表处理)
    trace(3, "Loading %s into temp storage...\n", file);

    gtime_t ts{0, 0.0};
    gtime_t te{0, 0.0};
    sta_t   sta = {0};  // 必须初始化，防止 readrnxt 读取垃圾栈内存

    // 调用核心读取函数
    int status = readrnxt(file, 0, ts, te, 0.0, "", nullptr, &temp_nav, &sta);

    if (status <= 0) {
        trace(2, "Warning: No ephemeris data loaded from %s (Code: %d)\n", file, status);
        freenav(&temp_nav, 0xFF);
        return;
    }

    // 3. 对临时数据去重和排序
    // 这一步非常关键，它将 readrnxt 读取的杂乱数据整理好
    uniqnav(&temp_nav);
    trace(3, "Loaded %d unique eph records, %d unique geph records.\n", temp_nav.n, temp_nav.ng);

    // 4. [关键步骤] 将临时数据映射到 Server 的“固定座位”上

    // --- A. 处理通用星历 (GPS, GAL, QZS, BDS, IRN, SBS) ---
    // RTKLIB 规则: eph 数组大小为 MAXSAT，索引为 sat - 1
    for (int i = 0; i < temp_nav.n; i++) {
        eph_t* src = &temp_nav.eph[i];
        int    sat = src->sat;

        // 边界检查：防止越界写入踩坏堆
        if (sat > 0 && sat <= MAXSAT) {
            // 深拷贝数据到 Server 的当前集合
            // rtksvr 可能会维护多套星历，这里我们更新默认集合 (set 0)
            server_nav.eph[sat - 1] = *src;
        }
    }

    // --- B. 处理 GLONASS 星历 (geph) ---
    // RTKLIB 规则: GLONASS 存储在 geph 数组中
    // 索引通常基于 PRN (1~24)，而不是全局 SAT ID
    if (temp_nav.ng > 0 && server_nav.geph) {
        for (int i = 0; i < temp_nav.ng; i++) {
            geph_t* src = &temp_nav.geph[i];
            int     sat = src->sat;
            int     prn;

            // 获取卫星系统和 PRN
            int sys = satsys(sat, &prn);

            if (sys == SYS_GLO) {
                // GLONASS 的 geph 数组大小通常是 NSATGLO
                // 索引方式: prn - 1
                // 注意：这里需要确保 prn 在合法范围内
                if (prn > 0 && prn <= NSATGLO) {
                    server_nav.geph[prn - 1] = *src;
                } else {
                    trace(2, "Warning: GLONASS PRN out of range: %d\n", prn);
                }
            }
        }
    }

    // --- C. 处理 SBAS 长期星历 (seph) [可选] ---
    // 仅当你的 RTKLIB 版本启用且使用了 seph 时需要
    if (temp_nav.ns > 0 && server_nav.seph) {
        for (int i = 0; i < temp_nav.ns; i++) {
            seph_t* src = &temp_nav.seph[i];
            int     sat = src->sat;
            int     prn;
            int     sys = satsys(sat, &prn);
            if (sys == SYS_SBS && prn > 0 && prn <= NSATSBS) {
                server_nav.seph[prn - 1] = *src;
            }
        }
    }

    trace(3, "Transfer to server nav complete.\n");
    freenav(&temp_nav, 0xFF);
}

#ifndef NO_MAIN
static std::atomic<bool> g_exit_flag{false};

static void handle_signal(int) { g_exit_flag = true; }

int main() {
    try {
        Rtkrcv service{"conf", "trace", 3};
        load_empharis("empharis.nav", svr.nav);
        service.start();

        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);

        while (!g_exit_flag) {
            using namespace std::literals::chrono_literals;
            std::this_thread::sleep_for(500ms);
            std::cout << service.get_error();
        }

        service.stop();
    } catch (std::exception& err) {
        std::cerr << "Errors occured: " << err.what() << std::endl;
    }
    return 0;
}
#endif
