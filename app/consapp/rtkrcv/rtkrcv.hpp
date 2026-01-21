#pragma once

#include <string>

#include "json.hpp"

struct Solution {
    struct Time {
        uint32_t year;
        uint32_t month;
        uint32_t day;
        uint32_t hour;
        uint32_t min;
        double   sec;
    };
    struct LLH {
        double lat;
        double lon;
        double height;
    };
    struct XYZ {
        double x;
        double y;
        double z;
    };
    enum Status : uint8_t { NONE = 0, FIX, FLOAT, SBAS, DGPS, SINGLE, PPP };

    Time   time;
    Status status;  // solution status=

    LLH llh;  // latitude, longtitude, height
    XYZ xyz;  // ECEF XYZ
    XYZ std;  // Standard deviations

    double age;    // age of differential (s)
    double ratio;  // AR ratio factor
    int    ns;     // number of satellites
};

void to_json(nlohmann::json&, const Solution&);

class Rtkrcv {
   public:
    Rtkrcv(const char* conf_file, const char* trace_file, int trace_level);
    ~Rtkrcv();
    void               start();
    void               stop();
    std::string        get_error();
    Solution           get_sol();
    [[nodiscard]] bool is_running() const;
};
