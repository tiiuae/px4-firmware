#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/satellite_info.h>
#include <uORB/topics/mavlink_log.h>

#include "sh1106.hpp"
#include <cstdio>

using namespace time_literals;

class Sh1106Notify : public ModuleBase<Sh1106Notify>, public px4::ScheduledWorkItem
{
public:
    Sh1106Notify(int i2c_bus, int i2c_addr) :
        ModuleBase<Sh1106Notify>(),
        ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
        _display(i2c_bus, i2c_addr)
    {}

    ~Sh1106Notify() override = default;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
    static int print_usage(const char *reason = nullptr);

    bool init();

private:
    void Run() override;

    SH1106 _display;

    uORB::Subscription _veh_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _batt_sub{ORB_ID(battery_status)};
    uORB::Subscription _tel_sub{ORB_ID(telemetry_status)};
    uORB::Subscription _sat_sub{ORB_ID(satellite_info)};
    uORB::Subscription _log_sub{ORB_ID(mavlink_log)};

    vehicle_status_s   _vs{};
    battery_status_s   _bs{};
    telemetry_status_s _ts{};
    satellite_info_s   _si{};
    mavlink_log_s      _ml{};

    uint64_t _last_log_ts{0};
    char _last_log_line[22]{}; // one OLED line

    const char *mode_name(uint8_t nav_state);
};

bool Sh1106Notify::init()
{
    if (_display.init() != PX4_OK) {
        PX4_ERR("display init failed");
        return false;
    }

    _display.print_line(0, "SH1106 Notify");
    _display.print_line(1, "PX4 starting...");
    ScheduleOnInterval(100_ms);
    return true;
}

void Sh1106Notify::Run()
{
    // VEHICLE STATUS
    if (_veh_status_sub.update(&_vs)) {
        // Nothing extra here; fields used below
    }

    // BATTERY
    if (_batt_sub.update(&_bs)) {
        // Nothing extra here; fields used below
    }

    // TELEMETRY
    if (_tel_sub.update(&_ts)) {
        // Nothing extra here; fields used below
    }

    // GPS sats (optional)
    if (_sat_sub.update(&_si)) {
        // Nothing extra; fields used below
    }

    // MAVLink log text (last line)
    if (_log_sub.update(&_ml)) {
        if (_ml.timestamp > _last_log_ts) {
            _last_log_ts = _ml.timestamp;
            // Truncate to fit 21 chars
            snprintf(_last_log_line, sizeof(_last_log_line), "%.*s", 21, _ml.text);
        }
    }

    // ----- Compose lines -----
    // L1: ARM + MODE
    char l1[22];
    const char *arm = (_vs.arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? "ARMED" : "DISARMED";
    snprintf(l1, sizeof(l1), "%-7s %s", arm, mode_name(_vs.nav_state));
    _display.print_line(0, l1);

    // L2: Battery (V & % & warn)
    char l2[22];
    float v = _bs.voltage_v;
    float rem_pct = (_bs.remaining >= 0.f && _bs.remaining <= 1.f) ? _bs.remaining * 100.f : -1.f;
    const char *warn = "OK";
    switch (_bs.warning) {
        case battery_status_s::WARNING_LOW: warn = "LOW"; break;
        case battery_status_s::WARNING_CRITICAL: warn = "CRIT"; break;
        case battery_status_s::WARNING_EMERGENCY: warn = "EMERG"; break;
        case battery_status_s::WARNING_FAILED: warn = "FAIL"; break;
        default: break;
    }
    if (rem_pct >= 0.f)
        snprintf(l2, sizeof(l2), "Bat %4.2fV %3.0f%% %s", (double)v, (double)rem_pct, warn);
    else
        snprintf(l2, sizeof(l2), "Bat %4.2fV  -- %% %s", (double)v, warn);
    _display.print_line(1, l2);

    // L3: Link + RSSI (if any)
    char l3[22];
    const char *gcs = _vs.gcs_connection_lost ? "GCS LOST" : "GCS OK";
    // TelemetryStatus RSSI is implementation-specific; show if non-zero
    int rssi = (int)_ts.rssi;
    if (rssi != 0) snprintf(l3, sizeof(l3), "%-8s RSSI:%3d", gcs, rssi);
    else           snprintf(l3, sizeof(l3), "%-8s", gcs);
    _display.print_line(2, l3);

    // L4: GPS sats if present OR last log
    char l4[22];
    if (_si.timestamp != 0 && _si.count > 0) {
        snprintf(l4, sizeof(l4), "GPS sats:%u", (unsigned)_si.count);
        _display.print_line(3, l4);
    } else if (_last_log_ts != 0) {
        _display.print_line(3, _last_log_line);
    } else {
        _display.print_line(3, " ");
    }
}

const char* Sh1106Notify::mode_name(uint8_t ns)
{
    using VS = vehicle_status_s;
    switch (ns) {
        case VS::NAVIGATION_STATE_MANUAL: return "MANUAL";
        case VS::NAVIGATION_STATE_ALTCTL: return "ALTCTL";
        case VS::NAVIGATION_STATE_POSCTL: return "POSCTL";
        case VS::NAVIGATION_STATE_AUTO_MISSION: return "MISSION";
        case VS::NAVIGATION_STATE_AUTO_LOITER: return "LOITER";
        case VS::NAVIGATION_STATE_AUTO_RTL: return "RTL";
        case VS::NAVIGATION_STATE_ACRO: return "ACRO";
        case VS::NAVIGATION_STATE_OFFBOARD: return "OFFBOARD";
        case VS::NAVIGATION_STATE_STAB: return "STAB";
        case VS::NAVIGATION_STATE_AUTO_TAKEOFF: return "TAKEOFF";
        case VS::NAVIGATION_STATE_AUTO_LAND: return "LAND";
        case VS::NAVIGATION_STATE_ORBIT: return "ORBIT";
        default: return "MODE?";
    }
}

int Sh1106Notify::task_spawn(int argc, char *argv[])
{
    int i2c_bus = 1;      // default external bus (adjust for board)
    int i2c_addr = SH1106_I2C_ADDR;

    int ch;
    while ((ch = px4_getopt(argc, argv, "b:a:", nullptr, nullptr)) != EOF) {
        switch (ch) {
        case 'b': i2c_bus = atoi(px4_optarg); break;
        case 'a': i2c_addr = strtol(px4_optarg, nullptr, 0); break;
        default: return print_usage("unrecognized option");
        }
    }

    Sh1106Notify *inst = new Sh1106Notify(i2c_bus, i2c_addr);
    if (!inst) return PX4_ERROR;

    _object.store(inst);
    _task_id = task_id_is_work_queue;

    if (!inst->init()) {
        PX4_ERR("init failed");
        delete inst;
        _object.store(nullptr);
        return PX4_ERROR;
    }
    return PX4_OK;
}

int Sh1106Notify::print_usage(const char *reason)
{
    if (reason) { PX4_WARN("%s", reason); }
    PRINT_MODULE_DESCRIPTION(
        R"DESCR(
### Description
SH1106 OLED notifier that mirrors ArduPilot AP_Notify display behavior on PX4.

### Examples
Start on I2C bus 1, address 0x3C:
  sh1106_notify start -b 1 -a 0x3C

Stop:
  sh1106_notify stop

Status:
  sh1106_notify status
)DESCR");

    PRINT_MODULE_USAGE_NAME("sh1106_notify", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_INT('b', 1, 0, 10, "I2C bus", true);
    PRINT_MODULE_USAGE_PARAM_INT('a', SH1106_I2C_ADDR, 0x00, 0x7F, "I2C addr", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

// Boilerplate
extern "C" __EXPORT int sh1106_notify_main(int argc, char *argv[]) { return Sh1106Notify::main(argc, argv); }
