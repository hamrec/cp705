#include "json_parser.h"
#include "json.hpp"
#include <fstream>
#include <iostream>

using json = nlohmann::json;

// ---------- Variable substitution ----------

static void substitute(std::string& str, const TestConfig& cfg) {
    auto replace_all = [&](const std::string& var, const std::string& val) {
        size_t pos = 0;
        while ((pos = str.find(var, pos)) != std::string::npos) {
            str.replace(pos, var.size(), val);
            pos += val.size();
        }
    };
    replace_all("${MY_CALLSIGN}", cfg.my_callsign);
    replace_all("${MY_GRID}",     cfg.my_grid);
    replace_all("${DX_CALLSIGN}", cfg.dx_callsign);
    replace_all("${DX_GRID}",     cfg.dx_grid);
}

// ---------- Parser ----------

bool load_test_data(const std::string& filename, TestData& out) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: cannot open '" << filename << "'\n";
        return false;
    }

    json j;
    try { file >> j; }
    catch (const std::exception& e) {
        std::cerr << "JSON parse error: " << e.what() << "\n";
        return false;
    }

    // --- config ---
    if (j.contains("config")) {
        auto& c = j["config"];
        out.config.my_callsign = c.value("MY_CALLSIGN", "");
        out.config.my_grid     = c.value("MY_GRID", "");
        out.config.dx_callsign = c.value("DX_CALLSIGN", "");
        out.config.dx_grid     = c.value("DX_GRID", "");
        out.config.tx_on_even  = c.value("TX_ON_EVEN", false);
        out.config.cq_type    = c.value("CQ_TYPE", "");
        out.config.free_text  = c.value("FREE_TEXT", "");
        out.config.max_retry  = c.value("MAX_RETRY", 5);
    }

    // --- periods ---
    if (!j.contains("periods")) {
        std::cerr << "Warning: no 'periods' in JSON\n";
        return true;
    }

    for (auto& pj : j["periods"]) {
        TestPeriod period;

        // messages
        if (pj.contains("messages")) {
            for (auto& mj : pj["messages"]) {
                UiRxLine rx;

                std::string call_to   = mj.value("call_to", "");
                std::string call_from = mj.value("call_from", "");
                std::string field3    = mj.value("locator", "");

                substitute(call_to,   out.config);
                substitute(call_from, out.config);
                substitute(field3,    out.config);

                rx.field1 = call_to;
                rx.field2 = call_from;
                rx.field3 = field3;
                rx.snr       = mj.value("snr", 0);
                rx.offset_hz = mj.value("freq_hz", 1500);
                rx.slot_id   = mj.value("slot", 0);

                // Build human-readable text for display
                rx.text = call_to + " " + call_from + " " + field3;

                // is_cq: field1 == "CQ" or starts with "CQ "
                rx.is_cq = (call_to == "CQ" || call_to.substr(0, 3) == "CQ ");

                // is_to_me: field1 matches my callsign
                rx.is_to_me = (call_to == out.config.my_callsign);

                period.messages.push_back(std::move(rx));
            }
        }

        // beacon_change
        if (pj.contains("beacon_change")) {
            auto& bc = pj["beacon_change"];
            period.has_beacon_change = true;
            period.beacon_change.time_offset = bc.value("time_offset", 0.0f);
            period.beacon_change.beacon_on   = bc.value("beacon_on", 0);
        }

        // touch_event
        if (pj.contains("touch_event")) {
            auto& te = pj["touch_event"];
            period.has_touch_event = true;
            period.touch_event.time_offset   = te.value("time_offset", 0.0f);
            period.touch_event.message_index = te.value("message_index", 0);
        }

        // freetext_event (inject Free Text via autoseq_schedule_freetext)
        if (pj.contains("freetext_event")) {
            auto& fe = pj["freetext_event"];
            period.has_freetext_event = true;
            period.freetext_event.time_offset = fe.value("time_offset", 0.0f);
            period.freetext_event.text        = fe.value("text", std::string(""));
        }

        out.periods.push_back(std::move(period));
    }

    std::cout << "Loaded " << out.periods.size() << " periods, station "
              << out.config.my_callsign << "/" << out.config.my_grid << "\n";
    return true;
}
