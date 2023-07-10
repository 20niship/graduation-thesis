#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include "net/telegram_parser.h"
#include "utils/hr4c_utils.h"

using namespace hr4c::telegram;
using namespace hr4c::utils;
using json = nlohmann::json;

int checkJsonKey(json input, const string& key) {
    if (input.find(key) == input.end()) {
        spdlog::error("No '" + key + "' key in data");
        return -1;
    }
    return 0;
}

void JSONTelegramParser::parseTelegrams(const string& input_str, vector<Command>& commands) {
    constexpr auto COMMAND = "command";
    constexpr auto ARGS = "args";
    string trimmed_str(input_str);
    trimmed_str.erase(trimmed_str.find_last_of('}') + 1);
    vector<string> json_strs = Split(trimmed_str, '}');

    for (auto& json_str : json_strs) {
        try {
            json input_json = json::parse(json_str);
            // バリデーション
            if (checkJsonKey(input_json, COMMAND) < 0) continue;
            if (checkJsonKey(input_json, ARGS) < 0) continue;

            // 値のパージング
            Command new_cmd;
            new_cmd.command = input_json[COMMAND].get<string>();
            spdlog::debug("parsing: " + new_cmd.command);
            if (!input_json[ARGS].is_null() && input_json[ARGS].is_array()) {
                for (auto& element : input_json[ARGS]) {
                    new_cmd.args.push_back(element);
                }
            }
            commands.push_back(new_cmd);
        }
        catch (...) {
            spdlog::error("Parse Error: " + json_str);
        }
    }
}

void JSONTelegramParser::createResponseString(const string& command,
                                              string& response, vector<double>& result,
                                              vector<string>& result_str, string& output_str) {
    json output_json;
    output_str.clear();

    output_json["command"] = command;
    output_json["response"] = ResponseOK;
    if (!result.empty()) {
        output_json["result"] = result;
    } else {
        output_json["result"] = nullptr;
    }
    if (!result_str.empty()) {
        output_json["result_str"] = result_str;
    } else {
        output_json["result_str"] = nullptr;
    }

    output_str = output_json.dump();
    spdlog::debug(output_str);
}
