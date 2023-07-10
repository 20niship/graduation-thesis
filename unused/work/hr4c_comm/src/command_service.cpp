#include <iostream>
#include <nlohmann/json.hpp>
#include "command_service.h"

using namespace hr4c_comm::command_service;
using json = nlohmann::json;

int checkJsonKey(json input, const string& key) {
    if (input.find(key) == input.end()) {
        cerr << "No '" << key << "' key in data" << endl;
        return -1;
    }
    return 0;
}

void JSONCommandService::createCommandString(const string& command, vector<double>& args, string& output_str) {
    json output_json;
    output_json["command"] = command;
    output_json["args"] = args;

    output_str = output_json.dump();
}

void JSONCommandService::parseCommandResponse(const string &input_str,
                                              string &command, string &response,
                                              vector<double> &result, vector<string> &result_str) {
    constexpr auto Command = "command";
    constexpr auto Response = "response";
    constexpr auto Result = "result";
    constexpr auto ResultStr = "result_str";
    string trimmed_str(input_str);
    trimmed_str.erase(trimmed_str.find_first_of('}') + 1);
    try {
        json input_json = json::parse(trimmed_str);

        // バリデーション
        if (checkJsonKey(input_json, Command) < 0) return;
        if (checkJsonKey(input_json, Response) < 0) return;
        if (checkJsonKey(input_json, Result) < 0) return;
        if (checkJsonKey(input_json, ResultStr) < 0) return;

        // 値のパージング
        if (input_json[Response] == ResponseOK) {
            response = ResponseOK;
            command = input_json[Command].get<string>();
            if (!input_json[Result].is_null() && input_json[Result].is_array()) {
                result.clear();
                for (auto &element : input_json[Result]) {
                    result.push_back(element);
                }
            }
            if (!input_json[ResultStr].is_null() && input_json[ResultStr].is_array()) {
                result_str.clear();
                for (auto &element : input_json[ResultStr]) {
                    result_str.push_back(element);
                }
            }
        } else {
            response = ResponseNG;
        }
    }
    catch (...)
    {
        cerr << "Parse Error: " << trimmed_str << endl;
    }
}
