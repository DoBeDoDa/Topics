#include "rgb_base0/runtime_utils.h"

#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace rgb_base0 {
namespace {

std::tm utcTime(const std::time_t value) {
    std::tm result{};
    if(gmtime_s(&result, &value) != 0) {
        throw std::runtime_error("Failed to convert UTC timestamp");
    }
    return result;
}

std::tm localTime(const std::time_t value) {
    std::tm result{};
    if(localtime_s(&result, &value) != 0) {
        throw std::runtime_error("Failed to convert local timestamp");
    }
    return result;
}

}  // namespace

Logger::Logger(const std::filesystem::path& path, const bool append) {
    std::filesystem::create_directories(path.parent_path());
    output_.open(path, std::ios::binary | (append ? std::ios::app : std::ios::trunc));
    if(!output_) {
        throw std::runtime_error("Cannot create terminal log: " + path.string());
    }
}

void Logger::line(const std::string& message) {
    std::cout << message << '\n';
    std::cout.flush();
    output_ << message << '\n';
    output_.flush();
    if(!output_) {
        throw std::runtime_error("Failed while writing terminal log");
    }
}

std::string utcIsoTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm converted = utcTime(time);
    std::ostringstream result;
    result << std::put_time(&converted, "%Y-%m-%dT%H:%M:%SZ");
    return result.str();
}

std::string localCompactTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm converted = localTime(time);
    std::ostringstream result;
    result << std::put_time(&converted, "%Y%m%d_%H%M%S");
    return result.str();
}

double parseFiniteDouble(const std::string& value, const std::string& label) {
    std::size_t consumed = 0;
    double parsed = 0.0;
    try {
        parsed = std::stod(value, &consumed);
    }
    catch(const std::exception&) {
        throw std::runtime_error(label + " must be a finite number; got: " + value);
    }
    if(consumed != value.size() || !std::isfinite(parsed)) {
        throw std::runtime_error(label + " must be a finite number; got: " + value);
    }
    return parsed;
}

std::string quoteWindowsArgument(const std::string& value) {
    std::string quoted = "\"";
    std::size_t backslashes = 0;
    for(const char character : value) {
        if(character == '\\') {
            ++backslashes;
        }
        else if(character == '"') {
            quoted.append(backslashes * 2 + 1, '\\');
            quoted.push_back('"');
            backslashes = 0;
        }
        else {
            quoted.append(backslashes, '\\');
            backslashes = 0;
            quoted.push_back(character);
        }
    }
    quoted.append(backslashes * 2, '\\');
    quoted.push_back('"');
    return quoted;
}

std::vector<std::vector<std::string>> readCsv(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if(!input) {
        throw std::runtime_error("Cannot open CSV: " + path.string());
    }
    std::vector<std::vector<std::string>> rows;
    std::vector<std::string> row;
    std::string field;
    bool quoted = false;
    char character = 0;
    while(input.get(character)) {
        if(quoted) {
            if(character == '"') {
                if(input.peek() == '"') {
                    input.get(character);
                    field.push_back('"');
                }
                else {
                    quoted = false;
                }
            }
            else {
                field.push_back(character);
            }
        }
        else if(character == '"' && field.empty()) {
            quoted = true;
        }
        else if(character == ',') {
            row.push_back(field);
            field.clear();
        }
        else if(character == '\n') {
            if(!field.empty() && field.back() == '\r') {
                field.pop_back();
            }
            row.push_back(field);
            field.clear();
            rows.push_back(row);
            row.clear();
        }
        else {
            field.push_back(character);
        }
    }
    if(quoted) {
        throw std::runtime_error("Unterminated quoted CSV field: " + path.string());
    }
    if(!field.empty() || !row.empty()) {
        row.push_back(field);
        rows.push_back(row);
    }
    if(!input.eof()) {
        throw std::runtime_error("Failed while reading CSV: " + path.string());
    }
    if(!rows.empty() && !rows.front().empty() && rows.front().front().size() >= 3
       && static_cast<unsigned char>(rows.front().front()[0]) == 0xEF
       && static_cast<unsigned char>(rows.front().front()[1]) == 0xBB
       && static_cast<unsigned char>(rows.front().front()[2]) == 0xBF) {
        rows.front().front().erase(0, 3);
    }
    return rows;
}

std::map<std::string, std::size_t> csvHeaderMap(const std::vector<std::string>& header) {
    std::map<std::string, std::size_t> result;
    for(std::size_t index = 0; index < header.size(); ++index) {
        if(header[index].empty() || !result.emplace(header[index], index).second) {
            throw std::runtime_error("CSV header contains an empty or duplicate column");
        }
    }
    return result;
}

}  // namespace rgb_base0
