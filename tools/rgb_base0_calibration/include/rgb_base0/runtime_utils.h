#pragma once

#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace rgb_base0 {

class Logger final {
public:
    explicit Logger(const std::filesystem::path& path, bool append = false);
    void line(const std::string& message);

private:
    std::ofstream output_;
};

std::string utcIsoTimestamp();
std::string localCompactTimestamp();
double parseFiniteDouble(const std::string& value, const std::string& label);
std::string quoteWindowsArgument(const std::string& value);
std::vector<std::vector<std::string>> readCsv(const std::filesystem::path& path);
std::map<std::string, std::size_t> csvHeaderMap(const std::vector<std::string>& header);

}  // namespace rgb_base0
