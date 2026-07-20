// 解析 Python 傳來的 32 個 CSV 座標值並產生 TableState。
#include "VisionDataParser.h"

#include <sstream>
#include <vector>

namespace {

const double MISSING_COORDINATE_LIMIT = -9000.0;

DetectedPoint makeDetectedPoint(double x, double y) {
    DetectedPoint point;
    point.detected = x > MISSING_COORDINATE_LIMIT && y > MISSING_COORDINATE_LIMIT;
    point.position = {x, y};
    return point;
}

}  // namespace

bool VisionDataParser::parse(
    const std::string& message,
    TableState& output,
    std::string& error
) const {
    std::stringstream stream(message);
    std::string token;
    std::vector<double> values;

    try {
        while (std::getline(stream, token, ',')) {
            if (!token.empty()) {
                values.push_back(std::stod(token));
            }
        }
    } catch (const std::exception&) {
        error = "影像資料包含無法解析的數值。";
        return false;
    }

    if (values.size() != 32) {
        error = "影像資料欄位數錯誤，預期 32 個數值，實際收到 " +
            std::to_string(values.size()) + " 個。";
        return false;
    }

    for (std::size_t index = 0; index < output.objectBalls.size(); ++index) {
        output.objectBalls[index] = makeDetectedPoint(
            values[index * 2], values[index * 2 + 1]
        );
    }

    output.cueBall = makeDetectedPoint(values[18], values[19]);

    for (std::size_t index = 0; index < output.pockets.size(); ++index) {
        const std::size_t valueIndex = 20 + index * 2;
        output.pockets[index] = makeDetectedPoint(
            values[valueIndex], values[valueIndex + 1]
        );
    }

    error.clear();
    return true;
}
