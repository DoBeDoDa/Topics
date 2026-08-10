#include "rgb_base0/calibration_io.h"

#include "rgb_base0/geometry.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace rgb_base0 {
namespace {

std::string escapeJson(const std::string& value) {
    std::ostringstream escaped;
    for(const char character : value) {
        switch(character) {
        case '\\': escaped << "\\\\"; break;
        case '"': escaped << "\\\""; break;
        case '\n': escaped << "\\n"; break;
        case '\r': escaped << "\\r"; break;
        case '\t': escaped << "\\t"; break;
        default: escaped << character; break;
        }
    }
    return escaped.str();
}

std::string quoteYaml(const std::string& value) {
    return "\"" + escapeJson(value) + "\"";
}

std::string readAll(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if(!input) {
        throw std::runtime_error("Cannot open calibration file: " + path.string());
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    if(!input.good() && !input.eof()) {
        throw std::runtime_error("Failed while reading calibration file: " + path.string());
    }
    return buffer.str();
}

std::string trim(std::string value) {
    const auto notSpace = [](const unsigned char character) { return std::isspace(character) == 0; };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), notSpace));
    value.erase(std::find_if(value.rbegin(), value.rend(), notSpace).base(), value.end());
    return value;
}

std::string unescapeQuoted(std::string value) {
    value = trim(value);
    if(value.size() < 2 || value.front() != '"' || value.back() != '"') {
        throw std::runtime_error("Expected a quoted string, got: " + value);
    }
    std::string output;
    bool escaped = false;
    for(std::size_t index = 1; index + 1 < value.size(); ++index) {
        const char character = value[index];
        if(escaped) {
            switch(character) {
            case 'n': output.push_back('\n'); break;
            case 'r': output.push_back('\r'); break;
            case 't': output.push_back('\t'); break;
            case '\\': output.push_back('\\'); break;
            case '"': output.push_back('"'); break;
            default: throw std::runtime_error("Unsupported string escape in calibration file");
            }
            escaped = false;
        }
        else if(character == '\\') {
            escaped = true;
        }
        else {
            output.push_back(character);
        }
    }
    if(escaped) {
        throw std::runtime_error("Unterminated string escape in calibration file");
    }
    return output;
}

std::string jsonRawValue(const std::string& text, const std::string& key) {
    const std::string token = "\"" + key + "\"";
    const std::size_t keyPosition = text.find(token);
    if(keyPosition == std::string::npos || text.find(token, keyPosition + token.size()) != std::string::npos) {
        throw std::runtime_error("JSON calibration field must occur exactly once: " + key);
    }
    const std::size_t colon = text.find(':', keyPosition + token.size());
    if(colon == std::string::npos) {
        throw std::runtime_error("Missing ':' after JSON calibration field: " + key);
    }
    std::size_t start = colon + 1;
    while(start < text.size() && std::isspace(static_cast<unsigned char>(text[start])) != 0) {
        ++start;
    }
    if(start >= text.size()) {
        throw std::runtime_error("Missing JSON value for calibration field: " + key);
    }
    if(text[start] == '"') {
        bool escaped = false;
        for(std::size_t index = start + 1; index < text.size(); ++index) {
            if(!escaped && text[index] == '"') {
                return text.substr(start, index - start + 1);
            }
            if(!escaped && text[index] == '\\') {
                escaped = true;
            }
            else {
                escaped = false;
            }
        }
        throw std::runtime_error("Unterminated JSON string for calibration field: " + key);
    }
    if(text[start] == '[') {
        int depth = 0;
        bool inString = false;
        bool escaped = false;
        for(std::size_t index = start; index < text.size(); ++index) {
            const char character = text[index];
            if(inString) {
                if(!escaped && character == '"') {
                    inString = false;
                }
                escaped = !escaped && character == '\\';
                if(character != '\\') {
                    escaped = false;
                }
                continue;
            }
            if(character == '"') {
                inString = true;
            }
            else if(character == '[') {
                ++depth;
            }
            else if(character == ']' && --depth == 0) {
                return text.substr(start, index - start + 1);
            }
        }
        throw std::runtime_error("Unterminated JSON array for calibration field: " + key);
    }
    std::size_t end = start;
    while(end < text.size() && text[end] != ',' && text[end] != '}' && text[end] != '\n' && text[end] != '\r') {
        ++end;
    }
    return trim(text.substr(start, end - start));
}

std::map<std::string, std::string> yamlRawValues(const std::string& text) {
    std::map<std::string, std::string> values;
    std::istringstream input(text);
    std::string line;
    while(std::getline(input, line)) {
        const std::string stripped = trim(line);
        if(stripped.empty() || stripped.front() == '#') {
            continue;
        }
        const std::size_t colon = stripped.find(':');
        if(colon == std::string::npos) {
            throw std::runtime_error("Invalid YAML calibration line: " + stripped);
        }
        const std::string key = trim(stripped.substr(0, colon));
        const std::string value = trim(stripped.substr(colon + 1));
        if(value.empty()) {
            continue;
        }
        if(!values.emplace(key, value).second) {
            throw std::runtime_error("YAML calibration field occurs more than once: " + key);
        }
    }
    return values;
}

std::string yamlRawValue(const std::map<std::string, std::string>& values, const std::string& key) {
    const auto found = values.find(key);
    if(found == values.end()) {
        throw std::runtime_error("Missing YAML calibration field: " + key);
    }
    return found->second;
}

std::vector<double> parseNumberArray(const std::string& raw, const std::size_t expectedCount) {
    std::string flattened;
    flattened.reserve(raw.size());
    for(const char character : raw) {
        if(character != '[' && character != ']') {
            flattened.push_back(character);
        }
    }
    std::vector<double> values;
    std::istringstream input(flattened);
    std::string token;
    while(std::getline(input, token, ',')) {
        token = trim(token);
        if(token.empty()) {
            continue;
        }
        std::size_t consumed = 0;
        const double value = std::stod(token, &consumed);
        if(consumed != token.size() || !std::isfinite(value)) {
            throw std::runtime_error("Invalid numeric array element: " + token);
        }
        values.push_back(value);
    }
    if(values.size() != expectedCount) {
        throw std::runtime_error("Expected " + std::to_string(expectedCount) + " numeric array values, got "
                                 + std::to_string(values.size()));
    }
    return values;
}

double parseDouble(const std::string& raw) {
    std::size_t consumed = 0;
    const double value = std::stod(trim(raw), &consumed);
    if(consumed != trim(raw).size() || !std::isfinite(value)) {
        throw std::runtime_error("Invalid finite floating-point value: " + raw);
    }
    return value;
}

int parseInt(const std::string& raw) {
    std::size_t consumed = 0;
    const int value = std::stoi(trim(raw), &consumed);
    if(consumed != trim(raw).size()) {
        throw std::runtime_error("Invalid integer value: " + raw);
    }
    return value;
}

bool parseBool(const std::string& raw) {
    const std::string value = trim(raw);
    if(value == "true") {
        return true;
    }
    if(value == "false") {
        return false;
    }
    throw std::runtime_error("Invalid Boolean value: " + raw);
}

template <typename RawGetter>
CalibrationData parseCalibration(RawGetter raw) {
    CalibrationData value;
    value.schemaVersion = unescapeQuoted(raw("schema_version"));
    value.createdUtc = unescapeQuoted(raw("timestamp"));
    value.experimental = parseBool(raw("experimental"));
    value.sdkVersion = unescapeQuoted(raw("sdk_version"));
    value.cameraModel = unescapeQuoted(raw("camera_model"));
    value.deviceName = unescapeQuoted(raw("device_name"));
    value.serialNumber = unescapeQuoted(raw("camera_serial_number"));
    value.firmwareVersion = unescapeQuoted(raw("firmware_version"));
    value.profile.width = parseInt(raw("rgb_width"));
    value.profile.height = parseInt(raw("rgb_height"));
    value.profile.fps = parseInt(raw("profile_fps"));
    value.profile.format = unescapeQuoted(raw("profile_format"));
    const std::string expectedProfile = std::to_string(value.profile.width) + "x"
                                        + std::to_string(value.profile.height) + "@"
                                        + std::to_string(value.profile.fps) + " " + value.profile.format;
    if(unescapeQuoted(raw("rgb_profile")) != expectedProfile) {
        throw std::runtime_error("rgb_profile summary is inconsistent with profile fields");
    }
    value.intrinsic = {parseDouble(raw("fx")), parseDouble(raw("fy")),
                       parseDouble(raw("cx")), parseDouble(raw("cy"))};
    const std::vector<double> k = parseNumberArray(raw("K"), 9);
    const std::vector<double> expectedK{value.intrinsic.fx, 0.0, value.intrinsic.cx,
                                        0.0, value.intrinsic.fy, value.intrinsic.cy,
                                        0.0, 0.0, 1.0};
    for(std::size_t index = 0; index < k.size(); ++index) {
        if(std::abs(k[index] - expectedK[index]) > 1e-9) {
            throw std::runtime_error("K is inconsistent with fx/fy/cx/cy");
        }
    }
    if(unescapeQuoted(raw("distortion_coefficient_order")) != "k1,k2,k3,k4,k5,k6,p1,p2") {
        throw std::runtime_error("Unsupported distortion coefficient order");
    }
    const std::vector<double> distortion = parseNumberArray(raw("D"), 8);
    value.distortion = {distortion[0], distortion[1], distortion[2], distortion[3],
                        distortion[4], distortion[5], distortion[6], distortion[7]};
    value.distortionFamily = unescapeQuoted(raw("distortion_family"));
    value.distortionVariant = unescapeQuoted(raw("distortion_variant"));
    value.distortionHandling = unescapeQuoted(raw("distortion_handling"));
    value.opticalAxes = unescapeQuoted(raw("optical_axes"));
    if(unescapeQuoted(raw("camera_frame_definition")) != value.opticalAxes) {
        throw std::runtime_error("camera_frame_definition is inconsistent with optical_axes");
    }
    value.cameraFrameName = unescapeQuoted(raw("camera_frame_name"));
    value.baseFrameName = unescapeQuoted(raw("base_frame_name"));
    value.robotModel = unescapeQuoted(raw("robot_model"));
    value.robotIp = unescapeQuoted(raw("robot_ip"));
    value.toolNumber = parseInt(raw("tool_number"));
    value.baseNumber = parseInt(raw("base_number"));
    const std::vector<double> tool2Position = parseNumberArray(raw("tool2_position_base0_mm"), 3);
    const std::vector<double> tool2Orientation = parseNumberArray(raw("tool2_orientation_raw_abc"), 3);
    value.robotPose = {tool2Position[0], tool2Position[1], tool2Position[2],
                       tool2Orientation[0], tool2Orientation[1], tool2Orientation[2]};
    value.xyzSpreadToleranceMm = parseDouble(raw("xyz_spread_tolerance_mm"));
    value.abcSpreadToleranceDeg = parseDouble(raw("abc_spread_tolerance_deg"));
    value.robotPoseSampleCount = parseInt(raw("robot_pose_sample_count"));
    value.robotPoseSampleWindowMs = parseInt(raw("robot_pose_sample_window_ms"));
    value.rotationConventionSource = unescapeQuoted(raw("rotation_convention_source"));
    value.rotationConvention = unescapeQuoted(raw("tool2_rotation_convention"));
    value.tool2AngleUnit = unescapeQuoted(raw("tool2_angle_unit"));
    const std::vector<double> baseFromTool = parseNumberArray(raw("R_Base0_from_Tool2"), 9);
    std::size_t offset = 0;
    for(auto& row : value.rBase0FromTool2) {
        for(double& element : row) {
            element = baseFromTool[offset++];
        }
    }
    const std::vector<double> toolFromRgb = parseNumberArray(raw("R_Tool2_from_RGB"), 9);
    offset = 0;
    for(auto& row : value.rTool2FromRgb) {
        for(double& element : row) {
            element = toolFromRgb[offset++];
        }
    }
    const std::vector<double> tool2Translation = parseNumberArray(raw("t_Tool2_to_RGB_mm"), 3);
    value.tTool2ToRgb = {tool2Translation[0], tool2Translation[1], tool2Translation[2]};
    const std::vector<double> rotation = parseNumberArray(raw("R_Base0_from_RGB"), 9);
    offset = 0;
    for(auto& row : value.rBase0FromRgb) {
        for(double& element : row) {
            element = rotation[offset++];
        }
    }
    const std::vector<double> translation = parseNumberArray(raw("C_Base0_mm"), 3);
    value.tBase0FromRgb = {translation[0], translation[1], translation[2]};
    value.zTableMm = parseDouble(raw("z_table_mm"));
    value.ballRadiusMm = parseDouble(raw("ball_radius_mm"));
    value.tablePlaneModel = unescapeQuoted(raw("table_plane_model"));
    value.translationUnit = unescapeQuoted(raw("translation_unit"));
    validateCalibration(value);
    return value;
}

void requireOutput(std::ofstream& output, const std::filesystem::path& path) {
    output.flush();
    if(!output) {
        throw std::runtime_error("Failed to write calibration file: " + path.string());
    }
}

bool near(const double left, const double right, const double tolerance) {
    return std::abs(left - right) <= tolerance;
}

}  // namespace

void writeCalibrationJson(const CalibrationData& value, const std::filesystem::path& path) {
    validateCalibration(value);
    std::ofstream output(path, std::ios::binary);
    if(!output) {
        throw std::runtime_error("Cannot create calibration JSON: " + path.string());
    }
    output << std::setprecision(17)
           << "{\n"
           << "  \"schema_version\": \"" << escapeJson(value.schemaVersion) << "\",\n"
           << "  \"timestamp\": \"" << escapeJson(value.createdUtc) << "\",\n"
           << "  \"experimental\": " << (value.experimental ? "true" : "false") << ",\n"
           << "  \"camera\": {\n"
           << "    \"sdk_version\": \"" << escapeJson(value.sdkVersion) << "\",\n"
           << "    \"camera_model\": \"" << escapeJson(value.cameraModel) << "\",\n"
           << "    \"device_name\": \"" << escapeJson(value.deviceName) << "\",\n"
           << "    \"camera_serial_number\": \"" << escapeJson(value.serialNumber) << "\",\n"
           << "    \"firmware_version\": \"" << escapeJson(value.firmwareVersion) << "\",\n"
           << "    \"rgb_width\": " << value.profile.width << ",\n"
           << "    \"rgb_height\": " << value.profile.height << ",\n"
           << "    \"profile_fps\": " << value.profile.fps << ",\n"
           << "    \"profile_format\": \"" << escapeJson(value.profile.format) << "\",\n"
           << "    \"rgb_profile\": \"" << value.profile.width << 'x' << value.profile.height << '@'
           << value.profile.fps << ' ' << escapeJson(value.profile.format) << "\",\n"
           << "    \"fx\": " << value.intrinsic.fx << ",\n"
           << "    \"fy\": " << value.intrinsic.fy << ",\n"
           << "    \"cx\": " << value.intrinsic.cx << ",\n"
           << "    \"cy\": " << value.intrinsic.cy << ",\n"
           << "    \"K\": [[" << value.intrinsic.fx << ", 0, " << value.intrinsic.cx << "], [0, "
           << value.intrinsic.fy << ", " << value.intrinsic.cy << "], [0, 0, 1]],\n"
           << "    \"distortion_coefficient_order\": \"k1,k2,k3,k4,k5,k6,p1,p2\",\n"
           << "    \"D\": [" << value.distortion.k1 << ", "
           << value.distortion.k2 << ", " << value.distortion.k3 << ", " << value.distortion.k4 << ", "
           << value.distortion.k5 << ", " << value.distortion.k6 << ", " << value.distortion.p1 << ", "
           << value.distortion.p2 << "],\n"
           << "    \"distortion_family\": \"" << escapeJson(value.distortionFamily) << "\",\n"
           << "    \"distortion_variant\": \"" << escapeJson(value.distortionVariant) << "\",\n"
           << "    \"distortion_handling\": \"" << escapeJson(value.distortionHandling) << "\",\n"
           << "    \"camera_frame_name\": \"" << escapeJson(value.cameraFrameName) << "\",\n"
           << "    \"camera_frame_definition\": \"" << escapeJson(value.opticalAxes) << "\",\n"
           << "    \"optical_axes\": \"" << escapeJson(value.opticalAxes) << "\"\n"
           << "  },\n"
           << "  \"robot\": {\n"
           << "    \"robot_model\": \"" << escapeJson(value.robotModel) << "\",\n"
           << "    \"robot_ip\": \"" << escapeJson(value.robotIp) << "\",\n"
           << "    \"tool_number\": " << value.toolNumber << ",\n"
           << "    \"base_number\": " << value.baseNumber << ",\n"
           << "    \"base_frame_name\": \"" << escapeJson(value.baseFrameName) << "\",\n"
           << "    \"tool2_position_base0_mm\": [" << value.robotPose.x << ", " << value.robotPose.y << ", "
           << value.robotPose.z << "],\n"
           << "    \"tool2_orientation_raw_abc\": [" << value.robotPose.a << ", " << value.robotPose.b << ", "
           << value.robotPose.c << "],\n"
           << "    \"xyz_spread_tolerance_mm\": " << value.xyzSpreadToleranceMm << ",\n"
           << "    \"abc_spread_tolerance_deg\": " << value.abcSpreadToleranceDeg << ",\n"
           << "    \"robot_pose_sample_count\": " << value.robotPoseSampleCount << ",\n"
           << "    \"robot_pose_sample_window_ms\": " << value.robotPoseSampleWindowMs << "\n"
           << "  },\n"
           << "  \"extrinsic\": {\n"
           << "    \"rotation_convention_source\": \"" << escapeJson(value.rotationConventionSource) << "\",\n"
           << "    \"tool2_rotation_convention\": \"" << escapeJson(value.rotationConvention) << "\",\n"
           << "    \"tool2_angle_unit\": \"" << escapeJson(value.tool2AngleUnit) << "\",\n"
           << "    \"R_Base0_from_Tool2\": [[" << value.rBase0FromTool2[0][0] << ", " << value.rBase0FromTool2[0][1]
           << ", " << value.rBase0FromTool2[0][2] << "], [" << value.rBase0FromTool2[1][0] << ", "
           << value.rBase0FromTool2[1][1] << ", " << value.rBase0FromTool2[1][2] << "], ["
           << value.rBase0FromTool2[2][0] << ", " << value.rBase0FromTool2[2][1] << ", "
           << value.rBase0FromTool2[2][2] << "]],\n"
           << "    \"R_Tool2_from_RGB\": [[" << value.rTool2FromRgb[0][0] << ", " << value.rTool2FromRgb[0][1]
           << ", " << value.rTool2FromRgb[0][2] << "], [" << value.rTool2FromRgb[1][0] << ", "
           << value.rTool2FromRgb[1][1] << ", " << value.rTool2FromRgb[1][2] << "], ["
           << value.rTool2FromRgb[2][0] << ", " << value.rTool2FromRgb[2][1] << ", "
           << value.rTool2FromRgb[2][2] << "]],\n"
           << "    \"t_Tool2_to_RGB_mm\": [" << value.tTool2ToRgb.x << ", " << value.tTool2ToRgb.y << ", "
           << value.tTool2ToRgb.z << "],\n"
           << "    \"R_Base0_from_RGB\": [[" << value.rBase0FromRgb[0][0] << ", " << value.rBase0FromRgb[0][1]
           << ", " << value.rBase0FromRgb[0][2] << "], [" << value.rBase0FromRgb[1][0] << ", "
           << value.rBase0FromRgb[1][1] << ", " << value.rBase0FromRgb[1][2] << "], ["
           << value.rBase0FromRgb[2][0] << ", " << value.rBase0FromRgb[2][1] << ", "
           << value.rBase0FromRgb[2][2] << "]],\n"
           << "    \"C_Base0_mm\": [" << value.tBase0FromRgb.x << ", " << value.tBase0FromRgb.y
           << ", " << value.tBase0FromRgb.z << "]\n"
           << "  },\n"
           << "  \"table\": {\n"
           << "    \"z_table_mm\": " << value.zTableMm << ",\n"
           << "    \"ball_radius_mm\": " << value.ballRadiusMm << ",\n"
           << "    \"table_plane_model\": \"" << escapeJson(value.tablePlaneModel) << "\",\n"
           << "    \"translation_unit\": \"" << escapeJson(value.translationUnit) << "\"\n"
           << "  }\n"
           << "}\n";
    requireOutput(output, path);
}

void writeCalibrationYaml(const CalibrationData& value, const std::filesystem::path& path) {
    validateCalibration(value);
    std::ofstream output(path, std::ios::binary);
    if(!output) {
        throw std::runtime_error("Cannot create calibration YAML: " + path.string());
    }
    output << std::setprecision(17)
           << "schema_version: " << quoteYaml(value.schemaVersion) << "\n"
           << "timestamp: " << quoteYaml(value.createdUtc) << "\n"
           << "experimental: " << (value.experimental ? "true" : "false") << "\n"
           << "camera:\n"
           << "  sdk_version: " << quoteYaml(value.sdkVersion) << "\n"
           << "  camera_model: " << quoteYaml(value.cameraModel) << "\n"
           << "  device_name: " << quoteYaml(value.deviceName) << "\n"
           << "  camera_serial_number: " << quoteYaml(value.serialNumber) << "\n"
           << "  firmware_version: " << quoteYaml(value.firmwareVersion) << "\n"
           << "  rgb_width: " << value.profile.width << "\n"
           << "  rgb_height: " << value.profile.height << "\n"
           << "  profile_fps: " << value.profile.fps << "\n"
           << "  profile_format: " << quoteYaml(value.profile.format) << "\n"
           << "  rgb_profile: " << quoteYaml(std::to_string(value.profile.width) + "x"
                                               + std::to_string(value.profile.height) + "@"
                                               + std::to_string(value.profile.fps) + " " + value.profile.format) << "\n"
           << "  fx: " << value.intrinsic.fx << "\n"
           << "  fy: " << value.intrinsic.fy << "\n"
           << "  cx: " << value.intrinsic.cx << "\n"
           << "  cy: " << value.intrinsic.cy << "\n"
           << "  K: [[" << value.intrinsic.fx << ", 0, " << value.intrinsic.cx << "], [0, "
           << value.intrinsic.fy << ", " << value.intrinsic.cy << "], [0, 0, 1]]\n"
           << "  distortion_coefficient_order: " << quoteYaml("k1,k2,k3,k4,k5,k6,p1,p2") << "\n"
           << "  D: [" << value.distortion.k1 << ", "
           << value.distortion.k2 << ", " << value.distortion.k3 << ", " << value.distortion.k4 << ", "
           << value.distortion.k5 << ", " << value.distortion.k6 << ", " << value.distortion.p1 << ", "
           << value.distortion.p2 << "]\n"
           << "  distortion_family: " << quoteYaml(value.distortionFamily) << "\n"
           << "  distortion_variant: " << quoteYaml(value.distortionVariant) << "\n"
           << "  distortion_handling: " << quoteYaml(value.distortionHandling) << "\n"
           << "  camera_frame_name: " << quoteYaml(value.cameraFrameName) << "\n"
           << "  camera_frame_definition: " << quoteYaml(value.opticalAxes) << "\n"
           << "  optical_axes: " << quoteYaml(value.opticalAxes) << "\n"
           << "robot:\n"
           << "  robot_model: " << quoteYaml(value.robotModel) << "\n"
           << "  robot_ip: " << quoteYaml(value.robotIp) << "\n"
           << "  tool_number: " << value.toolNumber << "\n"
           << "  base_number: " << value.baseNumber << "\n"
           << "  base_frame_name: " << quoteYaml(value.baseFrameName) << "\n"
           << "  tool2_position_base0_mm: [" << value.robotPose.x << ", " << value.robotPose.y << ", "
           << value.robotPose.z << "]\n"
           << "  tool2_orientation_raw_abc: [" << value.robotPose.a << ", " << value.robotPose.b << ", "
           << value.robotPose.c << "]\n"
           << "  xyz_spread_tolerance_mm: " << value.xyzSpreadToleranceMm << "\n"
           << "  abc_spread_tolerance_deg: " << value.abcSpreadToleranceDeg << "\n"
           << "  robot_pose_sample_count: " << value.robotPoseSampleCount << "\n"
           << "  robot_pose_sample_window_ms: " << value.robotPoseSampleWindowMs << "\n"
           << "extrinsic:\n"
           << "  rotation_convention_source: " << quoteYaml(value.rotationConventionSource) << "\n"
           << "  tool2_rotation_convention: " << quoteYaml(value.rotationConvention) << "\n"
           << "  tool2_angle_unit: " << quoteYaml(value.tool2AngleUnit) << "\n"
           << "  R_Base0_from_Tool2: [[" << value.rBase0FromTool2[0][0] << ", " << value.rBase0FromTool2[0][1]
           << ", " << value.rBase0FromTool2[0][2] << "], [" << value.rBase0FromTool2[1][0] << ", "
           << value.rBase0FromTool2[1][1] << ", " << value.rBase0FromTool2[1][2] << "], ["
           << value.rBase0FromTool2[2][0] << ", " << value.rBase0FromTool2[2][1] << ", "
           << value.rBase0FromTool2[2][2] << "]]\n"
           << "  R_Tool2_from_RGB: [[" << value.rTool2FromRgb[0][0] << ", " << value.rTool2FromRgb[0][1]
           << ", " << value.rTool2FromRgb[0][2] << "], [" << value.rTool2FromRgb[1][0] << ", "
           << value.rTool2FromRgb[1][1] << ", " << value.rTool2FromRgb[1][2] << "], ["
           << value.rTool2FromRgb[2][0] << ", " << value.rTool2FromRgb[2][1] << ", "
           << value.rTool2FromRgb[2][2] << "]]\n"
           << "  t_Tool2_to_RGB_mm: [" << value.tTool2ToRgb.x << ", " << value.tTool2ToRgb.y << ", "
           << value.tTool2ToRgb.z << "]\n"
           << "  R_Base0_from_RGB: [[" << value.rBase0FromRgb[0][0] << ", " << value.rBase0FromRgb[0][1]
           << ", " << value.rBase0FromRgb[0][2] << "], [" << value.rBase0FromRgb[1][0] << ", "
           << value.rBase0FromRgb[1][1] << ", " << value.rBase0FromRgb[1][2] << "], ["
           << value.rBase0FromRgb[2][0] << ", " << value.rBase0FromRgb[2][1] << ", "
           << value.rBase0FromRgb[2][2] << "]]\n"
           << "  C_Base0_mm: [" << value.tBase0FromRgb.x << ", " << value.tBase0FromRgb.y << ", "
           << value.tBase0FromRgb.z << "]\n"
           << "table:\n"
           << "  z_table_mm: " << value.zTableMm << "\n"
           << "  ball_radius_mm: " << value.ballRadiusMm << "\n"
           << "  table_plane_model: " << quoteYaml(value.tablePlaneModel) << "\n"
           << "  translation_unit: " << quoteYaml(value.translationUnit) << "\n";
    requireOutput(output, path);
}

CalibrationData readCalibration(const std::filesystem::path& path) {
    const std::string text = readAll(path);
    std::string extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](const unsigned char character) { return static_cast<char>(std::tolower(character)); });
    if(extension == ".json") {
        return parseCalibration([&text](const std::string& key) { return jsonRawValue(text, key); });
    }
    if(extension == ".yaml" || extension == ".yml") {
        const auto values = yamlRawValues(text);
        return parseCalibration([&values](const std::string& key) { return yamlRawValue(values, key); });
    }
    throw std::runtime_error("Calibration file must use .json, .yaml, or .yml extension");
}

bool equivalentCalibration(const CalibrationData& left, const CalibrationData& right, const double tolerance) {
    if(left.schemaVersion != right.schemaVersion || left.createdUtc != right.createdUtc
       || left.experimental != right.experimental || left.sdkVersion != right.sdkVersion
       || left.cameraModel != right.cameraModel
       || left.deviceName != right.deviceName || left.serialNumber != right.serialNumber
       || left.firmwareVersion != right.firmwareVersion || left.profile.width != right.profile.width
       || left.profile.height != right.profile.height || left.profile.fps != right.profile.fps
       || left.profile.format != right.profile.format || left.distortionFamily != right.distortionFamily
       || left.distortionVariant != right.distortionVariant || left.distortionHandling != right.distortionHandling
       || left.opticalAxes != right.opticalAxes || left.robotModel != right.robotModel
       || left.robotIp != right.robotIp || left.toolNumber != right.toolNumber
       || left.baseNumber != right.baseNumber || left.robotPoseSampleCount != right.robotPoseSampleCount
       || left.robotPoseSampleWindowMs != right.robotPoseSampleWindowMs
       || left.rotationConventionSource != right.rotationConventionSource
       || left.rotationConvention != right.rotationConvention || left.tool2AngleUnit != right.tool2AngleUnit
       || left.cameraFrameName != right.cameraFrameName || left.baseFrameName != right.baseFrameName
       || left.tablePlaneModel != right.tablePlaneModel || left.translationUnit != right.translationUnit) {
        return false;
    }
    std::vector<double> leftNumbers{
        left.intrinsic.fx, left.intrinsic.fy, left.intrinsic.cx, left.intrinsic.cy,
        left.distortion.k1, left.distortion.k2, left.distortion.k3, left.distortion.k4,
        left.distortion.k5, left.distortion.k6, left.distortion.p1, left.distortion.p2,
        left.robotPose.x, left.robotPose.y, left.robotPose.z, left.robotPose.a, left.robotPose.b, left.robotPose.c,
        left.xyzSpreadToleranceMm, left.abcSpreadToleranceDeg,
        left.tTool2ToRgb.x, left.tTool2ToRgb.y, left.tTool2ToRgb.z,
        left.tBase0FromRgb.x, left.tBase0FromRgb.y, left.tBase0FromRgb.z,
        left.zTableMm, left.ballRadiusMm,
    };
    std::vector<double> rightNumbers{
        right.intrinsic.fx, right.intrinsic.fy, right.intrinsic.cx, right.intrinsic.cy,
        right.distortion.k1, right.distortion.k2, right.distortion.k3, right.distortion.k4,
        right.distortion.k5, right.distortion.k6, right.distortion.p1, right.distortion.p2,
        right.robotPose.x, right.robotPose.y, right.robotPose.z, right.robotPose.a, right.robotPose.b, right.robotPose.c,
        right.xyzSpreadToleranceMm, right.abcSpreadToleranceDeg,
        right.tTool2ToRgb.x, right.tTool2ToRgb.y, right.tTool2ToRgb.z,
        right.tBase0FromRgb.x, right.tBase0FromRgb.y, right.tBase0FromRgb.z,
        right.zTableMm, right.ballRadiusMm,
    };
    for(std::size_t index = 0; index < leftNumbers.size(); ++index) {
        if(!near(leftNumbers[index], rightNumbers[index], tolerance)) {
            return false;
        }
    }
    for(std::size_t row = 0; row < 3; ++row) {
        for(std::size_t column = 0; column < 3; ++column) {
            if(!near(left.rBase0FromTool2[row][column], right.rBase0FromTool2[row][column], tolerance)
               || !near(left.rTool2FromRgb[row][column], right.rTool2FromRgb[row][column], tolerance)
               || !near(left.rBase0FromRgb[row][column], right.rBase0FromRgb[row][column], tolerance)) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace rgb_base0
