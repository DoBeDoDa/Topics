#include "MathUtils.h"

#include <algorithm>
#include <cmath>

namespace BilliardMath {
bool isFinite(Point point) noexcept
{
    return std::isfinite(point.x) && std::isfinite(point.y);
}

bool isFinite(Vector2D vector) noexcept
{
    return std::isfinite(vector.x) && std::isfinite(vector.y);
}

std::optional<Vector2D> getVector(Point start, Point end) noexcept
{
    if (!isFinite(start) || !isFinite(end)) {
        return std::nullopt;
    }

    const Vector2D result{end.x - start.x, end.y - start.y};
    if (!isFinite(result)) {
        return std::nullopt;
    }

    return result;
}

std::optional<double> getLength(Vector2D vector) noexcept
{
    if (!isFinite(vector)) {
        return std::nullopt;
    }

    const double length = std::hypot(vector.x, vector.y);
    if (!std::isfinite(length)) {
        return std::nullopt;
    }

    return length;
}

std::optional<double> getDistance(Point first, Point second) noexcept
{
    const auto vector = getVector(first, second);
    if (!vector) {
        return std::nullopt;
    }

    return getLength(*vector);
}

std::optional<Vector2D> normalize(Vector2D vector) noexcept
{
    const auto length = getLength(vector);
    if (!length || *length == 0.0) {
        return std::nullopt;
    }

    const Vector2D normalized{vector.x / *length, vector.y / *length};
    if (!isFinite(normalized)) {
        return std::nullopt;
    }

    return normalized;
}

std::optional<double> getVectorAngleDeg(Vector2D vector) noexcept
{
    const auto length = getLength(vector);
    if (!length || *length == 0.0) {
        return std::nullopt;
    }

    double angleDeg = std::atan2(vector.y, vector.x) * 180.0 / PI;
    if (!std::isfinite(angleDeg)) {
        return std::nullopt;
    }

    if (angleDeg == -180.0) {
        angleDeg = 180.0;
    }

    return angleDeg;
}

std::optional<double> getAngleBetweenVectorsDeg(
    Vector2D first,
    Vector2D second) noexcept
{
    const auto normalizedFirst = normalize(first);
    const auto normalizedSecond = normalize(second);
    if (!normalizedFirst || !normalizedSecond) {
        return std::nullopt;
    }

    const double dot =
        normalizedFirst->x * normalizedSecond->x +
        normalizedFirst->y * normalizedSecond->y;
    if (!std::isfinite(dot)) {
        return std::nullopt;
    }

    const double clampedDot = std::clamp(dot, -1.0, 1.0);
    const double angleDeg = std::acos(clampedDot) * 180.0 / PI;
    if (!std::isfinite(angleDeg)) {
        return std::nullopt;
    }

    return angleDeg;
}
}
