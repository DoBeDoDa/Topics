// 宣告不依賴業務設定的純二維數學函式。
#pragma once

#include <optional>

#include "Point.h"

namespace BilliardMath {
inline constexpr double PI = 3.14159265358979323846;

bool isFinite(Point point) noexcept;
bool isFinite(Vector2D vector) noexcept;

std::optional<Vector2D> getVector(Point start, Point end) noexcept;
std::optional<double> getLength(Vector2D vector) noexcept;
std::optional<double> getDistance(Point first, Point second) noexcept;
std::optional<Vector2D> normalize(Vector2D vector) noexcept;
std::optional<double> getVectorAngleDeg(Vector2D vector) noexcept;
std::optional<double> getAngleBetweenVectorsDeg(
    Vector2D first,
    Vector2D second) noexcept;
}
