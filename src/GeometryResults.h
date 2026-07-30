#pragma once

#include <optional>
#include <utility>

#include "Point.h"

enum class PathStatus {
    Clear,
    Blocked,
    Invalid
};

enum class IntersectionStatus {
    Intersects,
    NoIntersection,
    Invalid
};

class IntersectionResult {
public:
    static IntersectionResult intersects(Point point)
    {
        return IntersectionResult{
            IntersectionStatus::Intersects,
            std::optional<Point>{point}};
    }

    static IntersectionResult noIntersection()
    {
        return IntersectionResult{
            IntersectionStatus::NoIntersection,
            std::nullopt};
    }

    static IntersectionResult invalid()
    {
        return IntersectionResult{
            IntersectionStatus::Invalid,
            std::nullopt};
    }

    [[nodiscard]] bool isValid() const noexcept
    {
        return (status == IntersectionStatus::Intersects) == point.has_value();
    }

    const IntersectionStatus status;
    const std::optional<Point> point;

private:
    IntersectionResult(
        IntersectionStatus resultStatus,
        std::optional<Point> intersectionPoint)
        : status(resultStatus),
          point(std::move(intersectionPoint))
    {
    }
};
