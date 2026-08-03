#pragma once

#include <optional>
#include <cstddef>
#include <utility>

#include "Point.h"

enum class PathStatus {
    Clear,
    Blocked,
    Invalid
};

enum class GeometryStatus {
    Success,
    Clear,
    Blocked,
    OutsideValidRegion,
    NoIntersection,
    Parallel,
    Coincident,
    ConfigurationMissing,
    InvalidConfiguration,
    InvalidInput,
    DegenerateGeometry,
    DirectionRejected
};

struct GeometryDiagnostic {
    GeometryStatus status;
    std::optional<std::size_t> relatedIndex;
};

template <typename Value>
class GeometryValueResult {
public:
    static GeometryValueResult success(Value value)
    {
        return GeometryValueResult(
            GeometryStatus::Success,
            std::optional<Value>{std::move(value)},
            std::nullopt);
    }

    static GeometryValueResult failure(
        GeometryStatus status,
        std::optional<std::size_t> relatedIndex = std::nullopt)
    {
        return GeometryValueResult(
            status,
            std::nullopt,
            GeometryDiagnostic{status, relatedIndex});
    }

    [[nodiscard]] GeometryStatus status() const noexcept { return status_; }
    [[nodiscard]] const std::optional<Value>& value() const noexcept { return value_; }
    [[nodiscard]] const std::optional<GeometryDiagnostic>& diagnostic() const noexcept
    {
        return diagnostic_;
    }
    [[nodiscard]] bool isValid() const noexcept
    {
        const bool succeeded = status_ == GeometryStatus::Success;
        return value_.has_value() == succeeded &&
            diagnostic_.has_value() != succeeded &&
            (!diagnostic_ || diagnostic_->status == status_);
    }

private:
    GeometryValueResult(
        GeometryStatus status,
        std::optional<Value> value,
        std::optional<GeometryDiagnostic> diagnostic)
        : status_(status), value_(std::move(value)), diagnostic_(std::move(diagnostic))
    {
    }

    GeometryStatus status_;
    std::optional<Value> value_;
    std::optional<GeometryDiagnostic> diagnostic_;
};

class GeometryCheckResult {
public:
    static GeometryCheckResult clear()
    {
        return GeometryCheckResult(GeometryStatus::Clear, std::nullopt);
    }

    static GeometryCheckResult rejected(
        GeometryStatus status,
        std::optional<std::size_t> relatedIndex = std::nullopt)
    {
        return GeometryCheckResult(
            status,
            GeometryDiagnostic{status, relatedIndex});
    }

    [[nodiscard]] GeometryStatus status() const noexcept { return status_; }
    [[nodiscard]] const std::optional<GeometryDiagnostic>& diagnostic() const noexcept
    {
        return diagnostic_;
    }
    [[nodiscard]] bool isValid() const noexcept
    {
        return status_ == GeometryStatus::Clear
            ? !diagnostic_.has_value()
            : diagnostic_.has_value() && diagnostic_->status == status_;
    }

private:
    GeometryCheckResult(
        GeometryStatus status,
        std::optional<GeometryDiagnostic> diagnostic)
        : status_(status), diagnostic_(std::move(diagnostic))
    {
    }

    GeometryStatus status_;
    std::optional<GeometryDiagnostic> diagnostic_;
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
