#include "TestHarness.h"

#include "../src/GeometryResults.h"
#include "../src/MathUtils.h"

#include <cmath>
#include <limits>
#include <optional>
#include <type_traits>

namespace {
constexpr double TOLERANCE = 1e-12;

void expectVector(
    TestHarness& tests,
    Point start,
    Point end,
    double expectedX,
    double expectedY,
    const char* message)
{
    const auto vector = BilliardMath::getVector(start, end);
    tests.expectTrue(vector.has_value(), message);
    if (vector) {
        tests.expectNear(vector->x, expectedX, TOLERANCE, message);
        tests.expectNear(vector->y, expectedY, TOLERANCE, message);
    }
}

void expectAngle(
    TestHarness& tests,
    Vector2D vector,
    double expected,
    const char* message)
{
    const auto angle = BilliardMath::getVectorAngleDeg(vector);
    tests.expectTrue(angle.has_value(), message);
    if (angle) {
        tests.expectNear(*angle, expected, TOLERANCE, message);
    }
}

void expectAngleBetween(
    TestHarness& tests,
    Vector2D first,
    Vector2D second,
    double expected,
    const char* message)
{
    const auto angle =
        BilliardMath::getAngleBetweenVectorsDeg(first, second);
    tests.expectTrue(angle.has_value(), message);
    if (angle) {
        tests.expectNear(*angle, expected, TOLERANCE, message);
    }
}
}

int main()
{
    TestHarness tests;

    expectVector(tests, {0.0, 0.0}, {1.0, 0.0}, 1.0, 0.0, "getVector +X");
    expectVector(tests, {0.0, 0.0}, {0.0, 1.0}, 0.0, 1.0, "getVector +Y");
    expectVector(tests, {0.0, 0.0}, {-1.0, 0.0}, -1.0, 0.0, "getVector -X");
    expectVector(tests, {0.0, 0.0}, {0.0, -1.0}, 0.0, -1.0, "getVector -Y");
    expectVector(tests, {1.0, 1.0}, {2.0, 2.0}, 1.0, 1.0, "getVector quadrant I");
    expectVector(tests, {1.0, -1.0}, {0.0, 0.0}, -1.0, 1.0, "getVector quadrant II");
    expectVector(tests, {1.0, 1.0}, {0.0, 0.0}, -1.0, -1.0, "getVector quadrant III");
    expectVector(tests, {-1.0, 1.0}, {0.0, 0.0}, 1.0, -1.0, "getVector quadrant IV");

    const auto zeroLength = BilliardMath::getLength({0.0, 0.0});
    tests.expectTrue(zeroLength.has_value(), "zero vector length has a value");
    if (zeroLength) {
        tests.expectNear(*zeroLength, 0.0, TOLERANCE, "zero vector length is zero");
    }
    tests.expectFalse(
        BilliardMath::normalize({0.0, 0.0}).has_value(),
        "zero vector cannot be normalized");

    const auto normalized = BilliardMath::normalize({3.0, 4.0});
    tests.expectTrue(normalized.has_value(), "finite vector can be normalized");
    if (normalized) {
        tests.expectNear(normalized->x, 0.6, TOLERANCE, "normalized X");
        tests.expectNear(normalized->y, 0.8, TOLERANCE, "normalized Y");
    }

    expectAngle(tests, {1.0, 0.0}, 0.0, "+X angle");
    expectAngle(tests, {0.0, 1.0}, 90.0, "+Y angle");
    expectAngle(tests, {-1.0, 0.0}, 180.0, "-X angle");
    expectAngle(tests, {-1.0, -0.0}, 180.0, "-X negative signed-zero angle");
    expectAngle(tests, {0.0, -1.0}, -90.0, "-Y angle");
    expectAngle(tests, {1.0, 1.0}, 45.0, "quadrant I angle");
    expectAngle(tests, {-1.0, 1.0}, 135.0, "quadrant II angle");
    expectAngle(tests, {-1.0, -1.0}, -135.0, "quadrant III angle");
    expectAngle(tests, {1.0, -1.0}, -45.0, "quadrant IV angle");
    tests.expectFalse(
        BilliardMath::getVectorAngleDeg({0.0, 0.0}).has_value(),
        "zero vector has no direction angle");

    expectAngleBetween(tests, {1.0, 0.0}, {2.0, 0.0}, 0.0, "parallel angle");
    expectAngleBetween(tests, {1.0, 0.0}, {0.0, 1.0}, 90.0, "perpendicular angle");
    expectAngleBetween(tests, {1.0, 0.0}, {-1.0, 0.0}, 180.0, "opposite angle");
    tests.expectFalse(
        BilliardMath::getAngleBetweenVectorsDeg({0.0, 0.0}, {1.0, 0.0}).has_value(),
        "first zero vector has no angle");
    tests.expectFalse(
        BilliardMath::getAngleBetweenVectorsDeg({1.0, 0.0}, {0.0, 0.0}).has_value(),
        "second zero vector has no angle");

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();
    tests.expectFalse(BilliardMath::isFinite(Point{nan, 0.0}), "NaN Point is not finite");
    tests.expectFalse(BilliardMath::isFinite(Vector2D{0.0, infinity}), "infinite vector is not finite");
    tests.expectFalse(BilliardMath::getVector({nan, 0.0}, {0.0, 0.0}).has_value(), "getVector rejects NaN");
    tests.expectFalse(BilliardMath::getLength({infinity, 0.0}).has_value(), "getLength rejects infinity");
    tests.expectFalse(BilliardMath::getDistance({0.0, 0.0}, {nan, 0.0}).has_value(), "getDistance rejects NaN");
    tests.expectFalse(BilliardMath::normalize({0.0, infinity}).has_value(), "normalize rejects infinity");
    tests.expectFalse(BilliardMath::getVectorAngleDeg({nan, 1.0}).has_value(), "direction rejects NaN");
    tests.expectFalse(
        BilliardMath::getAngleBetweenVectorsDeg({1.0, 0.0}, {infinity, 0.0}).has_value(),
        "angle rejects infinity");

    const auto largeLength = BilliardMath::getLength({1e150, 1e150});
    tests.expectTrue(largeLength.has_value(), "hypot handles 1e150 vector");
    if (largeLength) {
        tests.expectTrue(std::isfinite(*largeLength), "1e150 hypot result is finite");
        tests.expectNear(*largeLength / 1e150, std::sqrt(2.0), TOLERANCE, "1e150 hypot magnitude");
    }

    tests.expectFalse(
        BilliardMath::getVector(
            {-std::numeric_limits<double>::max(), 0.0},
            {std::numeric_limits<double>::max(), 0.0}).has_value(),
        "Point subtraction overflow returns nullopt");

    const auto clampedAngle = BilliardMath::getAngleBetweenVectorsDeg(
        {1e150, 1e150},
        {1e150, 1e150});
    tests.expectTrue(clampedAngle.has_value(), "cosine clamp boundary returns an angle");
    if (clampedAngle) {
        tests.expectTrue(
            *clampedAngle >= 0.0 && *clampedAngle <= 180.0,
            "cosine clamp keeps angle in range");
    }

    const auto intersection = IntersectionResult::intersects({3.0, 4.0});
    tests.expectTrue(
        intersection.status == IntersectionStatus::Intersects,
        "intersects factory sets status");
    tests.expectTrue(intersection.point.has_value(), "intersects factory sets payload");
    tests.expectTrue(intersection.isValid(), "intersects result satisfies invariant");

    const auto noIntersection = IntersectionResult::noIntersection();
    tests.expectTrue(
        noIntersection.status == IntersectionStatus::NoIntersection,
        "noIntersection factory sets status");
    tests.expectFalse(noIntersection.point.has_value(), "noIntersection has no payload");
    tests.expectTrue(noIntersection.isValid(), "noIntersection satisfies invariant");

    const auto invalidIntersection = IntersectionResult::invalid();
    tests.expectTrue(
        invalidIntersection.status == IntersectionStatus::Invalid,
        "invalid factory sets status");
    tests.expectFalse(invalidIntersection.point.has_value(), "invalid has no payload");
    tests.expectTrue(invalidIntersection.isValid(), "invalid result satisfies invariant");

    tests.expectFalse(
        std::is_constructible_v<
            IntersectionResult,
            IntersectionStatus,
            std::optional<Point>>,
        "contradictory IntersectionResult cannot be constructed");

    return tests.exitCode();
}
