#pragma once

#include <cmath>
#include <iostream>
#include <string_view>

class TestHarness {
public:
    void expectTrue(bool condition, std::string_view message)
    {
        if (!condition) {
            reportFailure(message, "true", "false");
        }
    }

    void expectFalse(bool condition, std::string_view message)
    {
        if (condition) {
            reportFailure(message, "false", "true");
        }
    }

    template <typename Actual, typename Expected>
    void expectEqual(
        const Actual& actual,
        const Expected& expected,
        std::string_view message)
    {
        if (!(actual == expected)) {
            std::cerr << "[FAIL] " << message
                      << " (expected: " << expected
                      << ", actual: " << actual << ")\n";
            ++failureCount_;
        }
    }

    void expectNear(
        double actual,
        double expected,
        double tolerance,
        std::string_view message)
    {
        const bool validTolerance =
            std::isfinite(tolerance) && tolerance >= 0.0;
        const bool finiteValues =
            std::isfinite(actual) && std::isfinite(expected);

        if (!validTolerance ||
            !finiteValues ||
            std::fabs(actual - expected) > tolerance) {
            std::cerr << "[FAIL] " << message
                      << " (expected: " << expected
                      << " +/- " << tolerance
                      << ", actual: " << actual << ")\n";
            ++failureCount_;
        }
    }

    [[nodiscard]] int failureCount() const noexcept
    {
        return failureCount_;
    }

    [[nodiscard]] int exitCode() const noexcept
    {
        return failureCount_ == 0 ? 0 : 1;
    }

private:
    void reportFailure(
        std::string_view message,
        std::string_view expected,
        std::string_view actual)
    {
        std::cerr << "[FAIL] " << message
                  << " (expected: " << expected
                  << ", actual: " << actual << ")\n";
        ++failureCount_;
    }

    int failureCount_ = 0;
};
