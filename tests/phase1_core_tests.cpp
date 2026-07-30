#include "TestHarness.h"

int main()
{
    TestHarness tests;

    tests.expectTrue(true, "expectTrue accepts true");
    tests.expectFalse(false, "expectFalse accepts false");
    tests.expectEqual(42, 42, "expectEqual accepts equal values");
    tests.expectNear(
        1.0005,
        1.0,
        0.001,
        "expectNear accepts a value within tolerance");
    tests.expectEqual(
        tests.failureCount(),
        0,
        "successful assertions do not increase the failure count");

    return tests.exitCode();
}
