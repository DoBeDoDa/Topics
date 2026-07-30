#include "TestHarness.h"

int main()
{
    TestHarness tests;

    tests.expectTrue(
        true,
        "algorithm regression test skeleton is runnable");
    tests.expectEqual(
        tests.failureCount(),
        0,
        "algorithm regression test skeleton starts without failures");

    return tests.exitCode();
}
