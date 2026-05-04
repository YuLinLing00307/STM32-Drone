/**
 * @file unity.c
 * @brief Minimal Unity test framework implementation
 */

#include "unity.h"
#include <stdlib.h>
#include <string.h>

int unity_test_count = 0;
int unity_test_failures = 0;
jmp_buf unity_abort_frame;

void unity_init(void)
{
    unity_test_count = 0;
    unity_test_failures = 0;
}

void unity_finish(void)
{
    printf("\n==========================\n");
    printf("Tests: %d, Failures: %d\n", unity_test_count, unity_test_failures);
    if (unity_test_failures == 0) {
        printf("ALL TESTS PASSED\n");
    } else {
        printf("SOME TESTS FAILED\n");
    }
    printf("==========================\n");
}

void unity_abort(void)
{
    longjmp(unity_abort_frame, 1);
}

void unity_test_assert(int condition, const char *condition_str, const char *file, int line)
{
    unity_test_count++;
    if (!condition) {
        unity_test_failures++;
        printf("FAIL: %s (%s:%d)\n", condition_str, file, line);
    } else {
        printf("PASS: %s\n", condition_str);
    }
}

void unity_test_assert_equal(int expected, int actual, const char *file, int line)
{
    unity_test_count++;
    if (expected != actual) {
        unity_test_failures++;
        printf("FAIL: Expected %d, Got %d (%s:%d)\n", expected, actual, file, line);
    } else {
        printf("PASS: Expected %d, Got %d\n", expected, actual);
    }
}

void unity_test_assert_not_equal(int expected, int actual, const char *file, int line)
{
    unity_test_count++;
    if (expected == actual) {
        unity_test_failures++;
        printf("FAIL: Value %d should not equal %d (%s:%d)\n", expected, actual, file, line);
    } else {
        printf("PASS: Value %d != %d\n", expected, actual);
    }
}

void unity_test_fail(const char *msg, const char *file, int line)
{
    unity_test_count++;
    unity_test_failures++;
    printf("FAIL: %s (%s:%d)\n", msg, file, line);
}

void unity_run_test(void (*test_func)(void), const char *name, int line)
{
    if (setjmp(unity_abort_frame) == 0) {
        test_func();
    } else {
        unity_test_failures++;
        printf("ABORT: %s\n", name);
    }
}

#define TEST_CASE_LINE(cmds) ((void)0)

#define MAIN() \
    int main(int argc, char **argv) { \
        (void)argc; (void)argv; \
        UNITY_BEGIN(); \
        unity_run_tests(); \
        UNITY_END(); \
        return unity_test_failures; \
    }
