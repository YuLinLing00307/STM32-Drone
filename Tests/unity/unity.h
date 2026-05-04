/**
 * @file unity.h
 * @brief Minimal Unity test framework header
 */

#ifndef UNITY_H_
#define UNITY_H_

#include <stdio.h>
#include <setjmp.h>

/* Unity configuration */
#define UNITY_INCLUDE_CONFIG_H 0

/* Unity internal */
extern int unity_test_count;
extern int unity_test_failures;
extern jmp_buf unity_abort_frame;

#define UNITY_BEGIN() unity_init()
#define UNITY_END()   unity_finish()

void unity_init(void);
void unity_finish(void);
void unity_abort(void);

/* Assert macros */
#define TEST_ASSERT(condition) \
    unity_test_assert((condition), #condition, __FILE__, __LINE__)

#define TEST_ASSERT_TRUE(x)   TEST_ASSERT((x) != 0)
#define TEST_ASSERT_FALSE(x)  TEST_ASSERT((x) == 0)
#define TEST_ASSERT_EQUAL(expected, actual) \
    unity_test_assert_equal((expected), (actual), __FILE__, __LINE__)
#define TEST_ASSERT_NOT_EQUAL(expected, actual) \
    unity_test_assert_not_equal((expected), (actual), __FILE__, __LINE__)
#define TEST_ASSERT_NULL(ptr)  TEST_ASSERT((ptr) == NULL)
#define TEST_ASSERT_NOT_NULL(ptr) TEST_ASSERT((ptr) != NULL)
#define TEST_FAIL(message) unity_test_fail((message), __FILE__, __LINE__)

/* Ignore return value */
#define TEST_IGNORE_VALUE(func) ((void)(func))

/* Run macro */
#define RUN_TEST(func, line_num) \
    unity_run_test((func), #func, (line_num))

void unity_test_assert(int condition, const char *condition_str, const char *file, int line);
void unity_test_assert_equal(int expected, int actual, const char *file, int line);
void unity_test_assert_not_equal(int expected, int actual, const char *file, int line);
void unity_test_fail(const char *msg, const char *file, int line);
void unity_run_test(void (*test_func)(void), const char *name, int line);

#endif /* UNITY_H_ */
