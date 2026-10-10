/***
  This file is part of PulseAudio.
  Copyright (c) 2026 Herman van Hazendonk <github.com@herrie.org>

  PulseAudio is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published
  by the Free Software Foundation; either version 2.1 of the License,
  or (at your option) any later version.

  PulseAudio is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with PulseAudio; if not, see <http://www.gnu.org/licenses/>.
***/

#ifndef PALM_POLICY_TEST_CHECK_H
#define PALM_POLICY_TEST_CHECK_H

/*
 * A test runner small enough to cross-compile with the module and run on a
 * phone, in C because the module is. gtest is not in the LuneOS sysroot, and a
 * few dozen assertions do not justify adding it to the recipe for every
 * machine. Same interface as the C++ runner in notificationmgr's tests.
 *
 * Register with TEST(suite, name) { ... } and assert with CHECK / CHECK_EQ.
 * Every failure is reported; a case does not stop at its first one.
 */

#include <stdio.h>

struct test_case {
    const char *suite;
    const char *name;
    void (*fn)(void);
};

void test_register(const char *suite, const char *name, void (*fn)(void));
void test_failure(const char *file, int line, const char *what);
int test_run_all(const char *suite_filter);

#define TEST(suite, name)                                                      \
    static void suite##_##name##_body(void);                                   \
    __attribute__((constructor)) static void suite##_##name##_reg(void)        \
    {                                                                          \
        test_register(#suite, #name, suite##_##name##_body);                   \
    }                                                                          \
    static void suite##_##name##_body(void)

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond))                                                           \
            test_failure(__FILE__, __LINE__, "CHECK(" #cond ") is false");     \
    } while (0)

/* Integers only: both sides are compared and printed as long long. */
#define CHECK_EQ(actual, expected)                                             \
    do {                                                                       \
        long long a_ = (long long)(actual), e_ = (long long)(expected);        \
        if (a_ != e_) {                                                        \
            char what_[256];                                                   \
            snprintf(what_, sizeof(what_), "%s == %s : got %lld, wanted %lld", \
                     #actual, #expected, a_, e_);                              \
            test_failure(__FILE__, __LINE__, what_);                           \
        }                                                                      \
    } while (0)

#endif
