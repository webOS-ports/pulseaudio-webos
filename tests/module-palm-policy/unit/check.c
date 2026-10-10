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

#include "check.h"

#include <stdlib.h>
#include <string.h>

#define MAX_TESTS 256

static struct test_case tests[MAX_TESTS];
static int n_tests;
static int failures_in_case;

void test_register(const char *suite, const char *name, void (*fn)(void))
{
    if (n_tests == MAX_TESTS) {
        fprintf(stderr, "too many tests, raise MAX_TESTS\n");
        abort();
    }
    tests[n_tests].suite = suite;
    tests[n_tests].name = name;
    tests[n_tests].fn = fn;
    n_tests++;
}

void test_failure(const char *file, int line, const char *what)
{
    failures_in_case++;
    printf("    %s:%d: %s\n", file, line, what);
}

int test_run_all(const char *suite_filter)
{
    int run = 0, failed = 0;

    for (int i = 0; i < n_tests; i++) {
        if (suite_filter && strcmp(suite_filter, tests[i].suite) != 0)
            continue;
        failures_in_case = 0;
        tests[i].fn();
        run++;
        if (failures_in_case) {
            failed++;
            printf("FAIL %s.%s\n", tests[i].suite, tests[i].name);
        } else
            printf("ok   %s.%s\n", tests[i].suite, tests[i].name);
    }
    printf("%d tests, %d failed\n", run, failed);
    return run == 0 || failed ? 1 : 0;
}

int main(int argc, char **argv)
{
    return test_run_all(argc > 1 ? argv[1] : NULL);
}
