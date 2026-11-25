#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

static unsigned failures;
#define CHECK(x) do { if (!(x)) { ++failures; printf("FAIL line %d: %s\n", __LINE__, #x); } } while (0)
INT tm_putchar(INT c) { return c; }
INT tm_putstring(const UB *str) { return (INT)strlen((const char *)str); }

int main(void)
{
    struct { UB before; UB text[8]; UB after; } small = {0x53U, {0}, 0x97U};
    UB full[160];
    INT length;
    length = tm_snprintf(small.text, 8, (const UB *)"%s=%d", "wheel", 1234);
    CHECK(length == 10 && strcmp((const char *)small.text, "wheel=1") == 0);
    CHECK(small.before == 0x53U && small.after == 0x97U);
    CHECK(tm_snprintf(small.text, 1, (const UB *)"abc") == 3 && small.text[0] == 0U);
    small.text[0] = 0x12U;
    CHECK(tm_snprintf(small.text, 0, (const UB *)"abc") == 3 && small.text[0] == 0x12U);
    CHECK(tm_snprintf(NULL, 0, (const UB *)"%d", INT_MIN) == 11);
    CHECK(tm_snprintf(NULL, 1, (const UB *)"abc") < 0);
    CHECK(tm_snprintf(small.text, -1, (const UB *)"abc") < 0);
    CHECK(tm_snprintf(small.text, 8, NULL) < 0);
    CHECK(tm_snprintf(small.text, 8, (const UB *)"1234567") == 7);
    CHECK(strcmp((const char *)small.text, "1234567") == 0);
    CHECK(tm_snprintf(full, 160, (const UB *)"%d %u %08x %s", INT_MIN, UINT_MAX, 0xabU, "ok") == 34);
    CHECK(strcmp((const char *)full, "-2147483648 4294967295 000000ab ok") == 0);
    /* Report that previously overran a 96-byte array must truncate safely. */
    memset(full, 0x55, sizeof(full));
    length = tm_snprintf(full, 96, (const UB *)
        "obstacle width_mm=%d dist_mm=%d offset_mm=%d complete=%d position=%s avoid=%s shift_l_mm=%d shift_r_mm=%d",
        160, 153, -15, 1, "centre", "right", 180, 210);
    CHECK(length >= 96 && full[95] == 0U && full[96] == 0x55U);
    printf("bounded formatter: %u failures\n", failures);
    return failures == 0U ? 0 : 1;
}
