/* Host checks for the MQTT barcode route (app_program/subsystem3/route.c). */
#include <stdio.h>
#include <string.h>
#include "route.h"

static int failures;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL line %d: %s\n", __LINE__, #cond); ++failures; } } while (0)

static int set(const char *text) { return route_set(text, (int)strlen(text)); }

int main(void)
{
    /* Nothing received: ROUTE_TABLE (A = U-turn, Z = left). */
    uint32_t r = route_get();
    CHECK(route_action(r, 'A') == ROUTE_UTURN);
    CHECK(route_action(r, 'Z') == ROUTE_LEFT);
    CHECK(route_action(r, 'B') == ROUTE_NONE);
    CHECK(route_action(r, '\0') == ROUTE_NONE); /* Unused slots are 0, never a match. */

    /* Rejected, and the route stays the fallback. */
    CHECK(!set(""));
    CHECK(!set("L=B R=C S=D"));          /* Missing an action. */
    CHECK(!set("L=B R=B S=D U=A"));      /* Letter used twice. */
    CHECK(!set("L=B L=C S=D U=A"));      /* Key given twice. */
    CHECK(!set("L=B R=C S=D X=A"));      /* Unknown key. */
    CHECK(!set("L=B R=* S=D U=A"));      /* Not a letter or digit. */
    CHECK(!set("L=BC R=D S=E U=A"));     /* Two letters for one action. */
    CHECK(!set("L B R C S D U A"));      /* No '='. */
    CHECK(!set("L=B R=C S=D U="));       /* Cut short. */
    CHECK(route_get() == r);

    /* Accepted in any order, case and separator. */
    CHECK(set("U=a, s=D,R=c L=B"));
    r = route_get();
    CHECK(route_action(r, 'B') == ROUTE_LEFT);
    CHECK(route_action(r, 'C') == ROUTE_RIGHT);
    CHECK(route_action(r, 'D') == ROUTE_STRAIGHT);
    CHECK(route_action(r, 'A') == ROUTE_UTURN);
    CHECK(route_action(r, 'Z') == ROUTE_NONE);

    /* Length is honoured (MQTT payloads are not '\0'-terminated); a bad route keeps the last good one. */
    CHECK(route_set("L=1 R=2 S=3 U=4 junk", 15));
    CHECK(route_action(route_get(), '4') == ROUTE_UTURN);
    CHECK(!set("L=1 R=1 S=3 U=4"));
    CHECK(route_action(route_get(), '2') == ROUTE_RIGHT);

    printf("%s (%d failures)\n", failures ? "route_checks FAILED" : "route_checks passed", failures);
    return failures ? 1 : 0;
}
