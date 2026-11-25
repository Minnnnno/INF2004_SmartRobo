/* Barcode route: which letter means which action. On the day the letters arrive
 * over MQTT (robot/route, text "L=B R=C S=D U=A", see subsystem1/comms.c);
 * until then ROUTE_TABLE is used. The same route gives the barcode decoder its
 * allowed letters (barcode.c) and the line task its actions (line_following.c),
 * so the two cannot disagree. Turns and the U-turn wait for the junction after
 * the code; straight ignores the junction. */
#ifndef ROBO_ROUTE_H
#define ROBO_ROUTE_H
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ROUTE_NONE,
    ROUTE_LEFT,
    ROUTE_RIGHT,
    ROUTE_STRAIGHT,
    ROUTE_UTURN
} route_action_t;

/* Fallback until MQTT supplies a route: X(letter, action), at most one letter
 * per action. */
#define ROUTE_TABLE(X)  \
    X('A', ROUTE_UTURN) \
    X('Z', ROUTE_LEFT)

/* The code driven over after pressing GP21 (see line_following.c). */
#define ROUTE_CALIBRATION_LETTER 'A'

/* A route is one word: byte (action - 1) holds that action's letter, 0 =
 * unused. */
/* Parse "L=B R=C S=D U=A"; stored only if valid. */
bool route_set(const char * text, int len);
/* The MQTT route, or ROUTE_TABLE if none arrived. */
uint32_t       route_get(void);
route_action_t route_action(uint32_t route, char letter);
#endif

/*** end of file ***/
