/* Barcode route: the letters the course uses and what the car does for each.
 * On the day, Buddy 1 (MQTT) will supply four letters, one per action; until then they are fixed here.
 * The same table gives the barcode decoder its allowed letters (barcode.c) and the line task its
 * actions (line_following.c), so the two cannot disagree. */
#ifndef ROBO_ROUTE_H
#define ROBO_ROUTE_H

typedef enum { ROUTE_NONE, ROUTE_LEFT, ROUTE_RIGHT, ROUTE_STRAIGHT, ROUTE_UTURN } route_action_t;
typedef enum { ROUTE_AT_JUNCTION, ROUTE_IMMEDIATE } route_trigger_t; /* When the action happens. */

/* X(letter, action, trigger), at most four entries. For now the U-turn happens straight after the
 * read; later every action moves to the junction (change ROUTE_IMMEDIATE to ROUTE_AT_JUNCTION). */
#define ROUTE_TABLE(X) \
    X('A', ROUTE_UTURN, ROUTE_IMMEDIATE) \
    X('Z', ROUTE_LEFT,  ROUTE_AT_JUNCTION)

#define ROUTE_LETTER_OF(letter, action, trigger) letter,
#define ROUTE_CALIBRATION_LETTER 'A' /* The code driven over after pressing GP21 (see line_following.c). */
#endif
