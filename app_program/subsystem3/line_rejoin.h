/*
 *	line_rejoin.h
 *	Subsystem 3 -- tell an obstacle detour that the line is under the robot.
 *
 *	line_task publishes line status values[LINE_VALUE_BOTH_BLACK] = 1 while
 *	the centre and right IR sensors both read black (debounced, two samples
 *	in a row).  Subsystem 5 calls line_rejoin_seen() while passing the obstacle
 *	and coming back: true means stop the detour and hand back to line following.
 *	It is a header so that it also exists in profiles without line steering,
 *	where nothing publishes and the answer is always false.
 */

#ifndef LINE_REJOIN_H
#define LINE_REJOIN_H

#include "robot.h"

#define LINE_VALUE_BOTH_BLACK	2U	/* index into the line status values[] */
#define LINE_REJOIN_FRESH_S	0.2	/* readings older than this are stale */

static inline bool line_rejoin_seen(const robot_status_t *p_status, double now)
{
	return p_status->line.available &&
	       (now - p_status->line.timestamp <= LINE_REJOIN_FRESH_S) &&
	       (p_status->line.values[LINE_VALUE_BOTH_BLACK] > 0.5);
}

#endif /* LINE_REJOIN_H */
