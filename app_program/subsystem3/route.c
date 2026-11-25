/** @file route.c
 *
 * @brief Barcode route (route.h). The MQTT callback is the only writer; the
 * barcode and line tasks read. One aligned 32-bit store with SMP off, so a
 * reader always sees a whole route.
 */

#include <stdint.h>
#include "route.h"

static volatile uint32_t g_route_word; /* 0 = nothing received yet. */

#define ROUTE_BYTE(letter, action) \
    | ((uint32_t)(uint8_t)(letter) << (8U * ((unsigned)(action) - 1U)))
static const uint32_t g_route_fallback = 0U ROUTE_TABLE(ROUTE_BYTE);

static char route_upper(char letter);

uint32_t
route_get (void)
{
    uint32_t route = g_route_word;

    return (0U != route) ? route : g_route_fallback;
}

/*! @brief Action of a letter in a route. */
route_action_t
route_action (uint32_t route, char letter)
{
    for (unsigned index = 0U; ('\0' != letter) && (index < ROUTE_SLOTS);
         ++index, route >>= 8)
    {
        if ((char)(route & 0xFFU) == letter)
        {
            return (route_action_t)(index + 1U);
        }
    }

    return ROUTE_NONE;
}

bool
route_set (const char * p_text, int len)
{
    /* route_action_t order. */
    static const char keys[4] = {'L', 'R', 'S', 'U'};
    uint32_t          route   = 0U;
    int               index   = 0;

    while (index < len)
    {
        char key = route_upper(p_text[index]);

        if ((' ' == key) || (',' == key))
        {
            ++index;
        }
        else
        {
            if (((index + 2) >= len) || ('=' != p_text[index + 1]))
            {
                return false;
            }

            char     letter = route_upper(p_text[index + 2]);
            unsigned slot   = 0U;

            while ((slot < ROUTE_SLOTS) && (keys[slot] != key))
            {
                ++slot;
            }

            bool b_valid = ((letter >= 'A') && (letter <= 'Z'))
                           || ((letter >= '0') && (letter <= '9'));

            if ((4U == slot) || (!b_valid)
                || (0U != ((route >> (8U * slot)) & 0xFFU))
                || (ROUTE_NONE != route_action(route, letter)))
            {
                /* Unknown key, bad letter, key given twice, or letter used
                 * twice.
                 */
                return false;
            }

            route |= (uint32_t)(uint8_t)letter << (8U * slot);
            index += 3;

            if ((index < len) && (' ' != p_text[index])
                && (',' != p_text[index]))
            {
                return false;
            }
        }
    }

    for (unsigned slot = 0U; slot < ROUTE_SLOTS; ++slot)
    {
        if (0U == ((route >> (8U * slot)) & 0xFFU))
        {
            return false;
        } /* All four actions are needed. */
    }

    g_route_word = route;
    return true;
}

/*! @brief Upper-case letter. */
static char
route_upper (char letter)
{
    return ((letter >= 'a') && (letter <= 'z')) ? (char)(letter - ('a' - 'A'))
                                                : letter;
}

/*** end of file ***/
