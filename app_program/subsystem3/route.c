/* Barcode route (route.h). The MQTT callback is the only writer; the barcode
 * and line tasks read. One aligned 32-bit store with SMP off, so a reader
 * always sees a whole route. */
#include "route.h"

static volatile uint32_t route_word; /* 0 = nothing received yet. */

#define ROUTE_BYTE(letter, action) \
    | ((uint32_t)(unsigned char)(letter) << (8U * ((unsigned)(action) - 1U)))
static const uint32_t route_fallback = 0U ROUTE_TABLE(ROUTE_BYTE);

uint32_t
route_get (void)
{
    uint32_t route = route_word;

    return (0U != route) ? route : route_fallback;
}

route_action_t
route_action (uint32_t route, char letter)
{
    for (unsigned i = 0U; ('\0' != letter) && (i < 4U); ++i, route >>= 8)
    {
        if ((char)(route & 0xFFU) == letter)
        {
            return (route_action_t)(i + 1U);
        }
    }

    return ROUTE_NONE;
}

static char
route_upper (char c)
{
    return ((c >= 'a') && (c <= 'z')) ? (char)(c - ('a' - 'A')) : c;
}

bool
route_set (const char * text, int len)
{
    /* route_action_t order. */
    static const char keys[4] = {'L', 'R', 'S', 'U'};
    uint32_t          route   = 0U;
    int               i       = 0;

    while (i < len)
    {
        char key = route_upper(text[i]);

        if ((' ' == key) || (',' == key))
        {
            ++i;
            continue;
        }

        if (((i + 2) >= len) || ('=' != text[i + 1]))
        {
            return false;
        }

        char     letter = route_upper(text[i + 2]);
        unsigned slot   = 0U;

        while ((slot < 4U) && (keys[slot] != key))
        {
            ++slot;
        }

        bool valid = ((letter >= 'A') && (letter <= 'Z'))
                     || ((letter >= '0') && (letter <= '9'));

        if ((4U == slot) || (!valid) || (0U != ((route >> (8U * slot)) & 0xFFU))
            || (ROUTE_NONE != route_action(route, letter)))
        {
            /* Unknown key, bad letter, key given twice, or letter used twice.
             */
            return false;
        }

        route |= (uint32_t)(unsigned char)letter << (8U * slot);
        i += 3;

        if ((i < len) && (' ' != text[i]) && (',' != text[i]))
        {
            return false;
        }
    }

    for (unsigned slot = 0U; slot < 4U; ++slot)
    {
        if (0U == ((route >> (8U * slot)) & 0xFFU))
        {
            return false;
        } /* All four actions are needed. */
    }

    route_word = route;
    return true;
}

/*** end of file ***/
