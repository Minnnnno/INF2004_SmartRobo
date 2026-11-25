/** @file gpio_dispatch.c
 *
 * @brief One GPIO interrupt shared by handlers that own separate pins.
 */

#include "template_hal.h"

static struct
{
    uint32_t mask;

    void (*handler)(void);
} g_owners[4];

static unsigned g_owner_count;

static void dispatch(UINT number);

/*! @brief Register a handler for a set of GPIO pins. */
bool
board_gpio_irq_register (uint32_t mask, void (*p_handler)(void))
{
    unsigned index;
    UINT     state;
    T_DINT   definition = {.intatr = TA_HLNG, .inthdr = (FP)dispatch};

    if ((0U == mask) || (NULL == p_handler) || (4U == g_owner_count))
    {
        return false;
    }

    for (index = 0; index < g_owner_count; ++index)
    {
        if (0U != (g_owners[index].mask & mask))
        {
            return false;
        }
    }

    if ((0U == g_owner_count) && (E_OK != tk_def_int(13U, &definition)))
    {
        return false;
    }

    DI(state);
    g_owners[g_owner_count].mask    = mask;
    g_owners[g_owner_count].handler = p_handler;
    ++g_owner_count;
    EI(state);
    EnableInt(13U, 1);
    return true;
}

/*! @brief GPIO interrupt: call every registered handler. */
static void
dispatch (UINT number)
{
    unsigned index;

    (void)number;

    for (index = 0; index < g_owner_count; ++index)
    {
        g_owners[index].handler();
    }
}

/*** end of file ***/
