#include "template_hal.h"
static struct
{
    uint32_t mask;

    void (*handler)(void);
} owners[4];
static unsigned owner_count;
static void
dispatch (UINT number)
{
    unsigned i;

    (void)number;

    for (i = 0; i < owner_count; ++i)
    {
        owners[i].handler();
    }
}

bool
board_gpio_irq_register (uint32_t mask, void (*handler)(void))
{
    unsigned i;
    UINT     state;
    T_DINT   definition = {.intatr = TA_HLNG, .inthdr = (FP)dispatch};

    if ((0U == mask) || (NULL == handler) || (4U == owner_count))
    {
        return false;
    }

    for (i = 0; i < owner_count; ++i)
    {
        if (0U != (owners[i].mask & mask))
        {
            return false;
        }
    }

    if ((0U == owner_count) && (E_OK != tk_def_int(13U, &definition)))
    {
        return false;
    }

    DI(state);
    owners[owner_count].mask    = mask;
    owners[owner_count].handler = handler;
    ++owner_count;
    EI(state);
    EnableInt(13U, 1);
    return true;
}

/*** end of file ***/
