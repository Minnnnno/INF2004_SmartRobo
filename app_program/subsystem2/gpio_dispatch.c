#include "template_hal.h"
static struct { uint32_t mask; void (*handler)(void); } owners[4];
static unsigned owner_count;
static void dispatch(UINT number)
{
    unsigned i;
    (void)number;
    for (i = 0; i < owner_count; ++i) { owners[i].handler(); }
}
bool board_gpio_irq_register(uint32_t mask, void (*handler)(void))
{
    unsigned i;
    UINT state;
    T_DINT definition = {.intatr = TA_HLNG, .inthdr = (FP)dispatch};
    if ((mask == 0U) || (handler == NULL) || owner_count == 4U) { return false; }
    for (i = 0; i < owner_count; ++i) { if ((owners[i].mask & mask) != 0U) { return false; } }
    if ((owner_count == 0U) && (tk_def_int(13U, &definition) != E_OK)) { return false; }
    DI(state);
    owners[owner_count].mask = mask;
    owners[owner_count].handler = handler;
    ++owner_count;
    EI(state);
    EnableInt(13U, 1);
    return true;
}
