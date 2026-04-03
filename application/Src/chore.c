#include "chore.h"
#include "referee.h"
#include "stdint.h"

static uint32_t chore_count = 0;
void chore_task(void)
{
    chore_count = (chore_count + 1) % 200;

    if (chore_count % 20 == 0) { /* 20 hz */
        referee_ui_update();
    }
}