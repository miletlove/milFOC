/**
 ******************************************************************************
 * @file    daemon.c
 * @author  milFOC Team
 * @brief   Daemon module implementation.
 ******************************************************************************
 */

#include "daemon.h"
#include "string.h"

static DaemonInstance *daemon_instances[DAEMON_MAX_CNT] = {NULL};
static uint8_t idx;

DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config)
{
    if (idx >= DAEMON_MAX_CNT) return NULL;

    DaemonInstance *ins = (DaemonInstance *)malloc(sizeof(DaemonInstance));
    memset(ins, 0, sizeof(DaemonInstance));

    ins->owner_id     = config->owner_id;
    ins->reload_count = config->reload_count ? config->reload_count : 100;
    ins->callback     = config->callback;
    ins->temp_count   = config->init_count ? config->init_count : 100;

    daemon_instances[idx++] = ins;
    return ins;
}

void DaemonReload(DaemonInstance *instance)
{
    if (instance) instance->temp_count = instance->reload_count;
}

uint8_t DaemonIsOnline(DaemonInstance *instance)
{
    return (instance && instance->temp_count > 0) ? 1 : 0;
}

void DaemonTask(void)
{
    for (size_t i = 0; i < idx; ++i)
    {
        DaemonInstance *dins = daemon_instances[i];
        if (dins == NULL) continue;

        if (dins->temp_count > 0)
        {
            dins->temp_count--;
        }
        else if (dins->callback)
        {
            dins->callback(dins->owner_id);
        }
    }
}
