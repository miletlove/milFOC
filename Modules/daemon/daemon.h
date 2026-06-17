/**
 ******************************************************************************
 * @file    daemon.h
 * @author  milFOC Team
 * @brief   Daemon (watchdog) module for monitoring module health.
 *          Each module registers a daemon instance with a reload count.
 *          The daemon task decrements counters; if a counter reaches zero,
 *          the module is considered offline and its callback is invoked.
 ******************************************************************************
 */

#ifndef DAEMON_H
#define DAEMON_H

#include "general_def.h"

#define DAEMON_MAX_CNT  64   /* Max registered daemon instances */

typedef void (*offline_callback)(void *);

typedef struct daemon_ins
{
    uint16_t reload_count;          /* Reload value (timeout threshold) */
    offline_callback callback;     /* Called when module goes offline */
    uint16_t temp_count;           /* Current countdown value */
    void *owner_id;                /* Owning module pointer */
} DaemonInstance;

typedef struct
{
    uint16_t reload_count;
    uint16_t init_count;
    offline_callback callback;
    void *owner_id;
} Daemon_Init_Config_s;

DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config);
void DaemonReload(DaemonInstance *instance);
uint8_t DaemonIsOnline(DaemonInstance *instance);
void DaemonTask(void);

#endif /* DAEMON_H */
