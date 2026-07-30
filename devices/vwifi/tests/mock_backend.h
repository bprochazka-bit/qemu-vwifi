/*
 * vwifi-virt — mock backend header (for tests)
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef VWIFI_MOCK_BACKEND_H
#define VWIFI_MOCK_BACKEND_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "vwifi_backend.h"

struct mock_backend;

enum mock_event_kind {
    MOCK_EV_IRQ,
    MOCK_EV_MEDIUM_TX,
};

struct mock_event {
    enum mock_event_kind kind;
    unsigned             vec;                /* IRQ */
    uint8_t              medium_buf[4096];   /* MEDIUM_TX */
    size_t               medium_len;
};

struct mock_backend *mock_backend_new(size_t ram_size, size_t event_capacity);
void                  mock_backend_free(struct mock_backend *m);
const struct vwifi_backend_ops *mock_backend_ops_get(void);

uint64_t mock_backend_ram_base(const struct mock_backend *m);
uint8_t *mock_backend_ram(const struct mock_backend *m);

size_t                     mock_backend_event_count(const struct mock_backend *m);
const struct mock_event   *mock_backend_events(const struct mock_backend *m);
void                       mock_backend_clear_events(struct mock_backend *m);
void                       mock_backend_set_quiet(struct mock_backend *m, bool q);

/* Clock / timer control — tests drive time explicitly. */
uint64_t mock_backend_now_us(const struct mock_backend *m);
bool     mock_backend_timer_armed(const struct mock_backend *m);
void     mock_backend_advance_us(struct mock_backend *m, uint64_t us);
bool     mock_backend_timer_due(struct mock_backend *m);
bool     mock_backend_advance_to_timer(struct mock_backend *m);

#endif
