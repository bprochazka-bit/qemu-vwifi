/*
 * vwifi-virt — mock backend for tests
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * A simple in-memory backend used by the unit tests. "Guest RAM"
 * is a single large malloc; DMA is memcpy against that region.
 * IRQ raises and medium sends are recorded in ring buffers for
 * test assertions. Logs go to stderr (or /dev/null if quiet).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "vwifi_backend.h"
#include "mock_backend.h"

struct mock_backend {
    /* Fake guest RAM: [ram_base .. ram_base + ram_size). */
    uint8_t *ram;
    uint64_t ram_base;
    size_t   ram_size;

    /* Captured events for assertions. */
    struct mock_event *events;
    size_t             event_capacity;
    size_t             event_count;

    /* Manually-advanced clock (microseconds) — tests control time
     * explicitly so scan dwell progression is deterministic. */
    uint64_t now_us;

    /* Single one-shot timer. deadline_us is valid only when armed. */
    bool     timer_armed;
    uint64_t timer_deadline_us;

    bool  quiet;
};

/* ------------ backend ops ------------ */

static int mock_dma_read(void *be, uint64_t gpa, void *buf, size_t len)
{
    struct mock_backend *m = be;
    if (gpa < m->ram_base) return -1;
    uint64_t off = gpa - m->ram_base;
    if (off + len > m->ram_size) return -1;
    memcpy(buf, m->ram + off, len);
    return 0;
}

static int mock_dma_write(void *be, uint64_t gpa, const void *buf, size_t len)
{
    struct mock_backend *m = be;
    if (gpa < m->ram_base) return -1;
    uint64_t off = gpa - m->ram_base;
    if (off + len > m->ram_size) return -1;
    memcpy(m->ram + off, buf, len);
    return 0;
}

static void mock_push_event(struct mock_backend *m, struct mock_event ev)
{
    if (m->event_count >= m->event_capacity) return;
    m->events[m->event_count++] = ev;
}

static void mock_raise_irq(void *be, unsigned vec)
{
    struct mock_backend *m = be;
    struct mock_event ev = { .kind = MOCK_EV_IRQ, .vec = vec };
    mock_push_event(m, ev);
}

static int mock_medium_send(void *be, const void *buf, size_t len)
{
    struct mock_backend *m = be;
    struct mock_event ev = { .kind = MOCK_EV_MEDIUM_TX };
    if (len > sizeof(ev.medium_buf)) len = sizeof(ev.medium_buf);
    memcpy(ev.medium_buf, buf, len);
    ev.medium_len = len;
    mock_push_event(m, ev);
    return 0;
}

static void mock_log(void *be, const struct vwifi_dev *dev,
                     enum vwifi_log_level level, const char *fmt, va_list ap)
{
    struct mock_backend *m = be;
    (void)dev;
    if (m->quiet && level > VWIFI_LOG_ERR) return;
    fprintf(stderr, "vwifi[mock] ");
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
}

static uint64_t mock_now_us(void *be)
{
    struct mock_backend *m = be;
    return m->now_us;
}

static void mock_timer_mod(void *be, uint32_t ms)
{
    struct mock_backend *m = be;
    m->timer_armed = true;
    m->timer_deadline_us = m->now_us + (uint64_t)ms * 1000;
}

static void mock_timer_del(void *be)
{
    struct mock_backend *m = be;
    m->timer_armed = false;
}

static const struct vwifi_backend_ops mock_backend_ops = {
    .dma_read    = mock_dma_read,
    .dma_write   = mock_dma_write,
    .raise_irq   = mock_raise_irq,
    .medium_send = mock_medium_send,
    .log         = mock_log,
    .now_us      = mock_now_us,
    .timer_mod   = mock_timer_mod,
    .timer_del   = mock_timer_del,
};

/* ------------ public API ------------ */

struct mock_backend *mock_backend_new(size_t ram_size, size_t event_capacity)
{
    struct mock_backend *m = calloc(1, sizeof(*m));
    m->ram = calloc(1, ram_size);
    m->ram_base = 0x40000000;   /* pretend guest RAM starts here */
    m->ram_size = ram_size;
    m->events = calloc(event_capacity, sizeof(struct mock_event));
    m->event_capacity = event_capacity;
    return m;
}

void mock_backend_free(struct mock_backend *m)
{
    if (!m) return;
    free(m->ram); free(m->events); free(m);
}

const struct vwifi_backend_ops *mock_backend_ops_get(void)
{
    return &mock_backend_ops;
}

uint64_t mock_backend_ram_base(const struct mock_backend *m)  { return m->ram_base; }
uint8_t *mock_backend_ram(const struct mock_backend *m)        { return m->ram; }
size_t   mock_backend_event_count(const struct mock_backend *m){ return m->event_count; }
const struct mock_event *mock_backend_events(const struct mock_backend *m)
{
    return m->events;
}
void mock_backend_clear_events(struct mock_backend *m)         { m->event_count = 0; }
void mock_backend_set_quiet(struct mock_backend *m, bool q)    { m->quiet = q; }

/* ------------ clock / timer control for tests ------------ */

uint64_t mock_backend_now_us(const struct mock_backend *m) { return m->now_us; }

bool mock_backend_timer_armed(const struct mock_backend *m)
{
    return m->timer_armed;
}

/* Advance the clock by `us` microseconds. Does NOT fire timers —
 * call mock_backend_run_timer() for that, so tests control ordering. */
void mock_backend_advance_us(struct mock_backend *m, uint64_t us)
{
    m->now_us += us;
}

/* If a timer is armed and its deadline has passed, disarm it and
 * return true. The caller then invokes vwifi_timer_expired(dev).
 * Returns false if no timer is due. */
bool mock_backend_timer_due(struct mock_backend *m)
{
    if (!m->timer_armed) return false;
    if (m->now_us < m->timer_deadline_us) return false;
    m->timer_armed = false;
    return true;
}

/* Jump the clock straight to the armed deadline. Convenience for
 * tests that just want to skip a dwell without picking a duration. */
bool mock_backend_advance_to_timer(struct mock_backend *m)
{
    if (!m->timer_armed) return false;
    if (m->now_us < m->timer_deadline_us) {
        m->now_us = m->timer_deadline_us;
    }
    m->timer_armed = false;
    return true;
}
