#include "panelbus.h"

#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define BUS_UART       UART_NUM_1
#define BUS_BAUD       1000000
#define RX_BUF_SIZE    1024
#define ENUM_TRIES     6
#define POLL_PERIOD_MS 50
#define PSU_CODE       1      // connector-count code of the power supply node (§7.1)
#define SHORT_RUN      3      // consecutive short bulk pull replies taken as panels gone
#define ENUM_GAP_MIN_MS 250   // gap after an enumeration; doubles while they come back to back
#define ENUM_GAP_MAX_MS 3000  // the stock controller retries an empty edge every 3 s (§4.2)
#define ENUM_QUIET_MS  5000   // this long without an enumeration resets the gap
#define LAYOUT_CHECK_MS 1000  // re-read the layout this often: an added panel is not reliably reported by CC

static const char *TAG = "bus";

static SemaphoreHandle_t s_lock;
static pb_pins_t s_pins = { -1, -1, -1 };
static bool s_ready;
static pb_state_t s_state;
static bool s_touched[PB_MAX_PANELS];
static uint8_t s_status[PB_MAX_PANELS];
static pb_color_t s_colors[PB_MAX_PANELS];   // by layout position
static uint8_t s_brightness = 255;   // re-sent after every enumeration: new panels start at their own level
static pb_touch_cb_t s_touch_cb;
static volatile bool s_poll_on = true;   // also gates automatic enumeration
static volatile bool s_trace;
static int s_miss_run;

// Last traced exchange, so identical polls are counted instead of printed
static struct {
    bool valid;
    uint8_t tx[16], rx[2 * PB_MAX_PANELS + 1];
    size_t ntx, nrx;
    uint32_t repeats;
} s_last;

static void lock_create(void)
{
    if (!s_lock) {
        s_lock = xSemaphoreCreateRecursiveMutex();
    }
}

void pb_lock(void)
{
    lock_create();
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
}

void pb_unlock(void)
{
    xSemaphoreGiveRecursive(s_lock);
}

const pb_pins_t *pb_pins(void)
{
    return &s_pins;
}

const pb_state_t *pb_state(void)
{
    return &s_state;
}

void pb_deinit(void)
{
    pb_lock();
    if (uart_is_driver_installed(BUS_UART)) {
        uart_driver_delete(BUS_UART);
    }
    if (s_ready) {
        // input with pull-up: TX idles high, so U4 stays disabled
        gpio_reset_pin(s_pins.tx);
        gpio_reset_pin(s_pins.rx);
    }
    s_ready = false;
    s_pins = (pb_pins_t){ -1, -1, -1 };
    memset(&s_state, 0, sizeof s_state);
    memset(s_touched, 0, sizeof s_touched);
    memset(s_status, 0, sizeof s_status);
    memset(s_colors, 0, sizeof s_colors);
    pb_unlock();
}

esp_err_t pb_init(const pb_pins_t *pins)
{
    const uart_config_t cfg = {
        .baud_rate = BUS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    pb_lock();
    pb_deinit();
    esp_err_t err = uart_driver_install(BUS_UART, RX_BUF_SIZE, 0, 0, NULL, 0);
    if (err == ESP_OK) {
        err = uart_param_config(BUS_UART, &cfg);
    }
    if (err == ESP_OK) {
        // Push-pull TX: U4 already releases the bus, and an open-drain pad
        // would leave U4's input floating.
        err = uart_set_pin(BUS_UART, pins->tx, pins->rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err == ESP_OK && pins->rc >= 0) {
        // High impedance so the D2/R7/C17 one-shot controls U4. Selecting the
        // GPIO function also drops the USB D+ pull-up if RC is a USB pin.
        const gpio_config_t io = {
            .pin_bit_mask = BIT64(pins->rc),
            .mode = GPIO_MODE_INPUT,
        };
        err = gpio_config(&io);
    }
    if (err == ESP_OK) {
        s_pins = *pins;
        s_ready = true;
    } else if (uart_is_driver_installed(BUS_UART)) {
        uart_driver_delete(BUS_UART);
    }
    pb_unlock();
    return err;
}

void pb_flush(void)
{
    pb_lock();
    if (s_ready) {
        uart_flush_input(BUS_UART);
    }
    pb_unlock();
}

int pb_read_raw(uint8_t *buf, size_t max, int timeout_ms)
{
    pb_lock();
    int n = s_ready ? uart_read_bytes(BUS_UART, buf, max, pdMS_TO_TICKS(timeout_ms)) : -1;
    pb_unlock();
    return n;
}

// Transmit a frame and consume its echo. Lock held.
static bool bus_send(const uint8_t *frame, size_t len)
{
    uart_flush_input(BUS_UART);
    uart_write_bytes(BUS_UART, frame, len);
    uart_wait_tx_done(BUS_UART, pdMS_TO_TICKS(20));

    // U3 is always enabled, so the frame comes back before any reply
    uint8_t tmp[64];
    size_t got = 0;
    bool ok = true;
    int64_t deadline = esp_timer_get_time() + 5000;
    while (got < len && esp_timer_get_time() < deadline) {
        size_t want = len - got < sizeof tmp ? len - got : sizeof tmp;
        int n = uart_read_bytes(BUS_UART, tmp, want, pdMS_TO_TICKS(1));
        if (n > 0) {
            ok = ok && memcmp(tmp, frame + got, n) == 0;
            got += n;
        }
    }
    ok = ok && got == len;
    if (!ok) {
        s_state.echo_errors++;
    }
    return ok;
}

// Read a reply until expect bytes, the layout terminator or the deadline. Lock held.
static int bus_read(uint8_t *buf, size_t max, int expect, int wait_ms)
{
    size_t limit = (expect > 0 && (size_t)expect < max) ? (size_t)expect : max;
    size_t n = 0;
    int64_t deadline = esp_timer_get_time() + (int64_t)wait_ms * 1000;
    while (n < limit && esp_timer_get_time() < deadline) {
        int r = uart_read_bytes(BUS_UART, buf + n, limit - n, pdMS_TO_TICKS(1));
        if (r <= 0) {
            continue;
        }
        n += r;
        if (expect == PB_EXPECT_TERMINATOR && memchr(buf, PB_TERMINATOR, n)) {
            break;
        }
    }
    return n;
}

static void trace(const char *dir, const uint8_t *buf, size_t len, const char *note)
{
    char hex[3 * 48 + 1];
    size_t shown = len < 48 ? len : 48;
    for (size_t i = 0; i < shown; i++) {
        sprintf(hex + 3 * i, "%02X ", buf[i]);
    }
    hex[3 * shown] = '\0';
    ESP_LOGI(TAG, "%s %s%s%s", dir, hex, len > shown ? "..." : "", note);
}

static void trace_xact(const uint8_t *tx, size_t ntx, const uint8_t *rx, size_t nrx, bool reply, bool echo_ok)
{
    if (echo_ok && s_last.valid && ntx == s_last.ntx && nrx == s_last.nrx &&
        memcmp(tx, s_last.tx, ntx) == 0 && (nrx == 0 || memcmp(rx, s_last.rx, nrx) == 0)) {
        s_last.repeats++;
        return;
    }
    if (s_last.repeats) {
        ESP_LOGI(TAG, "   (same x%lu)", (unsigned long)s_last.repeats);
    }
    trace("tx", tx, ntx, echo_ok ? "" : " (echo BAD)");
    if (reply) {
        trace("rx", rx, nrx, nrx ? "" : "(nothing)");
    }
    s_last.valid = ntx <= sizeof s_last.tx && nrx <= sizeof s_last.rx;
    if (s_last.valid) {
        memcpy(s_last.tx, tx, ntx);
        if (nrx) {
            memcpy(s_last.rx, rx, nrx);
        }
        s_last.ntx = ntx;
        s_last.nrx = nrx;
    }
    s_last.repeats = 0;
}

int pb_xact(const uint8_t *frame, size_t len, uint8_t *reply, size_t reply_max,
            int expect, int wait_ms, bool *echo_ok)
{
    pb_lock();
    if (!s_ready) {
        pb_unlock();
        return -1;
    }
    bool ok = bus_send(frame, len);
    int n = 0;
    if (expect != PB_EXPECT_NONE && reply) {
        n = bus_read(reply, reply_max, expect, wait_ms);
    }
    if (s_trace) {
        bool has_reply = expect != PB_EXPECT_NONE && reply;
        trace_xact(frame, len, reply, has_reply ? n : 0, has_reply, ok);
    }
    pb_unlock();
    if (echo_ok) {
        *echo_ok = ok;
    }
    return n;
}

// --- layout string (§7) -----------------------------------------------------

typedef struct {
    const uint8_t *s;
    size_t len, pos;
    pb_state_t *st;
    bool err;
} parser_t;

static int connector_count(int code)
{
    switch (code) {
    case 0: return 8;
    case PSU_CODE: return 0;
    case 7: return 9;
    default: return code;
    }
}

static const char *panel_type(int connectors, const uint8_t *sep, int nsep)
{
    // subtype lives in the first separators of a node (§7.3)
    if (connectors == 3 && nsep >= 1 && ((sep[0] >> 2) & 7) <= 1) {
        return "Mini Triangle";
    }
    if (connectors == 6 && nsep >= 2) {
        switch (sep[1] & 7) {
        case 1: return "Hexagon";
        case 2: return "Triangle";
        case 3: return "Elements";
        }
    }
    return "unknown";
}

static void parse_node(parser_t *p, int parent, int parent_conn)
{
    uint8_t b = p->s[p->pos++];
    int code = (b >> 4) & 7;
    int root = b & 7;
    int n = connector_count(code);

    if (code == PSU_CODE) {
        p->st->psu_parent = parent;
        p->st->psu_conn = parent_conn;
        return;
    }
    if (p->st->npanels >= PB_MAX_PANELS || root >= n) {
        p->err = true;   // corrupted node byte (§9.2)
        return;
    }
    int idx = p->st->npanels++;
    pb_panel_t *panel = &p->st->panels[idx];
    *panel = (pb_panel_t){
        .node = b, .connectors = n, .root = root, .parent = parent, .parent_conn = parent_conn,
    };

    // Walk the connectors in a circle starting at root + 1 (§7.2)
    uint8_t sep[2] = { 0 };
    int nsep = 0;
    int e = (root + 1) % n;
    while (p->pos < p->len && !p->err) {
        uint8_t c = p->s[p->pos];
        if (c & 0x80) {
            parse_node(p, idx, e);
            continue;
        }
        int next = (e + 1) % n;
        if (next == root) {
            break;   // this separator belongs to the parent
        }
        if (nsep < 2) {
            sep[nsep] = c;
        }
        nsep++;
        e = next;
        p->pos++;
    }
    panel->type = panel_type(n, sep, nsep < 2 ? nsep : 2);
}

static esp_err_t parse_layout(pb_state_t *st)
{
    parser_t p = { .s = st->layout, .len = st->layout_len - 1, .st = st };   // without the terminator
    st->npanels = 0;
    st->psu_parent = st->psu_conn = -1;
    if (p.len > 0 && (p.s[0] & 0x80)) {
        parse_node(&p, -1, -1);
    }
    if (!p.err && p.pos == p.len && st->npanels > 0) {
        return ESP_OK;
    }

    // Fall back to counting panel node bytes, which is all colour needs
    st->npanels = 0;
    for (size_t i = 0; i < p.len && st->npanels < PB_MAX_PANELS; i++) {
        uint8_t b = p.s[i];
        if ((b & 0x80) && ((b >> 4) & 7) != PSU_CODE) {
            st->panels[st->npanels++] = (pb_panel_t){
                .node = b, .connectors = connector_count((b >> 4) & 7), .root = b & 7,
                .parent = -1, .parent_conn = -1, .type = "unparsed",
            };
        }
    }
    return ESP_ERR_INVALID_RESPONSE;
}

// Returns the length of the layout string including the terminator, 0 if none.
static int layout_detect(uint8_t *buf, int wait_ms, int *heard)
{
    static const uint8_t frame[] = { PB_LAYOUT_DETECT };
    int n = pb_xact(frame, sizeof frame, buf, PB_MAX_LAYOUT, PB_EXPECT_TERMINATOR, wait_ms, NULL);
    if (n <= 0) {
        return 0;
    }
    *heard += n;
    const uint8_t *end = memchr(buf, PB_TERMINATOR, n);
    return end ? end - buf + 1 : 0;
}

esp_err_t pb_enumerate(void)
{
    static const uint8_t root_detect[] = { PB_ROOT_DETECT };
    static uint8_t reads[ENUM_TRIES][PB_MAX_LAYOUT];
    int lens[ENUM_TRIES] = { 0 };
    int best = -1;
    bool stable = false;
    int heard = 0;

    pb_lock();
    if (!s_ready) {
        pb_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    int window = 200 + 40 * (s_state.npanels > 0 ? s_state.npanels : 1);

    // The head of the string can arrive corrupted on large assemblies (§9.2):
    // read until one value repeats.
    for (int i = 0; i < ENUM_TRIES && !stable; i++) {
        pb_xact(root_detect, sizeof root_detect, NULL, 0, PB_EXPECT_NONE, 0, NULL);
        vTaskDelay(pdMS_TO_TICKS(20));
        lens[i] = layout_detect(reads[i], window, &heard);
        if (!lens[i]) {
            lens[i] = layout_detect(reads[i], 1500, &heard);
        }
        if (lens[i] && best < 0) {
            best = i;
        }
        for (int j = 0; j < i && lens[i]; j++) {
            if (lens[j] == lens[i] && memcmp(reads[j], reads[i], lens[i]) == 0) {
                best = i;
                stable = true;
                break;
            }
        }
        if (i >= 1 && heard == 0) {
            break;   // silent bus, no point in waiting for four more windows
        }
    }

    memset(&s_state.layout, 0, sizeof s_state.layout);
    s_state.layout_len = 0;
    s_state.npanels = 0;
    s_state.hotplug = false;
    memset(s_touched, 0, sizeof s_touched);
    memset(s_status, 0, sizeof s_status);
    memset(s_colors, 0, sizeof s_colors);
    s_miss_run = 0;

    esp_err_t err = ESP_ERR_NOT_FOUND;
    if (best >= 0) {
        memcpy(s_state.layout, reads[best], lens[best]);
        s_state.layout_len = lens[best];
        s_state.layout_stable = stable;
        err = parse_layout(&s_state);
    }
    if (s_state.npanels > 0) {
        s_state.enumerations++;
        const uint8_t frame[] = { PB_GLOBAL_CMD, PB_CMD_BRIGHTNESS, s_brightness };
        pb_xact(frame, sizeof frame, NULL, 0, PB_EXPECT_NONE, 0, NULL);
    }
    pb_unlock();
    return err;
}

// Root detect and one layout detect, without adopting the result. True if a complete string came back and
// it differs from the enumerated one. A panel plugged in while running joins only through a fresh 00 + 80:
// on the real assembly it stayed unclaimed (dim white) until the next enumeration (2026-09-29).
static bool layout_changed(void)
{
    static const uint8_t root_detect[] = { PB_ROOT_DETECT };
    static uint8_t buf[PB_MAX_LAYOUT];
    int heard = 0;

    pb_lock();
    pb_xact(root_detect, sizeof root_detect, NULL, 0, PB_EXPECT_NONE, 0, NULL);
    vTaskDelay(pdMS_TO_TICKS(20));
    int len = layout_detect(buf, 200 + 40 * s_state.npanels, &heard);
    bool changed = len > 0 && (len != (int)s_state.layout_len || memcmp(buf, s_state.layout, len) != 0);
    pb_unlock();
    return changed;
}

// --- polling and colour (§6) --------------------------------------------------

int pb_bulk_pull(void)
{
    static const uint8_t frame[] = { PB_BULK_PULL };
    uint8_t reply[2 * PB_MAX_PANELS + 2];

    pb_lock();
    int n = s_state.npanels;
    int expect = 2 * n;
    int got = pb_xact(frame, sizeof frame, reply, sizeof reply, expect, 40 + 20 * n, NULL);
    if (got < 0) {
        pb_unlock();
        return got;
    }
    if (got == expect) {
        // A hot-plug CC can follow the pairs, and so can the pairs of panels added since the enumeration
        got += bus_read(reply + got, sizeof reply - got, PB_EXPECT_NONE, 2);
    }

    s_state.polls++;
    if (got < expect) {
        s_state.poll_misses++;
        s_miss_run++;
    } else {
        s_miss_run = 0;
    }

    // §8: a panel was added, removed or moved when the pairs of the panels still present end in CC.
    // Panels that answer without having been enumerated, or that stop answering, mean the same.
    const char *change = NULL;
    if (got > 0 && reply[got - 1] == PB_HOTPLUG && (got - 1) % 2 == 0) {
        change = "CC";
    } else if (got >= expect + 2) {
        change = "more panels answer";
    } else if (s_miss_run >= SHORT_RUN) {
        change = got ? "fewer panels answer" : "no answer";
    }
    if (change && !s_state.hotplug) {
        s_state.hotplug = true;
        ESP_LOGW(TAG, "hot-plug (%s, %d of %d bytes), re-enumerating", change, got, expect);
    }

    // One pair per panel in LAYOUT order: the first pair is the panel attached
    // to this board. That is the reverse of the push chunk order, and PROTOCOL.md
    // §5.3 gets it wrong (verified by touch on 2026-09-26).
    // 00 00 idle, 10 00 first poll or touch released, 11..17 touch. The far
    // panel once reported 20 00, which PROTOCOL.md does not describe, so test
    // the touch bit alone.
    for (int pos = 0; pos < n && 2 * pos + 1 < got; pos++) {
        uint8_t st = reply[2 * pos];
        bool touched = (st & 0x10) && (st & 0x0F);
        if (touched != s_touched[pos]) {
            ESP_LOGI(TAG, "panel %d touch %s  %02X %02X",
                     pos, touched ? "start" : "end", reply[2 * pos], reply[2 * pos + 1]);
            if (s_touch_cb) {
                s_touch_cb(pos, touched, st);
            }
        } else if (!touched && st != s_status[pos] && (st & ~0x10)) {
            ESP_LOGI(TAG, "panel %d status %02X %02X", pos, reply[2 * pos], reply[2 * pos + 1]);
        }
        s_touched[pos] = touched;
        s_status[pos] = st;
    }
    pb_unlock();
    return got;
}

esp_err_t pb_push(const pb_color_t *colors, int n)
{
    static uint8_t frame[2 + 6 * PB_MAX_PANELS];

    if (n > PB_MAX_PANELS) {
        return ESP_ERR_INVALID_ARG;
    }
    pb_lock();
    size_t len = 0;
    frame[len++] = PB_BULK_PUSH;
    frame[len++] = 0x03;
    for (int i = 0; i < n; i++) {
        const pb_color_t *c = &colors[i];
        if (c->skip) {
            frame[len++] = 0x01;
            frame[len++] = 0xFF;
        } else {
            frame[len++] = 0x05;
            frame[len++] = c->t;
            frame[len++] = c->r;
            frame[len++] = c->g;
            frame[len++] = c->b;
            frame[len++] = c->w;
        }
    }
    int r = pb_xact(frame, len, NULL, 0, PB_EXPECT_NONE, 0, NULL);
    if (r >= 0) {
        for (int i = 0; i < n; i++) {
            if (!colors[i].skip) {
                s_colors[n - 1 - i] = colors[i];
            }
        }
    }
    pb_unlock();
    return r < 0 ? ESP_ERR_INVALID_STATE : ESP_OK;
}

esp_err_t pb_set_panel(int layout_pos, const pb_color_t *color)
{
    pb_color_t colors[PB_MAX_PANELS];
    esp_err_t err = ESP_ERR_INVALID_ARG;

    pb_lock();
    int n = s_state.npanels;
    if (layout_pos >= 0 && layout_pos < n) {
        for (int i = 0; i < n; i++) {
            colors[i] = (pb_color_t){ .skip = true };
        }
        colors[pb_chunk_index(n, layout_pos)] = *color;
        colors[pb_chunk_index(n, layout_pos)].skip = false;
        err = pb_push(colors, n);
    }
    pb_unlock();
    return err;
}

pb_color_t pb_panel_color(int layout_pos)
{
    pb_color_t c = { 0 };
    pb_lock();
    if (layout_pos >= 0 && layout_pos < PB_MAX_PANELS) {
        c = s_colors[layout_pos];
    }
    pb_unlock();
    return c;
}

void pb_on_touch(pb_touch_cb_t cb)
{
    s_touch_cb = cb;
}

esp_err_t pb_fill(uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t t)
{
    pb_color_t colors[PB_MAX_PANELS];

    pb_lock();
    int n = s_state.npanels;
    for (int i = 0; i < n; i++) {
        colors[i] = (pb_color_t){ .r = r, .g = g, .b = b, .w = w, .t = t };
    }
    esp_err_t err = n > 0 ? pb_push(colors, n) : ESP_ERR_INVALID_STATE;
    pb_unlock();
    return err;
}

esp_err_t pb_brightness(uint8_t value)
{
    const uint8_t frame[] = { PB_GLOBAL_CMD, PB_CMD_BRIGHTNESS, value };
    s_brightness = value;
    return pb_xact(frame, sizeof frame, NULL, 0, PB_EXPECT_NONE, 0, NULL) < 0 ? ESP_ERR_INVALID_STATE : ESP_OK;
}

static void auto_enumerate(const char *why)
{
    static int reported = -1;

    int64_t start = esp_timer_get_time();
    esp_err_t err = pb_enumerate();
    if (s_state.npanels != reported || strcmp(why, "no panels") != 0) {   // quiet while retrying an empty bus
        reported = s_state.npanels;
        ESP_LOGI(TAG, "auto-enum (%s): %d panel(s) in %d ms%s", why, s_state.npanels,
                 (int)((esp_timer_get_time() - start) / 1000),
                 s_state.npanels && err != ESP_OK ? ", layout not parsed" : "");
    }
}

// Nobody is there to type `enum` when WLED or Matter drives the panels: enumerate at start-up, retry
// while nothing answers, and straight after a hot-plug. Once a second, the layout is read again to catch
// panels that were added without a CC. Enumerations that keep coming back to back are spaced out, up to 3 s.
static void poll_task(void *arg)
{
    int64_t last_enum = 0, next_enum = 0, next_check = 0;
    int gap_ms = ENUM_GAP_MIN_MS;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
        if (!s_poll_on || !s_ready) {
            continue;
        }
        int64_t now = esp_timer_get_time();
        const char *why = s_state.npanels == 0 ? "no panels" : s_state.hotplug ? "hot-plug" : NULL;
        if (why && now >= next_enum) {
            bool quiet = last_enum == 0 || now - last_enum > ENUM_QUIET_MS * 1000LL;
            gap_ms = quiet ? ENUM_GAP_MIN_MS : (gap_ms * 2 < ENUM_GAP_MAX_MS ? gap_ms * 2 : ENUM_GAP_MAX_MS);
            last_enum = now;
            next_enum = now + gap_ms * 1000LL;
            next_check = now + LAYOUT_CHECK_MS * 1000LL;
            auto_enumerate(why);
        } else if (s_state.npanels > 0 && !s_state.hotplug && now >= next_check) {
            next_check = now + LAYOUT_CHECK_MS * 1000LL;
            if (layout_changed()) {
                s_state.hotplug = true;
                ESP_LOGW(TAG, "hot-plug (layout changed), re-enumerating");
            }
        } else if (s_state.npanels > 0) {
            pb_bulk_pull();   // keeps the panels polled while an enumeration waits for its gap
        }
    }
}

void pb_poll_start(void)
{
    lock_create();
    xTaskCreate(poll_task, "bus_poll", 6144, NULL, 5, NULL);   // touch callbacks push colour from here
}

void pb_poll_enable(bool on)
{
    s_poll_on = on;
}

bool pb_poll_enabled(void)
{
    return s_poll_on;
}

void pb_trace(bool on)
{
    pb_lock();
    s_last.valid = false;
    s_last.repeats = 0;
    s_trace = on;
    pb_unlock();
}

bool pb_tracing(void)
{
    return s_trace;
}
