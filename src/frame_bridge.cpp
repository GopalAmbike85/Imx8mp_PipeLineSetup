#include "frame_bridge.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Bounded substring search - 'in' isn't guaranteed null-terminated, so
 * strstr() can't be used directly. */
static const char *find_substr(const char *haystack, size_t haystack_len, const char *needle)
{
    size_t needle_len = strlen(needle);
    if (needle_len == 0 || needle_len > haystack_len) {
        return NULL;
    }
    for (size_t i = 0; i + needle_len <= haystack_len; i++) {
        if (memcmp(haystack + i, needle, needle_len) == 0) {
            return haystack + i;
        }
    }
    return NULL;
}

/* Given the position right after a JSON key's colon, finds the extent of
 * its numeric value (skipping any whitespace first). */
static void find_value_span(const char *in, size_t in_len, size_t key_end,
                             size_t *value_start, size_t *value_end)
{
    size_t start = key_end;
    while (start < in_len && (in[start] == ' ' || in[start] == '\t')) {
        start++;
    }
    size_t end = start;
    while (end < in_len) {
        char c = in[end];
        if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E') {
            end++;
        } else {
            break;
        }
    }
    *value_start = start;
    *value_end = end;
}

/* Parses the numeric span [start, end) of 'in' as a double, without letting
 * strtod() scan past 'end' - 'in' isn't guaranteed null-terminated, and may
 * be a reused per-session buffer with stale bytes from a previous frame
 * right after 'end', so strtod(in + start, NULL) could otherwise keep
 * reading past the value found by find_value_span(). Copies the (capped)
 * span into a local null-terminated buffer first instead. */
static double parse_double_span(const char *in, size_t start, size_t end)
{
    char buf[64];
    size_t len = end - start;
    if (len >= sizeof(buf)) {
        len = sizeof(buf) - 1;
    }
    memcpy(buf, in + start, len);
    buf[len] = '\0';
    return strtod(buf, NULL);
}

/* The controller's own send-time field has been observed under two
 * different key names across real controller frame variants (see
 * controller-bridge-motor-inference memory) - "ntp_time" (the documented
 * schema and one live capture) and "timestamp" (a second live capture,
 * matching the offline test harness's CSV column naming). Try both, in
 * that order, so timing-chain fields get added regardless of which
 * variant a given controller/firmware sends. Returns true and fills
 * '*key_start'/'*key_len' if either was found. */
static int find_sent_time_key(const char *in, size_t in_len,
                               size_t *key_start, size_t *key_len)
{
    static const char NTP_KEY[] = "\"ntp_time\":";
    static const char TIMESTAMP_KEY[] = "\"timestamp\":";

    const char *pos = find_substr(in, in_len, NTP_KEY);
    if (pos) {
        *key_start = (size_t)(pos - in);
        *key_len = sizeof(NTP_KEY) - 1;
        return 1;
    }
    pos = find_substr(in, in_len, TIMESTAMP_KEY);
    if (pos) {
        *key_start = (size_t)(pos - in);
        *key_len = sizeof(TIMESTAMP_KEY) - 1;
        return 1;
    }
    return 0;
}

int bridge_extract_sent_time(const char *in, size_t in_len, double *out_value)
{
    size_t key_start, key_len;
    if (!find_sent_time_key(in, in_len, &key_start, &key_len)) {
        return -1;
    }
    size_t value_start, value_end;
    find_value_span(in, in_len, key_start + key_len, &value_start, &value_end);
    if (value_start == value_end) {
        return -1;
    }
    *out_value = parse_double_span(in, value_start, value_end);
    return 0;
}

int bridge_stamp_frame(const char *in, size_t in_len,
                        double received_ntp_time, double forward_ntp_time,
                        char *out, size_t out_cap,
                        double *out_frame_transfer_time, double *out_processing_time)
{
    size_t ntp_key_start, ntp_key_len;
    if (!find_sent_time_key(in, in_len, &ntp_key_start, &ntp_key_len)) {
        if (in_len > out_cap) {
            return -1;
        }
        memcpy(out, in, in_len);
        return (int)in_len;
    }
    size_t sent_value_start, sent_value_end;
    find_value_span(in, in_len, ntp_key_start + ntp_key_len, &sent_value_start, &sent_value_end);

    double sent_ntp_time = parse_double_span(in, sent_value_start, sent_value_end);
    double frame_transfer_time = received_ntp_time - sent_ntp_time;
    double processing_time = forward_ntp_time - received_ntp_time;

    if (out_frame_transfer_time) {
        *out_frame_transfer_time = frame_transfer_time;
    }
    if (out_processing_time) {
        *out_processing_time = processing_time;
    }

    /* frame_transfer_time is derived from the controller-supplied
     * "ntp_time"/"timestamp" value (sent_ntp_time above), so a malicious or
     * malformed frame can drive it to a magnitude whose "%.6f" rendering
     * doesn't fit in 32 bytes - snprintf() then reports the length it WOULD
     * have written, not what's actually in the buffer, so every one of
     * these must be checked for truncation (>= sizeof(buf)), not just a
     * negative return, or the APPEND() calls below would memcpy past the
     * end of the respective *_buf. */
    char forward_buf[32];
    int forward_len = snprintf(forward_buf, sizeof(forward_buf), "%.6f", forward_ntp_time);
    char received_buf[32];
    int received_len = snprintf(received_buf, sizeof(received_buf), "%.6f", received_ntp_time);
    char processing_buf[32];
    int processing_len = snprintf(processing_buf, sizeof(processing_buf), "%.6f", processing_time);
    char transfer_buf[32];
    int transfer_len = snprintf(transfer_buf, sizeof(transfer_buf), "%.6f", frame_transfer_time);
    if (forward_len < 0 || (size_t)forward_len >= sizeof(forward_buf) ||
        received_len < 0 || (size_t)received_len >= sizeof(received_buf) ||
        processing_len < 0 || (size_t)processing_len >= sizeof(processing_buf) ||
        transfer_len < 0 || (size_t)transfer_len >= sizeof(transfer_buf)) {
        return -1;
    }

    static const char SENT_KEY[] = "\"controller_sent_ntp_time\":";
    static const char NEW_NTP_KEY[] = ",\"ntp_time\":";
    static const char PROCESSING_KEY[] = ",\"base_station_processing_time\":";
    static const char TRANSFER_KEY[] = ",\"frame_transfer_time\":";
    static const char RECEIVED_KEY[] = ",\"controller_to_base_station_received_ntp_time\":";
    const size_t sent_key_len = sizeof(SENT_KEY) - 1;
    const size_t new_ntp_key_len = sizeof(NEW_NTP_KEY) - 1;
    const size_t processing_key_len = sizeof(PROCESSING_KEY) - 1;
    const size_t transfer_key_len = sizeof(TRANSFER_KEY) - 1;
    const size_t received_key_len = sizeof(RECEIVED_KEY) - 1;

    size_t sent_value_len = sent_value_end - sent_value_start;
    size_t prefix_len = ntp_key_start;               /* everything before the matched sent-time key */
    size_t tail_len = in_len - sent_value_end;        /* everything after the original value */

    size_t total = prefix_len
                    + sent_key_len + sent_value_len
                    + new_ntp_key_len + (size_t)forward_len
                    + processing_key_len + (size_t)processing_len
                    + transfer_key_len + (size_t)transfer_len
                    + received_key_len + (size_t)received_len
                    + tail_len;
    if (total > out_cap) {
        return -1;
    }

    size_t pos = 0;
#define APPEND(src, n) do { memcpy(out + pos, (src), (n)); pos += (n); } while (0)
    APPEND(in, prefix_len);
    APPEND(SENT_KEY, sent_key_len);
    APPEND(in + sent_value_start, sent_value_len);
    APPEND(NEW_NTP_KEY, new_ntp_key_len);
    APPEND(forward_buf, (size_t)forward_len);
    APPEND(PROCESSING_KEY, processing_key_len);
    APPEND(processing_buf, (size_t)processing_len);
    APPEND(TRANSFER_KEY, transfer_key_len);
    APPEND(transfer_buf, (size_t)transfer_len);
    APPEND(RECEIVED_KEY, received_key_len);
    APPEND(received_buf, (size_t)received_len);
    APPEND(in + sent_value_end, tail_len);
#undef APPEND

    return (int)pos;
}

/* Shared by bridge_extract_joystick() for one key - finds 'key' in 'in'
 * and parses the numeric value immediately following its colon, reusing
 * the same find_substr()/find_value_span() helpers as bridge_stamp_frame().
 * Returns 0 and fills '*out_value' if found, -1 otherwise. */
static int extract_double_field(const char *in, size_t in_len, const char *key,
                                 double *out_value)
{
    const char *key_pos = find_substr(in, in_len, key);
    if (!key_pos) {
        return -1;
    }
    size_t key_start = (size_t)(key_pos - in);
    size_t value_start, value_end;
    find_value_span(in, in_len, key_start + strlen(key), &value_start, &value_end);
    if (value_start == value_end) {
        return -1;
    }
    *out_value = parse_double_span(in, value_start, value_end);
    return 0;
}

int bridge_extract_joystick(const char *in, size_t in_len,
                             double *out_x, double *out_y)
{
    double x, y;
    if (extract_double_field(in, in_len, "\"joystick_x\":", &x) != 0) {
        return -1;
    }
    if (extract_double_field(in, in_len, "\"joystick_y\":", &y) != 0) {
        return -1;
    }
    *out_x = x;
    *out_y = y;
    return 0;
}

/* Shared by bridge_reformat_joystick() for one key - replaces the numeric
 * VALUE following 'key' (the key text itself is left untouched) with
 * 'value' formatted as "%.8f". The new value's length need not match the
 * old one, so the tail (everything after the old value) is shifted to
 * make room, same technique as bridge_append_motor_fields()'s brace
 * splice. Returns the new total length, 'len' unchanged if 'key' isn't
 * present (fail-open - a frame variant without this field just isn't
 * reformatted), or -1 if the result would exceed 'cap'. */
static int replace_value_for_key(char *buf, size_t len, size_t cap,
                                  const char *key, double value)
{
    const char *key_pos = find_substr(buf, len, key);
    if (!key_pos) {
        return (int)len;
    }
    size_t key_start = (size_t)(key_pos - buf);
    size_t value_start, value_end;
    find_value_span(buf, len, key_start + strlen(key), &value_start, &value_end);

    char value_buf[32];
    int value_len = snprintf(value_buf, sizeof(value_buf), "%.8f", value);
    if (value_len < 0 || (size_t)value_len >= sizeof(value_buf)) {
        return -1;
    }

    size_t old_value_len = value_end - value_start;
    size_t tail_len = len - value_end;
    size_t new_len = value_start + (size_t)value_len + tail_len;
    if (new_len > cap) {
        return -1;
    }

    memmove(buf + value_start + value_len, buf + value_end, tail_len);
    memcpy(buf + value_start, value_buf, (size_t)value_len);
    (void)old_value_len;  /* only needed conceptually, not by the move above */

    return (int)new_len;
}

int bridge_reformat_joystick(char *buf, size_t len, size_t cap,
                              double joystick_x, double joystick_y)
{
    int n = replace_value_for_key(buf, len, cap, "\"joystick_x\":", joystick_x);
    if (n < 0) {
        return -1;
    }
    return replace_value_for_key(buf, (size_t)n, cap, "\"joystick_y\":", joystick_y);
}

/* Shared by bridge_append_motor_fields()/bridge_append_field(): splices
 * 'text' ('text_len' bytes, already formatted as one or more leading-comma
 * JSON fields) into 'buf' just before the top-level closing brace. 'buf' is
 * always a single flat JSON object here (nested objects like
 * "joystick_position" close well before the end), so the last '}' in the
 * buffer is it. Returns the new total length, or -1 if 'buf' has no '}' or
 * the result would exceed 'cap' (buf is left unmodified in either case). */
static int insert_before_closing_brace(char *buf, size_t len, size_t cap,
                                        const char *text, size_t text_len)
{
    size_t close_pos = len;
    while (close_pos > 0 && buf[close_pos - 1] != '}') {
        close_pos--;
    }
    if (close_pos == 0) {
        return -1;
    }
    close_pos--; /* index of the '}' itself */

    size_t tail_len = len - close_pos; /* the '}' and anything after it */
    size_t total = close_pos + text_len + tail_len;
    if (total > cap) {
        return -1;
    }

    /* Shift the tail (closing brace onward) right to make room, then
     * insert the new text in the gap - done back-to-front so the shift
     * doesn't overwrite bytes it still needs to read (safe even though
     * source/destination ranges overlap, unlike memcpy). */
    memmove(buf + close_pos + text_len, buf + close_pos, tail_len);
    memcpy(buf + close_pos, text, text_len);

    return (int)total;
}

int bridge_append_motor_fields(char *buf, size_t len, size_t cap,
                                double m1, double m2, double m3, double m4)
{
    char fields[192];
    int fields_len = snprintf(fields, sizeof(fields),
                               ",\"motor_1_pos\":%.6f,\"motor_2_pos\":%.6f,"
                               "\"motor_3_pos\":%.6f,\"motor_4_insertion_pos\":%.6f",
                               m1, m2, m3, m4);
    if (fields_len < 0 || (size_t)fields_len >= sizeof(fields)) {
        return -1;
    }
    return insert_before_closing_brace(buf, len, cap, fields, (size_t)fields_len);
}

int bridge_append_field(char *buf, size_t len, size_t cap,
                         const char *key, double value)
{
    char field[128];
    int field_len = snprintf(field, sizeof(field), ",\"%s\":%.6f", key, value);
    if (field_len < 0 || (size_t)field_len >= sizeof(field)) {
        return -1;
    }
    return insert_before_closing_brace(buf, len, cap, field, (size_t)field_len);
}
