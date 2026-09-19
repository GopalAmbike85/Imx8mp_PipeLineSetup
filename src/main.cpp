#include "bridge_server.h"
#include "controller_receiver.h"
#include "gpio_toggle.h"
#include "motor_inference.h"
#include "timing_log.h"

#include <libwebsockets.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEFAULT_CONTROLLER_PORT 8081
#define DEFAULT_ICU_PORT 8080
#define DEFAULT_MODEL_PATH "TfLSTM_l2_w128_s20_int8.tflite"
/* NPU by default as of 2026-09-07 (explicit user request, accuracy caveat
 * unresolved - see motor_inference.h). Pass "cpu" as the 6th argument to
 * fall back to the CPU path instead. */
#define DEFAULT_DELEGATE_PATH "/usr/lib/libvx_delegate.so"

static volatile int s_running = 1;

static void handle_signal(int signum)
{
    (void)signum;
    s_running = 0;
}

static void print_usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s [controller_port] [icu_port] [cert_path] [key_path] [model_path] [delegate_path|cpu] [timing_log_path]\n",
            argv0);
}

int main(int argc, char **argv)
{
    int controller_port = DEFAULT_CONTROLLER_PORT;
    int icu_port = DEFAULT_ICU_PORT;
    const char *cert_path = NULL;
    const char *key_path = NULL;
    const char *model_path = DEFAULT_MODEL_PATH;
    const char *delegate_path = DEFAULT_DELEGATE_PATH;
    /* Empty = disabled (default) - see timing_log.h. Pass a 7th argument
     * (e.g. "timing_log.csv") to capture a per-frame CSV for offline
     * latency analysis. */
    const char *timing_log_path = "";

    if (argc > 1) {
        controller_port = atoi(argv[1]);
        if (controller_port <= 0 || controller_port > 65535) {
            print_usage(argv[0]);
            return 1;
        }
    }
    if (argc > 2) {
        icu_port = atoi(argv[2]);
        if (icu_port <= 0 || icu_port > 65535) {
            print_usage(argv[0]);
            return 1;
        }
    }
    /* cert_path/key_path are both optional - plain ws:// on both vhosts
     * unless both are given. */
    if (argc > 3) {
        cert_path = argv[3];
    }
    if (argc > 4) {
        key_path = argv[4];
    }
    if (argc > 5) {
        model_path = argv[5];
    }
    if (argc > 6) {
        /* "cpu" (any case not checked - exact lowercase match, matching
         * this project's plain/minimal argument-parsing style elsewhere)
         * selects the CPU fallback path; anything else is treated as an
         * external-delegate .so path. */
        delegate_path = (strcmp(argv[6], "cpu") == 0) ? "" : argv[6];
    }
    if (argc > 7) {
        timing_log_path = argv[7];
    }
    if ((cert_path != NULL) != (key_path != NULL)) {
        print_usage(argv[0]);
        fprintf(stderr, "cert_path and key_path must both be given, or both omitted\n");
        return 1;
    }
    int tls_enabled = (cert_path != NULL && key_path != NULL);

    /* Loads the model and builds the interpreter (see motor_inference.h)
     * before anything can connect - a controller frame arriving before
     * this completes is not a case this code handles, so it must finish
     * first. When delegate_path selects the NPU, this call also performs
     * a warm-up Invoke() to absorb its one-time compile cost here, before
     * either vhost can accept a connection - see motor_inference.h for
     * why that matters on this specific (single-threaded, synchronous)
     * server. */
    if (motor_inference_init(model_path, delegate_path) != 0) {
        fprintf(stderr, "failed to initialize motor inference from model: %s\n",
                model_path);
        return 1;
    }

    /* Diagnostic only - a failed open here is logged by timing_log_init()
     * itself and must never stop the real controller-to-ICU pipeline from
     * starting, so its return value is intentionally not checked here. */
    timing_log_init(timing_log_path);

    /* Diagnostic only, same non-fatal posture as timing_log_init() above -
     * a hardware-probeable marker of the receive-to-forward window is a
     * nice-to-have, not something that should be able to stop the real
     * pipeline if this board's GPIO setup ever differs from what
     * gpio_toggle.h documents. */
    gpio_toggle_init();

    /* SIGINT (Ctrl+C) and SIGTERM trigger a clean shutdown of the service
     * loop below instead of an abrupt process kill. */
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    lws_set_log_level(LLL_ERR | LLL_WARN | LLL_NOTICE, NULL);

    struct lws_protocols controller_protocols[] = {
        {
            .name = "controller",
            .callback = controller_receiver_callback(),
            .per_session_data_size = controller_receiver_session_size(),
            .rx_buffer_size = 0,
        },
        LWS_PROTOCOL_LIST_TERM
    };

    struct lws_protocols icu_protocols[] = {
        {
            .name = "telemetry",
            .callback = bridge_server_callback(),
            .per_session_data_size = 0,
            .rx_buffer_size = 0,
        },
        LWS_PROTOCOL_LIST_TERM
    };

    /* LWS_SERVER_OPTION_EXPLICIT_VHOSTS: without this, lws_create_context()
     * would create one implicit default vhost from this same info struct -
     * CONTEXT_PORT_NO_LISTEN alone does not suppress that. Both vhosts are
     * created explicitly below via lws_create_vhost() instead, each with
     * its own port and protocol table - same pattern as WebSocketApp's
     * ws_server.c. */
    struct lws_context_creation_info info;
    memset(&info, 0, sizeof(info));
    info.port = CONTEXT_PORT_NO_LISTEN;
    info.options = LWS_SERVER_OPTION_EXPLICIT_VHOSTS;
    info.gid = -1;
    info.uid = -1;
    if (tls_enabled) {
        /* Performs the actual one-time TLS library init; each vhost below
         * additionally needs this same flag on its own info to have TLS
         * enabled for that specific vhost. */
        info.options |= LWS_SERVER_OPTION_DO_SSL_GLOBAL_INIT;
    }

    struct lws_context *context = lws_create_context(&info);
    if (!context) {
        fprintf(stderr, "failed to create libwebsockets context\n");
        return 1;
    }

    struct lws_context_creation_info controller_vh_info;
    memset(&controller_vh_info, 0, sizeof(controller_vh_info));
    controller_vh_info.port = controller_port;
    controller_vh_info.protocols = controller_protocols;
    controller_vh_info.gid = -1;
    controller_vh_info.uid = -1;
    if (tls_enabled) {
        controller_vh_info.options |= LWS_SERVER_OPTION_DO_SSL_GLOBAL_INIT;
        controller_vh_info.ssl_cert_filepath = cert_path;
        controller_vh_info.ssl_private_key_filepath = key_path;
    }
    if (!lws_create_vhost(context, &controller_vh_info)) {
        fprintf(stderr, "failed to create controller vhost on port %d\n", controller_port);
        lws_context_destroy(context);
        return 1;
    }

    struct lws_context_creation_info icu_vh_info;
    memset(&icu_vh_info, 0, sizeof(icu_vh_info));
    icu_vh_info.port = icu_port;
    icu_vh_info.protocols = icu_protocols;
    icu_vh_info.gid = -1;
    icu_vh_info.uid = -1;
    if (tls_enabled) {
        icu_vh_info.options |= LWS_SERVER_OPTION_DO_SSL_GLOBAL_INIT;
        icu_vh_info.ssl_cert_filepath = cert_path;
        icu_vh_info.ssl_private_key_filepath = key_path;
    }
    if (!lws_create_vhost(context, &icu_vh_info)) {
        fprintf(stderr, "failed to create ICU vhost on port %d\n", icu_port);
        lws_context_destroy(context);
        return 1;
    }

    bridge_server_start(context);

    lwsl_notice("controller-facing listener on port %d, %s\n",
                controller_port, tls_enabled ? "wss:// (TLS)" : "ws:// (plain)");
    lwsl_notice("ICU-facing listener on port %d, %s\n",
                icu_port, tls_enabled ? "wss:// (TLS)" : "ws:// (plain)");
    lwsl_notice("bridging directly in memory - no log file, no second process\n");
    lwsl_notice("motor inference active (%s), model: %s\n",
                (delegate_path[0] != '\0') ? "NPU" : "CPU", model_path);
    if (timing_log_path[0] != '\0') {
        lwsl_notice("timing log: appending per-frame CSV to %s\n", timing_log_path);
    }

    while (s_running) {
        lws_service(context, 1000);
    }

    timing_log_close();
    lws_context_destroy(context);
    return 0;
}
