// gpio_toggle.cpp - see gpio_toggle.h for the pin choice and rationale.

#include "gpio_toggle.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

namespace {

constexpr int kGpioLine = 5; /* Dahlia X20 pin 29 "GPIO_3" = SODIMM 210 = gpio-5 */

bool g_ready = false;

/* Writes 'text' to the sysfs file at 'path'. Returns true on success. */
bool write_sysfs(const char *path, const char *text)
{
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        std::fprintf(stderr, "gpio_toggle: open(%s) failed: %s\n", path, std::strerror(errno));
        return false;
    }
    size_t len = std::strlen(text);
    ssize_t n = write(fd, text, len);
    close(fd);
    if (n != (ssize_t)len) {
        std::fprintf(stderr, "gpio_toggle: write(%s) failed: %s\n", path, std::strerror(errno));
        return false;
    }
    return true;
}

bool path_exists(const char *path)
{
    return access(path, F_OK) == 0;
}

} // namespace

int gpio_toggle_init(void)
{
    char value_path[64];
    std::snprintf(value_path, sizeof(value_path), "/sys/class/gpio/gpio%d/value", kGpioLine);

    if (!path_exists(value_path)) {
        char line_str[16];
        std::snprintf(line_str, sizeof(line_str), "%d", kGpioLine);
        /* Already-exported is not an error - a previous run of this same
         * process (or another) may have left it exported; only a missing
         * value file after this attempt is a real failure. */
        write_sysfs("/sys/class/gpio/export", line_str);
    }

    char direction_path[64];
    std::snprintf(direction_path, sizeof(direction_path), "/sys/class/gpio/gpio%d/direction", kGpioLine);
    if (!write_sysfs(direction_path, "out")) {
        std::fprintf(stderr, "gpio_toggle: failed to configure gpio%d as output - "
                             "timing marker disabled, rest of the pipeline unaffected\n",
                     kGpioLine);
        return -1;
    }

    if (!write_sysfs(value_path, "0")) {
        return -1;
    }

    g_ready = true;
    return 0;
}

void gpio_toggle_set(int state)
{
    if (!g_ready) {
        return;
    }
    char value_path[64];
    std::snprintf(value_path, sizeof(value_path), "/sys/class/gpio/gpio%d/value", kGpioLine);
    write_sysfs(value_path, state ? "1" : "0");
}
