#ifndef GPIO_TOGGLE_H
#define GPIO_TOGGLE_H

/*
 * Toggles one GPIO line as a hardware-visible (oscilloscope/logic-analyzer)
 * marker of the controller-frame processing window - a clock-independent
 * cross-check against the software timing fields in timing_log.h, which
 * depend on the controller's own (sometimes wildly desynced) clock.
 *
 * Pin: Dahlia carrier board, connector X20 (Primary Extension Header) pin
 * 29, silkscreened "GPIO_3" (SODIMM pin 210, Linux gpio-5 on gpiochip0 -
 * confirmed via the official Dahlia Carrier Board Datasheet and cross-
 * checked as unclaimed by any driver on this board via
 * /sys/kernel/debug/gpio). Logic level +1.8V. Ground reference for probing
 * is available right next to it at X20 pin 31.
 *
 * Implemented via the sysfs GPIO interface (/sys/class/gpio/...) rather
 * than libgpiod, since no libgpiod userspace tools (gpiodetect/gpioset/...)
 * are installed on this board's image.
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Exports the GPIO (if not already exported) and configures it as an
 * output, initially driven low (off). Call once at startup, before the
 * first gpio_toggle_set() call.
 *
 * Returns 0 on success, -1 on failure (e.g. sysfs GPIO interface not
 * present, or permission denied - a message is printed to stderr). Failure
 * here is intentionally non-fatal to the caller's own control flow - the
 * timing marker is a diagnostic aid, not something that should be able to
 * take down the real controller-to-ICU pipeline if the board's GPIO setup
 * is ever different than expected.
 */
int gpio_toggle_init(void);

/*
 * Drives the GPIO high (state != 0) or low (state == 0). No-op if
 * gpio_toggle_init() failed or was never called.
 */
void gpio_toggle_set(int state);

#ifdef __cplusplus
}
#endif

#endif /* GPIO_TOGGLE_H */
