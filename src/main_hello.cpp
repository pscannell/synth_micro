//
// Minimal OLED smoke-test firmware for bring-up.
//

extern "C" {
#include "BlinkLed.h"
#include "Timer.h"
#include "ssd1306.h"
}

int main(int argc, char* argv[]) {
	(void)argc;
	(void)argv;

	blink_led_init();
	blink_led_off();
	timer_start();

	ssd1306_init(0);

	char hello[] = "Hello";
	char world[] = "World";
	println_16(hello, 5, 18, 14);
	println_16(world, 5, 26, 36);
	ssd1306_refresh();

	while (1) {
		AUX_LED_GREEN_ON;
	}
}
