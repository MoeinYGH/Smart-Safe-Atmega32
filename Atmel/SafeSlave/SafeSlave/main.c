#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <string.h>
#include <util/atomic.h>

#include "lcd.h"

/* ------------------------------- CONFIG ------------------------------ */
#define RED_LED_PIN   PD3
#define BUZZER_PIN    PD7

#define ALERT_MS     5000UL
#define MSG_BUF_LEN  40

/* --------------------------- TIMER (inline) -------------------------- */
static volatile uint32_t ms_ticks = 0;

void timer0_init(void)
{
	TCCR0 = (1 << WGM01);
	OCR0  = 124;
	TIMSK |= (1 << OCIE0);
	TCCR0 |= (1 << CS01) | (1 << CS00);
}

ISR(TIMER0_COMP_vect)
{
	ms_ticks++;
}

uint32_t millis(void)
{
	uint32_t val;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		val = ms_ticks;
	}
	return val;
}

void delay_ms(uint16_t ms)
{
	uint32_t start = millis();
	while ((uint32_t)(millis() - start) < ms) {
		/* busy wait */
	}
}

/* --------------------------- USART (inline) -------------------------- */
void usart_init(void)
{
	UBRRH = 0;
	UBRRL = 51;
	UCSRB = (1 << TXEN) | (1 << RXEN) | (1 << RXCIE);
	UCSRC = (1 << UCSZ1) | (1 << UCSZ0);
}

void usart_transmit(char c)
{
	while (!(UCSRA & (1 << UDRE)));
	UDR = c;
}

void usart_print(const char *s)
{
	while (*s) usart_transmit(*s++);
}

/* -------------------- USART RX Interrupt (Slave) --------------------- */
static volatile char rx_buf[MSG_BUF_LEN];
static volatile uint8_t rx_len = 0;
static volatile uint8_t msg_ready = 0;

ISR(USART_RXC_vect)
{
	char c = UDR;
	if (c == '\n' || c == '\r') {
		if (rx_len > 0) {
			msg_ready = 1;
		}
		} else if (rx_len < MSG_BUF_LEN - 1) {
		rx_buf[rx_len++] = c;
	}
}

/* --------------------- LED / BUZZER helpers -------------------------- */
static void leds_init(void)
{
	DDRD |= (1 << RED_LED_PIN);
}
static void red_led(uint8_t on)
{
	if (on) PORTD |= (1 << RED_LED_PIN);
	else    PORTD &= ~(1 << RED_LED_PIN);
}

static void buzzer_init(void)
{
	DDRD |= (1 << BUZZER_PIN);
}
static void buzzer_on(void)
{
	TCCR2 = (1 << WGM21) | (1 << COM20) | (1 << CS21) | (1 << CS20);
	OCR2 = 120;
}
static void buzzer_off(void)
{
	TCCR2 = 0;
	PORTD &= ~(1 << BUZZER_PIN);
}

/* --------------------- String search helper -------------------------- */
static uint8_t contains(const char *haystack, const char *needle)
{
	return strstr(haystack, needle) != NULL;
}

/* ============================== MAIN ================================= */
int main(void)
{
	/* Disable JTAG to free PC2-PC5 for LCD */
	MCUCSR |= (1 << JTD);
	MCUCSR |= (1 << JTD);

	wdt_disable();

	timer0_init();
	sei();  // ? Enable interrupts BEFORE init_LCD()

	usart_init();
	leds_init();
	buzzer_init();
	init_LCD();

	wdt_enable(WDTO_2S);

	LCD_clear();
	LCD_write_string("Remote Alarm");
	LCD_goto(1, 0);
	LCD_write_string("Ready");

	uint8_t alarm_active = 0;
	uint32_t alarm_until = 0;

	while (1) {
		wdt_reset();  // ? CRITICAL: Pet the watchdog!

		uint32_t now = millis();

		if (msg_ready) {
			char local[MSG_BUF_LEN];
			uint8_t len;

			cli();
			len = rx_len;
			if (len > MSG_BUF_LEN - 1) len = MSG_BUF_LEN - 1;
			memcpy(local, (const char *)rx_buf, len);
			local[len] = '\0';
			rx_len = 0;
			msg_ready = 0;
			sei();

			LCD_clear();
			uint8_t total = (uint8_t)strlen(local);
			if (total <= 16) {
				LCD_write_string(local);
				} else {
				char line1[17];
				memcpy(line1, local, 16);
				line1[16] = '\0';
				LCD_write_string(line1);
				LCD_goto(1, 0);
				LCD_write_string(&local[16]);
			}

			if (contains(local, "RESTORED")) {
				alarm_active = 0;
				red_led(0);
				buzzer_off();
			} else if (contains(local, "BREACH") ||
			contains(local, "TAMPER") ||
			contains(local, "FAILURE")) {
				alarm_active = 1;
				alarm_until = now + ALERT_MS;
				red_led(1);
				buzzer_on();
			}
		}

		if (alarm_active && now >= alarm_until) {
			alarm_active = 0;
			red_led(0);
			buzzer_off();
		}
	}
}