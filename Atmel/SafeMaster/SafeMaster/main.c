
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <string.h>
#include <util/atomic.h>

#include "lcd.h"

/* Master pin map */
#define GREEN_LED_PIN PD4
#define RED_LED_PIN   PD3
#define BUZZER_PIN    PD7
#define WAKE_BTN_PIN  PD2

/* SPI EEPROM pins */
#define SPI_DDR  DDRB
#define SPI_PORT PORTB
#define SPI_SS   PB4
#define SPI_MOSI PB5
#define SPI_MISO PB6
#define SPI_SCK  PB7

/* Servo timing (adjust to your hardware) */
#define SERVO_LOCKED_US   1000
#define SERVO_UNLOCKED_US 2000

/* State machine timings */
#define DOOR_OPEN_MS       5000UL
#define LOCKOUT_MS         30000UL
#define SLEEP_TIMEOUT_MS   10000UL
#define TAMPER_POLL_MS     100UL
#define TAMPER_THRESHOLD   40      /* ADC counts */
#define MAX_FAILS          3
#define ALERT_MS           5000UL

/* Internal EEPROM addresses for persistent data */
#define EE_ADDR_MAGIC      0
#define EE_ADDR_PASSWORD   1     /* 4 bytes */
#define EE_ADDR_FAILCOUNT  5
#define EE_ADDR_LOG_WRIDX  6     /* 2 bytes */
#define EE_ADDR_LOG_COUNT  8     /* 2 bytes */

/* Log event types */
typedef enum {
    EVT_LOGIN_SUCCESS = 1,
    EVT_LOGIN_FAIL,
    EVT_LOCKOUT,
    EVT_TAMPER,
    EVT_POWER_FAIL,
    EVT_POWER_RESTORED,
    EVT_WDT_RESET,
    EVT_PIN_CHANGED
} log_event_t;

/* --------------------------- TIMER (inline) -------------------------- */
static volatile uint32_t ms_ticks = 0;

void timer0_init(void)
{
    TCCR0 = (1 << WGM01);          /* CTC */
    OCR0  = 124;                   /* 1 ms @ 8MHz, prescaler 64 */
    TIMSK |= (1 << OCIE0);
    TCCR0 |= (1 << CS01) | (1 << CS00); /* start */
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
    UBRRL = 51;                    /* 9600 @ 8MHz */
    UCSRB = (1 << TXEN) | (1 << RXEN);
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

uint8_t usart_available(void)
{
    return (UCSRA & (1 << RXC)) ? 1 : 0;
}

char usart_read(void)
{
    while (!usart_available());
    return UDR;
}

/* --------------------------- KEYPAD (inline) ------------------------- */
#define ROW0 PA3   
#define ROW1 PA2  
#define ROW2 PA1   
#define ROW3 PA4  
#define COL0 PA5   
#define COL1 PA6   
#define COL2 PA7   

static const char keymap[4][3] = {
    {'1','2','3'},
    {'4','5','6'},
    {'7','8','9'},
    {'*','0','#'}
};
static const uint8_t rowpins[4] = { ROW0, ROW1, ROW2, ROW3 };
static const uint8_t colpins[3] = { COL0, COL1, COL2 };

void keypad_init(void)
{
    DDRA |= (1<<ROW0)|(1<<ROW1)|(1<<ROW2)|(1<<ROW3);
    DDRA &= ~((1<<COL0)|(1<<COL1)|(1<<COL2));
    PORTA |= (1<<ROW0)|(1<<ROW1)|(1<<ROW2)|(1<<ROW3);
    PORTA |= (1<<COL0)|(1<<COL1)|(1<<COL2);
}

char keypad_scan_raw(void)
{
    for (uint8_t r=0; r<4; r++) {
        PORTA |= (1<<ROW0)|(1<<ROW1)|(1<<ROW2)|(1<<ROW3);
        PORTA &= ~(1 << rowpins[r]);
        for (uint8_t c=0; c<3; c++) {
            if (!(PINA & (1 << colpins[c]))) {
                PORTA |= (1<<ROW0)|(1<<ROW1)|(1<<ROW2)|(1<<ROW3);
                return keymap[r][c];
            }
        }
    }
    PORTA |= (1<<ROW0)|(1<<ROW1)|(1<<ROW2)|(1<<ROW3);
    return 0;
}

char keypad_get_key(void)
{
    static char last_key = 0;
    static uint32_t last_change = 0;
    char k = keypad_scan_raw();
    uint32_t now = millis();
    if (k != last_key) {
        if (now - last_change > 30) {   /* debounce 30ms */
            last_change = now;
            last_key = k;
            if (k != 0) return k;
        }
    }
    return 0;
}

/* ---------------------------- ADC (inline) --------------------------- */
void adc_init(void)
{
    ADMUX = (1 << REFS0);          /* AVCC reference */
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); /* prescaler 64 */
}

uint16_t adc_read(uint8_t channel)
{
    ADMUX = (ADMUX & 0xE0) | (channel & 0x1F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

/* --------------------------- SERVO (inline) -------------------------- */
void servo_init(void)
{
    DDRD |= (1 << PD5);            /* OC1A */
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1 = 19999;                  /* 20ms period */
    OCR1A = SERVO_LOCKED_US;
}

void servo_set_us(uint16_t pulse_us)
{
    OCR1A = pulse_us;
}

/* ------------------------ COMPARATOR (inline) ------------------------ */
volatile uint8_t power_event_flag = 0;

void comparator_init(void)
{
    DDRB &= ~(1 << PB3);           /* AIN1 input */
    PORTB &= ~(1 << PB3);
    ACSR = (1 << ACBG) | (1 << ACIE); /* bandgap + interrupt on toggle */
}

ISR(ANA_COMP_vect)
{
    if (ACSR & (1 << ACO))
        power_event_flag = 1;      /* power fail */
    else
        power_event_flag = 2;      /* power restored */
}

/* -------------------------- SPI (inline) ----------------------------- */
void spi_init(void)
{
    SPI_DDR |= (1 << SPI_MOSI) | (1 << SPI_SCK) | (1 << SPI_SS);
    SPI_DDR &= ~(1 << SPI_MISO);
    SPI_PORT |= (1 << SPI_SS);
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

uint8_t spi_transfer(uint8_t data)
{
    SPDR = data;
    while (!(SPSR & (1 << SPIF)));
    return SPDR;
}

/* --------------------- EXTERNAL EEPROM (inline) ---------------------- */
#define EE_WREN  0x06
#define EE_WRDI  0x04
#define EE_RDSR  0x05
#define EE_WRSR  0x01
#define EE_READ  0x03
#define EE_WRITE 0x02

static inline void ee_cs_low(void)  { SPI_PORT &= ~(1 << SPI_SS); }
static inline void ee_cs_high(void) { SPI_PORT |= (1 << SPI_SS); }

static uint8_t ee_opcode(uint8_t base, uint16_t addr)
{
    return base | (uint8_t)((addr & 0x100) >> 5);
}

static void ee_wait_ready(void)
{
    uint8_t sr;
    ee_cs_low();
    spi_transfer(EE_RDSR);
    do {
        sr = spi_transfer(0xFF);
    } while (sr & 0x01);   /* WIP bit */
    ee_cs_high();
}

void eeprom_ext_write_byte(uint16_t addr, uint8_t data)
{
    ee_cs_low();
    spi_transfer(EE_WREN);
    ee_cs_high();

    ee_cs_low();
    spi_transfer(ee_opcode(EE_WRITE, addr));
    spi_transfer((uint8_t)(addr & 0xFF));
    spi_transfer(data);
    ee_cs_high();

    ee_wait_ready();
}

uint8_t eeprom_ext_read_byte(uint16_t addr)
{
    uint8_t data;
    ee_cs_low();
    spi_transfer(ee_opcode(EE_READ, addr));
    spi_transfer((uint8_t)(addr & 0xFF));
    data = spi_transfer(0xFF);
    ee_cs_high();
    return data;
}

void eeprom_ext_write_bytes(uint16_t addr, const uint8_t *buf, uint8_t len)
{
    for (uint8_t i=0; i<len; i++)
        eeprom_ext_write_byte(addr + i, buf[i]);
}

void eeprom_ext_read_bytes(uint16_t addr, uint8_t *buf, uint8_t len)
{
    for (uint8_t i=0; i<len; i++)
        buf[i] = eeprom_ext_read_byte(addr + i);
}

/* --------------------------- LOG (inline) ---------------------------- */
#define MAGIC_VALUE       0xA5
#define LOG_RECORD_SIZE   4
#define LOG_MAX_RECORDS   128

static void print_uint(uint16_t val)
{
    char buf[6];
    uint8_t i = 0;
    if (val == 0) {
        usart_transmit('0');
        return;
    }
    while (val > 0 && i < 5) {
        buf[i++] = (char)('0' + (val % 10));
        val /= 10;
    }
    while (i > 0)
        usart_transmit(buf[--i]);
}

static const char *event_name(uint8_t e)
{
    switch (e) {
        case EVT_LOGIN_SUCCESS:   return "LOGIN_OK";
        case EVT_LOGIN_FAIL:      return "LOGIN_FAIL";
        case EVT_LOCKOUT:         return "LOCKOUT";
        case EVT_TAMPER:          return "TAMPER";
        case EVT_POWER_FAIL:      return "POWER_FAIL";
        case EVT_POWER_RESTORED:  return "POWER_RESTORED";
        case EVT_WDT_RESET:       return "WATCHDOG_RESET";
        case EVT_PIN_CHANGED:     return "PIN_CHANGED";
        default:                  return "UNKNOWN";
    }
}

void log_init(void)
{
    uint8_t magic = eeprom_read_byte((const uint8_t *)EE_ADDR_MAGIC);
    if (magic != MAGIC_VALUE) {
        const uint8_t default_pw[4] = {1,2,3,4};
        eeprom_update_byte((uint8_t *)EE_ADDR_MAGIC, MAGIC_VALUE);
        eeprom_update_block(default_pw, (void *)EE_ADDR_PASSWORD, 4);
        eeprom_update_byte((uint8_t *)EE_ADDR_FAILCOUNT, 0);
        eeprom_update_word((uint16_t *)EE_ADDR_LOG_WRIDX, 0);
        eeprom_update_word((uint16_t *)EE_ADDR_LOG_COUNT, 0);
    }
}

void log_event(uint8_t event, uint8_t extra)
{
    uint16_t wridx = eeprom_read_word((const uint16_t *)EE_ADDR_LOG_WRIDX);
    uint16_t count = eeprom_read_word((const uint16_t *)EE_ADDR_LOG_COUNT);

    uint32_t t = millis() / 1000UL;
    uint8_t rec[LOG_RECORD_SIZE];
    rec[0] = event;
    rec[1] = (uint8_t)(t & 0xFF);
    rec[2] = (uint8_t)((t >> 8) & 0xFF);
    rec[3] = extra;

    eeprom_ext_write_bytes((uint16_t)(wridx * LOG_RECORD_SIZE), rec, LOG_RECORD_SIZE);

    wridx = (uint16_t)((wridx + 1) % LOG_MAX_RECORDS);
    if (count < LOG_MAX_RECORDS) count++;

    eeprom_update_word((uint16_t *)EE_ADDR_LOG_WRIDX, wridx);
    eeprom_update_word((uint16_t *)EE_ADDR_LOG_COUNT, count);
}

void log_dump_all(void)
{
    uint16_t count = eeprom_read_word((const uint16_t *)EE_ADDR_LOG_COUNT);
    uint16_t wridx = eeprom_read_word((const uint16_t *)EE_ADDR_LOG_WRIDX);
    uint16_t start = (count < LOG_MAX_RECORDS) ? 0 : wridx;

    usart_print("\r\n--- LOG DUMP (");
    print_uint(count);
    usart_print(" records) ---\r\n");

    for (uint16_t i=0; i<count; i++) {
        uint16_t idx = (uint16_t)((start + i) % LOG_MAX_RECORDS);
        uint8_t rec[LOG_RECORD_SIZE];
        eeprom_ext_read_bytes((uint16_t)(idx * LOG_RECORD_SIZE), rec, LOG_RECORD_SIZE);
        uint16_t t = (uint16_t)(rec[1] | (rec[2] << 8));

        usart_print("#");
        print_uint(i+1);
        usart_print(" t=");
        print_uint(t);
        usart_print("s ");
        usart_print(event_name(rec[0]));
        usart_print(" extra=");
        print_uint(rec[3]);
        usart_print("\r\n");
    }
    usart_print("--- END OF LOG ---\r\n");
}

/* --------------------- LED / BUZZER helpers -------------------------- */
static void leds_init(void)
{
    DDRD |= (1 << GREEN_LED_PIN) | (1 << RED_LED_PIN);
}
static void green_led(uint8_t on)
{
    if (on) PORTD |= (1 << GREEN_LED_PIN);
    else    PORTD &= ~(1 << GREEN_LED_PIN);
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

/* ---------------- PASSWORD & FAIL COUNT helpers ---------------------- */
static void get_password(uint8_t pw[4])
{
    eeprom_read_block(pw, (const void *)EE_ADDR_PASSWORD, 4);
}
static void set_password(const uint8_t pw[4])
{
    eeprom_update_block(pw, (void *)EE_ADDR_PASSWORD, 4);
}
static uint8_t get_fail_count(void)
{
    return eeprom_read_byte((const uint8_t *)EE_ADDR_FAILCOUNT);
}
static void set_fail_count(uint8_t v)
{
    eeprom_update_byte((uint8_t *)EE_ADDR_FAILCOUNT, v);
}

/* ------------------- WAKE BUTTON (INT0) ISR -------------------------- */
ISR(INT0_vect)
{
    /* empty - just wakes the CPU from sleep */
}

static void enter_sleep(void)
{
    LCD_cmd(0x08);                    /* display off */
    ADCSRA &= ~(1 << ADEN);           /* ADC off */

    MCUCR &= ~((1 << ISC01) | (1 << ISC00)); /* low level trigger */
    GICR |= (1 << INT0);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sei();
    sleep_cpu();                      /* halt until button pressed */
    sleep_disable();

    GICR &= ~(1 << INT0);
    MCUCR |= (1 << ISC01);            /* back to falling edge */
    MCUCR &= ~(1 << ISC00);

    ADCSRA |= (1 << ADEN);
    LCD_cmd(0x0C);                    /* display on */
}

/* ---------- Helper: collect 4-digit PIN with '*' clear --------------- */
static uint8_t collect_pin(uint8_t *buf, uint8_t *count)
{
	char k = keypad_get_key();
	if (!k) return 0;

	if (k == '*') {
		*count = 0;
		LCD_goto(1, 0);
		LCD_write_string("                ");
		return 0;
	}
	if (k >= '0' && k <= '9' && *count < 4) {
		buf[(*count)++] = (uint8_t)(k - '0');
		LCD_goto(1, *count - 1);  // ? ADD THIS LINE!
		LCD_write('*');
	}
	return (*count == 4) ? 1 : 0;
}

/* ============================== MAIN ================================= */
int main(void)
{
    uint8_t reset_was_wdt = (MCUCSR & (1 << WDRF)) ? 1 : 0;
    MCUCSR &= ~(1 << WDRF);
    wdt_disable();

    /* Disable JTAG just in case you are using PORTC for LCD */
    MCUCSR |= (1 << JTD);
    MCUCSR |= (1 << JTD);

    timer0_init();
    
    /* INTERRUPTS MUST BE ENABLED HERE FOR delay_ms() TO WORK! */
    sei();

    usart_init();
    leds_init();
    buzzer_init();
    init_LCD();
    keypad_init();
    adc_init();
    servo_init();
    spi_init();
    comparator_init();
    log_init();

    DDRD &= ~(1 << WAKE_BTN_PIN);
    PORTD |= (1 << WAKE_BTN_PIN);          /* pull-up */
    MCUCR |= (1 << ISC01);                 /* falling edge for normal */
    MCUCR &= ~(1 << ISC00);

    wdt_enable(WDTO_2S);

    if (reset_was_wdt) {
        usart_print("ALERT: WATCHDOG RESET RECOVERY\r\n");
        log_event(EVT_WDT_RESET, 0);
    }

    LCD_clear();
    LCD_write_string("Smart Safe v1.0");
    delay_ms(800);
    LCD_clear();
    LCD_write_string("Enter PIN:");
    LCD_goto(1, 0);

    typedef enum {
        ST_ENTRY = 0,
        ST_UNLOCKED,
        ST_LOCKOUT,
        ST_ADMIN_VERIFY,
        ST_ADMIN_NEW,
        ST_ADMIN_CONFIRM
    } master_state_t;

    master_state_t state = ST_ENTRY;
    uint8_t entry_pin[4], entry_n = 0;
    uint8_t admin_new_pin[4], admin_new_n = 0;
    uint8_t admin_confirm_pin[4], admin_confirm_n = 0;

    uint32_t state_until = 0;
    uint32_t last_activity = millis();
    uint32_t last_tamper_poll = 0;
    uint16_t tamper_prev = 0;
    uint8_t tamper_have_prev = 0;
    uint32_t tamper_alarm_until = 0;
    uint8_t tamper_alarm_active = 0;
    uint32_t pound_press_start = 0;

    while (1) {
        wdt_reset();
        uint32_t now = millis();

        /* ---- Power-fail / restore ---- */
        if (power_event_flag) {
            uint8_t ev;
            cli(); ev = power_event_flag; power_event_flag = 0; sei();
            if (ev == 1) {
                usart_print("ALERT: POWER FAILURE DETECTED\r\n");
                log_event(EVT_POWER_FAIL, 0);
                red_led(1);
            } else if (ev == 2) {
                usart_print("POWER RESTORED\r\n");
                log_event(EVT_POWER_RESTORED, 0);
                if (state != ST_LOCKOUT && !tamper_alarm_active) red_led(0);
            }
            last_activity = now;
        }

        /* ---- Terminal command: 'L' dumps log ---- */
        if (usart_available()) {
            char c = usart_read();
            if (c == 'L' || c == 'l') {
                log_dump_all();
            }
            last_activity = now;
        }

        /* ---- Tamper polling (every 100ms) ---- */
        if (now - last_tamper_poll >= TAMPER_POLL_MS) {
	        last_tamper_poll = now;
	        uint16_t v = adc_read(0);
	        if (tamper_have_prev) {
		        int16_t diff = (int16_t)v - (int16_t)tamper_prev;
		        if (diff < 0) diff = (int16_t)(-diff);
		        if (diff > TAMPER_THRESHOLD) {
			        usart_print("ALERT: TAMPER DETECTED\r\n");
			        log_event(EVT_TAMPER, (uint8_t)(v >> 2));
			        red_led(1);
			        buzzer_on();
			        tamper_alarm_active = 1;
			        tamper_alarm_until = now + ALERT_MS;
			        last_activity = now;
		        }
	        }
	        tamper_prev = v;
	        tamper_have_prev = 1;
        }

        /* Clear tamper alarm after 5 seconds */
        if (tamper_alarm_active && now >= tamper_alarm_until) {
	        tamper_alarm_active = 0;
	        if (state != ST_LOCKOUT) red_led(0);
	        buzzer_off();
	        
	        /* RESTORE LCD */
	        if (state == ST_ENTRY) {
		        LCD_clear();
		        LCD_write_string("Enter PIN:");
		        LCD_goto(1, 0);
		        // Clear the second line
		        LCD_write_string("                ");
				LCD_goto(1, 0);
	        }
        }

        /* ---- Long press '#' at idle -> admin PIN change ---- */
        if (state == ST_ENTRY && entry_n == 0) {
            char raw = keypad_scan_raw();
            if (raw == '#') {
                if (pound_press_start == 0) pound_press_start = now;
                else if (now - pound_press_start > 3000UL) {
                    pound_press_start = 0;
                    state = ST_ADMIN_VERIFY;
                    entry_n = 0;
                    LCD_clear();
                    LCD_write_string("Admin: cur. PIN");
                    LCD_goto(1, 0);
                    last_activity = now;
                }
            } else {
                pound_press_start = 0;
            }
        }

        /* ---- Main state machine ---- */
        switch (state) {

        case ST_ENTRY:
			if (tamper_alarm_active) {
				LCD_goto(0, 0);
				LCD_write_string("TAMPER ALARM!");
				LCD_goto(1, 0);
				LCD_write_string("Wait 5s");
				break;  // Exit switch case - don't process PIN
			}
            if (collect_pin(entry_pin, &entry_n)) {
                last_activity = now;
                uint8_t pw[4];
                get_password(pw);
                if (memcmp(pw, entry_pin, 4) == 0) {
                    set_fail_count(0);
                    log_event(EVT_LOGIN_SUCCESS, 0);
                    usart_print("Access granted\r\n");
                    green_led(1);
                    servo_set_us(SERVO_UNLOCKED_US);
                    LCD_clear();
                    LCD_write_string("Access Granted");
                    state = ST_UNLOCKED;
                    state_until = now + DOOR_OPEN_MS;
                } else {
                    uint8_t fails = (uint8_t)(get_fail_count() + 1);
                    set_fail_count(fails);
                    log_event(EVT_LOGIN_FAIL, fails);
                    usart_print("Access denied\r\n");
                    LCD_clear();
                    if (fails >= MAX_FAILS) {
                        usart_print("ALERT: BREACH - 3 FAILED ATTEMPTS\r\n");
                        log_event(EVT_LOCKOUT, fails);
                        red_led(1);
                        buzzer_on();
                        LCD_write_string("LOCKED 30s");
                        state = ST_LOCKOUT;
                        state_until = now + LOCKOUT_MS;
                    } else {
                        LCD_write_string("Wrong PIN");
                        delay_ms(600);
                        LCD_clear();
                        LCD_write_string("Enter PIN:");
                        LCD_goto(1, 0);
                    }
                }
                entry_n = 0;
            }
            break;

        case ST_UNLOCKED:
            if (now >= state_until) {
                servo_set_us(SERVO_LOCKED_US);
                green_led(0);
                LCD_clear();
                LCD_write_string("Enter PIN:");
                LCD_goto(1, 0);
                entry_n = 0;
                state = ST_ENTRY;
                last_activity = now;
            }
            break;

        case ST_LOCKOUT:
            if (now >= state_until) {
                buzzer_off();
                if (!tamper_alarm_active) red_led(0);
                set_fail_count(0);
                LCD_clear();
                LCD_write_string("Enter PIN:");
                LCD_goto(1, 0);
                entry_n = 0;
                state = ST_ENTRY;
                last_activity = now;
            }
            break;

        case ST_ADMIN_VERIFY:
            if (collect_pin(entry_pin, &entry_n)) {
                uint8_t pw[4];
                get_password(pw);
                entry_n = 0;
                if (memcmp(pw, entry_pin, 4) == 0) {
                    admin_new_n = 0;
                    LCD_clear();
                    LCD_write_string("New PIN:");
                    LCD_goto(1, 0);
                    state = ST_ADMIN_NEW;
                } else {
                    LCD_clear();
                    LCD_write_string("Denied");
                    delay_ms(800);
                    LCD_clear();
                    LCD_write_string("Enter PIN:");
                    LCD_goto(1, 0);
                    state = ST_ENTRY;
                }
                last_activity = now;
            }
            break;

        case ST_ADMIN_NEW:
            if (collect_pin(admin_new_pin, &admin_new_n)) {
                admin_new_n = 0;
                admin_confirm_n = 0;
                LCD_clear();
                LCD_write_string("Confirm PIN:");
                LCD_goto(1, 0);
                state = ST_ADMIN_CONFIRM;
                last_activity = now;
            }
            break;

        case ST_ADMIN_CONFIRM:
            if (collect_pin(admin_confirm_pin, &admin_confirm_n)) {
                admin_confirm_n = 0;
                LCD_clear();
                if (memcmp(admin_new_pin, admin_confirm_pin, 4) == 0) {
                    set_password(admin_new_pin);
                    log_event(EVT_PIN_CHANGED, 0);
                    usart_print("PIN changed\r\n");
                    LCD_write_string("PIN Updated");
                } else {
                    LCD_write_string("Mismatch");
                    usart_print("PIN change aborted (mismatch)\r\n");
                }
                delay_ms(1000);
                LCD_clear();
                LCD_write_string("Enter PIN:");
                LCD_goto(1, 0);
                entry_n = 0;
                state = ST_ENTRY;
                last_activity = now;
            }
            break;
        }

        /* ---- Sleep after inactivity ---- */
        if (state == ST_ENTRY && entry_n == 0 &&
            (now - last_activity) >= SLEEP_TIMEOUT_MS) {
            enter_sleep();
            last_activity = millis();
            LCD_clear();
            LCD_write_string("Enter PIN:");
            LCD_goto(1, 0);
        }
    }
}