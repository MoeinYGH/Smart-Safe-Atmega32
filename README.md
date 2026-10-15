# Smart Safe with Remote Alert

A dual-ATmega32 security system: a **Master** safe unit that handles PIN authentication, door locking, tamper detection, power-fail monitoring, event logging and low-power sleep, and a **Slave** remote alarm that receives status messages over USART and drives a local LCD, LED and buzzer.

Every peripheral (timers, USART, keypad, ADC, PWM, analog comparator, SPI) is configured directly through AVR registers. No delay library is used anywhere, including `_delay_ms()`. All timing comes from a single Timer0 millisecond tick.

## Features

### Master (safe unit)

- **PIN authentication**: 4-digit PIN stored in internal EEPROM (default `1234`), entered on a keypad with each digit echoed as `*` on the LCD.
- **Door lock**: a correct PIN drives a servo to the unlocked position for 5 seconds with the green LED on, then relocks.
- **3-strikes lockout**: the failed-attempt counter is persistent (EEPROM). Three wrong PINs trigger a 30-second lockout with the red LED and buzzer on, plus a `BREACH` alert sent to the Slave.
- **Tamper detection**: a potentiometer simulates a shock sensor, sampled by the ADC every 100 ms. A jump of more than 40 counts raises a tamper alarm.
- **Power-fail detection**: the analog comparator compares a divided supply voltage against the internal 1.22 V bandgap and reports both power loss and restoration.
- **Event log**: successful and failed logins, lockouts, tamper events, power fail/restore, watchdog resets and PIN changes are stored as timestamped records in an external SPI EEPROM (25LC040) and survive resets.
- **Log dump**: sending `L` over USART prints every stored record to a terminal.
- **Power management**: after 10 seconds of inactivity the LCD and ADC are turned off and the MCU enters power-down sleep. A push button on INT0 wakes it.
- **Watchdog**: 2-second watchdog with recovery. A watchdog-caused reset is detected and reported over USART.
- **Admin PIN change**: holding `#` for more than 3 seconds at the idle prompt starts a flow that verifies the current PIN, then asks for and confirms a new one.

### Slave (remote alarm)

- Interrupt-driven USART reception (`RXCIE`) with a line buffer, so no message is dropped.
- Shows the latest message on its LCD, wrapping across both lines.
- If the message contains `BREACH`, `TAMPER` (or `Tamper`) or `FAILURE`, the red LED and buzzer run for 5 seconds.
- A message containing `RESTORED` silences any active alarm.
- 2-second watchdog.

## System Architecture

```
 +------------------------------+   USART, 9600 baud, 8N1   +-----------------------+
 |       MASTER (safe)          | ------------------------> |    SLAVE (alarm)      |
 |  ATmega32 @ 8 MHz            |                           |  ATmega32 @ 8 MHz     |
 |                              |                           |                       |
 |  Keypad    LCD    Servo      |                           |  LCD   Red LED        |
 |  Tamper (Pot + ADC)          |                           |  Buzzer               |
 |  Power-fail (Comparator)     |                           +-----------------------+
 |  SPI EEPROM (event log)      |
 +------------------------------+
```

### Message protocol

The Master sends short human-readable ASCII lines, for example:

| Message | Trigger |
|---|---|
| `ALERT: BREACH - 3 FAILED ATTEMPTS` | 3 consecutive wrong PINs |
| `ALERT: TAMPER DETECTED` | Tamper event |
| `ALERT: POWER FAILURE DETECTED` | Comparator detects supply loss |
| `POWER RESTORED` | Comparator detects supply recovery |
| `ALERT: WATCHDOG RESET RECOVERY` | Reset caused by the watchdog |

The Slave matches keywords with `strstr`. Because the messages are plain text, the same output works on a PC terminal for debugging.

## Hardware (Proteus)

| Component | Proteus part | Role |
|---|---|---|
| 2 x ATmega32 | `ATMEGA32` | Master and Slave MCUs |
| 16x2 LCD (x2) | `LM016L` | Status display on each unit |
| Calculator keypad | `SMALLCALC-KEYPAD` | PIN entry (4x3 matrix scan: digits, `*`, `#`) |
| Servo | `PWMSERVO-MOTOR` | Door bolt |
| Potentiometer | `HG-POT` | Shock/tamper sensor |
| 10 kOhm resistors | `K10MINRES` | Supply-monitor voltage divider |
| SPI EEPROM | `25LC040` | Event log |
| LEDs | `GREEN-LED`, `RED-LED` | Status and alarm indicators |
| Speaker | `SPEAKER` | Alarm buzzer |
| Push button | `BUTTON` | Wake from sleep (INT0) |

### Master pin usage

| Function | Pin(s) |
|---|---|
| Keypad rows (outputs) | PA1-PA4 |
| Keypad columns (inputs, pull-ups) | PA5-PA7 |
| Tamper ADC input | ADC0 |
| Servo PWM (OC1A) | PD5 |
| Wake button (INT0) | PD2 |
| Comparator negative input (AIN1) | PB3 |
| SPI to EEPROM | MOSI, MISO, SCK and SS on PB4-PB7 |
| USART TX to Slave | PD1 (TXD) |

## Register Configuration and Design Rationale

### Clock

Both MCUs run at **8 MHz** on the default internal RC oscillator.

### Timer0: 1 ms tick (CTC mode)

```c
TCCR0 = (1 << WGM01);                 // CTC, TOP = OCR0
OCR0  = 124;
TIMSK |= (1 << OCIE0);                // compare-match interrupt
TCCR0 |= (1 << CS01) | (1 << CS00);   // prescaler 64
```

```
OCR0 = (1 ms * 8,000,000) / 64 - 1 = 124
```

Prescaler 64 gives an exact integer OCR0 for a 1 ms period at 8 MHz, so the tick has no drift. Prescaler 8 would need OCR0 = 999, which does not fit in the 8-bit register. The tick drives `millis()` and a timer-based `delay_ms()`. This is how the 5 s door timer, 30 s lockout, 10 s sleep timeout and 30 ms debounce are built.

### USART: 9600 baud, 8N1

```c
UBRRH = 0;
UBRRL = 51;
UCSRB = (1 << TXEN) | (1 << RXEN);               // Slave also sets RXCIE
UCSRC = (1 << UCSZ1) | (1 << UCSZ0);             // 8 data bits, 1 stop, no parity
```

```
UBRR   = 8,000,000 / (16 * 9600) - 1 = 51.08 -> 51
Actual = 8,000,000 / (16 * 52) = 9615.4 bps
Error  = |9615.4 - 9600| / 9600 = 0.16 %
```

The error is well inside the +/-2% UART tolerance, so double-speed mode is not needed.

### Timer1: servo PWM (Fast PWM, mode 14)

```c
DDRD  |= (1 << PD5);
TCCR1A = (1 << COM1A1) | (1 << WGM11);
TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);   // prescaler 8
ICR1   = 19999;                                       // 20 ms period
OCR1A  = SERVO_LOCKED_US;
```

With prescaler 8 at 8 MHz, one timer tick is 1 us, so `ICR1 = 19999` gives a 20 ms (50 Hz) period. Pulse widths are set directly in microseconds: 1000 us for locked and 2000 us for unlocked.

### ADC: tamper sensor

```c
ADMUX  = (1 << REFS0);                                // AVCC reference, channel 0
ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);   // prescaler 64
```

```
f_ADC = 8,000,000 / 64 = 125 kHz   (valid range: 50-200 kHz)
```

The sensor is sampled every 100 ms. A change of more than 40 counts (about 4% of 1024) from the previous sample is a tamper event. That is well above normal ADC noise and still sensitive to a sudden disturbance.

### Analog comparator: power-fail detection

```c
DDRB  &= ~(1 << PB3);                 // AIN1 input
PORTB &= ~(1 << PB3);                 // no pull-up
ACSR   = (1 << ACBG) | (1 << ACIE);   // bandgap on AIN0, interrupt on toggle
```

`ACBG` routes the internal ~1.22 V bandgap to AIN0, so only one external pin (AIN1) and one resistor divider are needed. The interrupt fires on both edges, so failure and restoration are each reported immediately without polling.

### SPI: external EEPROM

```c
SPI_DDR  |= (1 << SPI_MOSI) | (1 << SPI_SCK) | (1 << SPI_SS);
SPI_DDR  &= ~(1 << SPI_MISO);
SPI_PORT |= (1 << SPI_SS);                            // idle high
SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);        // master, fosc/16
```

The SPI clock is 8 MHz / 16 = 500 kHz, a conservative rate that is safe for the 25LC040.

**Log record format** (4 bytes): event code (1 byte), 16-bit timestamp in seconds (2 bytes), extra data (1 byte, for example the fail count or a scaled ADC delta). Records are written to a circular buffer of 128 slots. The write index and record count are kept in internal EEPROM.

### Power-down sleep and wake

After 10 s idle, the Master turns off the LCD and ADC and enters `SLEEP_MODE_PWR_DOWN`. INT0 is switched to **low-level** triggering before sleep, because power-down mode only wakes on level interrupts and a few other sources. After waking, INT0 returns to falling-edge triggering and the LCD and ADC are re-enabled.

### Watchdog

`wdt_enable(WDTO_2S)` sets a 2 s timeout, longer than any main-loop pass because nothing blocks. `wdt_reset()` runs at the top of every loop iteration. On boot, `MCUCSR.WDRF` is checked before it is cleared, so a watchdog reset can be told apart from a power-on reset and reported.

### Summary

| Peripheral | Setting | Key registers | Reason |
|---|---|---|---|
| System clock | 8 MHz internal RC | n/a | Default clock, under 0.2% baud error |
| Timer0 | 1 ms, prescaler 64 | `TCCR0`, `OCR0=124`, `TIMSK` | Exact integer OCR0 |
| USART | 9600 baud, 8N1 | `UBRRL=51`, `UCSRB`, `UCSRC` | About 0.16% error |
| Timer1 | 20 ms, prescaler 8 | `TCCR1A/B`, `ICR1=19999` | 1 us per tick |
| ADC | AVCC, prescaler 64 | `ADMUX`, `ADCSRA` | 125 kHz ADC clock |
| Comparator | Bandgap reference | `ACSR` | One external pin to monitor supply |
| SPI | Master, fosc/16 | `SPCR` | 500 kHz, safe margin |
| Watchdog | 2 s | `wdt_enable(WDTO_2S)` | Fast recovery without false resets |

## Master State Machine

| State | Behavior |
|---|---|
| `ST_ENTRY` | Collects 4 digits, echoing `*`. `*` clears the buffer. |
| `ST_UNLOCKED` | Servo unlocked and green LED on for 5 s. |
| `ST_LOCKOUT` | 30 s lockout, with red LED and buzzer on. |
| `ST_ADMIN_VERIFY` | Verifies the current PIN. |
| `ST_ADMIN_NEW` | Reads the new PIN. |
| `ST_ADMIN_CONFIRM` | Confirms the new PIN and saves it to EEPROM. |

Every transition is driven by comparing `millis()` against a per-state deadline, so tamper polling, power-fail handling and watchdog resets keep running during lockouts and door-open periods.

## Repository Structure

```
.
├── master/
│   └── main.c          # Safe unit firmware
├── slave/
│   └── main.c          # Remote alarm firmware
├── lcd.c               # HD44780 LCD driver (instructor-provided)
├── lcd.h
├── LICENSE
└── README.md
```

Both `main.c` files supply their own `delay_ms()`, built on their own Timer0 tick. `lcd.c` calls it through an `extern` declaration, so the LCD driver stays identical on both boards.

## Building

Requires `avr-gcc` and `avr-libc`.

```bash
# Master
avr-gcc -mmcu=atmega32 -DF_CPU=8000000UL -Os -Wall \
        -o master.elf master/main.c lcd.c
avr-objcopy -O ihex master.elf master.hex

# Slave
avr-gcc -mmcu=atmega32 -DF_CPU=8000000UL -Os -Wall \
        -o slave.elf slave/main.c lcd.c
avr-objcopy -O ihex slave.hex
```

## Running the Simulation

1. Open the schematic in Proteus and load `master.hex` into the Master ATmega32 and `slave.hex` into the Slave.
2. Set both ATmega32 clock frequencies to 8 MHz. Connect Master TXD to Slave RXD.
3. Start the simulation.

### Things to try

| Action | Expected result |
|---|---|
| Enter `1234` | Green LED on, servo unlocks for 5 s, then relocks |
| Enter a wrong PIN 3 times | 30 s lockout with red LED and buzzer. The Slave shows `BREACH` and alarms for 5 s |
| Move the potentiometer suddenly | Tamper alarm on both units |
| Drop the supply-monitor voltage | Power-failure message on the Slave. Restoring it sends `POWER RESTORED` and silences the alarm |
| Send `L` over the USART terminal | Full event log printed |
| Leave the safe idle for 10 s | LCD turns off and the MCU sleeps. The button wakes it |
| Hold `#` for more than 3 s | Admin PIN-change flow |

## Acknowledgements

The LCD driver (`lcd.c` / `lcd.h`) was provided by the course instructor and is not covered by this repository's license.

## License

The original code and documentation in this repository are released under the [MIT License](LICENSE).

## Author

**Moein Yeganeh**
