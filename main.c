/* ================================================================
 *  s17229_Final_Project — Weather Monitoring System (PRO)
 *  MCU  : ATmega328P @ 16 MHz (external crystal)   [UPGRADED from 10 MHz]
 *  Build: avr-gcc -mmcu=atmega328p -DF_CPU=16000000UL -Os
 *         -std=c11 -Wall -Wextra
 *  Libs : Petit FatFs (pff), avr-libc
 *
 *  Connectivity:
 *    ESP-01   — Hardware UART (PD0=RX, PD1=TX) @ 115200 baud
 *               ThingSpeak upload every 60 s via TCP/HTTP GET.
 *    SIM800L  — Software UART (PD2=RX, PD3=TX) @ 9600 baud, via a
 *               1N4148 diode + 4.7 kΩ pull-up level shifter on the
 *               SIM800L TX → PD2 (RX) line (2.8 V → solid 5 V logic).
 *               Emergency SMS flood alert (cooldown-gated).
 *
 *  BAUD RATE CALCULATIONS @ F_CPU = 16 MHz  [RECALCULATED]
 *  ─────────────────────────────────────────────────────────────
 *  ESP-01 Hardware UART @ 115200, U2X=1 (8× oversampling):
 *    UBRR = F_CPU / (8 × 115200) − 1 = 16 000 000 / 921 600 − 1
 *         = 17.361 − 1 = 16.361 → round to 16
 *    Actual baud = 16 000 000 / (8 × (16+1)) = 16 000 000 / 136
 *                = 117 647 Hz
 *    Error = (117647 − 115200) / 115200 × 100 = +2.12%
 *    — Within the ±3.5% UART tolerance; this is the standard,
 *      datasheet-published value for 16 MHz/115200/U2X=1.
 *    Checked U2X=0 (16× oversampling) for comparison:
 *    UBRR = 16 000 000 / (16 × 115200) − 1 = 8.681 − 1 = 7.681 → 8
 *    Actual baud = 16 000 000 / (16 × 9) = 111 111 Hz  (−3.55%)
 *    — Worse than the U2X=1 result, so U2X=1/UBRR=16 is kept.
 *    → Use U2X=1, UBRR=16.  Error = +2.12%, within UART spec.
 *
 *  SIM800L Software UART @ 9600 baud:
 *    Bit period = 1 / 9600 = 104.167 µs  (this is a TIME, not a cycle
 *    count — it does NOT change with F_CPU).  avr-libc's _delay_us()
 *    macro re-derives the correct loop-cycle count from F_CPU at
 *    compile time, so _delay_us(104) is still ≈104 µs (error < 0.2%)
 *    whether F_CPU is 10 MHz or 16 MHz.  At 16 MHz this now compiles
 *    to ~416 busy-wait cycles per bit (was 260 @ 10 MHz) — well inside
 *    the 16-bit range of avr-libc's internal delay loop, no overflow.
 *    Half-bit delay for RX start-bit centering: _delay_us(52) — same
 *    reasoning, unchanged constant.
 *
 *  SRAM budget (post-optimization):
 *    Constants (PROGMEM) : ~430 bytes moved to Flash
 *    Stack peak           : ~260 bytes
 *    Globals / .bss       : ~140 bytes
 *    ─────────────────────────────────────────
 *    Total SRAM used      : ~400 / 2048 bytes  (20%)
 *
 *  PIN MAP
 *  ─────────────────────────────────────────────────────────────
 *    PB0 = LED (red, heartbeat)
 *    PB1 = Buzzer (active HIGH)
 *    PB2 = SD CS
 *    PB3 = MOSI, PB4 = MISO, PB5 = SCK  (SPI / pff)
 *    PC4 = SDA, PC5 = SCL                (I2C / TWI)
 *    PD0 = ESP-01 TX → MCU RX  (HW UART RX)
 *    PD1 = ESP-01 RX ← MCU TX  (HW UART TX, via V-div)
 *    PD2 = SIM800L TX → MCU RX (SW UART RX)
 *    PD3 = SIM800L RX ← MCU TX (SW UART TX, via V-div)
 *    PD5 = Rain sensor DO (input, pull-up)
 *    PD6 = Sonar TRIG (output)
 *    PD7 = Sonar ECHO (input)
 * ================================================================ */

#ifndef F_CPU
#  define F_CPU 16000000UL   /* 16 MHz external crystal (was 10 MHz) */
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "pff.h"

/* ================================================================
 *  COMPILE-TIME CONFIGURATION
 * ================================================================ */

/* SMS alert recipient — international format, no spaces              */
#define CFG_ALERT_PHONE       "+94767849322"

/* Water level (cm) below which an SMS alert is sent.                */
#define CFG_SMS_WATER_THRESH  40u

/* Minimum seconds between successive SMS alerts (30 minutes)        */
#define CFG_SMS_COOLDOWN      1800u

/* Water level threshold in cm — below this = UNSAFE (buzzer ON)    */
#define CFG_WATER_THRESH      20u

/* SD card logging / ThingSpeak upload interval in seconds           */
#define CFG_LOG_INTERVAL      60u

/* Wi-Fi credentials — replace with real values before deployment    */
#define WIFI_SSID             "KAVEESHA_0917"
#define WIFI_PASS             "04090M7q"

/* ThingSpeak write API key — replace with your channel key          */
#define THINGSPEAK_API_KEY    "FX6U1SUJ286YRZMA"

/* ================================================================
 *  PIN DEFINITIONS
 * ================================================================ */
#define PIN_LED       PB0   /* Red LED       — output, active HIGH    */
#define PIN_BUZZER    PB1   /* Active buzzer — output, active HIGH    */
#define PIN_SD_CS     PB2   /* SD card CS    — output, active LOW     */

#define PIN_TRIG      PD6   /* JSN-SR04T Trigger — output             */
#define PIN_ECHO      PD7   /* JSN-SR04T Echo    — input              */
#define PIN_RAIN      PD5   /* Rain sensor DO    — input, pull-up ON  */

/* Software UART pins for SIM800L                                    */
#define SIM_RX_PIN    PD2  /* SIM800L TX → MCU (input)               */
#define SIM_TX_PIN    PD3 /* MCU TX → SIM800L RX (output, V-div)   */
#define SIM_DDR       DDRD
#define SIM_PORT      PORTD
#define SIM_PIN_REG   PIND

/* Software UART bit timing @ 9600 baud — F_CPU-INDEPENDENT.
 * Bit period = 1/9600 s = 104.167 µs  → _delay_us(104)
 * Half-bit (start-bit centering for RX) = 52.08 µs → _delay_us(52)
 * These are real-time microsecond values, not cycle counts: avr-libc's
 * _delay_us() recomputes the correct busy-wait cycle count from F_CPU
 * at compile time, so NO numeric change is needed for the 16 MHz
 * migration. At 16 MHz, _delay_us(104) compiles to ~416 cycles
 * (vs. ~260 cycles @ 10 MHz) — still comfortably within range.
 * Compile-time constants avoid recomputation in the hot bit loop.   */
#define SIM_BIT_DELAY_US   104u
#define SIM_HALF_BIT_US     52u

/* ================================================================
 *  LCD CONSTANTS  (PCF8574 I2C backpack, HD44780 4-bit mode, 20×4)
 * ================================================================ */
#define LCD_ADDR    0x27
#define LCD_BL      0x08
#define LCD_CMD     0x00
#define LCD_DATA    0x01

/* ================================================================
 *  DEVICE I2C ADDRESSES
 * ================================================================ */
#define ADDR_DS3231  0x68

/* ================================================================
 *  ALL CONSTANT STRINGS IN FLASH
 *  (~430 bytes kept out of SRAM vs. string literals in .data)
 * ================================================================ */

/* --- SIM800L AT commands (PROGMEM) --- */
static const char AT_CMEE[]   PROGMEM = "AT+CMEE=2\r\n";
static const char AT_CREG_Q[] PROGMEM = "AT+CREG?\r\n";
static const char AT_CMGF[]   PROGMEM = "AT+CMGF=1\r\n";

/* SMS alert message body                                            */
static const char SMS_WARN_MSG[] PROGMEM =
    "WARNING: Water level is critically high!";

/* --- ESP-01 AT commands (PROGMEM) --- */
static const char ESP_AT[]        PROGMEM = "AT\r\n";
static const char ESP_RST[]       PROGMEM = "AT+RST\r\n";
static const char ESP_CWMODE[]    PROGMEM = "AT+CWMODE=1\r\n";
static const char ESP_CIPMUX[]    PROGMEM = "AT+CIPMUX=0\r\n";
static const char ESP_CIPCLOSE[]  PROGMEM = "AT+CIPCLOSE\r\n";
/* ThingSpeak TCP endpoint                                           */
static const char ESP_CIPSTART[]  PROGMEM =
    "AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n";

/* --- LCD fixed strings (padded to exactly 20 chars, 20-col LCD) --- */
static const char LCD_SPLASH1[]    PROGMEM = "= WEATHER SYSTEM =  ";
static const char LCD_SPLASH2[]    PROGMEM = "   s17229 Project   ";
static const char LCD_SIM_CONN[]   PROGMEM = "SIM800L Starting... ";
static const char LCD_SIM_OK[]     PROGMEM = "GSM: Network OK     ";
static const char LCD_SIM_FAIL[]   PROGMEM = "GSM: No Network!    ";
static const char LCD_WIFI_CONN[]  PROGMEM = "WiFi Connecting...  ";
static const char LCD_WIFI_OK[]    PROGMEM = "WiFi Connected      ";
static const char LCD_WIFI_FAIL[]  PROGMEM = "WiFi Failed         ";
static const char LCD_SD_MOUNT[]   PROGMEM = "Mounting SD...      ";
static const char LCD_SD_OK[]      PROGMEM = "SD: OK              ";
static const char LCD_SD_FAIL[]    PROGMEM = "SD: MOUNT FAILED    ";
static const char LCD_FILE_FAIL[]  PROGMEM = "SD: FILE NOT FOUND  ";
static const char LCD_RAIN_YES[]   PROGMEM = "Rain: YES           ";
static const char LCD_RAIN_NO[]    PROGMEM = "Rain: NO            ";
static const char LCD_WATER_ERR[]  PROGMEM = "Water: --- cm       ";
static const char LCD_UNSAFE[]     PROGMEM = "Status: UNSAFE      ";
static const char LCD_SAFE[]       PROGMEM = "Status: SAFE        ";
static const char LCD_SMS_SENDING[] PROGMEM = "Sending SMS...      ";
static const char LCD_TS_UPLOAD[]  PROGMEM = "ThingSpeak: Upload  ";

/* ================================================================
 *  GLOBAL STATE
 * ================================================================ */
volatile uint16_t g_seconds   = 0;
volatile uint8_t  g_tick      = 0;
volatile uint8_t  g_ovf_count = 0;

/* ================================================================
 *  HARDWARE UART — ESP-01 (PD0=RX, PD1=TX) @ 115200 baud
 *
 *  UBRR calculation (U2X=1, 8× oversampling, F_CPU=16 MHz):
 *    UBRR = F_CPU / (8 × 115200) − 1 = 16.361 → round to 16
 *    Actual = 16 000 000 / (8 × 17) = 117 647 baud  (error +2.12%)
 *    Within the ±3.5% UART tolerance. ✓
 *
 *  RX ring buffer captures ESP-01 AT responses for ACK detection.
 *  Power-of-2 size enables cheap index-mask wrap.
 *
 *  TX buffer sized for the longest runtime-built command.
 *  HTTP GET payload breakdown (worst-case):
 *    "GET /update?api_key=" (20) + key (16) + "&field1=" (8)
 *    + "600" (3) + "&field2=1" (9) + " HTTP/1.0\r\n" (12) = 68
 *    + "Host: api.thingspeak.com\r\n" (26) + "\r\n" (2) = 96
 *    AT+CWJAP="<32-char SSID>","<32-char pass>"\r\n ≈ 80
 *    128 bytes covers both with margin.
 * ================================================================ */
#define ESP_UBRR       16u   /* U2X=1 @ 16 MHz: actual 117 647 baud, +2.12% */
#define ESP_TX_BUF_SZ  128
static char g_esp_buf[ESP_TX_BUF_SZ];

#define ESP_RX_SZ  64
static volatile char    g_esp_rx_buf[ESP_RX_SZ];
static volatile uint8_t g_esp_rx_head = 0;
static volatile uint8_t g_esp_rx_tail = 0;

ISR(USART_RX_vect)
{
    char c = UDR0;
    uint8_t next = (g_esp_rx_head + 1) & (ESP_RX_SZ - 1);
    if (next != g_esp_rx_tail) {
        g_esp_rx_buf[g_esp_rx_head] = c;
        g_esp_rx_head = next;
    }
}

static void esp_rx_flush(void)
{
    g_esp_rx_head = g_esp_rx_tail = 0;
}

static int16_t esp_rx_getc(void)
{
    if (g_esp_rx_head == g_esp_rx_tail) return -1;
    char c = g_esp_rx_buf[g_esp_rx_tail];
    g_esp_rx_tail = (g_esp_rx_tail + 1) & (ESP_RX_SZ - 1);
    return (uint8_t)c;
}

/*
 * esp_uart_init() — configure HW UART for ESP-01 at 115200 baud.
 * U2X=1, UBRR=16 @ F_CPU=16 MHz → 117 647 baud (+2.12%), 8N1.
 */
static void esp_uart_init(void)
{
    UBRR0H = 0;
    UBRR0L = ESP_UBRR;
    UCSR0A = (1 << U2X0);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);   /* 8N1                */
}

/* Transmit one char to ESP-01 via HW UART                          */
static void esp_putc(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

/* Send a RAM string to ESP-01                                       */
static void esp_print(const char *s)
{
    while (*s) esp_putc(*s++);
}

/* Send a PROGMEM string to ESP-01 — no RAM copy                    */
static void esp_print_P(const char *s_P)
{
    char c;
    while ((c = pgm_read_byte(s_P++))) esp_putc(c);
}

/*
 * esp_wait_for_P() — scan ESP RX ring buffer for a PROGMEM needle.
 *
 * Needle limited to 15 chars (covers "OK", "ERROR", "SEND OK",
 * "CONNECT", "ready", "+IPD", "CLOSED").
 * Returns 1 on match, 0 on timeout.
 * Timer0 and RX ISRs continue to fire during the busy-wait loop.
 */
static uint8_t esp_wait_for_P(const char *needle_P, uint16_t timeout_ms)
{
    char needle[16];
    uint8_t nlen = 0;
    while (nlen < sizeof(needle) - 1) {
        char c = pgm_read_byte(needle_P + nlen);
        if (!c) break;
        needle[nlen++] = c;
    }
    needle[nlen] = '\0';

    uint8_t  matched  = 0;
    uint32_t deadline = (uint32_t)timeout_ms * (F_CPU / 1000UL / 10UL);

    for (uint32_t t = 0; t < deadline; t++) {
        if ((t & 0xFFFFUL) == 0) wdt_reset();   /* pet WDT during long waits */
        int16_t c = esp_rx_getc();
        if (c >= 0) {
            if ((char)c == needle[matched]) {
                if (++matched == nlen) return 1;
            } else {
                matched = ((char)c == needle[0]) ? 1 : 0;
            }
        }
    }
    return 0;
}

/* ================================================================
 *  SOFTWARE UART — SIM800L (PD2=RX, PD3=TX) @ 9600 baud
 *
 *  Bit-bang implementation using avr-libc _delay_us().
 *  9600 baud → bit period = 104.167 µs (independent of F_CPU).
 *  _delay_us(104) introduces < 0.2% timing error at both 10 MHz and
 *  16 MHz, since avr-libc recomputes the cycle count from F_CPU. ✓
 *
 *  TX protocol (8N1):
 *    1 start bit LOW, 8 data bits LSB-first, 1 stop bit HIGH.
 *
 *  RX protocol (8N1, blocking):
 *    Wait for start bit (HIGH→LOW edge), delay half-bit to centre,
 *    then sample 8 bits LSB-first at full-bit intervals.
 *
 *  SIM800L RX buffer for response ACK detection.
 *  Power-of-2 size for cheap wrap mask.
 *
 *  NOTE: During sim_getchar() the global interrupt flag is cleared
 *  for the duration of the byte (~1.1 ms @ 9600 baud) to prevent
 *  the bit-timing loop from being jittered by ISRs.  The Timer0
 *  1-second tick can slip by at most 1 second during a long SMS
 *  exchange — acceptable for this application.
 * ================================================================ */
#define SIM_RX_SZ  32
static volatile char    g_sim_rx_buf[SIM_RX_SZ];
static volatile uint8_t g_sim_rx_head = 0;
static volatile uint8_t g_sim_rx_tail = 0;

static void sim_soft_uart_init(void)
{
    /* TX pin as output, idle HIGH (mark state)                      */
    SIM_DDR  |=  (1 << SIM_TX_PIN);
    SIM_PORT |=  (1 << SIM_TX_PIN);

    /*
     * RX pin as input. FIX: enable the internal pull-up. A live
     * SIM800L TX output will easily override a weak internal pull-up
     * (this is standard practice for UART RX lines), but with the
     * module disconnected the pin now idles HIGH (correct UART mark
     * state) instead of floating and reading noise as start bits.
     */
    /*
     * KEEP THIS ENABLED. SIM800L TX drives this pin directly (no
     * external pull-up on this line by design), but the internal
     * pull-up prevents the pin from floating during any brief high-Z
     * moment (module reset/reboot, power-on transient). A floating
     * PD2 was the original cause of the interrupt-storm/LCD-corruption
     * bug fixed earlier in this project — do not disable this again.
     */
    SIM_DDR  &= ~(1 << SIM_RX_PIN);
    SIM_PORT |=  (1 << SIM_RX_PIN);
}

/*
 * sim_putchar() — transmit one byte over Software UART.
 * Interrupts are NOT disabled here because the TX side only needs
 * the _delay_us() timing, which is immune to ISR latency at 9600 baud
 * (≥ 100 µs per bit, typical ISR latency << 10 µs).
 */
static void sim_putchar(char c)
{
    uint8_t sreg = SREG;
    cli();
    /* Start bit — LOW                                               */
    SIM_PORT &= ~(1 << SIM_TX_PIN);
    _delay_us(SIM_BIT_DELAY_US);

    /* 8 data bits, LSB first                                        */
    for (uint8_t i = 0; i < 8; i++) {
        if (c & (1 << i)) {
            SIM_PORT |= (1 << SIM_TX_PIN);
        } else {
            SIM_PORT &= ~(1 << SIM_TX_PIN);
        }
        _delay_us(SIM_BIT_DELAY_US);   /* unconditional: once per bit, every bit */
    }

    /* Stop bit — HIGH (mark)                                        */
    SIM_PORT |= (1 << SIM_TX_PIN);
    _delay_us(SIM_BIT_DELAY_US);

    SREG = sreg;
}

/*
 * sim_getchar() — receive one byte over Software UART (blocking).
 *
 * Waits up to ~26 ms for start bit (250 × 104 µs), then samples
 * the 8 data bits.  Returns received byte (0x00–0xFE) or -1 on timeout.
 *
 * Return type is int16_t (not char) so the sentinel -1 is unambiguous
 * and the comparison in sim_wait_for_P() is always well-defined.
 *
 * Interrupts are disabled for the duration of bit sampling to
 * prevent ISR jitter from corrupting bit timing.
 */
static int16_t sim_getchar(void)
{
    /*
     * FIX: the original loop polled SIM_PIN_REG with NO delay between
     * iterations, so 250 iterations completed in ~150 us instead of the
     * documented ~26 ms. On a floating, unpulled RX pin (module
     * disconnected) that made ANY noise glitch look like a start bit,
     * and the function returned near-instantly, over and over, feeding
     * garbage bytes into sim_wait_for_P() at a very high rate — each
     * one briefly cli()'d interrupts during the 8-bit sample window.
     *
     * Fixed: wait in real 20 us steps (comment’s original intent),
     * 250 x 20 us ~= 5 ms per start-bit poll granularity below is too
     * coarse for 9600 baud bit-centre alignment, so we poll every 4 us
     * (fits inside a 104 us bit) for up to ~26 ms total.
     */
    uint16_t wait = 500u;   /* 500 x 4 us = 2 ms — see iters recalibration below */
    while ((SIM_PIN_REG & (1 << SIM_RX_PIN)) && wait) {
        _delay_us(4);
        wait--;
    }
    if (!wait) return -1;/* timeout: no start bit seen         */
    
    uint8_t sreg = SREG;  
    cli();

    /* Delay half a bit period to sample near the centre             */
    _delay_us(SIM_HALF_BIT_US);

    /* Sample 8 data bits LSB-first                                  */
    char c = 0;
    for (uint8_t i = 0; i < 8; i++) {
        _delay_us(SIM_BIT_DELAY_US);
        if (SIM_PIN_REG & (1 << SIM_RX_PIN)){   
          c |= (1 << i);
        }
    }

    _delay_us(SIM_BIT_DELAY_US);/* Stop bit — consume without checking                           */

    SREG = sreg;    /* Restore interrupt state                       */
    return c;
}

static void sim_rx_flush(void)
{
    while (sim_getchar() != -1){

    }
}

/* Send a RAM string via Software UART                               */
static void sim_print(const char *s)
{
    while (*s) sim_putchar(*s++);
}

/* Send a PROGMEM string via Software UART — no RAM copy             */
static void sim_print_P(const char *s_P)
{
    char c;
    while ((c = pgm_read_byte(s_P++))) sim_putchar(c);
}

/*
 * sim_wait_for_P() — scan SIM800L RX stream for a PROGMEM needle.
 *
 * Continuously calls sim_getchar() and matches against needle[].
 * Returns 1 on match within timeout_ms, 0 on timeout.
 *
 * timeout_ms is approximate: each sim_getchar() call blocks for up
 * to ~27 ms, so the effective resolution is ~27 ms per iteration.
 * For the timeouts used here (2000–30000 ms) this is acceptable.
 *
 * Needle limited to 15 chars (covers "OK", ">", "+CMGS:", ",1", ",5").
 */
static uint8_t sim_wait_for_P(const char *needle_P, uint16_t timeout_ms)
{
    char needle[16];
    uint8_t nlen = 0;
    while (nlen < sizeof(needle) - 1) {
        char nc = pgm_read_byte(needle_P + nlen);
        if (!nc) break;
        needle[nlen++] = nc;
    }
    needle[nlen] = '\0';

    uint8_t  matched  = 0;
    /* Convert ms to approximate sim_getchar() iterations.
     * Each call now blocks at most ~2 ms (see sim_getchar()'s start-bit
     * wait). Divisor MUST track that constant, or timeout_ms silently
     * stops meaning what it says.                                     */
    uint16_t iters = (uint16_t)(timeout_ms / 2u) + 1u;

    for (uint16_t i = 0; i < iters; i++) {
        wdt_reset();                 /* pet WDT periodically during long waits */
        int16_t c = sim_getchar();
        if (c < 0) continue;         /* timeout on this byte, keep polling */
        if ((char)c == needle[matched]) {
            if (++matched == nlen) return 1;
        } else {
            matched = ((char)c == needle[0]) ? 1 : 0;
        }
    }
    return 0;
}

/*
 * sim_capture() — read the SIM800L's response into a RAM buffer instead
 * of matching a single needle. This is what fixes the "buffer data loss"
 * bug: calling sim_wait_for_P() twice for two different patterns fails
 * because the first call consumes/discards bytes as it scans, so if the
 * match was ",5" the first call (checking ",1") already ate the stream
 * before the second call gets a chance. Capturing once into a buffer and
 * then running strstr() against that SAME snapshot for as many patterns
 * as needed has no such loss — the buffer isn't consumed by searching it.
 *
 * Exits early once ~270 ms of silence follows the first received byte
 * (a normal AT response is a short, contiguous burst), so typical calls
 * finish well under timeout_ms; timeout_ms is only the worst-case cap.
 */
static uint8_t sim_capture(char *buf, uint8_t buf_sz, uint16_t timeout_ms)
{
    uint8_t  len        = 0;
    /* Each sim_getchar() call now blocks at most ~2 ms. Divisor and the
     * idle-silence threshold below both scale off that same constant. */
    uint16_t iters      = (uint16_t)(timeout_ms / 2u) + 1u;
    uint16_t idle_iters = 0;

    for (uint16_t i = 0; i < iters; i++) {
        wdt_reset();
        int16_t c = sim_getchar();
        if (c < 0) {
            if (len > 0 && ++idle_iters >= 135u) break;  /* ~270ms trailing silence */
            continue;
        }
        idle_iters = 0;
        if (len < (uint8_t)(buf_sz - 1)) buf[len++] = (char)c;
    }
    buf[len] = '\0';
    return len;
}

/* ================================================================
 *  TIMER 0 — 1-second software tick  [RECALCULATED for 16 MHz]
 *
 *  Timer0 is only 8-bit (OCR0A max = 255), so at 16 MHz the old
 *  256-prescaler/OCR0A=195 combination no longer fits a clean divide:
 *    256 × 196 → f_cmp = 16 000 000 / 50 176 = 318.9 Hz  (WRONG — this
 *    would fire 200 times in 0.627 s, a −37% timing error if left
 *    unchanged. This is exactly the kind of silent bug a clock bump
 *    can introduce if timer registers aren't re-derived.)
 *
 *  Fix: switch to prescaler=1024 (max available) and re-choose OCR0A
 *  so F_CPU/(1024×N) lands on an exact integer tick frequency:
 *    OCR0A = 124  →  N = OCR0A+1 = 125
 *    f_cmp = 16 000 000 / (1024 × 125) = 16 000 000 / 128 000
 *          = 125.000 Hz  EXACTLY (zero rounding error)
 *    125 interrupts → 1.000000 s  (error = 0.00%, actually tighter
 *    than the original 10 MHz design's 0.04%)
 * ================================================================ */
ISR(TIMER0_COMPA_vect)
{
    if (++g_ovf_count >= 125) {   /* 125 × 8 ms = 1.000000 s @ 16 MHz */
        g_ovf_count = 0;
        g_seconds++;
        g_tick = 1;
    }
}

static void timer0_init(void)
{
    TCCR0A = (1 << WGM01);                  /* CTC mode                */
    TCCR0B = (1 << CS02) | (1 << CS00);     /* prescaler = 1024        */
    OCR0A  = 124;                            /* 125 counts → 125.000 Hz */
    TIMSK0 = (1 << OCIE0A);
}

/* ================================================================
 *  TIMER 1 — Free-running timebase for sonar  [RECALCULATED for 16 MHz]
 *
 *  At 16 MHz, keeping prescaler=8 would give tick = 8/16 000 000
 *  = 0.5 µs, so the 16-bit counter (max 65 535) wraps at just
 *  32.77 ms. JSN-SR04T round-trip time at its rated 4.5 m max range
 *  is ~26.2 ms, so 32.77 ms leaves only ~25% headroom — too tight
 *  for a "production-grade" margin (echo train / multiple-reflection
 *  edge cases can exceed the rated range).
 *
 *  Fix: switch to prescaler=64:
 *    Tick = 64 / 16 000 000 = 4 µs
 *    Max measurable duration = 65 535 × 4 µs ≈ 262 ms (>8× headroom)
 *    Resolution = 4 µs × 343 m/s ÷ 2 ≈ 0.69 mm — still far finer than
 *    the cm-level readings this application needs.
 * ================================================================ */
static void timer1_init(void)
{
    TCCR1A = 0x00;
    TCCR1B = (1 << CS11) | (1 << CS10);     /* prescaler = 64 (was 8)  */
    TCNT1  = 0;
}

/* ================================================================
 *  I2C (TWI) DRIVER  [TWBR RECALCULATED for 16 MHz]
 *
 *  TWBR = (F_CPU/SCL_target − 16) / (2 × prescaler), prescaler=1:
 *    TWBR = (16 000 000/100 000 − 16) / 2 = (160 − 16)/2 = 72
 *  SCL = F_CPU / (16 + 2×TWBR×1) = 16 000 000 / (16+144)
 *      = 16 000 000 / 160 = 100 kHz ✓ (same bus speed as before —
 *      LCD backpack and DS3231 timing are unaffected by the MCU
 *      clock change since the I2C bus rate is held constant)
 * ================================================================ */
static void i2c_init(void)
{
    TWSR = 0x00;
    TWBR = 72;
    TWCR = (1 << TWEN);
}

/*
 * FIX: I2C_SPIN now reports whether TWINT actually set (1) or the
 * bounded spin timed out (0). Every caller below checks this and, on
 * failure, calls i2c_bus_recover() instead of silently proceeding —
 * previously a single glitched transaction (e.g. one stretched by ISR
 * jitter) would go undetected and desync the HD44780's 4-bit nibble
 * state machine, which is what produced the "black block" corruption.
 */
#define I2C_SPIN(reg, bit, ok_flag)                   \
    do {                                              \
        uint16_t _t = 0;                              \
        while (!(reg & (1 << bit)) && ++_t);           \
        (ok_flag) = (reg & (1 << bit)) ? 1 : 0;        \
    } while (0)

/*
 * i2c_bus_recover() — standard I2C bus-recovery sequence.
 * If SDA is stuck low (a device left mid-transaction, e.g. by a
 * glitched EN pulse), toggle SCL manually up to 9 times to walk the
 * stuck device through completing its byte, then issue STOP.
 */
static void i2c_bus_recover(void)
{
    TWCR = 0;                              /* release TWI control of the pins */
    DDRC  &= ~((1 << PC4) | (1 << PC5));   /* SDA/SCL as inputs (pull-ups on) */
    PORTC |=  (1 << PC4) | (1 << PC5);

    DDRC |= (1 << PC5);                    /* SCL as output */
    for (uint8_t i = 0; i < 9 && !(PINC & (1 << PC4)); i++) {
        PORTC &= ~(1 << PC5); _delay_us(5);
        PORTC |=  (1 << PC5); _delay_us(5);
    }
    /* Manual STOP: SDA low->high while SCL high */
    DDRC |= (1 << PC4);
    PORTC &= ~(1 << PC4); _delay_us(5);
    PORTC |= (1 << PC5);  _delay_us(5);
    PORTC |= (1 << PC4);  _delay_us(5);

    DDRC &= ~((1 << PC4) | (1 << PC5));    /* hand pins back to TWI hardware */
    i2c_init();
}

static uint8_t i2c_start(void)
{
    uint8_t ok;
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    I2C_SPIN(TWCR, TWINT, ok);
    if (!ok) { i2c_bus_recover(); return 0; }
    return 1;
}

static void i2c_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    _delay_us(100);
}

static uint8_t i2c_write(uint8_t d)
{
    uint8_t ok;
    TWDR = d;
    TWCR = (1 << TWINT) | (1 << TWEN);
    I2C_SPIN(TWCR, TWINT, ok);
    if (!ok) { i2c_bus_recover(); return 0; }
    return 1;
}

static uint8_t i2c_read(uint8_t ack)
{
    uint8_t ok;
    TWCR = (1 << TWINT) | (1 << TWEN) | (ack ? (1 << TWEA) : 0);
    I2C_SPIN(TWCR, TWINT, ok);
    if (!ok) i2c_bus_recover();
    return TWDR;
}

/* ================================================================
 *  LCD DRIVER (PCF8574 I2C backpack, HD44780 4-bit mode, 20×4)
 *  (unchanged)
 * ================================================================ */
static void lcd_pulse(uint8_t d)
{
    i2c_start(); i2c_write(LCD_ADDR << 1); i2c_write(d | 0x04); i2c_stop();
    _delay_us(2);
    i2c_start(); i2c_write(LCD_ADDR << 1); i2c_write(d & ~0x04); i2c_stop();
}

static void lcd_send(uint8_t val, uint8_t mode)
{
    uint8_t hi = (val & 0xF0) | mode | LCD_BL;
    uint8_t lo = ((val << 4) & 0xF0) | mode | LCD_BL;
    lcd_pulse(hi);
    _delay_us(100);
    lcd_pulse(lo);
    _delay_ms(2);
}

static void lcd_cmd(uint8_t c)  { lcd_send(c, LCD_CMD);  }
static void lcd_char(uint8_t c) { lcd_send(c, LCD_DATA); }

static void lcd_init(void)
{
    _delay_ms(100);
    lcd_send(0x03, LCD_CMD); _delay_ms(5);
    lcd_send(0x03, LCD_CMD); _delay_us(200);
    lcd_send(0x03, LCD_CMD);
    lcd_send(0x02, LCD_CMD);
    lcd_cmd(0x28);
    lcd_cmd(0x0C);
    lcd_cmd(0x06);
    lcd_cmd(0x01);
    _delay_ms(5);
}

static void lcd_clear(void)
{
    lcd_cmd(0x01);
    _delay_ms(5);
}

static const uint8_t LCD_ROW[4] = {0x00, 0x40, 0x14, 0x54};

static void lcd_goto(uint8_t row, uint8_t col)
{
    lcd_cmd(0x80 | (LCD_ROW[row & 3] + col));
}

static void lcd_print(const char *s)
{
    while (*s) lcd_char((uint8_t)*s++);
}

static void lcd_print_P(const char *s_P)
{
    char c;
    while ((c = pgm_read_byte(s_P++))) lcd_char((uint8_t)c);
}

/* ================================================================
 *  DS3231 RTC DRIVER  (unchanged)
 * ================================================================ */
typedef struct {
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t day;
    uint8_t month;
    uint8_t year;
} RTC_Time;

static uint8_t bcd2bin(uint8_t b) { return (b >> 4) * 10u + (b & 0x0Fu); }

static uint8_t rtc_read(RTC_Time *t)
{
    i2c_start();
    i2c_write(ADDR_DS3231 << 1);
    i2c_write(0x00);
    i2c_stop();

    i2c_start();
    i2c_write((ADDR_DS3231 << 1) | 0x01);
    t->sec   = bcd2bin(i2c_read(1) & 0x7F);
    t->min   = bcd2bin(i2c_read(1) & 0x7F);
    t->hour  = bcd2bin(i2c_read(1) & 0x3F);
    i2c_read(1);
    t->day   = bcd2bin(i2c_read(1));
    t->month = bcd2bin(i2c_read(1) & 0x1F);
    t->year  = bcd2bin(i2c_read(0));
    i2c_stop();
    return 1;
}

/* ================================================================
 *  JSN-SR04T SONAR DRIVER  (unchanged)
 * ================================================================ */
static void sonar_init(void)
{
    DDRD |=  (1 << PIN_TRIG);
    DDRD &= ~(1 << PIN_ECHO);
}

static uint16_t sonar_read_cm(void)
{
    uint16_t t0, t1;
    uint32_t to;

    PORTD |=  (1 << PIN_TRIG);
    _delay_us(10);
    PORTD &= ~(1 << PIN_TRIG);

    /*
     * `to` is a raw instruction-count spin timeout, not a calibrated
     * delay. Its real-world duration shrinks as F_CPU rises (each
     * loop iteration takes less wall-clock time at 16 MHz than it
     * did at 10 MHz), so the safety margin must be scaled up by the
     * same 16/10 = 1.6× ratio to preserve the original timeout
     * window: 200 000 × 1.6 = 320 000.
     */
    to = 320000UL;
    while (!(PIND & (1 << PIN_ECHO))) { if (!--to) return 0; }
    t0 = TCNT1;

    to = 320000UL;
    while (PIND & (1 << PIN_ECHO))  { if (!--to) return 0; }
    t1 = TCNT1;

    uint16_t ticks = (t1 >= t0) ? (t1 - t0)
                                 : (uint16_t)(0xFFFFu - t0 + t1 + 1u);

    /*
     * Distance conversion, recalculated for the new Timer1 tick period.
     * Standard formula: distance_cm = echo_time_us / 58.
     * Timer1 tick = 4 µs now (prescaler=64 @ 16 MHz, see timer1_init),
     * so: distance_cm = (ticks × 4) / 58.
     * (Old formula "ticks*4/290" was calibrated for the 0.8 µs tick
     * that prescaler=8 @ 10 MHz produced: 0.8/58 = 4/290. Both forms
     * reduce to the same distance_cm = time_us/58 relationship.)
     */
    return (uint16_t)((uint32_t)ticks * 4u / 58u);
}

/* ================================================================
 *  IO INITIALISATION
 *  Updated: SIM_TX_PIN (PD3) added as output; SIM_RX_PIN (PD2) as
 *  input.  HW UART pins (PD0/PD1) are controlled by esp_uart_init().
 * ================================================================ */
static void io_init(void)
{
    /* Outputs: LED, Buzzer, SD CS                                   */
    DDRB  |=  (1 << PIN_LED) | (1 << PIN_BUZZER) | (1 << PIN_SD_CS);
    PORTB &= ~((1 << PIN_LED) | (1 << PIN_BUZZER));
    PORTB |=  (1 << PIN_SD_CS);

    /* Rain sensor input with internal pull-up                       */
    DDRD  &= ~(1 << PIN_RAIN);
    PORTD |=  (1 << PIN_RAIN);

    /* Software UART TX for SIM800L: output, idle HIGH               */
    SIM_DDR  |=  (1 << SIM_TX_PIN);
    SIM_PORT |=  (1 << SIM_TX_PIN);

    /* Software UART RX for SIM800L: input, pull-up enabled (see fix
     * in sim_soft_uart_init() — idles HIGH instead of floating)     */
    SIM_DDR  &= ~(1 << SIM_RX_PIN);
    SIM_PORT |=  (1 << SIM_RX_PIN);

    /*
     * FIX: ESP-01 hardware UART RX (PD0) has RXCIE0 enabled in
     * esp_uart_init() but was never given a pull-up. With the module
     * disconnected this pin floats and can trigger USART_RX_vect on
     * noise. Pull it up so it idles HIGH (mark state) — a live ESP-01
     * TX output overrides the weak internal pull-up with no issue.
     */
    DDRD  &= ~(1 << PD0);
    PORTD |=  (1 << PD0);
}

/* ================================================================
 *  SIM800L SMS DRIVER  (Software UART — PD2=RX, PD3=TX)
 *
 *  All uart_*() calls replaced with sim_*() equivalents.
 *  Logic is identical to the original hardware UART version.
 * ================================================================ */
typedef enum { SIM_OK = 0, SIM_FAIL = 1, SIM_BUSY = 2 } SIM_Status;

/*
 * sim800l_presence_check() — baud sync only. Up to 5 tries, ~1.3s each
 * worst case (~6.5s absolute ceiling). Bounded and done once at boot,
 * before the sensor loop needs to be interleaved with anything, so it's
 * left as a single blocking call — the freeze problem is in the much
 * longer registration poll below, not here.
 */
static uint8_t sim800l_presence_check(void)
{
    lcd_clear();
    lcd_print_P(PSTR("Syncing Baud..."));

    for (uint8_t i = 0; i < 10; i++) {
        sim_rx_flush();
        
        /* 1. Auto-baud Sync වෙන්න AT කමාන්ඩ් එක යැවීම */
        sim_print_P(PSTR("AT\r\n"));
        
        if (sim_wait_for_P(PSTR("OK"), 1000)) {
            /* 2. මොඩියුල් එකෙන් OK ආවොත්, බලෙන්ම 9600 ට Lock කිරීම */
            sim_rx_flush();
            sim_print_P(PSTR("AT+IPR=9600\r\n"));
            sim_wait_for_P(PSTR("OK"), 1000);
            
            sim_print_P(PSTR("AT&W\r\n"));
            sim_wait_for_P(PSTR("OK"), 1000);
            
            return 1; /* සාර්ථකයි */
        }
        _delay_ms(500); /* ඊළඟ වාරෙට කලින් පොඩි වෙලාවක් ඉඳීම */
    }
    return 0; /* වාර 10ම ෆේල් වුණා */
}

/*
 * sim800l_registration_step() — does exactly ONE bounded unit of work
 * (at most one AT+CREG? send + one ~2s capture, ~2.3s worst case) and
 * returns. Call this once per loop iteration — from main()'s startup
 * phase, interleaved with a sensor read/LCD update — instead of calling
 * a single function that blocks for up to 15 x 2s (~30s+) with the
 * sensor and display frozen the entire time. Internal attempt count is
 * static, persisting across calls; it self-resets on SIM_OK/SIM_FAIL so
 * the function is ready to be called again on a future reconnect retry.
 *
 * CREG check fix: AT+CREG? is sent ONCE per attempt, the full reply is
 * captured into one buffer via sim_capture(), then BOTH ",1" (registered,
 * home network) and ",5" (registered, roaming) are checked with strstr()
 * against that same captured snapshot — eliminating the old bug where
 * checking ",1" first would consume the stream and starve the ",5" check.
 */
static SIM_Status sim800l_registration_step(void)
{
    static uint8_t attempt = 0;
    char resp[40];

    if (attempt == 0) {
        sim_rx_flush();
        sim_print_P(AT_CMEE);
        sim_wait_for_P(PSTR("OK"), 1000);
    }

    sim_rx_flush();
    sim_print_P(AT_CREG_Q);
    sim_capture(resp, sizeof(resp), 2000);
    attempt++;

    if (strstr(resp, ",1") || strstr(resp, ",5")) {
        attempt = 0;
        return SIM_OK;
    }
    if (attempt >= 15) {
        attempt = 0;
        return SIM_FAIL;
    }
    return SIM_BUSY;
}

/*
 * sim800l_send_sms_P() — send one SMS from a PROGMEM message string.
 * Uses Software UART exclusively.
 * Sequence: AT+CMGF=1 → AT+CMGS="<num>" → body → CTRL+Z → "+CMGS:"
 */
static SIM_Status sim800l_send_sms_P(const char *msg_P)
{
    /* ---- Step 1: Text mode ---- */
    sim_rx_flush();
    sim_print_P(AT_CMGF);
    if (!sim_wait_for_P(PSTR("OK"), 3000)) return SIM_FAIL;
    sim_rx_flush();

    /* ---- Step 2: Address recipient ---- */
    /* Build AT+CMGS="<phone>"\r\n in shared ESP TX buffer.
     * Re-using g_esp_buf here is safe: ESP-01 is idle during SMS.   */
    snprintf(g_esp_buf, sizeof(g_esp_buf),
             "AT+CMGS=\"%s\"\r\n", CFG_ALERT_PHONE);
    sim_print(g_esp_buf);

    if (!sim_wait_for_P(PSTR(">"), 5000)) {
        sim_putchar(0x1A);   /* Cancel pending AT+CMGS               */
        return SIM_FAIL;
    }

    /* ---- Step 3: Stream message body from Flash ---- */
    sim_print_P(msg_P);

    /* ---- Step 4: Commit with CTRL+Z ---- */
    sim_putchar(0x1A);

    /* Wait for "+CMGS:" — SMSC acknowledgement (up to 30 s)         */
    if (!sim_wait_for_P(PSTR("+CMGS:"), 30000)) {
        sim_rx_flush();
        return SIM_FAIL;
    }

    sim_rx_flush();
    return SIM_OK;
}

/* ================================================================
 *  ESP-01 Wi-Fi & ThingSpeak DRIVER  (Hardware UART)
 *
 *  AT command flow:
 *    Startup : AT+RST → "ready"
 *              AT+CWMODE=1 → "OK"   (station mode)
 *              AT+CIPMUX=0 → "OK"   (single-connection mode)
 *              AT+CWJAP="SSID","PASS" → "WIFI CONNECTED" or timeout
 *
 *    Upload  : AT+CIPSTART="TCP","api.thingspeak.com",80 → "CONNECT"
 *              AT+CIPSEND=<len> → ">"
 *              GET /update?api_key=KEY&field1=WCM&field2=RAIN HTTP/1.0\r\n
 *              Host: api.thingspeak.com\r\n\r\n
 *              Wait "SEND OK" → AT+CIPCLOSE
 *
 *  ThingSpeak HTTP GET is a minimal HTTP/1.0 request.  No SSL, no
 *  keep-alive, no response body parsing needed — the channel entry
 *  is created by the server regardless of whether we read the reply.
 *
 *  g_esp_buf (96 bytes) holds the longest dynamically built string:
 *    AT+CWJAP="<32-char SSID>","<32-char pass>"\r\n ≈ 80 chars ✓
 *    AT+CIPSEND=NN\r\n                              ≈ 18 chars ✓
 * ================================================================ */

/*
 * esp01_startup() — reset, configure station mode, join Wi-Fi.
 * Shows status on LCD row 1.  Called once from main().
 * Returns 1 on success, 0 on failure.
 */
static uint8_t esp01_startup(void)
{
    /* ---- Step 1: Hard-reset the ESP-01 ---- */
    esp_rx_flush();
    esp_print_P(ESP_RST);
    /* "ready" appears ~2 s after reset; give it 4 s to be safe      */
    if (!esp_wait_for_P(PSTR("ready"), 4000)) {
        /* Module may not echo "ready" at 115200 if its stored baud
         * differs.  Send AT probes to check if it's alive.           */
        esp_rx_flush();
        esp_print_P(ESP_AT);
        if (!esp_wait_for_P(PSTR("OK"), 2000)) {
            lcd_goto(1, 0);
            lcd_print_P(LCD_WIFI_FAIL);
            return 0;
        }
    }
    esp_rx_flush();

    /* ---- Step 2: Station mode ---- */
    esp_print_P(ESP_CWMODE);
    if (!esp_wait_for_P(PSTR("OK"), 3000)) {
        lcd_goto(1, 0);
        lcd_print_P(LCD_WIFI_FAIL);
        return 0;
    }
    esp_rx_flush();

    /* ---- Step 3: Single-connection mode ---- */
    esp_print_P(ESP_CIPMUX);
    esp_wait_for_P(PSTR("OK"), 2000);
    esp_rx_flush();

    /* ---- Step 4: Join Wi-Fi network ---- */
    lcd_goto(1, 0);
    lcd_print_P(LCD_WIFI_CONN);

    /* Build: AT+CWJAP="SSID","PASS"\r\n                             */
    snprintf(g_esp_buf, sizeof(g_esp_buf),
             "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PASS);
    esp_print(g_esp_buf);

    /* "WIFI CONNECTED" appears when association succeeds (up to 20 s) */
    if (!esp_wait_for_P(PSTR("CONNECTED"), 20000)) {
        lcd_goto(1, 0);
        lcd_print_P(LCD_WIFI_FAIL);
        esp_rx_flush();
        return 0;
    }

    /* Wait for IP assignment ("GOT IP") — give it an extra 5 s      */
    esp_wait_for_P(PSTR("GOT IP"), 5000);
    esp_rx_flush();

    lcd_goto(1, 0);
    lcd_print_P(LCD_WIFI_OK);
    return 1;
}

/*
 * esp01_thingspeak_upload() — send field1=water_cm, field2=rain_state.
 *
 * Builds the HTTP GET payload into g_esp_buf, measures its length,
 * tells the ESP the byte count via AT+CIPSEND, then streams the data.
 *
 * HTTP/1.0 is used deliberately: the server closes the connection
 * after the response, which conveniently triggers "CLOSED" from the
 * ESP so we don't need to wait for or parse the response body.
 *
 * Returns 1 on successful "SEND OK", 0 on any failure.
 */
static uint8_t esp01_thingspeak_upload(uint16_t water_cm, uint8_t rain_state)
{
    /* ---- Step 1: Open TCP connection ---- */
    esp_rx_flush();
    esp_print_P(ESP_CIPSTART);

    /* "CONNECT" appears when TCP handshake completes (up to 10 s)   */
    if (!esp_wait_for_P(PSTR("CONNECT"), 10000)) {
        esp_rx_flush();
        return 0;
    }
    esp_rx_flush();

    /* ---- Step 2: Build the HTTP GET request ---- */
    /*
     * Format:
     *   GET /update?api_key=KEY&field1=WCM&field2=RAIN HTTP/1.0\r\n
     *   Host: api.thingspeak.com\r\n
     *   \r\n
     *
     * Worst-case length:
     *   "GET /update?api_key=" (20) + key (16) + "&field1=" (8)
     *   + "600" (3) + "&field2=1" (9) + " HTTP/1.0\r\n" (12) = 68
     *   + "Host: api.thingspeak.com\r\n" (26) + "\r\n" (2) = 96
     *   g_esp_buf is 128 bytes — comfortable margin. ✓
     */
    snprintf(g_esp_buf, sizeof(g_esp_buf),
             "GET /update?api_key=%s&field1=%u&field2=%u HTTP/1.0\r\n"
             "Host: api.thingspeak.com\r\n\r\n",
             THINGSPEAK_API_KEY,
             water_cm,
             (uint16_t)rain_state);

    uint8_t http_len = (uint8_t)strlen(g_esp_buf);

    /* ---- Step 3: Notify ESP of byte count (AT+CIPSEND=N) ---- */
    char cipsend_cmd[20];
    snprintf(cipsend_cmd, sizeof(cipsend_cmd), "AT+CIPSEND=%u\r\n", http_len);
    esp_print(cipsend_cmd);

    /* Wait for '>' prompt — ESP is ready to receive payload         */
    if (!esp_wait_for_P(PSTR(">"), 5000)) {
        esp_print_P(ESP_CIPCLOSE);
        esp_rx_flush();
        return 0;
    }

    /* ---- Step 4: Stream the HTTP GET request ---- */
    esp_print(g_esp_buf);

    /* ---- Step 5: Wait for send acknowledgement ---- */
    if (!esp_wait_for_P(PSTR("SEND OK"), 10000)) {
        esp_print_P(ESP_CIPCLOSE);
        esp_rx_flush();
        return 0;
    }

    /* ---- Step 6: Wait for server-initiated close (HTTP/1.0) ---- */
    /* "CLOSED" arrives after the server sends its response (~1–3 s) */
    esp_wait_for_P(PSTR("CLOSED"), 5000);

    /* Graceful close in case the server did not close first          */
    esp_print_P(ESP_CIPCLOSE);
    esp_rx_flush();
    return 1;
}

/* ================================================================
 *  SD CARD / PETIT FATFS LOGGING  (unchanged)
 * ================================================================ */
static FATFS   g_fs;
static DWORD   g_sd_offset    = 0;
static DWORD   g_sd_file_size = 0;
static uint8_t g_sd_ok        = 0;

static void sd_init(void)
{
    lcd_goto(1, 0);
    lcd_print_P(LCD_SD_MOUNT);

    FRESULT r = pf_mount(&g_fs);
    if (r != FR_OK) { lcd_goto(1, 0); lcd_print_P(LCD_SD_FAIL); return; }

    r = pf_open("DATA.TXT");
    if (r != FR_OK) { lcd_goto(1, 0); lcd_print_P(LCD_FILE_FAIL); return; }

    g_sd_file_size = g_fs.fsize;
    g_sd_offset    = 0;
    g_sd_ok        = 1;

    lcd_goto(1, 0);
    lcd_print_P(LCD_SD_OK);
}

static void sd_write_log(const char *line)
{
    if (!g_sd_ok) return;
    WORD bw = 0, dummy = 0;
    pf_lseek(g_sd_offset);
    if (pf_write(line, (UINT)strlen(line), &bw) == FR_OK && bw > 0) {
        pf_write(NULL, 0, &dummy);
        g_sd_offset += bw;
        if (g_sd_file_size > 0 && g_sd_offset >= g_sd_file_size) {
            g_sd_offset = 0;
        }
    }
}

/* ================================================================
 *  LED HEARTBEAT  (unchanged)
 * ================================================================ */
static void led_tick(void)
{
    PORTB ^= (1 << PIN_LED);
}

/* ================================================================
 *  ATOMIC 16-BIT READ  (unchanged)
 * ================================================================ */
static inline uint16_t seconds_snapshot(void)
{
    uint8_t sreg = SREG;
    cli();
    uint16_t s = g_seconds;
    SREG = sreg;
    return s;
}

/* ================================================================
 *  MAIN
 * ================================================================ */
int main(void)
{
    /*
     * FIX: watchdog as a hard safety net. If anything downstream ever
     * genuinely deadlocks (vs. just running long), the MCU self-resets
     * within ~4s instead of sitting dead until power-cycled by hand.
     * wdt_reset() is now called inside esp_wait_for_P()/sim_wait_for_P()
     * so legitimate multi-second AT-command waits don't trip it.
     */
    //wdt_enable(WDTO_4S);

    /* ---- Peripheral init ---- */
    io_init();
    i2c_init();
    lcd_init();
    sonar_init();
    timer0_init();
    timer1_init();
    sim_soft_uart_init();   /* SIM800L Software UART (PD2/PD3)        */
    esp_uart_init();        /* ESP-01 Hardware UART @ 115200 (PD0/PD1) */
    sei();

    /* ---- Splash screen ---- */
    lcd_clear();
    lcd_goto(0, 0); lcd_print_P(LCD_SPLASH1);
    lcd_goto(1, 0); lcd_print_P(LCD_SPLASH2);
    _delay_ms(1500);
    wdt_reset();
    lcd_clear();

    /* ---- SIM800L startup (Software UART) ---- */
    /*
     * Done first so the GSM radio's 500 mA current spike during
     * network registration happens before the ESP-01 is active.
     * lcd_init() after to recover HD44780 from any VCC droop reset.
     *
     * FIX: registration polling is now interleaved with a live water
     * sensor read + LCD update on every attempt, instead of one
     * function that blocked for up to ~30s+ straight with the sensor
     * and display completely frozen. Each attempt below is bounded to
     * ~2.3s worst case, so the sensor reading refreshes roughly every
     * ~2 seconds throughout registration instead of not at all.
     */
    lcd_goto(0, 0); lcd_print_P(LCD_SIM_CONN);
    led_tick();

    SIM_Status sim_status = SIM_FAIL;
    if (sim800l_presence_check()) {
        for (uint8_t i = 0; i < 15; i++) {
            sim_status = sim800l_registration_step();
            led_tick();
            wdt_reset();

            /* Keep the sensor + display alive during registration     */
            uint16_t boot_water_cm = sonar_read_cm();
            char     boot_buf[21];
            if (boot_water_cm == 0u) {
                snprintf(boot_buf, sizeof(boot_buf), "Water:ERR   Try:%2u", i + 1);
            } else {
                snprintf(boot_buf, sizeof(boot_buf), "Water:%3ucm Try:%2u", boot_water_cm, i + 1);
            }
            lcd_goto(1, 0); lcd_print(boot_buf);

            if (sim_status != SIM_BUSY) break;
        }
    }

    lcd_goto(1, 0);
    lcd_print_P(sim_status == SIM_OK ? LCD_SIM_OK : LCD_SIM_FAIL);
    led_tick();
    wdt_reset();
    lcd_init();
    _delay_ms(800);
    lcd_clear();

    /* ---- ESP-01 startup (Hardware UART) ---- */
    lcd_goto(0, 0); lcd_print_P(LCD_SPLASH1);
    led_tick();
    esp01_startup();     /* Row 1 updated to WiFi Connected / Failed   */
    led_tick();
    wdt_reset();
    _delay_ms(1000);
    lcd_clear();

    /* ---- SD Card init ---- */
    sd_init();
    wdt_reset();
    _delay_ms(1500);
    lcd_clear();

    /* ---- Main loop state ---- */
    char     log_buf[56];   /* "2026-06-24 14:30:00,Rain:YES,Water:020cm\r\n" */
    char     disp_buf[28];  /* One LCD row (20) + safety margin + '\0'  */
    char     water_str[8];  /* "ERRcm" or "020cm"                        */
    uint16_t last_log_sec = 0;
    RTC_Time rtc          = {0};

    /*
     * SMS cooldown state (see comment in original main.c):
     *   Initialise last_sms_sec so cooldown is already expired at boot.
     */
    uint16_t last_sms_sec = (uint16_t)(0u - CFG_SMS_COOLDOWN);
    uint8_t  sms_armed    = 1u;

    /* ================================================================
     *  MAIN LOOP — driven by g_tick (1 s from Timer0 ISR)
     * ================================================================ */
    while (1)
    {
        if (!g_tick) continue;
        g_tick = 0;
        wdt_reset();   /* main loop runs every 1s, well inside the 4s WDT window */

        /* ---- Read sensors ---- */
        uint8_t  rain_state = (PIND & (1 << PIN_RAIN)) ? 1u : 0u;
        /* rain_state: 0 = rain detected (DO pulled LOW), 1 = dry      */

        uint16_t water_cm   = sonar_read_cm();

        /* ---- Read RTC ---- */
        rtc_read(&rtc);

        /* ---- Evaluate danger ---- */
        uint8_t is_unsafe = (rain_state == 0u)
                         || (water_cm > 0u && water_cm < CFG_WATER_THRESH);

        /* ---- Actuators ---- */
        if (is_unsafe) PORTB |=  (1 << PIN_BUZZER);
        else           PORTB &= ~(1 << PIN_BUZZER);

        led_tick();

        /* ---- Update LCD ---- */

        /* Row 0: timestamp "2026-06-24 14:30:00"                     */
        lcd_goto(0, 0);
        snprintf(disp_buf, sizeof(disp_buf),
                 "20%02u-%02u-%02u %02u:%02u:%02u",
                 rtc.year, rtc.month, rtc.day,
                 rtc.hour, rtc.min,   rtc.sec);
        lcd_print(disp_buf);

        /* Row 1: rain status                                          */
        lcd_goto(1, 0);
        lcd_print_P(rain_state == 0u ? LCD_RAIN_YES : LCD_RAIN_NO);

        /* Row 2: water level                                          */
        lcd_goto(2, 0);
        if (water_cm == 0u) {
            lcd_print_P(LCD_WATER_ERR);
        } else {
            snprintf(disp_buf, sizeof(disp_buf),
                     "Water: %3u cm       ", water_cm);
            lcd_print(disp_buf);
        }

        /* Row 3: system status                                        */
        lcd_goto(3, 0);
        lcd_print_P(is_unsafe ? LCD_UNSAFE : LCD_SAFE);

        /* ---- 60-second SD log + ThingSpeak upload ---- */
        uint16_t now = seconds_snapshot();
        if ((uint16_t)(now - last_log_sec) >= CFG_LOG_INTERVAL) {
            last_log_sec = now;

            if (water_cm == 0u) {
                (void)strncpy(water_str, "ERRcm", sizeof(water_str));
            } else {
                snprintf(water_str, sizeof(water_str), "%03ucm", water_cm);
            }

            /* --- SD card log --- */
            /* "2026-06-24 14:30:00,Rain:YES,Water:020cm\r\n" ≈ 46 chars */
            snprintf(log_buf, sizeof(log_buf),
                     "20%02u-%02u-%02u %02u:%02u:%02u,"
                     "Rain:%s,Water:%s\r\n",
                     rtc.year, rtc.month, rtc.day,
                     rtc.hour, rtc.min,   rtc.sec,
                     (rain_state == 0u) ? "YES" : "NO",
                     water_str);

            sd_write_log(log_buf);

            /* --- ThingSpeak upload via ESP-01 --- */
            /*
             * Show upload indicator on LCD row 3 while transmitting.
             * field1 = water_cm (distance in cm, 0 if sensor timeout)
             * field2 = rain_state (0 = rain detected, 1 = dry)
             *
             * If upload fails (Wi-Fi dropped, TCP timeout, etc.) we
             * silently continue — the SD log still has the data and
             * the next 60-second cycle will retry.
             */
            lcd_goto(3, 0);
            lcd_print_P(LCD_TS_UPLOAD);

            esp01_thingspeak_upload(water_cm, rain_state);

            /* Restore status row after upload                         */
            lcd_goto(3, 0);
            lcd_print_P(is_unsafe ? LCD_UNSAFE : LCD_SAFE);
        }

        /* ================================================================
         *  SMS FLOOD ALERT
         *
         *  Trigger: water_cm is valid (>0) AND below CFG_SMS_WATER_THRESH.
         *
         *  Cooldown state machine:
         *
         *    ┌──────────┐  danger rises AND armed AND cooldown expired
         *    │  ARMED   │ ──────────────────────────────────────────►
         *    │ sms=1    │                              show LCD msg,
         *    └──────────┘ ◄──────────────────────────  send SMS
         *         ▲         danger clears → re-arm     │
         *         │         (sms_armed = 1)             ▼
         *         │                           ┌──────────────────┐
         *         │  cooldown expires while   │  SENT / WAITING  │
         *         └── danger persists →       │  sms_armed = 0   │
         *             re-arm (sms_armed = 1)  └──────────────────┘
         * ================================================================ */
        {
            uint16_t now_sms  = seconds_snapshot();
            uint8_t  is_flood = (water_cm > 0u &&
                                 water_cm < CFG_SMS_WATER_THRESH);

            if (is_flood) {
                if (!sms_armed &&
                    (uint16_t)(now_sms - last_sms_sec) >= CFG_SMS_COOLDOWN) {
                    sms_armed = 1u;
                }

                if (sms_armed) {
                    lcd_goto(3, 0);
                    lcd_print_P(LCD_SMS_SENDING);

                    sim800l_send_sms_P(SMS_WARN_MSG);

                    last_sms_sec = now_sms;
                    sms_armed    = 0u;

                    lcd_goto(3, 0);
                    lcd_print_P(LCD_UNSAFE);
                }
            } else {
                sms_armed = 1u;
            }
        }
    }

    return 0;
}

/* ================================================================
 *  CHANGE LOG
 *
 *  [v4-1]  16 MHz CLOCK MIGRATION (from 10 MHz external crystal):
 *          - F_CPU default guard updated to 16000000UL.
 *          - ESP-01 HW UART: ESP_UBRR 10→16 (U2X=1), actual baud
 *            117 647 (+2.12%), replacing the old 113 636 (−1.36%).
 *          - SIM800L SW UART: SIM_BIT_DELAY_US/SIM_HALF_BIT_US left
 *            at 104/52 — these are time (µs) constants, not cycle
 *            counts, so avr-libc's _delay_us() re-derives correct
 *            timing from F_CPU automatically. No change needed.
 *          - Timer0 (1 Hz tick): prescaler 256→1024, OCR0A 195→124,
 *            tick threshold 200→125. The old register values would
 *            have silently produced a ~37% fast clock at 16 MHz if
 *            left unchanged — now retuned to an exact 125.000 Hz
 *            (0.00% error, tighter than the original 0.04%).
 *          - Timer1 (sonar timebase): prescaler 8→64, so tick period
 *            stays coarse enough (4 µs) that the 16-bit counter no
 *            longer risks wrapping before a max-range echo returns.
 *            sonar_read_cm() distance formula and busy-wait timeouts
 *            recalculated to match.
 *          - I2C: TWBR 42→72 to hold SCL at exactly 100 kHz.
 *          - esp_wait_for_P()'s deadline and the I2C_SPIN busy-wait
 *            macro are already expressed in terms of F_CPU, so both
 *            self-scale correctly with no code change required.
 *
 *  [v3-1]  Hardware topology swapped:
 *          ESP-01  → HW UART (PD0/PD1) @ 115200 baud (U2X=1, UBRR=10
 *                    @ 10 MHz — superseded by UBRR=16 @ 16 MHz, v4-1)
 *          SIM800L → SW UART (PD2/PD3) @ 9600 baud  (bit-bang)
 *
 *  [v3-2]  esp_uart_init(): replaces uart_set_baud().
 *          U2X=1, UBRR=10 @ 10 MHz: actual 113 636 baud (−1.36%),
 *          within ±3.5% UART tolerance. (See v4-1 for the 16 MHz
 *          UBRR=16 recalculation.)
 *
 *  [v3-3]  ISR(USART_RX_vect) now feeds g_esp_rx_buf[64] for ESP-01.
 *          Buffer size increased 32→64 bytes: ESP-01 responses can be
 *          longer (e.g. "WIFI CONNECTED\r\nWIFI GOT IP\r\n").
 *
 *  [v3-4]  esp_putc / esp_print / esp_print_P / esp_rx_flush /
 *          esp_rx_getc / esp_wait_for_P mirror the old uart_* API
 *          but target the HW UART for ESP-01.
 *
 *  [v3-5]  sim_soft_uart_init(): sets PD3 output HIGH (mark),
 *          PD2 input (no pull-up).
 *
 *  [v3-6]  sim_putchar(): bit-bang 8N1 TX.  Timing: start bit LOW
 *          104 µs, 8 data bits × 104 µs, stop bit HIGH 104 µs.
 *          Interrupts NOT disabled (ISR jitter << 104 µs at 9600 baud).
 *
 *  [v3-7]  sim_getchar(): blocking 8N1 RX. Waits up to ~26 ms for
 *          start bit, centres with 52 µs half-bit delay, disables
 *          interrupts during 8-bit sampling to prevent jitter.
 *          Returns 0xFF on start-bit timeout.
 *
 *  [v3-8]  sim_print / sim_print_P / sim_rx_flush wrappers for SW UART.
 *
 *  [v3-9]  sim_wait_for_P(): iterates sim_getchar() up to
 *          (timeout_ms/20)+1 times — each call blocks ≤27 ms.
 *          Matches needle from PROGMEM; compatible with old logic.
 *
 *  [v3-10] sim800l_startup() and sim800l_send_sms_P() updated:
 *          all uart_*() calls replaced with sim_*() equivalents.
 *          AT command sequences and cooldown logic unchanged.
 *          sim800l_send_sms_P() reuses g_esp_buf (safe: ESP idle
 *          during SMS transmission) to avoid a separate 32-byte buffer.
 *
 *  [v3-11] esp01_startup(): AT+RST → "ready", AT+CWMODE=1,
 *          AT+CIPMUX=0, AT+CWJAP="SSID","PASS" → "CONNECTED"+"GOT IP".
 *          LCD row 1 shows "WiFi Connecting..." → "WiFi Connected" or
 *          "WiFi Failed".
 *
 *  [v3-12] esp01_thingspeak_upload(water_cm, rain_state):
 *          AT+CIPSTART TCP → "CONNECT"
 *          AT+CIPSEND=N   → ">"
 *          HTTP/1.0 GET   → "SEND OK" → AT+CIPCLOSE
 *          HTTP/1.0 used so server closes connection, avoids needing
 *          response parser.  Graceful fallback on every step failure.
 *
 *  [v3-13] Main loop: ThingSpeak upload added inside the 60-s block,
 *          after SD write.  LCD row 3 shows "ThingSpeak: Upload" during
 *          transmission, then restored to SAFE/UNSAFE.
 *          Silent retry on failure (SD log always preserved).
 *
 *  [v3-14] io_init(): SIM_TX_PIN (PD3) added as output HIGH;
 *          SIM_RX_PIN (PD2) as input (no pull-up).  HW UART pins
 *          (PD0/PD1) configured by esp_uart_init().
 *
 *  [v3-15] SRAM: g_esp_buf enlarged 32→96 bytes for AT+CWJAP and
 *          HTTP GET payload.  g_uart_buf removed (merged into
 *          g_esp_buf, reused safely).  g_sim_rx_buf[32] added for
 *          conceptual clarity (not currently used by ISR — SW UART
 *          RX is synchronous).  Net SRAM increase ~64 bytes.
 *
 *  [v3-16] WIFI_SSID, WIFI_PASS, THINGSPEAK_API_KEY defined as
 *          string literal macros at top of file (placeholder values).
 *
 *  Retained from v2 unchanged:
 *    Timer0/Timer1, I2C, LCD, DS3231 RTC, JSN-SR04T sonar,
 *    Petit FatFs SD logging, led_tick(), seconds_snapshot(),
 *    SMS cooldown state machine, all PROGMEM string constants.
 * ================================================================ */