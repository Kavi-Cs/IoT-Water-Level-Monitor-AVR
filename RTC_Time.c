
#ifndef F_CPU
#  define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#define SET_YEAR     26u   /* 2026 (DS3231 stores 2-digit year)      */
#define SET_MONTH     7u   /* July                                    */
#define SET_DATE     14u   /* 14th                                    */
#define SET_HOUR     10u   /* 24-hour format                         */
#define SET_MIN      30u
#define SET_SEC       0u

#define SET_WEEKDAY   3u
#define ADDR_DS3231   0x68
#define PIN_LED       PB0   

static void twi_init(void)
{
    TWSR = 0x00;      /* prescaler = 1                                */
    TWBR = 72;
    TWCR = (1 << TWEN);
}

#define TWI_SPIN_LIMIT  20000u

static uint8_t twi_wait(void)
{
    uint16_t spins = 0;
    while (!(TWCR & (1 << TWINT))) {
        if (++spins >= TWI_SPIN_LIMIT) return 0;   /* timeout -> fail */
    }
    return 1;                                       /* TWINT set -> ok */
}

static uint8_t twi_status(void)
{
    return TWSR & 0xF8;
}

static uint8_t twi_start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    if (!twi_wait()) return 0xFF;
    uint8_t st = twi_status();
    return (st == 0x08 || st == 0x10) ? 1 : 0;  
}

static void twi_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    _delay_us(100);
}

static uint8_t twi_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (!twi_wait()) return 0;
    uint8_t st = twi_status();
    return (st == 0x18 || st == 0x28) ? 1 : 0;   
}

static void blink_forever(uint8_t fast)
{
    DDRB |= (1 << PIN_LED);
    for (;;) {
        PORTB ^= (1 << PIN_LED);
        if (fast) _delay_ms(100);   /* ~10 Hz -> write FAILED         */
        else      _delay_ms(500);   /* ~1 Hz  -> write SUCCEEDED      */
    }
}

static uint8_t bin2bcd(uint8_t v) { return (uint8_t)(((v / 10u) << 4) | (v % 10u)); }

int main(void)
{
    twi_init();

    uint8_t ok = 1;
    ok &= twi_start();
    ok &= twi_write((ADDR_DS3231 << 1) | 0x00);   
    ok &= twi_write(0x00);                        

    ok &= twi_write(bin2bcd(SET_SEC));
    ok &= twi_write(bin2bcd(SET_MIN));
    ok &= twi_write(bin2bcd(SET_HOUR));            
    ok &= twi_write(bin2bcd(SET_WEEKDAY));
    ok &= twi_write(bin2bcd(SET_DATE));
    ok &= twi_write(bin2bcd(SET_MONTH));
    ok &= twi_write(bin2bcd(SET_YEAR));

    twi_stop();

    if (ok) {
        ok &= twi_start();
        ok &= twi_write((ADDR_DS3231 << 1) | 0x00);
        ok &= twi_write(0x0F);
        ok &= twi_write(0x00);   
        twi_stop();
    }

    blink_forever(ok ? 0 : 1);   
    return 0;
}
