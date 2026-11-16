
#define F_CPU 16000000UL

#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define IMU_ID_REG  0x0F
#define IMU_CTRL1   0x10
#define IMU_CTRL3   0x12
#define IMU_DATA    0x28

volatile uint32_t elapsed_ms = 0;

// 1 ms timer for timestamps
ISR(TIMER0_COMPA_vect)
{
    elapsed_ms++;
}

void timer_init(void)
{
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A = 249;
    TIMSK0 = (1 << OCIE0A);
    sei();
}

uint32_t get_ms(void)
{
    uint32_t t;
    uint8_t old_sreg = SREG;
    cli();
    t = elapsed_ms;
    SREG = old_sreg;
    return t;
}

// UART communication through the board's USB COM port
void uart_init(void)
{
    UBRR0H = 0;
    UBRR0L = 103;  // 9600 baud, 16 MHz
    UCSR0A = 0;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_print(const char *s)
{
    while (*s)
    {
        while (!(UCSR0A & (1 << UDRE0)));
        UDR0 = *s++;
    }
}

// I2C at 100 kHz
void i2c_init(void)
{
    DDRC &= ~((1 << 4) | (1 << 5));
    PORTC &= ~((1 << 4) | (1 << 5));

    TWSR0 = 0;
    TWBR0 = 72;
    TWCR0 = (1 << TWEN);
}

// Timeout prevents a disconnected sensor from freezing the code
uint8_t i2c_wait(void)
{
    uint16_t timeout = 60000;

    while (!(TWCR0 & (1 << TWINT)))
    {
        if (--timeout == 0)
            return 0;
    }
    return 1;
}

uint8_t i2c_start(uint8_t address)
{
    TWCR0 = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    if (!i2c_wait()) return 0;

    uint8_t status = TWSR0 & 0xF8;
    if (status != 0x08 && status != 0x10) return 0;

    TWDR0 = address;
    TWCR0 = (1 << TWINT) | (1 << TWEN);
    if (!i2c_wait()) return 0;

    status = TWSR0 & 0xF8;
    return (address & 1) ? (status == 0x40) : (status == 0x18);
}

uint8_t i2c_write(uint8_t data)
{
    TWDR0 = data;
    TWCR0 = (1 << TWINT) | (1 << TWEN);
    if (!i2c_wait()) return 0;

    return ((TWSR0 & 0xF8) == 0x28);
}

uint8_t i2c_read(uint8_t *data, uint8_t ack)
{
    TWCR0 = (1 << TWINT) | (1 << TWEN) |
            (ack ? (1 << TWEA) : 0);

    if (!i2c_wait()) return 0;

    uint8_t status = TWSR0 & 0xF8;
    if (status != (ack ? 0x50 : 0x58)) return 0;

    *data = TWDR0;
    return 1;
}

void i2c_stop(void)
{
    TWCR0 = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

uint8_t imu_write(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t ok = i2c_start(addr << 1);

    if (ok) ok = i2c_write(reg);
    if (ok) ok = i2c_write(value);

    i2c_stop();
    return ok;
}

uint8_t imu_read(uint8_t addr, uint8_t reg,
                 uint8_t *data, uint8_t count)
{
    uint8_t ok = i2c_start(addr << 1);

    if (ok) ok = i2c_write(reg);
    if (ok) ok = i2c_start((addr << 1) | 1);

    for (uint8_t i = 0; i < count && ok; i++)
    {
        ok = i2c_read(&data[i], i < (count - 1));
    }

    i2c_stop();
    return ok;
}

int main(void)
{
    char line[80];
    uint8_t addr = 0x6B;
    uint8_t id = 0;
    uint8_t data[6];

    uart_init();
    i2c_init();
    timer_init();
    _delay_ms(100);

    // Check the default address, then the alternate address
    if (!imu_read(addr, IMU_ID_REG, &id, 1) || id != 0x6C)
    {
        _delay_ms(5);
        addr = 0x6A;

        if (!imu_read(addr, IMU_ID_REG, &id, 1) || id != 0x6C)
        {
            uart_print("# ERROR: IMU not detected\r\n");
            while (1);
        }
    }

    // Block data update + register auto-increment
    if (!imu_write(addr, IMU_CTRL3, 0x44))
    {
        uart_print("# ERROR: IMU configuration failed\r\n");
        while (1);
    }

    // Accelerometer: 104 Hz output rate, +/-2g
    if (!imu_write(addr, IMU_CTRL1, 0x40))
    {
        uart_print("# ERROR: Accelerometer setup failed\r\n");
        while (1);
    }

    uart_print("# LSM6DSO connected\r\n");
    uart_print("time_ms,ax_mg,ay_mg,az_mg\r\n");

    uint32_t next_sample = get_ms();

    while (1)
    {
        uint32_t now = get_ms();

        // Output at 10 Hz
        if ((int32_t)(now - next_sample) >= 0)
        {
            next_sample += 100;

            if (!imu_read(addr, IMU_DATA, data, 6))
            {
                uart_print("# I2C read error\r\n");
                continue;
            }

            int16_t x = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
            int16_t y = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
            int16_t z = (int16_t)(((uint16_t)data[5] << 8) | data[4]);

            // Convert raw readings to mg
            int32_t ax = (int32_t)x * 61 / 1000;
            int32_t ay = (int32_t)y * 61 / 1000;
            int32_t az = (int32_t)z * 61 / 1000;

            snprintf(line, sizeof(line),
                     "%lu,%ld,%ld,%ld\r\n",
                     (unsigned long)now,
                     (long)ax, (long)ay, (long)az);

            uart_print(line);
        }
    }
}

