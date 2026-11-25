"""PCA9685 (I2C) 서보/ESC 출력 + 하드웨어 MUX SEL 핀. drive_record.py 와 같은 값."""

import time

MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06


class PCA9685Output:

    def __init__(self, bus=7, address=0x40, freq=300.0, steering_ch=8, throttle_ch=9,
                 mux_pin=15, mux_freq=50, mux_duty=10.0):
        from smbus2 import SMBus
        self.address = address
        self.steering_ch = steering_ch
        self.throttle_ch = throttle_ch
        self.prescale = int(round(25_000_000.0 / (4096.0 * freq) - 1.0))
        self.actual_freq = 25_000_000.0 / (4096.0 * (self.prescale + 1))
        self.bus = SMBus(bus)
        old = self.bus.read_byte_data(address, MODE1)
        awake = old & ~0x10
        self.bus.write_byte_data(address, MODE1, (awake & 0x7F) | 0x10)
        self.bus.write_byte_data(address, PRESCALE, self.prescale)
        self.bus.write_byte_data(address, MODE1, awake)
        time.sleep(0.005)
        self.bus.write_byte_data(address, MODE1, awake | 0xA1)
        time.sleep(0.01)
        self.mux_pwm = None
        if mux_pin > 0:
            import Jetson.GPIO as GPIO
            self.GPIO = GPIO
            GPIO.setmode(GPIO.BOARD)
            GPIO.setwarnings(False)
            GPIO.setup(mux_pin, GPIO.OUT)
            self.mux_pwm = GPIO.PWM(mux_pin, mux_freq)
            self.mux_pwm.start(mux_duty)  # Jetson 이 차를 제어하도록 MUX 선택

    def _write(self, channel, pulse_us):
        ticks = int(round(pulse_us * self.actual_freq * 4096 / 1_000_000))
        ticks = max(0, min(4095, ticks))
        reg = LED0_ON_L + 4 * channel
        # 4바이트 한 번에 (I2C 트랜잭션 수 줄여 지연 감소)
        self.bus.write_i2c_block_data(self.address, reg, [0, 0, ticks & 0xFF, (ticks >> 8) & 0x0F])

    def write(self, steering_us, throttle_us):
        self._write(self.steering_ch, steering_us)
        self._write(self.throttle_ch, throttle_us)

    def close(self, steering_us, neutral_us):
        try:
            self.write(steering_us, neutral_us)
        finally:
            if self.mux_pwm is not None:
                self.mux_pwm.stop()
                self.GPIO.cleanup()
            self.bus.close()


class DryRunOutput:

    def write(self, steering_us, throttle_us):
        pass

    def close(self, steering_us, neutral_us):
        pass
