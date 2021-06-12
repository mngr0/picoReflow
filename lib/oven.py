from statemachine import StateMachine, State
import pwmio
import adafruit_max31855
import digitalio
import board
import threading
import time
#import random
#import _strptime
from datetime import datetime
import logging
import json

import config

log = logging.getLogger(__name__)


try:
    if config.max31855spi:
        log.info("import MAX31855SPI")
        spi_reserved_gpio = [7, 8, 9, 10, 11]
        if config.gpio_air in spi_reserved_gpio:
            raise Exception("gpio_air pin %s collides with SPI pins %s" % (
                config.gpio_air, spi_reserved_gpio))
        if config.gpio_heat in spi_reserved_gpio:
            raise Exception("gpio_heat pin %s collides with SPI pins %s" % (
                config.gpio_heat, spi_reserved_gpio))
    sensor_available = True
except ImportError:
    log.exception(
        "Could not initialize temperature sensor, using dummy values!")
    sensor_available = False

try:
    #heat = digitalio.DigitalInOut(config.gpio_heat)
    #heat.direction = digitalio.Direction.OUTPUT
    #heat.value = False
    heat = pwmio.PWMOut(board.D12, frequency=50, duty_cycle=0)

    air = digitalio.DigitalInOut(config.gpio_air)
    air.direction = digitalio.Direction.OUTPUT
    air.value = False
    ledR = digitalio.DigitalInOut(config.gpio_LED_R)
    ledR.direction = digitalio.Direction.OUTPUT
    ledR.value = True
    ledG = digitalio.DigitalInOut(config.gpio_LED_G)
    ledG.direction = digitalio.Direction.OUTPUT
    ledG.value = True
    ledB = digitalio.DigitalInOut(config.gpio_LED_B)
    ledB.direction = digitalio.Direction.OUTPUT
    ledB.value = True

    button = digitalio.DigitalInOut(config.gpio_BUTTON)
    ledB.direction = digitalio.Direction.INPUT

except ImportError:
    msg = "Could not initialize GPIOs!"
    log.warning(msg)
    exit(1)


class OvenMachine(StateMachine):
    idle = State('idle', initial=True)
    reaching_base_temp = State('reaching_base_temp')
    reaching_heat_temp = State('reaching_heat_temp')
    reaching_peak_temp = State('reaching_peak_temp')
    doing_peak = State('doing_peak')
    cooling = State('cooling')

    start = idle.to(reaching_base_temp)
    reached_base_temp = reaching_base_temp.to(reaching_heat_temp)
    reached_heat_temp = reaching_heat_temp.to(reaching_peak_temp)
    reached_peak_temp = reaching_peak_temp.to(doing_peak)
    peak_done = doing_peak.to(cooling)
    cooling_done = cooling.to(idle)

    reset = idle.to.itself() | reaching_base_temp.to(idle) | reaching_heat_temp.to(
        idle) | reaching_peak_temp.to(idle) | doing_peak.to(idle)

    def on_start(self):
        print("start!!")

    def on_reached_base_temp(self):
        print("reached base temp!")

    def on_reached_heat_temp(self):
        print("reached heat temp!")

    def on_reached_peak_temp(self):
        print("reached peak temp!")

    def on_peak_done(self):
        print("peak done!")

    def on_cooling_done(self):
        print("cooling done!")

    def on_reset(self):
        print("reset!")


class OvenController:
    def __init__(self):
        self.time_stamp = datetime.now()
        self.profile = None
        self.start_time = None
        self.oven = OvenMachine()

    def set_profile(self, profile):
        self.profile = profile

    def start_run(self):
        self.start_time = datetime.now()
        self.time_stamp = datetime.now()
        self.oven.start()

    def get_target_temperature(self, current_temp):

        print("get_target", self.oven)
        target_temp = 0
        if self.oven.is_idle:
            target_temp = min(self.profile.conf["base_temp"], current_temp)

        elif self.oven.is_reaching_base_temp:
            target_temp = self.profile.conf["base_temp"]
            if current_temp >= self.profile.conf["base_temp"]:
                self.time_stamp = datetime.now()
                self.oven.reached_base_temp()

        elif self.oven.is_reaching_heat_temp:
            commanded_temp = (datetime.now() -
                              self.time_stamp).total_seconds() * self.profile.conf["heat_ramp"]
            target_temp = max(current_temp, commanded_temp)
            target_temp = min(target_temp, commanded_temp+10)
            if current_temp >= self.profile.conf["heat_temp"]:
                self.time_stamp = datetime.now()
                self.oven.reached_heat_temp()

        elif self.oven.is_reaching_peak_temp:
            commanded_temp = (datetime.now() -
                              self.time_stamp).total_seconds() * self.profile.conf["peak_ramp"]
            target_temp = max(current_temp, commanded_temp)
            target_temp = min(target_temp, commanded_temp+5)
            if current_temp >= self.profile.conf["peak_temp"]:
                self.time_stamp = datetime.now()
                self.oven.reached_peak_temp()

        elif self.oven.is_doing_peak:
            target_temp = self.profile.conf["peak_temp"]
            if (datetime.now() - self.time_stamp).total_seconds() > self.profile.conf["peak_time"]:
                self.time_stamp = datetime.now()
                self.oven.peak_done()

        elif self.oven.is_cooling:
            commanded_temp = (datetime.now() -
                              self.time_stamp).total_seconds() * self.profile.conf["cool_ramp"]
            target_temp = min(current_temp-5, commanded_temp)
            if current_temp <= self.profile.conf["cool_temp"]:
                self.oven.cooling_done()

        if current_temp >= self.profile.conf["limit_temp"]:
            target_temp = target_temp-15
        print("target=", target_temp)
        return target_temp


class Oven (threading.Thread):

    def __init__(self, time_step=config.sensor_time_wait):
        threading.Thread.__init__(self)
        self.daemon = True
        self.heat_pin = heat
        self.air_pin = air
        self.ledR_pin = ledR
        self.ledG_pin = ledG
        self.ledB_pin = ledB
        self.button_pin = button
        self.runtime = 0
        self.target = 0
        self.oven_controller = OvenController()
        self.time_step = time_step
        self.set_heat(False)
        self.pid = PID(ki=config.pid_ki, kd=config.pid_kd, kp=config.pid_kp)

        # self.reset()
        self.temp_sensor = TempSensorReal(self.time_step)
        self.temp_sensor.start()
        self.start()

    def reset(self):
        self.oven_controller.oven.reset()
        self.set_heat(False)
        self.pid = PID(ki=config.pid_ki, kd=config.pid_kd, kp=config.pid_kp)

    def set_profile(self, profile):
        log.info("configuring profile %s" % profile.name)
        self.oven_controller.set_profile(profile)

    def start_run(self):
        log.info("Running profile %s" % self.oven_controller.profile.name)
        self.oven_controller.start_run()
        self.start_time = datetime.now()
        log.info("Starting")

    def abort_run(self):
        self.reset()

    def status_LED(self):
        if self.air:
            self.ledB_pin.value = True
        else:
            self.ledB_pin.value = False

        if self.heating:
            self.ledR_pin.value = True
        else:
            self.ledR_pin.value = False

    def run(self):
        # temperature_count = 0
        # last_temp = 0
        pid = 0
        button_state = self.button_pin.value
        button_start_press = datetime.now()
        while True:
            # if button pressed start run
            new_button_state = self.button_pin.value
            if self.button_pin.value == 1:
                if button_state == 0:
                    button_start_press = datetime.now()
                if (datetime.now() - button_start_press).total_seconds() > 1:
                    pass
                    # start
            button_state = new_button_state
            # if long press, stop run
            #log.info("current temp %.1f deg C " %
            #        (self.temp_sensor.temperature))
            if not self.oven_controller.oven.is_idle:
                self.runtime = (
                    datetime.now() - self.oven_controller.start_time).total_seconds()
                log.info("running at %.1f deg C (Target: %.1f) , heat %.2f, air %.2f (%.1fs)" % (
                    self.temp_sensor.temperature, self.target, self.heating, self.air, self.runtime))

                self.target = self.oven_controller.get_target_temperature(
                    self.temp_sensor.temperature)

                pid = self.pid.compute(
                    self.target, self.temp_sensor.temperature)

                log.info("pid: %.3f" % pid)

                self.set_heat(pid)
                time.sleep(1)

                if self.temp_sensor.temperature > self.target+5:
                    self.set_air(True)
                else:
                    self.set_air(False)
            else:
                if self.oven_controller.profile is not None:
                    if self.temp_sensor.temperature > self.oven_controller.profile.conf["cool_temp"]:
                        self.set_air(False)
                    else:
                        self.set_air(False)
                else:
                    if self.temp_sensor.temperature > 80:
                        self.set_air(True)
                    else:
                        self.set_air(False)
                self.set_heat(0)

    def set_heat(self, value):
        # todo use pwmio
        if value > 0:

            self.heating = 1.0
            # self.heat_pin.duty_cycle = 65535*( (1-config.heater_invert) value)

            #self.heat_pin.duty_cycle = 65535*(1-value)
            self.heat_pin.duty_cycle = 65535*(value)

            #self.heat_pin.value = True
            #time.sleep(self.time_step * value)
            #self.heat_pin.value = False
            print("PWM TO ",self.heat_pin.duty_cycle)

        else:
            self.heating = 0.0
            self.heat_pin.duty_cycle = 0
            #self.heat_pin.duty_cycle = 65535
            #self.heat_pin.value = False
            #self.heat_pin.value = True

    def set_air(self, value):
        # todo: add PID, which start full, then slows down
        if value:
            self.air = 1.0
            self.air_pin.value = True
        else:
            self.air = 0.0
            self.air_pin.value = False

    def get_state(self):
        print(self.oven_controller.oven.current_state.value)
        state = {
            'runtime': self.runtime,
            'temperature': self.temp_sensor.temperature,
            'target': self.target,
            'state': "IDLE" if self.oven_controller.oven.current_state.value is not "idle" else "RUNNING" ,
            'heat': self.heating,
            'air': self.air,
            'totaltime': 900
        }
        return state


class TempSensor(threading.Thread):
    def __init__(self, time_step):
        threading.Thread.__init__(self)
        self.daemon = True
        self.temperature = 0
        self.time_step = time_step


class TempSensorReal(TempSensor):
    def __init__(self, time_step):
        TempSensor.__init__(self, time_step)

        log.info("init MAX31855-spi")
        cs1 = digitalio.DigitalInOut(config.gpio_sensor_cs1)
        cs1.direction = digitalio.Direction.OUTPUT
        cs1.value = True
        cs2 = digitalio.DigitalInOut(config.gpio_sensor_cs2)
        cs2.direction = digitalio.Direction.OUTPUT
        cs2.value = True
        self.thermocouple1 = adafruit_max31855.MAX31855(board.SPI(), cs1)
        self.thermocouple2 = adafruit_max31855.MAX31855(board.SPI(), cs2)
        self.t1 = None
        self.t2 = None

    def run(self):
        while True:
            try:
                self.t1 = 0
                self.t2 = 0
                try:
                    self.t2 = self.thermocouple2.temperature
                except RuntimeError:
                    self.t2 = None
                    log.exception("T2 READING ERROR")
                try:
                    self.t1 = self.thermocouple1.temperature
                except Exception:
                    self.t1 = None
                    log.exception("T1 READING ERROR")
                if self.t1 is None and self.t2 is None:
                    self.temperature = self.temperature
                elif self.t1 is None and self.t2 is not None:
                    self.temperature = self.t2
                elif self.t1 is not None and self.t2 is None:
                    self.temperature = self.t1
                else:
                    self.temperature = (self.t2 + self.t1) / 2
            except Exception:
                log.exception("problem reading temp")
            time.sleep(self.time_step)


class Profile():
    def __init__(self, json_data):
        obj = json.loads(json_data)
        self.name = obj["name"]
        self.conf1 = {
            "base_temp": 80,
            "heat_temp": 180,
            "heat_ramp": 1.5,  # C/s
            "time_above_melting_point": 90,  # s
            "melting_point": 217,
            "peak_temp": 230,
            "limit_temp": 260,
            "peak_ramp": 2,  # C/s
            "peak_time": 30,  # s
            "cool_temp": 50,
            "cool_ramp": -2,  # C/s
            "interval_targets": 4
        }

        self.conf2 = {
            "base_temp": 30,
            "heat_temp": 50,
            "heat_ramp": 1.5,  # C/s
            "time_above_melting_point": 90,  # s
            "melting_point": 217,
            "peak_temp": 100,
            "limit_temp": 150,
            "peak_ramp": 2,  # C/s
            "peak_time": 30,  # s
            "cool_temp": 50,
            "cool_ramp": -2  # C/s
        }

        self.conf = self.conf2


class Report():
    def __init__(self, json_data):
        obj = json.loads(json_data)
        self.name = obj["name"]
        self.time_over_melting_point = 0
        self.peak_time = 0


class PID():
    def __init__(self, ki=1, kp=1, kd=1):
        self.ki = ki
        self.kp = kp
        self.kd = kd
        self.lastNow = datetime.now()
        self.iterm = 0
        self.lastErr = 0

    def compute(self, setpoint, ispoint):
        now = datetime.now()
        timeDelta = (now - self.lastNow).total_seconds()

        error = float(setpoint - ispoint)
        self.iterm += (error * timeDelta * self.ki)
        self.iterm = sorted([-1, self.iterm, 1])[1]
        dErr = (error - self.lastErr) / timeDelta

        output = self.kp * error + self.iterm + self.kd * dErr
        output = sorted([-1, output, 1])[1]
        self.lastErr = error
        self.lastNow = now

        return output
