import logging
import board
########################################################################
#
#   General options

### Logging
log_level = logging.INFO
log_format = '%(asctime)s %(levelname)s %(name)s: %(message)s'

### Server
listening_ip = "0.0.0.0"
listening_port = 8080

### Cost Estimate
kwh_rate        = 0.26  # Rate in currency_type to calculate cost to run job
currency_type   = "EUR"   # Currency Symbol to show when calculating cost to run job

########################################################################
#
#   GPIO Setup (BCM SoC Numbering Schema)
#
#   Check the RasPi docs to see where these GPIOs are
#   connected on the P1 header for your board type/rev.
#   These were tested on a Pi B Rev2 but of course you
#   can use whichever GPIO you prefer/have available.

### Outputs
gpio_LED_R = board.D6
gpio_LED_G = board.D26
gpio_LED_B = board.D5

gpio_BUTTON = board.D16

gpio_heat = board.D12  # Switches zero-cross solid-state-relay
gpio_air  = board.D13   # Switches 0-phase det. solid-state-relay

heater_invert = 1 # switches the polarity of the heater control

### Inputs

### Thermocouple Adapter selection:
max31855spi = 1 # if you use this one, you MUST reassign the default GPIO pins

### Thermocouple Connection (using bitbang interfaces)
gpio_sensor_cs1 = board.D22
gpio_sensor_cs2 = board.D27

### amount of time, in seconds, to wait between reads of the thermocouple
sensor_time_wait = .5


########################################################################
#
#   PID parameters

pid_ki = 0.1  # Integration
pid_kd = 0.4  # Derivative
pid_kp = 0.5  # Proportional


########################################################################
#
#   Time and Temperature parameters

temp_scale          = "c" # c = Celsius | f = Fahrenheit - Unit to display 
time_scale_slope    = "s" # s = Seconds | m = Minutes | h = Hours - Slope displayed in temp_scale per time_scale_slope
time_scale_profile  = "s" # s = Seconds | m = Minutes | h = Hours - Enter and view target time in time_scale_profile

