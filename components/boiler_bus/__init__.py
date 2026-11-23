"""boiler_bus: passive decoder for a boiler's 2-wire display bus (ADC dips -> frames -> registers)."""

import esphome.codegen as cg
from esphome.components.esp32 import include_builtin_idf_component
from esphome.components import binary_sensor, sensor, text_sensor
import esphome.config_validation as cv
from esphome import pins
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_PRESSURE,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_CELSIUS,
    UNIT_HOUR,
    UNIT_SECOND,
    UNIT_VOLT,
)

CODEOWNERS = ["@krconv"]
AUTO_LOAD = ["sensor", "binary_sensor", "text_sensor"]
DEPENDENCIES = ["esp32"]

boiler_bus_ns = cg.esphome_ns.namespace("boiler_bus")
BoilerBus = boiler_bus_ns.class_("BoilerBus", cg.Component)

CONF_TX_PIN = "tx_pin"
CONF_ADC_PIN = "adc_pin"
CONF_SAMPLE_RATE = "sample_rate"
CONF_DIP_THRESHOLD_MV = "dip_threshold_mv"
CONF_DIP_DEPTH_MV = "dip_depth_mv"
CONF_AUTO_THRESHOLD = "auto_threshold"
CONF_ASSUME_HIGH_WORD_ZERO = "assume_high_word_zero"
CONF_LOG_FRAMES = "log_frames"
CONF_FAN_STALE = "fan_stale_timeout"

CAT = ENTITY_CATEGORY_DIAGNOSTIC
MEAS = STATE_CLASS_MEASUREMENT
INC = STATE_CLASS_TOTAL_INCREASING


def _s(unit=None, acc=0, dc=None, sc=MEAS, icon=None, cat=None):
    kw = dict(accuracy_decimals=acc, state_class=sc)
    if unit:
        kw["unit_of_measurement"] = unit
    if dc:
        kw["device_class"] = dc
    if icon:
        kw["icon"] = icon
    if cat:
        kw["entity_category"] = cat
    return kw


TEMP = dict(unit=UNIT_CELSIUS, acc=1, dc=DEVICE_CLASS_TEMPERATURE)

# key -> sensor_schema kwargs. Every key also needs set_<key>_sensor() in boiler_bus.h.
SENSORS = {
    # temperatures
    "supply_temperature": _s(**TEMP),
    "return_temperature": _s(**TEMP),
    "cold_in_temperature": _s(**TEMP),
    "dhw_out_temperature": _s(**TEMP),
    "exhaust_temperature": _s(**TEMP),
    "unknown_temperature": _s(**TEMP, cat=CAT),  # register 0023, meaning unresolved
    # setpoints
    "ch_setpoint": _s(**TEMP),
    "dhw_setpoint": _s(**TEMP),
    # counters
    "power_on_hours": _s(UNIT_HOUR, 0, DEVICE_CLASS_DURATION, INC),
    "ch_burner_hours": _s(UNIT_HOUR, 0, DEVICE_CLASS_DURATION, INC),
    "dhw_burner_hours": _s(UNIT_HOUR, 0, DEVICE_CLASS_DURATION, INC),
    "pump_hours": _s(UNIT_HOUR, 0, DEVICE_CLASS_DURATION, INC),
    "ch_ignitions": _s(None, 0, None, INC, "mdi:fire"),
    "dhw_counter": _s(None, 0, None, INC, "mdi:counter"),
    # status-screen registers
    "flow": _s("GPM", 3, None, MEAS, "mdi:water-pump"),
    "fan_rpm": _s("RPM", 0, None, MEAS, "mdi:fan"),
    "water_pressure": _s("psi", 0, DEVICE_CLASS_PRESSURE),
    "flame_sensor": _s(UNIT_VOLT, 1, DEVICE_CLASS_VOLTAGE),
    # gas estimate
    "gas_btu_h": _s("BTU/h", 0, None, MEAS, "mdi:fire"),
    "propane_gal_h": _s("gal/h", 3, None, MEAS, "mdi:propane-tank"),
    "propane_total": _s("gal", 3, None, INC, "mdi:propane-tank"),
    # diagnostics
    "frames_per_second": _s("frames/s", 1, None, MEAS, None, CAT),
    "frames_ok": _s(None, 0, None, INC, None, CAT),
    "crc_errors": _s(None, 0, None, INC, None, CAT),
    "malformed_frames": _s(None, 0, None, INC, None, CAT),
    "unknown_frames": _s(None, 0, None, INC, None, CAT),
    "adc_overruns": _s(None, 0, None, INC, None, CAT),
    "adc_baseline": _s("mV", 0, None, MEAS, None, CAT),
    "dip_depth": _s("mV", 0, None, MEAS, None, CAT),
    "bit_period": _s("µs", 0, None, MEAS, "mdi:timer-outline", CAT),
    "last_frame_age": _s(UNIT_SECOND, 0, DEVICE_CLASS_DURATION, MEAS, None, CAT),
    # malformed-burst breakdown: bursts that decode to fewer / more than 144 bits (the rest are bad framing bytes)
    "malformed_short": _s(None, 0, None, INC, None, CAT),
    "malformed_long": _s(None, 0, None, INC, None, CAT),
    "dip_width": _s("µs", 1, None, MEAS, "mdi:timer-outline", CAT),  # display dips, width at half depth (10 us sampler)
    "rejected_dips": _s(None, 0, None, INC, None, CAT),  # dips thrown away as too narrow / too wide
    # transmit test: depth of the dips we generate (mV at the ADC, and the equivalent on the bus)
    "tx_dip_depth": _s("mV", 0, None, MEAS, "mdi:flash", CAT),
    "tx_bursts": _s(None, 0, None, INC, "mdi:flash", CAT),  # test patterns sent since boot
    "tx_dip_depth_bus": _s(UNIT_VOLT, 2, DEVICE_CLASS_VOLTAGE, MEAS, "mdi:flash", CAT),
}

BINARY_SENSORS = {
    "pump": dict(device_class="running"),
    "ch_demand": dict(),
    "spark": dict(),
    "flame": dict(device_class="heat"),
    "dhw_demand": dict(),
    "bus_active": dict(device_class="connectivity", entity_category=CAT),
}

TEXT_SENSORS = {
    "state": dict(icon="mdi:state-machine"),
    "status_raw": dict(entity_category=CAT),
    "error_history": dict(icon="mdi:alert-circle-outline"),
    "tx_last_result": dict(icon="mdi:send-check", entity_category=CAT),
    "malformed_info": dict(icon="mdi:bug-outline", entity_category=CAT),  # latest malformed burst, at most 1 per second
    "malformed_trace": dict(icon="mdi:chart-line", entity_category=CAT),  # ADC trace around the damage, 1 hex digit/sample
    "ok_trace": dict(icon="mdi:chart-line", entity_category=CAT),  # ADC trace inside a good frame, every 10 s
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BoilerBus),
        # GPIO32: NPN that pulls the bus down. LOW unless a transmit test runs (set_tx_test_enabled / tx_read_register).
        cv.Required(CONF_TX_PIN): pins.internal_gpio_output_pin_number,
        # ADC1 pin (GPIO32-39) behind the divider.
        cv.Required(CONF_ADC_PIN): pins.internal_gpio_input_pin_number,
        cv.Optional(CONF_SAMPLE_RATE, default=100000): cv.int_range(min=20000, max=200000),
        # dip depth at the ADC pin (50k/10k divider: ~1 V on the bus -> ~170 mV)
        cv.Optional(CONF_DIP_DEPTH_MV, default=170.0): cv.float_range(min=20, max=1000),
        # with auto_threshold the threshold tracks 0.5 * measured depth (this value / depth is the fraction);
        # with auto_threshold: false it is the fixed threshold below the baseline.
        cv.Optional(CONF_DIP_THRESHOLD_MV, default=85.0): cv.float_range(min=10, max=1000),
        cv.Optional(CONF_AUTO_THRESHOLD, default=True): cv.boolean,
        # 32-bit counters: the high word (odd register) is rarely polled. true = treat an unseen high word as 0.
        cv.Optional(CONF_ASSUME_HIGH_WORD_ZERO, default=True): cv.boolean,
        cv.Optional(CONF_FAN_STALE, default="10s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_LOG_FRAMES, default=False): cv.boolean,
        **{cv.Optional(k): sensor.sensor_schema(**v) for k, v in SENSORS.items()},
        **{cv.Optional(k): binary_sensor.binary_sensor_schema(**v) for k, v in BINARY_SENSORS.items()},
        **{cv.Optional(k): text_sensor.text_sensor_schema(**v) for k, v in TEXT_SENSORS.items()},
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    include_builtin_idf_component("esp_adc")  # adc_continuous (DMA) driver
    include_builtin_idf_component("esp_driver_rmt")  # transmit tests: hardware-timed dips on GPIO32
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_tx_pin(config[CONF_TX_PIN]))
    cg.add(var.set_adc_pin(config[CONF_ADC_PIN]))
    cg.add(var.set_sample_rate(config[CONF_SAMPLE_RATE]))
    cg.add(var.set_dip_depth_mv(config[CONF_DIP_DEPTH_MV]))
    cg.add(var.set_dip_threshold_mv(config[CONF_DIP_THRESHOLD_MV]))
    cg.add(var.set_auto_threshold(config[CONF_AUTO_THRESHOLD]))
    cg.add(var.set_assume_high_word_zero(config[CONF_ASSUME_HIGH_WORD_ZERO]))
    cg.add(var.set_fan_stale_ms(config[CONF_FAN_STALE]))
    cg.add(var.set_log_frames(config[CONF_LOG_FRAMES]))

    for key in SENSORS:
        if key in config:
            s = await sensor.new_sensor(config[key])
            cg.add(getattr(var, f"set_{key}_sensor")(s))
    for key in BINARY_SENSORS:
        if key in config:
            s = await binary_sensor.new_binary_sensor(config[key])
            cg.add(getattr(var, f"set_{key}_binary_sensor")(s))
    for key in TEXT_SENSORS:
        if key in config:
            s = await text_sensor.new_text_sensor(config[key])
            cg.add(getattr(var, f"set_{key}_text_sensor")(s))
