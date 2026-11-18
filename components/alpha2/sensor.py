import esphome.codegen as cg
from esphome.components import ble_client, number, select, sensor, switch, text_sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_CURRENT,
    CONF_FLOW,
    CONF_HEAD,
    CONF_ID,
    CONF_POWER,
    CONF_SPEED,
    CONF_VOLTAGE,
    UNIT_AMPERE,
    UNIT_CUBIC_METER_PER_HOUR,
    UNIT_METER,
    UNIT_REVOLUTIONS_PER_MINUTE,
    UNIT_VOLT,
    UNIT_WATT,
)

from . import Alpha2

AUTO_LOAD = ["switch", "select", "number", "text_sensor", "alpha2_transport"]
CONF_PUMP_ENABLED = "pump_enabled"
CONF_CONTROL_MODE = "control_mode"
CONF_CP_REGULATION = "cp_regulation"
CONF_PRESSURE_SETPOINT = "pressure_setpoint"
CONF_FLOW_SETPOINT = "flow_setpoint"
CONF_CURVE_SETPOINT = "curve_setpoint"
CONF_MAX_RPM = "max_rpm"
CONF_COMMAND_STATUS = "command_status"
SETTING_SWITCHES = {
    "night_setback": "NIGHT_SETBACK",
    "air_detection": "AIR_DETECTION",
    "panel_lock": "PANEL_LOCK",
    "minimum_flow_enabled": "MIN_FLOW",
    "maximum_flow_enabled": "MAX_FLOW",
}
SETTING_NUMBERS = {
    "minimum_flow": "MIN_FLOW",
    "maximum_flow": "MAX_FLOW",
}
Alpha2Switch = cg.esphome_ns.namespace("alpha2").class_("Alpha2Switch", switch.Switch)
Alpha2ModeSelect = cg.esphome_ns.namespace("alpha2").class_("Alpha2ModeSelect", select.Select)
Alpha2RegulationSelect = cg.esphome_ns.namespace("alpha2").class_("Alpha2RegulationSelect", select.Select)
Alpha2SetpointNumber = cg.esphome_ns.namespace("alpha2").class_("Alpha2SetpointNumber", number.Number)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Alpha2),
            cv.Optional(CONF_FLOW): sensor.sensor_schema(
                unit_of_measurement=UNIT_CUBIC_METER_PER_HOUR, accuracy_decimals=2
            ),
            cv.Optional(CONF_HEAD): sensor.sensor_schema(
                unit_of_measurement=UNIT_METER, accuracy_decimals=2
            ),
            cv.Optional(CONF_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT, accuracy_decimals=2
            ),
            cv.Optional(CONF_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE, accuracy_decimals=2
            ),
            cv.Optional(CONF_SPEED): sensor.sensor_schema(
                unit_of_measurement=UNIT_REVOLUTIONS_PER_MINUTE,
                accuracy_decimals=2,
            ),
            cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT, accuracy_decimals=2
            ),
            cv.Optional(CONF_PUMP_ENABLED): switch.switch_schema(Alpha2Switch),
            **{cv.Optional(key): switch.switch_schema(Alpha2Switch) for key in SETTING_SWITCHES},
            **{cv.Optional(key): number.number_schema(
                Alpha2SetpointNumber, unit_of_measurement=UNIT_CUBIC_METER_PER_HOUR
            ) for key in SETTING_NUMBERS},
            cv.Optional(CONF_CONTROL_MODE): select.select_schema(Alpha2ModeSelect),
            cv.Optional(CONF_CP_REGULATION): select.select_schema(Alpha2RegulationSelect),
            cv.Optional(CONF_PRESSURE_SETPOINT): number.number_schema(
                Alpha2SetpointNumber, unit_of_measurement=UNIT_METER
            ),
            cv.Optional(CONF_FLOW_SETPOINT): number.number_schema(
                Alpha2SetpointNumber, unit_of_measurement=UNIT_CUBIC_METER_PER_HOUR
            ),
            cv.Optional(CONF_CURVE_SETPOINT): number.number_schema(
                Alpha2SetpointNumber, unit_of_measurement="%"
            ),
            cv.Optional(CONF_MAX_RPM, default=5837.0): cv.positive_float,
            cv.Optional(CONF_COMMAND_STATUS): text_sensor.text_sensor_schema(),
        }
    )
    .extend(ble_client.BLE_CLIENT_SCHEMA)
    .extend(cv.polling_component_schema("15s"))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await ble_client.register_ble_node(var, config)
    cg.add(var.set_max_rpm(config[CONF_MAX_RPM]))
    if status_config := config.get(CONF_COMMAND_STATUS):
        entity = await text_sensor.new_text_sensor(status_config)
        cg.add(var.set_command_status(entity))

    for key, setter in (
        (CONF_FLOW, var.set_flow_sensor),
        (CONF_HEAD, var.set_head_sensor),
        (CONF_POWER, var.set_power_sensor),
        (CONF_CURRENT, var.set_current_sensor),
        (CONF_SPEED, var.set_speed_sensor),
        (CONF_VOLTAGE, var.set_voltage_sensor),
    ):
        if sensor_config := config.get(key):
            sens = await sensor.new_sensor(sensor_config)
            cg.add(setter(sens))

    if switch_config := config.get(CONF_PUMP_ENABLED):
        pump_switch = cg.new_Pvariable(switch_config[CONF_ID])
        await switch.register_switch(pump_switch, switch_config)
        await cg.register_parented(pump_switch, config[CONF_ID])
        cg.add(var.set_pump_switch(pump_switch))

    for key, kind in SETTING_SWITCHES.items():
        if switch_config := config.get(key):
            entity = cg.new_Pvariable(switch_config[CONF_ID])
            await switch.register_switch(entity, switch_config)
            await cg.register_parented(entity, config[CONF_ID])
            expression = cg.RawExpression(f"alpha2::protocol::SettingKind::{kind}")
            cg.add(entity.set_setting_kind(expression))
            cg.add(var.set_setting_switch(expression, entity))

    for key, kind in SETTING_NUMBERS.items():
        if number_config := config.get(key):
            entity = await number.new_number(number_config, min_value=0.1, max_value=5.0, step=0.1)
            await cg.register_parented(entity, config[CONF_ID])
            expression = cg.RawExpression(f"alpha2::protocol::SettingKind::{kind}")
            cg.add(entity.set_setting_kind(expression))
            cg.add(var.set_setting_number(expression, entity))

    for key, options, setter in (
        (CONF_CONTROL_MODE,
         ["Constant pressure — Manual", "Constant pressure — AUTOADAPT",
          "Proportional pressure", "Constant curve", "Constant flow"], var.set_mode_select),
        (CONF_CP_REGULATION, ["Manual", "AUTOADAPT"], var.set_regulation_select),
    ):
        if select_config := config.get(key):
            entity = await select.new_select(select_config, options=options)
            await cg.register_parented(entity, config[CONF_ID])
            cg.add(setter(entity))

    for key, kind, minimum, maximum, step, setter in (
        (CONF_PRESSURE_SETPOINT, "PRESSURE", 0.1, 10.0, 0.1, var.set_pressure_number),
        (CONF_FLOW_SETPOINT, "FLOW", 0.1, 5.0, 0.1, var.set_flow_number),
        (CONF_CURVE_SETPOINT, "CURVE", 0.1, 100.0, 0.1, var.set_curve_number),
    ):
        if number_config := config.get(key):
            entity = await number.new_number(number_config, min_value=minimum, max_value=maximum, step=step)
            await cg.register_parented(entity, config[CONF_ID])
            cg.add(entity.set_kind(cg.RawExpression(f"alpha2::protocol::SetpointKind::{kind}")))
            cg.add(setter(entity))
