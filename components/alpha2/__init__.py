import esphome.codegen as cg
from esphome import automation
from esphome.components import ble_client
import esphome.config_validation as cv
from esphome.const import CONF_DATA, CONF_ID

CODEOWNERS = ["@quoing"]
DEPENDENCIES = ["ble_client"]
AUTO_LOAD = ["switch"]

CONF_OBJECT_ID = "object_id"
CONF_SUB_ID = "sub_id"

alpha2_ns = cg.esphome_ns.namespace("alpha2")
Alpha2 = alpha2_ns.class_("Alpha2", ble_client.BLEClientNode, cg.PollingComponent)
SendRawAction = alpha2_ns.class_("SendRawAction", automation.Action)
ReadObjectAction = alpha2_ns.class_("ReadObjectAction", automation.Action)


@automation.register_action(
    "alpha2.send_raw",
    SendRawAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(Alpha2),
            cv.Required(CONF_DATA): cv.templatable(cv.string_strict),
        }
    ),
    synchronous=True,
)
async def send_raw_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    data = await cg.templatable(config[CONF_DATA], args, cg.std_string)
    cg.add(var.set_data(data))
    return var


@automation.register_action(
    "alpha2.read_object",
    ReadObjectAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(Alpha2),
            cv.Required(CONF_OBJECT_ID): cv.templatable(cv.hex_uint8_t),
            cv.Required(CONF_SUB_ID): cv.templatable(cv.hex_uint16_t),
        }
    ),
    synchronous=True,
)
async def read_object_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    object_id = await cg.templatable(config[CONF_OBJECT_ID], args, cg.uint8)
    sub_id = await cg.templatable(config[CONF_SUB_ID], args, cg.uint16)
    cg.add(var.set_object_id(object_id))
    cg.add(var.set_sub_id(sub_id))
    return var
