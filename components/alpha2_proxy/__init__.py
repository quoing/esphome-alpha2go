import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client
from esphome.const import CONF_ID

CODEOWNERS = ["@quoing"]
DEPENDENCIES = ["ble_client", "web_server_base"]
AUTO_LOAD = ["alpha2_transport"]

alpha2_proxy_ns = cg.esphome_ns.namespace("alpha2_proxy")
Alpha2Proxy = alpha2_proxy_ns.class_("Alpha2Proxy", ble_client.BLEClientNode, cg.Component)

CONFIG_SCHEMA = cv.Schema({cv.GenerateID(): cv.declare_id(Alpha2Proxy)}).extend(
    ble_client.BLE_CLIENT_SCHEMA
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await ble_client.register_ble_node(var, config)
