#include "system/modbus_control.h"

ModBusControl::ModBusControl(Settings *settings, ModBus *modbus, Motion *motion, State *state, Callbacks *callbacks)
        : Control(settings, motion, state, callbacks) {
    this->modbus = modbus;
}

void ModBusControl::init() {
    Control::init();
    modbus->init();
}
