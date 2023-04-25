#ifndef OPEN_PNP_HEAD_MODBUS_CONTROL_H
#define OPEN_PNP_HEAD_MODBUS_CONTROL_H

#include "control.h"

class ModBusControl : public Control {
private:
    ModBus          *modbus;

public:
    explicit ModBusControl(Settings *settings, ModBus *modbus, Motion *motion, State *state, Callbacks *callbacks);
    void init() override;
};

#endif //OPEN_PNP_HEAD_MODBUS_CONTROL_H
