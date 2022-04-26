set(COMPONENT_SRCS "main.cpp"
                   "trigger_action.c"
                   "BleParser.c"
                   "JoystickDriver.c"
                   "sdnv.c"
                   "ButtonDriver.c"
                   "ProtocolGen.c"
                   "bmp180.c"
                   "TAParser.c"
                   "Bmp180Driver.c"
                   "BuzzerDriver.c"
                   "IRRecvDriver.c"
                   "LaserDriver.c"
                   "RgbLedDriver.c")

set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()
