### Hekateros hekateros_control/MotorHealth message

string          name        # motor name
float32         temp        # motor temperature (C)
float32         voltage     # motor input voltage (Volts)
uint8           alarm       # motor or'ed alarm bits (see below)

### alarm bits 
uint8   ALARM_NONE         = 0      # no alarms
uint8   ALARM_VOLTAGE      = 1      # over/under voltage
uint8   ALARM_TEMP         = 2      # over temperature
uint8   ALARM_COMM         = 4      # serious communitcation problems
