#include "vdb/builtins.hpp"

#include "vdb/protocol.hpp"
#include "vdb/types.hpp"

#include "vex_motor.h"
#include "vex_units.h"
#include <cstdint>
#include <string>
#include <utility>

namespace VDP {
Timestamped::Timestamped(std::string name, Part *data)
    : Record(name), timestamp(new Float(
                      "timestamp",
                      []() {
                          printf("vex time: %f\n", (float)vexSystemTimeGet() / 1000);
                          return (float)vexSystemTimeGet() / 1000;
                      }
                    )),
      data(data) {
    Record::setFields({timestamp, (PartPtr)data});
}
void Timestamped::fetch() {
    timestamp->fetch();
    data->fetch();
}

Motor::Motor(std::string name, vex::motor &motor)
    : Record(std::move(name)), mot(motor), pos(new Float("Position(deg)")), vel(new Float("velocity(dps)")),
      temp(new Uint8("Temperature(C)")), voltage(new Float("Voltage(V)")), current(new Float("Current(%)")) {
    Record::setFields({pos, vel, temp, voltage, current});
}

void Motor::fetch() {
    pos->setValue((float)mot.position(vex::rotationUnits::deg));
    vel->setValue((float)mot.velocity(vex::velocityUnits::dps));
    temp->setValue((uint8_t)mot.temperature(vex::temperatureUnits::celsius));
    voltage->setValue((float)mot.voltage(vex::voltageUnits::volt));
    current->setValue((float)mot.current(vex::percentUnits::pct));
}

Odometry::Odometry(std::string name, OdometryBase &odom)
    : Record(std::move(name)), odom(odom), X(new Float("X")), Y(new Float("Y")), ROT(new Float("Rotation")) {
    Record::setFields({X, Y, ROT});
}

void Odometry::fetch() {
    X->setValue((float)odom.get_position().x());
    Y->setValue((float)odom.get_position().y());
    ROT->setValue((float)odom.get_position().rotation().degrees());
}

PIDRecord::PIDRecord(std::string name, PID &pid)
    : Record(std::move(name)), pid(pid), P(new Float("P")), I(new Float("I")), D(new Float("D")),
      ERROR(new Float("Error")), OUTPUT(new Float("Output")), TYPE(new String("Type")) {
    Record::setFields({TYPE, P, I, D, ERROR, OUTPUT});
}

void PIDRecord::fetch() {
    P->setValue((float)pid.config.p);
    I->setValue((float)pid.config.i);
    D->setValue((float)pid.config.d);
    ERROR->setValue((float)pid.get_error());
    OUTPUT->setValue((float)pid.get_output());
    if (pid.config.error_method == PID::ANGULAR) {
        TYPE->setValue("Angular");
    } else {
        TYPE->setValue("Linear");
    }
}

} // namespace VDP
