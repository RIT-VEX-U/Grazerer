#pragma once

#include "../core/include/subsystems/odometry/odometry_base.h"
#include "vdb/types.hpp"
#include <memory>

#include "vex.h"
namespace VDP {

class Timestamped : public Record {
  public:
    Timestamped(std::string name, Part *data);

    void fetch();

  private:
    std::shared_ptr<Float> timestamp;
    PartPtr data;
};

class Motor : public Record {
  public:
    Motor(std::string name, vex::motor &mot);
    void fetch() override;

  private:
    vex::motor &mot;

    std::shared_ptr<Float> pos;
    std::shared_ptr<Float> vel;
    std::shared_ptr<Uint8> temp;
    std::shared_ptr<Float> voltage;
    std::shared_ptr<Float> current;
};

class Odometry : public Record {
  public:
    Odometry(std::string name, OdometryBase &odom);
    void fetch() override;

  private:
    OdometryBase &odom;

    std::shared_ptr<Float> X;
    std::shared_ptr<Float> Y;
    std::shared_ptr<Float> ROT;
};

class PIDRecord : public Record {
  public:
    PIDRecord(std::string name, PID &pid);
    void fetch() override;

  private:
    PID &pid;

    std::shared_ptr<Float> P;
    std::shared_ptr<Float> I;
    std::shared_ptr<Float> D;
    std::shared_ptr<Float> ERROR;
    std::shared_ptr<Float> OUTPUT;
    std::shared_ptr<String> TYPE;
};
} // namespace VDP
