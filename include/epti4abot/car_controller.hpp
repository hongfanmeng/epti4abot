#ifndef DF_CAR_CONTROLLER
#define DF_CAR_CONTROLLER

#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>

#include "serial.hpp"
#include "unistd.h"

namespace DF
{

enum DIRECTION_enum
{
  DIRECTION_ADV,
  DIRECTION_BACK
};

class CarController
{
public:
  // const bool DIRECTION_ADV = 0;
  const bool DIRECTION_BACK = 1;

  struct Motor
  {
    DIRECTION_enum direction;
    uint8_t speed;
  };

  CarController(const std::string& deviceName, const unsigned baud) : serial_(Serial(deviceName, baud))
  {
    if (!serial_.isOpened())
    {
      throw "Connection failed";
    }
  }

  uint8_t inline rad2speed(double rad) {
	  if (rad < 1) { return 0; }
	  double ret = 0.6138 * rad * rad + 4.3103 * rad + 57.584;
	  return static_cast<uint8_t>(std::min(ret, 255.));
  }

  void setCarLeft(const DIRECTION_enum direction, double rad)
  {
	  return setCarLeft(direction, rad2speed(rad));
  }

  void setCarRight(const DIRECTION_enum direction, double rad)
  {
	  return setCarRight(direction, rad2speed(rad));
  }

  void setCarLeft(const DIRECTION_enum direction, uint8_t speed)
  {
    mLeft_.direction = direction;
    mLeft_.speed = speed;
    syncCarLeft();
  }

  void setCarRight(const DIRECTION_enum direction, uint8_t speed)
  {
    mRight_.direction = direction;
    mRight_.speed = speed;
    syncCarRight();
  }

  inline void syncCarLeft() const
  {
    std::stringstream os;
    os << "L " << mLeft_.direction << ' ' << int(mLeft_.speed);
    const std::string command = os.str();
    std::cout << command << std::endl;
    while (!serial_.sendline(command))
      usleep(10000);
  }

  inline void syncCarRight() const
  {
    std::stringstream os;
    os << "R " << mRight_.direction << ' ' << int(mRight_.speed);
    const std::string command = os.str();
    std::cout << command << std::endl;
    while (!serial_.sendline(command))
      usleep(10000);
  }

  inline void sync() const
  {
    syncCarLeft();
    syncCarRight();
  }

  struct Motor left() const
  {
    return mLeft_;
  }
  struct Motor right() const
  {
    return mRight_;
  }
  const Serial& serial() const
  {
    return serial_;
  }

private:
  const Serial serial_;
  struct Motor mLeft_, mRight_;
};

}  // namespace DF

#endif
