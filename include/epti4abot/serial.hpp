// Ref: https://forums.raspberrypi.com/viewtopic.php?t=131208
#ifndef DF_SERIAL
#define DF_SERIAL

extern "C" {
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
}

#include <chrono>
#include <iostream>

using namespace std;
using namespace std::chrono;

int serialOpen(const char *device, const int baud) {
  struct termios options;
  speed_t myBaud;
  int status, fd;

  switch (baud) {
    case 50:
      myBaud = B50;
      break;
    case 75:
      myBaud = B75;
      break;
    case 110:
      myBaud = B110;
      break;
    case 134:
      myBaud = B134;
      break;
    case 150:
      myBaud = B150;
      break;
    case 200:
      myBaud = B200;
      break;
    case 300:
      myBaud = B300;
      break;
    case 600:
      myBaud = B600;
      break;
    case 1200:
      myBaud = B1200;
      break;
    case 1800:
      myBaud = B1800;
      break;
    case 2400:
      myBaud = B2400;
      break;
    case 4800:
      myBaud = B4800;
      break;
    case 9600:
      myBaud = B9600;
      break;
    case 19200:
      myBaud = B19200;
      break;
    case 38400:
      myBaud = B38400;
      break;
    case 57600:
      myBaud = B57600;
      break;
    case 115200:
      myBaud = B115200;
      break;
    case 230400:
      myBaud = B230400;
      break;
    case 460800:
      myBaud = B460800;
      break;
    case 500000:
      myBaud = B500000;
      break;
    case 576000:
      myBaud = B576000;
      break;
    case 921600:
      myBaud = B921600;
      break;
    case 1000000:
      myBaud = B1000000;
      break;
    case 1152000:
      myBaud = B1152000;
      break;
    case 1500000:
      myBaud = B1500000;
      break;
    case 2000000:
      myBaud = B2000000;
      break;
    case 2500000:
      myBaud = B2500000;
      break;
    case 3000000:
      myBaud = B3000000;
      break;
    case 3500000:
      myBaud = B3500000;
      break;
    case 4000000:
      myBaud = B4000000;
      break;

    default:
      return -2;
  }

  if ((fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
    return -1;

  fcntl(fd, F_SETFL, O_RDWR);

  // Get and modify current options:

  tcgetattr(fd, &options);

  cfmakeraw(&options);
  cfsetispeed(&options, myBaud);
  cfsetospeed(&options, myBaud);

  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;

  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 100;  // Ten seconds (100 deciseconds)

  tcsetattr(fd, TCSANOW, &options);

  ioctl(fd, TIOCMGET, &status);

  status |= TIOCM_DTR;
  status |= TIOCM_RTS;

  ioctl(fd, TIOCMSET, &status);

  usleep(1000000);  // 10mS

  return fd;
}

namespace DF {

class Serial {
 public:
  Serial(const std::string &deviceName, int baud)
      : deviceName_(deviceName), baud_(baud) {
    handle_ = serialOpen(deviceName.c_str(), baud_);
    usleep(1000000);
  }

  ~Serial() {
    if (handle_ >= 0) {
      close(handle_);
    }
  }

  /* returns if the connection is successfully opened */
  bool isOpened() const { return handle_ >= 0; }

  /**
   * send functions, return if all char is successfully sent
   */

  /**
   * Avoid using send(char) since it seems buggy.
   * If you get a bug, try to add `for (int j = 0; j < 1000000; ++j)`
   * after sending a char. Don't ask me why... it works...
   */
  bool send(const char value) const {
    if (!isOpened()) {
      return false;
    }
    unsigned rev = write(handle_, &value, 1);
    return rev == 1;
  }

  bool send(const std::string &value) const {
    if (!isOpened()) {
      return false;
    }
    unsigned rev = write(handle_, value.c_str(), value.size());
    return rev == value.size();
  }

  bool sendline(const std::string &value) const {
    if (!isOpened()) {
      return false;
    }
    return send(value + "\n");
  }

  /**
   * recv functions, return when reach EOF if force is false
   *
   * @param size, the size want to read
   * @param force, if force read enough byte before return
   *
   */
  std::string recv(const unsigned size, const bool force = false) const {
    if (!isOpened()) {
      throw "Port not opened";
    }

    unsigned rev, curRev;
    char *buf = new char[size];

    rev = 0;
    while (rev < size) {
      curRev = read(handle_, buf + rev, size - rev);
      if (curRev == 0 && !force) {
        break;
      }
      rev += curRev;
    }

    std::string data(buf, rev);
    delete[] buf;

    return data;
  }

  /**
   * @param maxSize maxSize to receive
   * @param timeout max timeout in milliseconds of reaching EOF
   */
  std::string recvUntil(const char ch, const unsigned maxSize = 1024,
                        const unsigned timeout = 0) const {
    if (!isOpened()) {
      throw "Port not opened";
    }

    unsigned rev, curRev;
    char *buf = new char[maxSize];

    rev = 0;
    auto start = high_resolution_clock::now();
    while (true) {
      curRev = read(handle_, buf + rev, maxSize - rev);
      rev += curRev;
      if (rev > 0 && buf[rev - 1] == ch) {
        break;
      }
      auto end = high_resolution_clock::now();
      if (timeout != 0 && difftime(start, end) > timeout) {
        throw "Timeout";
      }
    }

    std::string data(buf, rev);
    delete[] buf;

    return data;
  }

  std::string recvUntil(const std::string &charset,
                        const unsigned maxSize = 1024,
                        const unsigned timeout = 0) const {
    if (!isOpened()) {
      throw "Port not opened";
    }

    unsigned rev, curRev;
    char *buf = new char[maxSize];

    rev = 0;
    auto start = high_resolution_clock::now();
    while (true) {
      curRev = read(handle_, buf + rev, maxSize - rev);
      rev += curRev;
      if (rev > 0 && charset.find(buf[rev - 1]) != std::string::npos) {
        break;
      }
      auto end = high_resolution_clock::now();
      if (timeout != 0 && difftime(start, end) > timeout) {
        throw "Timeout";
      }
    }

    std::string data(buf, rev);
    delete[] buf;

    return data;
  }

  std::string recvline(const unsigned timeout = 0) const {
    return recvUntil('\n', 1024, timeout);
  }

 private:
  int handle_;
  const std::string deviceName_;
  const int baud_;

  /**
   * @brief return time difference in milliseconds
   *
   * @param start
   * @param end
   */
  long long difftime(chrono::_V2::system_clock::time_point start,
                     chrono::_V2::system_clock::time_point end) const {
    return chrono::duration_cast<chrono::milliseconds>(end - start).count();
  }
};

}  // namespace DF

#endif
