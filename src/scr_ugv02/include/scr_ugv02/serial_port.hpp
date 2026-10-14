// SerialPort: minimal Linux serial port for line-based JSON (one JSON object per line).
//
// Boilerplate on purpose (OS-level, written by Claude). The bridge LOGIC is Jasper's.
// You only need these four calls:
//
//   scr_ugv02::SerialPort port;
//   if (!port.open("/tmp/ugv02_bridge", 115200)) { ... port.error() ... }
//   port.write_line(R"({"T":13,"X":0.1,"Z":0.3})");     // adds '\n' for you
//   std::vector<std::string> lines; port.read_lines(lines); // non-blocking, only COMPLETE lines (bridge v1)
//
// Why it looks like this:
//  - Non-blocking: a read never waits. If it blocked, the whole node (executor) would freeze,
//    including the /cmd_vel callback and the safety timer.
//  - Line buffering: bytes can arrive split ("{\"T\":10" ... "01,...}\n"). read_lines() keeps the
//    partial piece in buffer_ until the '\n' arrives, so you never parse half a JSON.
//  - Raw mode (cfmakeraw): no echo, no line editing, no character translation by the OS.
#pragma once

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <string>
#include <vector>

namespace scr_ugv02
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort() {close();}
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // Open `path` (real: /dev/ttyAMA0 or /dev/ttyUSB0, sim: /tmp/ugv02_bridge) as raw 8N1.
  bool open(const std::string & path, int baud)
  {
    close();
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      return fail(path);
    }
    speed_t speed;
    if (!to_speed(baud, speed)) {
      error_ = "unsupported baud rate " + std::to_string(baud);
      close();
      return false;
    }
    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
      return fail("tcgetattr");
    }
    cfmakeraw(&tio);                 // 8 data bits, no parity, no echo, no translation
    tio.c_cflag |= (CLOCAL | CREAD); // ignore modem lines, enable receiver
    tio.c_cflag &= ~CSTOPB;          // 1 stop bit
    tio.c_cflag &= ~CRTSCTS;         // no hardware flow control
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
      return fail("tcsetattr");
    }
    tcflush(fd_, TCIOFLUSH);         // drop anything left over from before
    buffer_.clear();
    return true;
  }

  bool is_open() const {return fd_ >= 0;}

  // Write one line. '\n' is appended. Returns false on error (see error()).
  bool write_line(const std::string & line)
  {
    if (fd_ < 0) {
      error_ = "port not open";
      return false;
    }
    const std::string data = line + "\n";
    size_t sent = 0;
    while (sent < data.size()) {
      const ssize_t n = ::write(fd_, data.data() + sent, data.size() - sent);
      if (n < 0) {
        if (errno == EINTR) {continue;}
        return fail("write");        // EAGAIN here means the other side is not reading
      }
      sent += static_cast<size_t>(n);
    }
    return true;
  }

  // Append every COMPLETE line received so far to `out` (without '\n' / '\r').
  // Never waits. Returns false on a hard error (e.g. cable unplugged / socat stopped).
  bool read_lines(std::vector<std::string> & out)
  {
    if (fd_ < 0) {
      error_ = "port not open";
      return false;
    }
    char chunk[256];
    while (true) {
      const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
      if (n > 0) {
        buffer_.append(chunk, static_cast<size_t>(n));
        continue;
      }
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {break;}   // no more data now
      if (n < 0 && errno == EINTR) {continue;}
      return fail("read");           // n == 0 (EOF) or real error
    }
    size_t pos;
    while ((pos = buffer_.find('\n')) != std::string::npos) {
      std::string line = buffer_.substr(0, pos);
      buffer_.erase(0, pos + 1);
      if (!line.empty() && line.back() == '\r') {line.pop_back();}
      if (!line.empty()) {out.push_back(line);}
    }
    if (buffer_.size() > 4096) {     // no newline for a long time -> garbage, start over
      buffer_.clear();
    }
    return true;
  }

  void close()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  const std::string & error() const {return error_;}

private:
  bool fail(const std::string & what)
  {
    error_ = what + ": " + std::strerror(errno);
    close();
    return false;
  }

  static bool to_speed(int baud, speed_t & out)
  {
    switch (baud) {
      case 9600: out = B9600; return true;
      case 115200: out = B115200; return true;   // UGV02 default
      case 460800: out = B460800; return true;   // RPLIDAR C1 uses this (its own driver, not us)
      case 921600: out = B921600; return true;
      default: return false;
    }
  }

  int fd_{-1};
  std::string buffer_;
  std::string error_;
};

}  // namespace scr_ugv02
