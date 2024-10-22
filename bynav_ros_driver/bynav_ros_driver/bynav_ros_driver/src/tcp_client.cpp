#include "bynav_ros_driver/tcp_client.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <string.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>

#include <ros/ros.h>

using std::string;
using std::to_string;
using std::vector;

namespace {
  constexpr int16_t kBufSize{1024};
  constexpr time_t kLostConnectionInterval{60}; // s
  constexpr uint16_t kMaxPortNum{65535};
  constexpr uint16_t kMinPortNum{1};

  typedef enum IpType {
    kIpv4 = 0,
    kIpv6,
    kInvalidType
  } IpType;

  IpType GetTypeOfIpAddr(const string &ip_addr) {
    if (ip_addr.empty()) {
      return kInvalidType;
    }

    if (ip_addr.find(".") != string::npos) {
      return kIpv4;
    }

    if (ip_addr.find(":") != string::npos) {
      return kIpv6;
    }

    return kInvalidType;
  }

  bool CheckIpv4(const string &ipv4_addr) {
    struct sockaddr_in sa;
    return inet_pton(AF_INET, ipv4_addr.c_str(), &(sa.sin_addr)) != 0;
  }

  bool CheckIpv6(const string &ipv6_addr) {
    struct sockaddr_in6 sa;
    return inet_pton(AF_INET6, ipv6_addr.c_str(), &(sa.sin6_addr)) != 0;
  }
}  // namespace

class Timer {
 public:
  Timer(const time_t timeout) : timeout_(timeout) {
    current_time_ = std::chrono::steady_clock::steady_clock::now();
    last_time_ = std::chrono::steady_clock::steady_clock::now();
  }
  ~Timer() {}

  bool HasExpired() {
    current_time_ = std::chrono::steady_clock::steady_clock::now();
    std::chrono::duration<time_t> time_span =
        std::chrono::duration_cast<std::chrono::duration<time_t>>(
            current_time_ - last_time_);
    if (time_span.count() >= timeout_) {
      last_time_ = std::chrono::steady_clock::steady_clock::now();
      return true;
    }

    return false;
  }
  void SetTimeout(const time_t timeout) { timeout_ = timeout; }
  time_t GetTimeout() const { return timeout_; }
  void Restart(time_t timeout = 0) {
    if (timeout != 0) {
      timeout_ = timeout;
    }

    current_time_ = std::chrono::steady_clock::steady_clock::now();
    last_time_ = std::chrono::steady_clock::steady_clock::now();
  }

 private:
  std::chrono::steady_clock::time_point current_time_;
  std::chrono::steady_clock::time_point last_time_;
  time_t timeout_;
};

static bool isValidSockStatus() {
  return errno == EWOULDBLOCK || errno == EAGAIN || errno == EINTR;
}

bool CheckIp(const string &ip_addr) {
  IpType type = GetTypeOfIpAddr(ip_addr);
  if (type == kInvalidType) {
    return false;
  }

  if (type == kIpv4) {
    return CheckIpv4(ip_addr);
  }

  return CheckIpv6(ip_addr);
}

bool CheckPort(const uint16_t port) {
  if (port > kMaxPortNum || port < kMinPortNum) {
    return false;
  }

  return true;
}

namespace bynav_ros_driver {

TcpClient::TcpClient()
    : srv_ip_(""),
      srv_port_(0),
      client_fd_(-1),
      connection_status_(ConnectionStatus::kDisconnected),
      client_mode_(kBlock) {
  timeout_timer_ = new Timer(kLostConnectionInterval);
}

TcpClient::TcpClient(const string &srv_ip, uint16_t srv_port)
    : srv_ip_(srv_ip),
      srv_port_(srv_port),
      client_fd_(-1),
      connection_status_(ConnectionStatus::kDisconnected),
      client_mode_(kBlock) {
  timeout_timer_ = new Timer(kLostConnectionInterval);
}

TcpClient::~TcpClient() {
  DisconnectFromServer();

  if (timeout_timer_) {
    delete timeout_timer_;
    timeout_timer_ = nullptr;
  }
}

int32_t TcpClient::ConnectToServer() {
  if (srv_ip_.empty() || srv_port_ == 0) {
    return -1;
  }

  return Connect();
}

int32_t TcpClient::ConnectToServer(const string &srv_ip, uint16_t srv_port) {
  if (!CheckIp(srv_ip)) {
    return -1;
  }

  if (!CheckPort(srv_port)) {
    return -1;
  }

  srv_ip_ = srv_ip;
  srv_port_ = srv_port;

  return Connect();
}

void TcpClient::DisconnectFromServer() {
  if (client_fd_ == -1) {
    std::lock_guard<std::mutex> locker(status_mutex_);
    connection_status_ = ConnectionStatus::kDisconnected;
    return;
  }

  CloseClientFd();

  std::lock_guard<std::mutex> locker(status_mutex_);
  connection_status_ = ConnectionStatus::kDisconnected;
}

vector<char> TcpClient::ReadAll() {
  vector<char> result;
  uint32_t read_bytes = 0;
  const int32_t kReadChunkSize = 4096;
  char buf[kReadChunkSize];
  int64_t read_result = 0;
  do {
    if (read_bytes + kReadChunkSize > UINT32_MAX) {
      break;
    }
    result.reserve(read_bytes + kReadChunkSize);

    read_result = RecvData(buf, kReadChunkSize);
    if (read_result > 0 || read_bytes == 0) {
      read_bytes += read_result;
    }

    if (read_result < 0) {
      break;
    }

    result.insert(result.end(), buf, buf + read_result);
  } while (read_result > 0);

  if (read_result == -1) {
    vector<char>().swap(result);
    DisconnectFromServer();
    return result;
  }

  if (read_bytes <= 0) {
    vector<char>().swap(result);
  } else {
    result.resize(read_bytes);
  }

  return result;
}

int32_t TcpClient::RecvData(char *buf, uint32_t length) {
  int32_t ret = recv(client_fd_, buf, length, 0);
  if (ret == 0) {
    return -1;
  }

  if (ret == -1) {
    if (isValidSockStatus()) {
      return 0;
    }

    return -1;
  }

  UpdateTimer();

  return ret;
}

int32_t TcpClient::SendData(const char *data, uint32_t data_len) {
  return Send(data, data_len);
}

int32_t TcpClient::SendData(const string &data) {
  return Send(data.c_str(), data.length());
}

int32_t TcpClient::SendData(const vector<char> &data) {
  char *buf = new char[data.size()];
  copy(data.begin(), data.end(), buf);
  int32_t ret = Send(buf, data.size());
  delete []buf;

  return ret;
}

bool TcpClient::HasLostConnection() const {
  if (timeout_timer_->HasExpired()) {
    timeout_timer_->Restart();
    return true;
  }

  return false;
}

bool TcpClient::WaitForConnect(time_t timeout) {
  fd_set write_fds;
  fd_set except_fds;
  FD_ZERO(&write_fds);
  FD_ZERO(&except_fds);
  FD_SET(client_fd_, &write_fds);
  FD_SET(client_fd_, &except_fds);

  struct timeval wait_time;
  wait_time.tv_sec = timeout;
  wait_time.tv_usec = 0;

  int32_t ret = select(client_fd_ + 1, nullptr, &write_fds, &except_fds, &wait_time);
  if (ret == 0) {
    goto connect_fail;
  } else if (ret < 0) {
    goto connect_fail;
  } else {
    if (FD_ISSET(client_fd_, &write_fds)) {
      int32_t error = 0;
      socklen_t len = sizeof(errno);
      if (getsockopt(client_fd_, SOL_SOCKET, SO_ERROR, &error, &len) < 0) {
        goto connect_fail;
      }

      if (error != EISCONN && error != 0) {
        goto connect_fail;
      }

      return true;
    }
  }

connect_fail:
  return false;
}

int32_t TcpClient::ConnectInBlockMode() {
  struct sockaddr_in serv_addr;
  memset(&serv_addr, 0, sizeof(serv_addr));
  inet_pton(AF_INET, srv_ip_.c_str(), &serv_addr.sin_addr.s_addr);
  serv_addr.sin_port = htons(srv_port_);
  serv_addr.sin_family = AF_INET;

  int32_t ret = connect(client_fd_, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
  if (ret == -1) {
    return -1;
  }

  struct timeval timeout;
  timeout.tv_sec = 3;
  timeout.tv_usec = 0;

  ret = setsockopt(client_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  if (ret == -1) {
    ROS_ERROR_STREAM("set config 2 error");
    return -1;
  }

  ret = setsockopt(client_fd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
  if (ret == -1) {
    ROS_ERROR_STREAM("set config 2 error");
    return -1;
  }

  std::lock_guard<std::mutex> locker(status_mutex_);
  connection_status_ = ConnectionStatus::kConnected;
  return 0;
}

int32_t TcpClient::ConnectInNonBlockMode() {
  int32_t file_flags = fcntl(client_fd_, F_GETFL);
  if (file_flags == -1) {
    return -1;
  }

  int32_t ret = fcntl(client_fd_, F_SETFL, file_flags | O_NONBLOCK);
  if (ret == -1) {
    return -1;
  }

  struct sockaddr_in serv_addr;
  memset(&serv_addr, 0, sizeof(serv_addr));
  inet_pton(AF_INET, srv_ip_.c_str(), &serv_addr.sin_addr.s_addr);
  serv_addr.sin_port = htons(srv_port_);
  serv_addr.sin_family = AF_INET;

  ret = connect(client_fd_, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
  if (ret == -1 && errno != EINPROGRESS) {
    return -1;
  }

  if (!WaitForConnect()) {
    return -1;
  }

  std::lock_guard<std::mutex> locker(status_mutex_);
  connection_status_ = ConnectionStatus::kConnected;

  return 0;
}

int32_t TcpClient::Connect() {
  DisconnectFromServer();

  client_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (client_fd_ == -1) {
    std::lock_guard<std::mutex> locker(status_mutex_);
    connection_status_ = ConnectionStatus::kDisconnected;
    return -1;
  }

  int32_t ret = 0;
  if (client_mode_ == kBlock) {
    ret = ConnectInBlockMode();
  } else {
    ret = ConnectInNonBlockMode();
  }

  UpdateTimer();
  return ret;
}

void TcpClient::CloseClientFd() {
  shutdown(client_fd_, SHUT_RDWR);
  close(client_fd_);
  client_fd_ = -1;
}

int32_t TcpClient::Send(const char *data, uint32_t data_len) {
  int32_t byte_already_send = 0;
  int32_t ret = 0;

  for (;;) {
    ret = ::send(client_fd_, data, data_len, 0);
    if (ret == 0) {
      return -1;
    }

    if (ret == -1) {
      if (isValidSockStatus()) {
        return 0;
      }

      return -1;
    }

    byte_already_send += ret;
    if (byte_already_send == data_len) {
      break;
    }

    usleep(100);
  }

  return byte_already_send;
}

void TcpClient::UpdateTimer() {
  timeout_timer_->Restart();
}

}  // namespace bynav_ros_driver
