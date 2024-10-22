#ifndef __TCP_CLIENT_HPP__
#define __TCP_CLIENT_HPP__

#include <mutex>
#include <string>
#include <vector>

class Timer;

namespace bynav_ros_driver {

class TcpClient {
 private:
  typedef enum ClientMode { kNonBlock = 0, kBlock } ClientMode;
  enum class ConnectionStatus { kConnected = 0, kDisconnected };

 public:
  TcpClient();
  TcpClient(const std::string &srv_ip, uint16_t srv_port);
  ~TcpClient();

  int32_t ConnectToServer();
  int32_t ConnectToServer(const std::string &srv_ip, uint16_t srv_port);
  void DisconnectFromServer();
  std::vector<char> ReadAll();
  int32_t RecvData(char *buf, uint32_t length);
  int32_t SendData(const char *data, uint32_t data_len);
  int32_t SendData(const std::string &data);
  int32_t SendData(const std::vector<char> &data);

  bool HasLostConnection() const;

  bool IsConnected() {
    std::lock_guard<std::mutex> locker(status_mutex_);
    return connection_status_ == ConnectionStatus::kConnected;
  }

  void SetNonBlockMode() { client_mode_ = kNonBlock; }

  void SetBlockMode() { client_mode_ = kBlock; }

 private:
  bool WaitForConnect(time_t timeout = 3);
  int32_t ConnectInBlockMode();
  int32_t ConnectInNonBlockMode();
  int32_t Connect();
  void CloseClientFd();
  int32_t Send(const char *data, uint32_t data_len);
  void UpdateTimer();

 private:
  uint16_t srv_port_;
  std::mutex status_mutex_;
  ConnectionStatus connection_status_;
  ClientMode client_mode_;
  int32_t client_fd_;

  std::string srv_ip_;

  Timer *timeout_timer_;
};

} // bynav_ros_driver

#endif  // __TCP_CLIENT_HPP__
