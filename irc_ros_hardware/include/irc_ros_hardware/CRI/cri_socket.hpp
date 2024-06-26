// Based on the TruPhysics ROS project with permission to use
// https://bitbucket.org/truphysics/igus_rebel/src/master/

#pragma once

#include <stdio.h>
#include <sys/socket.h>

#include <array>
#include <list>
#include <mutex>
#include <string>
#include <thread>

namespace irc_hardware
{
class CriSocket
{
private:
  int sock;
  std::string ip;
  int port;
  int timeout;
  std::list<std::string> unprocessedMessages;

  bool continueReceive;
  std::thread receiveThread;
  std::thread listCheckThread;
  std::mutex socketWriteLock;
  std::mutex connectionLock;
  std::mutex messageLock;
  
  std::string oldMsg;
  int maxUnprocessedMessages;
  int listCheckWaitMs;

  bool connectionNeeded;
  static const int bufferSize = 400 * 1024;

  std::array<char, bufferSize> fragmentBuffer;
  int fragmentLength;

  void MakeConnection();
  void SeparateMessages(const char *);

  void ReceiveThreadFunction();
  void ListCheckThreadFunction();

  bool IsSocketOk();

public:
  CriSocket(const std::string &, const int &, const int &);
  ~CriSocket();

  void SetIp(std::string _ip);
  void Start();
  void Stop();
  bool HasMessage();
  std::string GetMessage();
  void SendMessage(const std::string &);

  void SendUsePositionInterface(int cmdNr, bool enabled);
  void SendConfigurePositionInterface(int cmdNr, bool enabled);
  void RequestGetPositionInterface(int cmdNr);
  

  bool IsConnected();
  std::string GetIP() {return ip;};
};
}  // namespace irc_hardware
