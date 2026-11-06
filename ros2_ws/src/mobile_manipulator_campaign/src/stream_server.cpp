#include "mobile_manipulator_campaign/stream_server.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include "mobile_manipulator_plant/frames.hpp"

namespace mobile_manipulator_campaign
{
namespace mmp = mobile_manipulator_plant;

namespace
{
constexpr size_t kMaxControlBytes = 64u << 10;  // a Control frame is a small JSON header: a larger one is an attack or a bug
void non_blocking(int fd) { fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK); }
uint32_t le32(const char * p)
{
  uint32_t v = 0;
  for (int i = 3; i >= 0; --i) v = (v << 8) | static_cast<unsigned char>(p[i]);
  return v;
}
}  // namespace

StreamServer::StreamServer(int port, size_t backlog_cap) : cap_(backlog_cap)
{
  listen_ = socket(AF_INET, SOCK_STREAM, 0);
  if (listen_ < 0) throw std::runtime_error("StreamServer: socket failed");
  const int one = 1;
  setsockopt(listen_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(static_cast<uint16_t>(port));
  a.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(listen_, reinterpret_cast<sockaddr *>(&a), sizeof a) < 0 || listen(listen_, 1) < 0) {
    close(listen_);
    throw std::runtime_error("StreamServer: cannot listen on port " + std::to_string(port) + ": " + std::strerror(errno));
  }
  socklen_t len = sizeof a;
  getsockname(listen_, reinterpret_cast<sockaddr *>(&a), &len);
  port_ = ntohs(a.sin_port);
  non_blocking(listen_);
}

StreamServer::~StreamServer()
{
  drop_client();
  if (listen_ >= 0) close(listen_);
}

bool StreamServer::poll(int timeout_ms)
{
  pollfd fds[2];
  int n = 0;
  if (client_ < 0) fds[n++] = {listen_, POLLIN, 0};
  else fds[n++] = {client_, static_cast<short>(POLLIN | (out_.empty() ? 0 : POLLOUT)), 0};
  if (::poll(fds, n, timeout_ms) <= 0) return false;
  if (client_ < 0) {
    const int fd = accept(listen_, nullptr, nullptr);
    if (fd < 0) return false;
    non_blocking(fd);
    const int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    client_ = fd;
    return true;
  }
  if (fds[0].revents & (POLLIN | POLLHUP | POLLERR)) {
    char chunk[65536];
    for (;;) {
      const ssize_t got = recv(client_, chunk, sizeof chunk, 0);
      if (got > 0) {
        in_.append(chunk, static_cast<size_t>(got));
        if (in_.size() >= 4 && le32(in_.data()) > kMaxControlBytes) {  // announced too large: cut off, never buffered
          drop_client();
          return false;
        }
        continue;
      }
      if (got == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
        drop_client();
        return false;
      }
      break;
    }
    try {
      mmp::FrameType type{};
      nlohmann::json header;
      std::vector<std::vector<float>> blocks;
      size_t consumed = 0;
      while (mmp::decode(in_, type, header, blocks, consumed)) {
        in_.erase(0, consumed);
        if (type == mmp::FrameType::Control) controls_.push_back(std::move(header));
      }
    } catch (const std::runtime_error &) {
      drop_client();
      return false;
    }
  }
  flush();
  return false;
}

std::vector<nlohmann::json> StreamServer::take_controls() { return std::exchange(controls_, {}); }

void StreamServer::send(const std::string & frame)
{
  if (client_ < 0 || out_.size() + frame.size() > cap_) {
    ++dropped_;
    return;
  }
  out_ += frame;
  flush();
}

void StreamServer::flush()
{
  while (client_ >= 0 && !out_.empty()) {
    const ssize_t sent = ::send(client_, out_.data(), out_.size(), MSG_NOSIGNAL);
    if (sent > 0) {
      out_.erase(0, static_cast<size_t>(sent));
      continue;
    }
    if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
    drop_client();
  }
}

void StreamServer::disconnect()
{
  flush();
  drop_client();
}

void StreamServer::drop_client()
{
  if (client_ >= 0) close(client_);
  client_ = -1;
  in_.clear();
  out_.clear();
  controls_.clear();
}
}  // namespace mobile_manipulator_campaign
