// ROS-TCP endpoint for this simulation, in C++ (replaces the Python ROS-TCP-Endpoint).
//
// Speaks the ROS-TCP-Connector v0.7.0 protocol on one TCP port (see tcp_protocol.hpp).
// Unity -> ROS topics are forwarded as serialized CDR through generic publishers, each
// from its own worker thread with a bounded queue, so the socket reader never waits on
// DDS; ROS -> Unity topics come from generic subscriptions (latest only for the arm and
// base command topics) through one sender thread. Unity uses no ROS services in this
// project; service registration is answered with an error. Accepted sockets use
// TCP_NODELAY so small command packets are flushed immediately.
// A stream that falls out of frame sync is dropped so Unity reconnects (see lost_sync).
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>

#include "mobile_manipulator_control/tcp_protocol.hpp"

namespace mmc = mobile_manipulator_control;
using json = nlohmann::json;
using Frame = std::vector<uint8_t>;

namespace
{
// Frames waiting to be written to Unity. Bounded: the oldest frame is dropped when full.
class SendQueue
{
public:
  explicit SendQueue(size_t capacity) : capacity_(capacity) {}

  void push(Frame frame)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closed_) return;
      if (frames_.size() >= capacity_) {
        frames_.pop_front();
        ++dropped_;
      }
      frames_.push_back(std::move(frame));
    }
    ready_.notify_one();
  }

  bool pop(Frame & frame)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ready_.wait(lock, [this] { return closed_ || !frames_.empty(); });
    if (frames_.empty()) return false;
    frame = std::move(frames_.front());
    frames_.pop_front();
    return true;
  }

  void close()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
    }
    ready_.notify_all();
  }

  size_t dropped() const { return dropped_; }

private:
  const size_t capacity_;
  std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<Frame> frames_;
  bool closed_ = false;
  std::atomic<size_t> dropped_{0};
};

// Publishes one Unity topic from its own thread, keeping at most `depth` pending messages.
class PublisherWorker
{
public:
  PublisherWorker(rclcpp::GenericPublisher::SharedPtr publisher, size_t depth)
  : publisher_(std::move(publisher)), depth_(std::max<size_t>(1, depth)), thread_([this] { run(); }) {}

  ~PublisherWorker()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    ready_.notify_all();
    thread_.join();
  }

  void push(std::vector<uint8_t> payload)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (pending_.size() >= depth_) pending_.pop_front();
      pending_.push_back(std::move(payload));
    }
    ready_.notify_one();
  }

private:
  void run()
  {
    for (;;) {
      std::vector<uint8_t> payload;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock, [this] { return stop_ || !pending_.empty(); });
        if (stop_) return;
        payload = std::move(pending_.front());
        pending_.pop_front();
      }
      rclcpp::SerializedMessage message(payload.size());
      auto & raw = message.get_rcl_serialized_message();
      std::memcpy(raw.buffer, payload.data(), payload.size());
      raw.buffer_length = payload.size();
      publisher_->publish(message);
    }
  }

  rclcpp::GenericPublisher::SharedPtr publisher_;
  const size_t depth_;
  std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<std::vector<uint8_t>> pending_;
  bool stop_ = false;
  std::thread thread_;
};

bool read_exactly(int fd, uint8_t * buffer, size_t size)
{
  size_t done = 0;
  while (done < size) {
    const ssize_t got = ::recv(fd, buffer + done, size - done, 0);
    if (got <= 0) return false;
    done += static_cast<size_t>(got);
  }
  return true;
}

bool write_all(int fd, const Frame & frame)
{
  size_t done = 0;
  while (done < frame.size()) {
    const ssize_t sent = ::send(fd, frame.data() + done, frame.size() - done, MSG_NOSIGNAL);
    if (sent <= 0) return false;
    done += static_cast<size_t>(sent);
  }
  return true;
}
}  // namespace

// Upper bounds used to detect a desynchronized stream rather than allocate garbage lengths.
constexpr uint32_t kMaxDestinationLength = 1024;
constexpr uint32_t kMaxPayloadLength = 64u << 20;

class UnityControlEndpoint : public rclcpp::Node
{
public:
  UnityControlEndpoint()
  : Node("UnityEndpoint"),
    ip_(declare_parameter("ROS_IP", std::string("0.0.0.0"))),
    port_(static_cast<int>(declare_parameter("ROS_TCP_PORT", 10000))),
    latest_only_(declare_parameter("latest_only_topics",
                                   std::vector<std::string>{"/arm/command", "/cmd_vel"}))
  {}

  ~UnityControlEndpoint() override { stop(); }

  // Binds the port; false if that fails.
  bool start()
  {
    listener_ = ::socket(AF_INET, SOCK_STREAM, 0);
    const int yes = 1;
    ::setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port_));
    ::inet_pton(AF_INET, ip_.c_str(), &address.sin_addr);
    if (::bind(listener_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0 ||
        ::listen(listener_, 10) != 0) {
      RCLCPP_FATAL(get_logger(), "Cannot listen on %s:%d: %s", ip_.c_str(), port_, std::strerror(errno));
      return false;
    }
    RCLCPP_INFO(get_logger(), "Control endpoint (C++) listening with TCP_NODELAY on %s:%d", ip_.c_str(), port_);
    acceptor_ = std::thread([this] { accept_loop(); });
    return true;
  }

  void stop()
  {
    if (stopping_.exchange(true)) return;
    if (listener_ >= 0) ::shutdown(listener_, SHUT_RDWR);
    if (acceptor_.joinable()) acceptor_.join();
    if (listener_ >= 0) ::close(listener_);
    end_connection();
    std::lock_guard<std::mutex> lock(entities_mutex_);
    publishers_.clear();
    subscriptions_.clear();
  }

private:
  void accept_loop()
  {
    const int yes = 1;
    while (!stopping_) {
      sockaddr_in peer{};
      socklen_t length = sizeof(peer);
      const int fd = ::accept(listener_, reinterpret_cast<sockaddr *>(&peer), &length);
      if (fd < 0) {
        if (stopping_) return;
        continue;
      }
      ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
      char text[INET_ADDRSTRLEN] = {};
      ::inet_ntop(AF_INET, &peer.sin_addr, text, sizeof(text));
      end_connection();  // Unity reconnects; the newest connection is the active one
      begin_connection(fd, text);
    }
  }

  void begin_connection(int fd, const std::string & peer)
  {
    RCLCPP_INFO(get_logger(), "Connection from %s", peer.c_str());
    auto queue = std::make_shared<SendQueue>(4096);
    queue->push(mmc::encode_command("__handshake", json{
      {"version", mmc::kProtocolVersion}, {"metadata", json{{"protocol", "ROS2"}}.dump()}}.dump()));
    {
      std::lock_guard<std::mutex> lock(connection_mutex_);
      connection_fd_ = fd;
      queue_ = queue;
    }
    sender_ = std::thread([this, fd, queue] {
      Frame frame;
      while (queue->pop(frame)) {
        if (!write_all(fd, frame)) break;
      }
      ::shutdown(fd, SHUT_RDWR);  // wakes the reader if the write side failed first
    });
    reader_ = std::thread([this, fd, queue, peer] {
      read_loop(fd);
      queue->close();
      RCLCPP_INFO(get_logger(), "Disconnected from %s", peer.c_str());
    });
  }

  void end_connection()
  {
    int fd = -1;
    std::shared_ptr<SendQueue> queue;
    {
      std::lock_guard<std::mutex> lock(connection_mutex_);
      std::swap(fd, connection_fd_);
      std::swap(queue, queue_);
    }
    if (fd >= 0) ::shutdown(fd, SHUT_RDWR);
    if (queue) queue->close();
    if (reader_.joinable()) reader_.join();
    if (sender_.joinable()) sender_.join();
    if (fd >= 0) ::close(fd);
  }

  void send_frame(Frame frame)
  {
    std::lock_guard<std::mutex> lock(connection_mutex_);
    if (queue_) queue_->push(std::move(frame));
  }

  void send_log(const char * command, const std::string & text)
  {
    // Replace invalid UTF-8 (e.g. a misread topic name) rather than throw.
    send_frame(mmc::encode_command(command, json{{"text", text}}.dump(-1, ' ', false,
                                                                     json::error_handler_t::replace)));
  }

  void error(const std::string & text)
  {
    RCLCPP_ERROR(get_logger(), "%s", text.c_str());
    send_log("__error", text);
  }

  void read_loop(int fd)
  {
    bool skip_next = false;  // the message after a service header belongs to that header
    uint8_t length_bytes[4];
    while (!stopping_) {
      if (!read_exactly(fd, length_bytes, 4)) return;
      const uint32_t destination_length = mmc::read_u32(length_bytes);
      if (destination_length > kMaxDestinationLength) {
        return lost_sync("destination length " + std::to_string(destination_length));
      }
      std::string destination(destination_length, '\0');
      if (!read_exactly(fd, reinterpret_cast<uint8_t *>(destination.data()), destination.size()) ||
          !read_exactly(fd, length_bytes, 4)) {
        return;
      }
      const uint32_t payload_length = mmc::read_u32(length_bytes);
      if (payload_length > kMaxPayloadLength) {
        return lost_sync("payload length " + std::to_string(payload_length));
      }
      std::vector<uint8_t> payload(payload_length);
      if (!read_exactly(fd, payload.data(), payload.size())) return;
      while (!destination.empty() && destination.back() == '\0') destination.pop_back();

      if (skip_next) {
        skip_next = false;
      } else if (destination.empty()) {
        // keepalive
      } else if (destination.rfind("__", 0) == 0) {
        const auto result = handle_command(destination.substr(2), mmc::command_json(payload));
        if (result == CommandResult::OutOfSync) return lost_sync("malformed command " + destination);
        skip_next = result == CommandResult::SkipNext;
      } else if (!mmc::valid_topic_name(destination)) {
        return lost_sync("destination is not a topic name");
      } else {
        std::shared_ptr<PublisherWorker> worker;
        {
          std::lock_guard<std::mutex> lock(entities_mutex_);
          const auto it = publishers_.find(destination);
          if (it != publishers_.end()) worker = it->second;
        }
        // Unity registers a publisher before sending on it, so data on an unregistered
        // topic means its registration was lost.
        if (!worker) return lost_sync("data on unregistered topic " + destination);
        worker->push(std::move(payload));
      }
    }
  }

  // Unity's stream no longer lines up with frames (see tcp_protocol.hpp). Registrations made
  // from it may be wrong or missing, so they are discarded and the connection is dropped:
  // Unity reconnects after about a second and re-registers every topic from one thread.
  void lost_sync(const std::string & reason)
  {
    RCLCPP_ERROR(get_logger(), "Stream from Unity is out of frame sync (%s); dropping the connection "
                 "and its registrations so Unity reconnects and re-registers", reason.c_str());
    std::lock_guard<std::mutex> lock(entities_mutex_);
    publishers_.clear();
    subscriptions_.clear();
  }

  enum class CommandResult { Handled, SkipNext, OutOfSync };

  CommandResult handle_command(const std::string & command, const std::string & text)
  {
    json params;
    try {
      params = text.empty() ? json::object() : json::parse(text);
    } catch (const json::exception &) {
      return CommandResult::OutOfSync;
    }
    if (!mmc::command_fits(command, params)) return CommandResult::OutOfSync;
    try {
      if (command == "subscribe") {
        subscribe(params.at("topic").get<std::string>(), params.at("message_name").get<std::string>());
      } else if (command == "publish") {
        publish(params.at("topic").get<std::string>(), params.at("message_name").get<std::string>(),
                params.value("queue_size", 10));
      } else if (command == "topic_list") {
        send_topic_list();
      } else if (command == "ros_service" || command == "unity_service") {
        error("Service " + params.value("topic", std::string()) +
              " not registered: this endpoint does not support ROS services");
      } else if (command == "request" || command == "response") {
        error("Service messages are not supported by this endpoint; ignoring the next message");
        return CommandResult::SkipNext;
      } else {
        error("Don't understand SysCommand.'__" + command + "'");
      }
    } catch (const std::exception & e) {
      error("SysCommand " + command + " failed: " + e.what());
    }
    return CommandResult::Handled;
  }

  void subscribe(const std::string & topic, const std::string & message_name)
  {
    if (topic.empty()) {
      error("Can't subscribe to a blank topic name");
      return;
    }
    const bool latest = std::find(latest_only_.begin(), latest_only_.end(), topic) != latest_only_.end();
    auto subscription = create_generic_subscription(
      topic, mmc::ros_type(message_name), rclcpp::QoS(latest ? 1 : 10),
      [this, topic](std::shared_ptr<const rclcpp::SerializedMessage> message) {
        const auto & raw = message->get_rcl_serialized_message();
        send_frame(mmc::encode_frame(topic, raw.buffer, raw.buffer_length));
      });
    std::lock_guard<std::mutex> lock(entities_mutex_);
    subscriptions_[topic] = subscription;
    RCLCPP_INFO(get_logger(), "RegisterSubscriber(%s, %s) OK%s", topic.c_str(), message_name.c_str(),
                latest ? " (latest only)" : "");
  }

  void publish(const std::string & topic, const std::string & message_name, int queue_size)
  {
    if (topic.empty()) {
      error("Can't publish to a blank topic name");
      return;
    }
    const size_t depth = static_cast<size_t>(std::max(1, queue_size));
    auto worker = std::make_shared<PublisherWorker>(
      create_generic_publisher(topic, mmc::ros_type(message_name), rclcpp::QoS(depth)), depth);
    std::lock_guard<std::mutex> lock(entities_mutex_);
    publishers_[topic] = worker;
    RCLCPP_INFO(get_logger(), "RegisterPublisher(%s, %s) OK", topic.c_str(), message_name.c_str());
  }

  void send_topic_list()
  {
    json topics = json::array(), types = json::array();
    for (const auto & [name, type_names] : get_topic_names_and_types()) {
      topics.push_back(name);
      types.push_back(type_names.empty() ? std::string() : mmc::unity_type(type_names.front()));
    }
    send_frame(mmc::encode_command("__topic_list", json{{"topics", topics}, {"types", types}}.dump()));
  }

  const std::string ip_;
  const int port_;
  const std::vector<std::string> latest_only_;
  std::atomic<bool> stopping_{false};
  int listener_ = -1;
  std::thread acceptor_, reader_, sender_;
  std::mutex connection_mutex_;
  int connection_fd_ = -1;
  std::shared_ptr<SendQueue> queue_;
  std::mutex entities_mutex_;
  std::map<std::string, std::shared_ptr<PublisherWorker>> publishers_;
  std::map<std::string, rclcpp::GenericSubscription::SharedPtr> subscriptions_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto endpoint = std::make_shared<UnityControlEndpoint>();
  if (!endpoint->start()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(endpoint);
  executor.spin();  // returns on SIGINT/SIGTERM
  endpoint->stop();
  rclcpp::shutdown();
  return 0;
}
