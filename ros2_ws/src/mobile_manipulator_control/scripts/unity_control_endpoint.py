#!/usr/bin/env python3
"""ROS-TCP-Endpoint with latest-only command subscriptions for this simulation.

Same wire protocol and port as upstream. Sensor publishers and service handling stay
upstream-owned; no package-cache/vendor edits or second connection are required.
"""
import rclpy
from rclpy.executors import MultiThreadedExecutor
from ros_tcp_endpoint import TcpServer
from ros_tcp_endpoint.server import SysCommands
from ros_tcp_endpoint.subscriber import RosSubscriber


class LatestCommands(SysCommands):
    def subscribe(self, topic, message_name):
        if topic not in ('/arm/command', '/cmd_vel'):
            return super().subscribe(topic, message_name)
        message_class=self.resolve_message_name(message_name)
        if message_class is None:
            self.tcp_server.send_unity_error('Unknown command message class: '+message_name)
            return
        old=self.tcp_server.subscribers_table.get(topic)
        if old is not None:
            self.tcp_server.unregister_node(old)
        subscriber=RosSubscriber(topic,message_class,self.tcp_server,queue_size=1)
        self.tcp_server.subscribers_table[topic]=subscriber
        if self.tcp_server.executor is not None:
            self.tcp_server.executor.add_node(subscriber)
        self.tcp_server.loginfo('Registered latest-only command subscription: '+topic)


def main():
    rclpy.init()
    server=TcpServer('UnityEndpoint')
    server.syscommands=LatestCommands(server)
    # Upstream sizes its pool before Unity registers topics, normally yielding one
    # worker. Keep a small fixed pool as registrations arrive dynamically.
    executor=MultiThreadedExecutor(num_threads=2)
    server.executor=executor
    executor.add_node(server)
    server.start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        server.destroy_nodes()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__': main()
