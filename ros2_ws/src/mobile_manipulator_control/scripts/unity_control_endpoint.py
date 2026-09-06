#!/usr/bin/env python3
"""ROS-TCP-Endpoint with latest-only command subscriptions for this simulation.

Same wire protocol and port as upstream. Sensor publishers and service handling stay
upstream-owned; no package-cache/vendor edits or second connection are required.
"""
import rclpy
import socket
from rclpy.executors import MultiThreadedExecutor
from ros_tcp_endpoint import TcpServer
from ros_tcp_endpoint.client import ClientThread
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


class ControlTcpServer(TcpServer):
    def listen_loop(self):
        # Flush small command packets promptly instead of waiting for Nagle's
        # coalescing/acknowledgement cycle at low Unity frame rates.
        with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as listener:
            listener.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            listener.bind((self.tcp_ip,self.tcp_port))
            listener.listen(self.connections)
            self.loginfo('Control endpoint listening with TCP_NODELAY on {}:{}'.format(self.tcp_ip,self.tcp_port))
            while True:
                connection,(ip,port)=listener.accept()
                connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                ClientThread(connection,self,ip,port).start()


def main():
    rclpy.init()
    server=ControlTcpServer('UnityEndpoint')
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
