#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import json
from typing import Dict, Set, List
import time
import re

class DynamicNodeHandler(Node):
    def __init__(self):
        super().__init__('dynamic_node_handler')
        
        # Store discovered nodes and their topics
        self.discovered_nodes: Dict[str, Dict] = {}
        self.active_subscriptions = {}
        
        # Callback group for services and timers
        self.callback_group = ReentrantCallbackGroup()
        
        # Create QoS profile that matches the system defaults
        self.discovery_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        # Subscribe to parameter events for node discovery
        self.param_subscriber = self.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.parameter_event_callback,
            self.discovery_qos,
            callback_group=self.callback_group
        )
        
        # Timer for node discovery
        self.discovery_timer = self.create_timer(
            5.0,  # Check every 5 seconds
            self.discover_nodes,
            callback_group=self.callback_group
        )
        
        # Create a service to get node information
        self.get_node_info_srv = self.create_service(
            GetParameters,
            '~/get_node_info',
            self.get_node_info_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Dynamic Node Handler initialized')

    def sanitize_node_name(self, node_name: str) -> str:
        """Sanitize node name to comply with ROS2 naming rules"""
        # Remove leading slash if present
        if node_name.startswith('/'):
            node_name = node_name[1:]
        # Replace invalid characters with underscore
        return re.sub(r'[^0-9a-zA-Z_]', '_', node_name)

    def parameter_event_callback(self, msg):
        """Handle parameter events for node discovery"""
        node_name = msg.node
        
        # Check if this is a new node
        if node_name not in self.discovered_nodes:
            self.get_logger().info(f'New node detected: {node_name}')
            self.discover_node_info(node_name)

    def discover_nodes(self):
        """Periodically discover new nodes"""
        try:
            # Get list of all nodes from ROS2 graph
            node_names_and_namespaces = self.get_node_names_and_namespaces()
            
            for node_name, namespace in node_names_and_namespaces:
                # Skip our own node
                if node_name == self.get_name():
                    continue
                
                # Construct full node name
                full_node_name = f'{namespace}/{node_name}' if namespace != '/' else f'/{node_name}'
                
                if full_node_name not in self.discovered_nodes:
                    self.get_logger().info(f'Discovered new node: {full_node_name}')
                    self.discover_node_info(full_node_name)
        
        except Exception as e:
            self.get_logger().error(f'Error in node discovery: {str(e)}')

    def discover_node_info(self, node_name: str):
        """Gather information about a newly discovered node"""
        try:
            # Sanitize node name for API calls
            sanitized_name = self.sanitize_node_name(node_name)
            namespace = ''  # Use empty namespace as we're using full node name
            
            # Get node's publishers and subscribers
            publishers = []
            subscribers = []
            
            try:
                publishers = self.get_publisher_names_and_types_by_node(sanitized_name, namespace)
                subscribers = self.get_subscriber_names_and_types_by_node(sanitized_name, namespace)
            except Exception as e:
                self.get_logger().warn(f'Could not get publishers/subscribers for {node_name}: {str(e)}')
            
            # Store node information
            self.discovered_nodes[node_name] = {
                'publishers': dict(publishers),
                'subscribers': dict(subscribers),
                'discovery_time': time.time()
            }
            
            # Automatically subscribe to relevant topics
            self.handle_new_node_topics(node_name, publishers)
            
            self.get_logger().info(
                f'Node info gathered for {node_name}:\n'
                f'Publishers: {json.dumps(dict(publishers), indent=2)}\n'
                f'Subscribers: {json.dumps(dict(subscribers), indent=2)}'
            )
        
        except Exception as e:
            self.get_logger().error(f'Error gathering node info for {node_name}: {str(e)}')

    def handle_new_node_topics(self, node_name: str, publishers: List):
        """Handle topics from newly discovered node"""
        for topic, msg_types in publishers:
            # Example: Subscribe to specific message types you're interested in
            if any(msg_type.endswith('sensor_msgs/msg/LaserScan') for msg_type in msg_types):
                self.create_dynamic_subscription(topic, node_name)

    def create_dynamic_subscription(self, topic: str, publisher_node: str):
        """Dynamically create a subscription to a topic"""
        if topic not in self.active_subscriptions:
            try:
                # Import message type dynamically
                msg_type = self.get_topic_message_type(topic)
                
                # Create subscription with default QoS
                sub = self.create_subscription(
                    msg_type,
                    topic,
                    lambda msg: self.dynamic_topic_callback(msg, topic),
                    10,
                    callback_group=self.callback_group
                )
                
                self.active_subscriptions[topic] = {
                    'subscription': sub,
                    'publisher_node': publisher_node
                }
                
                self.get_logger().info(f'Created subscription to {topic} from {publisher_node}')
            
            except Exception as e:
                self.get_logger().error(f'Error creating subscription to {topic}: {str(e)}')

    def dynamic_topic_callback(self, msg, topic_name: str):
        """Handle messages from dynamically created subscriptions"""
        self.get_logger().debug(f'Received message on topic {topic_name}')
        # Process message according to your needs

    def get_node_info_callback(self, request, response):
        """Service callback to get information about discovered nodes"""
        response.values = []
        for node_name, info in self.discovered_nodes.items():
            param = Parameter()
            param.name = node_name
            param.value.string_value = json.dumps(info)
            response.values.append(param)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DynamicNodeHandler()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()