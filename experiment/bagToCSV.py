"""Modified from https://mcap.dev/docs/python/ros2_example"""

"""script that reads ROS2 messages from an MCAP bag using the rosbag2_py API."""
import dataclasses
import random
import re

import pandas

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import dhtt_msgs.msg


@dataclasses.dataclass
class ReducedNodeMsg:
    timestamp: pandas.Timestamp
    id: int
    node_name: str
    type: str
    activation_potential: float
    num_resources: int


def read_messages(input_bag: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp
    del reader


def get_message_generator(mcap_file: str):
    # [0] is str str topic, [1] the actual message, [2] uuid?
    return read_messages(mcap_file)


def filter_messages_by_topic(message_generator, topic: str):
    # topic should include the forward slash (i.e., /rosout)
    return (x for x in message_generator if x[0] == topic)


def filter_status_by_node_names(message_generator, node_names: set[str]):
    """
    This is probably the function you want.

    Provide empty set to get all nodes. Case sensitive. Works with or without the nodename_xx number suffix
    """
    gen = filter_messages_by_topic(message_generator, "/status")

    match_without_number = False
    if not all((re.search(r".+_[0-9]+$", x) for x in node_names)):
        # interpret node names without the number at the end
        match_without_number = True

    for msg in gen:
        if type(msg[1]) is dhtt_msgs.msg.Node:
            pred: bool = msg[1].node_name.rsplit('_', 1)[0] in node_names if match_without_number else msg[
                                                                                                           1].node_name in node_names
            if len(node_names) == 0 or pred:
                ret: ReducedNodeMsg

                timestamp = pandas.Timestamp(msg[1].head.stamp.sec * 1_000_000_000 + msg[1].head.stamp.nanosec,
                                             unit='ns')
                id = msg[2]
                node_name = msg[1].node_name
                ttype = msg[1].type
                num_resources = random.randint(0, 5)  # TODO
                activation_potential = 0.1 * random.randint(0, 10)  # TODO

                ret = ReducedNodeMsg(timestamp, id, node_name, ttype, activation_potential, num_resources)

                yield ret
        pass  # skip
