# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

# 定义函数名称为：generate_launch_description
def generate_launch_description():

    awake_node = Node(
        package="caigou_pkg",
        executable="awake_node"
        )

    stt_node = Node(
        package="caigou_pkg",
        executable="stt_node"
        )
    
    tts_node = Node(
        package="caigou_pkg",
        executable="tts_node"
        )

    bridge_node = Node(
        package="caigou_pkg",
        executable="bridge_node"
        )
    
    ai_status_judge_node = Node(
        package="caigou_pkg",
        executable="ai_status_judge_node"
        )

    realsense_node = Node(
        package="caigou_pkg",
        executable="realsense_node"
        )

    init_node = Node(
        package="caigou_pkg",
        executable="init_node"
        )

    sort_node = Node(
        package="caigou_pkg",
        executable="sort_node"
        )
    
    pickup_node = Node(
        package="caigou_pkg",
        executable="pickup_node"
        )

    waveshare_node = Node(
        package="waveshare_pkg",
        executable="waveshare_node"
        )

    database_node = Node(
        package="caigou_pkg",
        executable="database_node"
        )

    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription([awake_node, stt_node, tts_node, bridge_node, database_node, ai_status_judge_node, realsense_node, waveshare_node, init_node, sort_node, pickup_node])
    # 返回让ROS2根据launch描述执行节点
    return launch_description


