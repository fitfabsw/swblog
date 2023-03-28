---
title: "ROS2 Launch 檔編程基礎"
date: 2023-03-28T08:49:00+08:00
summary: 這是此篇文章的總結
ShowToc: true
TocOpen: true
tags: ["ROS"]
author: ["Kevin Lee"]
draft: true
---

### 前言

xxx

### 1. 創建一個最簡單的 launch 檔案

假設已存在 package 名稱為 my_tutorials，先創建一個 launch 資料夾，並在裡面創建一個名為 demo.launch.py 的檔案。

```bash
my_tutorials
├── CMakeLists.txt
├── include
│   └── my_tutorials
├── launch
│   └── demo.launch.py
├── package.xml
└── src
```

#### generate_launch_description 函數

任何 launch 檔，都須定義一個 generate_launch_description 函數，並回傳一個 LaunchDescription 物件。

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
        ),
    ])
```

以上的 launch 檔案，等同於以下的指令：

```bash
$ ros2 run turtlesim turtlesim_node
```

#### Node 相關參數

修改上例中的 Node 如下，可以定義更多的參數。

```python
Node(
    package="turtlesim",
    executable="turtlesim_node",
    output="screen",
    name="sim",
    namespace="AAA",
    remappings=[("/AAA/turtle1/cmd_vel", "/AAA/turtle1/my_cmd_vel")],
)
```

以上的 launch 檔案，等同於以下的 cli 指令

```bash
$ ros2 run turtlesim turtlesim_node --ros-args -r __name:=sim -r __ns:=AAA -r /AAA/turtle1/cmd_vel:=/AAA/turtle1/
```

執行 node 和 topic 的檢查如下

```bash
$ ros2 topic list
/AAA/turtle1/color_sensor
/AAA/turtle1/my_cmd_vel
/AAA/turtle1/pose
/parameter_events
/rosout

$ ros2 node list
/AAA/sim
```

- 指定的 namespace，AAA，已順利套用到 node 和 topic
- node 已從預設的 turtlesim 改成 sim
- topic 已從預設的 /cmd_vel 改成 /my_cmd_vel

### 2. 包含多個 Node

定義多個Node也很容易

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```

以上的 launch 檔案，等同於以下的指令

```bash
$ ros2 run turtlesim turtlesim_node --ros-args -r __ns:=turtlesim1
$ ros2 run turtlesim turtlesim_node --ros-args -r __ns:=turtlesim2
$ ros2 run turtlesim mimic --ros-args -r /input/pose:=/turtlesim1/turtle1/pose -r /output/cmd_vel:=/turtlesim2/turtle1/cmd_vel
```

### 3. 包含其他 Launch 檔案

### 4. 參數傳遞: 使用 DeclareLaunchArgument 和 LaunchConfiguration

#### DeclareLaunchArgument

#### LaunchConfiguration

### References

- [Creating a launch file](https://docs.ros.org/en/galactic/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
