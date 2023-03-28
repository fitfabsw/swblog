---
title: "ROS2 Launch File 編程基礎"
date: 2023-03-28T08:49:00+08:00
summary: 三十分鐘學習launch file
ShowToc: true
TocOpen: true
tags: ["ROS"]
author: ["Kevin Lee"]
draft: true
---

### 前言

ROS2 launch 文件是機器人應用的強大工具。它們允許您用一個命令啟動多個節點，並在啟動序列中添加邏輯，而不是手動啟動每個進程。
本教程將解釋您需要了解的有關 ROS2 launch 文件的所有基本信息。

本教程將從創建一個基本的 launch 文件開始，接著介紹如何包含多個節點、包含其他 launch 文件，並說明如何使用參數傳遞。
本文將以實際操作為基礎，幫助您快速掌握ROS2 launch 文件的編程基礎。

<!-- 然後，我們將更詳細地了解啟動操作、事件處理器、替換和條件。 -->
<!-- 之後，我們將討論 ROS2 啟動文件的常見問題和用例，並在文章的最後，您可以找到一個包含所有常用操作、事件處理器和替換的列表，供您在啟動文件中使用。 -->

聽起來不錯？那我們開始吧！

### 1. 創建一個最簡單的 launch 文件

假設已存在名為 my_tutorials 的 package，先創建一個 launch 資料夾，並在裡面創建一個名為 demo1.launch.py 的檔案。

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
在此例中，我們定義一個 Node 物件，並加入至 LaunchDescription 的初始化函數中

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

### 2. 編譯與執行

#### 修改 CMakelist.txt (C++)

如果 package 是 C++ 的話，還需要在 CMakeLists.txt 中加入以下內容，以便將 launch 檔案安裝至系統中。

```cmake
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```

#### 修改 setup.py (python)

如果 package 是 python 的話，則需要修改 setup.py 中 data_files 內容，加上
`(f"share/{package_name}/launch", glob("launch/*.launch.py"))`

```python
from setuptools import setup
from glob import glob

package_name = "my_tutorials"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    ...
)
```

#### 使用 colcon 進行編譯

```bash
$ colcon build --packages-select my_tutorials
```

#### 執行 launch 檔

```bash
$ ros2 launch my_tutorials demo.launch.py
```

### 3. 包含多個 Node

定義多個 Node 也很容易，創建一個名為 demo2.launch.py 的檔案如下

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

### 4. 包含其他 Launch 檔案

launch 檔也可以在包含其他的 luanch 檔。創建一個名為 demo3.launch.py 的檔案如下

```python
import os
from launch.launch_description import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    launch_demo1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("my_ros2_tutorials"),
                "launch",
                "demo1.launch.py",
            )
        )
    )
    ld = LaunchDescription(
        [
            launch_demo1,
        ]
    )
    return ld
```

這個範例中，我們分別使用了以下三個類別，來實現包含其他 launch 檔案的功能

- **IncludeLaunchDescription** 來引用其他的 launch 檔案
- **PythonLaunchDescriptionSource** 用於指定其他 launch 檔案的路徑
- **get_package_share_directory** 則用於找到該軟體包的共享目錄

當我們運行 demo3.launch.py 時，它將運行 demo1.launch.py 中的所有節點。

### 5. 參數傳遞

在本節中，我們將介紹 launch 系統中的兩個重要概念：DeclareLaunchArgument 和 LaunchConfiguration。
這些功能允許我們在啟動 launch 文件時設置參數值，並將它們傳遞給 ROS 節點或是其他 launch 文件。

#### DeclareLaunchArgument

`DeclareLaunchArgument`用於在命令行界面中暴露一個可設置的參數。
它允許用戶在啟動 launch 文件時指定參數的值。
例如，要在命令行中設置一個名為 width 的參數，可以使用以下語法：

```bash
$ ros2 launch my_package my_launch_file.launch.py width:=800
```

在 launch 文件中，我們需要定義 DeclareLaunchArgument，如下所示：

```python
width_arg = DeclareLaunchArgument("width", default_value="640")
```

#### LaunchConfiguration

`LaunchConfiguration`用於將命令行中設置的參數值傳遞給 ROS 節點。
它將參數值存儲為一個變量，然後可以將該變量傳遞給節點的參數列表。
以下是創建一個 LaunchConfiguration 變量的示例：

```python
width = LaunchConfiguration("width")
```

下面是一個使用 DeclareLaunchArgument 和 LaunchConfiguration 的完整範例：

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    width_arg = DeclareLaunchArgument("width", default_value="640")
    width = LaunchConfiguration("width")

    ld = LaunchDescription(
        [
            width_arg,
            Node(
                package="my_package",
                executable="my_node",
                name="my_node",
                parameters=[{"width": width}],
            ),
        ]
    )
    return ld

```

在這個範例中，我們使用 DeclareLaunchArgument 定義了一個名為 width 的參數，並為其指定了一個默認值。
然後，我們使用 LaunchConfiguration 將該參數傳遞給名為 my_node 的 ROS 節點。在這種情況下，用戶可以在啟動 launch 文件時設置 width 參數的值。

透過 DeclareLaunchArgument 和 LaunchConfiguration，我們可以方便地向 ROS 2 節點傳遞參數，使我們的應用程序更具靈活性和可配置性。

### References

- [Creating a launch file](https://docs.ros.org/en/galactic/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [Integrating launch files into ROS 2 packages](https://docs.ros.org/en/galactic/Tutorials/Intermediate/Launch/Launch-system.html)
- [Tutorial: ROS2 launch files – All you need to know](https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/#cmd-line-args-hand-over)
