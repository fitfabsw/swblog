---
title: "ROS2 DeclareLaunchArgument 和 LaunchConfiguration 細談"
date: 2023-03-29T08:43:00+08:00
summary: 三十分鐘學習launch file
ShowToc: true
TocOpen: true
tags: ["ROS2"]
author: ["Kevin Lee"]
draft: true
---

### 前言

聽起來不錯？那我們開始吧！

### 1. 參數好搭擋

DeclareLaunchArgument 和 LaunchConfiguration 在設計上有不同的目的，它們通常一起使用以實現靈活且可讀的 launch 文件。讓我們來看看它們的作用以及為什麼需要一起使用它們。

- DeclareLaunchArgument：它的主要目的是在 launch 文件中定義一個命令行參數，用於讓使用者在啟動 launch 文件時指定某些參數值。DeclareLaunchArgument 還允許你為這些命令行參數設置默認值，以便在使用者未指定值時使用。

- LaunchConfiguration：它的主要目的是在 launch 文件中引用先前使用 DeclareLaunchArgument 定義的命令行參數。當在啟動 launch 文件時，這些命令行參數的值將被傳遞給使用 LaunchConfiguration 的各個 Node。

為什麼需要一起使用它們？

當你需要在不同的 Node 之間共享一個參數，或者想要在啟動 launch 文件時允許使用者指定某些參數值時，使用 DeclareLaunchArgument 和 LaunchConfiguration 是有意義的。這使得 launch 文件更靈活、更具可讀性，並有助於避免錯誤。

底下我們定義一個 demo4.launch.py，並觀察啟動時有無帶參數，對節點行為的影響。

```python
from launch.launch_description import DeclareLaunchArgument, LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    node_name_arg = DeclareLaunchArgument("node_name", default_value="my_talker")
    node_name = LaunchConfiguration("node_name")
    talker_node = Node(
        package="demo_nodes_cpp",
        executable="talker",
        name=node_name,
    )
    ld = LaunchDescription()
    ld.add_action(node_name_arg)
    ld.add_action(talker_node)
    return ld
```

- 執行 launch 文件不帶參數

```bash
$ ros2 launch my_ros2_tutorials demo4.launch.py
[INFO] [talker-1]: process started with pid [751246]
[talker-1] [INFO] [1680054713.365480642] [my_talker]: Publishing: 'Hello World: 1'
[talker-1] [INFO] [1680054714.364295786] [my_talker]: Publishing: 'Hello World: 2'
[talker-1] [INFO] [1680054715.364055889] [my_talker]: Publishing: 'Hello World: 3'

$ ros2 node list
/my_talker
```

- 執行 launch 文件帶參數

```bash
$ ros2 launch my_ros2_tutorials demo4.launch.py node_name:=changed_talker
[INFO] [talker-1]: process started with pid [751246]
[talker-1] [INFO] [1680054713.365480642] [my_talker]: Publishing: 'Hello World: 1'
[talker-1] [INFO] [1680054714.364295786] [my_talker]: Publishing: 'Hello World: 2'
[talker-1] [INFO] [1680054715.364055889] [my_talker]: Publishing: 'Hello World: 3'

$ ros2 node list
/changed_talker
```

在此 launch 文件中，我們啟動了一個在官方 package demo_nodes_cpp 下的執行檔，talker，
並且透過 demo4.launch.py 的參數，改變節點的行為。在此是舉改變節點的名稱為例。
這也相當於執行下列的指令。

```bash
$ ros2 run demo_nodes_cpp talker --ros-args --remap __node:=changed_talker
```

#### 查看 launch 文件參數

我們可以使用 `ros2 launch package_name launch_file.launch.py -s` 在指令列查看 launch 文件的參數，如下所示。
這主要是透過 DecalreLaunchArgument 定義的。

```bash
$ ros2 launch my_ros2_tutorials demo4.launch.py -s
Arguments (pass arguments as '<name>:=<value>'):

    'node_name':
        no description given
        (default: 'my_talker')
```

### 2. 使用 LogInfo 檢查傳入 Launch 文件的參數值

有時我們想要針對傳入 launch 文件的參數值進行檢查，直覺上會想直接打印傳入的參數值

- log_argument_naive_print.launch.py

```python
from launch.launch_description import DeclareLaunchArgument, LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

def generate_launch_description():
    width_arg = DeclareLaunchArgument("width", default_value="20")
    print("width", LaunchConfiguration("width"))
    ld = LaunchDescription()
    ld.add_action(width_arg)
    return ld
```

```bash
$ ros2 launch my_ros2_tutorials naive_print.launch.py
[INFO] [launch]: Default logging verbosity is set to INFO
width <launch.substitutions.launch_configuration.LaunchConfiguration object at 0x7fc98e778520>
```

可看到，無法直接使用 `print("width: ", LaunchConfiguration("width"))`的方式打印。
正確的方式要使用 LogInfo。LogInfo 是 ROS 2 Launch System 提供的一個 action，可以用來在執行 Launch 過程中打印訊息。
底下提供一個簡單的範例

- log_argument.launch.py

```python
from launch.launch_description import DeclareLaunchArgument, LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

def generate_launch_description():
    width_arg = DeclareLaunchArgument("width", default_value="20")
    ld = LaunchDescription(
        [
            width_arg,
            LogInfo(msg=LaunchConfiguration("width")),
            LogInfo(msg=["width: ", LaunchConfiguration("width")]),
        ]
    )
    return ld
```

```bash
$ ros2 launch my_ros2_tutorials log_argument.launch.py
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: 20
[INFO] [launch.user]: width: 20
```

在 LogInfo 中，我們可以傳入多種不同的參數類型，例如

- 單一的訊息字串: `LogInfo(msg=LaunchConfiguration("width"))`
- 訊息字串列表: `LogInfo(msg=["width: ", LaunchConfiguration("width")])`

這些參數會被轉換為一個訊息字串，並在執行 Launch 過程中顯示在控制台上。

### 3. 隱式參數傳遞

即使沒定義 DeclareLaunchArgument 和 LaunchConfiguration，在啟動 launch 文件時帶上參數，
還是可以傳遞到內部其他的 launch 文件。

底下我們用一個簡單範例講解此概念，分別定義 min.launch.py 和 demo5.launch.py。
其中 min.launch.py 是 demo5.launch.py 的子 launch 文件。

- demo5.launch.py

```python
import os
from launch.launch_description import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    min_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("my_ros2_tutorials"),
                "launch",
                "min.launch.py",
            )
        )
    )
    ld = LaunchDescription()
    ld.add_action(min_launch)
    return ld
```

- min.launch.py

```python
from launch.launch_description import DeclareLaunchArgument, LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

def generate_launch_description():
    width_arg = DeclareLaunchArgument("width", default_value="20")
    ld = LaunchDescription(
        [
            width_arg,
            LogInfo(msg=["width: ", LaunchConfiguration("width")]),
        ]
    )
    return ld
```

執行 demo5.launch.py，並帶上參數 width，可以看到參數被傳遞到 min.launch.py 中。

```bash
$ ros2 launch my_ros2_tutorials demo5.launch.py
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: width: 10

$ ros2 launch my_ros2_tutorials demo5.launch.py width:=200
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: width: 200
```

檢查可用的 launch 參數

```bash
$ ros2 launch my_ros2_tutorials demo5.launch.py -s
Arguments (pass arguments as '<name>:=<value>'):

  No arguments.
```

如預期，因為沒有使用 DeclareLaunchArgument 定義任何參數，所以會顯示 No arguments。
這很容易造成混肴，這種做法可能會降低代碼的可讀性和可維護性。
所以在撰寫 launch 文件時，應養成良好習慣，將所有需要用到的參數都定義在 launch 文件中。
意即使用DeclareLaunchArgument 定義所有需要用到的參數。

### 4. 僅定義 DecalreLaunchArgument

```python
import os

from launch.launch_description import DeclareLaunchArgument, LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    node_name_arg = DeclareLaunchArgument("node_name", default_value="my_talker")

    min_demo1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("my_ros2_tutorials"),
                "launch",
                "min.launch.py",
            )
        )
    )

    talker_node = Node(
        package="demo_nodes_cpp",
        executable="talker",
        # name=node_name,
    )
    ld = LaunchDescription()
    ld.add_action(node_name_arg)
    ld.add_action(min_demo1)
    ld.add_action(talker_node)
    return ld
```

```bash
$ ros2 launch my_ros2_tutorials demo_DeclareLaunchArgument_only.launch.py width:=33
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: width: 33
[INFO] [talker-1]: process started with pid [834635]
[talker-1] [INFO] [1680063960.490802713] [talker]: Publishing: 'Hello World: 1'
[talker-1] [INFO] [1680063961.490619321] [talker]: Publishing: 'Hello World: 2'
[talker-1] [INFO] [1680063962.494913624] [talker]: Publishing: 'Hello World: 3'
```

#### 使用場合

當此參數不需要傳值給內部的 Node 時，僅需要傳給內部其他的 launch 文件時

### 5. LoadLaunchArgument

如果我們把上述 demo4.launch.py 中的兩行註解掉，

```python
    # node_name_arg = DeclareLaunchArgument("node_name", default_value="my_talker")
    # node_name = LaunchConfiguration("node_name")
    node_name = LaunchConfiguration("node_name", default="my_talker")
```
