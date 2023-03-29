---
title: "ROS2 DeclareLaunchArgument 和 LaunchConfiguration 細談"
date: 2023-03-29T08:43:00+08:00
summary: 三十分鐘學習launch file
ShowToc: true
TocOpen: true
tags: ["ROS2"]
author: ["Kevin Lee"]
draft: false
---

### 前言

DeclareLaunchArgument 和 LaunchConfiguration 在設計上有不同的目的，它們通常一起使用以實現靈活且可讀的 launch 文件。讓我們來看看它們的作用以及為什麼需要一起使用它們。

### 1. 參數好搭擋


- DeclareLaunchArgument：它的主要目的是在 launch 文件中定義一個命令行參數，用於讓使用者在啟動 launch 文件時指定某些參數值。DeclareLaunchArgument 還允許你為這些命令行參數設置默認值，以便在使用者未指定值時使用。

- LaunchConfiguration：它的主要目的是在 launch 文件中引用先前使用 DeclareLaunchArgument 定義的命令行參數。當在啟動 launch 文件時，這些命令行參數的值將被傳遞給使用 LaunchConfiguration 的各個 Node。

為什麼需要一起使用它們？

當你需要在不同的 Node 之間共享一個參數，或者想要在啟動 launch 文件時允許使用者指定某些參數值時，使用 DeclareLaunchArgument 和 LaunchConfiguration 是有意義的。這使得 launch 文件更靈活、更具可讀性，並有助於避免錯誤。

底下我們定義一個 pass_to_child_node.launch.py，並觀察啟動時有無帶參數，對節點行為的影響。

- pass_to_child_node.launch.py

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
$ ros2 launch my_ros2_tutorials pass_to_child_node.launch.py
[INFO] [talker-1]: process started with pid [751246]
[talker-1] [INFO] [1680054713.365480642] [my_talker]: Publishing: 'Hello World: 1'
[talker-1] [INFO] [1680054714.364295786] [my_talker]: Publishing: 'Hello World: 2'
[talker-1] [INFO] [1680054715.364055889] [my_talker]: Publishing: 'Hello World: 3'

$ ros2 node list
/my_talker
```

- 執行 launch 文件帶參數

```bash
$ ros2 launch my_ros2_tutorials pass_to_child_node.launch.py node_name:=changed_talker
[INFO] [talker-1]: process started with pid [751246]
[talker-1] [INFO] [1680054713.365480642] [my_talker]: Publishing: 'Hello World: 1'
[talker-1] [INFO] [1680054714.364295786] [my_talker]: Publishing: 'Hello World: 2'
[talker-1] [INFO] [1680054715.364055889] [my_talker]: Publishing: 'Hello World: 3'

$ ros2 node list
/changed_talker
```

在此 launch 文件中，我們啟動了一個在官方 package demo_nodes_cpp 下的執行檔，talker，
並且透過 pass_to_child_node.launch.py 的參數，改變節點的行為。在此是舉改變節點的名稱為例。
這也相當於執行下列的指令。

```bash
$ ros2 run demo_nodes_cpp talker --ros-args --remap __node:=changed_talker
```

#### 查看 launch 文件參數

我們可以使用 `ros2 launch package_name launch_file.launch.py -s` 在指令列查看 launch 文件的參數，如下所示。
這主要是透過 DeclareLaunchArgument 定義的。

```bash
$ ros2 launch my_ros2_tutorials pass_to_child_node.launch.py -s
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

### 3. 傳遞參數給子 launch 文件

我們定義參數，目的是要傳遞給子節點或子 launch 文件。子節點的狀況已在第一節說明過了，即 pass_to_child_node.launch.py 的範例。
這裡我們來看看如何傳遞參數給子 launch 文件。

底下我們用一個簡單範例講解此概念，分別定義 min.launch.py 和 pass_to_child_launch.launch.py。
其中 min.launch.py 是 pass_to_child_launch.launch.py 的子 launch 文件。

- pass_to_child_launch.launch.py

```python
import os
from launch.launch_description import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    width_arg = DeclareLaunchArgument("width", default_value="10")
    min_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("my_ros2_tutorials"),
                "launch",
                "min.launch.py",
            ),
        ),
        launch_arguments=[("width", LaunchConfiguration("width"))]
        # launch_arguments={"width": LaunchConfiguration("width")}.items(),
    )
    ld = LaunchDescription()
    ld.add_action(width_arg)
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

執行 launch 文件，可以看到子 launch 文件的參數值被正確傳遞。

```bash
# 執行 launch 文件不帶參數
$ ros2 launch my_ros2_tutorials pass_to_child_launch.launch.py
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: width: 10

# 執行 launch 文件帶參數
$ ros2 launch my_ros2_tutorials pass_to_child_launch.launch.py width:=123
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: width: 123
```

launch_arguments 參數可以傳入一個字典，或是一個由 (key, value) 組成的列表。

- 列表: launch_arguments=[("width", LaunchConfiguration("width"))]

  launch_arguments 是一個列表，每個元素都是一個元組，元組的第一個元素是參數名稱，第二個元素是參數的值，這個值可以是一個 LaunchConfiguration 物件，也可以是一個固定的值。
  在這種情況下，("width", LaunchConfiguration("width")) 的意思是將 width 參數的值從當前 launch 檔案中取出，然後傳遞給被引用的 min.launch.py 中的 width 參數。

- 字典: launch_arguments={"width": LaunchConfiguration("width")}.items()

  launch_arguments 是一個字典，其鍵值對應於參數名稱和參數值。這種寫法和第一種情況是等效的。

### 4. 特殊狀況: 隱式參數傳遞

即使沒定義 DeclareLaunchArgument 和 LaunchConfiguration，在啟動 launch 文件時帶上參數，還是可以傳遞給子 launch 文件。

我們來看看下例。沿用 min.launch.py，並定義 pass_to_child_launch_hidden.launch.py。

- pass_to_child_launch_hidden.launch.py

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

執行 pass_to_child_launch_hidden.launch.py，並帶上參數 width，可以看到參數被傳遞到 min.launch.py 中。

```bash
$ ros2 launch my_ros2_tutorials pass_to_child_launch_hidden.launch.py
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: width: 10

$ ros2 launch my_ros2_tutorials pass_to_child_launch_hidden.launch.py width:=200
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: width: 200
```

檢查可用的 launch 參數

```bash
$ ros2 launch my_ros2_tutorials pass_to_child_launch_hidden.launch.py -s
Arguments (pass arguments as '<name>:=<value>'):

  No arguments.
```

如預期，因為沒有使用 DeclareLaunchArgument 定義任何參數，所以會顯示 No arguments。
這很容易造成混肴，這種做法可能會降低代碼的可讀性和可維護性。
所以在撰寫 launch 文件時，應養成良好習慣，將所有需要用到的參數都定義在 launch 文件中。
意即使用 DeclareLaunchArgument 定義所有需要用到的參數。

### 5. 特殊狀況: 僅定義 DeclareLaunchArgument

讓我們來看一些特殊狀況，當 launch 文件中只定義了 DeclareLaunchArgument，但沒有使用 LaunchConfiguration，這樣的 launch 文件會發生什麼事情呢？
因為沒有定義 LaunchConfiguration，所以無法傳遞參數值給子節點。
故這裡定義一個 demo_DeclareLaunchArgument_only.launch.py，僅包含一個子 launch 文件，即前例的 min.launch.py

- demo_DeclareLaunchArgument_only.launch.py

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
    ld = LaunchDescription()
    ld.add_action(node_name_arg)
    ld.add_action(min_demo1)
    return ld
```

```bash
$ ros2 launch my_ros2_tutorials demo_DeclareLaunchArgument_only.launch.py
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: width: 20

$ ros2 launch my_ros2_tutorials demo_DeclareLaunchArgument_only.launch.py width:=33
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: width: 33
```

#### 使用場合

顯然，此結果和第三節相同。但是不需定義 launch_argument，可以少點程式碼。
那要用哪種方式呢? 目前，我暫時沒有結論，但看起來沒有特別的壞處，保留給讀者自行決定吧 😆

### 6. 特殊狀況: 僅定義 LoadLaunchArgument

這也是一種特殊狀況，當 launch 文件中只定義了 LaunchConfiguration，這樣的 launch 文件會發生什麼事情呢？

基本上，綜合以上各節，我們應該也可以推斷出結果了。
原則上，父 launch 文件的參數，都可傳遞給子節點和子 launch 文件，但是容易誤導別的使用者。

#### 使用場合

原則上要儘量避免，但要可以讀懂別人的這種寫法。

### 7. 結論

在 ROS2 的 launch 文件中，DeclareLaunchArgument 和 LaunchConfiguration 是兩個常用的元素。
DeclareLaunchArgument 用於在 launch 文件中定義命令行參數，並可以為參數設置默認值。
LaunchConfiguration 用於引用先前使用 DeclareLaunchArgument 定義的命令行參數。透過這兩個元素的搭配使用，可以讓 launch 文件更靈活、具有可讀性，避免錯誤。

- 使用 LogInfo 可以檢查傳入的 Launch 文件的參數值，並在執行 Launch 過程中顯示在控制台上。

- 針對要傳值的參數，必定使用 DeclareLaunchArgument 定義，以確保 Launch 文件的可讀性和可維護性。

- 本文介紹了三種特殊情況，建議在實際應用中謹慎使用與理解。

最後，總結下可能碰到的狀況，與建議的寫法

- 僅需傳參給子 Node

  DeclareLaunchArgument 和 LaunchConfiguration 兩者皆需定義

- 僅需傳參給子 Launch 文件

  以下兩種寫法皆OK

  - 僅定義 DeclareLaunchArgument
  - DeclareLaunchArgument 和 LaunchConfiguration 兩者皆定義

- 僅需傳參給子 Node 和子 Launch 文件

  DeclareLaunchArgument 和 LaunchConfiguration 兩者皆需定義
