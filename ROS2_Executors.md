# Executors in ROS 2 #
ROS 2 (Robot Operating System 2) introduces a new concept called executors, which are responsible for executing the callbacks associated with various ROS entities, such as nodes, timers, subscriptions, and services.
Executors play a vital role in managing the execution of asynchronous tasks within a ROS 2 system.

ROS 2 executors provide flexibility in managing the execution of callbacks, allowing you to choose the appropriate executor based on your system requirements. 
You can configure the executor type and associated parameters in your ROS 2 application to achieve the desired behavior for callback execution.

| Executor Type                      | Description                                                                                              |
|------------------------------------|----------------------------------------------------------------------------------------------------------|
| SingleThreadedExecutor             | Executes callbacks sequentially within a single thread. Suitable for systems without parallelism needs.  |
| MultiThreadedExecutor              | Executes callbacks concurrently using multiple threads. Provides parallel execution for improved performance.  |
| StaticSingleThreadedExecutor       | Executes callbacks sequentially within a static thread pool. Allows specifying the number of threads used.  |
| StaticMultiThreadedExecutor        | Executes callbacks concurrently within a static thread pool. Provides control over the number of threads.  |
| WallRate                           | Executes callbacks at a specified frequency. Useful for time-triggered tasks requiring periodic execution. |


# Example of SingleThreadedExecutor: #
```
import rclpy

def callback(msg):
    print('Received message:', msg.data)

def main():
    rclpy.init()
    node = rclpy.create_node('single_threaded_node')

    subscription = node.create_subscription(
        String,
        'topic',
        callback,
        10
    )

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok():
            executor.spin_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

# MultiThreadedExecutor #
```
import rclpy
import threading

def callback(msg):
    print('Received message:', msg.data)

def main():
    rclpy.init()
    node = rclpy.create_node('multi_threaded_node')

    subscription = node.create_subscription(
        String,
        'topic',
        callback,
        10
    )

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    # Create and start a separate thread for the executor
    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.start()

    try:
        while rclpy.ok():
            pass  # Main thread can perform other tasks here
    finally:
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join()

if __name__ == '__main__':
    main()
```

# StaticSingleThreadedExecutor #
```
import rclpy

def callback(msg):
    print('Received message:', msg.data)

def main():
    rclpy.init()
    node = rclpy.create_node('static_single_threaded_node')

    subscription = node.create_subscription(
        String,
        'topic',
        callback,
        10
    )

    executor = rclpy.executors.StaticSingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok():
            executor.spin_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

```

# StaticMultiThreadedExecutor #

```
import rclpy
import threading

def callback(msg):
    print('Received message:', msg.data)

def main():
    rclpy.init()
    node = rclpy.create_node('static_multi_threaded_node')

    subscription = node.create_subscription(
        String,
        'topic',
        callback,
        10
    )

    executor = rclpy.executors.StaticMultiThreadedExecutor()
    executor.add_node(node)

    # Create and start a separate thread for the executor
    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.start()

    try:
        while rclpy.ok():
            pass  # Main thread can perform other tasks here
    finally:
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join()

if __name__ == '__main__':
    main()

```
# WallRate #
```
import rclpy
from std_msgs.msg import String

def main():
    rclpy.init()
    node = rclpy.create_node('wall_rate_node')

    publisher = node.create_publisher(String, 'topic', 10)

    rate = node.create_rate(1)  # 1 Hz

    msg = String()
    msg.data = 'Hello, ROS 2!'

    try:
        while rclpy.ok():
            publisher.publish(msg)
            node.get_logger().info('Published message: %s' % msg

```

# Type of Callback Groups #


A Callback Group controls when Callbacks are allowed to be executed.

You can find two different Callback Groups:

    ReentrantCallbackGroup: Allows Callbacks to be executed in parallel without restriction.

    MutuallyExclusiveCallbackGroup: Allows only one Callback to be executed at a time.

# Difference between MutuallyExclusiveCallbackGroup and  ReentrantCallbackGroup #

You have used only MutuallyExclusiveCallbackGroup Callback Groups. This is because they are the default type in ROS2. However, there is another type called ReentrantCallbackGroup.

The main difference is:

    ReentrantCallbackGroup: Any Callback inside this group can be executed in parallel if there are enough threads. For example, if you add three Callbacks inside the same ReentrantCallbackGroup and have two threads, you can simultaneously execute TWO of the THREE Callbacks.

    MutuallyExclusiveCallbackGroup: All the Callbacks inside this group will only be executed one by one in that group, no matter how many threads you have. For example, if you have three Callbacks inside this group and three threads, only ONE Callback at a time will be executed.




