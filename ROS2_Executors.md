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

