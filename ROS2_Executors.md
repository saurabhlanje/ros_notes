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

Please note that this table is provided in a markup format for readability. When using the table in an actual document or code, you may need to adjust the formatting to fit the desired context.
