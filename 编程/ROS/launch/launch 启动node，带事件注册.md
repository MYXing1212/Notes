[Using event handlers — ROS 2 Documentation: Kilted documentation](https://docs.ros.org/en/kilted/Tutorials/Intermediate/Launch/Using-Event-Handlers.html)



更加详细的描述launch的项目，具体如何启动，给什么事件绑定什么回调函数

**<font color="#f79646">相当于一个装饰器的效果，不取修改节点的源代码，而是在原来执行的特定阶段，增加额外的动作</font>**

比如启动节点时，增加额外的动作，比如打印一条日志

比如某一事件执行完毕后，延时2s，执行另一个动作

最复杂的，可以根据一定的判断条件，来判定是否要执行一个新的动作

比如退出时打印log

