import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

def main():
    rclpy.init()
    node = Node('sim_yaw_publisher')
    pub = Float32()
    publisher = node.create_publisher(Float32, '/sim_yaw', 10)
    
    current_yaw = 0.0
    # 模拟运行 10 秒
    start_time = time.time()
    while time.time() - start_time < 30:
        # 模拟 25 rad/s 的旋转速度 (参考论文实验数据 [cite: 650])
        # 换算成度约为 1432 度/秒
        current_yaw += 14.32 # 每 10ms 增加约 14.3 度
        if current_yaw >= 360.0:
            current_yaw -= 360.0
            
        msg = Float32()
        msg.data = current_yaw
        publisher.publish(msg)
        
        time.sleep(0.01) # 100Hz 发布频率
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
