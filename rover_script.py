import rclpy

class Turtlebot3ObstacleDetection(Node):

def main(args=None):
  #Setup
  rclpy.init(args=args)
  ObsDet = Turtlebot3ObstacleDetection()
  rclpy.spin(ObsDet)

  #Main Program
  
  
  #Closing
  ObsDet.destroy_node()
  rclpy.shutdown()
