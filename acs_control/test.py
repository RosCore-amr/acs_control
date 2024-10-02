import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String  # Message kiểu String


class JsonFormatterSubscriber(Node):

    def __init__(self):
        super().__init__("json_formatter_subscriber")

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.loop_timer)

    def loop_timer(self):
        # def listener_callback(self, msg):
        try:
            # Giả sử msg.data là một chuỗi JSON escaped như: '{\"key\": \"value\"}'
            escaped_json_str = {"robot_code": "robot_2", "position_collision": "Tmh"}

            # Lặp lại quá trình giải mã JSON cho đến khi có dạng dictionary
            while isinstance(escaped_json_str, str):
                try:
                    # Giải mã chuỗi JSON escape thành một đối tượng Python (dictionary hoặc list)
                    escaped_json_str = json.loads(escaped_json_str)
                except json.JSONDecodeError:
                    # Nếu không thể decode tiếp, dừng lại
                    break

            # In ra JSON với định dạng đẹp nếu kết quả là dictionary hoặc list
            if isinstance(escaped_json_str, (dict, list)):
                json_formatted_str = json.dumps(escaped_json_str, indent=4)
                self.get_logger().info(f"\n{json_formatted_str}")
            else:
                self.get_logger().info(f"Result: {escaped_json_str}")

        except Exception as e:
            # In ra lỗi nếu có bất kỳ lỗi nào khác
            self.get_logger().error(f"Lỗi: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    json_formatter_subscriber = JsonFormatterSubscriber()

    rclpy.spin(json_formatter_subscriber)

    json_formatter_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
