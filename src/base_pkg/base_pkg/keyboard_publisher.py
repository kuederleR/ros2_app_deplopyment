import sys
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow
from PyQt5.QtCore import Qt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyPressPublisher(Node):
    def __init__(self):
        super().__init__('key_press_publisher')
        self.publisher_ = self.create_publisher(String, 'key_press', 10)

    def publish_key(self, key):
        msg = String()
        msg.data = key
        self.publisher_.publish(msg)

class MainWindow(QMainWindow):
    def __init__(self, publisher):
        super().__init__()
        self.publisher = publisher
        self.label = QLabel("Press any key", self)
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        font = self.label.font()
        font.setPointSize(24)  # Set the font size to 24
        self.label.setFont(font)
        self.setCentralWidget(self.label)
        self.setGeometry(100, 100, 400, 300)
        self.setWindowTitle("Key Press App")

    def keyPressEvent(self, event):
        key = event.text()
        self.label.setText(key)
        self.publisher.publish_key(key)

def main(args=None):
    rclpy.init(args=args)
    key_press_publisher = KeyPressPublisher()

    app = QApplication(sys.argv)
    main_window = MainWindow(key_press_publisher)
    main_window.show()

    try:
        sys.exit(app.exec())
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()