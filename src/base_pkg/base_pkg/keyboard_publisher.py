import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty
import signal
import select
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt, QTimer
import threading

class KeyboardPublisher(Node):
    def __init__(self, gui):
        
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(String, 'keyboard_input', 10)
        self.get_logger().info('Keyboard Publisher Node has been started.')
        self.gui = gui
        self.running = True

    def get_key(self, timeout=0.5):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                key = sys.stdin.read(1)
                if key == '\x03':  # Ctrl+C
                    raise KeyboardInterrupt
                if key == '\x1b':
                    key += sys.stdin.read(2)  # Read the next two characters for arrow keys
                self.get_logger().debug(f'Key pressed: {key}')
                return key
            else:
                return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def publish_key(self):
        try:
            while rclpy.ok() and self.running:
                key = self.get_key()
                msg = String()
                if key == '\x1b[A':
                    msg.data = 'UP'
                elif key == '\x1b[B':
                    msg.data = 'DOWN'
                elif key == '\x1b[C':
                    msg.data = 'RIGHT'
                elif key == '\x1b[D':
                    msg.data = 'LEFT'
                else:
                    self.get_logger().debug(f'Unhandled key: {key}')
                    continue
                self.gui.highlight_key(msg.data)
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published: {msg.data}')
        except KeyboardInterrupt:
            self.get_logger().info('Keyboard Interrupt detected.')
            self.running = False
        except Exception as e:
            self.get_logger().error(f'Error in publish_key: {e}')

class KeyboardGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Keyboard Input')
        self.setGeometry(100, 100, 200, 200)
        self.layout = QVBoxLayout()

        self.label_up = QLabel('UP', self)
        self.label_up.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.label_up)

        self.label_down = QLabel('DOWN', self)
        self.label_down.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.label_down)

        self.label_left = QLabel('LEFT', self)
        self.label_left.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.label_left)

        self.label_right = QLabel('RIGHT', self)
        self.label_right.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.label_right)

        self.setLayout(self.layout)
        self.reset_labels()

    def reset_labels(self):
        self.label_up.setStyleSheet('')
        self.label_down.setStyleSheet('')
        self.label_left.setStyleSheet('')
        self.label_right.setStyleSheet('')

    def highlight_key(self, key):
        self.reset_labels()
        if key == 'UP':
            self.label_up.setStyleSheet('background-color: yellow')
        elif key == 'DOWN':
            self.label_down.setStyleSheet('background-color: yellow')
        elif key == 'LEFT':
            self.label_left.setStyleSheet('background-color: yellow')
        elif key == 'RIGHT':
            self.label_right.setStyleSheet('background-color: yellow')


def signal_handler(sig, frame):
    rclpy.shutdown()
    app.quit()
    node.running = False

def run_node(node):
    try:
        node.publish_key()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def kill_ros():
    node.running = False
    rclpy.shutdown()
    node.destroy_node()

    

def main(args=None):
    # rclpy.init(args=args)
    # run_gui(KeyboardPublisher())
    rclpy.init(args=args)
    global node
    global app
    app = QApplication(sys.argv)
    gui = KeyboardGUI()

    node = KeyboardPublisher(gui)
    signal.signal(signal.SIGINT, signal_handler)
    


    ros_thread = threading.Thread(target=run_node, args=(node,))
    ros_thread.start()
    gui.show()
    app.aboutToQuit.connect(kill_ros)  # Connect the signal handler to the aboutToQuit signal
    sys.exit(app.exec_())  # Start the QApplication event loop


if __name__ == '__main__':
    main()