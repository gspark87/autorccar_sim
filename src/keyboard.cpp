#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int8.hpp>

#include <iostream>
#include <termios.h>
#include <unistd.h>

class TeleopTwistKeyboard : public rclcpp::Node {
public:
    TeleopTwistKeyboard() : Node("teleop_twist_keyboard"),
          pub_cmd_vel_(this->create_publisher<geometry_msgs::msg::Twist>("/isaac/cmd_vel", 10)),
          pub_command_topic_(this->create_publisher<std_msgs::msg::Int8>("/command", 10)),
          speed_(0.0),
          turn_(0.0),
          step_(1.0),
          status_(0.0) {

        printMsg();
        printVels(speed_, turn_);

        while (rclcpp::ok()) {
            char key;
            key = getChar();
            processKey(key);
            publishTwist();
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_command_topic_;

    double speed_;
    double turn_;
    double step_;
    int status_;

    std::map<char, std::pair<int, int>> keyBindings = {
        {'i', {0, 0}},
        {'o', {1, 0}},
        {'p', {2, 0}},
    };

    std::map<char, std::pair<int, int>> speedBindings = {
        {'1', {1, 0}},
        {'2', {2, 0}},
        {'3', {3, 0}},
        {'4', {4, 0}},
        {'5', {5, 0}},
        {'6', {6, 0}},
        {'7', {10, 0}},
        {'8', {50, 0}},
        {'9', {100, 0}},
    };

    std::map<char, std::pair<int, int>> moveBindings = {
        {'w', {1, 0}},
        {'s', {-1, 0}},
        {'a', {0, -1}},
        {'d', {0, 1}},
        {'q', {0, 0}},
        {'e', {0, 0}},
        {'z', {0, -90}},
        {'c', {0, 90}},
    };

    char getChar() {
        struct termios oldt, newt;
        char ch;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }

    void printMsg() {
        std::cout << "This node takes keypresses from the keyboard and publishes them\n"
                  << "as Twist messages. It works best with a US keyboard layout.\n"
                  << "---------------------------\n"
                  << "Input key layout:\n"
                  << "   q    w    e\n"
                  << "   a    s    d\n"
                  << "   z         c\n"
                  << "1~9 : increase/decrease speed step size\n"
                  << "q/e : reset speed/turn value\n"
                  << "z/c : minimum/maximum servo direction\n"
                  << "w/s : increase/decrease only linear speed by step size\n"
                  << "a/d : increase/decrease only angular speed by 1\n"
                  << "i/o/p : publish command_topic (i=Stop, o=Start, p=Manual)\n"
                  << "CTRL-C to quit\n";
    }

    void printVels(double speed, double turn) {
        std::cout << "currently:\tspeed " << speed << "\tturn " << turn << '\n';
    }


    void processKey(char key) {
        if (speedBindings.find(key) != speedBindings.end()) {
            step_ = speedBindings[key].first;
        }
        else if (keyBindings.find(key) != keyBindings.end()) {
            std_msgs::msg::Int8 cmd_msg;
            cmd_msg.data = keyBindings[key].first;
            pub_command_topic_->publish(cmd_msg);

            if (cmd_msg.data == 0) {
                std::cout << "[Publish] command_topic = " << keyBindings[key].first << " (Stop)\n";
            } else if (cmd_msg.data == 1) {
                std::cout << "[Publish] command_topic = " << keyBindings[key].first << " (Auto)\n";
            } else if (cmd_msg.data == 2) {
                std::cout << "[Publish] command_topic = " << keyBindings[key].first << " (Manual)\n";
            }
        }
        else if (moveBindings.find(key) != moveBindings.end()) {
            speed_ += moveBindings[key].first * step_;
            turn_ += moveBindings[key].second * -1;

            if (key == 'q') {
                speed_ = 0.0;
            }
            else if (key == 'e') {
                turn_ = 0.0;
            }

            if (key == 'z') {
                turn_ = -90.0;
            }
            else if (key == 'c') {
                turn_ = 90.0;
            }

            if (speed_ < 0) {
                speed_ = 0.0;
            }
            if (turn_ > 90) {
                turn_ = 90.0;
            }
            else if (turn_ < -90) {
                turn_ = -90.0;
            }

            printVels(speed_, turn_);
            if (status_ == 14) {
                printMsg();
            }
            status_ = (status_ + 1) % 15;
        }
        else {
            if (key == '\x03')
                rclcpp::shutdown();
        }
    }

    void publishTwist() {
        auto twist = std::make_unique<geometry_msgs::msg::Twist>();
        twist->linear.x = speed_;
        twist->linear.y = 0.0;
        twist->linear.z = 0.0;
        twist->angular.x = 0.0;
        twist->angular.y = 0.0;
        twist->angular.z = turn_;
        pub_cmd_vel_->publish(std::move(twist));
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopTwistKeyboard>());
    rclcpp::shutdown();
    return 0;
}
