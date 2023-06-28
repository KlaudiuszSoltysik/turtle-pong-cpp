#include <functional>
#include <stdexcept>
#include <thread>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/move.hpp"

static constexpr char KEYCODE_UP = 0x41;
static constexpr char KEYCODE_DOWN = 0x42;

bool running = true;

class KeyboardReader final {
  public: 
    KeyboardReader() {
      if (tcgetattr(0, & cooked_) < 0) {
        throw std::runtime_error("Failed to get old console mode");
      }
      struct termios raw;
      memcpy( & raw, & cooked_, sizeof(struct termios));
      raw.c_lflag &= ~(ICANON | ECHO);
      raw.c_cc[VEOL] = 1;
      raw.c_cc[VEOF] = 2;
      raw.c_cc[VTIME] = 1;
      raw.c_cc[VMIN] = 0;
      if (tcsetattr(0, TCSANOW, & raw) < 0) {
        throw std::runtime_error("Failed to set new console mode");
      }
    }

    char readOne() {
      char c = 0;
      int rc = read(0, & c, 1);
      if (rc < 0) {
        throw std::runtime_error("read failed");
      }

      return c;
    }

    ~KeyboardReader() {
      tcsetattr(0, TCSANOW, & cooked_);
    }

  private: 
    struct termios cooked_;
};

class MovePlayerTurtle {
  public: 
    MovePlayerTurtle() {
      nh_ = rclcpp::Node::make_shared("move_player_turtle");

      publisher = nh_ -> create_publisher < interfaces::msg::Move > ("move_player_turtle", 10);
    }

    int keyLoop() {
      char c;

      std::thread {
        std::bind( & MovePlayerTurtle::spin, this)
      }.detach();

      puts("Reading from keyboard");
      puts("---------------------------");
      puts("Use arrow keys to move the turtle.");

      while (running) {
        try {
          c = input_.readOne();
        } catch (const std::runtime_error & ) {
          perror("read():");
          return -1;
        }

        RCLCPP_DEBUG(nh_ -> get_logger(), "value: 0x%02X\n", c);

        switch (c) {
        case KEYCODE_UP:
          RCLCPP_DEBUG(nh_ -> get_logger(), "UP");
          direction = 1;
          break;
        case KEYCODE_DOWN:
          RCLCPP_DEBUG(nh_ -> get_logger(), "DOWN");
          direction = -1;
          break;
        default:
          direction = 0;
          break;
        }

        auto msg = interfaces::msg::Move();
        msg.direction = direction;
        publisher -> publish(msg);
      }

      return 0;
    }

  private:
    int direction = 0;

    void spin() {
      rclcpp::spin(nh_);
    }

    rclcpp::Node::SharedPtr nh_;
    rclcpp::Publisher < interfaces::msg::Move > ::SharedPtr publisher;

    KeyboardReader input_;
};

void quit(int sig) {
  (void) sig;
  running = false;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  signal(SIGINT, quit);

  MovePlayerTurtle move_player_turtle;

  int rc = move_player_turtle.keyLoop();

  rclcpp::shutdown();

  return rc;
}
