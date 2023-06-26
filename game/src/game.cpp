#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/move.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;


class Game : public rclcpp::Node {
  public:
    Game() : Node("game") {
      std::srand(static_cast<unsigned int>(std::time(nullptr)));

      vel_y = (std::rand() % 401 - 200) / 100;

      // CONNECT WITH SPAWN SERVICE
      spawn_cli = this->create_client<turtlesim::srv::Spawn>("spawn");

      while (!spawn_cli->wait_for_service(1s)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      send_spawn_turtle_computer_request();
      send_spawn_turtle_player_request();

      RCLCPP_INFO(this->get_logger(), "----------------------");
      RCLCPP_INFO(this->get_logger(), "Game has been started.");
    }

  private:
    double vel_x = -2.0;
    double vel_y;
    int computer_score = 0;
    int player_score = 0;
    
    turtlesim::msg::Pose ball_pose = turtlesim::msg::Pose();
    turtlesim::msg::Pose computer_pose = turtlesim::msg::Pose();
    turtlesim::msg::Pose player_pose = turtlesim::msg::Pose();
    interfaces::msg::Move direction = interfaces::msg::Move();

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_cli;


    void send_spawn_turtle_computer_request() {
      auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
      
      request->x = 1.0;
      request->y = 5.5;
      request->theta = 0.0;
      request->name = "computer";

      spawn_cli->async_send_request(request);
    }

    void send_spawn_turtle_player_request() {
      auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
      
      request->x = 10.0;
      request->y = 5.5;
      request->theta = 3.14;
      request->name = "player";

      spawn_cli->async_send_request(request);
    }
};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Game>());
  rclcpp::shutdown();
  return 0;
}