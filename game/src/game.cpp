#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/msg/move.hpp"

using namespace std::chrono_literals;


class Game : public rclcpp::Node {
  public:
    Game() : Node("game") {
      // RANDOMIZE Y VELOCITY
      std::srand(static_cast<unsigned int>(std::time(nullptr)));
      vel_y = (std::rand() % 401 - 200) / 100;

      // CONNECT WITH SPAWN SERVICE
      spawn_cli = this->create_client<turtlesim::srv::Spawn>("spawn");

      while (!spawn_cli->wait_for_service(1s)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      // SPAWN COMPUTER AND PLAYER TURTLE
      send_spawn_turtle_computer_request();
      send_spawn_turtle_player_request();

      // CONNECT WITH SET_PEN SERVICES
      set_pen_ball_cli = this->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");

      while (!set_pen_ball_cli->wait_for_service(1s)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      set_pen_computer_cli = this->create_client<turtlesim::srv::SetPen>("computer/set_pen");

      while (!set_pen_computer_cli->wait_for_service(1s)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      set_pen_player_cli = this->create_client<turtlesim::srv::SetPen>("player/set_pen");

      while (!set_pen_player_cli->wait_for_service(1s)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      // TURN OF PENS
      send_set_pen_ball_request();
      send_set_pen_computer_request();
      send_set_pen_player_request();

      // CONNECT WITH TELEPORT SERVICES
      teleport_cli = this->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");

      while (!teleport_cli->wait_for_service(1s)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      // CREATE SUBSCRIBERS TO POSE TOPICS
      ball_pose_sub = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&Game::ball_pose_sub_callback, this, std::placeholders::_1));
      computer_pose_sub = this->create_subscription<turtlesim::msg::Pose>("computer/pose", 10, std::bind(&Game::computer_pose_sub_callback, this, std::placeholders::_1));
      player_pose_sub = this->create_subscription<turtlesim::msg::Pose>("player/pose", 10, std::bind(&Game::player_pose_sub_callback, this, std::placeholders::_1));
    
      // CREATE PUBLISHERS ON CMD_VEL TOPICS
      ball_pose_pub = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
      computer_pose_pub = this->create_publisher<geometry_msgs::msg::Twist>("computer/cmd_vel", 10);
      player_pose_pub = this->create_publisher<geometry_msgs::msg::Twist>("player/cmd_vel", 10);

      // CREATE SUBSCRIBER TO MOVE_PLAYER_TURTLE TOPIC
      move_player_turtle_sub = this->create_subscription<interfaces::msg::Move>("move_player_turtle", 10, std::bind(&Game::move_player_turtle_sub_callback, this, std::placeholders::_1));

      // RUN GAME LOOP FUNCTION
      timer = this->create_wall_timer(30ms, std::bind(&Game::game_loop, this));

      // NOTIFY USER
      RCLCPP_INFO(this->get_logger(), "---------------------------------------------------");
      RCLCPP_INFO(this->get_logger(), "Use arrow keys to control turtle on the right side.");
      RCLCPP_INFO(this->get_logger(), "---------------------------------------------------");
      RCLCPP_INFO(this->get_logger(), "Game has been started.");
    }

  private:
    // VARIABLES
    double vel_x = -2.0;
    double vel_y;
    int computer_score = 0;
    int player_score = 0;
    int direction;

    rclcpp::TimerBase::SharedPtr timer;
    
    turtlesim::msg::Pose ball_pose = turtlesim::msg::Pose();
    turtlesim::msg::Pose computer_pose = turtlesim::msg::Pose();
    turtlesim::msg::Pose player_pose = turtlesim::msg::Pose();

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_cli;

    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_ball_cli;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_computer_cli;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_player_cli;

    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_cli;

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr ball_pose_sub;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr computer_pose_sub;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr player_pose_sub;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ball_pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr computer_pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr player_pose_pub;

    rclcpp::Subscription<interfaces::msg::Move>::SharedPtr move_player_turtle_sub;
    
    // METHODS
    void game_loop() {
     
    }

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

    void send_set_pen_ball_request() {
      auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
      
      request->off = 1;

      set_pen_ball_cli->async_send_request(request);
    }

    void send_set_pen_computer_request() {
      auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
      
      request->off = 1;

      set_pen_computer_cli->async_send_request(request);
    }

    void send_set_pen_player_request() {
      auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
      
      request->off = 1;

      set_pen_player_cli->async_send_request(request);
    }

    void send_teleport_turtle_request() {
      auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
      
      request->x = 5.5;
      request->y = 5.5;

      teleport_cli->async_send_request(request);
    }

    void ball_pose_sub_callback(const turtlesim::msg::Pose & msg) {
      ball_pose = msg;
    }

    void computer_pose_sub_callback(const turtlesim::msg::Pose & msg) {
      computer_pose = msg;
    }

    void player_pose_sub_callback(const turtlesim::msg::Pose & msg) {
      player_pose = msg;
    }

    void move_player_turtle_sub_callback(const interfaces::msg::Move & msg) {
      direction = msg.direction;
    }
};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Game>());
  rclcpp::shutdown();
  return 0;
}