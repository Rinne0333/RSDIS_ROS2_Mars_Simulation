#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <thread>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class IngenuityTeleop : public rclcpp::Node
{
public:
	IngenuityTeleop()
	: Node("ingenuity_teleop")
	{
		gravity_ = declare_parameter("gravity", 3.71);
		// The model's main body mass and prior stable-hover command support this estimate.
		mass_kg_ = declare_parameter("mass_kg", 16.0);
		max_thrust_per_rotor_ = declare_parameter("max_thrust_per_rotor", 100.0);
		takeoff_thrust_per_rotor_ = declare_parameter("takeoff_thrust_per_rotor", 35.0);
		hover_thrust_per_rotor_ = declare_parameter("hover_thrust_per_rotor", 30.0);
		hover_height_gain_ = declare_parameter("hover_height_gain", 0.8);
		hover_vertical_damping_ = declare_parameter("hover_vertical_damping", 1.5);
		max_hover_acceleration_ = declare_parameter("max_hover_acceleration", 1.0);
		arrival_tolerance_ = declare_parameter("arrival_tolerance", 0.03);
		position_gain_xy_ = declare_parameter("position_gain_xy", 0.8);
		velocity_gain_xy_ = declare_parameter("velocity_gain_xy", 1.2);
		position_gain_z_ = declare_parameter("position_gain_z", 0.6);
		velocity_gain_z_ = declare_parameter("velocity_gain_z", 1.2);
		max_acceleration_xy_ = declare_parameter("max_acceleration_xy", 1.0);
		max_acceleration_z_ = declare_parameter("max_acceleration_z", 0.48);
		// Max desired BODY tilt (rad). ~0.15 is about 8.6 deg.
		max_tilt_rad_ = declare_parameter("max_tilt_rad", 0.15);

		ki_z_ = declare_parameter("ki_z", 0.0);
		max_z_integral_ = declare_parameter("max_z_integral", 5.0);
		max_climb_rate_ = declare_parameter("max_climb_rate", 0.40);
		att_kp_ = declare_parameter("att_kp", 1.0);
		att_kd_ = declare_parameter("att_kd", 0.3);
		swash1_sign_ = declare_parameter("swash1_sign", -1.0);
		swash2_sign_ = declare_parameter("swash2_sign", 1.0);
		max_swash_rad_ = declare_parameter("max_swash_rad", 0.3);
		xy_enable_height_ = declare_parameter("xy_enable_height", 0.5);
		// Your odometry reports roll = +-pi at rest (body frame is z-down).
		// This offset is removed so roll/pitch are relative to "level".
		roll_offset_rad_ = declare_parameter("roll_offset_rad", M_PI);
		vel_tau_ = declare_parameter("vel_filter_tau", 0.15);
		rate_tau_ = declare_parameter("rate_filter_tau", 0.05);

		top_thrust_publisher_ = create_publisher<std_msgs::msg::Float64>(
			"/ingenuity/top_blades/thrust", 10);
		bottom_thrust_publisher_ = create_publisher<std_msgs::msg::Float64>(
			"/ingenuity/bottom_blades/thrust", 10);
		swashplate_1_publisher_ = create_publisher<std_msgs::msg::Float64>(
			"/ingenuity/swashplate_1/joint", 10);
		swashplate_2_publisher_ = create_publisher<std_msgs::msg::Float64>(
			"/ingenuity/swashplate_2/joint", 10);

		odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
			"/ingenuity/odometry", 10,
			[this](const nav_msgs::msg::Odometry::SharedPtr message) {
				updateOdometry(*message);
			});
		control_timer_ = create_wall_timer(
			std::chrono::milliseconds(50), [this]() { controlStep(); });
	}

	bool handleCommand(const std::string & line)
	{
		if (line == "quit" || line == "exit") {
			rclcpp::shutdown();
			return false;
		}
		if (line == "help") {
			printHelp();
			return true;
		}
		if (line == "hold") {
			std::lock_guard<std::mutex> lock(state_mutex_);
			if (!have_odometry_) {
				RCLCPP_WARN(get_logger(), "Waiting for /ingenuity/odometry before holding position");
			} else {
				target_x_ = position_x_;
				target_y_ = position_y_;
				target_z_ = position_z_;
				takeoff_active_ = false;
				RCLCPP_INFO(get_logger(), "Holding current position");
			}
			return true;
		}
		if (line == "stop") {
			std::lock_guard<std::mutex> lock(state_mutex_);
			if (!have_odometry_) {
				RCLCPP_WARN(get_logger(), "Waiting for /ingenuity/odometry before stopping horizontal motion");
			} else {
				target_x_ = position_x_;
				target_y_ = position_y_;
				RCLCPP_INFO(get_logger(), "Horizontal target captured; altitude target unchanged at z=%.2f m",
					target_z_);
			}
			return true;
		}

		static const std::regex target_pattern(
			R"(^\s*([xyz])\s*(\+=|-=|=)\s*(-?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+))\s*$)");
		std::smatch match;
		if (!std::regex_match(line, match, target_pattern)) {
			RCLCPP_WARN(get_logger(), "Unrecognized command. Type 'help' for examples.");
			return true;
		}

		const double value = std::stod(match[3].str());
		const char axis = match[1].str()[0];
		const std::string operation = match[2].str();
		std::lock_guard<std::mutex> lock(state_mutex_);
		if (!have_odometry_) {
			RCLCPP_WARN(get_logger(), "Waiting for /ingenuity/odometry; command not applied");
			return true;
		}

		double * target = axis == 'x' ? &target_x_ : (axis == 'y' ? &target_y_ : &target_z_);
		if (operation == "=") {
			const double current_position = axis == 'x' ? position_x_ :
				(axis == 'y' ? position_y_ : position_z_);
			*target = current_position + value;
		} else {
			*target += operation == "+=" ? value : -value;
		}
		if (axis == 'z') {
			takeoff_active_ = target_z_ > position_z_ + arrival_tolerance_ &&
				(position_z_ - ground_z_) < xy_enable_height_;
		}
		RCLCPP_INFO(get_logger(), "Target position: x=%.2f y=%.2f z=%.2f m",
			target_x_, target_y_, target_z_);
		return true;
	}

	static void printHelp()
	{
		std::cout << "Commands move along world axes in meters:\n"
							<< "  x=10    move 10 m in positive x from the current position\n"
							<< "  y+=2    move the y target 2 m positive\n"
							<< "  z-=1    lower the z target by 1 m\n"
							<< "  stop    stop horizontal movement, keep altitude target\n"
							<< "  hold    hold the current position\n"
							<< "  quit    stop this node\n"
							<< std::flush;
	}

private:
	static double wrapPi(double a)
	{
		while (a > M_PI) {a -= 2.0 * M_PI;}
		while (a < -M_PI) {a += 2.0 * M_PI;}
		return a;
	}

	void updateOdometry(const nav_msgs::msg::Odometry & message)
	{
		const auto & pose = message.pose.pose;
		const double qx = pose.orientation.x, qy = pose.orientation.y;
		const double qz = pose.orientation.z, qw = pose.orientation.w;

		const double raw_roll = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
		const double new_pitch = std::asin(std::clamp(2.0 * (qw * qy - qz * qx), -1.0, 1.0));
		const double new_yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
		const double new_roll = wrapPi(raw_roll - roll_offset_rad_);

		const double stamp = static_cast<double>(message.header.stamp.sec) +
			1e-9 * static_cast<double>(message.header.stamp.nanosec);

		std::lock_guard<std::mutex> lock(state_mutex_);

		// Velocity and angular rates by finite difference + low-pass filter.
		// This avoids guessing which frame the odometry twist is expressed in.
		if (have_odometry_) {
			const double dt = stamp - prev_stamp_;
			if (dt > 1e-4) {
				const double av = dt / (vel_tau_ + dt);
				const double ar = dt / (rate_tau_ + dt);
				velocity_x_ += av * ((pose.position.x - position_x_) / dt - velocity_x_);
				velocity_y_ += av * ((pose.position.y - position_y_) / dt - velocity_y_);
				velocity_z_ += av * ((pose.position.z - position_z_) / dt - velocity_z_);
				rate_roll_ += ar * (wrapPi(new_roll - roll_) / dt - rate_roll_);
				rate_pitch_ += ar * (wrapPi(new_pitch - pitch_) / dt - rate_pitch_);
			}
		}
		prev_stamp_ = stamp;

		position_x_ = pose.position.x;
		position_y_ = pose.position.y;
		position_z_ = pose.position.z;
		roll_ = new_roll;
		pitch_ = new_pitch;
		yaw_ = new_yaw;

		if (!have_odometry_) {
			target_x_ = position_x_;
			target_y_ = position_y_;
			target_z_ = position_z_;
			ground_z_ = position_z_;
			have_odometry_ = true;
			RCLCPP_INFO(get_logger(), "Odometry received; holding initial position x=%.2f y=%.2f z=%.2f m",
				target_x_, target_y_, target_z_);
		}
	}

	void controlStep()
	{
		double px, py, pz, vx, vy, vz, tx, ty, tz, roll, pitch, yaw, wr, wp, ground_z;
		bool takeoff_active;
		{
			std::lock_guard<std::mutex> lock(state_mutex_);
			if (!have_odometry_) {
				return;
			}
			px = position_x_; py = position_y_; pz = position_z_;
			vx = velocity_x_; vy = velocity_y_; vz = velocity_z_;
			tx = target_x_; ty = target_y_; tz = target_z_;
			roll = roll_; pitch = pitch_; yaw = yaw_;
			wr = rate_roll_; wp = rate_pitch_;
			ground_z = ground_z_;
			takeoff_active = takeoff_active_;
		}
		if (takeoff_active && pz >= tz - arrival_tolerance_) {
			std::lock_guard<std::mutex> lock(state_mutex_);
			if (takeoff_active_ && std::abs(target_z_ - tz) < 1e-6) {
				target_x_ = position_x_;
				target_y_ = position_y_;
				takeoff_active_ = false;
				takeoff_active = false;
				tx = target_x_;
				ty = target_y_;
				RCLCPP_INFO(get_logger(),
					"Altitude reached at z=%.2f m; switching to hover and stopping horizontal movement",
					pz);
			}
		}

		const bool airborne = (pz - ground_z) > xy_enable_height_;
		const bool idle = !airborne && (tz - ground_z) < 0.1;

		// Idle on the ground: rotors off, swashplates neutral, integrator cleared.
		if (idle) {
			z_integral_ = 0.0;
			RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
				"idle on ground: z=%.2f rpy=(%.3f %.3f %.3f). Give a command, e.g. z=3",
				pz, roll, pitch, yaw);
			publish(top_thrust_publisher_, 0.0);
			publish(bottom_thrust_publisher_, 0.0);
			publish(swashplate_1_publisher_, 0.0);
			publish(swashplate_2_publisher_, 0.0);
			return;
		}

		// Takeoff thrust is fixed; after reaching altitude, hover baseline is corrected by height and vertical speed.
		const double hover_acceleration = std::clamp(
			hover_height_gain_ * (tz - pz) - hover_vertical_damping_ * vz,
			-max_hover_acceleration_, max_hover_acceleration_);
		const double lift = takeoff_active ?
			gravity_ + 2.0 * (takeoff_thrust_per_rotor_ - hover_thrust_per_rotor_) / mass_kg_ :
			gravity_ + hover_acceleration;

		// Keep takeoff vertical: level the body first, then engage XY position hold in hover.
		const bool level = std::abs(roll) < 0.2 && std::abs(pitch) < 0.2;
		double ax = 0.0;
		double ay = 0.0;
		if (!takeoff_active && airborne && level) {
			ax = std::clamp(
				position_gain_xy_ * (tx - px) - velocity_gain_xy_ * vx,
				-max_acceleration_xy_, max_acceleration_xy_);
			ay = std::clamp(
				position_gain_xy_ * (ty - py) - velocity_gain_xy_ * vy,
				-max_acceleration_xy_, max_acceleration_xy_);
		}

		// 3. Desired attitude in the heading frame
		const double ax_h = std::cos(yaw) * ax + std::sin(yaw) * ay;
		const double ay_h = -std::sin(yaw) * ax + std::cos(yaw) * ay;
		const double des_pitch = std::clamp(
			std::atan2(ax_h, lift), -max_tilt_rad_, max_tilt_rad_);
		const double des_roll = std::clamp(
			-std::atan2(ay_h, lift), -max_tilt_rad_, max_tilt_rad_);

		// 4. Inner attitude loop -> swashplates (flip swash*_sign if it diverges)
		const double swashplate_1 = std::clamp(
			swash1_sign_ * (att_kp_ * (des_pitch - pitch) - att_kd_ * wp),
			-max_swash_rad_, max_swash_rad_);
		const double swashplate_2 = std::clamp(
			swash2_sign_ * (att_kp_ * (des_roll - roll) - att_kd_ * wr),
			-max_swash_rad_, max_swash_rad_);

		// 5. Thrust, compensated for actual tilt
		const double tilt_cos = std::max(0.5, std::cos(roll) * std::cos(pitch));
		const double hover_thrust = std::clamp(
			(hover_thrust_per_rotor_ + mass_kg_ * hover_acceleration / 2.0) / tilt_cos,
			0.0, max_thrust_per_rotor_);
		const double rotor_thrust = takeoff_active ?
			std::clamp(takeoff_thrust_per_rotor_, 0.0, max_thrust_per_rotor_) : hover_thrust;

		RCLCPP_INFO_THROTTLE(
			get_logger(), *get_clock(), 1000,
			"mode=%s xy=(%.2f,%.2f)/(%.2f,%.2f) vxy=(%.2f,%.2f) z=%.2f/%.2f vz=%.2f "
			"rpy=(%.3f %.3f %.3f) swash=(%.3f %.3f) thrust=%.1f N/rotor",
			takeoff_active ? "takeoff" : "hover",
			px, py, tx, ty, vx, vy, pz, tz, vz,
			roll, pitch, yaw, swashplate_1, swashplate_2, rotor_thrust);

		publish(top_thrust_publisher_, rotor_thrust);
		publish(bottom_thrust_publisher_, rotor_thrust);
		publish(swashplate_1_publisher_, swashplate_1);
		publish(swashplate_2_publisher_, swashplate_2);
	}

	static void publish(
		const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & publisher,
		double value)
	{
		std_msgs::msg::Float64 message;
		message.data = value;
		publisher->publish(message);
	}

	std::mutex state_mutex_;
	bool have_odometry_{false};
	bool takeoff_active_{false};
	double prev_stamp_{0.0};
	double position_x_{0.0}, position_y_{0.0}, position_z_{0.0};
	double velocity_x_{0.0}, velocity_y_{0.0}, velocity_z_{0.0};
	double target_x_{0.0}, target_y_{0.0}, target_z_{0.0};
	double roll_{0.0}, pitch_{0.0}, yaw_{0.0};
	double rate_roll_{0.0}, rate_pitch_{0.0};
	double ground_z_{0.0};
	double z_integral_{0.0};

	double gravity_{3.71};
	double mass_kg_{16.0};
	double max_thrust_per_rotor_{100.0};
	double takeoff_thrust_per_rotor_{35.0};
	double hover_thrust_per_rotor_{30.0};
	double hover_height_gain_{0.8};
	double hover_vertical_damping_{1.5};
	double max_hover_acceleration_{1.0};
	double arrival_tolerance_{0.03};
	double position_gain_xy_{0.8};
	double velocity_gain_xy_{1.2};
	double position_gain_z_{0.6};
	double velocity_gain_z_{1.2};
	double max_acceleration_xy_{1.0};
	double max_acceleration_z_{0.48};
	double max_tilt_rad_{0.15};
	double ki_z_{0.0};
	double max_z_integral_{5.0};
	double max_climb_rate_{0.40};
	double att_kp_{1.0};
	double att_kd_{0.3};
	double swash1_sign_{-1.0};
	double swash2_sign_{1.0};
	double max_swash_rad_{0.3};
	double xy_enable_height_{0.5};
	double roll_offset_rad_{M_PI};
	double vel_tau_{0.15};
	double rate_tau_{0.05};

	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr top_thrust_publisher_;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bottom_thrust_publisher_;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr swashplate_1_publisher_;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr swashplate_2_publisher_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
	rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<IngenuityTeleop>();
	const std::weak_ptr<IngenuityTeleop> weak_node = node;
	std::thread([weak_node]() {
		IngenuityTeleop::printHelp();
		std::string line;
		while (rclcpp::ok() && std::getline(std::cin, line)) {
			const auto locked_node = weak_node.lock();
			if (!locked_node || !locked_node->handleCommand(line)) {
				break;
			}
		}
	}).detach();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}