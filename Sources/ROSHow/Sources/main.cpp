#include <ros/ros.h>
#include <std_msgs/builtin_int32.h>
#include <visualization_msgs/Marker.h>

#include <spdlog/spdlog.h>

void ProcessMessage(const std_msgs::Int32::ConstPtr& messagePtr)
{
	spdlog::info("Received message: {}", messagePtr->data);
}

int main(int argc, char* argv[])
{
	const std::string app_name{"ROShow"};

	ros::init(argc, argv, app_name);
	ros::NodeHandle node{};

	ros::Publisher publisher = node.advertise<visualization_msgs::Marker>("test2", 1000);
	ros::Subscriber subscriber = node.subscribe<std_msgs::Int32>("test1", 10, ProcessMessage);
	ros::Rate loop_rate(10);
	std::int64_t count{0};

	visualization_msgs::Marker marker{};
	const auto current_time = ros::Time::now();
	marker.header.frame_id = "odom";
	marker.header.stamp = current_time;
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 0.5;
    marker.pose.position.x = 1.0;
    marker.pose.position.y = 1.0;
    marker.pose.position.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
	marker.ns = "marker";
	marker.text = "hello, foxglove!";

	while (ros::ok())
	{
		marker.scale.x += 0.01;
		marker.pose.position.z += 0.01;

		publisher.publish(marker);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	node.shutdown();

	return 0;
}