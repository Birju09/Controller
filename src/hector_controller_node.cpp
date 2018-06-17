// This code is a very simple implementation of the control of differential drive robot which takes the path from hector exploration and drives the robot to follow that path
// For obstacle avoidance it performs a maneuver of turning and travelling some distance in the rotated direction and do the path planning again
// Author : Birju Vachhani 
// Time stamp: 1st June, 2018, 8:57pm
// Rutgers University , Zou Lab, Department of Mechanical and Aerospace Engineering, Piscatway, NJ08854

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <sensor_msgs/Range.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>
#include <nav_msgs/Path.h>

//class for positions
class Position
{
public:
	float x;
	float y;
	float th;
	friend float distance(Position &point1, Position &point2);
	Position()
	{

	}
	void setPosition(float x, float y, float th)
	{
		this->x = x;
		this->y = y;
		this->th = th;
	}
};

struct min
{
	float minimum;
	int index;
};

//global objects
Position cur;
Position initial;
Position pos;
std::vector<float> sonarlist;
std::vector<Position> point_list;

class Pubsub
{

public:
	ros::NodeHandle n;
	ros::ServiceClient exploration_plan_service_client;
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::Subscriber sub1 = n.subscribe("/slam_out_pose", 1, &Pubsub::callback, this);
        ros::Subscriber sonar1 = n.subscribe("/sonar1", 2, &Pubsub::callback1, this);
	ros::Subscriber sonar2 = n.subscribe("/sonar2", 2, &Pubsub::callback2, this);
	ros::Subscriber sonar3 = n.subscribe("/sonar3", 2, &Pubsub::callback3, this);
	ros::Subscriber sonar4 = n.subscribe("/sonar4", 2, &Pubsub::callback4, this);
        ros::Subscriber click = n.subscribe("/clicked_point",1,&Pubsub::callback_click,this);
	
	
        void callservice()
	{
		hector_nav_msgs::GetRobotTrajectory srv_exploration_plan;
		exploration_plan_service_client = n.serviceClient<hector_nav_msgs::GetRobotTrajectory>("get_exploration_path");
		if (exploration_plan_service_client.call(srv_exploration_plan))
		{
			point_collection(srv_exploration_plan.response.trajectory.poses);
		}
		else
		{
			ROS_ERROR("Failed to call service");
		}
	}
	
        //callback for the clicked point

        void callback_click(const geometry_msgs::PointStamped& click)
        {
            ROS_WARN("Click Data received, do you want to stop the robot from following the path and instead want to reach the clicked location ? (y/n)");
            char response;
            std::cin>>response;
            if (response=='y' || response=='Y')
            {
                point_list.clear();
                Position* point = new Position;
                point->setPosition(click.point.x,click.point.y,0.0);
                point_list.push_back(*point);
                delete point;
            }
            if (response=='n' || response=='N')
            {
                ROS_INFO("Ignoring the click command");
            }
            execute_points();

        }

	//callback to set the current position

	void callback(const geometry_msgs::PoseStamped &data)
	{
        double roll,pitch,yaw;
		tf::Quaternion quater;
		tf::quaternionMsgToTF(data.pose.orientation, quater);
		tf::Matrix3x3(quater).getRPY(roll, pitch, yaw);
		cur.setPosition(data.pose.position.x, data.pose.position.y, yaw);
	}

	//callback to get the initial position
	void callback_init(const geometry_msgs::PoseStamped& data)
	{
        double roll,pitch,yaw;
		tf::Quaternion quater;
		tf::quaternionMsgToTF(data.pose.orientation, quater);
		tf::Matrix3x3(quater).getRPY(roll, pitch, yaw);
		initial.setPosition(data.pose.position.x, data.pose.position.y, yaw);
	}


	//Sonar callbacks
	void callback1(const sensor_msgs::Range& son1data)
	{
		sonarlist[0] = son1data.range;
        //ROS_INFO("distance is %f",sonarlist[0]);

	}
	void callback2(const sensor_msgs::Range& son2data)
	{
		sonarlist[1] = son2data.range;

	}
	void callback3(const sensor_msgs::Range& son3data)
	{
		sonarlist[2] = son3data.range;

	}
	void callback4(const sensor_msgs::Range& son4data)
	{
		sonarlist[3] = son4data.range;
	}
    
	//This callback collect all the position and stores them in a vector to be fed to the controller
	void point_collection(const std::vector<geometry_msgs::PoseStamped>& msg)
	{
        ROS_INFO("Making a list now");
		
        ROS_INFO("size is %d",msg.size());
        int element_skip=1;
        element_skip = msg.size()%5;
        std::vector<geometry_msgs::PoseStamped>::const_iterator it;
                for( it = msg.begin();it!=msg.end();it=it+element_skip)
		{
                    Position* point = new Position;
                    point->setPosition(it->pose.position.x,it->pose.position.y,0);
                    point_list.push_back(*point);
                    delete point;
            //ROS_INFO("point_added");
		}
		execute_points();
	}
		
	//execute all the point in the list
	void execute_points()
	{
        ROS_INFO("Execution Starts");
		while (point_list.size() > 0)
		{
			control_run(point_list[0]);
		}
	}
	
	//main control run
	void control_run(Position current_point)
	{
		ROS_INFO("Starting the controller now");
        //ros::Rate r(50);
		pos.setPosition(current_point.x, current_point.y, 0.0);
		float angle, curr_angle;
		curr_angle = cur.th;
        float sign=1.0;
		if (pos.x > cur.x)
		{
			sign = 1.0;
		}
		else
		{
			sign = -1.0;
		}
        int angsign = 1;
		int obsign= 1;
		while (distance(cur, pos) > 0.20)
		{
            //ROS_INFO("distance from obsacle is %f",obstacle_range());
            angle = atan((cur.y - pos.y) / (cur.x - pos.x));
			if (obstacle_range() < 40.0)
			{
				ROS_WARN("Obstacle on the way, performing maneuver now");
                if(cur.x  > pos.x)
                {
                    obsign = 1;
                }
                else
                {
                    obsign = -1;
                }
				do_maneuver(obsign);
				ros::spinOnce();
			}
			geometry_msgs::Twist twist;
            float kp=0.15,ki=0.025,error,error_sum=0.0;
			//std::cout << curr_angle - angle << std::endl;
			while (fabs(curr_angle - angle) > 0.025)
			{
                ros::spinOnce();
                angle = atan((cur.y - pos.y) / (cur.x - pos.x));
                if((curr_angle-angle) > 0.0)
                {
                    angsign = -1.0;
                }
                else if ((curr_angle-angle) < 0.0)
                {
                    angsign = 1.0;
                }
				curr_angle = cur.th;
                error = fabs(curr_angle - angle);
                error_sum = error_sum + error;
				//ROS_INFO("current angle absolute difference is %f", error);
				twist.angular.z = fmin(0.25,(error_sum*ki) + (kp*error))*angsign;
				pub.publish(twist);
                //r.sleep();
				
			}
			twist.linear.x = -1.75 * sign;
			twist.angular.z = 0.0;
			pub.publish(twist);
			ros::spinOnce();
			//ROS_INFO("Distance is %f", distance(cur, pos));
		}
		ROS_INFO("Desitination has arrived");
		geometry_msgs::Twist twist;
		twist.linear.x = 0.0;
		twist.angular.z = 0.0;
		pub.publish(twist);
		point_list.erase(point_list.begin());

	}

	//maneuver direction for obstacle avoidance
	void do_maneuver(int obsign)
	{
		min minsonar;
        ROS_INFO("In now");
		minsonar = minlist(sonarlist);
		ros::Time begin = ros::Time::now();
		geometry_msgs::Twist twist;
		while (obstacle_range() > 50)
		{
            ROS_INFO("In the loop of turn now and obsign is %d",obsign);
			twist.angular.z = 0.3;
			twist.linear.x = 0.0;
			pub.publish(twist);
            ros::spinOnce();
		}
		begin = ros::Time::now();
		while (((ros::Time::now() - begin).toSec() < 10.5) && (obstacle_range()>50.0))
		{
			twist.angular.z = 0.0;
			twist.linear.x = 0.5 * obsign;
			pub.publish(twist);
            ros::spinOnce();
		}	
		ROS_INFO("maneuver is completed");
		
		
	}
    float average(int n)
    {
        float average = 0.0;
        for(int i=0;i<=4;i++)
        {
            average += sonarlist[n];
            ros::spinOnce();
        }
        return average;
    }
	//function to calculate the distance from the obstacle
	float obstacle_range()
	{
        float obstacle_distance = fmin(fmin(sonarlist[0], sonarlist[1]), fmin(sonarlist[2], sonarlist[3]));
		return obstacle_distance;
	}

	//function to calculate the distance between two points
	float distance(Position &point1, Position &point2)
	{
		return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
	}

	//minimum function
	min minlist(std::vector<float> &sonarlist)
	{
		min minsonar;
		float a = sonarlist[0];
		for (int it = 0; it<sonarlist.size(); it++)
		{
			if (sonarlist[it] < a)
			{
				a = sonarlist[it];
				minsonar.index = it;
			}

		}
		minsonar.minimum = a;
		return minsonar;
	}


};


//build the sonars
void buildsonar(int n)
{
	for (int i = 0; i<n; i++)
	{
		sonarlist.push_back(100.0);
	}
	ROS_INFO("Sonar array built");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Controller");

	buildsonar(4);
    
	Pubsub control;


	while (ros::ok())
	{
		control.callservice();
		ros::spinOnce();        
	}
	return 0;
}


