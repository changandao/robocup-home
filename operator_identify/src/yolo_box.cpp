#include <darknet_ros_msgs/BoundingBoxes.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <all_msgs/Box.h>
#include <all_msgs/send_box.h>

class Position_retrieval
{

    private:
      //! The node handle
      ros::NodeHandle nh_;
      //! Node handle in the private namespace
      ros::NodeHandle priv_nh_;

      //! Define publishers and subscribers
      ros::Publisher pub_box;
      ros::ServiceClient client_box;

      ros::Subscriber sub_box;
      // ros::Subscriber sub2_;

      std::string desired_id = "person";
      all_msgs::send_box box_srv;


      int c = 0;

      void processBoxes(const darknet_ros_msgs::BoundingBoxesConstPtr& bx);
      
    public:
      //! Subscribes to and advertises topics
      Position_retrieval(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
      {

        sub_box = nh_.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 10, &Position_retrieval::processBoxes, this);
        client_box = nh_.serviceClient<all_msgs::send_box>("/follow_me/Box");
        // pub_box = priv_nh_.advertise<all_msgs::Box>("Box", 10);

        //ROS_INFO("yolo_box initialized ...");
      }

      ~Position_retrieval() {}
};

void Position_retrieval::processBoxes(const darknet_ros_msgs::BoundingBoxesConstPtr& bx)
{
	float prob = 0.5;
	int candidate = -1;
	int count = 0;
	for(int i = 0; i < bx->bounding_boxes.size(); i++)
	{
		if(bx->bounding_boxes[i].Class == desired_id && bx->bounding_boxes[i].probability > prob)
		{
			count++;
			candidate = i;
		}
	}
	if(count == 0)
	{
		ROS_INFO("God can't see legal person!");
	}
	else if(count > 1)
	{
		ROS_INFO("God is confusing!");
	}
	else
	{
		c++;
		if(c > 301) c--;
	}

	if(c == 300)
	{
		box_srv.request.id = desired_id;
		box_srv.request.x = bx->bounding_boxes[candidate].xmin;
		box_srv.request.y = bx->bounding_boxes[candidate].ymin;
		box_srv.request.height = bx->bounding_boxes[candidate].ymax - bx->bounding_boxes[candidate].ymin;
		box_srv.request.width = bx->bounding_boxes[candidate].xmax - bx->bounding_boxes[candidate].xmin;
		client_box.call(box_srv);
	}
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "yolo_box");
    ros::NodeHandle nh;
    Position_retrieval position_retrieval(nh);

    ros::spin();
    return 0;
}

