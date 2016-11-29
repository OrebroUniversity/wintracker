
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

class WinRebaseNode {

    private:
	// Our NodeHandle, points to home
	ros::NodeHandle nh_;
	ros::NodeHandle n_;
	Eigen::Affine3d wrist2body, win2base, reference;
	
	//ros::Publisher gripper_map_publisher_;
	ros::Publisher pub_pose;
	ros::Subscriber sub_pose;

	tf::TransformListener tl;
	tf::Transform gripper2map;

	std::string wrist_frame, base_frame;
	std::string wintracker_topic, rebased_topic;

	bool is_set, is_sim_time;


    public:
	WinRebaseNode() {

	    is_set = false;
	    nh_ = ros::NodeHandle("~");
	    n_ = ros::NodeHandle();

	    nh_.param<std::string>("wrist_frame",wrist_frame,"gripper_r_base");
	    nh_.param<std::string>("base_frame",base_frame,"yumi_body");
	    nh_.param<std::string>("wintracker_topic",wintracker_topic,"/wintracker/pose_old");
	    nh_.param<std::string>("rebased_topic",rebased_topic,"/wintracker/pose");
	    //n_.param<bool>("use_sim_time",is_sim_time,false);
	    is_sim_time = true;

	    pub_pose = nh_.advertise<geometry_msgs::PoseStamped> (rebased_topic,10);
	    sub_pose = n_.subscribe(wintracker_topic, 5, &WinRebaseNode::poseCallback, this);
	}
        
	
	void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
	    if(!is_set) {
		ROS_INFO("Got first message, computing transform");
		//set the reference transform
		tf::StampedTransform wrist_to_body;
		try {
		    ros::Time t_now;
		    if(is_sim_time) {
		       t_now = ros::Time::now();
		    } else {
		       t_now = msg->header.stamp;
		    }
		    tl.waitForTransform(base_frame, wrist_frame, t_now, ros::Duration(1.) );
		    tl.lookupTransform(base_frame, wrist_frame, t_now, wrist_to_body);
		} catch (tf::TransformException ex) {
		    ROS_ERROR("%s",ex.what());
		    return;
		}
		tf::transformTFToEigen(wrist_to_body,wrist2body);
		tf::poseMsgToEigen(msg->pose, win2base);

		reference = wrist2body*win2base.inverse();
		std::cout<<reference.matrix()<<std::endl;
		is_set = true;
	    }
	    else {
		//publish the rebased pose
		Eigen::Affine3d current;
		tf::poseMsgToEigen(msg->pose, current);
		current = reference*current;

		geometry_msgs::PoseStamped newmsg;
		newmsg.header.stamp = msg->header.stamp;
		newmsg.header.frame_id = msg->header.frame_id;
		newmsg.header.seq = msg->header.seq;

		tf::poseEigenToMsg(current, newmsg.pose);
		pub_pose.publish(newmsg);
	    }
	}

};

int main(int argc, char **argv) {

    ros::init(argc,argv,"rebase_node");
    std::cout<<"Starting up...\n"; 
    WinRebaseNode nd;
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
    std::cout<<"Dying\n";

    return 0;
}
