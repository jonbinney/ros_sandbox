#include <ros/ros.h>
#include <ros_sandbox/A.h>
#include <boost/thread.hpp>

namespace ros_sandbox
{

template <class MessageT>
class PollingSubscription
{
  typedef typename MessageT::ConstPtr MessageTConstPtr;

public:
  void subscribe(ros::NodeHandle &nh, const std::string &topic)
  {
    sub_ = nh.subscribe(topic, 1, &PollingSubscription::messageCb, this);
  }

  void messageCb(const MessageTConstPtr &msg)
  {
    ROS_INFO("Callback called");
    boost::lock_guard<boost::mutex> last_msg_lock(last_msg_mutex_);
    ROS_INFO("pub/sub delay: %.6f seconds", (ros::Time::now() - msg->header.stamp).toSec());
    last_msg_ = msg;
  }

  MessageTConstPtr getLastMessage() const
  {
    boost::lock_guard<boost::mutex> last_msg_lock(last_msg_mutex_);
    return last_msg_;
  }

private:
  ros::Subscriber sub_;

  // Most recently received message
  MessageTConstPtr last_msg_;
  mutable boost::mutex last_msg_mutex_;
};

class IntraSub
{
public:
  IntraSub(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  {
    psub1_.subscribe(nh, "input1");
  }

private:
  PollingSubscription<A> psub1_;
};

class IntraPub
{
public:
  IntraPub(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  {
    pub1_ = nh.advertise<A>("output1", 1);
  }

  void execute()
  {
    A::Ptr msg(new A);
    msg->header.stamp = ros::Time::now();
    pub1_.publish(msg);
  }

private:
  ros::Publisher pub1_;
};

} // namespace ros_sandbox

int main(int argc, char **argv)
{
  ros::init(argc, argv, "intra");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::AsyncSpinner spinner(20);
  spinner.start();

  std::map<std::string, std::string> intra_pub_remappings;
  intra_pub_remappings["output1"] = "foo";
  ros::NodeHandle intra_pub_nh(nh, "", intra_pub_remappings);
  ros::NodeHandle intra_pub_pnh(pnh, "intra_pub");
  ros_sandbox::IntraPub intra_pub(intra_pub_nh, intra_pub_pnh);

  std::map<std::string, std::string> intra_sub_remappings;
  intra_sub_remappings["input1"] = "foo";
  ros::NodeHandle intra_sub_nh(nh, "intra_sub", intra_sub_remappings);
  ros::NodeHandle intra_sub_pnh(pnh, "intra_sub", intra_sub_remappings);
  ros_sandbox::IntraSub intra_sub(intra_sub_nh, intra_sub_pnh);

  ros::Rate r(1);
  while (ros::ok())
  {
    ROS_INFO("Main loop running");
    intra_pub.execute();
    r.sleep();
  }

  return 0;
}

