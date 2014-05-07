/**
 * Prove to myself that there is no speed difference between accessing shared_ptr's and
 * accessing raw pointers.
 *
 * When creating/destroying shared_ptr's, an atomic operation is needed to change the reference
 * count. This should slow things down a bit (I haven't tested that case yet). But in the case
 * where we are simply accessing an object through a shared pointer, there shouldn't be any
 * atomic operation, and the speed should be about the same as using a raw pointer.
 */
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

class Simple
{
public:
  Simple() : x_(2) {}
  int getXMult(int y) const {return x_ * y;}

private:
  int x_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "shared_ptr_speed");
  ros::NodeHandle nh;
  int sum;

  boost::shared_ptr<const Simple> s(new Simple());
  int n = 100000000;

  // Raw pointer version
  sum = 0;
  const Simple *sptr = s.get();
  ros::Time p_0 = ros::Time::now();
  for(size_t ii = 0; ii < n; ii++)
  {
    sum += sptr->getXMult(ii);
  }
  ros::Time p_1 = ros::Time::now();
  ROS_INFO("Raw ptr sum: %d", sum);

  // Shared pointer version
  sum = 0;
  ros::Time sp_0 = ros::Time::now();
  for(size_t ii = 0; ii < n; ii++)
  {
    sum += s->getXMult(ii);
  }
  ros::Time sp_1 = ros::Time::now();
  ROS_INFO("Shared ptr sum: %d", sum);

  ROS_INFO("shared_ptr: %.4f seconds  raw ptr: %.4f seconds",
    (sp_1 - sp_0).toSec(),
    (p_1 - p_0).toSec());

  return 0;
}
