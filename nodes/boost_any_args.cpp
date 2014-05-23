/**
 * Create a general way to call functions which have varying numbers
 * and types of arguments.
 */
#include <map>
#include <string>
#include <iostream>
#include <boost/any.hpp>
#include <boost/shared_ptr.hpp>
#include <topic_tools/shape_shifter.h>
#include <ros_sandbox/B.h>

class B
{
public:
  ros_sandbox::B toRosMsg() const
  {
    return msg_;
  }

  ros_sandbox::B msg_;
};

class Args
{
public:
  template<class ARG_T>
  void setArg(const std::string &name, const ARG_T &value)
  {
    values_[name] = value;
  }

  template<class ARG_T>
  ARG_T getArg(const std::string &name)
  {
    return boost::any_cast<const ARG_T>(values_[name]);
  }

private:
  std::map<std::string, boost::any> values_;
};

void foo(Args &inputs, Args *outputs)
{
  B b_new = inputs.getArg<B>("b");
  b_new.msg_.x *= 2.0;
  outputs->setArg<B>("b", b_new);
}

void bar(Args &inputs, Args *outputs)
{
  B b_new = inputs.getArg<B>("b");
  b_new.msg_.x += 3.0;
  outputs->setArg<B>("b", b_new);
}

int main(int argc, char **argv)
{
  B b;
  b.msg_.x = 2.0;

  Args foo_inputs;
  Args foo_outputs;
  foo_inputs.setArg<B>("b", b);
  foo(foo_inputs, &foo_outputs);

  Args bar_inputs;
  Args bar_outputs;
  bar_inputs.setArg<B>("b", foo_outputs.getArg<B>("b"));
  bar(bar_inputs, &bar_outputs);

  std::cout << "Bar output x:" << bar_outputs.getArg<B>("b").msg_.x << std::endl;


  return 0;
}

