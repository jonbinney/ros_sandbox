/**
 * Tests use of shared pointers to members of objects which
 * are held by another shared pointer.
 */


#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <iostream>

struct BarStruct
{
  int y;
};

struct FooStruct
{
  int x;
  BarStruct bar;

  ~FooStruct() {
    std::cout << "Foo destructed\n";
  }
};

int main(int argc, char **argv)
{
  std::cout << "Creating parent pointer\n";
  boost::shared_ptr<FooStruct> parent_ptr(new FooStruct());

  // Create a pointer to parent_ptr->bar that knows that it is handling the same object
  // as parent_ptr
  std::cout << "Creating child pointer\n";
  boost::shared_ptr<BarStruct> child_ptr(parent_ptr, &parent_ptr->bar);

  std::cout << "Deleting parent pointer\n";
  parent_ptr.reset();

  // No remaining pointers to the FooStruct that we allocated, but it doesn't get
  // deleted because child_ptr still points to a member of it (and we told child_ptr
  // about the parent object when we created it)

  std::cout << "Deleting child pointer\n";
  child_ptr.reset();

  // FooStruct gets deleted here

  std::cout << "Exiting\n";
  return 0;
}

