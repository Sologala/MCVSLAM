// Generated by gencpp from file multi_msgs/time_cost.msg
// DO NOT EDIT!


#ifndef MULTI_MSGS_MESSAGE_TIME_COST_H
#define MULTI_MSGS_MESSAGE_TIME_COST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace multi_msgs
{
template <class ContainerAllocator>
struct time_cost_
{
  typedef time_cost_<ContainerAllocator> Type;

  time_cost_()
    {
    }
  time_cost_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::multi_msgs::time_cost_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::multi_msgs::time_cost_<ContainerAllocator> const> ConstPtr;

}; // struct time_cost_

typedef ::multi_msgs::time_cost_<std::allocator<void> > time_cost;

typedef boost::shared_ptr< ::multi_msgs::time_cost > time_costPtr;
typedef boost::shared_ptr< ::multi_msgs::time_cost const> time_costConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::multi_msgs::time_cost_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::multi_msgs::time_cost_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace multi_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::multi_msgs::time_cost_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::multi_msgs::time_cost_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::multi_msgs::time_cost_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::multi_msgs::time_cost_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::multi_msgs::time_cost_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::multi_msgs::time_cost_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::multi_msgs::time_cost_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::multi_msgs::time_cost_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::multi_msgs::time_cost_<ContainerAllocator> >
{
  static const char* value()
  {
    return "multi_msgs/time_cost";
  }

  static const char* value(const ::multi_msgs::time_cost_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::multi_msgs::time_cost_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::multi_msgs::time_cost_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::multi_msgs::time_cost_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct time_cost_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::multi_msgs::time_cost_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::multi_msgs::time_cost_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // MULTI_MSGS_MESSAGE_TIME_COST_H
