// Generated by gencpp from file ikid_motion_control/cmd_walk.msg
// DO NOT EDIT!


#ifndef IKID_MOTION_CONTROL_MESSAGE_CMD_WALK_H
#define IKID_MOTION_CONTROL_MESSAGE_CMD_WALK_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ikid_motion_control
{
template <class ContainerAllocator>
struct cmd_walk_
{
  typedef cmd_walk_<ContainerAllocator> Type;

  cmd_walk_()
    : sx(0.0)
    , sy(0.0)
    , var_theta(0.0)  {
    }
  cmd_walk_(const ContainerAllocator& _alloc)
    : sx(0.0)
    , sy(0.0)
    , var_theta(0.0)  {
  (void)_alloc;
    }



   typedef double _sx_type;
  _sx_type sx;

   typedef double _sy_type;
  _sy_type sy;

   typedef double _var_theta_type;
  _var_theta_type var_theta;





  typedef boost::shared_ptr< ::ikid_motion_control::cmd_walk_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ikid_motion_control::cmd_walk_<ContainerAllocator> const> ConstPtr;

}; // struct cmd_walk_

typedef ::ikid_motion_control::cmd_walk_<std::allocator<void> > cmd_walk;

typedef boost::shared_ptr< ::ikid_motion_control::cmd_walk > cmd_walkPtr;
typedef boost::shared_ptr< ::ikid_motion_control::cmd_walk const> cmd_walkConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ikid_motion_control::cmd_walk_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ikid_motion_control::cmd_walk_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ikid_motion_control::cmd_walk_<ContainerAllocator1> & lhs, const ::ikid_motion_control::cmd_walk_<ContainerAllocator2> & rhs)
{
  return lhs.sx == rhs.sx &&
    lhs.sy == rhs.sy &&
    lhs.var_theta == rhs.var_theta;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ikid_motion_control::cmd_walk_<ContainerAllocator1> & lhs, const ::ikid_motion_control::cmd_walk_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ikid_motion_control

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ikid_motion_control::cmd_walk_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ikid_motion_control::cmd_walk_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ikid_motion_control::cmd_walk_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ikid_motion_control::cmd_walk_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ikid_motion_control::cmd_walk_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ikid_motion_control::cmd_walk_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ikid_motion_control::cmd_walk_<ContainerAllocator> >
{
  static const char* value()
  {
    return "642b1806da11ae8975a1c844cb320e6d";
  }

  static const char* value(const ::ikid_motion_control::cmd_walk_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x642b1806da11ae89ULL;
  static const uint64_t static_value2 = 0x75a1c844cb320e6dULL;
};

template<class ContainerAllocator>
struct DataType< ::ikid_motion_control::cmd_walk_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ikid_motion_control/cmd_walk";
  }

  static const char* value(const ::ikid_motion_control::cmd_walk_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ikid_motion_control::cmd_walk_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 sx\n"
"float64 sy\n"
"float64 var_theta\n"
;
  }

  static const char* value(const ::ikid_motion_control::cmd_walk_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ikid_motion_control::cmd_walk_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.sx);
      stream.next(m.sy);
      stream.next(m.var_theta);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct cmd_walk_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ikid_motion_control::cmd_walk_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ikid_motion_control::cmd_walk_<ContainerAllocator>& v)
  {
    s << indent << "sx: ";
    Printer<double>::stream(s, indent + "  ", v.sx);
    s << indent << "sy: ";
    Printer<double>::stream(s, indent + "  ", v.sy);
    s << indent << "var_theta: ";
    Printer<double>::stream(s, indent + "  ", v.var_theta);
  }
};

} // namespace message_operations
} // namespace ros

#endif // IKID_MOTION_CONTROL_MESSAGE_CMD_WALK_H
