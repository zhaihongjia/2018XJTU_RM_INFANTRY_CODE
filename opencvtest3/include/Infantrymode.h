// Generated by gencpp from file serial_common/Infantrymode.msg
// DO NOT EDIT!


#ifndef SERIAL_COMMON_MESSAGE_INFANTRYMODE_H
#define SERIAL_COMMON_MESSAGE_INFANTRYMODE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace serial_common
{
template <class ContainerAllocator>
struct Infantrymode_
{
  typedef Infantrymode_<ContainerAllocator> Type;

  Infantrymode_()
    : mode(0)  {
    }
  Infantrymode_(const ContainerAllocator& _alloc)
    : mode(0)  {
  (void)_alloc;
    }



   typedef int32_t _mode_type;
  _mode_type mode;





  typedef boost::shared_ptr< ::serial_common::Infantrymode_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::serial_common::Infantrymode_<ContainerAllocator> const> ConstPtr;

}; // struct Infantrymode_

typedef ::serial_common::Infantrymode_<std::allocator<void> > Infantrymode;

typedef boost::shared_ptr< ::serial_common::Infantrymode > InfantrymodePtr;
typedef boost::shared_ptr< ::serial_common::Infantrymode const> InfantrymodeConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::serial_common::Infantrymode_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::serial_common::Infantrymode_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace serial_common

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'serial_common': ['/home/xjturm/serial_ws/src/serial_common/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::serial_common::Infantrymode_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::serial_common::Infantrymode_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::serial_common::Infantrymode_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::serial_common::Infantrymode_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::serial_common::Infantrymode_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::serial_common::Infantrymode_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::serial_common::Infantrymode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff63f6ea3c3e9b7504b2cb3ee0a09d92";
  }

  static const char* value(const ::serial_common::Infantrymode_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xff63f6ea3c3e9b75ULL;
  static const uint64_t static_value2 = 0x04b2cb3ee0a09d92ULL;
};

template<class ContainerAllocator>
struct DataType< ::serial_common::Infantrymode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "serial_common/Infantrymode";
  }

  static const char* value(const ::serial_common::Infantrymode_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::serial_common::Infantrymode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 mode\n\
";
  }

  static const char* value(const ::serial_common::Infantrymode_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::serial_common::Infantrymode_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mode);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Infantrymode_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::serial_common::Infantrymode_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::serial_common::Infantrymode_<ContainerAllocator>& v)
  {
    s << indent << "mode: ";
    Printer<int32_t>::stream(s, indent + "  ", v.mode);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SERIAL_COMMON_MESSAGE_INFANTRYMODE_H
