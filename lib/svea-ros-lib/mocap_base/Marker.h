#ifndef _ROS_mocap_base_Marker_h
#define _ROS_mocap_base_Marker_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"

namespace mocap_base
{

  class Marker : public ros::Msg
  {
    public:
      typedef const char* _id_type;
      _id_type id;
      typedef bool _labeled_type;
      _labeled_type labeled;
      typedef bool _tracked_type;
      _tracked_type tracked;
      typedef geometry_msgs::Point _position_type;
      _position_type position;
      typedef float _residual_type;
      _residual_type residual;

    Marker():
      id(""),
      labeled(0),
      tracked(0),
      position(),
      residual(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_id = strlen(this->id);
      varToArr(outbuffer + offset, length_id);
      offset += 4;
      memcpy(outbuffer + offset, this->id, length_id);
      offset += length_id;
      union {
        bool real;
        uint8_t base;
      } u_labeled;
      u_labeled.real = this->labeled;
      *(outbuffer + offset + 0) = (u_labeled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->labeled);
      union {
        bool real;
        uint8_t base;
      } u_tracked;
      u_tracked.real = this->tracked;
      *(outbuffer + offset + 0) = (u_tracked.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->tracked);
      offset += this->position.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->residual);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_id;
      arrToVar(length_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_id-1]=0;
      this->id = (char *)(inbuffer + offset-1);
      offset += length_id;
      union {
        bool real;
        uint8_t base;
      } u_labeled;
      u_labeled.base = 0;
      u_labeled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->labeled = u_labeled.real;
      offset += sizeof(this->labeled);
      union {
        bool real;
        uint8_t base;
      } u_tracked;
      u_tracked.base = 0;
      u_tracked.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->tracked = u_tracked.real;
      offset += sizeof(this->tracked);
      offset += this->position.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->residual));
     return offset;
    }

    virtual const char * getType() override { return "mocap_base/Marker"; };
    virtual const char * getMD5() override { return "a62789b380c165fa69fbe21beba33bb6"; };

  };

}
#endif
