#ifndef _ROS_svea_msgs_Battery_h
#define _ROS_svea_msgs_Battery_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "svea_msgs/energy_sensor.h"

namespace svea_msgs
{

  class Battery : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef svea_msgs::energy_sensor _battery_type;
      _battery_type battery;
      typedef svea_msgs::energy_sensor _charger_type;
      _charger_type charger;
      typedef float _percentage_type;
      _percentage_type percentage;
      typedef float _net_energy_type;
      _net_energy_type net_energy;

    Battery():
      header(),
      battery(),
      charger(),
      percentage(0),
      net_energy(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->battery.serialize(outbuffer + offset);
      offset += this->charger.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_percentage;
      u_percentage.real = this->percentage;
      *(outbuffer + offset + 0) = (u_percentage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_percentage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_percentage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_percentage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->percentage);
      union {
        float real;
        uint32_t base;
      } u_net_energy;
      u_net_energy.real = this->net_energy;
      *(outbuffer + offset + 0) = (u_net_energy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_net_energy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_net_energy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_net_energy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->net_energy);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->battery.deserialize(inbuffer + offset);
      offset += this->charger.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_percentage;
      u_percentage.base = 0;
      u_percentage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_percentage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_percentage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_percentage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->percentage = u_percentage.real;
      offset += sizeof(this->percentage);
      union {
        float real;
        uint32_t base;
      } u_net_energy;
      u_net_energy.base = 0;
      u_net_energy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_net_energy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_net_energy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_net_energy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->net_energy = u_net_energy.real;
      offset += sizeof(this->net_energy);
     return offset;
    }

    virtual const char * getType() override { return "svea_msgs/Battery"; };
    virtual const char * getMD5() override { return "1753c9c92d8b1311ff4d5f64c8cc480c"; };

  };

}
#endif
