// Anthony Garcia

#pragma once

#include "rclcpp/rclcpp.hpp"

namespace svg_mppi {

class SVGMPPIPlannerROS : public rclcpp::Node {
private:
    // timer
    rclcpp::TimerBase::SharedPtr timer_;

    /*
    * be called in constant period of timer
    */
    void timer_callback();

public:
    SVGMPPIPlannerROS();
    ~SVGMPPIPlannerROS() {};
};
}