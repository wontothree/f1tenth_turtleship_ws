// Anthony Garcia

#pragma once

#include "rclcpp/rclcpp.hpp"

#include "svg_mppi_planner/svg_mppi.hpp"

namespace svg_mppi {

class SVGMPPIPlannerROS : public rclcpp::Node {
private:
    // timer
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<svg_mppi::planning::SVGMPPI> svg_mppi_pointer_;

    svg_mppi::planning::State current_state_;
    /*
    * be called in constant period of timer
    */
    void timer_callback();

public:
    SVGMPPIPlannerROS();
    ~SVGMPPIPlannerROS() {};
};
}