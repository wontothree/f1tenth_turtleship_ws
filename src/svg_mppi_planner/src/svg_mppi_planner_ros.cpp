#include "svg_mppi_planner/svg_mppi_planner_ros.hpp"

namespace svg_mppi { 
SVGMPPIPlannerROS::SVGMPPIPlannerROS() : Node("svg_mppi_planner_node")
{
    svg_mppi_pointer_ = std::make_unique<svg_mppi::planning::SVGMPPI>();
}

void SVGMPPIPlannerROS::timer_callback()
{
}

}