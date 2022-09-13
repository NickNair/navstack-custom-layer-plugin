#include <pluginlib/class_list_macros.h>
#include <simple_layer/simple_layer.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace simple_layer_namespace {

SimpleLayer::SimpleLayer() {}

// Function to set up the layer when it is initialized, usually as a callback
void SimpleLayer::onInitialize() {
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  // This layer uses the GenericPluginConfig which consists of only a flag
  // called enabled, for easy enabling and disabling of this particular layer.
  // You can create your own custom dynamic_reconfigure configuration and insert
  // it instead.
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType
      cb = boost::bind(&SimpleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

// Function to enable this class's callback methods to be used
void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig& config,
                                uint32_t level) {
  enabled_ = config.enabled;
}

// The updateBounds method does not change the costmap just yet. It just defines
// the area that will need to be updated. We calculate the point we want to
// change (mark_x_, mark_y_) and then expand the min/max bounds to be sure it
// includes the new point.
void SimpleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                               double* min_x, double* min_y, double* max_x,
                               double* max_y) {
  if (!enabled_) return;

  std::cout<<"Hi!\n x:"<<robot_x<<"\n y:"<<robot_y<<"\n";

  mark_x_ = robot_x + 2*cos(robot_yaw);
  mark_y_ = robot_y + 2*sin(robot_yaw);

  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
}

// First, we calculate which grid cell our point is in using worldToMap. Then we
// set the cost of that cell. Pretty simple.
void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i,
                              int min_j, int max_i, int max_j) {
  if (!enabled_) return;
  unsigned int mx;
  unsigned int my;
  if (master_grid.worldToMap(mark_x_, mark_y_, mx, my)) {
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }
  if (master_grid.worldToMap(-mark_x_, mark_y_, mx, my)) {
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }
  if (master_grid.worldToMap(-mark_x_, -mark_y_, mx, my)) {
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }
  if (master_grid.worldToMap(mark_x_, -mark_y_, mx, my)) {
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }
}

}  // end namespace