#include <cstdlib>

#include <gazebo/gui/GuiEvents.hh>
#include <gazebo/gui/GuiPlugin.hh>

namespace create2_demo
{
class GazeboFpsLimiter : public gazebo::GUIPlugin
{
public:
  void Load(sdf::ElementPtr) override
  {
    const char * value = std::getenv("ROBOT_GAZEBO_GUI_FPS");
    if (value == nullptr) {
      return;
    }
    const double fps = std::strtod(value, nullptr);
    if (fps > 0.0) {
      gazebo::gui::Events::setRenderRate(fps);
    }
  }
};
}

GZ_REGISTER_GUI_PLUGIN(create2_demo::GazeboFpsLimiter)
