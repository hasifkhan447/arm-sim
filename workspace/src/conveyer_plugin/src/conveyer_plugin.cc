#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <iostream>

namespace gazebo
{
  class HelloPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      std::cout << "[HelloPlugin] Plugin loaded for model: "
                << _parent->GetName() << std::endl;

      // Connect to the update event (called every simulation iteration)
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&HelloPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
      // Print every iteration (can be spammy, but good for testing)
      std::cout << "[HelloPlugin] Simulation update tick" << std::endl;
    }

  private:
    event::ConnectionPtr updateConnection;
  };

  // Register the plugin with Gazebo
  GZ_REGISTER_MODEL_PLUGIN(HelloPlugin)
}

