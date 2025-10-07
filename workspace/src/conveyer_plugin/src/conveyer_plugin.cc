#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <string>

namespace gazebo
{
  class ConveyorPlugin : public ModelPlugin
  {
    physics::ModelPtr model;
    physics::WorldPtr world;
    event::ConnectionPtr updateConnection;
    ignition::math::Vector3d direction;
    double velocity;

  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override
    {
      model = _parent;
      world = model->GetWorld();

      velocity = _sdf->HasElement("belt_velocity") ?
                 _sdf->Get<double>("belt_velocity") : 0.2;

      if (_sdf->HasElement("belt_direction"))
        direction = _sdf->Get<ignition::math::Vector3d>("belt_direction").Normalize();
      else
        direction = ignition::math::Vector3d(1, 0, 0);

      gzdbg << "[ConveyorPlugin] Loaded with velocity=" << velocity
            << " direction=" << direction << std::endl;

      updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ConveyorPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
      // Conveyor belt link AABB
      auto beltLink = model->GetLink("belt");
      if (!beltLink) return;

      ignition::math::AxisAlignedBox beltBox = beltLink->BoundingBox();

      for (auto &m : world->Models())
      {
        if (m == model) continue; // skip conveyor itself

        std::string modelName = m->GetName();
        if (modelName.find("cardboard_box") != 0) continue;

        auto link = m->GetLink();
        if (!link) continue;

        // Cube position
        ignition::math::Vector3d pos = link->WorldPose().Pos();
        ignition::math::Vector3d vel;

        // Check if cube X/Y is inside conveyor top area
        if (pos.X() >= beltBox.Min().X() && pos.X() <= beltBox.Max().X() &&
            pos.Y() >= beltBox.Min().Y() && pos.Y() <= beltBox.Max().Y())
        {
          // Check if cube is above the conveyor top surface (with tolerance)
          double beltTopZ = beltBox.Max().Z();
          if (pos.Z() >= beltTopZ && pos.Z() <= beltTopZ + 0.2)
          { 
            vel = direction * velocity;
          }
        }
        else {
          vel = direction * 0;
        }
        link->SetLinearVel(vel);
      }
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(ConveyorPlugin)
}

