#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
	class Generator : public WorldPlugin
	{
		public : void Load(physics::WorldPtr _parent, sdf::ElementPtr) {
			// First trying to insert something simple
			//_parent->InsertModelFile("model://bus");
		}
	};
	
	// Register this plugin with the simulator
	GZ_REGISTER_WORLD_PLUGIN(Generator)
}
