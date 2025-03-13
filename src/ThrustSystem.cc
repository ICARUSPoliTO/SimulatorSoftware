#include "ThrustSystem.hh"
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/JointVelocity.hh>

#include <string>

GZ_ADD_PLUGIN(
    thrust_system::ThrustSystem,
    gz::sim::System,
    thrust_system::ThrustSystem::ISystemConfigure,
    thrust_system::ThrustSystem::ISystemPreUpdate,
    thrust_system::ThrustSystem::ISystemPostUpdate)
    
using namespace thrust_system;

// Constructor, instantiates DataHolder
ThrustSystem::ThrustSystem() : dataPtr(new DataHolder()){
}

// Destructor
ThrustSystem::~ThrustSystem(){
}

// Reads fields from plugin tag and uses them to find the joints and links of interest
void ThrustSystem::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{	

	gzdbg <<"Starting configuration\n";

	// Check if plugin is attached to model and save it in dataPtr
	this->dataPtr->model = gz::sim::Model(_entity);
  	if (!this->dataPtr->model.Valid(_ecm)){
    	gzerr << "The plugin should be attached to a model but there's none!" <<std::endl;
    	return;
 	}
	
	// Check if model is in a world and save it in dataPtr
  	this->dataPtr->world = gz::sim::World(
  		_ecm.EntityByComponents(gz::sim::components::World())
  	);
  	if (!this->dataPtr->world.Valid(_ecm)){
    	gzerr << "World entity not found" <<std::endl;
    	return;
  	}	
	
	// Check if the propeller link was specified
	if(!_sdf -> HasElement("propeller_joint")){
		gzerr << "No propeller joint!" <<std::endl;
		return;		
	}
	// Read the name of the propeller's link and get a pointer to it
	std::string propName = _sdf->Get<std::string>("propeller_joint");
	this->dataPtr->prop_joint = gz::sim::Joint(this->dataPtr->model.JointByName(_ecm, propName));
	//Enable velocity checks or WorldAngularVelocity(...) returns nullopt
	this->dataPtr->prop_joint.EnableVelocityCheck(_ecm, true);
	gzdbg <<"Prop Joint loaded correctly!\n";
	
	// Check if the body was specified
	if(!_sdf -> HasElement("body_link")){
		gzerr << "No body link!" <<std::endl;
		return;		
	}
	// Read the name of the body's link and get a pointer to it
	std::string bodyName = _sdf->Get<std::string>("body_link");
	this->dataPtr->body_link = gz::sim::Link(this->dataPtr->model.LinkByName(_ecm, bodyName));
	//Enable velocity checks or WorldAngularVelocity(...) returns nullopt
	this->dataPtr->body_link.EnableVelocityChecks(_ecm, true);
	
	// Read the pose of the body
	this->dataPtr->pose = _ecm.Component<gz::sim::components::WorldPose>(this->dataPtr->body_link.Entity())->Data();
	
	// Read prop angular velocity and convert it to RPM
	auto ang_vel = _ecm.Component<gz::sim::components::JointVelocity>(this->dataPtr->prop_joint.Entity());
	if(ang_vel->Data().empty()){
		this->dataPtr->RPM = 0;
	}
	else{
		this->dataPtr->RPM = ang_vel->Data()[0] * 60/(2*3.1415926);
	}
	gzdbg <<"RPM: " <<this->dataPtr->RPM <<std::endl;
	
	// Read body's velocity vector (world's frame of reference)
	this->dataPtr->lin_vel = _ecm.Component<gz::sim::components::WorldLinearVelocity>(this->dataPtr->body_link.Entity())->Data();
	
	gzlog <<"ThrustSystem configured correctly" <<std::endl;
}

void ThrustSystem::PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm)
{	
	//gzdbg<<"PreUpdate\n";
	
	// Compute thrust magnitude from speed and rpm (temporary)
	double force = dataPtr->RPM * 1/(this->dataPtr->lin_vel.Length()*0.5 + 1);
	gz::math::Vector3<double> bodyForce(force, 0, 0);
	
	gzdbg<<"RPM: " <<dataPtr->RPM <<std::endl;
	
	// Express force vector in the world's frame of reference
	gz::math::Vector3<double> worldForce = this->dataPtr->pose.Rot().RotateVector(bodyForce);
	
	// Apply the force
	//gzdbg <<"About to apply force with value: " <<worldForce.Length() <<std::endl;
	this->dataPtr->body_link.AddWorldForce(_ecm, worldForce);
}

void ThrustSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm)
{
	// Read the pose of the body
	this->dataPtr->pose = _ecm.Component<gz::sim::components::WorldPose>(this->dataPtr->body_link.Entity())->Data();
	
	// Read prop angular velocity and convert it to RPM
	auto ang_vel = _ecm.Component<gz::sim::components::JointVelocity>(this->dataPtr->prop_joint.Entity());
	if(ang_vel->Data().empty()){
		this->dataPtr->RPM = 0;
	}
	else{
		this->dataPtr->RPM = ang_vel->Data()[0] * 60/(2*3.1415926);
	}
	
	// Read body's velocity vector (world's frame of reference)
	this->dataPtr->lin_vel = _ecm.Component<gz::sim::components::WorldLinearVelocity>(this->dataPtr->body_link.Entity())->Data();
}
