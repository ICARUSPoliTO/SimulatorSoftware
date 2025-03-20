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
	
	// Check if the propeller's diameter was specified and read the value
	if(!_sdf -> HasElement("prop_diameter")){
		gzerr << "Diameter of the propeller missing!" <<std::endl;
		return;		
	}
	std::string prop_diam = _sdf->Get<std::string>("prop_diameter");
	dataPtr->prop_diam = std::stod(prop_diam)/100.0;
	
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
	
	gzlog <<"ThrustSystem configured correctly" <<std::endl;
}

// DATA RELATIVE TO APC 20.5x20.5E
double ThrustSystem::interpolate_Kt(double adv_ratio){
	double j_known[] = {0, 0.0396, 0.0793, 0.1189, 0.1585, 0.1982, 0.2378, 0.2774, 0.3171, 0.3567, 0.3963, 0.4360, 0.4756, 0.5153, 0.5549, 0.5945, 0.6342, 0.6738, 0.7134, 0.7531, 0.7927, 0.8323, 0.8720, 0.9116, 0.9512, 0.9909, 1.0305, 1.0701, 1.1098, 1.1494};
	
	double Kt_known[] = {0.1087, 0.1084, 0.1080, 0.1075, 0.1070, 0.1064, 0.1057, 0.1048, 0.1036, 0.1023, 0.1006, 0.0986, 0.0962, 0.0934, 0.0901, 0.0863, 0.0821, 0.0772, 0.0719, 0.0661, 0.0599, 0.0535, 0.0469, 0.0402, 0.0333, 0.0265, 0.0196, 0.0129, 0.0063, 0};
	
	int N = 30;
	
	if(adv_ratio > j_known[N-1]){
		return 0;
	}
	
	// Find index of closest j_known
	int ind=0;
	while(ind < N && adv_ratio < j_known[ind])
		ind++;
		
	if(j_known[ind]==adv_ratio){
		return Kt_known[ind];
	}
	
	double slope = (Kt_known[ind+1]-Kt_known[ind])/(j_known[ind+1]-j_known[ind]);
	double K_t = Kt_known[ind] + (adv_ratio-j_known[ind])*slope;
	
	return K_t;
}

void ThrustSystem::PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm)
{	
	double IAS = dataPtr->lin_vel.Length();
	
	// Apply threshold to avoid nonsensical values
	/*if(IAS < 1e3){
		IAS = 0;
	}*/
	
	double adv_ratio;
	if(IAS == 0 || dataPtr->RPM == 0){
		adv_ratio = 0;
	}
	else{
		adv_ratio = IAS*60/dataPtr->RPM;
	}
	
	double air_density = 1.225;
	//double K_t = interpolate_Kt(adv_ratio);
	
	//gzdbg <<"IAS: " <<IAS <<" RPM: " <<dataPtr->RPM <<" J: " <<adv_ratio <<std::endl;
	
	double K_t = interpolate_Kt(adv_ratio);	
	
	
	double thrust = K_t * air_density * dataPtr->RPM*dataPtr->RPM/3600 * dataPtr->prop_diam*dataPtr->prop_diam*dataPtr->prop_diam*dataPtr->prop_diam;
	gz::math::Vector3<double> bodyForce(thrust, 0, 0);
	
	// Express thrust vector in the world's frame of reference
	gz::math::Vector3<double> worldForce = this->dataPtr->pose.Rot().RotateVector(bodyForce);
	
	// Apply the thrsut
	gzdbg <<"IAS: " <<IAS <<" RPM: " <<dataPtr->RPM <<" J: " <<adv_ratio <<" Kt: " <<K_t <<" Thrust: " <<thrust <<std::endl;
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
		
		// Apply threshold to avoid nonsensical values
		if(dataPtr->RPM < 1e3)
			dataPtr->RPM = 0;
	}
	
	// Read body's velocity vector (world's frame of reference)
	this->dataPtr->lin_vel = _ecm.Component<gz::sim::components::WorldLinearVelocity>(this->dataPtr->body_link.Entity())->Data();
}
