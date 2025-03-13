#include <gz/sim/System.hh>
#include <sdf/sdf.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Pose.hh>


namespace thrust_system{


	/*
	Esempio di utilizzo (force e period sono temporanei per debug): 
	
		<plugin filename="ThrustSystem" name="thrust_system::ThrustSystem">
    		<propeller_link> prop </propeller_link>
			<body_link> body </body_link>
		</plugin>

	*/

	class GZ_SIM_VISIBLE ThrustSystem:
		public gz::sim::System,
		public gz::sim::ISystemConfigure,
		public gz::sim::ISystemPreUpdate,
		public gz::sim::ISystemPostUpdate
	{
		public:
			// default constructor
			ThrustSystem();
			
			// destructor
			~ThrustSystem();
			
			// Applies the thrust and the torque
			void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;
                
            // Reads the plane's speed and the prop's rpm for the next update
            void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
                
            // Assigns sdf model from the world to dataPtr
            void Configure(
    			const gz::sim::Entity &_entity,
    			const std::shared_ptr<const sdf::Element> &_sdf,
    			gz::sim::EntityComponentManager &_ecm,
    			gz::sim::EventManager &_eventMgr) override;   
        private:
        	
        	class DataHolder{
				public:
					// pointer to the propeller's link
					gz::sim::Joint prop_joint{gz::sim::kNullEntity};
					
					//Pointer to the body's link
					gz::sim::Link body_link{gz::sim::kNullEntity};
					
					// Pointer to the sdf world
					gz::sim::World world{gz::sim::kNullEntity};
					
					// Pointer to the model
					gz::sim::Model model{gz::sim::kNullEntity};
					
					// Prop's angular velocity
					double RPM;
					
					// Plane's linear velocity
					gz::math::Vector3<double> lin_vel;
					
					// Plane's world pose
					gz::math::Pose3<double> pose;
			};
					
        	// Holds relevant data, check .cc for implementation
        	std::unique_ptr<DataHolder> dataPtr;
	};
}
