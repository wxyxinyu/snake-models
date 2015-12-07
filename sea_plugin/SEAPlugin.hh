#ifndef _GAZEBO_SEA_PLUGIN_HH_
#define _GAZEBO_SEA_PLUGIN_HH_

#include <string>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>

namespace gazebo
{
    // Properties for a model of a series elastic actuator
    class SEAProperties
    {
        public:
            // An identifier for the actuator
            std::string name;

            // Which joint indexs are to be considered
            // Note: this refers to the index of the DOF, and defaults to
            // zero
            int motorJointIndex;
            int loadJointIndex;

            // constants
            float motorInertia;
            float motorDamping;
            float motorFriction;
            float loadInertia;
            float loadDamping;
            float loadFriction;
            float springStiffness;
    };

    class SEAPlugin : public ModelPlugin
    {
        public: 
            // Initialize plugin
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        private: 
            // Callback on the world update event
            void WorldUpdateCallback();
            // A list of pairs of joints we are modelling
            std::vector<std::pair<physics::JointPtr, physics::JointPtr> >
                joints;
            // properties of the joints we are modelling
            std::vector<SEAProperties> actuators;
            // Connections to events associated with this class
            std::vector<event::ConnectionPtr> connections;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(SEAPlugin);
}

#endif
