#include "SEAPlugin.hh"

using namespace gazebo;

void SEAPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Read SDF
    if (_sdf->HasElement("series-elastic"))
    {
        for (sdf::ElementPtr elem = _sdf->GetElement("series-elastic");
             elem != nullptr; elem = elem->GetNextElement("series-elastic"))
        {
            // Get SEA properties
            SEAProperties properties;
            if (elem->HasElement("name"))
                properties.name = elem->Get<std::string>("name");

            if (!elem->HasElement("motor-joint"))
            {
                gzwarn << "Invalid SDF: " 
                       << "series elastic element without motor joint"
                       << std::endl;
                continue;
            }
            std::string motorJointName = 
                elem->Get<std::string>("motor-joint");

            if (!elem->HasElement("load-joint"))
            {
                gzwarn << "Invalid SDF: " 
                       << "series elastic element without load joint"
                       << std::endl;
                continue;
            }
            std::string loadJointName = 
                elem->Get<std::string>("load-joint");

            if (!elem->HasElement("motor-inertia"))
            {
                gzwarn << 
                    "Invalid SDF: series elastic element without motor inertia"
                    << std::endl;
                continue;
            }
            properties.motorInertia = elem->Get<float>("motor-inertia");

            if (!elem->HasElement("motor-damping"))
            {
                gzwarn << 
                    "Invalid SDF: series elastic element without motor damping"
                    << std::endl;
                continue;
            }
            properties.motorDamping = elem->Get<float>("motor-damping");

            if (!elem->HasElement("motor-friction"))
            {
                gzwarn << 
                    "Invalid SDF: series elastic element without motor friction"
                    << std::endl;
                continue;
            }
            properties.motorFriction = elem->Get<float>("motor-friction");

            if (!elem->HasElement("load-inertia"))
            {
                gzwarn << 
                    "Invalid SDF: series elastic element without load inertia"
                    << std::endl;
                continue;
            }
            properties.loadInertia = elem->Get<float>("load-inertia");

            if (!elem->HasElement("load-damping"))
            {
                gzwarn << 
                    "Invalid SDF: series elastic element without load damping"
                    << std::endl;
                continue;
            }
            properties.loadDamping = elem->Get<float>("load-damping");

            if (!elem->HasElement("load-friction"))
            {
                gzwarn << 
                    "Invalid SDF: series elastic element without load friction"
                    << std::endl;
                continue;
            }
            properties.loadFriction = elem->Get<float>("load-friction");

            if (!elem->HasElement("spring-stiffness"))
            {
                gzwarn << 
                    "Invalid SDF: series elastic element without spring stiffness"
                    << std::endl;
                continue;
            }
            properties.springStiffness = 
                elem->Get<float>("spring-stiffness");

            if (elem->HasElement("motor-index"))
            {
                properties.motorJointIndex = 
                    elem->Get<unsigned int>("motor-index");
            }
            else
            {
                properties.motorJointIndex = 0;
            }

            if (elem->HasElement("load-index"))
            {
                properties.loadJointIndex = 
                    elem->Get<unsigned int>("load-index");
            }
            else
            {
                properties.loadJointIndex = 0;
            }

            // Store pointer to the motor and load joint
            physics::JointPtr motorJoint = _parent->GetJoint(motorJointName);
            if (!motorJoint)
            {
                gzwarn << "Invalid SDF: series elastic joint " 
                       << motorJointName << "does not exist!" << std::endl;
                continue;
            }
            physics::JointPtr loadJoint = _parent->GetJoint(loadJointName);
            if (!loadJoint)
            {
                gzwarn << "Invalid SDF: series elastic joint " 
                       << loadJointName << "does not exist!" << std::endl;
                continue;
            }

            this->joints.push_back(std::make_pair(motorJoint, loadJoint));
            this->actuators.push_back(properties);
        }
        // Set up a physics update callback
        this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
            boost::bind(&SEAPlugin::WorldUpdateCallback, this)));
    }
}

void SEAPlugin::WorldUpdateCallback()
{
    // Update the stored joints
    for (unsigned int i = 0; i < this->joints.size(); i++)
    {
        const int mIndex = this->actuators[i].motorJointIndex;
        const int lIndex = this->actuators[i].loadJointIndex;
        SEAProperties properties = this->actuators[i];
        physics::JointPtr mJoint = this->joints[i].first;
        physics::JointPtr lJoint = this->joints[i].second;
        double m = mJoint->GetAngle(mIndex).Radian();
        double dm = mJoint->GetVelocity(mIndex);
        double ddm = mJoint->GetForceTorque(mIndex).body2Force.GetLength() /
            properties.motorInertia;
        double l = lJoint->GetAngle(lIndex).Radian();
        double dl = lJoint->GetVelocity(lIndex);
        double ddl = lJoint->GetForceTorque(lIndex).body1Force.GetLength() /
            properties.loadInertia;
        std::cout << "m: " << m << std::endl;
        std::cout << "dm: " << dm << std::endl;
        lJoint->SetForce(lIndex, properties.motorInertia * ddm + 
            properties.motorDamping * dm + ((dm > 0) ? 1 : -1) *
            properties.motorFriction + properties.springStiffness *
            m - l);
    }
}
