	// Author: Stuart Glaser && Rob Janssen && Janno Lunenburg
	
	#include <ros/ros.h>
	#include <actionlib/server/action_server.h>
	
	#include <control_msgs/FollowJointTrajectoryAction.h>

    #include <sensor_msgs/JointState.h>
	#include <std_msgs/Float64.h>

    #include <diagnostic_msgs/DiagnosticArray.h>
	
	using namespace std;

	class JointTrajectoryExecuter
	{
	private:
	    typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JTAS;
	    typedef JTAS::GoalHandle GoalHandle;
	    std::vector<double> ref_pos_, cur_pos_;
	public:
	    JointTrajectoryExecuter(ros::NodeHandle &n) :
	        node_(n),
	        action_server_(node_, "joint_trajectory_action",
	                       boost::bind(&JointTrajectoryExecuter::goalCB, this, _1),
	                       boost::bind(&JointTrajectoryExecuter::cancelCB, this, _1),
	                       false),
	        has_active_goal_(false),
	        current_point(0)
	    {
	        using namespace XmlRpc;
	        ros::NodeHandle pn("~");
	
	        // Gets all of the joints
	        XmlRpc::XmlRpcValue joint_names;
	        if (!pn.getParam("joint_names", joint_names))
	        {
	            ROS_FATAL("No joints given. (namespace: %s)", pn.getNamespace().c_str());
	            exit(1);
	        }
	        if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
	        {
	            ROS_FATAL("Malformed joint specification.  (namespace: %s)", pn.getNamespace().c_str());
	            exit(1);
	        }
	        for (int i = 0; i < joint_names.size(); ++i)
	        {
	            XmlRpcValue &name_value = joint_names[i];
	            if (name_value.getType() != XmlRpcValue::TypeString)
	            {
	                ROS_FATAL("Array of joint names should contain all strings.  (namespace: %s)",
	                          pn.getNamespace().c_str());
	                exit(1);
	            }
	
                joint_index_[(std::string)name_value] = joint_names_.size();
                joint_names_.push_back((std::string)name_value);

	        }
	
	        pn.param("constraints/goal_time", goal_time_constraint_, 0.0);
	
	        // Gets the constraints for each joint.
	        for (size_t i = 0; i < joint_names_.size(); ++i)
	        {
	            std::string ns = std::string("constraints/") + joint_names_[i];
	            double ig, fg,t;
	            pn.param(ns + "/intermediate_goal", ig, -1.0);
	            pn.param(ns + "/final_goal", fg, -1.0);
	            pn.param(ns + "/trajectory", t, -1.0);
	            intermediate_goal_constraints_[joint_names_[i]] = ig;
	            final_goal_constraints_[joint_names_[i]] = fg;
	            trajectory_constraints_[joint_names_[i]] = t;
	        }
	        ///ROS_INFO("Intermediate goal constraints %f, %f, %f, %f, %f, %f, %f, %f", intermediate_goal_constraints_[0], intermediate_goal_constraints_[1], intermediate_goal_constraints_[2], intermediate_goal_constraints_[3], intermediate_goal_constraints_[4], intermediate_goal_constraints_[5], intermediate_goal_constraints_[6], intermediate_goal_constraints_[7]);
	        ///ROS_INFO("Final goal constraints        %f, %f, %f, %f, %f, %f, %f, %f", final_goal_constraints_[0], final_goal_constraints_[1], final_goal_constraints_[2], final_goal_constraints_[3], final_goal_constraints_[4], final_goal_constraints_[5], final_goal_constraints_[6], final_goal_constraints_[7]);
	        ///ROS_INFO("Trajectory constraints        %f, %f, %f, %f, %f, %f, %f, %f", trajectory_constraints_[0], trajectory_constraints_[1], trajectory_constraints_[2], trajectory_constraints_[3], trajectory_constraints_[4], trajectory_constraints_[5], trajectory_constraints_[6], trajectory_constraints_[7]);
	
	        // Here we start sending the references
            pub = node_.advertise<sensor_msgs::JointState>("/references", 1);
            torso_pub = node_.advertise<sensor_msgs::JointState>("/amigo/torso/references",1);
	        // Here we start listening for the measured positions
            sub = node_.subscribe("/measurements", 1, &JointTrajectoryExecuter::armCB, this);
            torso_sub = node_.subscribe("/amigo/torso/measurements", 1, &JointTrajectoryExecuter::torsoCB, this);

            // Diagnostics sub
            diag_sub = node_.subscribe("/hardware_status", 1, &JointTrajectoryExecuter::diagnosticsCB, this);
	
	        ref_pos_.resize(joint_names_.size());
	        cur_pos_.resize(joint_names_.size());

            // Start with hardware status OK
            torso_status = 2;
            arm_status = 2;
            torso_diag_name = "spindle"; // ToDo: don't hardcode
            std::string leftstr = "left";
            std::string rightstr= "right";
            for (unsigned int i = 0; i < joint_names_.size(); i++ ) {
                if (joint_names_[i].find(leftstr) != std::string::npos) {
                    arm_diag_name = "left_arm";
                    break;
                } else if (joint_names_[i].find(rightstr) != std::string::npos) {
                    arm_diag_name = "right_arm";
                    break;
                }
            }
            ROS_INFO("Torso diag name = %s, arm diag name = %s", torso_diag_name.c_str(), arm_diag_name.c_str());
	
	        action_server_.start();
	    }
	
	    ~JointTrajectoryExecuter()
	    {
	        pub.shutdown();
	        sub.shutdown();
	    }
	
	private:
	
	    void goalCB(GoalHandle gh)
	    {
	        current_point = 0;
	        now = ros::Time::now();
	
	        // Cancels the currently active goal.
	        if (has_active_goal_)
	        {
	            // Stops the controller.

                // spindle
                sensor_msgs::JointState torso_msg;
                torso_msg.name.push_back(joint_names_[0]);
                torso_msg.position.push_back(cur_pos_[0]);
                torso_pub.publish(torso_msg);

                // arm
                sensor_msgs::JointState arm_msg;
                for (uint i = 0; i < 7; i++) {
                    arm_msg.name.push_back(joint_names_[i+1]);
                    arm_msg.position.push_back(cur_pos_[i+1]);
                }
                pub.publish(arm_msg);
	
	            // Marks the current goal as canceled.
	            active_goal_.setCanceled();
	            has_active_goal_ = false;
	            ROS_WARN("Canceling previous goal");
	        }
	
	        // By default: spindle is not used, only if explicitly stated in message
	        ///goal_includes_spindle_ = false;
	        number_of_goal_joints_ = 7;
	        for (uint i = 0; i < gh.getGoal()->trajectory.joint_names.size(); i++) {
                if (!std::strcmp(gh.getGoal()->trajectory.joint_names[i].c_str(),"torso_joint")) number_of_goal_joints_ = 8;
	        }
	        
	        // Check feasibility of arm joint goals
	        uint m = 1;
	        for (uint i = number_of_goal_joints_-7; i < joint_names_.size(); ++i) {
				double ref = gh.getGoal()->trajectory.points[0].positions[i];
				if (ref > 3.14 || ref < -3.14) {
					ROS_WARN("Reference for joint %i is %f but should be between %f and %f.",i,ref,-3.14,3.14);
					gh.setRejected();
	                has_active_goal_=false;
	                return;
				}
			}
				
				
				
				
                //ROS_INFO("Number of goal joints = %i",number_of_goal_joints_);
	        gh.setAccepted();
	        active_goal_ = gh;
	        has_active_goal_ = true;

            // Start by assuming hardware works
            arm_status = 2;
            torso_status = 2;
	
	    }
	
	    void cancelCB(GoalHandle gh)
	    {
	        if (active_goal_ == gh)
	        {
	            // Stops the controller.

                //spindle
                sensor_msgs::JointState torso_msg;
                torso_msg.name.push_back(joint_names_[0]);
                torso_msg.position.push_back(cur_pos_[0]);
                torso_pub.publish(torso_msg);

                // arm
                sensor_msgs::JointState arm_msg;
                for (uint i = 0; i < 7; i++) {
                    arm_msg.name.push_back(joint_names_[i+1]);
                    arm_msg.position.push_back(cur_pos_[i+1]);
                }
                pub.publish(arm_msg);
	
	            // Marks the current goal as canceled.
	            active_goal_.setCanceled();
	            has_active_goal_ = false;
	        }
	    }
	
	
	    ros::NodeHandle node_;
	    JTAS action_server_;
	    ros::Publisher pub;
	    ros::Subscriber sub;
	
	    ros::Publisher torso_pub;
	    ros::Subscriber torso_sub;
        ros::Subscriber diag_sub;

        unsigned int torso_status, arm_status;
        std::string torso_diag_name; // Name of the torso in the diagnostics message array
        std::string arm_diag_name; // Name of the arm in the diagnostics message array
	
	    ros::Time now;
	
	    bool has_active_goal_;
	    int current_point;
	    GoalHandle active_goal_;
	    ///bool goal_includes_spindle_;
	    uint number_of_goal_joints_;
	
	    std::vector<std::string> joint_names_;
        std::map<std::string, unsigned int> joint_index_;
	    std::map<std::string,double> intermediate_goal_constraints_;
	    std::map<std::string,double> final_goal_constraints_;
	    std::map<std::string,double> trajectory_constraints_;
	    double goal_time_constraint_;
	
        void armCB(const sensor_msgs::JointState& joint_meas)
	    {

            for(unsigned int i = 0; i < joint_meas.name.size(); ++i) {
                std::map<std::string, unsigned int>::iterator it_joint = joint_index_.find(joint_meas.name[i]);
                if (it_joint != joint_index_.end()) {
                    cur_pos_[it_joint->second] = joint_meas.position[i];
                } else {
                    ROS_ERROR("Unknown joint name: %s", joint_meas.name[i].c_str());
                }
            }

	        // If no active goal --> Do nothing
	        if (!has_active_goal_)
	            return;
	
	        controllerCB();

	    }
	
        void torsoCB(const sensor_msgs::JointState& torso_meas) {
	
            cur_pos_[0] = torso_meas.position[0];
	        ///ROS_INFO("Torso message copied");
	
	        ///ROS_INFO("Torso message received");
	        // If no active goal --> do nothing
	        if (!has_active_goal_)
	            return;
	
	        controllerCB();
	
	    }
	
	    void controllerCB() {
	
	        int i=0,converged_joints=0;
	        float abs_error=0.0;

            // Check hardware status
            // ToDo: are we happy with this?
            if ( arm_status == 2 && torso_status == 2 ) {
                //ROS_INFO("Hardware status OK");
            } else if ( arm_status == 4 || torso_status == 4 ) {
                ROS_WARN("Arm (%u) or torso (%u) is in error, joint trajectory goal cannot be reached, aborting", arm_status, torso_status);
                active_goal_.setAborted();
                has_active_goal_=false;
                return;
            } else if ( arm_status == 0 || torso_status == 0 ) {
                ROS_WARN("Arm (%u) or torso (%u) is stale, joint trajectory goal cannot be reached, aborting", arm_status, torso_status);
                active_goal_.setAborted();
                has_active_goal_=false;
                return;
            } else if ( arm_status == 3 || torso_status == 3 ) {
                ROS_WARN("Arm (%u) or torso (%u) is still homing, joint trajectory goal may not be reached", arm_status, torso_status);
            } else if ( arm_status == 1 || torso_status == 1 ) {
                ROS_WARN("Arm (%u) or torso (%u) is in idle, joint trajectory goal cannot be reached, aborting", arm_status, torso_status);
                active_goal_.setAborted();
                has_active_goal_=false;
                return;
            }
	
	        // Check if the time constraint is not violated
	        if(ros::Time::now().toSec() > goal_time_constraint_ + now.toSec())
	        {
	            ROS_WARN("Aborting because the time constraint was violated");
	            for (i = 0; i < (int)joint_names_.size(); ++i)
	            {
					abs_error = fabs(ref_pos_[i] - cur_pos_[i]);
					ROS_WARN("Error %s = %f (intermediate constraint = %f, final constraint = %f",joint_names_[i].c_str(),abs_error,intermediate_goal_constraints_[joint_names_[i]],final_goal_constraints_[joint_names_[i]]);
				}
	            active_goal_.setAborted();
	            has_active_goal_=false;
	            return;
	        }
	
	        ///ROS_INFO("Number of joints received goal = %i",active_goal_.getGoal()->trajectory.joint_names.size());
	        ///for (uint ii = 0; ii < active_goal_.getGoal()->trajectory.joint_names.size(); ii++) ROS_INFO("Joint name = %s",active_goal_.getGoal()->trajectory.joint_names[ii].c_str());
	
	        for (i = 0; i < (int)joint_names_.size(); ++i)
	        {
	            ///joint_ref.time.data   = ros::Time::now().toSec() + active_goal_.getGoal()->trajectory.points[current_point].time_from_start.toSec();
	            ///joint_ref.pos[i].data = active_goal_.getGoal()->trajectory.points[current_point].positions[i];
	            //joint_ref.vel[i].data = active_goal_.getGoal()->trajectory.points[current_point].velocities[i];
	            //joint_ref.acc[i].data = active_goal_.getGoal()->trajectory.points[current_point].accelerations[i];
	            if (number_of_goal_joints_ == 8) {
					ref_pos_[i] = active_goal_.getGoal()->trajectory.points[current_point].positions[i];
				}
				else if (number_of_goal_joints_ == 7) {
					ref_pos_[i+1] = active_goal_.getGoal()->trajectory.points[current_point].positions[i];
					// Set ref_pos_[0] to cur_pos_[0] to make sure the error equals zero
					ref_pos_[0] = cur_pos_[0];
				}
	
	            abs_error = fabs(ref_pos_[i] - cur_pos_[i]);
	            ROS_DEBUG("%s: r: %f\t q: %f\t e: %f",joint_names_[i].c_str(), ref_pos_[i], cur_pos_[i], abs_error);
	
	            if(abs_error > trajectory_constraints_[joint_names_[i]]) {
	                ROS_WARN("Aborting because the trajectory constraint was violated");
	                for (unsigned int j = 0; j < number_of_goal_joints_; j++) {
						if ( fabs(ref_pos_[j] - cur_pos_[j]) > intermediate_goal_constraints_[joint_names_[j]]) {
							ROS_WARN("Error joint %s = %f exceeds intermediate joint constraint (%f)",joint_names_[j].c_str(),ref_pos_[j] - cur_pos_[j],intermediate_goal_constraints_[joint_names_[j]]);
						}
						else if ( fabs(ref_pos_[j] - cur_pos_[j]) > final_goal_constraints_[joint_names_[j]]) {
							ROS_WARN("Error joint %s = %f exceeds final joint contraint (%f)",joint_names_[j].c_str(),ref_pos_[j] - cur_pos_[j],final_goal_constraints_[joint_names_[j]]);
						}
					
	                    active_goal_.setAborted();
	                    has_active_goal_=false;
	                    return;
	                }
	            }
	
	            // Check if this joint has converged
	            if(current_point < ((int)active_goal_.getGoal()->trajectory.points.size()-1))
	            {
	                if(abs_error < intermediate_goal_constraints_[joint_names_[i]])
	                {
	                    converged_joints = converged_joints + 1;
	                }
	            }
	            else
	            {
	                if(abs_error < final_goal_constraints_[joint_names_[i]])
	                {
	                    converged_joints = converged_joints + 1;
	                }
	            }
	        }

            // Joint trajectory action should work for both seven (old situation, only arm) and eight (new situation, incl torso) joints

            // Only publish torso if requested
            if( number_of_goal_joints_ == 8) {
                sensor_msgs::JointState torso_msg;
                torso_msg.name.push_back("torso_joint");
                torso_msg.position.push_back(ref_pos_[0]);
                torso_pub.publish(torso_msg);
            }

            // Always publish arm msg
            sensor_msgs::JointState arm_msg;
            for (unsigned int i = 0; i < 7; i++) {
                arm_msg.name.push_back(joint_names_[i+1]);
                arm_msg.position.push_back(ref_pos_[i+1]);
            }
            pub.publish(arm_msg);

	        ///ROS_INFO("Publishing done");
	
	        //if(converged_joints==(int)number_of_goal_joints_)
	        if (converged_joints==(int)joint_names_.size())
	        {
	            now = ros::Time::now();
	            current_point = current_point + 1;
	        }
	
	        if(current_point==(int)active_goal_.getGoal()->trajectory.points.size())
	        {
	            active_goal_.setSucceeded();
	            has_active_goal_ = false;
	        }
	
		ROS_DEBUG("Converged joints = %i of %i, current_point = %i of %i", converged_joints, (int)number_of_goal_joints_, current_point, (int)active_goal_.getGoal()->trajectory.points.size());
	    }

        void diagnosticsCB(const diagnostic_msgs::DiagnosticArray& diag_array) {
            // Only process data if there is an active goal
            if (has_active_goal_) {
                // Loop through message
                for (unsigned int i = 0; i < diag_array.status.size(); i++ ) {
                    // Check if there is a torso or an arm status status
                    if (diag_array.status[i].name == torso_diag_name) {
                        torso_status = diag_array.status[i].level;
                    } else if (diag_array.status[i].name == arm_diag_name) {
                        arm_status = diag_array.status[i].level;
                    }
                }
            //ROS_INFO("Arm status %s = %u, torso status %s = %u", arm_diag_name.c_str(), arm_status, torso_diag_name.c_str(), torso_status);
            }
        }
	
	};
	
	
	int main(int argc, char** argv)
	{
	    ros::init(argc, argv, "joint_trajectory_action_node");
	    ros::NodeHandle node;//("~");
	    JointTrajectoryExecuter jte(node);
	
	    ros::spin();
	
	    return 0;
	}
	
	
