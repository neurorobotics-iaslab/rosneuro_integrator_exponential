#ifndef ROSNEURO_INTEGRATOR_EXPONENTIAL_H_
#define ROSNEURO_INTEGRATOR_EXPONENTIAL_H_

#include <Eigen/Dense>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include "rosneuro_integrator/GenericIntegrator.h"


namespace rosneuro {

class Exponential : public GenericIntegrator {

	public:
		Exponential(void);
		~Exponential(void);


		bool configure(void);
		Eigen::VectorXf apply(const Eigen::VectorXf& input);
		bool reset(void);


	private:
		Eigen::VectorXf uniform_vector(float value);

	private:
		ros::NodeHandle p_nh_;
		float alpha_;
		Eigen::Vector2f data_;
		


};

PLUGINLIB_EXPORT_CLASS(rosneuro::Exponential, rosneuro::GenericIntegrator)

}

#endif
