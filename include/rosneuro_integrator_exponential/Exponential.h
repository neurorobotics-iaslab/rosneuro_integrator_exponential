#ifndef ROSNEURO_INTEGRATOR_EXPONENTIAL_H_
#define ROSNEURO_INTEGRATOR_EXPONENTIAL_H_

#include <Eigen/Dense>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>

#include "rosneuro_integrator/GenericIntegrator.h"
#include "rosneuro_integrator_exponential/ExponentialConfig.h"


namespace rosneuro {
	namespace integrator {

using rosneuro_config_exponential = rosneuro_integrator_exponential::ExponentialConfig;
using dyncfg_exponential          = dynamic_reconfigure::Server<rosneuro_config_exponential>;

class Exponential : public GenericIntegrator {

	public:
		Exponential(void);
		~Exponential(void);

		bool configure(void);
		Eigen::VectorXf apply(const Eigen::VectorXf& input);
		bool reset(void);
		void setalpha(float value);
		void setrejection(float value);


	private:
		Eigen::VectorXf uniform_vector(float value);
		void on_request_reconfigure(rosneuro_config_exponential &config, uint32_t level);

	private:
		ros::NodeHandle p_nh_;
		float alpha_;
		float rejection_;
		bool has_rejection_;
		const float alpha_default_ = 0.98f;
		Eigen::Vector2f data_;

		dyncfg_exponential recfg_srv_;
  		dyncfg_exponential::CallbackType recfg_callback_type_;
		


};

PLUGINLIB_EXPORT_CLASS(rosneuro::integrator::Exponential, rosneuro::integrator::GenericIntegrator)

	}
}

#endif
