#include "rosneuro_integrator_exponential/Exponential.h"

namespace rosneuro {
	namespace integrator {

Exponential::Exponential(void) : p_nh_("~") {
	this->setname("exponential");
	this->data_ = this->uniform_vector(0.5f);
	this->has_rejection_ = true;
}

Exponential::~Exponential(void) {
}

bool Exponential::configure(void) {
	
	float alpha, rejection;
	
	this->p_nh_.param<float>("alpha", alpha, this->alpha_default_);
	this->setalpha(alpha);
	
	if(this->p_nh_.param("rejection", rejection) == false) {
		this->has_rejection_ = false;
	} else {
		this->setrejection(rejection);
	}

	// Bind dynamic reconfigure callback
	this->recfg_callback_type_ = boost::bind(&Exponential::on_request_reconfigure, this, _1, _2);
	this->recfg_srv_.setCallback(this->recfg_callback_type_);

	return true;
}

Eigen::VectorXf Exponential::apply(const Eigen::VectorXf& input) {

	if(input.size() != 2) {
		ROS_WARN("[%s] Input size is not 2: only 2-class input is allowed", this->name().c_str()); 
		return this->data_;
	}

	if(this->has_rejection_ == true && input.maxCoeff() <= this->rejection_)
		return this->data_;

	this->data_ = this->data_ * this->alpha_ + input * (1 - this->alpha_);

	return this->data_;
}

bool Exponential::reset(void) {
	this->data_ = this->uniform_vector(0.5f);
	return true;
}

Eigen::VectorXf Exponential::uniform_vector(float value) {
	return Eigen::Vector2f::Constant(value);
}

void Exponential::setalpha(float value) {

	if(value < 0.0f | value > 1.0f) {
		this->alpha_ = this->alpha_default_;
		ROS_INFO("[%s] Alpha value is not legal (alpha=%f). Alpha set to the default value (alpha=%f)",
				 this->name().c_str(), value, this->alpha_);
	} else {
		this->alpha_ = value;
		ROS_INFO("[%s] Alpha set to %f", this->name().c_str(), this->alpha_);
	}
}

void Exponential::setrejection(float value) {

	if(value < 0.5f | value > 1.0f) {
		this->has_rejection_ = false;
		this->alpha_ = this->alpha_default_;
		ROS_INFO("[%s] Rejection value is not legal (rejection=%f)",
				 this->name().c_str(), value);
	} else {
		this->rejection_ = value;
		ROS_INFO("[%s] Rejection set to %f", this->name().c_str(), this->rejection_);
	}
}


void Exponential::on_request_reconfigure(rosneuro_config_exponential &config, uint32_t level) {

	if( std::fabs(config.alpha - this->alpha_) > 0.00001) {
		this->setalpha(config.alpha);
	}
	
	if( std::fabs(config.rejection - this->rejection_) > 0.00001) {
		this->setrejection(config.rejection);
	}
}

	}
}
