#include "rosneuro_integrator_exponential/Exponential.h"

namespace rosneuro {

Exponential::Exponential(void) : p_nh_("~") {
	this->setname("exponential");
	this->data_ = this->uniform_vector(0.5f);
}

Exponential::~Exponential(void) {
}

bool Exponential::configure(void) {
	float alpha;
	
	this->p_nh_.param<float>("alpha", alpha, this->alpha_default_);
	this->setalpha(alpha);

	return true;
}

Eigen::VectorXf Exponential::apply(const Eigen::VectorXf& input) {

	if(input.size() != 2) {
		ROS_WARN("[%s] Input size is not 2: only 2-class input is allowed", this->name().c_str()); 
		return this->data_;
	}


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
		ROS_INFO("[%s] Alpha set to %f\n", this->name().c_str(), this->alpha_);
	}
}

}
