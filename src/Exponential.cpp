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

	this->p_nh_.param<bool>("to_reset", this->to_reset, false);
	
	if(this->p_nh_.param("rejection", rejection) == false) {
		this->has_rejection_ = false;
	} else {
		this->setrejection(rejection);
	}

	std::string tstf = "1.0, 1.0";
    ros::param::param("~threshold_final", tstf, tstf);
    this->thresholds_final_ = this->string2vector_converter(tstf);

	// subscribe to the event_bus
	this->subevt_  = this->p_nh_.subscribe("/events/bus", 1, &Exponential::on_received_neuroevent, this);

	// Bind dynamic reconfigure callback
	this->recfg_callback_type_ = boost::bind(&Exponential::on_request_reconfigure, this, _1, _2);
	this->recfg_srv_.setCallback(this->recfg_callback_type_);

	// Bind dynamic reconfigure callback
	ros::NodeHandle node_handle("~thresholds_margin");
	dyncfg_exponential_margin *recfg_srv_margin = new dyncfg_exponential_margin(node_handle);
	this->recfg_callback_type_margin_ = boost::bind(&Exponential::on_request_reconfigure_margin, this, _1, _2);
	recfg_srv_margin->setCallback(this->recfg_callback_type_margin_);

	return true;
}

Eigen::VectorXf Exponential::apply(const Eigen::VectorXf& input) {

	if(input.size() != 2) {
		ROS_WARN("[%s] Input size is not 2: only 2-class input is allowed", this->name().c_str()); 
		return this->data_;
	}

	if(this->has_rejection_ == true && input.maxCoeff() <= this->rejection_)
		return this->data_;

	Eigen::VectorXf old_data_ = this->data_;

	this->data_ = this->data_ * this->alpha_ + input * (1 - this->alpha_);

	if (this->data_[0] > this->thresholds_final_[0] || this->data_[1] > this->thresholds_final_[1] )
		this->data_ = old_data_;

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

std::vector<double> Exponential::string2vector_converter(std::string msg){
	// If possible, always prefer std::vector to naked array
  std::vector<double> v;

	msg.replace(msg.find(", "), 2, " ");

  // Build an istream that holds the input string
  std::istringstream iss(msg);

  // Iterate over the istream, using >> to grab floats
  // and push_back to store them in the vector
  std::copy(std::istream_iterator<double>(iss),
        std::istream_iterator<double>(),
        std::back_inserter(v));

  // Put the result on standard out
  std::copy(v.begin(), v.end(),
        std::ostream_iterator<double>(std::cout, ", "));
  std::cout << "\n";
  return v;
	
}

void Exponential::on_received_neuroevent(const rosneuro_msgs::NeuroEvent& msg){
	if(msg.event == 1024+0x8000 && this->to_reset){
		reset();
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

void Exponential::on_request_reconfigure_margin(rosneuro_config_exponentialmargin &config, uint32_t level) {
	this->thresholds_final_ = {config.thfl, config.thfr};
}

}
}
