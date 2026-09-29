#include "qube_servo2_gui/core/services/waveform_generator.hpp"
#include <algorithm>
#include <cmath>
namespace qube_servo2::gui::models {
QString WaveformConfig::name() const { switch(type){case WaveformType::Constant:return "Constant";case WaveformType::Step:return "Step";case WaveformType::Ramp:return "Ramp";case WaveformType::Pulse:return "Pulse";case WaveformType::Square:return "Square";case WaveformType::Triangle:return "Triangular";case WaveformType::Sine:return "Sinusoidal";case WaveformType::Exponential:return "Exponential";case WaveformType::Chirp:return "Chirp";} return "Unknown"; }
QString WaveformConfig::summary() const { return QString("%1 | A=%2 | offset=%3 | f=%4 Hz | delay=%5 s | duration=%6 s | rate=%7 Hz").arg(name()).arg(amplitude,0,'f',3).arg(offset,0,'f',3).arg(frequency_hz,0,'f',3).arg(delay_s,0,'f',3).arg(duration_s,0,'f',3).arg(publish_rate_hz); }
}
namespace qube_servo2::gui::services { namespace {constexpr double kPi=3.14159265358979323846;}
void WaveformGenerator::configure(const models::WaveformConfig& c) noexcept{config_=c;} const models::WaveformConfig& WaveformGenerator::config() const noexcept{return config_;}
double WaveformGenerator::localTime_(double t) const noexcept{const double s=t-config_.delay_s;if(!config_.repeat||config_.duration_s<=0.0||s<0.0)return s;return std::fmod(s,config_.duration_s);}
double WaveformGenerator::value(double t_s) const noexcept{
 if(t_s<config_.delay_s){if(config_.type==models::WaveformType::Step||config_.type==models::WaveformType::Ramp||config_.type==models::WaveformType::Exponential)return config_.initial_value;return config_.offset;}
 const double t=localTime_(t_s), phase=config_.phase_deg*kPi/180.0, f=std::max(1e-9,config_.frequency_hz);
 switch(config_.type){case models::WaveformType::Constant:return config_.offset+config_.amplitude;case models::WaveformType::Step:return config_.final_value;case models::WaveformType::Ramp:{double a=std::clamp(t/std::max(1e-9,config_.rise_time_s),0.0,1.0);return config_.initial_value+a*(config_.final_value-config_.initial_value);}case models::WaveformType::Pulse:return config_.offset+((t>=0.0&&t<std::max(0.0,config_.pulse_width_s))?config_.amplitude:0.0);case models::WaveformType::Square:{double p=1.0/f,pt=std::fmod(std::max(0.0,t),p),on=p*std::clamp(config_.duty_cycle_percent/100.0,0.0,1.0);return config_.offset+(pt<on?config_.amplitude:-config_.amplitude);}case models::WaveformType::Triangle:{double x=std::fmod(std::max(0.0,t)*f,1.0);return config_.offset+config_.amplitude*(1.0-4.0*std::abs(x-0.5));}case models::WaveformType::Sine:return config_.offset+config_.amplitude*std::sin(2.0*kPi*f*t+phase);case models::WaveformType::Exponential:{double tau=std::max(1e-9,config_.tau_s);return config_.final_value+(config_.initial_value-config_.final_value)*std::exp(-std::max(0.0,t)/tau);}case models::WaveformType::Chirp:{double T=std::max(1e-9,config_.duration_s),k=(config_.chirp_end_frequency_hz-config_.frequency_hz)/T,phi=2.0*kPi*(config_.frequency_hz*t+0.5*k*t*t)+phase;return config_.offset+config_.amplitude*std::sin(phi);}}
 return 0.0;
}}
