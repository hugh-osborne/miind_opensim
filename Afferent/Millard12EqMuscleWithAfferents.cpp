/* -------------------------------------------------------------------------- *
 *                 OpenSim:  Millard12EqMuscleWithAfferents.cpp               *
 * -------------------------------------------------------------------------- *
 */
 
//=============================================================================
// INCLUDES
//=============================================================================
#include "Millard12EqMuscleWithAfferents.h"
#include <iostream>  // remove later

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

///@cond  
const std::string Millard12EqMuscleWithAfferents::STATE_LPF_VELOCITY_NAME = "LPF_velocity";
const std::string Millard12EqMuscleWithAfferents::STATE_LPF_ACCELERATION_NAME = "LPF_acceleration";
// The below names are declared private in Millard2012EquilibriumMuscle so we have to duplicate them here. Boooo!
const string Millard12EqMuscleWithAfferents::AFF_STATE_ACTIVATION_NAME = "activation";
const string Millard12EqMuscleWithAfferents::AFF_STATE_FIBER_LENGTH_NAME = "fiber_length";
///@endcond  

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
/*
 * Default constructor.
 */
Millard12EqMuscleWithAfferents::Millard12EqMuscleWithAfferents()
{
	constructProperties();
}

/*
 * Constructor.
 */
Millard12EqMuscleWithAfferents::Millard12EqMuscleWithAfferents(const std::string &name, 
					double maxIsometricForce, double optimalFiberLength, 
					double tendonSlackLength, double pennationAngle) : 
	Super(name, maxIsometricForce, optimalFiberLength, tendonSlackLength, 
          pennationAngle)
{
	constructProperties();
}

Millard12EqMuscleWithAfferents::Millard12EqMuscleWithAfferents(const Millard2012EquilibriumMuscle& muscle)
: Super(muscle.getName(), muscle.getMaxIsometricForce(), muscle.getOptimalFiberLength(), muscle.getTendonSlackLength(), 
          muscle.getPennationAngleAtOptimalFiberLength())
{
	constructProperties();
}

/*
 * Construct and initialize properties.
 * All properties are added to the property set. Once added, they can be
 * read in and written to files.
 */
void Millard12EqMuscleWithAfferents::constructProperties()
{
	setAuthors("Sergio Verduzco from code by Ajay Seth");
	constructProperty_lpf_tau(0.001); // LPF time constant

	for (unsigned int i = 0; i < SMOOTHING_WINDOW; i++) {
		acc_approx_vels[i] = 0.0;
		acc_approx_ts[i] = 0.0;
		vel_approx_lens[i] = 0.0;
		vel_approx_ts[i] = 0.0;
	}
}

// Define new states and their derivatives in the underlying system
void Millard12EqMuscleWithAfferents::extendAddToSystem(SimTK::MultibodySystem& system) const
{
	// Allow Millard2012EquilibriumMuscle to add its states, cache, etc.
	// to the system
	Super::extendAddToSystem(system);
	
	// low-pass filtered state variables used to calculate derivatives 
	addStateVariable(STATE_LPF_VELOCITY_NAME); // fiber velocity
	addStateVariable(STATE_LPF_ACCELERATION_NAME); // fiber acceleration
}

void Millard12EqMuscleWithAfferents::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);
	
	// I'll init the state, but not from properties
	setLPFvelocity(s, 0.0);
	setLPFacceleration(s, 0.0);
}

void Millard12EqMuscleWithAfferents::extendSetPropertiesFromState(const SimTK::State& s)
{
    Super::extendSetPropertiesFromState(s);
}

void Millard12EqMuscleWithAfferents::extendConnectToModel(Model& aModel)
{
	Super::extendConnectToModel(aModel);
}

//--------------------------------------------------------------------------
// GET & SET Properties
//--------------------------------------------------------------------------
void Millard12EqMuscleWithAfferents::setLPFtau(double aLPFtau) {
	set_lpf_tau(aLPFtau);
}

//--------------------------------------------------------------------------
// GET & SET States and their derivatives
//--------------------------------------------------------------------------

double Millard12EqMuscleWithAfferents::getLPFvelocity(const SimTK::State& s) const {
	return getStateVariableValue(s, STATE_LPF_VELOCITY_NAME);
}	
void Millard12EqMuscleWithAfferents::setLPFvelocity(SimTK::State& s, double Velocity) const {
	setStateVariableValue(s, STATE_LPF_VELOCITY_NAME, Velocity);
}	
double Millard12EqMuscleWithAfferents::getLPFacceleration(const SimTK::State& s) const {
	return getStateVariableValue(s, STATE_LPF_ACCELERATION_NAME);
}
void Millard12EqMuscleWithAfferents::setLPFacceleration(SimTK::State& s, double Acceleration) const {
	setStateVariableValue(s, STATE_LPF_ACCELERATION_NAME, Acceleration);
}

//=============================================================================
// COMPUTATION
//=============================================================================
void Millard12EqMuscleWithAfferents::
computeInitialFiberEquilibrium(SimTK::State& s) const
{
	// First let the muscle find an equilibrium state
	Super::computeInitialFiberEquilibrium(s);
	
	setLPFvelocity(s, 0.0);
	// a simplifying assumption is a steady state
	setLPFacceleration(s, 0.0); 

	for (unsigned int i = 0; i < SMOOTHING_WINDOW; i++) {
		acc_approx_vels[i] = 0.0;
		acc_approx_ts[i] = 0.0;
		vel_approx_lens[i] = getFiberLength(s);
		vel_approx_ts[i] = 0.0;
	}

	spindle.computeInitialSpindleEquilibrium(s);
	GTO.initFromMuscle(s);
}

void Millard12EqMuscleWithAfferents::computeStateVariableDerivatives(const SimTK::State& s) const
{	
	// vector of the derivatives to be returned
	SimTK::Vector derivs(getNumStateVariables(), 0.0);
	int nd = derivs.size();

	SimTK_ASSERT1(nd == 4, "Millard12EqMuscleWithAfferents: Expected 4 state variables"
        " but encountered  %f.", nd);

// This is the parent's computeStateVariableDerivatives
/*--------------------------------------------------------------------
	int idx = 0;

    if (!isDisabled(s)) {
        // Activation is the first state (if it is a state at all)
        if(!get_ignore_activation_dynamics() &&
           idx+1 <= getNumStateVariables()) {
               derivs[idx] = getActivationDerivative(s);
               idx++;
        }

        // Fiber length is the next state (if it is a state at all)
        if(!get_ignore_tendon_compliance() && idx+1 <= getNumStateVariables()) {
            derivs[idx] = getFiberVelocity(s);
        }
    }
--------------------------------------------------------------------*/
// This is a "carefree" version of that:
	derivs[0] = getActivationDerivative(s);
	derivs[1] = getFiberVelocity(s);
	
	// next state is the LPF velocity
	derivs[2] = (approxFiberVelocity(s) - getLPFvelocity(s)) / getLPFtau();
	 
	// the LPF acceleration
	derivs[3] = (approxFiberAcceleration(s) - getLPFacceleration(s)) / getLPFtau();

	setStateVariableDerivativeValue(s, AFF_STATE_ACTIVATION_NAME, derivs[0]);
	setStateVariableDerivativeValue(s, AFF_STATE_FIBER_LENGTH_NAME, derivs[1]);
	setStateVariableDerivativeValue(s, STATE_LPF_VELOCITY_NAME, derivs[2]);
	setStateVariableDerivativeValue(s, STATE_LPF_ACCELERATION_NAME, derivs[3]);
}

//--------------------------------------------------------------------------
// Approximate the muscle fiber acceleration
//--------------------------------------------------------------------------
// HO : Originally approxFiberAcceleration used a more sophisticated way to estimate the
// acceleration but it doesn't play nice with a static time step which we need
// for syncing with the neural simulation so we just use the less accurate, naive, but
// faster solution just using the previous value.
double Millard12EqMuscleWithAfferents::
approxFiberVelocity(const SimTK::State& s) const
{
	double curr_time = s.getTime();
	double curr_vel = getFiberLength(s);
	const unsigned int smooth_window = SMOOTHING_WINDOW;
	double v, vs[smooth_window];
	v = 0.0;
	for (unsigned int i = 0; i < smooth_window; i++)
		vs[i] = 0.0;

	if (vel_approx_ts[smooth_window - 2] - vel_approx_ts[smooth_window - 1] > 0) {
		if (curr_time - vel_approx_ts(0) > 0.0) {
			vs[0] = (curr_vel - vel_approx_lens(0)) / (curr_time - vel_approx_ts(0));
			v += vs[0];
			for (unsigned int i = 1; i < smooth_window; i++) {
				vs[i] = (vel_approx_lens(i - 1) - vel_approx_lens(i)) / (vel_approx_ts(i - 1) - vel_approx_ts(i));
				v += vs[i];
			}

			v /= (double)smooth_window;

			for (unsigned int i = smooth_window - 1; i > 0; i--) {
				vel_approx_ts[i] = vel_approx_ts[i - 1];
				vel_approx_lens[i] = vel_approx_lens[i - 1];
			}
			vel_approx_ts[0] = curr_time;
			vel_approx_lens[0] = curr_vel;
		}
		else {
			v = 0.0;
		}
	}
	else {
		if (curr_time - vel_approx_ts(0) > 0.0) {
			vs[0] = (curr_vel - vel_approx_lens(0)) / (curr_time - vel_approx_ts(0));

			for (unsigned int i = smooth_window - 1; i > 0; i--) {
				vel_approx_ts[i] = vel_approx_ts[i - 1];
				vel_approx_lens[i] = vel_approx_lens[i - 1];
			}
			vel_approx_ts[0] = curr_time;
			vel_approx_lens[0] = curr_vel;

			v = vs[0];
		}
		else {
			v = 0.0;
		}
	}

	return v;
}

//--------------------------------------------------------------------------
// Approximate the muscle fiber acceleration
//--------------------------------------------------------------------------
// HO : Originally approxFiberAcceleration used a more sophisticated way to estimate the
// acceleration but it doesn't play nice with a static time step which we need
// for syncing with the neural simulation so we just use the less accurate, naive, but
// faster solution just using the previous value.
double Millard12EqMuscleWithAfferents::
       approxFiberAcceleration(const SimTK::State& s) const
{
	double curr_time = s.getTime();
	double curr_vel = getLPFvelocity(s);
	const unsigned int smooth_window = SMOOTHING_WINDOW;
	double v, vs[smooth_window];
	v = 0.0;
	for (unsigned int i = 0; i < smooth_window; i++)
		vs[i] = 0.0;

	if (acc_approx_ts[smooth_window - 2] - acc_approx_ts[smooth_window - 1] > 0) {
		if (curr_time - acc_approx_ts(0) > 0.0) {
			vs[0] = (curr_vel - acc_approx_vels(0)) / (curr_time - acc_approx_ts(0));
			v += vs[0];
			for (unsigned int i = 1; i < smooth_window; i++) {
				vs[i] = (acc_approx_vels(i - 1) - acc_approx_vels(i)) / (acc_approx_ts(i - 1) - acc_approx_ts(i));
				v += vs[i];
			}

			v /= (double)smooth_window;

			for (unsigned int i = smooth_window - 1; i > 0; i--) {
				acc_approx_ts[i] = acc_approx_ts[i - 1];
				acc_approx_vels[i] = acc_approx_vels[i - 1];
			}
			acc_approx_ts[0] = curr_time;
			acc_approx_vels[0] = curr_vel;
		}
		else {
			v = 0.0;
		}
	}
	else {
		if (curr_time - acc_approx_ts(0) > 0.0) {
			vs[0] = (curr_vel - acc_approx_vels(0)) / (curr_time - acc_approx_ts(0));

			for (unsigned int i = smooth_window - 1; i > 0; i--) {
				acc_approx_ts[i] = acc_approx_ts[i - 1];
				acc_approx_vels[i] = acc_approx_vels[i - 1];
			}
			acc_approx_ts[0] = curr_time;
			acc_approx_vels[0] = curr_vel;

			v = vs[0];
		}
		else {
			v = 0.0;
		}
	}

	return v;
}