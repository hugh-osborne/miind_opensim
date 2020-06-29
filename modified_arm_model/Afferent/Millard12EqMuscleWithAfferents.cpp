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
 * Default constructor. Not used.
 */
Millard12EqMuscleWithAfferents::Millard12EqMuscleWithAfferents()
{
	constructProperties();
	spindle.setOwnerMuscleName("no_spindle_name");
	GTO.setOwnerMuscleName("no_GTO_name");
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
	spindle.setOwnerMuscleName(getName());
	GTO.setOwnerMuscleName(getName());
	
	// Initialize the work variables to calculate the acceleration
	vel = 0;
	ts[2] = 0.0; ts[1] = -0.01; ts[0] = -0.02; 
	C0 = 0.0; C1 = 0.0;
}

Millard12EqMuscleWithAfferents::Millard12EqMuscleWithAfferents(const Millard2012EquilibriumMuscle& muscle)
: Super(muscle.getName(), muscle.getMaxIsometricForce(), muscle.getOptimalFiberLength(), muscle.getTendonSlackLength(), 
          muscle.getPennationAngleAtOptimalFiberLength())
{
	constructProperties();
	spindle.setOwnerMuscleName(getName());
	GTO.setOwnerMuscleName(getName());
	
	// Initialize the work variables to calculate the acceleration
	vel = 0;
	ts[2] = 0.0; ts[1] = -0.01; ts[0] = -0.02; 
	C0 = 0.0; C1 = 0.0;
}

/*
 * Construct and initialize properties.
 * All properties are added to the property set. Once added, they can be
 * read in and written to files.
 */
void Millard12EqMuscleWithAfferents::constructProperties()
{
	setAuthors("Sergio Verduzco from code by Ajay Seth");
	constructProperty_lpf_tau(0.01); // LPF time constant
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

	// Allowing the afferent elements to become part of the system
	spindle.extendAddToSystem(system);
	GTO.extendAddToSystem(system);
}

void Millard12EqMuscleWithAfferents::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);
	
	// I'll init the state, but not from properties
	setLPFvelocity(s, 0.0);
	setLPFacceleration(s, 0.0);

	spindle.extendInitStateFromProperties(s); // because spindle was not
	                                    // declared a subcomponent
	GTO.extendInitStateFromProperties(s);	    // ditto
										
	// Initialize the work variables to calculate the acceleration
	vel = 0;
	ts[2] = 0.0; ts[1] = -0.01; ts[0] = -0.02; 
	C0 = 0.0; C1 = 0.0;
}

void Millard12EqMuscleWithAfferents::extendSetPropertiesFromState(const SimTK::State& s)
{
    Super::extendSetPropertiesFromState(s);
	
	spindle.extendSetPropertiesFromState(s); // because spindle was not
	                                    // declared a subcomponent
	GTO.extendSetPropertiesFromState(s); // ditto
}

void Millard12EqMuscleWithAfferents::extendConnectToModel(Model& aModel)
{
	Super::extendConnectToModel(aModel);
	
	// The afferents need the name of their owner muscle
	spindle.setOwnerMuscleName(getName());
	GTO.setOwnerMuscleName(getName());
	
	// I read in Ligament.cpp that includeAsSubComponent should 
	// appear before Super::connectToModel() 
	/// I REMOVED IT. For some reason the state variables of the spindle
	/// object were not being added to the system despite this
	/// statement.
	/// includeAsSubComponent(&spindle);
	
	/// What allowed the state variables of spindle to be
	/// connected was to invoke this here.
	spindle.extendConnectToModel(aModel);
	GTO.extendConnectToModel(aModel); // ditto for GTO
	
	
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
	
	setLPFvelocity(s, getFiberVelocity(s));
	// a simplifying assumption is a steady state
	setLPFacceleration(s, 0.0); 
	
	// the spindle's initial conditions depend on the muscle,
	// so this method should be called last.
	spindle.computeInitialSpindleEquilibrium(s);
	
	// get a reasonable initial value for the GTO nonlinearity
	GTO.initFromMuscle(s);
	
	// update the work vectors assuming no acceleration
	vel[0] = vel[1] = vel[2] = getFiberVelocity(s);
	ts[2] = s.getTime();
	ts[1] = ts[2] - 0.001; ts[0] = ts[1] - 0.001;
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
	derivs[2] = (getFiberVelocity(s) - getLPFvelocity(s))/getLPFtau();
	 
	// the LPF acceleration
	derivs[3] = (approxFiberAcceleration(s) - getLPFacceleration(s)) 
	          / getLPFtau();

	setStateVariableDerivativeValue(s, AFF_STATE_ACTIVATION_NAME, derivs[0]);
	setStateVariableDerivativeValue(s, AFF_STATE_FIBER_LENGTH_NAME, derivs[1]);
	setStateVariableDerivativeValue(s, STATE_LPF_VELOCITY_NAME, derivs[2]);
	setStateVariableDerivativeValue(s, STATE_LPF_ACCELERATION_NAME, derivs[3]);
}

//--------------------------------------------------------------------------
// Approximate the muscle fiber acceleration
//--------------------------------------------------------------------------
double Millard12EqMuscleWithAfferents::
       approxFiberAcceleration(const SimTK::State& s) const
{
	double accel;   // muscle fiber acceleration 
	double curr_vel;	//  muscle fiber velocity		
	double curr_time = s.getTime();	// time in the simulation
	
	curr_vel = getLPFvelocity(s);
	
	if( curr_time > ts(2) )
	{	// vel and ts are not ahead of current time.
		// Using 4-point Fornberg's method.
		// The formula below assumes current time = 0
		ts(0) = ts(0) - curr_time;
		ts(1) = ts(1) - curr_time;
		ts(2) = ts(2) - curr_time;
		
		// calculate coefficients
		C0(0,1) = ts(1)/(ts(1)-ts(0));
		C0(1,1) = ts(0)/(ts(0)-ts(1));
		C0(0,2) = ts(2)*C0(0,1)/(ts(2)-ts(0));
		C0(1,2) = ts(2)*C0(1,1)/(ts(2)-ts(1));
		C0(2,2) = ts(1)*ts(0)/((ts(2)-ts(0))*(ts(2)-ts(1)));
		C1(0,1) = 1/(ts(0)-ts(1));
		C1(1,1) = -C1(0,1);
		C1(2,2) = ((ts(1)-ts(0))/( (ts(2)-ts(1))*(ts(2)-ts(0)) ))
		          *(C0(1,1) - ts(1)*C1(1,1));
		C1(0,3) = C0(0,2)/ts(0);
		C1(1,3) = C0(1,2)/ts(1);
		C1(2,3) = C0(2,2)/ts(2);
		C1(3,3) = ( (ts(1)-ts(2))*(ts(2)-ts(0))/(ts(0)*ts(1)*ts(2)) )
		          *(C0(2,2) - ts(2)*C1(2,2));
				  
		// use the coefficients
		accel = C1(3,3)*curr_vel + C1(2,3)*vel(2) + C1(1,3)*vel(1) + C1(0,3)*vel(0);
		
		// shift velocities and times
		vel(0) = vel(1); vel(1) = vel(2); vel(2) = curr_vel;
		ts(0) = ts(1) + curr_time; // changing to absolute times
		ts(1) = ts(2) + curr_time;
		ts(2) = curr_time;
	} 
	else  // computeStateVariableDerivatives was called before for a more advanced time
	{
		if( curr_time > ts(1) )
		{  // vel(1) and vel(0) still useful.
			// Using a 3-point rule for differentiation
			accel = ( 3*curr_vel - 4*vel(1) + vel(0) )/(curr_time - ts(0));
			
			// shift velocities and times
			vel(2) = curr_vel; ts(2) = curr_time;
		}
		else if( s.getTime() > ts(0) )
		{ // We only have one value before current time.
			// Using a 2-point rule for differentiation
			accel = (curr_vel - vel(0))/(curr_time - ts(0));
			
			// shift velocities and times
			vel(2) = curr_vel; vel(1) = vel(0);
			ts(2) = curr_time; ts(1) = ts(0); ts(0) = ts(1) - 1.0e-5;
		}
		else // we have no data to do this calculation
		{
			accel = getLPFacceleration(s);
			vel(2) = curr_vel; ts(2) = curr_time;
			vel(1) = vel(2); ts(1) = ts(2) - 1.0e-5;
			vel(0) = vel(1); ts(0) = ts(1) - 1.0e-5;
			//std::cout << "computed acceleration with no data \n";
		}
	}
	return accel;
}
