
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  AfferentAnalysis.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Sergio Verduzco from MyAnalysis.h                               *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//=============================================================================
// INCLUDES
//=============================================================================
// Headers define the various property types that OpenSim objects can read 
#include <OpenSim/Simulation/Model/Analysis.h>
//#include "osimPluginDLL.h"      // Header to define plugin (DLL) interface

// includes used in MuscleAnalysis
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Common/Storage.h>


//=============================================================================
//=============================================================================
/*
 * A class template for writing a custom Analysis 
 * Currently reports various values from Millard12EqMuscleWithAfferents objects.
 */
namespace OpenSim { 

//class OSIMPLUGIN_API AfferentAnalysis : public Analysis
class AfferentAnalysis : public Analysis 
{
OpenSim_DECLARE_CONCRETE_OBJECT(AfferentAnalysis, Analysis);

	
public:
//=======================================================================
// PROPERTIES
//=======================================================================
/** @name Property declarations
	These are the serializable properties associated with this class. **/
/**@{**/

    /** String list property containing the name of the muscles to analyze.
	 * This is the main reference as to which muscles should be analyzed. 
	 * _muscleList and _muscleArray are updated based on muscleListProp */
    OpenSim_DECLARE_LIST_PROPERTY(muscleListProp, std::string, 
    "Names of the muscles on which to perform the analysis."
    "The key word 'All' indicates that the analysis should be performed for all muscles.");

    // Here are some examples of other scalar property types.
    // Uncomment them as you need them.
    // ------------------------------------------------------
    //// My string property
    //OpenSim_DECLARE_PROPERTY(string_property, std::string, 
    //"My string property."); 

    //// My int property
    //OpenSim_DECLARE_PROPERTY(int_property, int, 
    //"My int property."); 

    //// My bool property
    //OpenSim_DECLARE_PROPERTY(bool_property, bool, 
    //"My bool property."); 

    //// My double property
    //OpenSim_DECLARE_PROPERTY(double_property, double, 
    //"My double property."); 

/**@}**/


//=======================================================================
// INTERNAL MEMBER VARIABLES
//=======================================================================

	// In addition to properties, add any additional member variables
	// you need for your analysis.  These variables are not read from
	// or written to file.  They are just variables you use to execute
	// your analysis.  For example, you will almost certainly need a
	// storage object for storing the results of your analysis.

	// Storage object for storing and writing out results.  In general,
	// each storage file that you create will contain only one kind of data.
	// Create a different storage object for each kind of data.  For example,
	// create a _storePos for positions, _storeVel for velocities,
	// _storeAcc for accelerations, etc. */

	/** Array of active muscles. */
	ArrayPtrs<Muscle> _muscleArray;
	
	/** storage for recording muscle inputs */
	Storage *_muscleInputs;
	
	/** storage for recording state variables of the spindle object */
	Storage *_spindleVariables;
	
	/** storage for recording variables of the GTO object */
	Storage *_gtoVariables;
	
	/** Work array for holding the names of recorded muscles. */
	Array<std::string> _muscleList;
	
	/** Container for muscle controls, used as an auxiliary variable
	 *  in the record method. Initialized in setModel. */
	Array<double> ctrlInputs; 
	
	/** Containers for spindle and gto variables, used as auxiliary 
	 *  variableS in the record method. Initialized in setModel. */
	Array<double> spinVars, gtoVars;

//=============================================================================
// METHODS
//=============================================================================
private:
    /** Construct default values for internal member variables, */
	/** i.e., zero data and set pointers to Null */
	void setNull();

	/** Construct default values for properties */
	void constructProperties();

public:
    /** Default constructor */
    AfferentAnalysis();
	
	/** Constructor with model. Basically the default constructor with
	 * a call to setModel at the end */
	AfferentAnalysis(Model *aModel);

    /** setModel */
	virtual void setModel(Model& aModel);
	
	/**
	* Specify a single muscle for which the analysis will be 
	* performed. Currently it's one muscle or all muscles.
	* Make sure the muscle is of the Millard12EqMuscleWithAfferents class.
	*/
	void specifyMuscle(const std::string &muscName);

	//-------------------------------------------------------------------------
	// METHODS THAT MUST BE OVERRIDDEN
	//-------------------------------------------------------------------------
	virtual int
		begin(SimTK::State& s);
	virtual int
		step(const SimTK::State& s, int stepNumber);
	virtual int
		end(SimTK::State& s);

	//-------------------------------------------------------------------------
	// IO
	//-------------------------------------------------------------------------
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");


protected:
	//========================== Internal Methods =============================
	int record(const SimTK::State& s);
	void constructDescription();
	void constructColumnLabels();
	Array<std::string> columnLabels(int stor); 
	void setupStorage();

//=============================================================================
}; // END of class AfferentAnalysis

}; //namespace



