#ifndef INCLUDE_GUARD_NEURALCONTROLLER
#define INCLUDE_GUARD_NEURALCONTROLLER

#include <string>
#include <TwoDLib/SimulationParserCPU.h>
#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Simulation/Control/Controller.h>

namespace OpenSim {

	class NeuralController : public Controller {
		OpenSim_DECLARE_CONCRETE_OBJECT(NeuralController, Controller);
	public:

		OpenSim_DECLARE_PROPERTY(InputFunctions, FunctionSet,
			"Functions describing the inputs to the network"
			"specified for this controller.");

		NeuralController(std::string xmlfile);

		NeuralController(std::string xmlfile, const std::string& controlsFileName,
			int interpMethodType);

		virtual ~NeuralController();

		double getCurrentTime() { return sim->getCurrentSimTime(); }

		void computeControls(const SimTK::State& s,
			SimTK::Vector& controls) const OVERRIDE_11;

		void prescribeInputForControl(int input_index, Function* function);

		void prescribeControlForActuator(int index, std::string alpha_node, std::string beta_node, std::string gamma_node);

		void prescribeControlForActuator(const std::string actName, std::string alpha_node, std::string beta_node, std::string gamma_node);

	public:
		/** Model component interface */
		void extendConnectToModel(Model& model) OVERRIDE_11;
	private:
		// construct and initialize properties
		void constructProperties();

		void preamble();

		void setNull();

		SimulationParserCPU<MPILib::CustomConnectionParameters> *sim;

		std::vector<std::string> alpha_mapping;
		std::vector<std::string> beta_mapping;
		std::vector<std::string> gamma_mapping;
	};
}
#endif