#include <OpenSim/OpenSim.h>

#include "Afferent/Millard12EqMuscleWithAfferents.h"

using namespace SimTK;
using namespace OpenSim;

Millard2012EquilibriumMuscle* biceps_long;
Constant* biceps_long_activation;

Millard2012EquilibriumMuscle* triceps_med;
Constant* triceps_med_activation;

Millard2012EquilibriumMuscle* triceps_long;
Constant* triceps_long_activation;

Millard2012EquilibriumMuscle* ecr;
Constant* ecr_activation;

Millard2012EquilibriumMuscle* fcr;
Constant* fcr_activation;

StatesTrajectory my_simulate(const Model& model, const State& initState, double finalTime) {
    StatesTrajectory states;
    SimTK::RungeKutta2Integrator integrator(model.getSystem());
    integrator.setFixedStepSize(0.001);
    integrator.setAccuracy(1e-005);
    integrator.setAllowInterpolation(true);
    integrator.setProjectInterpolatedStates(true);
    SimTK::TimeStepper ts(model.getSystem(), integrator);
    ts.initialize(initState);
    ts.setReportAllSignificantStates(true);
    integrator.setReturnEveryInternalStep(true);
    std::cout << "Starting Simulation...\n";

    while (ts.getState().getTime() < finalTime) {
        std::cout << "(" << ts.getState().getTime() << ") " << integrator.getSuccessfulStepStatusString(ts.stepTo(finalTime)) << "                  \r" << std::flush;
        states.append(ts.getState());

        if(ts.getState().getTime() > 1.0){
            biceps_long_activation->setValue(0.1);
            triceps_med_activation->setValue(0.1);
            triceps_long_activation->setValue(0.1);
        }
            
    }
    return states;
}

void setElbowExtensionPosture(Model& model){
    model.updCoordinateSet().get("elv_angle").setDefaultLocked(true);
    model.updCoordinateSet().get("elv_angle").setDefaultValue(0.0);

    model.updCoordinateSet().get("shoulder_elv").setDefaultLocked(true);
    model.updCoordinateSet().get("shoulder_elv").setDefaultValue(0.0);

    model.updCoordinateSet().get("shoulder_rot").setDefaultLocked(true);
    model.updCoordinateSet().get("shoulder_rot").setDefaultValue(0.0);

    model.updCoordinateSet().get("elbow_flexion").setDefaultLocked(false);
    model.updCoordinateSet().get("elbow_flexion").setDefaultValue(0.698132); // 0.698132 = 40 degrees in radians

    model.updCoordinateSet().get("pro_sup").setDefaultLocked(true);
    model.updCoordinateSet().get("pro_sup").setDefaultValue(0.0);

    model.updCoordinateSet().get("deviation").setDefaultLocked(true);
    model.updCoordinateSet().get("deviation").setDefaultValue(0.0);

    model.updCoordinateSet().get("flexion").setDefaultLocked(true);
    model.updCoordinateSet().get("flexion").setDefaultValue(0.0);
}

int main() {
    Model model("MoBL_ARMS_module2_4_allmuscles.osim");
    model.setUseVisualizer(true);

    setElbowExtensionPosture(model);
    
    biceps_long = (Millard2012EquilibriumMuscle*)&model.updMuscles().get("BIClong");
    triceps_med = (Millard2012EquilibriumMuscle*)&model.updMuscles().get("TRImed");
    triceps_long = (Millard2012EquilibriumMuscle*)&model.updMuscles().get("TRIlong");
    ecr = (Millard2012EquilibriumMuscle*)&model.updMuscles().get("ECRL");
    fcr = (Millard2012EquilibriumMuscle*)&model.updMuscles().get("FCR");

    Millard2012EquilibriumMuscle test = Millard12EqMuscleWithAfferents(*biceps_long);


    PrescribedController* brain = new PrescribedController();
    brain->addActuator(*biceps_long);
    brain->addActuator(*triceps_med);
    brain->addActuator(*triceps_long);
    brain->addActuator(*ecr);
    brain->addActuator(*fcr);





    biceps_long_activation = new Constant(0.0);
    brain->prescribeControlForActuator("BIClong", biceps_long_activation);

    triceps_med_activation = new Constant(0.0);
    brain->prescribeControlForActuator("TRImed", triceps_med_activation);

    triceps_long_activation = new Constant(0.0);
    brain->prescribeControlForActuator("TRIlong", triceps_long_activation);

    ecr_activation = new Constant(0.0);
    brain->prescribeControlForActuator("ECRL", ecr_activation);

    fcr_activation = new Constant(0.0);
    brain->prescribeControlForActuator("FCR", fcr_activation);

    model.addController(brain);

    State& state = model.initSystem();

    // Configure the visualizer.
    //model.updMatterSubsystem().setShowDefaultGeometry(true);
    Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
    viz.setBackgroundType(viz.SolidColor);
    viz.setBackgroundColor(Black);

    model.equilibrateMuscles(state);

    // Simulate.
    my_simulate(model, state, 10.0);

    return 0;
};
