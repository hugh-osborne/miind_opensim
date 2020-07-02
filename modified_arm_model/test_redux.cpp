#include <OpenSim/OpenSim.h>

#include "Afferent/Millard12EqMuscleWithAfferents.h"

using namespace SimTK;
using namespace OpenSim;

Millard12EqMuscleWithAfferents* biceps_long;
Constant* biceps_long_activation;

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
            biceps_long_activation->setValue(0.4);

            biceps_long->setStageForOutputs(model, ts.getState());
            //std::cout << biceps_long->getGTO()->getGTOout(ts.getState()) << " activate!\n" << std::flush;
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
    // Register Afferent Muscle Type so it can be read by the XML
    Millard12EqMuscleWithAfferents test_object;
    Object::registerType(test_object);
    

    Model model("MoBL_ARMS_module2_4_allmuscles.osim");
    model.setUseVisualizer(true);

    setElbowExtensionPosture(model);
    
    biceps_long = (Millard12EqMuscleWithAfferents*)&model.updMuscles().get("BIClong");
    biceps_long->setupAfferents();

    ConsoleReporter* reporter = new ConsoleReporter();
    reporter->set_report_time_interval(0.1);
    reporter->addToReport(biceps_long->getSpindle()->getOutput("primary_Ia"));
    reporter->addToReport(biceps_long->getSpindle()->getOutput("secondary_II"));
    reporter->addToReport(biceps_long->getGTO()->getOutput("gto_out"));
    model.addComponent(reporter);
    

    PrescribedController* brain = new PrescribedController();
    brain->addActuator(*biceps_long);


    biceps_long_activation = new Constant(0.0);
    brain->prescribeControlForActuator("BIClong", biceps_long_activation);


    model.addController(brain);

    State& state = model.initSystem();

    // Configure the visualizer.
    //model.updMatterSubsystem().setShowDefaultGeometry(true);
    Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
    viz.setBackgroundType(viz.SolidColor);
    viz.setBackgroundColor(Black);

    model.equilibrateMuscles(state);

    // // Simulate.
    my_simulate(model, state, 10.0);

    return 0;
};
