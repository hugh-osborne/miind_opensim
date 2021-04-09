#include <OpenSim/OpenSim.h>

#include "Afferent/Millard12EqMuscleWithAfferents.h"
#include "NeuralController.h"

using namespace SimTK;
using namespace OpenSim;

Millard12EqMuscleWithAfferents* biceps_long;
Millard12EqMuscleWithAfferents* triceps_long;
Millard12EqMuscleWithAfferents* triceps_med;
Millard12EqMuscleWithAfferents* delt_ant;
Millard12EqMuscleWithAfferents* delt_med;
Millard12EqMuscleWithAfferents* delt_post;
Millard12EqMuscleWithAfferents* fcr;
Millard12EqMuscleWithAfferents* ecr_l;
Millard12EqMuscleWithAfferents* pec_m_1;
Millard12EqMuscleWithAfferents* pec_m_2;
Millard12EqMuscleWithAfferents* pec_m_3;
Millard12EqMuscleWithAfferents* lat_1;
Millard12EqMuscleWithAfferents* lat_2;
Millard12EqMuscleWithAfferents* lat_3;
Constant* biceps_long_activation[3];
Constant* triceps_long_activation[3];
Constant* triceps_med_activation;
Constant* delt_ant_activation;
Constant* delt_med_activation;
Constant* delt_post_activation;
Constant* fcr_activation;
Constant* ecr_l_activation;
Constant* pec_m_1_activation;
Constant* pec_m_2_activation;
Constant* pec_m_3_activation;
Constant* lat_1_activation;
Constant* lat_2_activation;
Constant* lat_3_activation;

StatesTrajectory my_simulate(const Model& model, const SimTK::State& initState, double finalTime) {
    StatesTrajectory states;
    SimTK::RungeKutta3Integrator integrator(model.getSystem());
    integrator.setFixedStepSize(0.001);
    integrator.setAccuracy(1e-006);
    integrator.setAllowInterpolation(true);
    integrator.setProjectInterpolatedStates(true);
    SimTK::TimeStepper ts(model.getSystem(), integrator);
    ts.initialize(initState);
    ts.setReportAllSignificantStates(true);
    integrator.setReturnEveryInternalStep(true);
    integrator.setProjectEveryStep(true);
    std::cout << "Starting Simulation...\n";

    while (ts.getState().getTime() < finalTime) {
        std::cout << "(" << ts.getState().getTime() << ") " << integrator.getSuccessfulStepStatusString(ts.stepTo(finalTime)) << "                  \r" << std::flush;
        states.append(ts.getState());

        std::vector<double> input_rates;

        if (ts.getState().getTime() > 0.1) {
            if (ts.getState().getTime() > 0.8) {
                input_rates.push_back(130000);
                input_rates.push_back(80000);
                input_rates.push_back(80000);

                input_rates.push_back(130000);
                input_rates.push_back(80000);
                input_rates.push_back(80000);
            }
            else {
                input_rates.push_back(130000);
                input_rates.push_back(80000);
                input_rates.push_back(80000);

                input_rates.push_back(80000);
                input_rates.push_back(80000);
                input_rates.push_back(80000);
            }
        }
        else {
            input_rates.push_back(100000);
            input_rates.push_back(80000);
            input_rates.push_back(80000);

            input_rates.push_back(100000);
            input_rates.push_back(80000);
            input_rates.push_back(80000);
        }

        biceps_long_activation[0]->setValue(input_rates[0]);
        biceps_long_activation[1]->setValue(input_rates[1]);
        biceps_long_activation[2]->setValue(input_rates[2]);

        triceps_long_activation[0]->setValue(input_rates[3]);
        triceps_long_activation[1]->setValue(input_rates[4]);
        triceps_long_activation[2]->setValue(input_rates[5]);
            
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
    model.updCoordinateSet().get("elbow_flexion").setDefaultValue(0.298132); // 0.698132 = 40 degrees in radians

    model.updCoordinateSet().get("pro_sup").setDefaultLocked(true);
    model.updCoordinateSet().get("pro_sup").setDefaultValue(0.0);

    model.updCoordinateSet().get("deviation").setDefaultLocked(true);
    model.updCoordinateSet().get("deviation").setDefaultValue(0.0);

    model.updCoordinateSet().get("flexion").setDefaultLocked(true);
    model.updCoordinateSet().get("flexion").setDefaultValue(0.0);
}

int main() {

    // Register Afferent Muscle Type so it can be read by the XML
    Object::registerType(Millard12EqMuscleWithAfferents());
    

    Model model("MoBL_ARMS_module2_4_onemuscle_afferent.osim");
    model.setUseVisualizer(true);

    setElbowExtensionPosture(model);
    
    biceps_long = (Millard12EqMuscleWithAfferents*)&model.updMuscles().get("BIClong");
    biceps_long->setupAfferents();

    triceps_long = (Millard12EqMuscleWithAfferents*)&model.updMuscles().get("TRIlong");
    triceps_long->setupAfferents();

    triceps_med = (Millard12EqMuscleWithAfferents*)&model.updMuscles().get("TRImed");
    triceps_med->setupAfferents();

    delt_ant = (Millard12EqMuscleWithAfferents*)&model.updMuscles().get("DELT1");
    delt_ant->setupAfferents();

    delt_med = (Millard12EqMuscleWithAfferents*)&model.updMuscles().get("DELT2");
    delt_med->setupAfferents();

    delt_post = (Millard12EqMuscleWithAfferents*)&model.updMuscles().get("DELT3");
    delt_post->setupAfferents();

    fcr = (Millard12EqMuscleWithAfferents*)&model.updMuscles().get("FCR");
    fcr->setupAfferents();

    ecr_l = (Millard12EqMuscleWithAfferents*)&model.updMuscles().get("ECRL");
    ecr_l->setupAfferents();

    pec_m_1 = (Millard12EqMuscleWithAfferents*)&model.updMuscles().get("PECM1");
    pec_m_1->setupAfferents();

    pec_m_2 = (Millard12EqMuscleWithAfferents*)&model.updMuscles().get("PECM2");
    pec_m_2->setupAfferents();

    pec_m_3 = (Millard12EqMuscleWithAfferents*)&model.updMuscles().get("PECM3");
    pec_m_3->setupAfferents();

    lat_1 = (Millard12EqMuscleWithAfferents*)&model.updMuscles().get("LAT1");
    lat_1->setupAfferents();

    lat_2 = (Millard12EqMuscleWithAfferents*)&model.updMuscles().get("LAT2");
    lat_2->setupAfferents();

    lat_3 = (Millard12EqMuscleWithAfferents*)&model.updMuscles().get("LAT3");
    lat_3->setupAfferents();

    ConsoleReporter* reporter = new ConsoleReporter();
    reporter->set_report_time_interval(0.001);
    reporter->addToReport(biceps_long->getSpindle()->getOutput("primary_Ia"));
    reporter->addToReport(biceps_long->getSpindle()->getOutput("secondary_II"));
    reporter->addToReport(biceps_long->getGTO()->getOutput("gto_out"));
    reporter->addToReport(biceps_long->getOutput("fiber_force"));
    reporter->addToReport(biceps_long->getOutput("fiber_length"));
    reporter->addToReport(biceps_long->getOutput("fiber_velocity"));
    reporter->addToReport(biceps_long->getOutput("lpf_velocity"));
    reporter->addToReport(biceps_long->getOutput("lpf_acceleration"));
    model.addComponent(reporter);

    TableReporter* table_reporter = new TableReporter();
    table_reporter->set_report_time_interval(0.01);
    table_reporter->addToReport(biceps_long->getSpindle()->getOutput("primary_Ia"));
    table_reporter->addToReport(biceps_long->getSpindle()->getOutput("secondary_II"));
    table_reporter->addToReport(biceps_long->getGTO()->getOutput("gto_out"));
    table_reporter->addToReport(biceps_long->getOutput("fiber_force"));
    table_reporter->addToReport(biceps_long->getOutput("fiber_length"));
    table_reporter->addToReport(biceps_long->getOutput("lpf_velocity"));
    table_reporter->addToReport(biceps_long->getOutput("lpf_acceleration"));
    model.addComponent(table_reporter);
    
    NeuralController* brain = new NeuralController("arm.mmxml");
    //brain->setActuators(model.updActuators());
    brain->addActuator(*biceps_long);
    biceps_long_activation[0] = new Constant(0.0);
    biceps_long_activation[1] = new Constant(0.0);
    biceps_long_activation[2] = new Constant(0.0);
    brain->prescribeInputForControl(0, biceps_long_activation[0]);
    brain->prescribeInputForControl(1, biceps_long_activation[1]);
    brain->prescribeInputForControl(2, biceps_long_activation[2]);
    brain->prescribeControlForActuator("BIClong", "BIClongA_0", "BIClongB_0", "BICmedG_0");

    brain->addActuator(*triceps_long);
    triceps_long_activation[0] = new Constant(0.0);
    triceps_long_activation[1] = new Constant(0.0);
    triceps_long_activation[2] = new Constant(0.0);
    brain->prescribeInputForControl(3, triceps_long_activation[0]);
    brain->prescribeInputForControl(4, triceps_long_activation[1]);
    brain->prescribeInputForControl(5, triceps_long_activation[2]);
    brain->prescribeControlForActuator("TRIlong", "TRIlongA_0", "TRIlongB_0", "TRImedG_0");

    /*triceps_long_activation = new Constant(0.0);
    brain->prescribeControlForActuator("TRIlong", triceps_long_activation);

    triceps_med_activation = new Constant(0.0);
    brain->prescribeControlForActuator("TRImed", triceps_med_activation);

    delt_ant_activation = new Constant(0.0);
    brain->prescribeControlForActuator("DELT1", delt_ant_activation);

    delt_med_activation = new Constant(0.0);
    brain->prescribeControlForActuator("DELT2", delt_med_activation);

    delt_post_activation = new Constant(0.0);
    brain->prescribeControlForActuator("DELT3", delt_post_activation);

    fcr_activation = new Constant(0.0);
    brain->prescribeControlForActuator("FCR", fcr_activation);

    ecr_l_activation = new Constant(0.0);
    brain->prescribeControlForActuator("ECRL", ecr_l_activation);

    pec_m_1_activation = new Constant(0.0);
    brain->prescribeControlForActuator("PECM1", pec_m_1_activation);

    pec_m_2_activation = new Constant(0.0);
    brain->prescribeControlForActuator("PECM2", pec_m_2_activation);

    pec_m_3_activation = new Constant(0.0);
    brain->prescribeControlForActuator("PECM3", pec_m_3_activation);

    lat_1_activation = new Constant(0.0);
    brain->prescribeControlForActuator("LAT1", lat_1_activation);

    lat_2_activation = new Constant(0.0);
    brain->prescribeControlForActuator("LAT2", lat_2_activation);

    lat_3_activation = new Constant(0.0);
    brain->prescribeControlForActuator("LAT3", lat_3_activation);*/

    model.addController(brain);

    SimTK::State& state = model.initSystem();

    // Configure the visualizer.
    //model.updMatterSubsystem().setShowDefaultGeometry(true);
    Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
    viz.setBackgroundType(viz.SolidColor);
    viz.setBackgroundColor(Black);

    model.equilibrateMuscles(state);

    // // Simulate.
    my_simulate(model, state, 5);
    //simulate(model, state, 10.0);

    auto table = table_reporter->getTable();

    ofstream dataFile;
    dataFile.open("output.txt");
    for (int i = 0; i < table.getNumRows(); i++) {
        for (int j = 0; j < 7; j++) { // Currently recording 7 metrics...
            dataFile << table.getRowAtIndex(i).getAsVector()[j] << "," << std::flush;
        }
        dataFile << "\n" << std::flush;
    }
    dataFile.close();

    return 0;
};