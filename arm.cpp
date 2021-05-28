#include <OpenSim/OpenSim.h>

#include "Afferent/Millard12EqMuscleWithAfferents.h"
#include "NeuralController.h"

using namespace SimTK;
using namespace OpenSim;

bool skeletal_model_viz = true;
double sim_length = 0.5;
double timestep = 0.0002;

Arrow *forceArrow;
JointReaction* jr;
ForceReporter* force_reporter;

CustomJoint* anchor_joint;
OpenSim::Body* radius;

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
Constant* biceps_afferents[3]; // Ia, II, Ib
Constant* biceps_long_activation[3];
Constant* triceps_long_afferents[3];
Constant* triceps_long_activation[3];
Constant* triceps_med_activation;
Constant* delt_ant_afferents[3];
Constant* delt_ant_activation[3];
Constant* delt_med_afferents[3];
Constant* delt_med_activation[3];
Constant* delt_post_afferents[3];
Constant* delt_post_activation[3];
Constant* fcr_afferents[3];
Constant* fcr_activation[3];
Constant* ecr_l_afferents[3];
Constant* ecr_l_activation[3];
Constant* pec_m_1_activation;
Constant* pec_m_2_afferents[3];
Constant* pec_m_2_activation[3];
Constant* pec_m_3_activation;
Constant* lat_1_activation;
Constant* lat_2_afferents[3];
Constant* lat_2_activation[3];
Constant* lat_3_activation;

bool biceps_active;
bool triceps_active;
bool delt_ant_active;
bool delt_med_active;
bool delt_post_active;
bool pec_m_2_active;
bool lat_2_active;
bool fcr_active;
bool ecr_l_active;

StatesTrajectory my_simulate(const Model& model, const SimTK::State& initState, double finalTime) {
    StatesTrajectory states;
    SimTK::RungeKutta3Integrator integrator(model.getSystem());
    integrator.setFixedStepSize(timestep);
    integrator.setAccuracy(1e-003);
    integrator.setAllowInterpolation(true);
    integrator.setProjectInterpolatedStates(true);
    SimTK::TimeStepper ts(model.getSystem(), integrator);
    ts.initialize(initState);
    ts.setReportAllSignificantStates(false);
    integrator.setReturnEveryInternalStep(true);
    integrator.setProjectEveryStep(false);
    std::cout << "Starting Simulation...\n";

    unsigned int update_vis_count = 1;
    unsigned int update_vis_counter = 0;

    double time = 0;

    jr->begin(ts.getState());
    force_reporter->begin(ts.getState());

    while (time < finalTime) {
        time += timestep;
        update_vis_counter++;
        std::cout << "(" << ts.getState().getTime() << ") " << integrator.getSuccessfulStepStatusString(ts.stepTo(time)) << "                  \r" << std::flush;
        states.append(ts.getState());

        model.realizeReport(ts.getState());

        int input_rate_index = 0;

        std::vector<double> input_rates;

        if (biceps_active) {
            input_rates.push_back(130000 + (biceps_long->getSpindle()->getIaOutput(ts.getState()) * 300));
            input_rates.push_back(0);
            input_rates.push_back(230000);

            biceps_afferents[0]->setValue(biceps_long->getSpindle()->getIaOutput(ts.getState()) * 0);
            biceps_afferents[1]->setValue(biceps_long->getGTO()->getGTOout(ts.getState()) * 0);
            biceps_afferents[2]->setValue(biceps_long->getSpindle()->getIIOutput(ts.getState()) * 0);

            biceps_long_activation[0]->setValue(input_rates[input_rate_index]);
            biceps_long_activation[1]->setValue(input_rates[input_rate_index+1]);
            biceps_long_activation[2]->setValue(input_rates[input_rate_index+2]);
            input_rate_index += 3;
        }
        
        if (triceps_active) {
            input_rates.push_back(130000 + (triceps_long->getSpindle()->getIaOutput(ts.getState()) * 300));
            input_rates.push_back(0);
            input_rates.push_back(230000);

            triceps_long_afferents[0]->setValue(triceps_long->getSpindle()->getIaOutput(ts.getState()) * 0);
            triceps_long_afferents[1]->setValue(triceps_long->getGTO()->getGTOout(ts.getState()) * 0);
            triceps_long_afferents[2]->setValue(triceps_long->getSpindle()->getIIOutput(ts.getState()) * 0);

            triceps_long_activation[0]->setValue(input_rates[input_rate_index]);
            triceps_long_activation[1]->setValue(input_rates[input_rate_index+1]);
            triceps_long_activation[2]->setValue(input_rates[input_rate_index+2]);

            input_rate_index += 3;
        }

        if (delt_ant_active) {
            input_rates.push_back(130000 + (delt_ant->getSpindle()->getIaOutput(ts.getState()) * 300));
            input_rates.push_back(0);
            input_rates.push_back(230000);

            delt_ant_afferents[0]->setValue(delt_ant->getSpindle()->getIaOutput(ts.getState()) * 0);
            delt_ant_afferents[1]->setValue(delt_ant->getGTO()->getGTOout(ts.getState()) * 0);
            delt_ant_afferents[2]->setValue(delt_ant->getSpindle()->getIIOutput(ts.getState()) * 0);

            delt_ant_activation[0]->setValue(input_rates[input_rate_index]);
            delt_ant_activation[1]->setValue(input_rates[input_rate_index+1]);
            delt_ant_activation[2]->setValue(input_rates[input_rate_index+2]);

            input_rate_index += 3;
        }
        
        if (delt_med_active) {
            input_rates.push_back(130000 + (delt_med->getSpindle()->getIaOutput(ts.getState()) * 300));
            input_rates.push_back(0);
            input_rates.push_back(230000);

            delt_med_afferents[0]->setValue(delt_med->getSpindle()->getIaOutput(ts.getState()) * 0);
            delt_med_afferents[1]->setValue(delt_med->getGTO()->getGTOout(ts.getState()) * 0);
            delt_med_afferents[2]->setValue(delt_med->getSpindle()->getIIOutput(ts.getState()) * 0);

            delt_med_activation[0]->setValue(input_rates[input_rate_index]);
            delt_med_activation[1]->setValue(input_rates[input_rate_index+1]);
            delt_med_activation[2]->setValue(input_rates[input_rate_index+2]);

            input_rate_index += 3;
        }
        
        if (delt_post_active) {
            input_rates.push_back(130000 + (delt_post->getSpindle()->getIaOutput(ts.getState()) * 300));
            input_rates.push_back(0);
            input_rates.push_back(230000);

            delt_post_afferents[0]->setValue(delt_post->getSpindle()->getIaOutput(ts.getState()) * 0);
            delt_post_afferents[1]->setValue(delt_post->getGTO()->getGTOout(ts.getState()) * 0);
            delt_post_afferents[2]->setValue(delt_post->getSpindle()->getIIOutput(ts.getState()) * 0);

            delt_post_activation[0]->setValue(input_rates[input_rate_index]);
            delt_post_activation[1]->setValue(input_rates[input_rate_index+1]);
            delt_post_activation[2]->setValue(input_rates[input_rate_index+2]);

            input_rate_index += 3;
        }
        
        if (pec_m_2_active) {
            input_rates.push_back(130000 + (pec_m_2->getSpindle()->getIaOutput(ts.getState()) * 300));
            input_rates.push_back(0);
            input_rates.push_back(230000);

            pec_m_2_afferents[0]->setValue(pec_m_2->getSpindle()->getIaOutput(ts.getState()) * 0);
            pec_m_2_afferents[1]->setValue(pec_m_2->getGTO()->getGTOout(ts.getState()) * 0);
            pec_m_2_afferents[2]->setValue(pec_m_2->getSpindle()->getIIOutput(ts.getState()) * 0);

            pec_m_2_activation[0]->setValue(input_rates[input_rate_index]);
            pec_m_2_activation[1]->setValue(input_rates[input_rate_index+1]);
            pec_m_2_activation[2]->setValue(input_rates[input_rate_index+2]);

            input_rate_index += 3;
        }
        
        if (lat_2_active) {
            input_rates.push_back(130000 + (lat_2->getSpindle()->getIaOutput(ts.getState()) * 300));
            input_rates.push_back(0);
            input_rates.push_back(230000);

            lat_2_afferents[0]->setValue(lat_2->getSpindle()->getIaOutput(ts.getState()) * 0);
            lat_2_afferents[1]->setValue(lat_2->getGTO()->getGTOout(ts.getState()) * 0);
            lat_2_afferents[2]->setValue(lat_2->getSpindle()->getIIOutput(ts.getState()) * 0);

            lat_2_activation[0]->setValue(input_rates[input_rate_index]);
            lat_2_activation[1]->setValue(input_rates[input_rate_index+1]);
            lat_2_activation[2]->setValue(input_rates[input_rate_index+2]);

            input_rate_index += 3;
        }

        if (fcr_active) {
            input_rates.push_back(130000 + (fcr->getSpindle()->getIaOutput(ts.getState()) * 300));
            input_rates.push_back(0);
            input_rates.push_back(230000);

            fcr_afferents[0]->setValue(fcr->getSpindle()->getIaOutput(ts.getState()) * 0);
            fcr_afferents[1]->setValue(fcr->getGTO()->getGTOout(ts.getState()) * 0);
            fcr_afferents[2]->setValue(fcr->getSpindle()->getIIOutput(ts.getState()) * 0);

            fcr_activation[0]->setValue(input_rates[input_rate_index]);
            fcr_activation[1]->setValue(input_rates[input_rate_index + 1]);
            fcr_activation[2]->setValue(input_rates[input_rate_index + 2]);

            input_rate_index += 3;
        }

        if (ecr_l_active) {
            input_rates.push_back(130000 + (ecr_l->getSpindle()->getIaOutput(ts.getState()) * 300));
            input_rates.push_back(0);
            input_rates.push_back(230000);

            ecr_l_afferents[0]->setValue(ecr_l->getSpindle()->getIaOutput(ts.getState()) * 0);
            ecr_l_afferents[1]->setValue(ecr_l->getGTO()->getGTOout(ts.getState()) * 0);
            ecr_l_afferents[2]->setValue(ecr_l->getSpindle()->getIIOutput(ts.getState()) * 0);

            ecr_l_activation[0]->setValue(input_rates[input_rate_index]);
            ecr_l_activation[1]->setValue(input_rates[input_rate_index + 1]);
            ecr_l_activation[2]->setValue(input_rates[input_rate_index + 2]);

            input_rate_index += 3;
        }


        if (update_vis_counter > update_vis_count && skeletal_model_viz) {
            update_vis_counter = 0;
            model.getVisualizer().show(ts.getState());
        }   

        jr->step(ts.getState(), 1);
        force_reporter->step(ts.getState(), 1);

        //auto vec = anchor_joint->calcReactionOnChildExpressedInGround(ts.getState());
        //std::cout << vec[0][0] << " " << vec[0][1] << " " << vec[0][2] << " " << vec[1][0] << " " << vec[1][1] << " " << vec[1][2] << " " << "\n";

    }

    jr->end(ts.getState());
    force_reporter->end(ts.getState());
    return states;
}

void setElbowExtensionPosture(Model& model){
    
    model.updCoordinateSet().get("sternoclavicular_r2").setDefaultLocked(false);
    model.updCoordinateSet().get("sternoclavicular_r2").setDefaultValue(0.0);

    model.updCoordinateSet().get("sternoclavicular_r3").setDefaultLocked(false);
    model.updCoordinateSet().get("sternoclavicular_r3").setDefaultValue(0.0);
    
    model.updCoordinateSet().get("unrotscap_r3").setDefaultLocked(false);
    model.updCoordinateSet().get("unrotscap_r3").setDefaultValue(0.0);
    
    model.updCoordinateSet().get("unrotscap_r2").setDefaultLocked(false);
    model.updCoordinateSet().get("unrotscap_r2").setDefaultValue(0.0);

    model.updCoordinateSet().get("acromioclavicular_r2").setDefaultLocked(false);
    model.updCoordinateSet().get("acromioclavicular_r2").setDefaultValue(0.0);

    model.updCoordinateSet().get("acromioclavicular_r3").setDefaultLocked(false);
    model.updCoordinateSet().get("acromioclavicular_r3").setDefaultValue(0.0);

    model.updCoordinateSet().get("acromioclavicular_r1").setDefaultLocked(false);
    model.updCoordinateSet().get("acromioclavicular_r1").setDefaultValue(0.0);

    model.updCoordinateSet().get("unrothum_r1").setDefaultLocked(false);
    model.updCoordinateSet().get("unrothum_r1").setDefaultValue(0.0);

    model.updCoordinateSet().get("unrothum_r3").setDefaultLocked(false);
    model.updCoordinateSet().get("unrothum_r3").setDefaultValue(0.0);

    model.updCoordinateSet().get("unrothum_r2").setDefaultLocked(false);
    model.updCoordinateSet().get("unrothum_r2").setDefaultValue(0.0);

    model.updCoordinateSet().get("elv_angle").setDefaultLocked(false);
    model.updCoordinateSet().get("elv_angle").setDefaultValue(0.0);

    model.updCoordinateSet().get("shoulder_elv").setDefaultLocked(false);
    model.updCoordinateSet().get("shoulder_elv").setDefaultValue(0.0);

    model.updCoordinateSet().get("shoulder_rot").setDefaultLocked(false);
    model.updCoordinateSet().get("shoulder_rot").setDefaultValue(0.0);

    model.updCoordinateSet().get("elbow_flexion").setDefaultLocked(false);
    model.updCoordinateSet().get("elbow_flexion").setDefaultValue(1.698132); // 0.698132 = 40 degrees in radians

    model.updCoordinateSet().get("pro_sup").setDefaultLocked(false);
    model.updCoordinateSet().get("pro_sup").setDefaultValue(0.0);

    model.updCoordinateSet().get("deviation").setDefaultLocked(true);
    model.updCoordinateSet().get("deviation").setDefaultValue(0.0);

    model.updCoordinateSet().get("flexion").setDefaultLocked(true);
    model.updCoordinateSet().get("flexion").setDefaultValue(0.0);
}

int main() {

    biceps_active = true;
    triceps_active = true;
    delt_ant_active = true;
    delt_med_active = true;
    delt_post_active = true;
    pec_m_2_active = true;
    lat_2_active = true;
    fcr_active = true;
    ecr_l_active = true;

    // Register Afferent Muscle Type so it can be read by the XML
    Object::registerType(Millard12EqMuscleWithAfferents());
    

    Model model("MoBL_ARMS_module2_4_onemuscle_afferent.osim");
    //Model model("MoBL_ARMS_module2_4_bictri_afferent.osim");
    model.setUseVisualizer(skeletal_model_viz);

    setElbowExtensionPosture(model);

    anchor_joint = (CustomJoint*)&model.updJointSet().get("anchor_joint");

    radius = (OpenSim::Body*)&model.updBodySet().get("radius");
    
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
    reporter->set_report_time_interval(0.01);
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
    table_reporter->set_report_time_interval(0.001);
    table_reporter->addToReport(biceps_long->getSpindle()->getOutput("primary_Ia"));
    table_reporter->addToReport(biceps_long->getSpindle()->getOutput("secondary_II"));
    table_reporter->addToReport(biceps_long->getGTO()->getOutput("gto_out"));
    table_reporter->addToReport(biceps_long->getOutput("fiber_force"));
    table_reporter->addToReport(biceps_long->getOutput("fiber_length"));
    table_reporter->addToReport(biceps_long->getOutput("lpf_velocity"));
    table_reporter->addToReport(biceps_long->getOutput("lpf_acceleration"));
    model.addComponent(table_reporter);

    force_reporter = new ForceReporter();
    force_reporter->setModel(model);
    model.addAnalysis(force_reporter);

    NeuralController* brain = new NeuralController("arm.mmxml");

    int input_index = 0;

    //brain->setActuators(model.updActuators());
    if (biceps_active) {
        brain->addActuator(*biceps_long);
        biceps_afferents[0] = new Constant(0.0); // Biceps Ia
        biceps_afferents[1] = new Constant(0.0); // Biceps Ib
        biceps_afferents[2] = new Constant(0.0); // Biceps II
        biceps_long_activation[0] = new Constant(0.0);
        biceps_long_activation[1] = new Constant(0.0);
        biceps_long_activation[2] = new Constant(0.0);
        brain->prescribeInputForControl(input_index, biceps_afferents[0]);
        brain->prescribeInputForControl(input_index + 1, biceps_afferents[1]);
        brain->prescribeInputForControl(input_index + 2, biceps_afferents[2]);
        brain->prescribeInputForControl(input_index + 3, biceps_long_activation[0]);
        brain->prescribeInputForControl(input_index + 4, biceps_long_activation[1]);
        brain->prescribeInputForControl(input_index + 5, biceps_long_activation[2]);
        brain->prescribeControlForActuator("BIClong", "BIClongA_0", "BIClongB_0", "BIClongG_0");

        input_index += 6;
    }
    
    if (triceps_active) {
        brain->addActuator(*triceps_long);
        triceps_long_afferents[0] = new Constant(0.0); // Triceps Ia
        triceps_long_afferents[1] = new Constant(0.0); // Triceps Ib
        triceps_long_afferents[2] = new Constant(0.0); // Triceps II
        triceps_long_activation[0] = new Constant(0.0);
        triceps_long_activation[1] = new Constant(0.0);
        triceps_long_activation[2] = new Constant(0.0);
        brain->prescribeInputForControl(input_index, triceps_long_afferents[0]);
        brain->prescribeInputForControl(input_index + 1, triceps_long_afferents[1]);
        brain->prescribeInputForControl(input_index + 2, triceps_long_afferents[2]);
        brain->prescribeInputForControl(input_index + 3, triceps_long_activation[0]);
        brain->prescribeInputForControl(input_index + 4, triceps_long_activation[1]);
        brain->prescribeInputForControl(input_index + 5, triceps_long_activation[2]);
        brain->prescribeControlForActuator("TRIlong", "TRIlongA_0", "TRIlongB_0", "TRIlongG_0");

        input_index += 6;
    }
    
    if (delt_ant_active) {
        brain->addActuator(*delt_ant);
        delt_ant_afferents[0] = new Constant(0.0); // delt_ant Ia
        delt_ant_afferents[1] = new Constant(0.0); // delt_ant Ib
        delt_ant_afferents[2] = new Constant(0.0); // delt_ant II
        delt_ant_activation[0] = new Constant(0.0);
        delt_ant_activation[1] = new Constant(0.0);
        delt_ant_activation[2] = new Constant(0.0);
        brain->prescribeInputForControl(input_index, delt_ant_afferents[0]);
        brain->prescribeInputForControl(input_index + 1, delt_ant_afferents[1]);
        brain->prescribeInputForControl(input_index + 2, delt_ant_afferents[2]);
        brain->prescribeInputForControl(input_index + 3, delt_ant_activation[0]);
        brain->prescribeInputForControl(input_index + 4, delt_ant_activation[1]);
        brain->prescribeInputForControl(input_index + 5, delt_ant_activation[2]);
        brain->prescribeControlForActuator("DELT1", "DELTantA_0", "DELTantB_0", "DELTantG_0");

        input_index += 6;
    }
    
    if (delt_med_active) {
        brain->addActuator(*delt_med);
        delt_med_afferents[0] = new Constant(0.0); // delt_med Ia
        delt_med_afferents[1] = new Constant(0.0); // delt_med Ib
        delt_med_afferents[2] = new Constant(0.0); // delt_med II
        delt_med_activation[0] = new Constant(0.0);
        delt_med_activation[1] = new Constant(0.0);
        delt_med_activation[2] = new Constant(0.0);
        brain->prescribeInputForControl(input_index, delt_med_afferents[0]);
        brain->prescribeInputForControl(input_index + 1, delt_med_afferents[1]);
        brain->prescribeInputForControl(input_index + 2, delt_med_afferents[2]);
        brain->prescribeInputForControl(input_index + 3, delt_med_activation[0]);
        brain->prescribeInputForControl(input_index + 4, delt_med_activation[1]);
        brain->prescribeInputForControl(input_index + 5, delt_med_activation[2]);
        brain->prescribeControlForActuator("DELT2", "DELTmedA_0", "DELTmedB_0", "DELTmedG_0");

        input_index += 6;
    }

    if (delt_post_active) {
        brain->addActuator(*delt_post);
        delt_post_afferents[0] = new Constant(0.0); // delt_post Ia
        delt_post_afferents[1] = new Constant(0.0); // delt_post Ib
        delt_post_afferents[2] = new Constant(0.0); // delt_post II
        delt_post_activation[0] = new Constant(0.0);
        delt_post_activation[1] = new Constant(0.0);
        delt_post_activation[2] = new Constant(0.0);
        brain->prescribeInputForControl(input_index, delt_post_afferents[0]);
        brain->prescribeInputForControl(input_index + 1, delt_post_afferents[1]);
        brain->prescribeInputForControl(input_index + 2, delt_post_afferents[2]);
        brain->prescribeInputForControl(input_index + 3, delt_post_activation[0]);
        brain->prescribeInputForControl(input_index + 4, delt_post_activation[1]);
        brain->prescribeInputForControl(input_index + 5, delt_post_activation[2]);
        brain->prescribeControlForActuator("DELT3", "DELTpostA_0", "DELTpostB_0", "DELTpostG_0");

        input_index += 6;
    }
    
    if (pec_m_2_active) {
        brain->addActuator(*pec_m_2);
        pec_m_2_afferents[0] = new Constant(0.0); // pec_m_2 Ia
        pec_m_2_afferents[1] = new Constant(0.0); // pec_m_2 Ib
        pec_m_2_afferents[2] = new Constant(0.0); // pec_m_2 II
        pec_m_2_activation[0] = new Constant(0.0);
        pec_m_2_activation[1] = new Constant(0.0);
        pec_m_2_activation[2] = new Constant(0.0);
        brain->prescribeInputForControl(input_index, pec_m_2_afferents[0]);
        brain->prescribeInputForControl(input_index + 1, pec_m_2_afferents[1]);
        brain->prescribeInputForControl(input_index + 2, pec_m_2_afferents[2]);
        brain->prescribeInputForControl(input_index + 3, pec_m_2_activation[0]);
        brain->prescribeInputForControl(input_index + 4, pec_m_2_activation[1]);
        brain->prescribeInputForControl(input_index + 5, pec_m_2_activation[2]);
        brain->prescribeControlForActuator("PECM2", "PECm2A_0", "PECm2B_0", "PECm2G_0");

        input_index += 6;
    }
    
    if (lat_2_active) {
        brain->addActuator(*lat_2);
        lat_2_afferents[0] = new Constant(0.0); // lat_2 Ia
        lat_2_afferents[1] = new Constant(0.0); // lat_2 Ib
        lat_2_afferents[2] = new Constant(0.0); // lat_2 II
        lat_2_activation[0] = new Constant(0.0);
        lat_2_activation[1] = new Constant(0.0);
        lat_2_activation[2] = new Constant(0.0);
        brain->prescribeInputForControl(input_index, lat_2_afferents[0]);
        brain->prescribeInputForControl(input_index + 1, lat_2_afferents[1]);
        brain->prescribeInputForControl(input_index + 2, lat_2_afferents[2]);
        brain->prescribeInputForControl(input_index + 3, lat_2_activation[0]);
        brain->prescribeInputForControl(input_index + 4, lat_2_activation[1]);
        brain->prescribeInputForControl(input_index + 5, lat_2_activation[2]);
        brain->prescribeControlForActuator("LAT2", "LATm2A_0", "LATm2B_0", "LATm2G_0");

        input_index += 6;
    }

    if (fcr_active) {
        brain->addActuator(*fcr);
        fcr_afferents[0] = new Constant(0.0); // fcr Ia
        fcr_afferents[1] = new Constant(0.0); // fcr Ib
        fcr_afferents[2] = new Constant(0.0); // fcr II
        fcr_activation[0] = new Constant(0.0);
        fcr_activation[1] = new Constant(0.0);
        fcr_activation[2] = new Constant(0.0);
        brain->prescribeInputForControl(input_index, fcr_afferents[0]);
        brain->prescribeInputForControl(input_index + 1, fcr_afferents[1]);
        brain->prescribeInputForControl(input_index + 2, fcr_afferents[2]);
        brain->prescribeInputForControl(input_index + 3, fcr_activation[0]);
        brain->prescribeInputForControl(input_index + 4, fcr_activation[1]);
        brain->prescribeInputForControl(input_index + 5, fcr_activation[2]);
        brain->prescribeControlForActuator("FCR", "FCRA_0", "FCRB_0", "FCRG_0");

        input_index += 6;
    }

    if (ecr_l_active) {
        brain->addActuator(*ecr_l);
        ecr_l_afferents[0] = new Constant(0.0); // ecr_l Ia
        ecr_l_afferents[1] = new Constant(0.0); // ecr_l Ib
        ecr_l_afferents[2] = new Constant(0.0); // ecr_l II
        ecr_l_activation[0] = new Constant(0.0);
        ecr_l_activation[1] = new Constant(0.0);
        ecr_l_activation[2] = new Constant(0.0);
        brain->prescribeInputForControl(input_index, ecr_l_afferents[0]);
        brain->prescribeInputForControl(input_index + 1, ecr_l_afferents[1]);
        brain->prescribeInputForControl(input_index + 2, ecr_l_afferents[2]);
        brain->prescribeInputForControl(input_index + 3, ecr_l_activation[0]);
        brain->prescribeInputForControl(input_index + 4, ecr_l_activation[1]);
        brain->prescribeInputForControl(input_index + 5, ecr_l_activation[2]);
        brain->prescribeControlForActuator("ECRL", "ECRLA_0", "ECRLB_0", "ECRLG_0");

        input_index += 6;
    }

    model.addController(brain);

    // Reaction Forces on Wrist

    jr = new JointReaction(&model);

    Array<std::string> joints;
    joints.append("anchor_joint");

    Array<std::string> on_body;
    on_body.append("child");

    Array<std::string> in_frame;
    in_frame.append("ground");

    jr->setJointNames(joints);
    jr->setOnBody(on_body);
    jr->setInFrame(in_frame);

    model.addAnalysis(jr);

    SimTK::State& state = model.initSystem();


    if (skeletal_model_viz) {
        // Configure the visualizer.
        //model.updMatterSubsystem().setShowDefaultGeometry(true);
        Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundType(viz.SolidColor);
        viz.setBackgroundColor(Black);
    }

    model.equilibrateMuscles(state);

    // // Simulate.
    my_simulate(model, state, sim_length);

    jr->printResults("jointreaction");
    force_reporter->printResults("forces");

    auto table = table_reporter->getTable();

    ofstream dataFile;
    dataFile.open("output.txt");
    for (int i = 0; i < table.getNumRows(); i++) {
        for (int j = 0; j < 7; j++) { // Currently recording 8 metrics...
            dataFile << table.getRowAtIndex(i).getAsVector()[j] << "," << std::flush;
        }
        dataFile << "\n" << std::flush;
    }
    dataFile.close();

    return 0;
};