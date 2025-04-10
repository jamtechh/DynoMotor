#include "objInfo1.h"

double Duty_PWM = 100.0; //[s] PWM Duty
double dcV = 7.4; // Volt
double R_motor = 0.15;
double L_motor = 1.0e-5;
double kv_motor = 3216.117534;
double ke_motor =  1/(kv_motor * ((2.0 * M_PI)/60.0)); //[V/rpm]
// double ke_motor =  0.002969;
double kt_motor = .001842; //[Nm/A]
double dampConst = 0.000001; //[(N*m*s)/rad]

int main(int argc, char* argv[]) {
    if (argc > 1) dcV = std::stod(argv[1]);
    if (argc > 2) R_motor = std::stod(  argv[2]);
    if (argc > 3) L_motor = std::stod(  argv[3]);
    if (argc > 4) kv_motor = std::stod( argv[4]);
    if (argc > 5) kt_motor = std::stod( argv[5]);
    if (argc > 6) dampConst = std::stod( argv[6]);
    
    int counter = 0;
    bool startPwm = 1;
    bool once = true;
    double motorSpd = 0.0;
    double motorRPM = 0.0;
    int pulseCount = 0;

    ChSystemNSC sys = GravetySetup();

    ChQuaternion<> SpringDamper_Orientation;
    SpringDamper_Orientation.SetFromAngleAxis(90.0 * M_PI / 180.0, ChVector3d(0, 0, 1)); // !!! IMPORTANT !!! the Torsional Spring is oriented always arround Z-axis -> Set correctly the orientation 
    jointOrientation.SetFromAngleAxis(90.0 * (CH_PI / 180.0), ChVector3d(0, 0, 1));
    ChVector3d TorqueDir(0, 0, 1); // IMPORTANT!! the direction vertex need to be normalized 
    int AngVelAxis = 2;     // 0-x, 1-y, 2-z
    
    std::vector<std::unique_ptr<RigidBody>> bodies(file_locations.size());
    std::vector<std::shared_ptr<ChBody>> body_ptrs(file_locations.size());

    for (size_t i = 1; i < file_locations.size(); ++i) {
        bodies[i] = std::make_unique<RigidBody>(sys, std::get<0>(objData[i])); 
        body_ptrs[i] = bodies[i]->GetBody();
        bodies[i]->setData(objData[i]);
        if(i==1)bodies[i]->setColor(ChColor(0.5f,0.0f,0.5f));
        if(i==9)bodies[i]->setColor(ChColor(1,1,0));    // FlyWheel
        if(i==4)bodies[i]->setColor(ChColor(0.9f,0.9f,0.9f));    // Frame
    }
    bodies[4]->setTransparent();
    body_ptrs[1]->SetFixed(true);
    body_ptrs[4]->SetFixed(true);
    body_ptrs[10]->SetFixed(true);
    body_ptrs[2]->SetPos(std::get<1>(objData[2]) - posOffset);
    body_ptrs[3]->SetPos(std::get<1>(objData[3]) - posOffset);

    auto StatorBody = body_ptrs[1];
    auto RotorBody = body_ptrs[2];
    auto dynoBody = body_ptrs[3];
    auto FlyWheel = body_ptrs[9];
    Frame_body = body_ptrs[4];
    
    double radA = 10, radB = 20;
    createJoint(sys, FlyWheel, Frame_body, JointType::REVOLUTE, std::get<1>(objData[9]) - posOffset);
    createJoint(sys, FlyWheel, body_ptrs[6], JointType::FIXED, std::get<1>(objData[9]) - posOffset);
    createJoint(sys, FlyWheel, body_ptrs[7], JointType::FIXED, std::get<1>(objData[9]) - posOffset);
    createJoint(sys, RotorBody, Frame_body, JointType::REVOLUTE, std::get<1>(objData[2]) - posOffset);
    createJoint(sys, RotorBody, Frame_body, JointType::REVOLUTE, std::get<1>(objData[2]) - posOffset, true);
    createJoint(sys, body_ptrs[5], Frame_body, JointType::REVOLUTE, std::get<1>(objData[5]) - posOffset);
    createJoint(sys, body_ptrs[6], Frame_body, JointType::REVOLUTE, std::get<1>(objData[6]) - posOffset);
    createJoint(sys, body_ptrs[7], Frame_body, JointType::REVOLUTE, std::get<1>(objData[7]) - posOffset);
    createJoint(sys, body_ptrs[8], Frame_body, JointType::REVOLUTE, std::get<1>(objData[8]) - posOffset);
    createJoint(sys, body_ptrs[3], Frame_body, JointType::REVOLUTE, std::get<1>(objData[3]) - posOffset);

    // printf("\t\t\t\t 7 \n\n\n");
    gearMate(sys, RotorBody, body_ptrs[5], radA, radB);
    // gearMate(sys, body_ptrs[5], body_ptrs[6], radB, radB);
    // gearMate(sys, body_ptrs[7], body_ptrs[8], radB, radB);
    gearMate(sys, body_ptrs[8], body_ptrs[3], radB, radA);
    

    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(1200, 800);
    vis->SetWindowTitle("Modeling a simplified trackjed vehicle");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(-50, 20, -500), std::get<1>(objData[2]) - posOffset);
    // auto camera = vis->GetActiveCamera();
    vis->AddLight(ChVector3d(0, 0, -900), 950, ChColor(0.1f, 0.1f, 0.1f));
    vis->AddLight(ChVector3d(0, 0, 900), 1100, ChColor(0.1f, 0.1f, 0.1f));
    vis->AddLight(ChVector3d(-300, -500, 0), 2000, ChColor(0.1f, 0.1f, 0.1f));
    vis->AddLight(ChVector3d(-300, 500, 0), 2000, ChColor(0.1f, 0.1f, 0.1f));
    vis->EnableBodyFrameDrawing(true);
    vis->EnableLinkFrameDrawing(true);
    // camera->setTarget(irr::core::vector3df(1300, 0, 0));

    // ===========================================================================================================================================================================================
    // ======== SOLVER SETTINGS ====================================================================================================================================================================
    // ===========================================================================================================================================================================================
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);
    //sys.SetTimestepperType(ChTimestepper::Type::RUNGEKUTTA45);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(1000.0);
    sys.SetMaxPenetrationRecoverySpeed(1000.1);
    sys.SetMinBounceSpeed(0.001);
    ChRealtimeStepTimer realtime_timer;

    // printf("\t\t\t\t 8 \n\n\n");
    // ===========================================================================================================================================================================================
    double t_simulation_STOP = 10.0; //[s]
    double f_ToSample_mechanic = 1.0e2;//1.0e5;//8.0e3;// 0.5e4; // [Hz]
    double t_step_mechanic = 1 / f_ToSample_mechanic; // [s] Simulation Timestep
    double myStep = 1 / f_ToSample_mechanic; // [s] Simulation Timestep
    // ======== Electronic domain ====================================================================================================================================================================
    double f_ToSample_electronic = f_ToSample_mechanic;//1.0e5;// 0.5e4; // [Hz]          Frequency at which the electronic domain is called respect to the global time line
    double T_ToSample_electronic = 1 / f_ToSample_electronic;               // Period at which the electronic domain is called respect to the global time line
    double t_step_electronic = 1.0e-4;//1.0e-6; // ngSpice Timestep                                 Discretization of the electronic time window
    double t_sampling_electronic_counter = 0; //[s] This variable is needed to count the event at which the Electronic domain need to be called respect to the Global Time-line
    double t_sim_mechanics = 0.0; //[s] 

    double Imotor = 0.0;
    double T_PWM = 0.001; //[s] PWM Period   1 kHz
    double t_PWM_counter = 0.0; //[s] PWM Period
     
    double motorTorque = 0.0 * 1e3 * 1e3; //[Nm] converted to ([kg]-[mm]-[s]) 
    ChVector3d rotorTorque = motorTorque * TorqueDir;
    double loadTorque = 0.0;
    ChVector3d dynoTorque = loadTorque * TorqueDir;

    // printf("\t\t\t\t 9 \n\n\n");
    // ===========================================================================================================================================================================================
    // ======== INITIALIZE THE ELECTRONIC CIRCUIT ====================================================================================================================================================================
    // ===========================================================================================================================================================================================
    std::map<std::string, double> PWLIn = {
        {"VmotorVAR", 0.0},
        {"VpwmVAR", 0.0}
    };
    std::map<std::string, double> FlowIn = {
        {"Rmotor", R_motor},            // checked
        {"Lmotor", L_motor}          // checked
    };

    std::map<std::string, std::vector<double>> OutputMap;
    std::map<std::string, std::vector<double>> InputMap;
    OutputMap["Applied_Volt"] = {};
    OutputMap["Back_EMF"] = {};
    OutputMap["Current"] = {};
    OutputMap["t_electronics"] = {};
    OutputMap["loadTorque"] = {};
    OutputMap["angVel"] = {};
    OutputMap["angVel_data"] = {};
    OutputMap["t_mechanics"] = {};
    OutputMap["T_magnetic"] = {};
    OutputMap["T_motor"] = {};
    OutputMap["PWM"] = {};

    std::ifstream inputFile("../data/dynoTestData/run1_load1.csv");
    std::ofstream outputFile("output.csv");

    if (!inputFile.is_open() || !outputFile.is_open()) {
        std::cerr << "Error opening files.\n";
        return 1;
    }

    std::string line;
    std::vector<std::string> headers;

    // Read header row
    if (std::getline(inputFile, line)) {
        std::stringstream ss(line);
        std::string column;
        while (std::getline(ss, column, ',')) {
            headers.push_back(column);
            InputMap[column] = {};
        }
    }

    // printf("\t\t\t\t 10 \n\n\n");
    // Read data rows into InputMap
    while (std::getline(inputFile, line)) {
        std::stringstream ss(line);
        std::string value;
        size_t colIdx = 0;

        while (std::getline(ss, value, ',')) {
            if (colIdx < headers.size()) {
                InputMap[headers[colIdx]].push_back(std::stod(value));
            }
            ++colIdx;
        }
    }
    // printf("\t\t\t\t 11 \n\n\n");
    std::string Netlist_location = "../data/my_project/SPICE/Circuit_Netlist.cir";   
    ChElectronicGeneric Generic_Circuit(Netlist_location, t_step_electronic); 
    Generic_Circuit.Initialize(t_step_mechanic);
    Generic_Circuit.InputDefinition(PWLIn, FlowIn);
    // printf("\t\t\t\t 12 \n\n\n");

    setDamper(sys, StatorBody, RotorBody, SpringDamper_Orientation, dampConst);

    high_resolution_clock::time_point start = high_resolution_clock::now();
    // RotorBody->AccumulateTorque(0.058 * 1e3 * 1e3, true); // Apply to the body the force

    double prevRpm = 0.0;
    double diffRpm = 0.0;
    double threshold = 1;
    while (t_sim_mechanics <= t_simulation_STOP && vis->Run()) {
        // printf("\t\t\t\t 13 - Frame Start\n\n\n");
        
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
    
        // printf("\t\t\t\t 14 - Visualization Done\n\n\n");
    
        ChVector3d Rotor_Euler_Vel;
    
        if (t_sampling_electronic_counter >= T_ToSample_electronic) {
            Generic_Circuit.Advance(t_step_mechanic);
            auto res1 = Generic_Circuit.GetResult();
    
            Rotor_Euler_Vel = RotorBody->GetAngVelLocal();
            motorSpd = Rotor_Euler_Vel[AngVelAxis];
            motorRPM = motorSpd * (60.0 / (2.0 * M_PI));
            // std::cout << "RPM: " << motorRPM << "\t";

            diffRpm = motorRPM - prevRpm;
            // std::cout << "difference: " << diffRpm << "\t";
            if(abs(diffRpm) < threshold)break;
            else prevRpm = motorRPM;
    
            kv_motor = kv_motor;
            ke_motor = 1 / (kv_motor * ((2.0 * M_PI) / 60.0));
    
            double Vbackemf = ke_motor * Rotor_Euler_Vel[AngVelAxis];
            Imotor = -res1[toLowerCase("VmotorVAR")].back();
    
            if (t_sim_mechanics >= 0.0) {
                if (t_PWM_counter <= T_PWM * Duty_PWM / 100 && startPwm) {
                    PWLIn["VpwmVAR"] = dcV;
                    t_PWM_counter += t_step_mechanic;
                } else {
                    PWLIn["VpwmVAR"] = 0.0;
                    t_PWM_counter += t_step_mechanic;
                }
                if (t_PWM_counter > T_PWM) {
                    t_PWM_counter = 0.0;
                    pulseCount = 0;
                }
            }
    
            PWLIn["VmotorVAR"] = -Vbackemf;
            Generic_Circuit.InputDefinition(PWLIn, FlowIn);
    
            OutputMap["Applied_Volt"].push_back(res1["n1"].back());
            OutputMap["Back_EMF"].push_back(Vbackemf);
            OutputMap["Current"].push_back(-res1[toLowerCase("VmotorVAR")].back());
            OutputMap["angVel"].push_back(motorRPM);
    
            t_sampling_electronic_counter = 0;
            // printf("\t\t\t\t 15 - Electronic Circuit Advanced\n\n\n");
        }
    
        std::vector<double> Rotor_Euler_Ang = GetEulerAngPos(RotorBody, t_step_mechanic);
    
        motorTorque = kt_motor * Imotor * 1e3 * 1e3;
        rotorTorque = motorTorque * TorqueDir;
        RotorBody->EmptyAccumulators();
        RotorBody->AccumulateTorque(rotorTorque, true);
	    RotorBody->AccumulateTorque(-1882.*ChVector3d(0,0,1), true);
    
        // printf("\t\t\t\t 16 - Torque Applied to Rotor\n\n\n");
    
        sys.DoStepDynamics(t_step_mechanic);
        // printf("\t\t\t\t 17 - Mechanical Step Done\n\n\n");
    
        t_sampling_electronic_counter += t_step_mechanic;
        t_sim_mechanics += t_step_mechanic;
        myStep += t_step_mechanic;
    
        high_resolution_clock::time_point end = high_resolution_clock::now();
        duration<double, std::milli> duration_sec = std::chrono::duration_cast<duration<double, std::milli>>(end - start);
        auto time_passed = duration_sec.count();
    
        loadTorque = 10;
        // dynoTorque = 1000.0 * loadTorque * TorqueDir;
        // dynoBody->EmptyAccumulators();
        // dynoBody->AccumulateTorque(dynoTorque, true);
    
        OutputMap["loadTorque"].push_back(loadTorque);
        OutputMap["T_motor"].push_back(motorTorque);
        // OutputMap["angVel_data"].push_back(InputMap["RPM"][checkIndex]);
        // OutputMap["PWM"].push_back(InputMap["PWM"][checkIndex]);
        OutputMap["t_mechanics"].push_back(t_sim_mechanics);
    
        // printf("\t\t\t\t 18 - Output Values Logged\n\n\n");
        std::cout << dcV << ","  << motorRPM << ","  << Imotor << ","  << R_motor << ","  << kv_motor << ","  << kt_motor << '\n';
    }

    // int readCount = seeCache("cache.txt");

    // std::ostringstream oss;
    // oss << "outputPlot/run_" << readCount << "_output_" << f_ToSample_mechanic << "_" << abs(t_step_electronic) << ".json";
    // std::string filename = oss.str();
    // std::cout<< filename;

    // json j; // Create a json object to contain the output data
    // for (const auto& item : OutputMap) { // Populate the JSON object with data
    //     j[item.first] = item.second;
    // }
    // j["meta"] = {
    //     {"timestep", t_step_mechanic},
    //     {"kv_motor", kv_motor},
    //     {"kt_motor", kt_motor},
    //     {"ke_motor", ke_motor},
    //     {"Resistance", R_motor},
    //     {"Inductance", L_motor},
    //     {"dampConst", dampConst},
    //     {"AppliedVolt", dcV},
    //     {"version", "v1.2"} 
    // };
    // // Export the output data in a .json file
    // std::ofstream out_file(filename);
    // out_file << j.dump(4); // "4" is the indentation parameter, you can change it to have a more or less readable structure
    // out_file.close();
    // std::cout << "\t exported" << std::endl;


    return 0;
}