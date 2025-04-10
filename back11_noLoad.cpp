#include "objInfo1.h"
#include "helpers.h"

int main(int argc, char* argv[]) {
    // std::cout<<"\n\n\nhelloooooasdfadsf\n\n"<<posst[2]<<"\n\n\n";
    ChSystemNSC sys = GravetySetup();

    ChQuaternion<> SpringDamper_Orientation;
    SpringDamper_Orientation.SetFromAngleAxis(90.0 * M_PI / 180.0, ChVector3d(0, 0, 1)); // !!! IMPORTANT !!! the Torsional Spring is oriented always arround Z-axis -> Set correctly the orientation 
    ChQuaterniond jointOrientation;
    jointOrientation.SetFromAngleAxis(90.0 * (CH_PI / 180.0), ChVector3d(0, 0, 1));
    ChVector3d TorqueDir(0, 0, 1); // IMPORTANT!! the direction vertex need to be normalized 
    int AngVelAxis = 2;     // 0-x, 1-y, 2-z
    

    float len = 50, thickk = 2;
    // AddAxis(sys, ChVector3b(0,0,0),len,thickk,thickk, ChColor(1,0,0));
    // AddAxis(sys, ChVector3b(0,0,0),thickk,len,thickk, ChColor(0,1,0));
    // AddAxis(sys, ChVector3b(0,0,0),thickk,thickk,len, ChColor(0,0,1));
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetKdTexture(GetChronoDataFile("textures/pinkwhite.png"));
    std::vector<std::unique_ptr<RigidBody>> bodies(file_locations.size());
    std::vector<std::shared_ptr<ChBody>> body_ptrs(file_locations.size());

    for (size_t i = 1; i < file_locations.size(); ++i) {
        bodies[i] = std::make_unique<RigidBody>(sys, std::get<0>(objData[i])); 
        body_ptrs[i] = bodies[i]->GetBody();
        bodies[i]->setData(objData[i], posOffset);

        if(i==1) bodies[i]->setColor(ChColor(0.5f,0.0f,0.5f));
        if(i==9) bodies[i]->setColor(ChColor(1,1,0));    // FlyWheel
        if(i==4) bodies[i]->setColor(ChColor(0.9f,0.9f,0.9f));    // Frame
    }
    bodies[4]->setTransparent();
    body_ptrs[1]->SetFixed(true);
    body_ptrs[4]->SetFixed(true);
    body_ptrs[10]->SetFixed(true);
    body_ptrs[2]->SetPos(std::get<1>(objData[2]) - posOffset);
    body_ptrs[3]->SetPos(std::get<1>(objData[3]) - posOffset);
    // body_ptrs[2]->SetPos(std::get<1>(objData[2]) - posOffset);
    // body_ptrs[3]->SetPos(std::get<1>(objData[3]) - posOffset);
    auto StatorBody = body_ptrs[1];
    auto RotorBody = body_ptrs[2];
    auto dynoBody = body_ptrs[3];
    auto FlyWheel = body_ptrs[9];
    auto Frame_body = body_ptrs[4];
    
    double radA = 10, radB = 20;

    createJoint(jointOrientation, sys, FlyWheel, Frame_body, JointType::REVOLUTE, std::get<1>(objData[9]) - posOffset);
    createJoint(jointOrientation, sys, FlyWheel, body_ptrs[6], JointType::FIXED, std::get<1>(objData[9]) - posOffset);
    createJoint(jointOrientation, sys, FlyWheel, body_ptrs[7], JointType::FIXED, std::get<1>(objData[9]) - posOffset);
    createJoint(jointOrientation, sys, RotorBody, Frame_body, JointType::REVOLUTE, std::get<1>(objData[2]) - posOffset);
    //createJoint(jointOrientation, sys, RotorBody, Frame_body, JointType::REVOLUTE, std::get<1>(objData[2]) - posOffset, true);
    createJoint(jointOrientation, sys, body_ptrs[5], Frame_body, JointType::REVOLUTE, std::get<1>(objData[5]) - posOffset);
    createJoint(jointOrientation, sys, body_ptrs[6], Frame_body, JointType::REVOLUTE, std::get<1>(objData[6]) - posOffset);
    createJoint(jointOrientation, sys, body_ptrs[7], Frame_body, JointType::REVOLUTE, std::get<1>(objData[7]) - posOffset);
    createJoint(jointOrientation, sys, body_ptrs[8], Frame_body, JointType::REVOLUTE, std::get<1>(objData[8]) - posOffset);
    createJoint(jointOrientation, sys, body_ptrs[3], Frame_body, JointType::REVOLUTE, std::get<1>(objData[3]) - posOffset);

    // gearMate(sys, RotorBody, body_ptrs[5], radA, radB);
    // gearMate(sys, body_ptrs[5], body_ptrs[6], radB, radB);
    // gearMate(sys, body_ptrs[7], body_ptrs[8], radB, radB);
    //gearMate(sys, body_ptrs[8], body_ptrs[3], radB, radA);
    
    // ===========================================================================================================================================================================================
    // ======== F / T DEFINITION -> TORSIONAL SPRING/DAMPER: RotorWinding - Stator ====================================================================================================================================
    
    // ===========================================================================================================================================================================================
    // ======== Torsional spring coefficient ===========================================================================================================================================================================
    double springConst = 0.0; // [(N * m) / rad]
    springConst = springConst * 1e3 * 1e3; // Conversion to ([kg]-[mm]-[s]) 
    // ======== Torsional damping coefficient ===========================================================================================================================================================================
    //double dampConst = 0.0003; //[(N*m*s)/rad]
    //double r_eq_RotorWinding_Stator_spr = dampConst * 1e3 * 1e3; // Conversion to ([kg]-[mm]-[s])  
    // ======== Torsional spring/damper implementation ===========================================================================================================================================================================
    auto springDamper = chrono_types::make_shared<ChLinkRSDA>();
    ChVector3d springDamper_Position(StatorBody->GetPos());  //[mm] set the position in the 3D space of the link respect to the absolute frame
    ChFrame<> springDamper_Frame(springDamper_Position, SpringDamper_Orientation);
    springDamper->Initialize(RotorBody,                                   // Body 1  
        StatorBody,                                  // Body 2 
        false,                                        // the two following frames are in absolute, not relative, coords.
        springDamper_Frame,          // Location and orientation of the Body 1 frame 
        springDamper_Frame);         // Location and orientation of the Body 1 frame
    springDamper->SetRestAngle(0.0 * (M_PI / 180.0)); //[rad] Starting angular position
    springDamper->SetSpringCoefficient(springConst); // [(kg mm mm)/(s^2 rad)] that should be the SI conversion ([kg]-[mm]-[s]) of [N m/rad]
    //springDamper->SetDampingCoefficient(r_eq_RotorWinding_Stator_spr); // [(kg mm mm s)/(s^2 mm rad)] that should be the SI conversion ([kg]-[mm]-[s]) of [N m s/rad]
    sys.AddLink(springDamper);
    springDamper->AddVisualShape(chrono_types::make_shared<ChVisualShapeRotSpring>(60, 50)); // var1 = radius of the spring, var2 = graphical resolution of the spring
    auto RotorWinding_Stator_Spring_Visual = chrono_types::make_shared<ChVisualShapeRotSpring>(2.5, 70); // var1 = radius of the spring, var2 = graphical resolution of the spring
    RotorWinding_Stator_Spring_Visual->SetColor(ChColor(0.0f, 1.0f, 0.0f));  // RGB values
    springDamper->AddVisualShape(RotorWinding_Stator_Spring_Visual); 

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

    // ===========================================================================================================================================================================================
    double t_simulation_STOP = 2.0; //[s]
    double f_ToSample_mechanic = 1.0e3;//1.0e5;//8.0e3;// 0.5e4; // [Hz]
    double t_step_mechanic = 1 / f_ToSample_mechanic; // [s] Simulation Timestep
    double myStep = 1 / f_ToSample_mechanic; // [s] Simulation Timestep
    // ======== Electronic domain ====================================================================================================================================================================
    double f_ToSample_electronic = f_ToSample_mechanic;//1.0e5;// 0.5e4; // [Hz]          Frequency at which the electronic domain is called respect to the global time line
    double T_ToSample_electronic = 1 / f_ToSample_electronic;               // Period at which the electronic domain is called respect to the global time line
    double t_step_electronic = 1.0e-5;//1.0e-6; // ngSpice Timestep                                 Discretization of the electronic time window
    double t_sampling_electronic_counter = 0; //[s] This variable is needed to count the event at which the Electronic domain need to be called respect to the Global Time-line
   
    double time_match_factor = 166.2;
    double time_offset = 49600;
    // double t_sim_mechanics = time_offset/(time_match_factor*1000); //[s] 
    double t_sim_mechanics = 0.0; //[s] 
    double time_synch = time_offset;
    t_simulation_STOP += t_sim_mechanics;

    double Imotor = 0.0;
    double Idyno= 0.0;
    double T_PWM = 0.001; //[s] PWM Period   1 kHz
    double t_PWM_counter = 0.0; //[s] PWM Period
     
    double motorTorque = 0.0 * 1e3 * 1e3; //[Nm] converted to ([kg]-[mm]-[s]) 
    ChVector3d rotorTorque = motorTorque * TorqueDir;
    double loadTorque = 0.0;
    ChVector3d dynoTorque = loadTorque * TorqueDir;

    // ===========================================================================================================================================================================================
    // ======== INITIALIZE THE ELECTRONIC CIRCUIT ====================================================================================================================================================================
    // ===========================================================================================================================================================================================
    std::map<std::string, double> PWLIn = {
        {"VmotorVAR", 0.0},
        {"VpwmVAR", 0.0}
    };
    //double R_motor = 0.4, L_motor = 1.0e-5;
    //std::map<std::string, double> FlowIn = {
    //    {"Rmotor", R_motor},            // checked
    //    {"Lmotor", L_motor}          // checked
    //};

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

    double Duty_PWM = 100.0;              // [%] PWM Duty

    double dcV = 1.8;                     // [V] Supply voltage
    double kv_motor = 3216;              // [rpm/V]
    double kt_motor = 0.001842;              // [Nm/A]
    double ke_motor = 1 / (kv_motor * ((2.0 * M_PI) / 60.0)); // [V/rad/s]
    double B = 0.000001; // [(N*m*s)/rad]
    //double J = 4.695; // kg*mm^2
    double R_motor = 0.15; // [Ohms]
    double L_motor = 6.0e-7; // [H]
   
    if(argc >= 2) {
	dcV = std::stod(argv[1]); 
    }
    
    //body_ptrs[3]->SetInertiaXX(ChVector3d(9.576,9.786,4.288)*1e6);
     
    std::map<std::string, double> FlowIn = {
        {"Rmotor", R_motor},            // checked
        {"Lmotor", L_motor}          // checked
    };

    std::string Netlist_location = "../data/my_project/SPICE/Circuit_Netlist.cir";   
    ChElectronicGeneric Generic_Circuit(Netlist_location, t_step_electronic); 
    Generic_Circuit.Initialize(t_step_mechanic);
    Generic_Circuit.InputDefinition(PWLIn, FlowIn);

    high_resolution_clock::time_point start = high_resolution_clock::now();


    double r_eq_RotorWinding_Stator_spr = B * 1e3* 1e3; // Conversion to ([kg]-[mm]-[s])  

    springDamper->SetDampingCoefficient(r_eq_RotorWinding_Stator_spr);

    double motorSpd = 0.0;               // [rad/s]
    double motorRPM = 0.0;               // [rpm]
    double kv_Drop_factor = 0.0;
    
    int pulseCount = 0;
    int checkIndex = 309;
    int counter = 0;
    
    bool startPwm = true;
    bool once = true;

    double m_Vbackemf = 0.0;
    while (t_sim_mechanics <= t_simulation_STOP && vis->Run()) {
        if(myStep > 0.02){
            myStep = 0;
            // if(Duty_PWM <= 90)Duty_PWM += 10;
            std::cout<<Duty_PWM<<"\t";
        }
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        ChVector3d Rotor_Euler_Vel;
        ChVector3d dyno_Euler_Vel;

        if (t_sampling_electronic_counter >= T_ToSample_electronic) {
             
            PWLIn["VpwmVAR"] = dcV; 
            ///if (t_sim_mechanics >= 0.0){
            ///    if (t_PWM_counter <= T_PWM * Duty_PWM/100 && startPwm){
            ///        PWLIn["VpwmVAR"] = dcV;     
            ///    } else{   
            ///        PWLIn["VpwmVAR"] = 0.0;     
            ///    }
            ///    t_PWM_counter += t_step_mechanic;
            ///    if (t_PWM_counter > T_PWM){
	    ///        t_PWM_counter = 0.0;
	    ///        pulseCount=0;
	    ///    }
	    ///}

            PWLIn["VmotorVAR"] = -m_Vbackemf;
            Generic_Circuit.InputDefinition(PWLIn, FlowIn);
		
	    // Step motor
	    Generic_Circuit.Advance(t_step_mechanic);
            auto res1 = Generic_Circuit.GetResult();
	    
	    m_Vbackemf = ke_motor * motorSpd;
            Imotor = -res1[toLowerCase("VmotorVAR")].back();

	    //motorRPM = RotorBody->GetAngVelLocal()[AngVelAxis];      // rpm 
            //motorSpd = motorRPM * 0.10472; 	     // rpm -> rad/s
            
            motorSpd = RotorBody->GetAngVelLocal()[AngVelAxis];      // rad/s
            motorRPM = motorSpd * (60.0 / (2.0 * M_PI)); 	     // rad/s -> rpm
            std::cout << "RPM: " <<  motorRPM << "\t";
           
            // for (const auto& [key, values] : res1) { 
            //     std::cout << key << ": " << res1[key].back() << "\t";
            // }
            
            OutputMap["Applied_Volt"].push_back(res1["n1"].back());
            OutputMap["Back_EMF"].push_back(m_Vbackemf);
            OutputMap["Current"].push_back(-res1[toLowerCase("VmotorVAR")].back());
            OutputMap["angVel"].push_back(motorRPM);

            // ======== UPDATE -> the TIME variables ====================================================================================================================================================================
            t_sampling_electronic_counter = 0;      // The variable is nulled to re-start with the counter for the next call of the electronic domain
        }

        // ======== EXTRACT -> Kinematic variables ====================================================================================================================================================================
        std::vector<double> Rotor_Euler_Ang = GetEulerAngPos(RotorBody, t_step_mechanic);

        if(1){
            motorTorque = kt_motor * Imotor * 1e3 * 1e3; // Conversion to ([kg]-[mm]^2/[s^2])
            rotorTorque = motorTorque * TorqueDir;
            RotorBody->EmptyAccumulators(); 
	    RotorBody->AccumulateTorque(rotorTorque, true); // Apply to the body the force
	    RotorBody->AccumulateTorque(-1882.*ChVector3d(0,0,1), true);
        }

        std::cout<<"Imotor "<< Imotor << '\t';
        std::cout<<"motorTorque "<< motorTorque << '\t';
            // std::cout<<"given\n";

        // ======== RUN -> the Mechanic solver ====================================================================================================================================
        sys.DoStepDynamics(t_step_mechanic);
        // realtime_timer.Spin(t_step_mechanic);

        // ======== UPDATE -> the Multi-physics timeline ====================================================================================================================================
        t_sampling_electronic_counter += t_step_mechanic;
        t_sim_mechanics += t_step_mechanic;
        myStep += t_step_mechanic;

        high_resolution_clock::time_point end = high_resolution_clock::now();
        duration<double, std::milli> duration_sec = std::chrono::duration_cast<duration<double, std::milli>>(end - start);
        auto time_passed = duration_sec.count() + time_offset;
        
        
        double matched_time = t_sim_mechanics*time_match_factor*1000;
        if(once){
            // time_synch = matched_time;
            once=false;
        }
        if(!(time_synch < matched_time && matched_time < time_synch + 100 )){time_synch += 100;}
        
        // std::cout<<"matched_time "<< matched_time << '\t';
        // std::cout<<"time_passed "<< time_passed << '\t';

        // double sinWave = 0.75 * sin(400 * t_sim_mechanics); 
        loadTorque = InputMap["CounterTorque"][checkIndex];
        dynoTorque = 1000.0 * loadTorque * TorqueDir;
        dynoBody->EmptyAccumulators(); // Clean the body from the previous force/torque IMPORTANT!!!!: Uncomment this line if you never clean the F/T to this body
        // dynoBody->AccumulateTorque(dynoTorque, true); // Apply to the body the force
        
        // ======== SAVE -> the needed variables ====================================================================================================================================================================
        OutputMap["loadTorque"].push_back(loadTorque);
        OutputMap["T_motor"].push_back(motorTorque);
        OutputMap["angVel_data"].push_back(InputMap["RPM"][checkIndex]);
        OutputMap["PWM"].push_back(InputMap["PWM"][checkIndex]);
        OutputMap["t_mechanics"].push_back(t_sim_mechanics);

        // std::cout << "loadTorque: " << loadTorque/1e6 << "Nm \t";
        std::cout << std::endl;
    }

    int readCount = seeCache("cache.txt");

    std::ostringstream oss;
    oss << "outputPlot/run_" << readCount << "_output_" << f_ToSample_mechanic << "_" << abs(t_step_electronic) << ".csv";
    std::string filename = oss.str();
    std::cout << filename << std::endl;
    
    // Open the CSV file for writing
    std::ofstream out_file(filename);
    
    // First, write the header (column names)
    bool header_written = false;
    for (auto it = OutputMap.begin(); it != OutputMap.end(); ++it) {
        out_file << it->first;
        if (std::next(it) != OutputMap.end()) out_file << ",";
    }
    out_file << "\n";

    size_t num_rows = 0;
    for (const auto& pair : OutputMap) {
        num_rows = std::max(num_rows, pair.second.size());
    }
    
    // Write rows safely
    for (size_t i = 0; i < num_rows; ++i) {
        for (auto it = OutputMap.begin(); it != OutputMap.end(); ++it) {
            const auto& vec = it->second;
            if (i < vec.size()) {
                out_file << vec[i];
            } else {
                out_file << "";  // Leave blank for missing data
            }
            if (std::next(it) != OutputMap.end()) out_file << ",";
        }
        out_file << "\n";
    }
    out_file.close();



    std::cout << "\t exported" << std::endl;
    return 0;
}

