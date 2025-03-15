#include "objInfo1.h"

// ======== Class: allows to compute the integral in-between a single simulation time-step through the cumulative trapezoidal method ==============================================================================================================================================
class CumTrapezIntegration {
public:
    double Integrate(double& dt, double& f_new)
    {
        f_new1 = f_new;
        dt1 = dt;
        Integral_res += dt1 * ((f_old1 + f_new1) / 2);
        //std::cout << "\n!!!!! f_new1: " << f_new1 << " !!!!!\n";            // DEBUG: Scope some needed results
        //std::cout << "\n!!!!! f_old1: " << f_old1 << " !!!!!\n";            // DEBUG: Scope some needed results
        f_old1 = f_new1;
        return Integral_res;
    }
private:
    double Integral_res = 0.0;
    double f_old1 = 0.0;
    double f_new1;
    double dt1;
};

// ======== Method: calculate the effective Euler angular position of a body from the angular velocity along x-y-z- axis ==============================================================================================================================================
std::vector<double> GetEulerAngPos(std::shared_ptr<chrono::ChBody> body, double& t_step_mechanic)
{
    // Get the effective angular velocity along x-y-z axis
    ChVector3d body_Euler_Vel = body->GetAngVelLocal(); // Get the angular velocity 
    double Rotor_Euler_dt_Yaw = body_Euler_Vel[0];
    double Rotor_Euler_dt_Pitch = body_Euler_Vel[1];
    double Rotor_Euler_dt_Roll = body_Euler_Vel[2];

    // Create the object only once through a static variable (the static variable allows to initialize it only once during the execution of the entire code)
    static CumTrapezIntegration body_Euler_Yaw_Integrator;
    static CumTrapezIntegration body_Euler_Pitch_Integrator;
    static CumTrapezIntegration body_Euler_Roll_Integrator;

    // Compute the effective angular position along x-y-z axis
    double body_Euler_Yaw = body_Euler_Yaw_Integrator.Integrate(t_step_mechanic, body_Euler_Vel[0]);
    double body_Euler_Pitch = body_Euler_Pitch_Integrator.Integrate(t_step_mechanic, body_Euler_Vel[1]);
    double body_Euler_Roll = body_Euler_Roll_Integrator.Integrate(t_step_mechanic, body_Euler_Vel[2]);

    // Populate the result vector
    std::vector<double> Results = { body_Euler_Yaw , body_Euler_Pitch, body_Euler_Roll };

    return Results;
}

// ======== Method: converts all the characters in the input string to lowercase and returns the resulting string ==============================================================================================================================================
std::string toLowerCase(const std::string& str) {
    std::string lower_str = str;
    std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(),
                [](unsigned char c){ return std::tolower(c); });
    return lower_str;
}

std::shared_ptr<ChBody> Frame_body;
ChQuaternion<> jointOrientation;

enum class JointType {
    FIXED,
    REVOLUTE,
    PRISMATIC
};
ChSystemNSC GravetySetup(){
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    ChSystemNSC sys; // Create a Chrono physical system
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    
    ChVector3d gravity_acc = sys.GetGravitationalAcceleration(); 
    std::cout << "The gravity acceleration  vector is: " << gravity_acc << "\n\n";
    double gravity = 9.81e3; //[mm/s^2]
    sys.SetGravitationalAcceleration(ChVector3d(gravity, 0, 0));
    ChVector3d gravity_acc_new = sys.GetGravitationalAcceleration(); 
    std::cout << "The new gravity acceleration  vector is: " << gravity_acc_new << "\n\n";
    return sys;
}           
void AddVisualizationBall(ChSystemNSC& sys, const ChVector3d& position, const ChColor& color = ChColor(1,0,0), int rad = 8){
    // set a ball on the center of mass_calc of the frame for visualization purposes        
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.4f);
    mat->SetCompliance(0.0);
    mat->SetComplianceT(0.0);
    mat->SetDampingF(0.2f);
    auto mrigidBall = chrono_types::make_shared<ChBodyEasySphere>(rad, 10, true, true, mat);
    mrigidBall->SetPos(position);
    mrigidBall->SetPosDt(ChVector3d(0, 0, 0));  // set initial speed
    auto sphere_shape = chrono_types::make_shared<ChVisualShapeSphere>(rad);
    sphere_shape->SetColor(color);
    mrigidBall->AddVisualShape(sphere_shape);
    mrigidBall->SetFixed(true);
    sys.Add(mrigidBall);
}
void AddAxis(ChSystemNSC& sys, const ChVector3d& position, float x = 2, float y = 2, float z = 2, const ChColor& color = ChColor(1.0f, 0.0f, 0.0f)){
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto x_axis = chrono_types::make_shared<ChBodyEasyBox>(
        x, y, z, 1000, true, false, mat);
    x_axis->SetPos(ChVector3d(position[0]-x/2, position[1]-y/2, position[2]-z/2));  // Position along X-axis
    x_axis->SetFixed(true);
    x_axis->GetVisualShape(0)->SetColor(color); // Red color
    sys.Add(x_axis);
}
void AddCylAxis(ChSystemNSC& sys, const ChVector3d& position, ChQuaternion<> jointOrientation, float rad = 0.5, float len = 100, const ChColor& color = ChColor(0.0f, 0.0f, 1.0f)){
    auto axisShape = chrono_types::make_shared<ChVisualShapeCylinder>(rad, len); // Radius = 2, Length = 50
    axisShape->SetColor(color);
    auto axisBody = chrono_types::make_shared<ChBody>();
    axisBody->SetPos(position);
    axisBody->SetFixed(true); // The axis is just for visualization
    axisBody->AddVisualShape(axisShape, ChFrame<>(ChVector3d(0, 0, 0), jointOrientation));
    sys.Add(axisBody);
}
void createJoint(ChSystemNSC& sys, std::shared_ptr<ChBody> bodyA, std::shared_ptr<ChBody> bodyB, JointType jointType, const ChVector3d& position, bool showAxis = false) {
    ChFrame<> jointFrame(position, jointOrientation);

    std::shared_ptr<ChLinkLock> Joint;
    switch (jointType) {
        case JointType::FIXED:      {Joint = chrono_types::make_shared<ChLinkLockLock>();      break;}
        case JointType::REVOLUTE:   {Joint = chrono_types::make_shared<ChLinkLockRevolute>();  break;}
        case JointType::PRISMATIC:  {Joint = chrono_types::make_shared<ChLinkLockPrismatic>(); break;}
        default:    throw std::invalid_argument("Invalid joint type.");
    }

    Joint->Initialize(bodyA, bodyB, jointFrame);  
    sys.AddLink(Joint);
    
    if(showAxis)AddCylAxis(sys, bodyA->GetPos(), jointOrientation);
}
ChFrame<> GetFramee(std::shared_ptr<ChBody> body, const ChVector3d& orinn = ChVector3d(1, 0, 0)){
    ChVector3d jointPos(body->GetPos());
    ChQuaternion<> jointOr;
    jointOr.SetFromAngleAxis(90.0 * (CH_PI / 180.0), orinn);    
    ChFrame<> jointFrame(jointPos, jointOr);
    return jointFrame;
}
std::shared_ptr<ChBodyEasyCylinder> makeGears(ChSystemNSC& sys, float rad, const ChVector3d& position){
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetKdTexture(GetChronoDataFile("textures/pinkwhite.png"));
    auto gear = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Z, rad, 0.4, 1000, true, false, mat);
    gear->SetPos(position);
    gear->SetRot(QuatFromAngleX(CH_PI_2));
    gear->GetVisualShape(0)->SetMaterial(0, vis_mat);
    sys.Add(gear);

    auto link_revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    link_revolute->Initialize(gear, Frame_body, ChFrame<>(position, jointOrientation));
    sys.AddLink(link_revolute);

    return gear;
}
void gearMate(ChSystemNSC& sys, std::shared_ptr<ChBody> bodyA, std::shared_ptr<ChBody> bodyB, double radA, double radB){
    auto link_gearAB = chrono_types::make_shared<ChLinkLockGear>();
    link_gearAB->Initialize(bodyA, bodyB, ChFrame<>());
    link_gearAB->SetFrameShaft1(ChFrame<>(VNULL, chrono::QuatFromAngleZ(CH_PI_2)));  // Shaft 1 at gear A
    link_gearAB->SetFrameShaft2(ChFrame<>(VNULL, chrono::QuatFromAngleZ(CH_PI_2)));  // Shaft 2 at gear B
    link_gearAB->SetTransmissionRatio(radA / radB);
    // link_gearAB->SetEnforcePhase(true);
    sys.AddLink(link_gearAB);
}

class RigidBody {
    public:
        RigidBody(ChSystemNSC& sys, const std::string& file_name, bool is_fixed = false)
            : system(sys), obj_file(file_name), is_fixed(is_fixed) {SetupRigidBody();}
    
        std::shared_ptr<ChBody> GetBody() const {return body;}
        ChVector3d GetCOG() const {return cog;}
        ChVector3d getPos() const {return body->GetPos();}
        std::tuple<std::shared_ptr<ChBody>, ChVector3d> GetBodyAndCOG() const {return std::make_tuple(body, cog);}
        void HideBody() {mesh->SetVisible(false);}
        void ShowCG() {AddVisualizationBall(system, body->GetPos());}
        void setPos(ChVector3d poss){body->SetPos(poss);}
        void setColor(ChColor colll){mesh->SetColor(colll);}
        
        void setData(const std::tuple<std::string, ChVector3d, ChQuaternion<>, ChVector3d, double>& objData) { 
            // Get the Values
            position = std::get<1>(objData);
            rotation = std::get<2>(objData);  
            inertia_SW = std::get<3>(objData);  
            mass_SW = std::get<4>(objData); 
            if(prnt)std::cout << mass_SW << "\n\n\n";

            // Set the Values
            body->SetPos(position-posOffset);
            body->SetRot(rotation);
            body->SetMass(mass_SW);
            body->SetInertiaXX(inertia_SW);
        }
    private:
        bool prnt=0;
        ChSystemNSC& system;
        std::string obj_file;
        double density;
        bool is_fixed;
        std::shared_ptr<ChBody> body;
        std::shared_ptr<ChVisualShapeTriangleMesh> mesh;
        std::shared_ptr<ChVisualShapeTriangleMesh> coll_mesh;
        std::shared_ptr<ChCollisionModel> coll_model;
        ChVector3d cog;
        ChVector3d position;
        ChQuaternion<> rotation;
        double mass_calc;
        ChMatrix33<> inertia_calc;
        ChMatrix33<> geometric_inertia_calc;

        double mass_SW;
        ChVector3d inertia_SW;

    
        void SetupRigidBody() {
            // Load visualization mesh
            obj_file = std::string("my_project/CAD/View/") + obj_file + std::string(".obj");
            auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(obj_file));
            std::string coll_file = obj_file;
            if (coll_file.find("body") == std::string::npos){
                std::cout<<"\t\t\treplacing!!!!"<<std::endl;
                coll_file.replace(coll_file.find("View"), 4, "Collision");
                coll_file.replace(coll_file.find("_OBJ"), 4, "_Collision_OBJ");
            }            
            auto coll_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(coll_file));
            // std::cout<<"\t\t\t2"<<std::endl;
    
            // Compute mass_calc properties
            double volume;
            trimesh->ComputeMassProperties(true, volume, cog, geometric_inertia_calc);
            // std::cout<<"\t\t\t3"<<std::endl;
    
            // Calculate mass_calc and inertia_calc
            density = 7850.00 / (1e9);
            mass_calc = density * volume;
            inertia_calc = density * geometric_inertia_calc;
    
            // Create rigid body
            body = chrono_types::make_shared<ChBody>();
            body->SetFixed(is_fixed);
            // body->SetMass(mass_calc);
            // body->SetInertiaXX(ChVector3d(inertia_calc(0, 0), inertia_calc(1, 1), inertia_calc(2, 2)));
            if(prnt){
            std::cout <<"mass_calc = "<< mass_calc << "  \t";
            std::cout <<"volume = "<< volume << "   \t";
            std::cout<<geometric_inertia_calc(0,0)<<" \t"<<geometric_inertia_calc(1,1)<<"    \t"<<geometric_inertia_calc(2,2)<<std::endl;
            }

            system.Add(body);
    
            // Visualization
            mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            mesh->SetMesh(trimesh);
            body->AddVisualShape(mesh, ChFrame<>(ChVector3d(0,0,0), ChMatrix33<>(1)));
    
            // Collision
            coll_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            coll_mesh->SetMesh(coll_trimesh);
            // coll_mesh->SetVisible(true);
            // body->AddVisualShape(coll_mesh, ChFrame<>(ChVector3d(0,0,0), ChMatrix33<>(1)));
    
            // Setup collision model
            coll_model = chrono_types::make_shared<ChCollisionModel>();
            coll_model->SetSafeMargin(0.1f);
            coll_model->SetEnvelope(0.001f);
            coll_trimesh->Transform(position, ChMatrix33<>(1));
            auto coll_mat = chrono_types::make_shared<ChContactMaterialNSC>();
            coll_mat->SetFriction(0.30);
            coll_mat->SetRestitution(0.001);
            auto coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(coll_mat, coll_trimesh, false, false, 0.001);
            coll_model->AddShape(coll_shape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
            body->AddCollisionModel(coll_model);
            // body->EnableCollision(true);

            body->GetCollisionModel()->SetFamily(1);
            body->GetCollisionModel()->DisallowCollisionsWith(2);
        }
    };

// ===============================================================================================================================================================================
// ======== MAIN LOOP ==============================================================================================================================================
// ===============================================================================================================================================================================

int main(int argc, char* argv[]) {
    // std::cout<<"\n\n\nhelloooooasdfadsf\n\n"<<posst[2]<<"\n\n\n";
    ChSystemNSC sys = GravetySetup();
    jointOrientation.SetFromAngleAxis(90.0 * (CH_PI / 180.0), ChVector3d(1, 0, 0));
    float len = 50, thickk = 2;
    AddAxis(sys, ChVector3b(0,0,0),len,thickk,thickk, ChColor(1,0,0));
    AddAxis(sys, ChVector3b(0,0,0),thickk,len,thickk, ChColor(0,1,0));
    AddAxis(sys, ChVector3b(0,0,0),thickk,thickk,len, ChColor(0,0,1));
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetKdTexture(GetChronoDataFile("textures/pinkwhite.png"));
    std::vector<std::unique_ptr<RigidBody>> bodies(file_names.size());
    std::vector<std::shared_ptr<ChBody>> body_ptrs(file_names.size());

    for (size_t i = 1; i < file_names.size(); ++i) {
        bodies[i] = std::make_unique<RigidBody>(sys, std::get<0>(objData[i])); 
        body_ptrs[i] = bodies[i]->GetBody();
        bodies[i]->setData(objData[i]);
        if(i==1)bodies[i]->setColor(ChColor(0.5f,0.0f,0.5f));
        if(i==9)bodies[i]->setColor(ChColor(1,1,0));    // FlyWheel
        if(i==4)bodies[i]->setColor(ChColor(0.9f,0.9f,0.9f));    // Frame
    }
    body_ptrs[1]->SetFixed(true);
    body_ptrs[4]->SetFixed(true);
    body_ptrs[10]->SetFixed(true);
    body_ptrs[2]->SetPos(positions[2]- posOffset - ChVector3d(0,5,0));
    body_ptrs[3]->SetPos(positions[3]- posOffset + ChVector3d(0,5,0));

    auto Stator_body = body_ptrs[1];
    auto Rotor_body = body_ptrs[2];
    Frame_body = body_ptrs[4];
    
    double radA = 10, radB = 20;
    auto mbody_gearA = makeGears(sys,radA,positions[2]- posOffset);
    auto mbody_gearB = makeGears(sys,radB,positions[5]- posOffset);
    auto mbody_gearC = makeGears(sys,radB,positions[6]- posOffset);
    auto mbody_gearD = makeGears(sys,radB,positions[7]- posOffset);
    auto mbody_gearE = makeGears(sys,radB,positions[8]- posOffset);
    auto mbody_gearF = makeGears(sys,radA,positions[3]- posOffset);

    auto link_motorA = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    link_motorA->Initialize(mbody_gearA, Frame_body, ChFrame<>(positions[2]- posOffset, jointOrientation));
    link_motorA->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(20));
    sys.AddLink(link_motorA);

    createJoint(sys, body_ptrs[9], Frame_body, JointType::REVOLUTE, positions[9]- posOffset);
    createJoint(sys, body_ptrs[9], mbody_gearC, JointType::FIXED, positions[9]- posOffset);
    createJoint(sys, body_ptrs[9], mbody_gearD, JointType::FIXED, positions[9]- posOffset);
    createJoint(sys, body_ptrs[2], mbody_gearA, JointType::FIXED, positions[2]- posOffset);
    createJoint(sys, body_ptrs[5], mbody_gearB, JointType::FIXED, positions[5]- posOffset);
    createJoint(sys, body_ptrs[6], mbody_gearC, JointType::FIXED, positions[6]- posOffset);
    createJoint(sys, body_ptrs[7], mbody_gearD, JointType::FIXED, positions[7]- posOffset);
    createJoint(sys, body_ptrs[8], mbody_gearE, JointType::FIXED, positions[8]- posOffset);
    createJoint(sys, body_ptrs[3], mbody_gearF, JointType::FIXED, positions[3]- posOffset);

    gearMate(sys, mbody_gearA, mbody_gearB, radA, radB);
    gearMate(sys, mbody_gearB, mbody_gearC, radB, radB);
    gearMate(sys, mbody_gearD, mbody_gearE, radB, radB);
    gearMate(sys, mbody_gearE, mbody_gearF, radB, radA);

    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(1200, 800);
    vis->SetWindowTitle("Modeling a simplified trackjed vehicle");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(-300, 400, 300), ChVector3d(0,0,100));
    auto camera = vis->GetActiveCamera();
    vis->AddLight(ChVector3d(0, 0, -900), 1100, ChColor(0.5f, 0.5f, 0.5f));
    vis->AddLight(ChVector3d(0, 0, 900), 1100, ChColor(0.5f, 0.5f, 0.5f));
    vis->AddLight(ChVector3d(-300, -500, 0), 2000, ChColor(0.1f, 0.1f, 0.1f));
    vis->AddLight(ChVector3d(-300, 500, 0), 2000, ChColor(0.1f, 0.1f, 0.1f));
    vis->EnableBodyFrameDrawing(true);
    vis->EnableLinkFrameDrawing(true);
    // camera->setTarget(irr::core::vector3df(1300, 0, 0));
    
    ChVector3d Torque_direction_RotorWinding_Stator(1, 0, 0); // IMPORTANT!! the direction vertex need to be normalized  
    double Torque_magnitude_RotorWinding_Stator = -0.0 * 1e3 * 1e3; //[Nm] converted to ([kg]-[mm]-[s]) 
    ChVector3d RotorWinding_Stator_Torque = Torque_magnitude_RotorWinding_Stator * Torque_direction_RotorWinding_Stator;

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
    // ======== SET THE MULTI-PHYSICS SYMULATION PARAMETERS ====================================================================================================================================
    // ===========================================================================================================================================================================================
    // ======== Mechanical domain ====================================================================================================================================================================
    double f_ToSample_mechanic = 1.0e3;//1.0e5;//8.0e3;// 0.5e4; // [Hz]
    double t_step_mechanic = 1 / f_ToSample_mechanic; // [s]
    // ======== Electronic domain ====================================================================================================================================================================
    double f_ToSample_electronic = 1.0e3;//1.0e5;// 0.5e4; // [Hz]                              Frequency at which the electronic domain is called respect to the global time line
    double T_ToSample_electronic = 1 / f_ToSample_electronic;               // Period at which the electronic domain is called respect to the global time line
    double T_sampling_electronic = t_step_mechanic;                         // Time window of the electronic (SPICE) simulation
    double t_step_electronic = 1.0e-5;//1.0e-6; // [s]                                  Discretization of the electronic time window

    std::string Netlist_location = "../data/my_project/SPICE/Circuit_Netlist.cir";   
    
    ChElectronicGeneric Generic_Circuit(Netlist_location, t_step_electronic); 
    Generic_Circuit.Initialize(t_step_mechanic);

    std::map<std::string, double> PWLIn = {
        {"VmotorVAR", 0.0},
        {"VpwmVAR", 0.0}
    };
    std::map<std::string, double> FlowIn = {
        {"Rmotor", 0.5},
        {"Lmotor", 12.0 * 1.0e-6}
    };

    std::map<std::string, std::vector<double>> OutputMap;
    OutputMap["n1"] = {};
    OutputMap["n3"] = {};
    OutputMap["VmotorVAR"] = {};
    OutputMap["t_electronics"] = {};
    OutputMap["alpha"] = {};
    OutputMap["dalpha"] = {};
    OutputMap["t_mechanics"] = {};
    OutputMap["T_magnetic"] = {};
    OutputMap["T_motor"] = {};

    Generic_Circuit.InputDefinition(PWLIn, FlowIn);

    double t_simulation_STOP = 10.0;//400.0e-3; //[s]
    double t_sim_mechanics = 0.0; //[s] 
    double t_sim_electronics = 0.0; //[s]
    double t_sampling_electronic_counter = 0; //[s] This variable is needed to count the event at which the Electronic domain need to be called respect to the Global Time-line
    int brake_flag = 1; // Set a brake flag in the case you want to stop the simulation before: t_simulation_STOP
    double Imotor = 0.0;
    
    std::cout << "\n";
    std::cout << "===================================================" << "\n";
    std::cout << "======= PRESS ENTER TO START THE SIMULATION =======" << "\n"; 
    std::cout << "===================================================" << "\n";
    std::cout << "\n";
    
    double T_PWM = 0.04; //[s] PWM Period
    double Duty_PWM = 100.0 / 100; //[s] PWM Duty
    double t_PWM_counter = 0.0; //[s] PWM Period

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        if (t_sampling_electronic_counter >= T_ToSample_electronic) {
            // ======== EXECUTE -> the Electronic co-simulation process ====================================================================================================================================================================

            Generic_Circuit.Advance(t_step_mechanic);


            auto res1 = Generic_Circuit.GetResult();
             // ======== COMPUTE -> the Mechanics ====================================================================================================================================================================
            
            ChVector3d Rotor_Euler_Vel = Rotor_body->GetAngVelLocal(); // Get the effective euler angular velocity 
            
            // ======== COMPUTE -> the Multiphysics ====================================================================================================================================================================
            double ke_motor = -0.022446; //[Nm/A]
            double Vbackemf = ke_motor * Rotor_Euler_Vel[0];
            Imotor = res1[toLowerCase("VmotorVAR")].back();

            // ======== UPDATE -> the Electronic parameters ====================================================================================================================================================================
            if (t_sim_mechanics >= 0.0){
                if (t_PWM_counter <= T_PWM * Duty_PWM)
                {
                    PWLIn["VpwmVAR"] = 12.0;
                    t_PWM_counter += t_step_mechanic;
                }
                else
                {
                    PWLIn["VpwmVAR"] = 0.0;
                    t_PWM_counter += t_step_mechanic;
                }
                if (t_PWM_counter > T_PWM)
                {
                    t_PWM_counter = 0.0;
                }

            }
            PWLIn["VmotorVAR"] = Vbackemf;
            Generic_Circuit.InputDefinition(PWLIn, FlowIn);

            // ======== SAVE -> the needed variables ====================================================================================================================================================================
            OutputMap["n1"].push_back(res1["n1"].back());
            OutputMap["n3"].push_back(res1["n3"].back());
            OutputMap["VmotorVAR"].push_back(res1[toLowerCase("VmotorVAR")].back());
            OutputMap["t_electronics"].push_back(t_sim_mechanics);
            OutputMap["dalpha"].push_back(-Rotor_Euler_Vel[0]);

            // ======== UPDATE -> the TIME variables ====================================================================================================================================================================
            t_sampling_electronic_counter = 0;      // The variable is nulled to re-start with the counter for the next call of the electronic domain
        }

        // ======== EXTRACT -> Kinematic variables ====================================================================================================================================================================
        std::vector<double> Rotor_Euler_Ang = GetEulerAngPos(Rotor_body, t_step_mechanic);

        // ======== TORQUE TEMEPLATE ====================================================================================================================================================================
        // ======== UPDATE -> Forces and Torques: RotorWinding - Stator ====================================================================================================================================================================
        double kt_motor = 0.022446; //[Nm/A] 150
        Torque_magnitude_RotorWinding_Stator = kt_motor * Imotor * 1e3 * 1e3; // Conversion to ([kg]-[mm]-[s])    
        RotorWinding_Stator_Torque = -1.0 * Torque_magnitude_RotorWinding_Stator * Torque_direction_RotorWinding_Stator;
        Rotor_body->EmptyAccumulators(); // Clean the body from the previous force/torque IMPORTANT!!!!: Uncomment this line if you never clean the F/T to this body
        Rotor_body->AccumulateTorque(RotorWinding_Stator_Torque, false); // Apply to the body the force
        
        // ======== SAVE -> the needed variables ====================================================================================================================================================================
        OutputMap["alpha"].push_back(-Rotor_Euler_Ang[0]);
        OutputMap["t_mechanics"].push_back(t_sim_mechanics);
        OutputMap["T_motor"].push_back(-1.0 * Torque_magnitude_RotorWinding_Stator);

        // ======== RUN -> the Mechanic solver ====================================================================================================================================
        sys.DoStepDynamics(t_step_mechanic);
        realtime_timer.Spin(t_step_mechanic);

        // ======== UPDATE -> the Multi-physics timeline ====================================================================================================================================
        t_sampling_electronic_counter += t_step_mechanic;
        t_sim_electronics += t_step_mechanic;
        t_sim_mechanics += t_step_mechanic;

        // std::cout << -1.0 * Torque_magnitude_RotorWinding_Stator << "\t";
        // std::cout << -Rotor_Euler_Ang[0] << "\t";
        // std::cout << std::endl;
    }
    // Export the output data in a .json file
    // std::ofstream out_file("output.json");
    // out_file << j.dump(4); // "4" is the indentation parameter, you can change it to have a more or less readable structure
    // out_file.close();
    // std::cout << "Data exported to 'output.json'" << std::endl;

    return 0;
}