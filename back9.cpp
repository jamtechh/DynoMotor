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
    auto gear = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Z, rad, 0.1, 0.05, true, false, mat);
    gear->SetPos(position);
    gear->SetRot(QuatFromAngleZ(CH_PI_2));
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
        RigidBody(ChSystemNSC& sys, const std::string& file_name, bool is_fixed = false, bool transprnt = false)
            : system(sys), obj_file(file_name), is_fixed(is_fixed) {SetupRigidBody(transprnt);}
    
        std::shared_ptr<ChBody> GetBody() const {return body;}
        ChVector3d GetCOG() const {return cog;}
        ChVector3d getPos() const {return body->GetPos();}
        std::tuple<std::shared_ptr<ChBody>, ChVector3d> GetBodyAndCOG() const {return std::make_tuple(body, cog);}
        void HideBody() {mesh->SetVisible(false);}
        void ShowCG() {AddVisualizationBall(system, body->GetPos());}
        void setPos(ChVector3d poss){body->SetPos(poss);}
        void setColor(ChColor colll){mesh->SetColor(colll);}
        void setTransparent(){mesh->SetOpacity(0.5f);}
        
        void setData(const std::tuple<std::string, ChVector3d, ChQuaternion<>, ChVector3d, double, std::string>& objData) { 
            // Get the Values
            position = std::get<1>(objData);
            rotation = std::get<2>(objData);  
            inertia_SW = std::get<3>(objData);  
            mass_SW = std::get<4>(objData); 
            file_name = std::get<5>(objData); 
            // if(prnt)std::cout << mass_SW << "\n\n\n";

            // Set the Values
            body->SetPos(position-posOffset);
            body->SetRot(rotation);
            body->SetMass(mass_SW);
            body->SetInertiaXX(inertia_SW);
            if (prnt) {
                std::cout << std::fixed << std::setprecision(3); // Set precision for floating-point numbers
                std::cout << std::setw(15) << file_name << "  "
                          << std::setw(5) << "m= " << std::setw(8) << mass_calc << "  "
                          << std::setw(5) << "v= " << std::setw(12) << volume << "  "
                          << std::setw(20) << geometric_inertia_calc(0,0) 
                          << std::setw(20) << geometric_inertia_calc(1,1) 
                          << std::setw(20) << geometric_inertia_calc(2,2) 
                          << std::setw(20) << inertia_SW[0] 
                          << std::setw(20) << inertia_SW[1] 
                          << std::setw(20) << inertia_SW[2] 
                          << std::endl;
            }
        }
    private:
        bool prnt=1;
        ChSystemNSC& system;
        std::string obj_file;
        std::string file_name;
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
        double volume;
        ChMatrix33<> inertia_calc;
        ChMatrix33<> geometric_inertia_calc;

        double mass_SW;
        ChVector3d inertia_SW;

    
        void SetupRigidBody(bool trans) {
            // Load visualization mesh
            obj_file = std::string("my_project/CAD/DynoObj3_shapes/") + obj_file + std::string(".obj");
            auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(obj_file));
            std::string coll_file = obj_file;
            auto coll_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(coll_file));
            // std::cout<<"\t\t\t2"<<std::endl;
    
            trimesh->ComputeMassProperties(true, volume, cog, geometric_inertia_calc);
            // std::cout<<"\t\t\t3"<<std::endl;
    
            // Calculate mass_calc and inertia_calc
            density = 8970.00 / (1e9);
            mass_calc = density * volume;
            inertia_calc = density * geometric_inertia_calc;
    
            // Create rigid body
            body = chrono_types::make_shared<ChBody>();
            body->SetFixed(is_fixed);
            // body->SetMass(mass_calc);
            // body->SetInertiaXX(ChVector3d(inertia_calc(0, 0), inertia_calc(1, 1), inertia_calc(2, 2)));
            
            system.Add(body);
    
            // Visualization
            mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            mesh->SetMesh(trimesh);
            mesh->SetMutable(false);
            if(trans)mesh->SetOpacity(0.5f);
            mesh->SetBackfaceCull(true);
            body->AddVisualShape(mesh, ChFrame<>(ChVector3d(0,0,0), ChMatrix33<>(1)));
        }
    };


int main(int argc, char* argv[]) {
    // std::cout<<"\n\n\nhelloooooasdfadsf\n\n"<<posst[2]<<"\n\n\n";
    ChSystemNSC sys = GravetySetup();

    ChQuaternion<> SpringDamper_Orientation;
    SpringDamper_Orientation.SetFromAngleAxis(90.0 * M_PI / 180.0, ChVector3d(0, 0, 1)); // !!! IMPORTANT !!! the Torsional Spring is oriented always arround Z-axis -> Set correctly the orientation 
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
    // body_ptrs[2]->SetPos(std::get<1>(objData[2]) - posOffset);
    // body_ptrs[3]->SetPos(std::get<1>(objData[3]) - posOffset);

    auto StatorBody = body_ptrs[1];
    auto RotorBody = body_ptrs[2];
    Frame_body = body_ptrs[4];
    
    double radA = 10, radB = 20;
    // auto mbody_gearA = makeGears(sys,radA,std::get<1>(objData[2]) - posOffset);
    // auto mbody_gearB = makeGears(sys,radB,std::get<1>(objData[5]) - posOffset);
    // auto mbody_gearC = makeGears(sys,radB,std::get<1>(objData[6]) - posOffset);
    // auto mbody_gearD = makeGears(sys,radB,std::get<1>(objData[7]) - posOffset);
    // auto mbody_gearE = makeGears(sys,radB,std::get<1>(objData[8]) - posOffset);
    // auto mbody_gearF = makeGears(sys,radA,std::get<1>(objData[3]) - posOffset);
    // auto link_motorA = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    // link_motorA->Initialize(mbody_gearA, Frame_body, ChFrame<>(std::get<1>(objData[2]) - posOffset, jointOrientation));
    // link_motorA->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(80));
    // sys.AddLink(link_motorA);

    createJoint(sys, body_ptrs[9], Frame_body, JointType::REVOLUTE, std::get<1>(objData[9]) - posOffset);
    createJoint(sys, body_ptrs[9], body_ptrs[6], JointType::FIXED, std::get<1>(objData[9]) - posOffset);
    createJoint(sys, body_ptrs[9], body_ptrs[7], JointType::FIXED, std::get<1>(objData[9]) - posOffset);
    createJoint(sys, RotorBody, Frame_body, JointType::REVOLUTE, std::get<1>(objData[2]) - posOffset);
    createJoint(sys, RotorBody, Frame_body, JointType::REVOLUTE, std::get<1>(objData[2]) - posOffset, true);
    createJoint(sys, body_ptrs[5], Frame_body, JointType::REVOLUTE, std::get<1>(objData[5]) - posOffset);
    createJoint(sys, body_ptrs[6], Frame_body, JointType::REVOLUTE, std::get<1>(objData[6]) - posOffset);
    createJoint(sys, body_ptrs[7], Frame_body, JointType::REVOLUTE, std::get<1>(objData[7]) - posOffset);
    createJoint(sys, body_ptrs[8], Frame_body, JointType::REVOLUTE, std::get<1>(objData[8]) - posOffset);
    createJoint(sys, body_ptrs[3], Frame_body, JointType::REVOLUTE, std::get<1>(objData[3]) - posOffset);

    gearMate(sys, RotorBody, body_ptrs[5], radA, radB);
    gearMate(sys, body_ptrs[5], body_ptrs[6], radB, radB);
    gearMate(sys, body_ptrs[7], body_ptrs[8], radB, radB);
    gearMate(sys, body_ptrs[8], body_ptrs[3], radB, radA);
    
    // ===========================================================================================================================================================================================
    // ======== F / T DEFINITION -> TORSIONAL SPRING/DAMPER: RotorWinding - Stator ====================================================================================================================================
    // ===========================================================================================================================================================================================
    // ======== Torsional spring coefficient ===========================================================================================================================================================================
    double springConst = 0.0; // [(N * m) / rad]
    springConst = springConst * 1e3 * 1e3; // Conversion to ([kg]-[mm]-[s]) 
    // ======== Torsional damping coefficient ===========================================================================================================================================================================
    double dampConst = 0.0003; //[(N*m*s)/rad]
    double r_eq_RotorWinding_Stator_spr = dampConst * 1e3 * 1e3; // Conversion to ([kg]-[mm]-[s])  
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
    springDamper->SetDampingCoefficient(r_eq_RotorWinding_Stator_spr); // [(kg mm mm s)/(s^2 mm rad)] that should be the SI conversion ([kg]-[mm]-[s]) of [N m s/rad]
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
    double t_simulation_STOP = 1.0; //[s]
    double f_ToSample_mechanic = 1.0e4;//1.0e5;//8.0e3;// 0.5e4; // [Hz]
    double t_step_mechanic = 1 / f_ToSample_mechanic; // [s]
    // ======== Electronic domain ====================================================================================================================================================================
    double f_ToSample_electronic = f_ToSample_mechanic;//1.0e5;// 0.5e4; // [Hz]          Frequency at which the electronic domain is called respect to the global time line
    double T_ToSample_electronic = 1 / f_ToSample_electronic;               // Period at which the electronic domain is called respect to the global time line
    double t_step_electronic = 1.0e-5;//1.0e-6; // [s]                                  Discretization of the electronic time window
    // ===========================================================================================================================================================================================
    double t_sim_mechanics = 0.0; //[s] 
    double t_sim_electronics = 0.0; //[s]
    double t_sampling_electronic_counter = 0; //[s] This variable is needed to count the event at which the Electronic domain need to be called respect to the Global Time-line

    double Imotor = 0.0;
    double T_PWM = 0.001; //[s] PWM Period
    double Duty_PWM = 45.0 / 100; //[s] PWM Duty
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
    std::map<std::string, double> FlowIn = {
        {"Rmotor", 0.4},            // checked
        {"Lmotor", 1.0e-5}          // checked
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

    std::string Netlist_location = "../data/my_project/SPICE/Circuit_Netlist.cir";   
    ChElectronicGeneric Generic_Circuit(Netlist_location, t_step_electronic); 
    Generic_Circuit.Initialize(t_step_mechanic);
    Generic_Circuit.InputDefinition(PWLIn, FlowIn);

    high_resolution_clock::time_point start = high_resolution_clock::now();

    while (t_sim_mechanics < t_simulation_STOP && vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        ChVector3d Rotor_Euler_Vel;
        if (t_sampling_electronic_counter >= T_ToSample_electronic) {
            Generic_Circuit.Advance(t_step_mechanic);
            auto res1 = Generic_Circuit.GetResult();
            Rotor_Euler_Vel = RotorBody->GetAngVelLocal(); // Get the effective euler angular velocity 
            
            double kv_motor = 1600;       // rpm/v
            double ke_motor = 1/kv_motor; //[V/rpm]
            double Vbackemf = ke_motor * Rotor_Euler_Vel[AngVelAxis];
            Imotor = res1[toLowerCase("VmotorVAR")].back();
            
            // if(1){
                // for (const auto& [key, values] : res1) {
                //     std::cout << key << "= ";std::cout << res1[key].back()<<"   \t";}
                // std::cout << t_sim_mechanics << "\n";}
            if(1){
                std::cout << "Current = " << res1["vmotorvar"].back()<<" A\t";
                std::cout << "t_sim = " << t_sim_mechanics * 16.620 <<"\t";
            }
            double dcV = 7.5; // Volt
            if (t_sim_mechanics >= 0.0){
                if (t_PWM_counter <= T_PWM * Duty_PWM){
                        PWLIn["VpwmVAR"] = dcV;     t_PWM_counter += t_step_mechanic;}
                else{   PWLIn["VpwmVAR"] = 0.0;     t_PWM_counter += t_step_mechanic;}
                if (t_PWM_counter > T_PWM){         t_PWM_counter = 0.0;}}

            PWLIn["VmotorVAR"] = -Vbackemf;
            Generic_Circuit.InputDefinition(PWLIn, FlowIn);

            OutputMap["n1"].push_back(res1["n1"].back());
            // OutputMap["n3"].push_back(dcV-res1["n3"].back());
            OutputMap["n3"].push_back(Vbackemf);
            OutputMap["VmotorVAR"].push_back(-res1[toLowerCase("VmotorVAR")].back());
            OutputMap["dalpha"].push_back(Rotor_Euler_Vel[AngVelAxis]/2.08);

            // ======== UPDATE -> the TIME variables ====================================================================================================================================================================
            t_sampling_electronic_counter = 0;      // The variable is nulled to re-start with the counter for the next call of the electronic domain
        }

        // ======== EXTRACT -> Kinematic variables ====================================================================================================================================================================
        std::vector<double> Rotor_Euler_Ang = GetEulerAngPos(RotorBody, t_step_mechanic);

        // We apply the constant torque here !!!
        double kt_motor = 10.6; //[Nm/A]
        motorTorque = kt_motor * Imotor * 1e3 * 1e3; // Conversion to ([kg]-[mm]^2/[s^2])    
        rotorTorque = -1.0 * motorTorque * TorqueDir;
        RotorBody->EmptyAccumulators(); // Clean the body from the previous force/torque IMPORTANT!!!!: Uncomment this line if you never clean the F/T to this body
        RotorBody->AccumulateTorque(rotorTorque, true); // Apply to the body the force
        
        double sinWave = 0.75 * sin(40 * t_sim_mechanics);
        double loadTorque = 4e6 * 1;
        dynoTorque = 1.0 * loadTorque * TorqueDir;
        body_ptrs[3]->EmptyAccumulators(); // Clean the body from the previous force/torque IMPORTANT!!!!: Uncomment this line if you never clean the F/T to this body
        body_ptrs[3]->AccumulateTorque(dynoTorque, true); // Apply to the body the force
        

        // ======== SAVE -> the needed variables ====================================================================================================================================================================
        OutputMap["alpha"].push_back(Rotor_Euler_Ang[0]);
        OutputMap["t_mechanics"].push_back(t_sim_mechanics);
        OutputMap["T_motor"].push_back(-1.0 * motorTorque);
        OutputMap["t_electronics"].push_back(loadTorque);

        // ======== RUN -> the Mechanic solver ====================================================================================================================================
        sys.DoStepDynamics(t_step_mechanic);
        realtime_timer.Spin(t_step_mechanic);

        // ======== UPDATE -> the Multi-physics timeline ====================================================================================================================================
        t_sampling_electronic_counter += t_step_mechanic;
        t_sim_electronics += t_step_mechanic;
        t_sim_mechanics += t_step_mechanic;

        high_resolution_clock::time_point end = high_resolution_clock::now();
        duration<double, std::milli> duration_sec = std::chrono::duration_cast<duration<double, std::milli>>(end - start);
        std::cout << "time: " << duration_sec.count() / 1000 << " s \t";
        std::cout << "RPM: " <<  Rotor_Euler_Vel[AngVelAxis]/2.08 << "\t";
        // std::cout << "loadTorque: " << loadTorque/1e6 << "Nm \t";
        std::cout << std::endl;
    }
    json j; // Create a json object to contain the output data
    for (const auto& item : OutputMap) { // Populate the JSON object with data
        j[item.first] = item.second;
    }
    // Export the output data in a .json file
    std::ofstream out_file("output2.json");
    out_file << j.dump(4); // "4" is the indentation parameter, you can change it to have a more or less readable structure
    out_file.close();
    std::cout << "Data exported" << std::endl;
    return 0;
}