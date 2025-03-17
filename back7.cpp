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

// ======== Method: allows to disp into prompt the content of a std::vector<std::string>  ==============================================================================================================================================
void prompt_vector_string(std::vector<std::string>& str) {
    for (const auto& line : str) {
        std::cout << line << std::endl;}
}

// ======== Method: allows to disp into prompt the content of a std::map<std::string, std::vector<double>> ==============================================================================================================================================
void prompt_map_string_vector_double(std::map<std::string, std::vector<double>>& map, int print_values) {
    std::cout << "Map content:\n" << std::endl;
    for (const auto& [key, values] : map) {  
        std::cout << key << ": ";
        if (print_values==1) {
            for (double value : values) {
            std::cout << value << " ";
            }
        }
        std::cout << std::endl;}
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
        RigidBody(ChSystemNSC& sys, const std::string& file_name, bool is_fixed = false, bool transparent = false)
            : system(sys), obj_file(file_name), is_fixed(is_fixed) {SetupRigidBody(transparent);}
    
        std::shared_ptr<ChBody> GetBody() const {return body;}
        ChVector3d GetCOG() const {return cog;}
        ChVector3d getPos() const {return body->GetPos();}
        std::tuple<std::shared_ptr<ChBody>, ChVector3d> GetBodyAndCOG() const {return std::make_tuple(body, cog);}
        void HideBody() {mesh->SetVisible(false);}
        void ShowCG() {AddVisualizationBall(system, body->GetPos());}
        void setPos(ChVector3d poss){body->SetPos(poss);}
        void setColor(ChColor colll){mesh->SetColor(colll);}
        void RotateBody(double angle, ChVector3d axis) {
            ChQuaternion<> rot_q;
            rot_q.SetFromAngleAxis(angle, axis);
            body->SetRot(rot_q * body->GetRot()); // Apply rotation
        }
        
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
            obj_file = std::string("my_project/CAD/View/") + obj_file + std::string(".obj");
            auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(obj_file));
            std::string coll_file = obj_file;
            auto coll_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(coll_file));
            // std::cout<<"\t\t\t2"<<std::endl;
    
            trimesh->ComputeMassProperties(true, volume, cog, geometric_inertia_calc);
            // std::cout<<"\t\t\t3"<<std::endl;
    
            // Calculate mass_calc and inertia_calc
            density = 8900.00 / (1e9);
            mass_calc = density * volume;
            inertia_calc = density * geometric_inertia_calc;
    
            // Create rigid body
            body = chrono_types::make_shared<ChBody>();
            body->SetFixed(is_fixed);
            body->SetMass(mass_calc);
            // body->SetPos(cog);
            body->SetInertiaXX(ChVector3d(inertia_calc(0, 0), inertia_calc(1, 1), inertia_calc(2, 2)));
            
            system.Add(body);
    
            // Visualization
            mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            mesh->SetMesh(trimesh);
            mesh->SetMutable(false);
            if(trans)mesh->SetOpacity(0.5f);
            mesh->SetBackfaceCull(true);
            body->AddVisualShape(mesh, ChFrame<>(-cog, ChMatrix33<>(1)));
        }
    };

// ===============================================================================================================================================================================
// ======== MAIN LOOP ==============================================================================================================================================
// ===============================================================================================================================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    ChSystemNSC sys; // Create a Chrono physical system
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    // NSC = Non Smooth Contact that is suitable for impact phenomena
    // SMC = SMooth Contacy that i suitable for continuous phenomena
    // Mechanical Unit system: [kg]-[mm]-[s] -> The use of [mm] is require cause extrimelly small collisiion parameters crash the simulation
    // Electric Unit system: [kg]-[m]-[s]

    ChVector3d gravity_acc = sys.GetGravitationalAcceleration(); 
    std::cout << "The gravity acceleration  vector is: " << gravity_acc << "\n\n";
    double gravity = 9.81e3; //[mm/s^2]
    sys.SetGravitationalAcceleration(ChVector3d(-gravity, -0, 0));
    ChVector3d gravity_acc_new = sys.GetGravitationalAcceleration(); 
    std::cout << "The new gravity acceleration  vector is: " << gravity_acc_new << "\n\n";
  
    RigidBody RotorWinding(sys, "RotorWinding_OBJ");
    auto RotorWinding_body = RotorWinding.GetBody();

    RigidBody Stator(sys, "Stator_OBJ", true, true);
    auto Stator_body = Stator.GetBody();

    ChQuaternion<> RotorWinding_Stator_Torsional_Spring_Orientation;
    RotorWinding_Stator_Torsional_Spring_Orientation.SetFromAngleAxis(90.0 * M_PI / 180.0, ChVector3d(0, 1, 0)); // !!! IMPORTANT !!! the Torsional Spring is oriented always arround Z-axis -> Set correctly the orientation 
    jointOrientation.SetFromAngleAxis(90.0 * (CH_PI / 180.0), ChVector3d(0, 0, 1));
    ChVector3d Torque_direction_RotorWinding_Stator(1, 0, 0); // IMPORTANT!! the direction vertex need to be normalized 
    

    // RotorWinding.RotateBody(CH_PI_2, ChVector3d(0, 1, 0));
    // Stator.RotateBody(CH_PI_2, ChVector3d(0, 1, 0));
    // RotorWinding_Stator_Torsional_Spring_Orientation.SetFromAngleAxis(90.0 * M_PI / 180.0, ChVector3d(0, 1, 0)); // !!! IMPORTANT !!! the Torsional Spring is oriented always arround Z-axis -> Set correctly the orientation 
    // jointOrientation.SetFromAngleAxis(90.0 * (CH_PI / 180.0), ChVector3d(0, 0, 1));
    // ChVector3d Torque_direction_RotorWinding_Stator(1, 0, 0); 

    // ===========================================================================================================================================================================================
    // ======== RIGID BODY DEFINITION: WaveFront Shape -> RotorWinding ====================================================================================================================================
    // ===========================================================================================================================================================================================
    // ======== File name ========================================================================================================================================================================
    // std::string RotorWinding_file_name = "my_project/CAD/View/RotorWinding_OBJ.obj";
    // std::string RotorWinding_Collision_file_name = "my_project/CAD/Collision/RotorWinding_Collision_OBJ.obj";
    // // ======== MESHES ===========================================================================================================================================================================
    // // ======== Visualization Mesh ===============================================================================================================================================================
    // auto RotorWinding_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(RotorWinding_file_name));
    // auto RotorWinding_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    // RotorWinding_mesh->SetMesh(RotorWinding_trimesh);
    // RotorWinding_mesh->SetVisible(true);
    // // ======== Compute mass inertia from mesh ===================================================================================================================================================
    // double RotorWinding_volume; // [mm^3]  
    // ChVector3d RotorWinding_cog; // [mm]   
    // ChMatrix33<> RotorWinding_geometric_inertia; // [mm^5] it is the geometric inertia tensor (see when you call: RotorWinding_mesh->ComputeMassProperties) 
    // RotorWinding_trimesh->ComputeMassProperties(true, RotorWinding_volume, RotorWinding_cog, RotorWinding_geometric_inertia); // It returns: RotorWinding_volume:[mm^3], RotorWinding_cog:[mm], RotorWinding_inertia:[mm^5] that is the geometric inertia tensor 
    // double RotorWinding_density = 8900.00 / (1e9); // [kg/mm^3]
    // double RotorWinding_mass = RotorWinding_density * RotorWinding_volume; // [kg]
    // ChMatrix33<> RotorWinding_inertia = RotorWinding_density * RotorWinding_geometric_inertia; // [kg*mm^2]
    // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    // std::cout << "!!!!!!! RotorWinding -> Inertia properies: !!!!!!!" << "\n"; // Display the Inertia properties of the body
    // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    // std::cout << "The RotorWinding mass is: " << RotorWinding_mass << " [kg]" << "\n\n";
    // std::cout << "The RotorWinding cog is: " << RotorWinding_cog << " [mm]" << "\n\n";
    // std::cout << "The RotorWinding inertia tensor is:\n" << RotorWinding_inertia << " [kg*mm^2]" << "\n\n";
    // // ======== Define the rigid body ============================================================================================================================================================
    // auto RotorWinding_body = chrono_types::make_shared<ChBody>();
    // sys.Add(RotorWinding_body);
    // RotorWinding_body->SetFixed(false);
    // RotorWinding_body->SetMass(RotorWinding_mass);
    // RotorWinding_body->SetInertiaXX(ChVector3d(RotorWinding_inertia(0, 0), RotorWinding_inertia(1, 1), RotorWinding_inertia(2, 2)));
    // RotorWinding_body->SetPos(RotorWinding_cog);
    // // ======== Visulaization ====================================================================================================================================================================
    // RotorWinding_mesh->SetMutable(false);
    // RotorWinding_mesh->SetColor(ChColor(0.0f, 0.616f, 1.0f));
    // RotorWinding_mesh->SetOpacity(0.5f);
    // RotorWinding_mesh->SetBackfaceCull(true);
    // RotorWinding_body->AddVisualShape(RotorWinding_mesh, ChFrame<>(-RotorWinding_cog, ChMatrix33<>(1)));

    // ===========================================================================================================================================================================================
    // ======== RIGID BODY DEFINITION: WaveFront Shape -> Stator ====================================================================================================================================
    // ===========================================================================================================================================================================================
    // ======== File name ========================================================================================================================================================================
    // std::string Stator_file_name = "my_project/CAD/View/Stator_OBJ.obj";
    // std::string Stator_Collision_file_name = "my_project/CAD/Collision/Stator_Collision_OBJ.obj";
    // // ======== MESHES ===========================================================================================================================================================================
    // // ======== Visualization Mesh ===============================================================================================================================================================
    // auto Stator_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Stator_file_name));
    // auto Stator_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    // Stator_mesh->SetMesh(Stator_trimesh);
    // Stator_mesh->SetVisible(true);
    // // ======== Compute mass inertia from mesh ===================================================================================================================================================
    // double Stator_volume; // [mm^3]  
    // ChVector3d Stator_cog; // [mm]   
    // ChMatrix33<> Stator_geometric_inertia; // [mm^5] it is the geometric inertia tensor (see when you call: Stator_mesh->ComputeMassProperties) 
    // Stator_trimesh->ComputeMassProperties(true, Stator_volume, Stator_cog, Stator_geometric_inertia); // It returns: Stator_volume:[mm^3], Stator_cog:[mm], Stator_inertia:[mm^5] that is the geometric inertia tensor 
    // double Stator_density = 7850.00 / (1e9); // [kg/mm^3]
    // double Stator_mass = Stator_density * Stator_volume; // [kg]
    // ChMatrix33<> Stator_inertia = Stator_density * Stator_geometric_inertia; // [kg*mm^2]
    // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    // std::cout << "!!!!!!! Stator -> Inertia properies: !!!!!!!" << "\n"; // Display the Inertia properties of the body
    // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    // std::cout << "The Stator mass is: " << Stator_mass << " [kg]" << "\n\n";
    // std::cout << "The Stator cog is: " << Stator_cog << " [mm]" << "\n\n";
    // std::cout << "The Stator inertia tensor is:\n" << Stator_inertia << " [kg*mm^2]" << "\n\n";
    // // ======== Define the rigid body ============================================================================================================================================================
    // auto Stator_body = chrono_types::make_shared<ChBody>();
    // sys.Add(Stator_body);
    // Stator_body->SetFixed(true);
    // Stator_body->SetMass(Stator_mass);
    // Stator_body->SetInertiaXX(ChVector3d(Stator_inertia(0, 0), Stator_inertia(1, 1), Stator_inertia(2, 2)));
    // Stator_body->SetPos(Stator_cog);
    // // ======== Visulaization ====================================================================================================================================================================
    // Stator_mesh->SetMutable(false);
    // Stator_mesh->SetColor(ChColor(1.0f, 0.0f, 0.0f));
    // Stator_mesh->SetOpacity(0.5f);
    // Stator_mesh->SetBackfaceCull(true);
    // Stator_body->AddVisualShape(Stator_mesh, ChFrame<>(-Stator_cog, ChMatrix33<>(1)));
    // Stator_body->AddVisualShape(Stator_coll_mesh, ChFrame<>(-Stator_cog, ChMatrix33<>(1)));

    // ===========================================================================================================================================================================================
    // ======== LINK DEFINITION -> REVOLUTE JOINT: RotorWinding - Stator ====================================================================================================================================
    // ===========================================================================================================================================================================================
    ChVector3d RotorWinding_Stator_Link_Position(RotorWinding_body->GetPos());            // [mm] set the position in the 3D space of the link respect to the absolute frame
    //RotorWinding_Stator_Link_Position[2] = RotorWinding_Stator_Link_Position[2] + 7.0;
    ChQuaternion<> RotorWinding_Stator_Link_Orientation;
    RotorWinding_Stator_Link_Orientation.SetFromAngleAxis(90.0 * (M_PI / 180.0), ChVector3d(0, 1, 0));       // !!! IMPORTANT !!! the Revolute is always arround Z-axis -> Set correctly the orientation 
    ChFrame<> RotorWinding_Stator_Link_Frame(RotorWinding_Stator_Link_Position, RotorWinding_Stator_Link_Orientation);
    auto RotorWinding_Stator_Link_Revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    RotorWinding_Stator_Link_Revolute->Initialize(RotorWinding_body,                      // Body 1  
        Stator_body,                     // Body 2  
        RotorWinding_Stator_Link_Frame);        // Location and orientation of the frame  
    sys.AddLink(RotorWinding_Stator_Link_Revolute);

    // ===========================================================================================================================================================================================
    // ======== DYNAMIC FORCES AND TORQUES CRATION ================================================================================================================================================
    // ===========================================================================================================================================================================================

    // ===========================================================================================================================================================================================
    // ======== F / T DEFINITION -> UNIVERSAL FORCE: RotorWinding - Stator ====================================================================================================================================
    // ===========================================================================================================================================================================================
    // ======== TORQUE TEMEPLATE ===========================================================================================================================================================================
    double Torque_magnitude_RotorWinding_Stator = -0.0 * 1e3 * 1e3; //[Nm] converted to ([kg]-[mm]-[s]) 
    ChVector3d RotorWinding_Stator_Torque = Torque_magnitude_RotorWinding_Stator * Torque_direction_RotorWinding_Stator;
 
    
    // ===========================================================================================================================================================================================
    // ======== F / T DEFINITION -> TORSIONAL SPRING/DAMPER: RotorWinding - Stator ====================================================================================================================================
    // ===========================================================================================================================================================================================
    // ======== Torsional spring coefficient ===========================================================================================================================================================================
    double k_eq_RotorWinding_Stator_spr = 0.0; // [(N * m) / rad]
    k_eq_RotorWinding_Stator_spr = k_eq_RotorWinding_Stator_spr * 1e3 * 1e3; // Conversion to ([kg]-[mm]-[s]) 
    // ======== Torsional damping coefficient ===========================================================================================================================================================================
    double r_ShaftBushing_experimental = 0.0003; //[(N*m*s)/rad]
    double r_eq_RotorWinding_Stator_spr = r_ShaftBushing_experimental * 1e3 * 1e3; // Conversion to ([kg]-[mm]-[s])  
    // ======== Torsional spring/damper implementation ===========================================================================================================================================================================
    auto RotorWinding_Stator_Torsional_Spring = chrono_types::make_shared<ChLinkRSDA>();
    ChVector3d RotorWinding_Stator_Torsional_Spring_Position(Stator_body->GetPos());  //[mm] set the position in the 3D space of the link respect to the absolute frame
    //RotorWinding_Stator_Torsional_Spring_Position[2] += 6.0;  //[mm] Rise the position of the spring along y-axis in order to see it better in the animation
    ChFrame<> RotorWinding_Stator_Torsional_Spring_Frame(RotorWinding_Stator_Torsional_Spring_Position, RotorWinding_Stator_Torsional_Spring_Orientation);
    RotorWinding_Stator_Torsional_Spring->Initialize(RotorWinding_body,                                   // Body 1  
        Stator_body,                                  // Body 2 
        false,                                        // the two following frames are in absolute, not relative, coords.
        RotorWinding_Stator_Torsional_Spring_Frame,          // Location and orientation of the Body 1 frame 
        RotorWinding_Stator_Torsional_Spring_Frame);         // Location and orientation of the Body 1 frame
    RotorWinding_Stator_Torsional_Spring->SetRestAngle(0.0 * (M_PI / 180.0)); //[rad] Starting angular position
    RotorWinding_Stator_Torsional_Spring->SetSpringCoefficient(k_eq_RotorWinding_Stator_spr); // [(kg mm mm)/(s^2 rad)] that should be the SI conversion ([kg]-[mm]-[s]) of [N m/rad]
    RotorWinding_Stator_Torsional_Spring->SetDampingCoefficient(r_eq_RotorWinding_Stator_spr); // [(kg mm mm s)/(s^2 mm rad)] that should be the SI conversion ([kg]-[mm]-[s]) of [N m s/rad]
    sys.AddLink(RotorWinding_Stator_Torsional_Spring);
    RotorWinding_Stator_Torsional_Spring->AddVisualShape(chrono_types::make_shared<ChVisualShapeRotSpring>(60, 50)); // var1 = radius of the spring, var2 = graphical resolution of the spring
    // ======== Torsional spring/damper visualization ===========================================================================================================================================================================
    auto RotorWinding_Stator_Spring_Visual = chrono_types::make_shared<ChVisualShapeRotSpring>(2.5, 70); // var1 = radius of the spring, var2 = graphical resolution of the spring
    RotorWinding_Stator_Spring_Visual->SetColor(ChColor(0.0f, 1.0f, 0.0f));  // RGB values
    RotorWinding_Stator_Torsional_Spring->AddVisualShape(RotorWinding_Stator_Spring_Visual); 

    // ===========================================================================================================================================================================================
    // ======== MULTI-PHYSICS SIMULATION ===========================================================================================================================================================
    // ===========================================================================================================================================================================================
    
    // ===========================================================================================================================================================================================
    // ======== IRRLICHT VISUALIZATION SYSTEM ====================================================================================================================================================================
    // ===========================================================================================================================================================================================
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(1200, 800);
    vis->SetWindowTitle("Modeling a simplified trackjed vehicle");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(-300, -150, -300));
    vis->AddLight(ChVector3d(300.f, 300.f, -300.f), 3000, ChColor(0.1f, 0.1f, 0.1f));
    vis->AddLight(ChVector3d(300.f, 300.f, 300.f), 3000, ChColor(0.1f, 0.1f, 0.1f));
    vis->EnableBodyFrameDrawing(true);
    vis->EnableLinkFrameDrawing(true);

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

    // ===========================================================================================================================================================================================
    // ======== MULTI-PHYSICS CO-SYMULATION LOOP ====================================================================================================================================================================
    // ===========================================================================================================================================================================================
    // ======== SET -> the Multi-physics timeline ====================================================================================================================================================================
    double t_simulation_STOP = 2.0;//400.0e-3; //[s]
    double t_sim_mechanics = 0.0; //[s] 
    double t_sim_electronics = 0.0; //[s]
    double t_sampling_electronic_counter = 0; //[s] This variable is needed to count the event at which the Electronic domain need to be called respect to the Global Time-line
    double Imotor = 0.0;
    double T_PWM = 0.004; //[s] PWM Period
    double Duty_PWM = 0.0 / 100; //[s] PWM Duty
    double t_PWM_counter = 0.0; //[s] PWM Period
    // ===========================================================================================================================================================================================
    // ======== INITIALIZE THE ELECTRONIC CIRCUIT ====================================================================================================================================================================
    // ===========================================================================================================================================================================================
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

    Generic_Circuit.InputDefinition(PWLIn, FlowIn);

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

    while (t_sim_mechanics < t_simulation_STOP && vis->Run()) {
        // ======== RUN -> the Irrlicht visualizer ====================================================================================================================================================================
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        if (t_sampling_electronic_counter >= T_ToSample_electronic) {
            // ======== EXECUTE -> the Electronic co-simulation process ====================================================================================================================================================================

            Generic_Circuit.Advance(t_step_mechanic);


            auto res1 = Generic_Circuit.GetResult();
             // ======== COMPUTE -> the Mechanics ====================================================================================================================================================================
            
            ChVector3d Rotor_Euler_Vel = RotorWinding_body->GetAngVelLocal(); // Get the effective euler angular velocity 
            
            // ======== COMPUTE -> the Multiphysics ====================================================================================================================================================================
            double ke_motor = 0.01; //[V/rpm]
            double Vbackemf = ke_motor * Rotor_Euler_Vel[0];
            Imotor = res1[toLowerCase("VmotorVAR")].back();
            if(1){
                for (const auto& [key, values] : res1) {
                    std::cout << key << "= ";
                    std::cout << res1[key].back()<<"   \t";}
                std::cout << "\n";}
            // ======== UPDATE -> the Electronic parameters ====================================================================================================================================================================
            if (t_sim_mechanics >= 0.0){
                if (t_PWM_counter <= T_PWM * Duty_PWM){
                    PWLIn["VpwmVAR"] = 12.0;
                    t_PWM_counter += t_step_mechanic;}
                else{
                    PWLIn["VpwmVAR"] = 0.0;
                    t_PWM_counter += t_step_mechanic;}
                if (t_PWM_counter > T_PWM){t_PWM_counter = 0.0;}

            }
            PWLIn["VmotorVAR"] = -Vbackemf;
            Generic_Circuit.InputDefinition(PWLIn, FlowIn);

            // ======== SAVE -> the needed variables ====================================================================================================================================================================
            OutputMap["n1"].push_back(res1["n1"].back());
            OutputMap["n3"].push_back(-res1["n3"].back());
            OutputMap["VmotorVAR"].push_back(res1[toLowerCase("VmotorVAR")].back());
            OutputMap["t_electronics"].push_back(t_sim_mechanics);
            OutputMap["dalpha"].push_back(Rotor_Euler_Vel[0]);

            // ======== UPDATE -> the TIME variables ====================================================================================================================================================================
            t_sampling_electronic_counter = 0;      // The variable is nulled to re-start with the counter for the next call of the electronic domain
        }

        // ======== EXTRACT -> Kinematic variables ====================================================================================================================================================================
        std::vector<double> Rotor_Euler_Ang = GetEulerAngPos(RotorWinding_body, t_step_mechanic);

        // ======== TORQUE TEMEPLATE ====================================================================================================================================================================
        // ======== UPDATE -> Forces and Torques: RotorWinding - Stator ====================================================================================================================================================================
        double kt_motor = 10.0; //[Nm/A] 150
        Torque_magnitude_RotorWinding_Stator = kt_motor * Imotor * 1e3 * 1e3; // Conversion to ([kg]-[mm]-[s])    
        RotorWinding_Stator_Torque = -1.0 * Torque_magnitude_RotorWinding_Stator * Torque_direction_RotorWinding_Stator;
        RotorWinding_body->EmptyAccumulators(); // Clean the body from the previous force/torque IMPORTANT!!!!: Uncomment this line if you never clean the F/T to this body
        RotorWinding_body->AccumulateTorque(RotorWinding_Stator_Torque, false); // Apply to the body the force
        
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
    }

    // ===========================================================================================================================================================================================
    // ======== EXPORT THE RESULTS INTO A JSON FILE ====================================================================================================================================
    // ===========================================================================================================================================================================================
    json j; // Create a json object to contain the output data
    for (const auto& item : OutputMap) { // Populate the JSON object with data
        j[item.first] = item.second;
    }
    // Export the output data in a .json file
    std::ofstream out_file("output2.json");
    out_file << j.dump(4); // "4" is the indentation parameter, you can change it to have a more or less readable structure
    out_file.close();
    std::cout << "Data exported to 'output.json'" << std::endl;

    // ===========================================================================================================================================================================================
    // ======== CLOSE THE MULTI-PHYSICS CO-SIMULATION LOOP ====================================================================================================================================
    // ===========================================================================================================================================================================================
    // system("pause>0"); // Pause the execution of the code to see the results onto the cmd terminal

    return 0;
    //;
}
