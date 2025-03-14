// ===============================================================================================================================================================================
// ======== HEADERS ==============================================================================================================================================
// ===============================================================================================================================================================================
///   t his is changeeddddd in test 1111111
// ======== Multi-purposes headers ==============================================================================================================================================
#if defined(_DEBUG)
#undef _DEBUG
#include <Python.h>
#define _DEBUG
#else

#endif

#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <unordered_set>
#include <tuple>
#include <filesystem>
#include <exception>
#define _USE_MATH_DEFINES 
#include <cmath>

#include <future>
#include <sstream>

#include <nlohmann/json.hpp>
#include <thread>
#include <fstream>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/geometry/ChTriangleMeshSoup.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChRandom.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_irrlicht/ChIrrMeshTools.h"
#include <chrono/physics/ChSystem.h>
// ======== ChElectronics headers ==============================================================================================================================================
#include "chrono_powerelectronics/ChElectronicsCosimulation.h"
#include "chrono_powerelectronics/circuits/ChElectronicMotor.h"
#include "chrono_powerelectronics/circuits/ChElectronicCircuit.h"
#include "chrono_powerelectronics/circuits/ChElectronicGeneric.h"

// ===============================================================================================================================================================================
// ======== NAMESPACES ==============================================================================================================================================
// ===============================================================================================================================================================================
using namespace chrono;
using namespace chrono::irrlicht;
using namespace ::chrono::powerelectronics;
using json = nlohmann::json;

using namespace irr; // Use the main namespaces of Irrlicht
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

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
    // set a ball on the center of mass of the frame for visualization purposes        
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
void AddCylAxis(ChSystemNSC& sys, const ChVector3d& position, ChQuaternion<> jointOrientation, float rad = 2.5, float len = 100, const ChColor& color = ChColor(0.0f, 0.0f, 1.0f)){
    auto axisShape = chrono_types::make_shared<ChVisualShapeCylinder>(rad, len); // Radius = 2, Length = 50
    axisShape->SetColor(color);
    auto axisBody = chrono_types::make_shared<ChBody>();
    axisBody->SetPos(position);
    axisBody->SetFixed(true); // The axis is just for visualization
    axisBody->AddVisualShape(axisShape, ChFrame<>(ChVector3d(0, 0, 0), jointOrientation));
    sys.Add(axisBody);
}
void CreateJoint(std::shared_ptr<ChBody> bodyA, std::shared_ptr<ChBody> bodyB, ChSystemNSC& sys, JointType jointType, bool showAxis = false) {
    ChVector3d jointPosition(bodyA->GetPos());

    ChQuaternion<> jointOrientation;
    if (jointType == JointType::PRISMATIC)jointOrientation.SetFromAngleAxis(90.0 * (CH_PI / 180.0), ChVector3d(0, 0, 1));
    else jointOrientation.SetFromAngleX(0);
    
    ChFrame<> jointFrame(jointPosition, jointOrientation);
    std::shared_ptr<ChLinkLock> Joint;
    switch (jointType) {
        case JointType::FIXED:      {Joint = chrono_types::make_shared<ChLinkLockLock>();      break;}
        case JointType::REVOLUTE:   {Joint = chrono_types::make_shared<ChLinkLockRevolute>();  break;}
        case JointType::PRISMATIC:  {Joint = chrono_types::make_shared<ChLinkLockPrismatic>(); break;}
        default:    throw std::invalid_argument("Invalid joint type.");
    }

    Joint->Initialize(bodyA, bodyB, jointFrame);
    sys.AddLink(Joint);

    if(showAxis){
        auto axisShape = chrono_types::make_shared<ChVisualShapeCylinder>(2.5, 100); // Radius = 2, Length = 50
        axisShape->SetColor(ChColor(1, 0, 0)); // Green color for rotation axis

        auto axisBody = chrono_types::make_shared<ChBody>();
        axisBody->SetPos(jointPosition);
        axisBody->SetFixed(true); // The axis is just for visualization
        axisBody->AddVisualShape(axisShape, ChFrame<>(ChVector3d(0, 0, 0), jointOrientation));
        sys.Add(axisBody);
    } 
}

class RigidBody {
    public:
        RigidBody(ChSystemNSC& sys, const std::string& file_name, double density, bool is_fixed = false)
            : system(sys), obj_file(file_name), density(density), is_fixed(is_fixed) {SetupRigidBody();}
    
        std::shared_ptr<ChBody> GetBody() const {return body;}
        ChVector3d GetCOG() const {return cog;}
        ChVector3d getPos() const {return body->GetPos();}
        std::tuple<std::shared_ptr<ChBody>, ChVector3d> GetBodyAndCOG() const {return std::make_tuple(body, cog);}
        void HideBody() {mesh->SetVisible(false);}
        void ShowCG() {AddVisualizationBall(system, body->GetPos());}
        void setPos(ChVector3d poss){body->SetPos(poss);}
        void setColor(ChColor colll){mesh->SetColor(colll);}
    
    private:
        ChSystemNSC& system;
        std::string obj_file;
        double density;
        bool is_fixed;
        std::shared_ptr<ChBody> body;
        std::shared_ptr<ChVisualShapeTriangleMesh> mesh;
        std::shared_ptr<ChVisualShapeTriangleMesh> coll_mesh;
        std::shared_ptr<ChCollisionModel> coll_model;
        ChVector3d cog;
    
        void SetupRigidBody() {
            // Load visualization mesh
            obj_file = std::string("my_project/CAD/View/") + obj_file + std::string(".obj");
            auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(obj_file));
    
            std::cout<<"\t\t\t1"<<std::endl;
            // Load collision mesh
            std::cout<<"\t\t"<<obj_file<<std::endl;
            std::string coll_file = obj_file;
            if (coll_file.find("body") == std::string::npos){
                std::cout<<"\t\t\treplacing!!!!"<<std::endl;
                coll_file.replace(coll_file.find("View"), 4, "Collision");
                coll_file.replace(coll_file.find("_OBJ"), 4, "_Collision_OBJ");
            }            
            auto coll_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(coll_file));
            // std::cout<<"\t\t\t2"<<std::endl;
    
            // Compute mass properties
            double volume;
            ChMatrix33<> geometric_inertia;
            trimesh->ComputeMassProperties(true, volume, cog, geometric_inertia);
            // std::cout<<"\t\t\t3"<<std::endl;
    
            // Calculate mass and inertia
            double mass = density * volume;
            ChMatrix33<> inertia = density * geometric_inertia;
    
            // Create rigid body
            body = chrono_types::make_shared<ChBody>();
            body->SetFixed(is_fixed);
            body->SetMass(mass);
            body->SetInertiaXX(ChVector3d(inertia(0, 0), inertia(1, 1), inertia(2, 2)));
            // body->SetPos(cog);
            
            // std::cout<<"\t\t\t4"<<std::endl;
            system.Add(body);
    
            // std::cout<<"\t\t\t5"<<std::endl;
            // Visualization
            mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            mesh->SetMesh(trimesh);
            // mesh->SetOpacity(0.5f);
            // mesh->SetBackfaceCull(true);
            // mesh->SetColor(colorr);
            body->AddVisualShape(mesh, ChFrame<>(ChVector3d(0,0,0), ChMatrix33<>(1)));
    
            // Collision
            coll_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            coll_mesh->SetMesh(coll_trimesh);
            coll_mesh->SetVisible(false);
            body->AddVisualShape(coll_mesh, ChFrame<>(ChVector3d(0,0,0), ChMatrix33<>(1)));
    
            // Setup collision model
            coll_model = chrono_types::make_shared<ChCollisionModel>();
            coll_model->SetSafeMargin(0.1f);
            coll_model->SetEnvelope(0.001f);
            coll_trimesh->Transform(-cog, ChMatrix33<>(1));
            auto coll_mat = chrono_types::make_shared<ChContactMaterialNSC>();
            coll_mat->SetFriction(0.30);
            coll_mat->SetRestitution(0.001);
            auto coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(coll_mat, coll_trimesh, false, false, 0.001);
            coll_model->AddShape(coll_shape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
            body->AddCollisionModel(coll_model);
            body->EnableCollision(false);

            body->GetCollisionModel()->SetFamily(1);
            body->GetCollisionModel()->DisallowCollisionsWith(2);
        }
    };

// ===============================================================================================================================================================================
// ======== MAIN LOOP ==============================================================================================================================================
// ===============================================================================================================================================================================

int main(int argc, char* argv[]) {
    ChSystemNSC sys = GravetySetup();
    // printf("Changed the obj names!!!\n\n");
    RigidBody rotor_winding(sys, "RotorWinding_OBJ", 8900.00 / (1e9));
    RigidBody shaft(sys, "Shaft_OBJ", 7850.00 / (1e9));
    RigidBody stator(sys, "Stator_OBJ", 7850.00 / (1e9), true);

    auto Shaft_body = shaft.GetBody();
    // auto RotorWinding_body = rotor_winding.GetBody();
    // auto Stator_body = stator.GetBody();

    std::vector<std::string> file_names = {"" , // empty value to start with index 1
        "../dynoObj/body_1_1",     // Part1_Motor
        "../dynoObj/body_2_1",     // GearA_Driver
        "../dynoObj/body_3_1",     // GearF
        "../dynoObj/body_4_1",     // frame
        "../dynoObj/body_5_1",     // GearB
        "../dynoObj/body_6_1",     // GearC
        "../dynoObj/body_7_1",     // GearD
        "../dynoObj/body_8_1",     // GearE
        "../dynoObj/body_9_1",     // Part2_flywheel
        "../dynoObj/body_10_1"     // Part2_dyno
    };
    std::vector<std::unique_ptr<RigidBody>> bodies(file_names.size());
    std::vector<std::shared_ptr<ChBody>> body_ptrs(file_names.size());
    std::vector<ChVector3d> cogs(file_names.size());
    std::vector<ChVector3d> positions = {ChVector3d(0,0,0),
        ChVector3d(-153.681408502864,232.071341649174,257.256405065421),
        ChVector3d(-153.681408502864,316.571341649174,257.256405065421),
        ChVector3d(-153.681408502864,-62.0286583508263,257.256405065421),
        ChVector3d(10.0501781487224,127.271341649174,219.573424974887),
        ChVector3d(-183.793091163963,320.571341649174,254.366929798653),
        ChVector3d(-213.749821851278,320.571341649174,280.873424974888),
        ChVector3d(-213.749821851277,-66.0286583508263,280.873424974888),
        ChVector3d(-183.793091163963,-66.0286583508263,254.366929798653),
        ChVector3d(-213.749821851277,127.271341649174,280.873424974888),
        ChVector3d(-153.681408502864,15.4713416491736,257.256405065421)
    };
    std::vector<ChQuaternion<>> rotss = { ChQuaternion<>(0.0,0.0,0.0,0.0),
        ChQuaternion<>(0.344860417986101,-0.344860417986101,0.617309721376921,0.617309721376921),
        ChQuaternion<>(0.69219181681608,-0.69219181681608,0.144466220040721,0.144466220040721),
        ChQuaternion<>(0.682163121105785,0.682163121105785,0.186154441803611,-0.186154441803611),
        ChQuaternion<>(1,0,0,0),
        ChQuaternion<>(0.590890270601115,0.590890270601115,0.388392440849382,-0.388392440849382),
        ChQuaternion<>(-0.0817797748423181,-0.0817797748423181,0.702361778876627,-0.702361778876627),
        ChQuaternion<>(0.707106781186548,-0.707106781186547,-8.1335083852247e-17,0),
        ChQuaternion<>(-0.227257494306092,0.227257494306092,0.669592436696918,0.669592436696918),
        ChQuaternion<>(0.661716586109785,0.661716586109785,-0.249261227765594,0.249261227765594),
        ChQuaternion<>(0.679959230544171,0.679959230544171,0.194050108986773,-0.194050108986773)
    };
    // Initialize RigidBody objects and store values (starting from index 1)
    bodies[1] = std::make_unique<RigidBody>(sys, file_names[1], 7850.00 / (1e9));
    bodies[2] = std::make_unique<RigidBody>(sys, file_names[2], 785000.00 / (1e9));
    bodies[3] = std::make_unique<RigidBody>(sys, file_names[3], 7850.00 / (1e9), true);
    bodies[4] = std::make_unique<RigidBody>(sys, file_names[4], 7850.00 / (1e9), true);
    bodies[5] = std::make_unique<RigidBody>(sys, file_names[5], 7850.00 / (1e9), true);
    bodies[6] = std::make_unique<RigidBody>(sys, file_names[6], 7850.00 / (1e9), true);
    bodies[7] = std::make_unique<RigidBody>(sys, file_names[7], 7850.00 / (1e9), true);
    bodies[8] = std::make_unique<RigidBody>(sys, file_names[8], 7850.00 / (1e9), true);
    bodies[9] = std::make_unique<RigidBody>(sys, file_names[9], 7850.00 / (1e9), true);
    bodies[10] = std::make_unique<RigidBody>(sys, file_names[10], 7850.00 / (1e9), true);
    for (size_t i = 1; i < file_names.size(); ++i) {
        body_ptrs[i] = bodies[i]->GetBody();
        cogs[i] = bodies[i]->GetCOG();
        body_ptrs[i]->SetPos(positions[i] - positions[4]);
        body_ptrs[i]->SetRot(rotss[i]);
        if(i==1)bodies[1]->setColor(ChColor(1,0,0));
    }
    auto Stator_body = body_ptrs[1];
    auto RotorWinding_body = body_ptrs[2];

    // ===========================================================================================================================================================================================
    // ======== LINK DEFINITION -> FIXED JOINT: frame - stator ====================================================================================================================================
    // ===========================================================================================================================================================================================
    ChVector3d RotorWinding_Shaft_Link_Position(body_ptrs[4]->GetPos());   // [mm] set the position in the 3D space of the link respect to the absolute frame
    //RotorWinding_Shaft_Link_Position[2] = RotorWinding_Shaft_Link_Position[2] + 7.0;  
    ChQuaternion<> RotorWinding_Shaft_Link_Orientation;
    RotorWinding_Shaft_Link_Orientation.SetFromAngleAxis(0.0 * (M_PI / 180.0), ChVector3d(1, 0, 0));       // !!! IMPORTANT !!! the Revolute is always arround Z-axis -> Set correctly the orientation 
    ChFrame<> RotorWinding_Shaft_Link_Frame(RotorWinding_Shaft_Link_Position, RotorWinding_Shaft_Link_Orientation);
    auto RotorWinding_Shaft_Link_Fixed = chrono_types::make_shared<ChLinkLockLock>();
    RotorWinding_Shaft_Link_Fixed->Initialize(body_ptrs[4],                      // Body 1  
        Stator_body,                     // Body 2  
        RotorWinding_Shaft_Link_Frame);        // Location and orientation of the frame   
    sys.AddLink(RotorWinding_Shaft_Link_Fixed);

    // ===========================================================================================================================================================================================
    // ======== LINK DEFINITION -> REVOLUTE JOINT: RotorWinding - Stator ====================================================================================================================================
    // ===========================================================================================================================================================================================
    
    AddVisualizationBall(sys, RotorWinding_body->GetPos());
    ChVector3d RotorWinding_Stator_Link_Position(RotorWinding_body->GetPos());            // [mm] set the position in the 3D space of the link respect to the absolute frame
    //RotorWinding_Stator_Link_Position[2] = RotorWinding_Stator_Link_Position[2] + 7.0;
    ChQuaternion<> RotorWinding_Stator_Link_Orientation;
    RotorWinding_Stator_Link_Orientation.SetFromAngleAxis(90.0 * (M_PI / 180.0), ChVector3d(1, 0, 0));       // !!! IMPORTANT !!! the Revolute is always arround Z-axis -> Set correctly the orientation 
    ChFrame<> RotorWinding_Stator_Link_Frame(RotorWinding_Stator_Link_Position, RotorWinding_Stator_Link_Orientation);
    auto RotorWinding_Stator_Link_Revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    RotorWinding_Stator_Link_Revolute->Initialize(RotorWinding_body,                      // Body 1  
        Stator_body,                     // Body 2  
        RotorWinding_Stator_Link_Frame);        // Location and orientation of the frame  
    sys.AddLink(RotorWinding_Stator_Link_Revolute);
    AddCylAxis(sys, RotorWinding_body->GetPos(), RotorWinding_Stator_Link_Orientation);

    // ===========================================================================================================================================================================================
    // ======== DYNAMIC FORCES AND TORQUES CRATION ================================================================================================================================================
    // ===========================================================================================================================================================================================

    // ===========================================================================================================================================================================================
    // ======== F / T DEFINITION -> UNIVERSAL FORCE: RotorWinding - Stator ====================================================================================================================================
    // ===========================================================================================================================================================================================
    // ======== TORQUE TEMEPLATE ===========================================================================================================================================================================
    ChVector3d Torque_direction_RotorWinding_Stator(0, 1, 0); // IMPORTANT!! the direction vertex need to be normalized  
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
    AddVisualizationBall(sys, Stator_body->GetPos(), ChColor(0,0,1));
    ChVector3d RotorWinding_Stator_Torsional_Spring_Position(RotorWinding_body->GetPos());  //[mm] set the position in the 3D space of the link respect to the absolute frame
    //RotorWinding_Stator_Torsional_Spring_Position[2] += 6.0;  //[mm] Rise the position of the spring along y-axis in order to see it better in the animation
    ChQuaternion<> RotorWinding_Stator_Torsional_Spring_Orientation;
    RotorWinding_Stator_Torsional_Spring_Orientation.SetFromAngleAxis(90.0 * M_PI / 180.0, ChVector3d(0, 1, 0)); // !!! IMPORTANT !!! the Torsional Spring is oriented always arround Z-axis -> Set correctly the orientation 
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

    // ===========================================================================================================================================================================================
    // ======== MULTI-PHYSICS CO-SYMULATION LOOP ====================================================================================================================================================================
    // ===========================================================================================================================================================================================
    // ======== SET -> the Multi-physics timeline ====================================================================================================================================================================
    double t_simulation_STOP = 10.0;//400.0e-3; //[s]
    double t_sim_mechanics = 0.0; //[s] 
    double t_sim_electronics = 0.0; //[s]
    double t_sampling_electronic_counter = 0; //[s] This variable is needed to count the event at which the Electronic domain need to be called respect to the Global Time-line
    int brake_flag = 1; // Set a brake flag in the case you want to stop the simulation before: t_simulation_STOP
    double Imotor = 0.0;
    
    std::cout << "\n";
    std::cout << "===================================================" << "\n";
    std::cout << "======= AsASfsadfsdfsadfsadfsadfsadfsadfasdfsadfasdfasdfsadfsadf =======" << "\n"; 
    std::cout << "===================================================" << "\n";
    std::cout << "\n";
    // system("pause>0");
    double T_PWM = 0.04; //[s] PWM Period
    double Duty_PWM = 100.0 / 100; //[s] PWM Duty
    double t_PWM_counter = 0.0; //[s] PWM Period

    while (t_sim_mechanics < t_simulation_STOP && brake_flag == 1) {
        // ======== RUN -> the Irrlicht visualizer ====================================================================================================================================================================
        vis->Run();
        //tools::drawGrid(vis.get(), 2, 2, 30, 30, ChCoordsys<>(ChVector3d(0, 0.01, 0), QuatFromAngleX(CH_PI_2)),ChColor(0.3f, 0.3f, 0.3f), true);
        if (vis->Run()) { brake_flag = 1; } // Check if the User wanted to stop de simulation before: t_simulation_STOP
        else { brake_flag = 0; }
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
        std::vector<double> Rotor_Euler_Ang = GetEulerAngPos(RotorWinding_body, t_step_mechanic);

        // ======== TORQUE TEMEPLATE ====================================================================================================================================================================
        // ======== UPDATE -> Forces and Torques: RotorWinding - Stator ====================================================================================================================================================================
        double kt_motor = 0.022446; //[Nm/A] 150
        Torque_magnitude_RotorWinding_Stator = kt_motor * Imotor * 1e3 * 1e3; // Conversion to ([kg]-[mm]-[s])    
        RotorWinding_Stator_Torque = -1.0 * Torque_magnitude_RotorWinding_Stator * Torque_direction_RotorWinding_Stator;
        RotorWinding_body->EmptyAccumulators(); // Clean the body from the previous force/torque IMPORTANT!!!!: Uncomment this line if you never clean the F/T to this body
        RotorWinding_body->AccumulateTorque(RotorWinding_Stator_Torque, false); // Apply to the body the force
        
        // ======== SAVE -> the needed variables ====================================================================================================================================================================
        OutputMap["alpha"].push_back(-Rotor_Euler_Ang[0]);
        OutputMap["t_mechanics"].push_back(t_sim_mechanics);
        // OutputMap["T_magnetic"].push_back(-1.0 * Torque_magnitude_Cam_Magnet);
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
    std::ofstream out_file("output.json");
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
