// ===============================================================================================================================================================================
// ======== HEADERS ==============================================================================================================================================
// ===============================================================================================================================================================================

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

// ===============================================================================================================================================================================
// ======== FUNCTIONS & CLASSES ==============================================================================================================================================
// ===============================================================================================================================================================================

// ======== Method: allows to put in pause the execution for sec ==============================================================================================================================================
void sleep(int seconds) {
    std::this_thread::sleep_for(std::chrono::seconds(seconds)); 
}

// ======== Method: allows to read a .json file ==============================================================================================================================================
bool readJsonFile(const std::string& filename, json& j) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening JSON file." << std::endl;
        return false;
    }
    file >> j;
    return true;
}

// ======== Method: allows to write into a .json file ==============================================================================================================================================
auto writeJsonFile(const std::string& filename, json data)
{
    // Specify the file name
    std::string fileName = "Sim_Res/" + filename + ".json";
    // Open a file stream for writing
    std::ofstream file(fileName);
    // Check if the file stream is open
    if (!file.is_open()) {
        std::cerr << "Error opening file for writing!" << std::endl;
        return 1;
    }
    // Write the JSON object to the file
    file << data.dump(4); // Pretty print with indentation of 4 spaces
    // Close the file stream
    file.close();
    std::cout << "\n!!!!! Results exported to: " << fileName << " !!!!!\n";
}

// ======== Class: allows to compute the derivative in-between a single simulation time-step through the numerical differentiation method ==============================================================================================================================================
class NumericalDifferentiation {
public:
    double Differentiate(double& dt, double& f_new)
    {
        f_new1 = f_new;
        dt1 = dt;
        if (flag == 0)
        {
            Derivative_res = 0.0;
            f_old1 = f_new1;
            flag = 1;
        }
        else
        {
            Derivative_res = (f_new1 - f_old1) / dt1;
            f_old1 = f_new1;
        }
        return Derivative_res;
    }
private:
    double Derivative_res = 0.0;
    double f_old1 = 0.0;
    double f_new1;
    double dt1;
    int flag = 0;
};

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

// ======== Class: Replica of the Simulink block: 1-D Lookup Table ==============================================================================================================================================
class LookupTable1D {
public:
    LookupTable1D(const std::vector<double>& breakpoints, const std::vector<double>& table_data)
        : breakpoints(breakpoints), table_data(table_data) {}

    double interpolate(double x) {
        // Find the index of the left breakpoint
        size_t left_index = 0;
        for (size_t i = 1; i < breakpoints.size(); ++i) {
            if (breakpoints[i] <= x) {
                left_index = i;
            }
            else {
                break;
            }
        }
        // If x is beyond the last breakpoint, use the last data point
        if (left_index == breakpoints.size() - 1) {
            return table_data.back();
        }
        // Linear interpolation
        double x_left = breakpoints[left_index];
        double x_right = breakpoints[left_index + 1];
        double y_left = table_data[left_index];
        double y_right = table_data[left_index + 1];
        double y_interp = y_left + (y_right - y_left) * (x - x_left) / (x_right - x_left);
        return y_interp;
    }
private:
    std::vector<double> breakpoints;
    std::vector<double> table_data;
};

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


// ===============================================================================================================================================================================
// ======== MAIN LOOP ==============================================================================================================================================
// ===============================================================================================================================================================================

int main(int argc, char* argv[]) {
    

    // ===============================================================================================================================================================================
    // ======== MECHANICAL DOMAIN ==============================================================================================================================================
    // ===============================================================================================================================================================================

    // ===============================================================================================================================================================================
    // ======== CREATE A NSC CHRONO SYSTEM ==============================================================================================================================================
    // ===============================================================================================================================================================================
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    ChSystemNSC sys; // Create a Chrono physical system
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    // NSC = Non Smooth Contact that is suitable for impact phenomena
    // SMC = SMooth Contacy that i suitable for continuous phenomena
    // Mechanical Unit system: [kg]-[mm]-[s] -> The use of [mm] is require cause extrimelly small collisiion parameters crash the simulation
    // Electric Unit system: [kg]-[m]-[s]

    // ===============================================================================================================================================================================
    // ======== SET THE GRAVITY ACCELERATION ==============================================================================================================================================
    // ===============================================================================================================================================================================
    ChVector3d gravity_acc = sys.GetGravitationalAcceleration(); 
    std::cout << "The gravity acceleration  vector is: " << gravity_acc << "\n\n";
    double gravity = 9.81e3; //[mm/s^2]
    sys.SetGravitationalAcceleration(ChVector3d(0, -gravity, 0));
    ChVector3d gravity_acc_new = sys.GetGravitationalAcceleration(); 
    std::cout << "The new gravity acceleration  vector is: " << gravity_acc_new << "\n\n";
  
    // ===============================================================================================================================================================================
    // ======== RIGID BODIES CREATION ==============================================================================================================================================
    // ===============================================================================================================================================================================

    // ===========================================================================================================================================================================================
    // ======== RIGID BODY DEFINITION: Floor ====================================================================================================================================
    // ===========================================================================================================================================================================================
    /*
    auto floor_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    floor_mat->SetFriction(1.0); 
    auto floor = chrono_types::make_shared<ChBodyEasyBox>(2, 300, 300, 1000, true, true, floor_mat);
    floor->SetPos(ChVector3d(100.0, 0.0, 0.0));
    floor->SetFixed(true);
    floor->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
    sys.Add(floor);*/

    // ===========================================================================================================================================================================================
    // ======== RIGID BODY DEFINITION: WaveFront Shape -> RotorWinding ====================================================================================================================================
    // ===========================================================================================================================================================================================
    // ======== File name ========================================================================================================================================================================
    std::string RotorWinding_file_name = "my_project/CAD/dynoObj/body_2_1.obj";
    std::string RotorWinding_Collision_file_name = "my_project/CAD/dynoObj/body_2_1.obj";
    // ======== MESHES ===========================================================================================================================================================================
    // ======== Visualization Mesh ===============================================================================================================================================================
    auto RotorWinding_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(RotorWinding_file_name));
    auto RotorWinding_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    RotorWinding_mesh->SetMesh(RotorWinding_trimesh);
    RotorWinding_mesh->SetVisible(true);
    // ======== Visualization Collision Mesh =====================================================================================================================================================
    auto RotorWinding_coll_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(RotorWinding_Collision_file_name));
    auto RotorWinding_coll_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    RotorWinding_coll_mesh->SetMesh(RotorWinding_coll_trimesh);
    RotorWinding_coll_mesh->SetVisible(false);
    // ======== Triangle Mesh for collision the model ============================================================================================================================================
    auto trimesh_RotorWinding = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(RotorWinding_Collision_file_name));
    // ======== Compute mass inertia from mesh ===================================================================================================================================================
    double RotorWinding_volume; // [mm^3]  
    ChVector3d RotorWinding_cog; // [mm]   
    ChMatrix33<> RotorWinding_geometric_inertia; // [mm^5] it is the geometric inertia tensor (see when you call: RotorWinding_mesh->ComputeMassProperties) 
    RotorWinding_trimesh->ComputeMassProperties(true, RotorWinding_volume, RotorWinding_cog, RotorWinding_geometric_inertia); // It returns: RotorWinding_volume:[mm^3], RotorWinding_cog:[mm], RotorWinding_inertia:[mm^5] that is the geometric inertia tensor 
    // double RotorWinding_density = 8900.00 / (1e9); // [kg/mm^3]
    double RotorWinding_density = 8900.00 / (1e9); // [kg/mm^3]
    double RotorWinding_mass = RotorWinding_density * RotorWinding_volume; // [kg]
    ChMatrix33<> RotorWinding_inertia = RotorWinding_density * RotorWinding_geometric_inertia; // [kg*mm^2]
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "!!!!!!! RotorWinding -> Inertia properies: !!!!!!!" << "\n"; // Display the Inertia properties of the body
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "The RotorWinding mass is: " << RotorWinding_mass << " [kg]" << "\n\n";
    std::cout << "The RotorWinding cog is: " << RotorWinding_cog << " [mm]" << "\n\n";
    std::cout << "The RotorWinding inertia tensor is:\n" << RotorWinding_inertia << " [kg*mm^2]" << "\n\n";
    // ======== Define the rigid body ============================================================================================================================================================
    auto RotorWinding_body = chrono_types::make_shared<ChBody>();
    sys.Add(RotorWinding_body);
    RotorWinding_body->SetFixed(false);
    RotorWinding_body->SetMass(100000);
    RotorWinding_body->SetInertiaXX(ChVector3d(RotorWinding_inertia(0, 0), RotorWinding_inertia(1, 1), RotorWinding_inertia(2, 2)));
    RotorWinding_body->SetPos(RotorWinding_cog);
    // ======== Visulaization ====================================================================================================================================================================
    RotorWinding_mesh->SetMutable(false);
    RotorWinding_mesh->SetColor(ChColor(0.0f, 0.616f, 1.0f));
    RotorWinding_mesh->SetOpacity(0.5f);
    RotorWinding_mesh->SetBackfaceCull(true);
    RotorWinding_body->AddVisualShape(RotorWinding_mesh, ChFrame<>(-RotorWinding_cog, ChMatrix33<>(1)));
    RotorWinding_body->AddVisualShape(RotorWinding_coll_mesh, ChFrame<>(-RotorWinding_cog, ChMatrix33<>(1)));
    // ======== Collision ========================================================================================================================================================================
    auto RotorWinding_coll_model = chrono_types::make_shared<ChCollisionModel>();
    RotorWinding_coll_model->SetSafeMargin(0.1f);  // inward safe margin
    RotorWinding_coll_model->SetEnvelope(0.001f);    // distance of the outward "collision envelope"
    trimesh_RotorWinding->Transform(-RotorWinding_cog, ChMatrix33<>(1));
    auto RotorWinding_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    RotorWinding_mat->SetFriction(0.30);
    RotorWinding_mat->SetRestitution(0.001); //In the range[0, 1].
    auto RotorWinding_coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(RotorWinding_mat, trimesh_RotorWinding, false, false, 0.001);
    RotorWinding_coll_model->AddShape(RotorWinding_coll_shape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    RotorWinding_body->AddCollisionModel(RotorWinding_coll_model);
    RotorWinding_body->EnableCollision(false);
    RotorWinding_body->GetCollisionModel()->SetFamily(1);
    RotorWinding_body->GetCollisionModel()->DisallowCollisionsWith(2);

    // ===========================================================================================================================================================================================
    // ======== RIGID BODY DEFINITION: WaveFront Shape -> Shaft ====================================================================================================================================
    // ===========================================================================================================================================================================================
    // ======== File name ========================================================================================================================================================================
    std::string Shaft_file_name = "my_project/CAD/dynoObj/body_2_1.obj";
    std::string Shaft_Collision_file_name = "my_project/CAD/dynoObj/body_2_1.obj";
    // ======== MESHES ===========================================================================================================================================================================
    // ======== Visualization Mesh ===============================================================================================================================================================
    auto Shaft_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Shaft_file_name));
    auto Shaft_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    Shaft_mesh->SetMesh(Shaft_trimesh);
    Shaft_mesh->SetVisible(true);
    // ======== Visualization Collision Mesh =====================================================================================================================================================
    auto Shaft_coll_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Shaft_Collision_file_name));
    auto Shaft_coll_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    Shaft_coll_mesh->SetMesh(Shaft_coll_trimesh);
    Shaft_coll_mesh->SetVisible(false);
    // ======== Triangle Mesh for collision the model ============================================================================================================================================
    auto trimesh_Shaft = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Shaft_Collision_file_name));
    // ======== Compute mass inertia from mesh ===================================================================================================================================================
    double Shaft_volume; // [mm^3]  
    ChVector3d Shaft_cog; // [mm]   
    ChMatrix33<> Shaft_geometric_inertia; // [mm^5] it is the geometric inertia tensor (see when you call: Shaft_mesh->ComputeMassProperties) 
    Shaft_trimesh->ComputeMassProperties(true, Shaft_volume, Shaft_cog, Shaft_geometric_inertia); // It returns: Shaft_volume:[mm^3], Shaft_cog:[mm], Shaft_inertia:[mm^5] that is the geometric inertia tensor 
    double Shaft_density = 7850.00 / (1e9); // [kg/mm^3]
    double Shaft_mass = Shaft_density * Shaft_volume; // [kg]
    ChMatrix33<> Shaft_inertia = Shaft_density * Shaft_geometric_inertia; // [kg*mm^2]
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "!!!!!!! Shaft -> Inertia properies: !!!!!!!" << "\n"; // Display the Inertia properties of the body
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "The Shaft mass is: " << Shaft_mass << " [kg]" << "\n\n";
    std::cout << "The Shaft cog is: " << Shaft_cog << " [mm]" << "\n\n";
    std::cout << "The Shaft inertia tensor is:\n" << Shaft_inertia << " [kg*mm^2]" << "\n\n";
    // ======== Define the rigid body ============================================================================================================================================================
    auto Shaft_body = chrono_types::make_shared<ChBody>();
    sys.Add(Shaft_body);
    Shaft_body->SetFixed(false);
    Shaft_body->SetMass(10000000);
    Shaft_body->SetInertiaXX(ChVector3d(Shaft_inertia(0, 0), Shaft_inertia(1, 1), Shaft_inertia(2, 2)));
    Shaft_body->SetPos(Shaft_cog);
    // ======== Visulaization ====================================================================================================================================================================
    Shaft_mesh->SetMutable(false);
    Shaft_mesh->SetColor(ChColor(1.0f, 0.761f, 0.0f));
    Shaft_mesh->SetOpacity(0.5f);
    Shaft_mesh->SetBackfaceCull(true);
    Shaft_body->AddVisualShape(Shaft_mesh, ChFrame<>(-Shaft_cog, ChMatrix33<>(1)));
    Shaft_body->AddVisualShape(Shaft_coll_mesh, ChFrame<>(-Shaft_cog, ChMatrix33<>(1)));
    // ======== Collision ========================================================================================================================================================================
    auto Shaft_coll_model = chrono_types::make_shared<ChCollisionModel>();
    Shaft_coll_model->SetSafeMargin(0.1f);  // inward safe margin
    Shaft_coll_model->SetEnvelope(0.001f);    // distance of the outward "collision envelope"
    trimesh_Shaft->Transform(-Shaft_cog, ChMatrix33<>(1));
    auto Shaft_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    Shaft_mat->SetFriction(0.30);
    Shaft_mat->SetRestitution(0.001); //In the range[0, 1].
    auto Shaft_coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(Shaft_mat, trimesh_Shaft, false, false, 0.001);
    Shaft_coll_model->AddShape(Shaft_coll_shape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    Shaft_body->AddCollisionModel(Shaft_coll_model);
    Shaft_body->EnableCollision(false);
    Shaft_body->GetCollisionModel()->SetFamily(1);
    Shaft_body->GetCollisionModel()->DisallowCollisionsWith(2);

    // ===========================================================================================================================================================================================
    // ======== RIGID BODY DEFINITION: WaveFront Shape -> Stator ====================================================================================================================================
    // ===========================================================================================================================================================================================
    // ======== File name ========================================================================================================================================================================
    std::string Stator_file_name = "my_project/CAD/dynoObj/body_1_1.obj";
    std::string Stator_Collision_file_name = "my_project/CAD/dynoObj/body_1_1.obj";
    // ======== MESHES ===========================================================================================================================================================================
    // ======== Visualization Mesh ===============================================================================================================================================================
    auto Stator_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Stator_file_name));
    auto Stator_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    Stator_mesh->SetMesh(Stator_trimesh);
    Stator_mesh->SetVisible(true);
    // ======== Visualization Collision Mesh =====================================================================================================================================================
    auto Stator_coll_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Stator_Collision_file_name));
    auto Stator_coll_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    Stator_coll_mesh->SetMesh(Stator_coll_trimesh);
    Stator_coll_mesh->SetVisible(false);
    // ======== Triangle Mesh for collision the model ============================================================================================================================================
    auto trimesh_Stator = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Stator_Collision_file_name));
    // ======== Compute mass inertia from mesh ===================================================================================================================================================
    double Stator_volume; // [mm^3]  
    ChVector3d Stator_cog; // [mm]   
    ChMatrix33<> Stator_geometric_inertia; // [mm^5] it is the geometric inertia tensor (see when you call: Stator_mesh->ComputeMassProperties) 
    Stator_trimesh->ComputeMassProperties(true, Stator_volume, Stator_cog, Stator_geometric_inertia); // It returns: Stator_volume:[mm^3], Stator_cog:[mm], Stator_inertia:[mm^5] that is the geometric inertia tensor 
    double Stator_density = 7850.00 / (1e9); // [kg/mm^3]
    double Stator_mass = Stator_density * Stator_volume; // [kg]
    ChMatrix33<> Stator_inertia = Stator_density * Stator_geometric_inertia; // [kg*mm^2]
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "!!!!!!! Stator -> Inertia properies: !!!!!!!" << "\n"; // Display the Inertia properties of the body
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "The Stator mass is: " << Stator_mass << " [kg]" << "\n\n";
    std::cout << "The Stator cog is: " << Stator_cog << " [mm]" << "\n\n";
    std::cout << "The Stator inertia tensor is:\n" << Stator_inertia << " [kg*mm^2]" << "\n\n";
    // ======== Define the rigid body ============================================================================================================================================================
    auto Stator_body = chrono_types::make_shared<ChBody>();
    sys.Add(Stator_body);
    Stator_body->SetFixed(true);
    Stator_body->SetMass(Stator_mass);
    Stator_body->SetInertiaXX(ChVector3d(Stator_inertia(0, 0), Stator_inertia(1, 1), Stator_inertia(2, 2)));
    Stator_body->SetPos(Stator_cog);
    // ======== Visulaization ====================================================================================================================================================================
    Stator_mesh->SetMutable(false);
    Stator_mesh->SetColor(ChColor(1.0f, 0.0f, 0.0f));
    Stator_mesh->SetOpacity(0.5f);
    Stator_mesh->SetBackfaceCull(true);
    Stator_body->AddVisualShape(Stator_mesh, ChFrame<>(-Stator_cog, ChMatrix33<>(1)));
    Stator_body->AddVisualShape(Stator_coll_mesh, ChFrame<>(-Stator_cog, ChMatrix33<>(1)));
    // ======== Collision ========================================================================================================================================================================
    auto Stator_coll_model = chrono_types::make_shared<ChCollisionModel>();
    Stator_coll_model->SetSafeMargin(0.1f);  // inward safe margin
    Stator_coll_model->SetEnvelope(0.001f);    // distance of the outward "collision envelope"
    trimesh_Stator->Transform(-Stator_cog, ChMatrix33<>(1));
    auto Stator_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    Stator_mat->SetFriction(0.30);
    Stator_mat->SetRestitution(0.001); //In the range[0, 1].
    auto Stator_coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(Stator_mat, trimesh_Stator, false, false, 0.001);
    Stator_coll_model->AddShape(Stator_coll_shape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    Stator_body->AddCollisionModel(Stator_coll_model);
    Stator_body->EnableCollision(false);
    Stator_body->GetCollisionModel()->SetFamily(1);
    Stator_body->GetCollisionModel()->DisallowCollisionsWith(2);

    // ===========================================================================================================================================================================================
    // ======== KINEMATIC LINKS CREATION ====================================================================================================================================
    // ===========================================================================================================================================================================================
    
    // ===========================================================================================================================================================================================
    // ======== LINK DEFINITION -> FIXED JOINT: RotorWinding - Shaft ====================================================================================================================================
    // ===========================================================================================================================================================================================
    ChVector3d RotorWinding_Shaft_Link_Position(RotorWinding_body->GetPos());   // [mm] set the position in the 3D space of the link respect to the absolute frame
    //RotorWinding_Shaft_Link_Position[2] = RotorWinding_Shaft_Link_Position[2] + 7.0;  
    ChQuaternion<> RotorWinding_Shaft_Link_Orientation;
    RotorWinding_Shaft_Link_Orientation.SetFromAngleAxis(0.0 * (M_PI / 180.0), ChVector3d(0, 1, 0));       // !!! IMPORTANT !!! the Revolute is always arround Z-axis -> Set correctly the orientation 
    ChFrame<> RotorWinding_Shaft_Link_Frame(RotorWinding_Shaft_Link_Position, RotorWinding_Shaft_Link_Orientation);
    auto RotorWinding_Shaft_Link_Fixed = chrono_types::make_shared<ChLinkLockLock>();
    RotorWinding_Shaft_Link_Fixed->Initialize(RotorWinding_body,                      // Body 1  
        Shaft_body,                     // Body 2  
        RotorWinding_Shaft_Link_Frame);        // Location and orientation of the frame   
    sys.AddLink(RotorWinding_Shaft_Link_Fixed);

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
    // ======== F / T DEFINITION -> TORSIONAL SPRING/DAMPER: RotorWinding - Stator ====================================================================================================================================
    // ===========================================================================================================================================================================================
    // ======== Torsional spring coefficient ===========================================================================================================================================================================
    // double k_eq_RotorWinding_Stator_spr = 0.0; // [(N * m) / rad]
    // k_eq_RotorWinding_Stator_spr = k_eq_RotorWinding_Stator_spr * 1e3 * 1e3; // Conversion to ([kg]-[mm]-[s]) 
    // // ======== Torsional damping coefficient ===========================================================================================================================================================================
    // double r_ShaftBushing_experimental = 0.0003; //[(N*m*s)/rad]
    // double r_eq_RotorWinding_Stator_spr = r_ShaftBushing_experimental * 1e3 * 1e3; // Conversion to ([kg]-[mm]-[s])  
    // // ======== Torsional spring/damper implementation ===========================================================================================================================================================================
    // auto RotorWinding_Stator_Torsional_Spring = chrono_types::make_shared<ChLinkRSDA>();
    // ChVector3d RotorWinding_Stator_Torsional_Spring_Position(Stator_body->GetPos());  //[mm] set the position in the 3D space of the link respect to the absolute frame
    // //RotorWinding_Stator_Torsional_Spring_Position[2] += 6.0;  //[mm] Rise the position of the spring along y-axis in order to see it better in the animation
    // ChQuaternion<> RotorWinding_Stator_Torsional_Spring_Orientation;
    // RotorWinding_Stator_Torsional_Spring_Orientation.SetFromAngleAxis(90.0 * M_PI / 180.0, ChVector3d(0, 1, 0)); // !!! IMPORTANT !!! the Torsional Spring is oriented always arround Z-axis -> Set correctly the orientation 
    // ChFrame<> RotorWinding_Stator_Torsional_Spring_Frame(RotorWinding_Stator_Torsional_Spring_Position, RotorWinding_Stator_Torsional_Spring_Orientation);
    // RotorWinding_Stator_Torsional_Spring->Initialize(RotorWinding_body,                                   // Body 1  
    //     Stator_body,                                  // Body 2 
    //     false,                                        // the two following frames are in absolute, not relative, coords.
    //     RotorWinding_Stator_Torsional_Spring_Frame,          // Location and orientation of the Body 1 frame 
    //     RotorWinding_Stator_Torsional_Spring_Frame);         // Location and orientation of the Body 1 frame
    // RotorWinding_Stator_Torsional_Spring->SetRestAngle(0.0 * (M_PI / 180.0)); //[rad] Starting angular position
    // RotorWinding_Stator_Torsional_Spring->SetSpringCoefficient(k_eq_RotorWinding_Stator_spr); // [(kg mm mm)/(s^2 rad)] that should be the SI conversion ([kg]-[mm]-[s]) of [N m/rad]
    // RotorWinding_Stator_Torsional_Spring->SetDampingCoefficient(r_eq_RotorWinding_Stator_spr); // [(kg mm mm s)/(s^2 mm rad)] that should be the SI conversion ([kg]-[mm]-[s]) of [N m s/rad]
    // sys.AddLink(RotorWinding_Stator_Torsional_Spring);
    // RotorWinding_Stator_Torsional_Spring->AddVisualShape(chrono_types::make_shared<ChVisualShapeRotSpring>(60, 50)); // var1 = radius of the spring, var2 = graphical resolution of the spring
    // // ======== Torsional spring/damper visualization ===========================================================================================================================================================================
    // auto RotorWinding_Stator_Spring_Visual = chrono_types::make_shared<ChVisualShapeRotSpring>(2.5, 70); // var1 = radius of the spring, var2 = graphical resolution of the spring
    // RotorWinding_Stator_Spring_Visual->SetColor(ChColor(0.0f, 1.0f, 0.0f));  // RGB values
    // RotorWinding_Stator_Torsional_Spring->AddVisualShape(RotorWinding_Stator_Spring_Visual); 

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
    double f_ToSample_electronic = 1.0e3;//1.0e5;// 0.5e4; // [Hz]          Frequency at which the electronic domain is called respect to the global time line
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
    std::cout << "======= PRESS ENTER TO START THE SIMULATION =======" << "\n"; 
    std::cout << "===================================================" << "\n";
    std::cout << "\n";
    double T_PWM = 0.04; //[s] PWM Period
    double Duty_PWM = 80.0 / 100; //[s] PWM Duty
    double t_PWM_counter = 0.0; //[s] PWM Period

    // ===========================================================================================================================================================================================
    // ======== F / T DEFINITION -> UNIVERSAL FORCE: RotorWinding - Stator ====================================================================================================================================
    // ===========================================================================================================================================================================================
    // ======== TORQUE TEMEPLATE ===========================================================================================================================================================================
    ChVector3d Torque_direction_RotorWinding_Stator(1, 0, 0); // IMPORTANT!! the direction vertex need to be normalized  
    double Torque_magnitude_RotorWinding_Stator = 0.0 * 1e3 * 1e3; //[Nm] converted to ([kg]-[mm]-[s]) 
    ChVector3d RotorWinding_Stator_Torque = Torque_magnitude_RotorWinding_Stator * Torque_direction_RotorWinding_Stator;

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
            double ke_motor = -0.01; //[V/rpm]
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
            OutputMap["dalpha"].push_back(Rotor_Euler_Vel[0]);

            // ======== UPDATE -> the TIME variables ====================================================================================================================================================================
            t_sampling_electronic_counter = 0;      // The variable is nulled to re-start with the counter for the next call of the electronic domain
        }

        // ======== EXTRACT -> Kinematic variables ====================================================================================================================================================================
        std::vector<double> Rotor_Euler_Ang = GetEulerAngPos(RotorWinding_body, t_step_mechanic);

        // We apply the constant torque here !!!
        double kt_motor = 0.6; //[Nm/A] 150
        Torque_magnitude_RotorWinding_Stator = kt_motor * Imotor * 1e3 * 1e3; // Conversion to ([kg]-[mm]^2/[s^2])    
        RotorWinding_Stator_Torque = -1.0 * Torque_magnitude_RotorWinding_Stator * Torque_direction_RotorWinding_Stator;
        RotorWinding_body->EmptyAccumulators(); // Clean the body from the previous force/torque IMPORTANT!!!!: Uncomment this line if you never clean the F/T to this body
        RotorWinding_body->AccumulateTorque(RotorWinding_Stator_Torque, false); // Apply to the body the force
        
        // ======== SAVE -> the needed variables ====================================================================================================================================================================
        OutputMap["alpha"].push_back(Rotor_Euler_Ang[0]);
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
