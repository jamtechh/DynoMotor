// objInfo1.h
#ifndef OBJINFO1_H
#define OBJINFO1_H

#include <iostream>
#include <iomanip>
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
#include <chrono>

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
using namespace std::chrono;
using namespace chrono;
// using namespace std;
using namespace chrono::irrlicht;
using namespace ::chrono::powerelectronics;
using json = nlohmann::json;

using namespace irr; // Use the main namespaces of Irrlicht
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

// Declare global variables (they will be defined in objInfo1.cpp)
extern std::vector<std::string> file_locations;
extern std::vector<std::string> file_names;
extern std::vector<ChVector3d> positions;
extern std::vector<ChQuaternion<>> rotss;
extern std::vector<ChVector3d> inertiaXX;
extern std::vector<double> mass;

extern std::vector<std::tuple<std::string, ChVector3d, ChQuaternion<>, ChVector3d, double, std::string>> objData;

extern ChVector3d posOffset;


int seeCache(const std::string cacheFile);

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!   functions !!!!!!!!!!!!
enum class JointType { FIXED, REVOLUTE, PRISMATIC };
class CumTrapezIntegration {
public:
    double Integrate(double& dt, double& f_new);
private:
    double Integral_res = 0.0, f_old1 = 0.0, f_new1 = 0.0, dt1 = 0.0;
};

std::vector<double> GetEulerAngPos(std::shared_ptr<ChBody> body, double& t_step);
std::string toLowerCase(const std::string& str);
ChSystemNSC GravetySetup();

void createJoint(ChSystemNSC& sys, std::shared_ptr<ChBody> a, std::shared_ptr<ChBody> b, JointType type, const ChVector3d& pos, bool showAxis = false);
void gearMate(ChSystemNSC& sys, std::shared_ptr<ChBody> a, std::shared_ptr<ChBody> b, double radA, double radB);

class RigidBody {
public:
    RigidBody(ChSystemNSC& sys, const std::string& file_name, bool is_fixed = false, bool transparent = false);
    std::shared_ptr<ChBody> GetBody() const;
    ChVector3d GetCOG() const;
    std::tuple<std::shared_ptr<ChBody>, ChVector3d> GetBodyAndCOG() const;
    void setPos(const ChVector3d& pos);
    void setColor(const ChColor& color);
    void setTransparent();
    void setData(const std::tuple<std::string, ChVector3d, ChQuaternion<>, ChVector3d, double, std::string>& data);

private:
    void SetupRigidBody(bool transparent);
    ChSystemNSC& system;
    std::string obj_file, file_name;
    bool is_fixed, debugPrint = true;
    double density, mass_calc, volume, mass_SW;
    ChVector3d cog, position, inertia_SW;
    ChQuaternion<> rotation;
    ChMatrix33<> inertia_calc, geometric_inertia_calc;
    std::shared_ptr<ChBody> body;
    std::shared_ptr<ChVisualShapeTriangleMesh> mesh;
};

extern std::shared_ptr<ChBody> Frame_body;
extern ChQuaternion<> jointOrientation;


#endif // OBJINFO1_H