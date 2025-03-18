// objInfo1.h
#ifndef OBJINFO1_H
#define OBJINFO1_H

#include <chrono>
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
// using namespace std;
using namespace chrono::irrlicht;
using namespace ::chrono::powerelectronics;
using json = nlohmann::json;
using std::chrono::high_resolution_clock;
using std::chrono::duration;

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

#endif // OBJINFO1_H