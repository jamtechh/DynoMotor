#include "objInfo1.h"
std::vector<ChVector3d> positions = {ChVector3d(0,0,0),
    ChVector3d( -60.0684, 23.617, 0 ),
    ChVector3d( 163.732, -37.683, 0 ),
    ChVector3d( 0, 0, -188.7 ),
    ChVector3d( -30.112, -2.889, -193.3 ),
    ChVector3d( 0, 0, -115.8 ),
    ChVector3d( -60.0684, 23.617, -193.3 ),
    ChVector3d( -60.0684, 23.617, 192.7 ),
    ChVector3d( -30.112, -2.889, 192.7 ),
    ChVector3d( 0, 0, 188.7 ),
    ChVector3d( 0, 0, 122.8 )
};
std::vector<ChQuaternion<>> rotss = { ChQuaternion<>(0.0,0.0,0.0,0.0),
    ChQuaternion<>( 0, 0, 0, 1 ),
    ChQuaternion<>( 0.7071, -0.7071, 0, 0 ),
    ChQuaternion<>( 0, 1, 0, 0 ),
    ChQuaternion<>( 1, 0, 0, 0 ),
    ChQuaternion<>( 0, 0, 1, 0 ),
    ChQuaternion<>( 0, 0, 0, 1 ),
    ChQuaternion<>( 0, 1, 0, 0 ),
    ChQuaternion<>( 0, 0, 1, 0 ),
    ChQuaternion<>( 1, 0, 0, 0 ),
    ChQuaternion<>( 1, 0, 0, 0)
};

std::vector<ChVector3d> inertiaXX = {ChVector3d(0,0,0),
    ChVector3d( 28809.63, 28809.63, 1282.39 ),
    ChVector3d( 29568.92, 73541.07, 45865.33 ),
    ChVector3d( 17.171, 17.171, 6.269 ),
    ChVector3d( 0.604, 0.604, 1.184 ),
    ChVector3d( 9.576, 9.786, 4.288 ),
    ChVector3d( 0.604, 0.604, 1.184 ),
    ChVector3d( 0.604, 0.604, 1.184 ),
    ChVector3d( 0.604, 0.604, 1.184 ),
    ChVector3d( 14.586, 14.586, 8.289 ),
    ChVector3d( 12.643, 12.435, 6.701)
};

std::vector<double> mass = {0.0,
    2.7008,     // Corrected
    1.542,
    0.0741,     // Corrected
    0.006,
    0.016,
    0.006,
    0.006,
    0.006,
    0.050,
    0.023
};

std::vector<std::string> file_locations = {"" , // empty value to start with index 1
    "body_1_1",     
    "body_2_1",     
    "body_3_1",     
    "body_4_1",     
    "body_5_1",     
    "body_6_1",     
    "body_7_1",     
    "body_8_1",     
    "body_9_1",     
    "body_10_1"     
};
std::vector<std::string> file_names = {"" , // empty value to start with index 1
    "Part2_flywheel-1",
    "frame-1",
    "GearA-2",
    "GearB-1",
    "Part1-1",
    "GearC-1",
    "GearD-1",
    "GearE-1",
    "GearF-1",
    "Part2_dyno-1"        
};
                                            // Part1
                                            // GearA
                                            // GearF
                                            // frame
                                            // GearB
                                            // GearC
                                            // GearD
                                            // GearE
                                            // Part2_flywheel
                                            // Part2_dyno

std::vector<std::tuple<std::string, ChVector3d, ChQuaternion<>, ChVector3d, double, std::string>> objData = {
    {file_locations[0], positions[0], rotss[0], inertiaXX[0], mass[0], file_names[0]},      //
    {file_locations[5], positions[5], rotss[5], inertiaXX[5], mass[5], file_names[5]},      //Part1
    {file_locations[3], positions[3], rotss[3], inertiaXX[3], mass[3], file_names[3]},      //GearA
    {file_locations[9], positions[9], rotss[9], inertiaXX[9], mass[9], file_names[9]},      //GearF
    {file_locations[2], positions[2], rotss[2], inertiaXX[2], mass[2], file_names[2]},      //frame
    {file_locations[4], positions[4], rotss[4], inertiaXX[4], mass[4], file_names[4]},      //GearB
    {file_locations[6], positions[6], rotss[6], inertiaXX[6], mass[6], file_names[6]},      //GearC
    {file_locations[7], positions[7], rotss[7], inertiaXX[7], mass[7], file_names[7]},      //GearD
    {file_locations[8], positions[8], rotss[8], inertiaXX[8], mass[8], file_names[8]},      //GearE
    {file_locations[1], positions[1], rotss[1], inertiaXX[1], mass[1], file_names[1]},      //Part2_flywheel
    {file_locations[10], positions[10], rotss[10], inertiaXX[10], mass[10], file_names[10]} //Part2_dyno    
};

ChVector3d posOffset =  positions[9];

int seeCache(const std::string cacheFile){
    // const std::string cacheFile = "cache.txt";
    int runCount = 0;

    // Read existing count
    std::ifstream inFile(cacheFile);
    if (inFile.is_open()) {
        inFile >> runCount;
        inFile.close();
    }

    // Increment count
    runCount++;

    // Write updated count
    std::ofstream outFile(cacheFile);
    if (outFile.is_open()) {
        outFile << runCount;
        outFile.close();
    }

    std::cout << "This program has been run " << runCount << " times.\n";

    return runCount;
}


// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! functions !!!!!!!!!!!!!!!!!!!!!
std::shared_ptr<ChBody> Frame_body;
ChQuaternion<> jointOrientation;

double CumTrapezIntegration::Integrate(double& dt, double& f_new) {
    f_new1 = f_new;
    dt1 = dt;
    Integral_res += dt1 * ((f_old1 + f_new1) / 2);
    f_old1 = f_new1;
    return Integral_res;
}

std::vector<double> GetEulerAngPos(std::shared_ptr<ChBody> body, double& t_step) {
    static CumTrapezIntegration yawInt, pitchInt, rollInt;
    auto vel = body->GetAngVelLocal();
    return {
        yawInt.Integrate(t_step, vel[0]),
        pitchInt.Integrate(t_step, vel[1]),
        rollInt.Integrate(t_step, vel[2])
    };
}

std::string toLowerCase(const std::string& str) {
    std::string lower = str;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c){ return std::tolower(c); });
    return lower;
}

ChSystemNSC GravetySetup() {
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(9.81e3, 0, 0));
    return sys;
}

void createJoint(ChSystemNSC& sys, std::shared_ptr<ChBody> a, std::shared_ptr<ChBody> b, JointType type, const ChVector3d& pos, bool showAxis) {
    auto jointFrame = ChFrame<>(pos, jointOrientation);
    std::shared_ptr<ChLinkLock> joint;
    if (type == JointType::FIXED) joint = chrono_types::make_shared<ChLinkLockLock>();
    else if (type == JointType::REVOLUTE) joint = chrono_types::make_shared<ChLinkLockRevolute>();
    else if (type == JointType::PRISMATIC) joint = chrono_types::make_shared<ChLinkLockPrismatic>();
    joint->Initialize(a, b, jointFrame);
    sys.AddLink(joint);
}

void gearMate(ChSystemNSC& sys, std::shared_ptr<ChBody> a, std::shared_ptr<ChBody> b, double radA, double radB) {
    auto gear = chrono_types::make_shared<ChLinkLockGear>();
    gear->Initialize(a, b, ChFrame<>());
    auto rotZ = chrono::QuatFromAngleZ(CH_PI_2);
    gear->SetFrameShaft1(ChFrame<>(VNULL, rotZ));
    gear->SetFrameShaft2(ChFrame<>(VNULL, rotZ));
    gear->SetTransmissionRatio(radA / radB);
    sys.AddLink(gear);
}

RigidBody::RigidBody(ChSystemNSC& sys, const std::string& file_name, bool is_fixed, bool transparent)
    : system(sys), obj_file(file_name), is_fixed(is_fixed) {
    SetupRigidBody(transparent);
}

std::shared_ptr<ChBody> RigidBody::GetBody() const { return body; }
ChVector3d RigidBody::GetCOG() const { return cog; }
std::tuple<std::shared_ptr<ChBody>, ChVector3d> RigidBody::GetBodyAndCOG() const { return { body, cog }; }
void RigidBody::setPos(const ChVector3d& pos) { body->SetPos(pos); }
void RigidBody::setColor(const ChColor& color) { mesh->SetColor(color); }
void RigidBody::setTransparent() { mesh->SetOpacity(0.5f); }

void RigidBody::setData(const std::tuple<std::string, ChVector3d, ChQuaternion<>, ChVector3d, double, std::string>& data) {
    position = std::get<1>(data);
    rotation = std::get<2>(data);
    inertia_SW = std::get<3>(data);
    mass_SW = std::get<4>(data);
    file_name = std::get<5>(data);
    double scaleInertia = 8.5 / mass_SW;

    body->SetPos(position - posOffset);
    body->SetRot(rotation);
    body->SetMass(mass_SW);
    body->SetInertiaXX(inertia_SW);

    if (debugPrint) {
        std::cout << std::fixed << std::setprecision(3)
                  << std::setw(15) << file_name << "  "
                  << std::setw(5) << "m= " << std::setw(8) << mass_SW << "  "
                  << std::setw(5) << "v= " << std::setw(12) << volume << "  "
                  << std::setw(20) << inertia_SW[0] 
                  << std::setw(20) << inertia_SW[1] 
                  << std::setw(20) << inertia_SW[2] 
                  << std::setw(20) << scaleInertia << std::endl;
    }
}

void RigidBody::SetupRigidBody(bool transparent) {
    obj_file = "my_project/CAD/DynoObj3_shapes/" + obj_file + ".obj";
    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(obj_file));
    trimesh->ComputeMassProperties(true, volume, cog, geometric_inertia_calc);

    density = 8970.0 / 1e9;
    mass_calc = density * volume;
    inertia_calc = density * geometric_inertia_calc;

    body = chrono_types::make_shared<ChBody>();
    body->SetFixed(is_fixed);
    system.Add(body);

    mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    mesh->SetMesh(trimesh);
    mesh->SetMutable(false);
    if (transparent) mesh->SetOpacity(0.5f);
    mesh->SetBackfaceCull(true);
    body->AddVisualShape(mesh);
}