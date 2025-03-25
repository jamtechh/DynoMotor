// C++ Chrono::Engine model automatically generated using Chrono::SolidWorks add-in
// Assembly: C:\Users\Jamiul\OneDrive - UW-Madison\Design\chronoMotor\Assem1.SLDASM

// dim  -> in mm
// mass -> in kg
// inert-> in kg.mm^2

#include <string>
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "DynoObj2.h"


/// Function to import Solidworks assembly directly into Chrono ChSystem.
void ImportSolidworksSystemCpp(chrono::ChSystem& system, std::unordered_map<std::string, std::shared_ptr<chrono::ChFunction>>* motfun_map) {
std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> bodylist;
std::vector<std::shared_ptr<chrono::ChLinkBase>> linklist;
ImportSolidworksSystemCpp(bodylist, linklist, motfun_map);
for (auto& body : bodylist)
    system.Add(body);
for (auto& link : linklist)
    system.Add(link);
}


/// Function to import Solidworks bodies and mates into dedicated containers.
void ImportSolidworksSystemCpp(std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>& bodylist, std::vector<std::shared_ptr<chrono::ChLinkBase>>& linklist, std::unordered_map<std::string, std::shared_ptr<chrono::ChFunction>>* motfun_map) {

// Some global settings
double sphereswept_r = 0.001;
chrono::ChCollisionModel::SetDefaultSuggestedEnvelope(3);
chrono::ChCollisionModel::SetDefaultSuggestedMargin(3);
chrono::ChCollisionSystemBullet::SetContactBreakingThreshold(2);

std::string shapes_dir = "DynoObj2_shapes/";

// Prepare some data for later use
std::shared_ptr<chrono::ChVisualShapeModelFile> body_shape;
chrono::ChMatrix33<> mr;
std::shared_ptr<chrono::ChLinkBase> link;
chrono::ChVector3d cA;
chrono::ChVector3d cB;
chrono::ChVector3d dA;
chrono::ChVector3d dB;

// Assembly ground body
auto body_0 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_0->SetName("SLDW_GROUND");
body_0->SetFixed(true);
bodylist.push_back(body_0);

// Rigid body part
auto body_1 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_1->SetName("GearC-1");
body_1->SetPos(chrono::ChVector3d(-213.749821851277,320.571341649174,280.873424974888));
body_1->SetRot(chrono::ChQuaternion<>(-0.0817797748423181,-0.0817797748423181,0.702361778876627,-0.702361778876627));
body_1->SetMass(0.00573763521549478);
body_1->SetInertiaXX(chrono::ChVector3d(0.634526836838321,1.15329562580959,0.603909758681937));
body_1->SetInertiaXY(chrono::ChVector3d(-0.129694217418744,1.23298291883905e-16,3.23260450253029e-17));
body_1->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(2.13036378266841e-15,2.61858173847454e-15,2.5),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_1);



// Rigid body part
auto body_2 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_2->SetName("GearF-1");
body_2->SetPos(chrono::ChVector3d(-153.681408502864,-61.4286583508263,257.256405065421));
body_2->SetRot(chrono::ChQuaternion<>(0.649830826959365,0.649830826959365,-0.278782883860017,0.278782883860017));
body_2->SetMass(0.0501455152739229);
body_2->SetInertiaXX(chrono::ChVector3d(11.2789699070089,11.5952382532535,14.5855497562076));
body_2->SetInertiaXY(chrono::ChVector3d(-3.14447193635037,2.5699341386905e-16,1.95598044948725e-16));
body_2->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-3.46493046987738e-15,3.82266315121593e-17,-39.8541013999262),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_2_1.obj");
body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_2);



// Rigid body part
auto body_3 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_3->SetName("GearA-2");
body_3->SetPos(chrono::ChVector3d(-153.681408502864,315.971341649174,257.256405065421));
body_3->SetRot(chrono::ChQuaternion<>(0.568770873369917,-0.568770873369916,-0.420118666100451,-0.420118666100451));
body_3->SetMass(0.0477640623787958);
body_3->SetInertiaXX(chrono::ChVector3d(7.21167604690774,16.228522846043,17.1708050876363));
body_3->SetInertiaXY(chrono::ChVector3d(3.06338218915213,-5.81739616209904e-15,1.76886639717352e-15));
body_3->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-1.92253939340124e-17,1.03367134939672e-16,-44.1449180223437),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_3_1.obj");
body_3->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_3);



// Rigid body part
auto body_4 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_4->SetName("GearB-1");
body_4->SetPos(chrono::ChVector3d(-183.793091163963,320.571341649174,254.366929798653));
body_4->SetRot(chrono::ChQuaternion<>(0.590890270601115,0.590890270601115,0.388392440849382,-0.388392440849382));
body_4->SetMass(0.00573763521549478);
body_4->SetInertiaXX(chrono::ChVector3d(1.09268072169601,0.695141740951902,0.603909758681937));
body_4->SetInertiaXY(chrono::ChVector3d(-0.211167099311834,-4.76994403450505e-17,-1.18304851362856e-16));
body_4->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(2.53576249071668e-15,2.9109981046097e-15,2.5),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_4_1.obj");
body_4->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_4);



// Rigid body part
auto body_5 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_5->SetName("GearD-1");
body_5->SetPos(chrono::ChVector3d(-213.749821851277,-65.4286583508263,280.873424974888));
body_5->SetRot(chrono::ChQuaternion<>(0.707106781186548,-0.707106781186547,-8.1335083852247e-17,0));
body_5->SetMass(0.00573763521549478);
body_5->SetInertiaXX(chrono::ChVector3d(0.603909758681937,1.18391270396598,0.603909758681937));
body_5->SetInertiaXY(chrono::ChVector3d(-6.68097235284266e-17,1.28759415537667e-16,-3.13275101861351e-18));
body_5->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(2.28311378076172e-15,2.62662036785091e-15,2.5),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_5_1.obj");
body_5->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_5);



// Rigid body part
auto body_6 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_6->SetName("Part1-1");
body_6->SetPos(chrono::ChVector3d(-153.681408502864,243.071341649174,257.256405065421));
body_6->SetRot(chrono::ChQuaternion<>(0.132211074481703,-0.132211074481703,0.694636762476903,0.694636762476903));
body_6->SetMass(0.0161771922143864);
body_6->SetInertiaXX(chrono::ChVector3d(8.94033623042991,4.92384567286665,9.78576029322433));
body_6->SetInertiaXY(chrono::ChVector3d(1.72367568060765,0.035353144802671,-0.0929757122109471));
body_6->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.137806764960663,-0.106884958735738,27.7658735013664),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_6_1.obj");
body_6->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_6);



// Rigid body part
auto body_7 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_7->SetName("GearE-1");
body_7->SetPos(chrono::ChVector3d(-183.793091163963,-65.4286583508263,254.366929798653));
body_7->SetRot(chrono::ChQuaternion<>(-0.227257494306092,0.227257494306092,0.669592436696918,0.669592436696918));
body_7->SetMass(0.00573763521549478);
body_7->SetInertiaXX(chrono::ChVector3d(0.81879554426816,0.969026918379756,0.603909758681937));
body_7->SetInertiaXY(chrono::ChVector3d(0.280104422836692,-6.58738933254936e-18,2.00763111792369e-16));
body_7->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(2.41869881321172e-15,2.47441422320288e-15,2.5),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_7_1.obj");
body_7->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_7);

// dim  -> in mm
// mass -> in kg
// inert-> in kg.mm^2

// Rigid body part
auto body_8 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_8->SetName("Part2_flywheel-1");
body_8->SetPos(chrono::ChVector3d(-213.749821851277,127.271341649174,280.873424974888));
body_8->SetRot(chrono::ChQuaternion<>(0.661716586109785,0.661716586109785,-0.249261227765594,0.249261227765594));
body_8->SetMass(2.7593573274781);
body_8->SetInertiaXX(chrono::ChVector3d(16827.443821127,13264.5805396432,28809.6343767049));
body_8->SetInertiaXY(chrono::ChVector3d(-13647.8495475438,1.73940788712187e-13,1.82312144815032e-13));
body_8->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(1.6770496422426e-15,-1.6193758085084e-15,-1.31044872236957e-14),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_8_1.obj");
body_8->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_8);



// Rigid body part
auto body_9 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_9->SetName("frame-1");
body_9->SetPos(chrono::ChVector3d(10.0501781487224,127.271341649174,219.573424974887));
body_9->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_9->SetMass(1.54162568183963);
body_9->SetInertiaXX(chrono::ChVector3d(29568.9201124322,45865.3318996888,73541.0736970342));
body_9->SetInertiaXY(chrono::ChVector3d(-0.0138791051507927,-1731.0556183721,-0.0228813539244026));
body_9->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(37.437693135653,-0.000143977620512136,19.8392235533775),chrono::ChQuaternion<>(1,0,0,0)));
body_9->SetFixed(true);

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_9_1.obj");
body_9->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_9);



// Rigid body part
auto body_10 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_10->SetName("Part2_dyno-1");
body_10->SetPos(chrono::ChVector3d(-153.681408502864,4.47134164917363,257.256405065421));
body_10->SetRot(chrono::ChQuaternion<>(0.699115859811739,0.699115859811739,0.106004785550905,-0.106004785550905));
body_10->SetMass(0.0234167508805127);
body_10->SetInertiaXX(chrono::ChVector3d(12.1095503099806,7.23515323489345,12.4345268420207));
body_10->SetInertiaXY(chrono::ChVector3d(1.69907122028445,0.084707339713336,-0.238004906902844));
body_10->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.0233854187713135,0.451628932812181,17.3769964334117),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_10_1.obj");
body_10->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_10);




// Mate constraint: Concentric1 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_9 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_3 , SW name: GearA-2 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-153.681408502864,311.571341649174,257.256405065421);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
cB = chrono::ChVector3d(-153.681408502864,297.971341649174,257.256405065421);
dB = chrono::ChVector3d(-1.03808692663488e-15,1,5.38820602242433e-16);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_9,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric1");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-153.681408502864,311.571341649174,257.256405065421);
cB = chrono::ChVector3d(-153.681408502864,297.971341649174,257.256405065421);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
dB = chrono::ChVector3d(-1.03808692663488e-15,1,5.38820602242433e-16);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_9,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric1");
linklist.push_back(link);


// Mate constraint: Concentric3 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_9 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_1 , SW name: GearC-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-213.749821851277,311.571341649174,280.873424974888);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
cB = chrono::ChVector3d(-213.749821851277,315.571341649174,280.873424974888);
dB = chrono::ChVector3d(1.150251786806e-16,1,-1.97167559868079e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_9,body_1,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric3");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-213.749821851277,311.571341649174,280.873424974888);
cB = chrono::ChVector3d(-213.749821851277,315.571341649174,280.873424974888);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
dB = chrono::ChVector3d(1.150251786806e-16,1,-1.97167559868079e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_9,body_1,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric3");
linklist.push_back(link);


// Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_9 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_5 , SW name: GearD-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-213.749821851277,-57.0286583508263,280.873424974888);
dA = chrono::ChVector3d(-1.15025178680601e-16,1,0);
cB = chrono::ChVector3d(-213.749821851277,-60.4286583508263,280.873424974888);
dB = chrono::ChVector3d(1.15025178680601e-16,-1,0);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_9,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric4");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-213.749821851277,-57.0286583508263,280.873424974888);
cB = chrono::ChVector3d(-213.749821851277,-60.4286583508263,280.873424974888);
dA = chrono::ChVector3d(-1.15025178680601e-16,1,0);
dB = chrono::ChVector3d(1.15025178680601e-16,-1,0);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_9,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric4");
linklist.push_back(link);


// Mate constraint: Concentric5 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_9 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_2 , SW name: GearF-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-153.681408502864,-57.0286583508263,257.256405065421);
dA = chrono::ChVector3d(-1.15025178680601e-16,1,0);
cB = chrono::ChVector3d(-153.681408502864,-21.4286583508263,257.256405065421);
dB = chrono::ChVector3d(9.98048005582594e-17,-1,-4.44467336474743e-17);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_9,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric5");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-153.681408502864,-57.0286583508263,257.256405065421);
cB = chrono::ChVector3d(-153.681408502864,-21.4286583508263,257.256405065421);
dA = chrono::ChVector3d(-1.15025178680601e-16,1,0);
dB = chrono::ChVector3d(9.98048005582594e-17,-1,-4.44467336474743e-17);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_9,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric5");
linklist.push_back(link);


// Mate constraint: Coincident9 [MateCoincident] type:0 align:0 flip:False
//   Entity 0: C::E name: body_5 , SW name: GearD-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_2 , SW name: GearF-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-213.749821851277,-65.4286583508263,280.873424974888);
cB = chrono::ChVector3d(-153.681408502864,-65.4286583508263,257.256405065421);
dA = chrono::ChVector3d(1.15025178680601e-16,-1,0);
dB = chrono::ChVector3d(9.98048005582594e-17,-1,-4.44467336474743e-17);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_5,body_2,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident9");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-213.749821851277,-65.4286583508263,280.873424974888);
dA = chrono::ChVector3d(1.15025178680601e-16,-1,0);
cB = chrono::ChVector3d(-153.681408502864,-65.4286583508263,257.256405065421);
dB = chrono::ChVector3d(9.98048005582594e-17,-1,-4.44467336474743e-17);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_5,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident9");
linklist.push_back(link);


// Mate constraint: Coincident11 [MateCoincident] type:0 align:0 flip:False
//   Entity 0: C::E name: body_4 , SW name: GearB-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_1 , SW name: GearC-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-183.793091163963,320.571341649174,254.366929798653);
cB = chrono::ChVector3d(-213.749821851277,320.571341649174,280.873424974888);
dA = chrono::ChVector3d(1.150251786806e-16,1,1.63318859284038e-30);
dB = chrono::ChVector3d(1.150251786806e-16,1,-1.97167559868079e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_4,body_1,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident11");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-183.793091163963,320.571341649174,254.366929798653);
dA = chrono::ChVector3d(1.150251786806e-16,1,1.63318859284038e-30);
cB = chrono::ChVector3d(-213.749821851277,320.571341649174,280.873424974888);
dB = chrono::ChVector3d(1.150251786806e-16,1,-1.97167559868079e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_1,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident11");
linklist.push_back(link);


// Mate constraint: Coincident13 [MateCoincident] type:0 align:0 flip:False
//   Entity 0: C::E name: body_5 , SW name: GearD-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_7 , SW name: GearE-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-213.749821851277,-65.4286583508263,280.873424974888);
cB = chrono::ChVector3d(-183.793091163963,-65.4286583508263,254.366929798653);
dA = chrono::ChVector3d(1.15025178680601e-16,-1,0);
dB = chrono::ChVector3d(4.69262190912459e-16,-1,-2.62319926058665e-16);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_5,body_7,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident13");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-213.749821851277,-65.4286583508263,280.873424974888);
dA = chrono::ChVector3d(1.15025178680601e-16,-1,0);
cB = chrono::ChVector3d(-183.793091163963,-65.4286583508263,254.366929798653);
dB = chrono::ChVector3d(4.69262190912459e-16,-1,-2.62319926058665e-16);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_5,body_7,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident13");
linklist.push_back(link);


// Mate constraint: Concentric16 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_9 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_8 , SW name: Part2_flywheel-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-213.749821851277,311.571341649174,280.873424974888);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
cB = chrono::ChVector3d(-213.749821851278,-47.7286583508263,280.873424974888);
dB = chrono::ChVector3d(1.150251786806e-16,1,-7.53206554000315e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_9,body_8,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric16");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-213.749821851277,311.571341649174,280.873424974888);
cB = chrono::ChVector3d(-213.749821851278,-47.7286583508263,280.873424974888);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
dB = chrono::ChVector3d(1.150251786806e-16,1,-7.53206554000315e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_9,body_8,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric16");
linklist.push_back(link);


// Mate constraint: Width1 [MateWidth] type:11 align:1 flip:False
//   Entity 0: C::E name: body_8 , SW name: Part2_flywheel-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_8 , SW name: Part2_flywheel-1 ,  SW ref.type:2 (2)
//   Entity 2: C::E name: body_9 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 3: C::E name: body_9 , SW name: frame-1 ,  SW ref.type:2 (2)

// Mate constraint: Concentric18 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_9 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_7 , SW name: GearE-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-183.793091163963,-57.0286583508263,254.366929798653);
dA = chrono::ChVector3d(-1.15025178680601e-16,1,0);
cB = chrono::ChVector3d(-183.793091163963,-60.4286583508263,254.366929798653);
dB = chrono::ChVector3d(4.69262190912459e-16,-1,-2.62319926058665e-16);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_9,body_7,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric18");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-183.793091163963,-57.0286583508263,254.366929798653);
cB = chrono::ChVector3d(-183.793091163963,-60.4286583508263,254.366929798653);
dA = chrono::ChVector3d(-1.15025178680601e-16,1,0);
dB = chrono::ChVector3d(4.69262190912459e-16,-1,-2.62319926058665e-16);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_9,body_7,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric18");
linklist.push_back(link);


// Mate constraint: Concentric19 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_9 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_4 , SW name: GearB-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-183.793091163963,311.571341649174,254.366929798653);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
cB = chrono::ChVector3d(-183.793091163963,315.571341649174,254.366929798653);
dB = chrono::ChVector3d(1.150251786806e-16,1,1.63318859284038e-30);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_9,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric19");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-183.793091163963,311.571341649174,254.366929798653);
cB = chrono::ChVector3d(-183.793091163963,315.571341649174,254.366929798653);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
dB = chrono::ChVector3d(1.150251786806e-16,1,1.63318859284038e-30);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_9,body_4,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric19");
linklist.push_back(link);


// Mate constraint: Concentric20 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_9 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_10 , SW name: Part2_dyno-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-153.681408502864,-57.0286583508263,257.256405065421);
dA = chrono::ChVector3d(-1.15025178680601e-16,1,0);
cB = chrono::ChVector3d(-153.681408502864,-50.5286583508264,257.256405065421);
dB = chrono::ChVector3d(-1.15025274152835e-16,1,-1.45428219820965e-23);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_9,body_10,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric20");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-153.681408502864,-57.0286583508263,257.256405065421);
cB = chrono::ChVector3d(-153.681408502864,-50.5286583508264,257.256405065421);
dA = chrono::ChVector3d(-1.15025178680601e-16,1,0);
dB = chrono::ChVector3d(-1.15025274152835e-16,1,-1.45428219820965e-23);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_9,body_10,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric20");
linklist.push_back(link);


// Mate constraint: Concentric21 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_9 , SW name: frame-1 ,  SW ref.type:1 (1)
//   Entity 1: C::E name: body_6 , SW name: Part1-1 ,  SW ref.type:1 (1)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-153.681408502864,305.071341649174,257.256405065421);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
cB = chrono::ChVector3d(-153.681408502864,305.071341649174,257.256405065421);
dB = chrono::ChVector3d(1.15025178680596e-16,1,1.21121444541331e-30);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_9,body_6,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric21");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-153.681408502864,305.071341649174,257.256405065421);
cB = chrono::ChVector3d(-153.681408502864,305.071341649174,257.256405065421);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
dB = chrono::ChVector3d(1.15025178680596e-16,1,1.21121444541331e-30);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_9,body_6,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric21");
linklist.push_back(link);


// Mate constraint: Coincident18 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_9 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_6 , SW name: Part1-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-213.749821851277,305.071341649174,280.873424974888);
cB = chrono::ChVector3d(-153.681408502864,305.071341649174,257.256405065421);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
dB = chrono::ChVector3d(1.15025178680596e-16,1,1.21121444541331e-30);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_9,body_6,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident18");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-213.749821851277,305.071341649174,280.873424974888);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
cB = chrono::ChVector3d(-153.681408502864,305.071341649174,257.256405065421);
dB = chrono::ChVector3d(1.15025178680596e-16,1,1.21121444541331e-30);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_9,body_6,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident18");
linklist.push_back(link);


// Mate constraint: Coincident19 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_9 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_10 , SW name: Part2_dyno-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-213.749821851277,-50.5286583508263,280.873424974888);
cB = chrono::ChVector3d(-153.681408502864,-50.5286583508264,257.256405065421);
dA = chrono::ChVector3d(-1.15025178680601e-16,1,0);
dB = chrono::ChVector3d(1.15025274152835e-16,-1,1.45428219820965e-23);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_9,body_10,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident19");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-213.749821851277,-50.5286583508263,280.873424974888);
dA = chrono::ChVector3d(-1.15025178680601e-16,1,0);
cB = chrono::ChVector3d(-153.681408502864,-50.5286583508264,257.256405065421);
dB = chrono::ChVector3d(1.15025274152835e-16,-1,1.45428219820965e-23);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_9,body_10,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident19");
linklist.push_back(link);


// Mate constraint: Distance1 [MateDistanceDim] type:5 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: GearB-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_9 , SW name: frame-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-183.793091163963,315.571341649174,254.366929798653);
cB = chrono::ChVector3d(-213.749821851277,311.571341649174,280.873424974888);
dA = chrono::ChVector3d(-1.150251786806e-16,-1,-1.63318859284038e-30);
dB = chrono::ChVector3d(1.15025178680601e-16,1,0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_4,body_9,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(4);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Distance1");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-183.793091163963,315.571341649174,254.366929798653);
dA = chrono::ChVector3d(-1.150251786806e-16,-1,-1.63318859284038e-30);
cB = chrono::ChVector3d(-213.749821851277,311.571341649174,280.873424974888);
dB = chrono::ChVector3d(1.15025178680601e-16,1,0);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_9,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Distance1");
linklist.push_back(link);


// Mate constraint: Width2 [MateWidth] type:11 align:0 flip:False
//   Entity 0: C::E name: body_3 , SW name: GearA-2 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_3 , SW name: GearA-2 ,  SW ref.type:2 (2)
//   Entity 2: C::E name: body_6 , SW name: Part1-1 ,  SW ref.type:2 (2)
//   Entity 3: C::E name: body_6 , SW name: Part1-1 ,  SW ref.type:2 (2)

// Mate constraint: Width3 [MateWidth] type:11 align:1 flip:False
//   Entity 0: C::E name: body_2 , SW name: GearF-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_2 , SW name: GearF-1 ,  SW ref.type:2 (2)
//   Entity 2: C::E name: body_10 , SW name: Part2_dyno-1 ,  SW ref.type:2 (2)
//   Entity 3: C::E name: body_10 , SW name: Part2_dyno-1 ,  SW ref.type:2 (2)


} // end function
