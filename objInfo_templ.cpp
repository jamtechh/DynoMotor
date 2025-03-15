// C++ Chrono::Engine model automatically generated using Chrono::SolidWorks add-in
// Assembly: C:\Users\Jamiul\OneDrive - UW-Madison\Design\chronoMotor\Assem1.SLDASM


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
#include "dynoCPP.h"


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

std::string shapes_dir = "dynoCPP_shapes/";

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
body_1->SetName("Part1-1");
body_1->SetPos(chrono::ChVector3d(-153.681408502864,232.071341649174,257.256405065421));
body_1->SetRot(chrono::ChQuaternion<>(0.344860417986101,-0.344860417986101,0.617309721376921,0.617309721376921));
body_1->SetMass(0.016029213366013);
body_1->SetInertiaXX(chrono::ChVector3d(5.77976073009449,7.96307088073259,9.73060284283223));
body_1->SetInertiaXY(chrono::ChVector3d(2.41692490861642,0.0837790436764728,-0.0530757027275727));
body_1->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.139083582689284,-0.107870586674347,27.5964334240644),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_1_1.obj");
body_1->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_1);



// Rigid body part
auto body_2 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_2->SetName("GearA-2");
body_2->SetPos(chrono::ChVector3d(-153.681408502864,316.571341649174,257.256405065421));
body_2->SetRot(chrono::ChQuaternion<>(0.69219181681608,-0.69219181681608,0.144466220040721,0.144466220040721));
body_2->SetMass(0.000870269104083074);
body_2->SetInertiaXX(chrono::ChVector3d(0.0245232834116315,0.0348513174025659,0.0220932767457664));
body_2->SetInertiaXY(chrono::ChVector3d(0.00556795508597199,-1.39917117738131e-17,9.19843766249956e-18));
body_2->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-1.96368346622425e-15,2.76276818372194e-15,0.394433254710844),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_2_1.obj");
body_2->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_2);



// Rigid body part
auto body_3 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_3->SetName("GearF-1");
body_3->SetPos(chrono::ChVector3d(-153.681408502864,-62.0286583508263,257.256405065421));
body_3->SetRot(chrono::ChQuaternion<>(0.682163121105785,0.682163121105785,0.186154441803611,-0.186154441803611));
body_3->SetMass(0.000870269104083074);
body_3->SetInertiaXX(chrono::ChVector3d(0.0260120055120133,0.0333625953021842,0.0220932767457664));
body_3->SetInertiaXY(chrono::ChVector3d(-0.00664540463802123,1.36335435030819e-17,1.15565959252578e-17));
body_3->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(-2.02308898578366e-15,2.67365990438282e-15,0.394433254710843),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_3_1.obj");
body_3->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_3);



// Rigid body part
auto body_4 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_4->SetName("frame-1");
body_4->SetPos(chrono::ChVector3d(10.0501781487224,127.271341649174,219.573424974887));
body_4->SetRot(chrono::ChQuaternion<>(1,0,0,0));
body_4->SetMass(1.54162568183963);
body_4->SetInertiaXX(chrono::ChVector3d(29568.9201124321,45865.3318996888,73541.0736970342));
body_4->SetInertiaXY(chrono::ChVector3d(-0.0138791051526975,-1731.0556183721,-0.0228813539240712));
body_4->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(37.4376931356531,-0.000143977620523125,19.8392235533775),chrono::ChQuaternion<>(1,0,0,0)));
body_4->SetFixed(true);

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_4_1.obj");
body_4->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_4);



// Rigid body part
auto body_5 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_5->SetName("GearB-1");
body_5->SetPos(chrono::ChVector3d(-183.793091163963,320.571341649174,254.366929798653));
body_5->SetRot(chrono::ChQuaternion<>(0.590890270601115,0.590890270601115,0.388392440849382,-0.388392440849382));
body_5->SetMass(0.00573763521549478);
body_5->SetInertiaXX(chrono::ChVector3d(1.09268072169601,0.695141740951902,0.603909758681937));
body_5->SetInertiaXY(chrono::ChVector3d(-0.211167099311834,-4.86196921754609e-17,-1.20434881916057e-16));
body_5->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(2.4993650536692e-15,2.77437524831689e-15,2.5),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_5_1.obj");
body_5->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_5);



// Rigid body part
auto body_6 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_6->SetName("GearC-1");
body_6->SetPos(chrono::ChVector3d(-213.749821851278,320.571341649174,280.873424974888));
body_6->SetRot(chrono::ChQuaternion<>(-0.0817797748423181,-0.0817797748423181,0.702361778876627,-0.702361778876627));
body_6->SetMass(0.00573763521549478);
body_6->SetInertiaXX(chrono::ChVector3d(0.634526836838321,1.15329562580959,0.603909758681937));
body_6->SetInertiaXY(chrono::ChVector3d(-0.129694217418744,1.25647520447832e-16,3.28806303875533e-17));
body_6->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(2.59890820730129e-15,2.65462361729245e-15,2.5),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_6_1.obj");
body_6->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_6);



// Rigid body part
auto body_7 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_7->SetName("GearD-1");
body_7->SetPos(chrono::ChVector3d(-213.749821851277,-66.0286583508263,280.873424974888));
body_7->SetRot(chrono::ChQuaternion<>(0.707106781186548,-0.707106781186547,-8.1335083852247e-17,0));
body_7->SetMass(0.00573763521549478);
body_7->SetInertiaXX(chrono::ChVector3d(0.603909758681937,1.18391270396598,0.603909758681937));
body_7->SetInertiaXY(chrono::ChVector3d(-6.68097235284266e-17,1.28759415537667e-16,-3.13275101861351e-18));
body_7->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(2.27453129794006e-15,2.61858173847454e-15,2.5),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_7_1.obj");
body_7->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_7);



// Rigid body part
auto body_8 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_8->SetName("GearE-1");
body_8->SetPos(chrono::ChVector3d(-183.793091163963,-66.0286583508263,254.366929798653));
body_8->SetRot(chrono::ChQuaternion<>(-0.227257494306092,0.227257494306092,0.669592436696918,0.669592436696918));
body_8->SetMass(0.00573763521549478);
body_8->SetInertiaXX(chrono::ChVector3d(0.81879554426816,0.969026918379755,0.603909758681937));
body_8->SetInertiaXY(chrono::ChVector3d(0.280104422836692,-8.50253870869868e-18,2.02232343781072e-16));
body_8->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(2.4993650536692e-15,2.77437524831689e-15,2.5),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_8_1.obj");
body_8->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_8);



// Rigid body part
auto body_9 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_9->SetName("Part2_flywheel-1");
body_9->SetPos(chrono::ChVector3d(-213.749821851277,127.271341649174,280.873424974888));
body_9->SetRot(chrono::ChQuaternion<>(0.661716586109785,0.661716586109785,-0.249261227765594,0.249261227765594));
body_9->SetMass(1.02740423420996);
body_9->SetInertiaXX(chrono::ChVector3d(6609.88825669769,5209.36075890446,11319.9638141394));
body_9->SetInertiaXY(chrono::ChVector3d(-5364.83913451316,-0.00029120923437435,0.000265996941534389));
body_9->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(8.92766810034472e-07,8.92766809937858e-07,1.29505962482037e-14),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_9_1.obj");
body_9->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_9);



// Rigid body part
auto body_10 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
body_10->SetName("Part2_dyno-1");
body_10->SetPos(chrono::ChVector3d(-153.681408502864,15.4713416491736,257.256405065421));
body_10->SetRot(chrono::ChQuaternion<>(0.679959230544171,0.679959230544171,0.194050108986773,-0.194050108986773));
body_10->SetMass(0.0195543893524915);
body_10->SetInertiaXX(chrono::ChVector3d(8.64325141424593,6.23841160373604,10.0061631055424));
body_10->SetInertiaXY(chrono::ChVector3d(2.47769437441725,0.136114194517743,-0.20644974843194));
body_10->SetFrameCOMToRef(chrono::ChFramed(chrono::ChVector3d(0.0176453705510941,0.529772669645291,17.8045092646342),chrono::ChQuaternion<>(1,0,0,0)));

// Visualization shape
body_shape = chrono_types::make_shared<chrono::ChVisualShapeModelFile>();
body_shape->SetFilename(shapes_dir + "body_10_1.obj");
body_10->AddVisualShape(body_shape, chrono::ChFramed(chrono::ChVector3d(0,0,0), chrono::ChQuaternion<>(1,0,0,0)));

bodylist.push_back(body_10);




// Mate constraint: Coincident5 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_1 , SW name: Part1-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-213.749821851277,305.071341649174,280.873424974888);
cB = chrono::ChVector3d(-153.681408502864,305.071341649174,257.256405065421);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
dB = chrono::ChVector3d(1.150251786806e-16,1,-2.13289118002379e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_4,body_1,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident5");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-213.749821851277,305.071341649174,280.873424974888);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
cB = chrono::ChVector3d(-153.681408502864,305.071341649174,257.256405065421);
dB = chrono::ChVector3d(1.150251786806e-16,1,-2.13289118002379e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_1,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident5");
linklist.push_back(link);


// Mate constraint: Concentric1 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_2 , SW name: GearA-2 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-153.681408502864,311.571341649174,257.256405065421);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
cB = chrono::ChVector3d(-153.681408502864,311.571341649174,257.256405065421);
dB = chrono::ChVector3d(1.150251786806e-16,1,1.29422492262822e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric1");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-153.681408502864,311.571341649174,257.256405065421);
cB = chrono::ChVector3d(-153.681408502864,311.571341649174,257.256405065421);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
dB = chrono::ChVector3d(1.150251786806e-16,1,1.29422492262822e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_4,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric1");
linklist.push_back(link);


// Mate constraint: Concentric3 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_6 , SW name: GearC-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-213.749821851277,311.571341649174,280.873424974888);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
cB = chrono::ChVector3d(-213.749821851278,315.571341649174,280.873424974888);
dB = chrono::ChVector3d(1.150251786806e-16,1,-1.97167559868079e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_6,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric3");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-213.749821851277,311.571341649174,280.873424974888);
cB = chrono::ChVector3d(-213.749821851278,315.571341649174,280.873424974888);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
dB = chrono::ChVector3d(1.150251786806e-16,1,-1.97167559868079e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_4,body_6,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric3");
linklist.push_back(link);


// Mate constraint: Concentric4 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_7 , SW name: GearD-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-213.749821851277,-57.0286583508263,280.873424974888);
dA = chrono::ChVector3d(-1.15025178680601e-16,1,0);
cB = chrono::ChVector3d(-213.749821851277,-61.0286583508263,280.873424974888);
dB = chrono::ChVector3d(1.15025178680601e-16,-1,0);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_7,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric4");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-213.749821851277,-57.0286583508263,280.873424974888);
cB = chrono::ChVector3d(-213.749821851277,-61.0286583508263,280.873424974888);
dA = chrono::ChVector3d(-1.15025178680601e-16,1,0);
dB = chrono::ChVector3d(1.15025178680601e-16,-1,0);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_4,body_7,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric4");
linklist.push_back(link);


// Mate constraint: Concentric5 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_3 , SW name: GearF-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-153.681408502864,-57.0286583508263,257.256405065421);
dA = chrono::ChVector3d(-1.15025178680601e-16,1,0);
cB = chrono::ChVector3d(-153.681408502864,-57.0286583508263,257.256405065421);
dB = chrono::ChVector3d(1.150251786806e-16,-1,2.82315246648629e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric5");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-153.681408502864,-57.0286583508263,257.256405065421);
cB = chrono::ChVector3d(-153.681408502864,-57.0286583508263,257.256405065421);
dA = chrono::ChVector3d(-1.15025178680601e-16,1,0);
dB = chrono::ChVector3d(1.150251786806e-16,-1,2.82315246648629e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_4,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric5");
linklist.push_back(link);


// Mate constraint: Coincident6 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_3 , SW name: GearF-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-213.749821851277,-57.0286583508263,280.873424974888);
cB = chrono::ChVector3d(-153.681408502864,-57.0286583508263,257.256405065421);
dA = chrono::ChVector3d(1.15025178680601e-16,-1,0);
dB = chrono::ChVector3d(-1.150251786806e-16,1,-2.82315246648629e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_4,body_3,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident6");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-213.749821851277,-57.0286583508263,280.873424974888);
dA = chrono::ChVector3d(1.15025178680601e-16,-1,0);
cB = chrono::ChVector3d(-153.681408502864,-57.0286583508263,257.256405065421);
dB = chrono::ChVector3d(-1.150251786806e-16,1,-2.82315246648629e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident6");
linklist.push_back(link);


// Mate constraint: Coincident7 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_2 , SW name: GearA-2 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-213.749821851277,311.571341649174,280.873424974888);
cB = chrono::ChVector3d(-153.681408502864,311.571341649174,257.256405065421);
dA = chrono::ChVector3d(1.15025178680601e-16,1,0);
dB = chrono::ChVector3d(-1.150251786806e-16,-1,-1.29422492262822e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_4,body_2,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident7");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-213.749821851277,311.571341649174,280.873424974888);
dA = chrono::ChVector3d(1.15025178680601e-16,1,0);
cB = chrono::ChVector3d(-153.681408502864,311.571341649174,257.256405065421);
dB = chrono::ChVector3d(-1.150251786806e-16,-1,-1.29422492262822e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_2,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident7");
linklist.push_back(link);


// Mate constraint: Coincident8 [MateCoincident] type:0 align:0 flip:False
//   Entity 0: C::E name: body_2 , SW name: GearA-2 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_6 , SW name: GearC-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-153.681408502864,320.571341649174,257.256405065421);
cB = chrono::ChVector3d(-213.749821851278,320.571341649174,280.873424974888);
dA = chrono::ChVector3d(1.150251786806e-16,1,1.29422492262822e-31);
dB = chrono::ChVector3d(1.150251786806e-16,1,-1.97167559868079e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_2,body_6,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident8");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-153.681408502864,320.571341649174,257.256405065421);
dA = chrono::ChVector3d(1.150251786806e-16,1,1.29422492262822e-31);
cB = chrono::ChVector3d(-213.749821851278,320.571341649174,280.873424974888);
dB = chrono::ChVector3d(1.150251786806e-16,1,-1.97167559868079e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_2,body_6,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident8");
linklist.push_back(link);


// Mate constraint: Coincident9 [MateCoincident] type:0 align:0 flip:False
//   Entity 0: C::E name: body_7 , SW name: GearD-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_3 , SW name: GearF-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-213.749821851277,-66.0286583508263,280.873424974888);
cB = chrono::ChVector3d(-153.681408502864,-66.0286583508263,257.256405065421);
dA = chrono::ChVector3d(1.15025178680601e-16,-1,0);
dB = chrono::ChVector3d(1.150251786806e-16,-1,2.82315246648629e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_7,body_3,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident9");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-213.749821851277,-66.0286583508263,280.873424974888);
dA = chrono::ChVector3d(1.15025178680601e-16,-1,0);
cB = chrono::ChVector3d(-153.681408502864,-66.0286583508263,257.256405065421);
dB = chrono::ChVector3d(1.150251786806e-16,-1,2.82315246648629e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_7,body_3,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident9");
linklist.push_back(link);


// Mate constraint: Coincident11 [MateCoincident] type:0 align:0 flip:False
//   Entity 0: C::E name: body_5 , SW name: GearB-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_6 , SW name: GearC-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-183.793091163963,320.571341649174,254.366929798653);
cB = chrono::ChVector3d(-213.749821851278,320.571341649174,280.873424974888);
dA = chrono::ChVector3d(1.150251786806e-16,1,1.63318859284038e-30);
dB = chrono::ChVector3d(1.150251786806e-16,1,-1.97167559868079e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_5,body_6,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident11");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-183.793091163963,320.571341649174,254.366929798653);
dA = chrono::ChVector3d(1.150251786806e-16,1,1.63318859284038e-30);
cB = chrono::ChVector3d(-213.749821851278,320.571341649174,280.873424974888);
dB = chrono::ChVector3d(1.150251786806e-16,1,-1.97167559868079e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_5,body_6,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident11");
linklist.push_back(link);


// Mate constraint: Coincident13 [MateCoincident] type:0 align:0 flip:False
//   Entity 0: C::E name: body_7 , SW name: GearD-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_8 , SW name: GearE-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-213.749821851277,-66.0286583508263,280.873424974888);
cB = chrono::ChVector3d(-183.793091163963,-66.0286583508263,254.366929798653);
dA = chrono::ChVector3d(1.15025178680601e-16,-1,0);
dB = chrono::ChVector3d(4.69262190912459e-16,-1,-2.62319926058665e-16);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_7,body_8,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident13");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-213.749821851277,-66.0286583508263,280.873424974888);
dA = chrono::ChVector3d(1.15025178680601e-16,-1,0);
cB = chrono::ChVector3d(-183.793091163963,-66.0286583508263,254.366929798653);
dB = chrono::ChVector3d(4.69262190912459e-16,-1,-2.62319926058665e-16);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_7,body_8,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident13");
linklist.push_back(link);


// Mate constraint: Concentric16 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_9 , SW name: Part2_flywheel-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-213.749821851277,311.571341649174,280.873424974888);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
cB = chrono::ChVector3d(-213.749821851278,-47.7286583508263,280.873424974888);
dB = chrono::ChVector3d(1.150251786806e-16,1,-7.53206554000315e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_9,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric16");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-213.749821851277,311.571341649174,280.873424974888);
cB = chrono::ChVector3d(-213.749821851278,-47.7286583508263,280.873424974888);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
dB = chrono::ChVector3d(1.150251786806e-16,1,-7.53206554000315e-31);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_4,body_9,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric16");
linklist.push_back(link);


// Mate constraint: Width1 [MateWidth] type:11 align:1 flip:False
//   Entity 0: C::E name: body_9 , SW name: Part2_flywheel-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_9 , SW name: Part2_flywheel-1 ,  SW ref.type:2 (2)
//   Entity 2: C::E name: body_4 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 3: C::E name: body_4 , SW name: frame-1 ,  SW ref.type:2 (2)

// Mate constraint: Concentric18 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_8 , SW name: GearE-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-183.793091163963,-57.0286583508263,254.366929798653);
dA = chrono::ChVector3d(-1.15025178680601e-16,1,0);
cB = chrono::ChVector3d(-183.793091163963,-61.0286583508263,254.366929798653);
dB = chrono::ChVector3d(4.69262190912459e-16,-1,-2.62319926058665e-16);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_8,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric18");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-183.793091163963,-57.0286583508263,254.366929798653);
cB = chrono::ChVector3d(-183.793091163963,-61.0286583508263,254.366929798653);
dA = chrono::ChVector3d(-1.15025178680601e-16,1,0);
dB = chrono::ChVector3d(4.69262190912459e-16,-1,-2.62319926058665e-16);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_4,body_8,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric18");
linklist.push_back(link);


// Mate constraint: Concentric19 [MateConcentric] type:1 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_5 , SW name: GearB-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-183.793091163963,311.571341649174,254.366929798653);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
cB = chrono::ChVector3d(-183.793091163963,315.571341649174,254.366929798653);
dB = chrono::ChVector3d(1.150251786806e-16,1,1.63318859284038e-30);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric19");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-183.793091163963,311.571341649174,254.366929798653);
cB = chrono::ChVector3d(-183.793091163963,315.571341649174,254.366929798653);
dA = chrono::ChVector3d(-1.15025178680601e-16,-1,0);
dB = chrono::ChVector3d(1.150251786806e-16,1,1.63318859284038e-30);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_4,body_5,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric19");
linklist.push_back(link);


// Mate constraint: Concentric20 [MateConcentric] type:1 align:0 flip:False
//   Entity 0: C::E name: body_4 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_10 , SW name: Part2_dyno-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-153.681408502864,-57.0286583508263,257.256405065421);
dA = chrono::ChVector3d(-1.15025178680601e-16,1,0);
cB = chrono::ChVector3d(-153.681408502864,-39.5286583508263,257.256405065421);
dB = chrono::ChVector3d(-1.15025274152838e-16,1,-1.45428225327743e-23);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_10,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Concentric20");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(true, true, false, false, false, false);
cA = chrono::ChVector3d(-153.681408502864,-57.0286583508263,257.256405065421);
cB = chrono::ChVector3d(-153.681408502864,-39.5286583508263,257.256405065421);
dA = chrono::ChVector3d(-1.15025178680601e-16,1,0);
dB = chrono::ChVector3d(-1.15025274152838e-16,1,-1.45428225327743e-23);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(body_4,body_10,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName("Concentric20");
linklist.push_back(link);


// Mate constraint: Coincident17 [MateCoincident] type:0 align:1 flip:False
//   Entity 0: C::E name: body_4 , SW name: frame-1 ,  SW ref.type:2 (2)
//   Entity 1: C::E name: body_10 , SW name: Part2_dyno-1 ,  SW ref.type:2 (2)
link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
cA = chrono::ChVector3d(-213.749821851277,-50.5286583508263,280.873424974888);
cB = chrono::ChVector3d(-153.681408502864,-50.5286583508263,257.256405065421);
dA = chrono::ChVector3d(-1.15025178680601e-16,1,0);
dB = chrono::ChVector3d(1.15025274152838e-16,-1,1.45428225327743e-23);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(body_4,body_10,false,cA,cB,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(0);
std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName("Coincident17");
linklist.push_back(link);

link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
cA = chrono::ChVector3d(-213.749821851277,-50.5286583508263,280.873424974888);
dA = chrono::ChVector3d(-1.15025178680601e-16,1,0);
cB = chrono::ChVector3d(-153.681408502864,-50.5286583508263,257.256405065421);
dB = chrono::ChVector3d(1.15025274152838e-16,-1,1.45428225327743e-23);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(true);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(body_4,body_10,false,cA,cB,dA,dB);
std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName("Coincident17");
linklist.push_back(link);



} // end function
