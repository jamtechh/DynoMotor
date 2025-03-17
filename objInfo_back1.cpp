#include "objInfo1.h"
std::vector<std::string> file_locations = {"" , // empty value to start with index 1
    "body_1_1",     // Part1_Motor
    "body_2_1",     // GearA_Driver_Motor
    "body_3_1",     // GearF_Driven_Dyno
    "body_4_1",     // frame
    "body_5_1",     // GearB
    "body_6_1",     // GearC
    "body_7_1",     // GearD
    "body_8_1",     // GearE
    "body_9_1",     // Part2_flywheel
    "body_10_1"     // Part2_dyno
};

std::vector<std::string> file_names = {"" , // empty value to start with index 1
    "Part1_Motor",     
    "GearA_Dr",     
    "GearF_Drvn",     
    "frame",     
    "GearB",     
    "GearC",     
    "GearD",     
    "GearE",     
    "Part2_flywheel",     
    "Part2_dyno1"     
};

std::vector<ChVector3d> positions = {ChVector3d(0,0,0),
    ChVector3d(-153.681408502864,232.071341649174,257.256405065421),
    ChVector3d(-153.681408502864,320,257.256405065421),
    ChVector3d(-153.681408502864,-66,257.256405065421),
    ChVector3d(10.0501781487224,127.271341649174,219.573424974887),
    ChVector3d(-183.793091163963,320,254.366929798653),
    ChVector3d(-213.749821851278,320,280.873424974888),
    ChVector3d(-213.749821851277,-66,280.873424974888),
    ChVector3d(-183.793091163963,-66,254.366929798653),
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

std::vector<ChVector3d> inertiaXX = {ChVector3d(0,0,0),
    ChVector3d(5.77976073009449,7.96307088073259,9.73060284283223),
    ChVector3d(.0245232834116315,.0348513174025659,.0220932767457664),
    ChVector3d(0.0260120055120133,0.0333625953021842,0.0220932767457664),
    ChVector3d(29568.9201124321,45865.3318996888,73541.0736970342),
    ChVector3d(1.09268072169601,0.695141740951902,0.603909758681937),
    ChVector3d(0.634526836838321,1.15329562580959,0.603909758681937),
    ChVector3d(0.603909758681937,1.18391270396598,0.603909758681937),
    ChVector3d(0.81879554426816,0.969026918379755,0.603909758681937),
    ChVector3d(6609.88825669769,5209.36075890446,11319.9638141394),
    ChVector3d(8.64325141424593,6.23841160373604,10.0061631055424)
};

std::vector<double> mass = {0.0,
    0.016029213366013,
    0.000870269104083074,
    0.000870269104083074,
    1.54162568183963,
    0.00573763521549478,
    0.00573763521549478,
    0.00573763521549478,
    0.00573763521549478,
    100.02740423420996,
    0.0195543893524915
};


std::vector<std::tuple<std::string, ChVector3d, ChQuaternion<>, ChVector3d, double, std::string>> objData = {
    {file_locations[0], positions[0], rotss[0], inertiaXX[0], mass[0], file_names[0]},
    {file_locations[1], positions[1], rotss[1], inertiaXX[1], mass[1], file_names[1]},
    {file_locations[2], positions[2], rotss[2], inertiaXX[2], mass[2], file_names[2]},
    {file_locations[3], positions[3], rotss[3], inertiaXX[3], mass[3], file_names[3]},
    {file_locations[4], positions[4], rotss[4], inertiaXX[4], mass[4], file_names[4]},
    {file_locations[5], positions[5], rotss[5], inertiaXX[5], mass[5], file_names[5]},
    {file_locations[6], positions[6], rotss[6], inertiaXX[6], mass[6], file_names[6]},
    {file_locations[7], positions[7], rotss[7], inertiaXX[7], mass[7], file_names[7]},
    {file_locations[8], positions[8], rotss[8], inertiaXX[8], mass[8], file_names[8]},
    {file_locations[9], positions[9], rotss[9], inertiaXX[9], mass[9], file_names[9]},
    {file_locations[10], positions[10], rotss[10], inertiaXX[10], mass[10], file_names[10]}
};

ChVector3d posOffset =  positions[4];