#include "objInfo1.h"
std::vector<ChVector3d> positions = {ChVector3d(0,0,0),
    ChVector3d(-213.749821851277,320.571341649174,280.873424974888),
    ChVector3d(-153.681408502864,-61.4286583508263,257.256405065421),
    ChVector3d(-153.681408502864,315.971341649174,257.256405065421),
    ChVector3d(-183.793091163963,320.571341649174,254.366929798653),
    ChVector3d(-213.749821851277,-65.4286583508263,280.873424974888),
    ChVector3d(-153.681408502864,243.071341649174,257.256405065421),
    ChVector3d(-183.793091163963,-65.4286583508263,254.366929798653),
    ChVector3d(-213.749821851277,127.271341649174,280.873424974888),
    ChVector3d(10.0501781487224,127.271341649174,219.573424974887),
    ChVector3d(-153.681408502864,4.47134164917363,257.256405065421)
};
std::vector<ChQuaternion<>> rotss = { ChQuaternion<>(0.0,0.0,0.0,0.0),
    ChQuaternion<>(-0.0817797748423181,-0.0817797748423181,0.702361778876627,-0.702361778876627),
    ChQuaternion<>(0.649830826959365,0.649830826959365,-0.278782883860017,0.278782883860017),
    ChQuaternion<>(0.568770873369917,-0.568770873369916,-0.420118666100451,-0.420118666100451),
    ChQuaternion<>(0.590890270601115,0.590890270601115,0.388392440849382,-0.388392440849382),
    ChQuaternion<>(0.707106781186548,-0.707106781186547,-8.1335083852247e-17,0),
    ChQuaternion<>(0.132211074481703,-0.132211074481703,0.694636762476903,0.694636762476903),
    ChQuaternion<>(-0.227257494306092,0.227257494306092,0.669592436696918,0.669592436696918),
    ChQuaternion<>(0.661716586109785,0.661716586109785,-0.249261227765594,0.249261227765594),
    ChQuaternion<>(1,0,0,0),
    ChQuaternion<>(0.699115859811739,0.699115859811739,0.106004785550905,-0.106004785550905)
};

std::vector<ChVector3d> inertiaXX = {ChVector3d(0,0,0),
    ChVector3d(0.634526836838321,1.15329562580959,0.603909758681937),
    ChVector3d(11.2789699070089,11.5952382532535,14.5855497562076),
    ChVector3d(7.21167604690774,16.228522846043,17.1708050876363),
    ChVector3d(1.09268072169601,0.695141740951902,0.603909758681937),
    ChVector3d(0.603909758681937,1.18391270396598,0.603909758681937),
    ChVector3d(8.94033623042991,4.92384567286665,9.78576029322433),
    ChVector3d(0.81879554426816,0.969026918379756,0.603909758681937),
    ChVector3d(16827.443821127,13264.5805396432,28809.6343767049),
    ChVector3d(29568.9201124322,45865.3318996888,73541.0736970342),
    ChVector3d(12.1095503099806,7.23515323489345,12.4345268420207)
};

std::vector<double> mass = {0.0,
    0.00573763521549478,
    0.0501455152739229,
    0.0477640623787958,
    0.00573763521549478,
    0.00573763521549478,
    0.0161771922143864,
    0.00573763521549478,
    2.7593573274781,
    1.54162568183963,
    0.0234167508805127
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
    "GearC",            // Part1
    "GearF",            // GearA
    "GearA",            // GearF
    "GearB",            // frame
    "GearD",            // GearB
    "Part1",            // GearC
    "GearE",            // GearD
    "Part2_flywheel",   // GearE
    "frame",            // Part2_flywheel
    "Part2_dyno"        // Part2_dyno
};
std::vector<std::tuple<std::string, ChVector3d, ChQuaternion<>, ChVector3d, double, std::string>> objData = {
    {file_locations[0], positions[0], rotss[0], inertiaXX[0], mass[0], file_names[0]},      //
    {file_locations[6], positions[6], rotss[6], inertiaXX[6], mass[6], file_names[6]},      //Part1
    {file_locations[3], positions[3], rotss[3], inertiaXX[3], mass[3], file_names[3]},      //GearA
    {file_locations[2], positions[2], rotss[2], inertiaXX[2], mass[2], file_names[2]},      //GearF
    {file_locations[9], positions[9], rotss[9], inertiaXX[9], mass[9], file_names[9]},      //frame
    {file_locations[4], positions[4], rotss[4], inertiaXX[4], mass[4], file_names[4]},      //GearB
    {file_locations[1], positions[1], rotss[1], inertiaXX[1], mass[1], file_names[1]},      //GearC
    {file_locations[5], positions[5], rotss[5], inertiaXX[5], mass[5], file_names[5]},      //GearD
    {file_locations[7], positions[7], rotss[7], inertiaXX[7], mass[7], file_names[7]},      //GearE
    {file_locations[8], positions[8], rotss[8], inertiaXX[8], mass[8], file_names[8]},      //Part2_flywheel
    {file_locations[10], positions[10], rotss[10], inertiaXX[10], mass[10], file_names[10]} //Part2_dyno
};

ChVector3d posOffset =  positions[9];