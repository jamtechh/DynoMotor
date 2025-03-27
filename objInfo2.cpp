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
    ChVector3d( 21.949, 21.949, 4.695 ),
    ChVector3d( 0.604, 0.604, 1.184 ),
    ChVector3d( 9.576, 9.786, 4.288 ),
    ChVector3d( 0.604, 0.604, 1.184 ),
    ChVector3d( 0.604, 0.604, 1.184 ),
    ChVector3d( 0.604, 0.604, 1.184 ),
    ChVector3d( 14.586, 14.586, 8.289 ),
    ChVector3d( 12.643, 12.435, 6.701)
};

std::vector<double> mass = {0.0,
    2.700,
    1.542,
    0.0741,
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