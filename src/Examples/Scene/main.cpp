#include "ForgeScan/Simulation/Scene.hpp"


int main(const int argc, const char **argv)
{
    forge_scan::utilities::ArgParser parser(argc, argv);

    std::filesystem::path fpath   = parser.get<std::filesystem::path>("--save", FORGE_SCAN_SHARE_DIR "/Examples/Scene.h5");


    // ************************************ Set up a scene ************************************* //

    forge_scan::Extrinsic scene_lower_bound = forge_scan::Extrinsic::Identity();
    scene_lower_bound.translation() = forge_scan::Point( -1, -1, -1);

    auto scene = forge_scan::simulation::Scene::create(scene_lower_bound);

    scene->add("--file abc0.stl --x 0.5");
    scene->add("--file abc1.stl --z 0.5 --scale 5");
    scene->add("--file abc2.stl --y 0.5 --scale 5");


    // ****************************** Calculate the ground truth ******************************* //

    scene->calculateGroundTruthOccupancy();
    scene->calculateGroundTruthTSDF();


    // **************************** Write the scene to an HDF5 file **************************** //

    auto updated_fpath = scene->save(fpath);
    std::cout << "Saved scene at " << updated_fpath << std::endl;


    // *********************** Verify that we can re-load the HDF5 file ************************ //

    scene->load(updated_fpath);

    auto scene2 = forge_scan::simulation::Scene::create();
    scene2->load(updated_fpath);

    std::cout << "Success! Reloaded the scene." << std::endl;

    return 0;
}