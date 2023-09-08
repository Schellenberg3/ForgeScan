#include "ForgeScan/Simulation/Scene.hpp"


int main(const int argc, const char **argv)
{
    forge_scan::utilities::ArgParser parser(argc, argv);

    std::filesystem::path fpath   = parser.get<std::filesystem::path>("--save", FORGE_SCAN_SHARE_DIR "/Examples/Scene.h5");


    // ************************************ Set up a scene ************************************* //

    forge_scan::Extrinsic scene_lower_bound = forge_scan::Extrinsic::Identity();
    scene_lower_bound.translation() = forge_scan::Point( -1, -1, -1);

    auto scene = forge_scan::simulation::Scene::create(scene_lower_bound);

    scene->add("--name sphere1 --shape sphere --radius 0.35");
    scene->add("--name sphere2 --shape sphere --radius 0.25 --x 0.25 --y 0.25 --z 0.25");
    scene->add("--name box1    --shape box --l 1.25 --w 0.25 --h 0.75 --rx 6");


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