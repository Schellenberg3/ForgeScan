#include "ForgeScan/Manager.hpp"
#include "ForgeScan/Metrics/OccupancyConfusion.hpp"
#include "ForgeScan/Simulation/Scene.hpp"
#include "ForgeScan/Sensor/ImShow.hpp"

#include "ForgeScan/Utilities/ArgParser.hpp"
#include "ForgeScan/Utilities/Random.hpp"
#include "ForgeScan/Utilities/Timer.hpp"


int main(const int argc, const char **argv)
{
    forge_scan::utilities::ArgParser parser(argc, argv);
    const float reject_rate   = std::clamp(parser.get<float>("--reject", 0.0), 0.0f, 1.0f);
    const bool  show_im       = parser.has("--show");
    const bool  sphere_policy = parser.has("--sphere");


    // ************************************ Load the scene ************************************* //

    auto scene = forge_scan::simulation::Scene::create();
    scene->load("GroundTruth.h5");


    // ************************************ Create a camera ************************************ //

    auto intr   = forge_scan::sensor::Intrinsics::create(parser);
    auto camera = forge_scan::sensor::Camera::create(intr);
    auto camera_pose = forge_scan::Extrinsic::Identity();


    // ***************************** Set up a Manager and a Policy ***************************** //

    auto manager = forge_scan::Manager::create(scene->grid_properties);

    if (sphere_policy)
    {
        std::cout << "using Sphere policy..." << std::endl;
        manager->policyAdd("--set-active --policy-type Sphere --n-views 10 --uniform --unordered --seed 50");
    }
    else
    {
        std::cout << "using Axis policy..." << std::endl;
        manager->policyAdd("--set-active --policy-type Axis   --n-views 7 --n-repeat 3 --x -1.0 --y -1.0 --z -1.0 --seed 50 --uniform");
    }
    manager->reconstructionAddChannel("--channel-name tsdf           --grid-type TSDF           --data-type double");
    manager->reconstructionAddChannel("--channel-name update         --grid-type UpdateCount    --data-type uint");
    manager->reconstructionAddChannel("--channel-name binary         --grid-type Binary         --data-type uint");
    manager->reconstructionAddChannel("--channel-name binary_tsdf    --grid-type BinaryTSDF     --data-type uint");
    manager->reconstructionAddChannel("--channel-name probability    --grid-type Probability    --data-type float");


    // ******* Set up an OccupancyConfusion Metric for the Scene, add it to the Manager ******** //

    auto occ_conf = forge_scan::metrics::OccupancyConfusion::create(manager->reconstruction,
                                                                    scene->getGroundTruthOccupancy(), 
                                                                    "binary");
    manager->metricAdd(occ_conf);


    // ****************************** Collect and register images ****************************** //

    forge_scan::utilities::RandomSampler<float> rand_sample;
    forge_scan::utilities::Timer timer;

    forge_scan::PointMatrix sensed_points;
    size_t n = 0;

    timer.start();
    while (!manager->policyIsComplete())
    {
        camera_pose = manager->policyGetView();

        float val = rand_sample.uniform(); 
        if (val >= reject_rate)
        {
            manager->policyAcceptView();

            camera->setExtr(camera_pose);

            scene->image(camera);
            if (show_im)
            {
                forge_scan::sensor::imshow_depth(camera, true);
            }

            camera->getPointMatrix(sensed_points);
            manager->reconstructionUpdate(sensed_points, camera->getExtr());

            std::cout << "Added view: " << n++ << std::endl;
        }
        else
        {
            manager->policyRejectView();
            std::cout << "Rejected view: " << n++ << std::endl;
        }
    }
    timer.stop();

    manager->save("DemoRayTrace.h5");

    std::cout << "Finished! Process took " << timer.elapsedSeconds() << " seconds." << std::endl;

    return 0;
}