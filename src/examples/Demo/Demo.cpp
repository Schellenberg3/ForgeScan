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
    forge_scan::utilities::RandomSampler<float> rand_sample;

    /************** Load the scene **************/

    auto scene = forge_scan::simulation::Scene::create();
    scene->load("GroundTruth.h5");

    /************** Create a camera **************/

    auto intr   = forge_scan::sensor::Intrinsics::create(parser);
    auto camera = forge_scan::sensor::Camera::create(intr);
    auto camera_pose = forge_scan::Extrinsic::Identity();

    /************** Set up a Manager and a Policy **************/

    auto manager = forge_scan::Manager::create(scene->grid_properties);

    manager->policyAdd("--policy-type RandomSphere --n-views 10 --seed 50 --set-active");
    manager->reconstructionAddChannel("--channel-name tsdf           --grid-type TSDF          --data-type double");
    manager->reconstructionAddChannel("--channel-name update         --grid-type UpdateCount   --data-type uint");
    manager->reconstructionAddChannel("--channel-name occupancy      --grid-type Occupancy     --data-type uint");
    manager->reconstructionAddChannel("--channel-name occupancy_tsdf --grid-type OccupancyTSDF --data-type uint");

    /************** Set up an OccupancyConfusion Metric for the Scene and add it to the Manager **************/

    auto occ_conf = forge_scan::metrics::OccupancyConfusion::create(manager->reconstruction,
                                                                    scene->getGroundTruthOccupancy(), 
                                                                    "occupancy_tsdf");
    manager->metricAdd(occ_conf);

    /************** Get and image and add it to the reconstruction **************/

    forge_scan::utilities::Timer timer;

    forge_scan::PointMatrix sensed_points;

    size_t n = 0;

    timer.start();
    while (!manager->policyIsComplete())
    {
        camera_pose = manager->policyGetView();

        if (rand_sample.uniform() >= 0)
        {
            manager->policyAcceptView();

            camera->setExtr(camera_pose);

            scene->image(camera);
            forge_scan::sensor::imshow_depth(camera, true);

            camera->getPointMatrix(sensed_points);
            manager->reconstructionUpdate(sensed_points, camera->getExtr());

            std::cout << "Added view: " << n++ << std::endl;
        }
        else
        {
            manager->policyRejectView();
        }
    }
    timer.stop();

    manager->save("DemoRayTrace.h5");

    std::cout << "Finished! Process took " << timer.elapsedSeconds() << " seconds." << std::endl;

    return 0;
}