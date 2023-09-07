#include "ForgeScan/Simulation/Scene.hpp"


/// @brief Helper to get the user's input properties.
/// @return Shared, constant pointer to the Grid Properties to use.
inline std::shared_ptr<const forge_scan::Grid::Properties> get_properties()
{
    auto properties = forge_scan::Grid::Properties::createConstInteractive();
    std::cout << "\nGround Truth will be generated with the following Properties:"
              << "\n\t" << *properties << std::endl;
    return properties;
}


/// @brief Helper to set the lower bounds rotation.
/// @param parser[out] ArgParser to use. On return this is set to the arguments used for rotation.
/// @param scan_lower_bound[out] Lower bounds to rotate.
inline void get_rotation(forge_scan::utilities::ArgParser& parser,
                         forge_scan::Extrinsic& scan_lower_bound)
{
    scan_lower_bound = forge_scan::Extrinsic::Identity();
    while (true)
    {
        parser.getInput("\nPlease specify rotation of the Reconstruction's lower-bound [-h for help]:");
        if (parser[0] != "-h")
        {
            forge_scan::Entity::setRotation(parser, scan_lower_bound);
            std::string rotation_units = parser.has("--degrees") ? " [degrees]" : " [radians]";
            std::cout << "\nGround Truth will be generated from a lower bound with the following rotation:"
                      << "\n" << scan_lower_bound.rotation()
                      << std::endl;
            break;
        }
        std::cout << "\n" << forge_scan::Entity::helpMessageRotation() << std::endl;
    }
}


/// @brief Helper to set the lower bounds rotation.
/// @param parser[out] ArgParser to use. On return this is set to the arguments used for rotation.
/// @param scan_lower_bound[out] Lower bounds to translate.
inline void get_translation(forge_scan::utilities::ArgParser& parser,
                            forge_scan::Extrinsic& scan_lower_bound)
{
    scan_lower_bound.translation() = forge_scan::Translation::Zero();
    while (true)
    {
        parser.getInput("\nPlease specify position of the Reconstruction's lower-bound [-h for help]:");
        if (parser[0] != "-h")
        {
            forge_scan::Entity::setTranslation(parser, scan_lower_bound);
            std::cout << "\nGround Truth will be generated from a lower bound at the following position:\n\t"
                      << scan_lower_bound.translation().transpose() << std::endl;
            break;
        }
        std::cout << "\n" << forge_scan::Entity::helpMessageTranslation() << std::endl;
    }
}


/// @brief Helper to add shapes to a Scene.
/// @param parser[out] ArgParser to use. On return this is set to the last arguments used to make a shape.
/// @param scene[out]  Scene to add shapes to.
inline void add_shapes(forge_scan::utilities::ArgParser& parser,
                       const std::shared_ptr<forge_scan::simulation::Scene>& scene)
{
    while (true)
    {
        parser.getInput("\nAdd primitive shapes to the scene [-h for help or ENTER to finish]:");
        if (parser[0] == "-h")
        {
            if (parser[1] == "sphere")
            {
                std::cout << forge_scan::simulation::Sphere::helpMessage() << std::endl;
            }
            else if (parser[1] == "box")
            {
                std::cout << "\n" << forge_scan::simulation::Box::helpMessage() << std::endl;
            }
            else
            {
                std::cout << "\n" << forge_scan::simulation::Primitive::helpMessage() << std::endl;
            }
        }
        else if (!parser.hasArgs())
        {
            std::cout << "\n" << *scene << std::endl;;
            return;
        }
        else
        {
            try
            {
                scene->add(parser);
            }
            catch(const std::exception& e)
            {
                std::cerr << "Could not add shape: " << e.what() << '\n';
            }
        }
    }
}


int main()
{
    forge_scan::Extrinsic scan_lower_bound;
    forge_scan::utilities::ArgParser parser;
    static const std::string default_file_path = "GroundTruth.h5";

    parser.getInput("Enter file path for this ground truth data:");
    std::filesystem::path fpath = parser.get<std::string>(0, default_file_path);


    // ************************************** SETUP SCENE ************************************** //

    auto properties = get_properties();
    get_rotation(parser, scan_lower_bound);
    get_translation(parser, scan_lower_bound);

    auto scene = forge_scan::simulation::Scene::create(scan_lower_bound);
    scene->setGridProperties(properties);
    add_shapes(parser, scene);


    // ************************************* GENERATE DATA ************************************* //

    parser.getInput("\nGenerate ground truth occupancy? [y/n]:");
    if (parser[0] == "y")
    {
        std::cout << "\n\tGenerating occupancy... ";
        scene->calculateGroundTruthOccupancy();
        std::cout << "Done!" << std::endl;
    }
    else
    {
        std::cout << "\n\tSkipped occupancy." << std::endl;
    }

    parser.getInput("\nGenerate ground truth TSDF? [y/n]:");
    if (parser[0] == "y")
    {
        std::cout << "\n\tGenerating TSDF... ";
        scene->calculateGroundTruthTSDF();
        std::cout << "Done!" << std::endl;
    }
    else
    {
        std::cout << "\n\tSkipped TSDF." << std::endl;
    }


    // *************************************** SAVE DATA *************************************** //

    scene->save(fpath);
    std::cout << "\nThe generated data was successfully saved at:\n\t"
              << std::filesystem::absolute(fpath) << std::endl;

    return 0;
}