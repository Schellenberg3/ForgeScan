#include "ForgeScan/Policies/Precomputed/Normal.hpp"


/// @brief Helper to get the user's input properties.
/// @return Shared, constant pointer to the Grid Properties to use.
inline std::shared_ptr<const forge_scan::Grid::Properties> get_properties()
{
    auto properties = forge_scan::Grid::Properties::createConstInteractive();
    std::cout << "\nPrecomputed views will be generated for a reconstruction with the following Grid Properties:"
              << "\n\t" << *properties << std::endl;
    return properties;
}


/// @brief Helper to set the lower bounds rotation.
/// @param [out] parser ArgParser to use. On return this is set to the arguments used for rotation.
/// @param [out] grid_lower_bound Lower bounds to rotate.
inline void get_rotation(forge_scan::utilities::ArgParser& parser,
                         forge_scan::Extrinsic& grid_lower_bound)
{
    grid_lower_bound = forge_scan::Extrinsic::Identity();
    while (true)
    {
        parser.getInput("\nPlease specify rotation of the Reconstruction's lower-bound [-h for help]:");
        if (parser[0] != "-h")
        {
            forge_scan::Entity::setRotation(parser, grid_lower_bound);
            std::string rotation_units = parser.has("--degrees") ? " [degrees]" : " [radians]";
            std::cout << "\nGround Truth will be generated from a lower bound with the following rotation:"
                      << "\n" << grid_lower_bound.rotation()
                      << std::endl;
            break;
        }
        std::cout << "\n" << forge_scan::Entity::helpMessageRotation() << std::endl;
    }
}


/// @brief Helper to set the lower bounds rotation.
/// @param [out] parser ArgParser to use. On return this is set to the arguments used for rotation.
/// @param [out] grid_lower_bound Lower bounds to translate.
inline void get_translation(forge_scan::utilities::ArgParser& parser,
                            forge_scan::Extrinsic& grid_lower_bound)
{
    grid_lower_bound.translation() = forge_scan::Translation::Zero();
    while (true)
    {
        parser.getInput("\nPlease specify position of the Reconstruction's lower-bound [-h for help]:");
        if (parser[0] != "-h")
        {
            forge_scan::Entity::setTranslation(parser, grid_lower_bound);
            std::cout << "\nPrecomputed views will be generated for a reconstruction grid with a lower bound at the following position:\n\t"
                      << grid_lower_bound.translation().transpose() << std::endl;
            break;
        }
        std::cout << "\n" << forge_scan::Entity::helpMessageTranslation() << std::endl;
    }
}


/// @brief Helper to add shapes to a Scene.
/// @param [out] parser ArgParser to use. On return this is set to the last arguments used to make a shape.
/// @param [out] scene  Scene to add shapes to.
inline void add_shapes(forge_scan::utilities::ArgParser& parser,
                       const std::shared_ptr<forge_scan::simulation::Scene>& scene)
{
    while (true)
    {
        parser.getInput("\nAdd meshes to the scene [-h for help or ENTER to finish]:");
        if (parser.has("-h"))
        {
            std::cout << "\n" << forge_scan::simulation::MeshLoader::help(parser) << std::endl;
        }
        else if (!parser.hasArgs())
        {
            std::cout << "\n" << *scene << std::endl;
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
    forge_scan::Extrinsic grid_lower_bound;
    forge_scan::utilities::ArgParser parser;
    static const std::string default_file_path = "PrecomputedViews.h5";

    parser.getInput("Enter file path for this ground truth data:");
    std::filesystem::path fpath = parser.get<std::string>(0, default_file_path);


    // ************************************* SETUP NORMALS ************************************* //

    auto properties = get_properties();
    get_rotation(parser, grid_lower_bound);
    get_translation(parser, grid_lower_bound);

    auto reconstruction = forge_scan::data::Reconstruction::create(properties);

    auto intr = forge_scan::sensor::Intrinsics::create(parser);
    std::cout << "\nCamera model has the following intrinsics:" << "\n\t" << *intr << std::endl;

    parser.getInput("\nPlease enter a radius: ");
    float radius = parser.get<float>(0, 2.5);

    parser.getInput("\nPlease enter min similarity: ");
    float min_similarity = parser.get<float>(0, 0.7);

    parser.getInput("\nPlease enter n_views: ");
    int sample = parser.get<int>(0, 10);

    parser.getInput("\nPlease enter n_store: ");
    int store = parser.get<float>(0, 3);

    parser.getInput("\nPlease enter alpha: ");
    float alpha = parser.get<float>(0, 0.5);

    auto policy = forge_scan::policies::Normal::create(reconstruction, intr, grid_lower_bound, radius, min_similarity, sample, store, alpha);

    add_shapes(parser, policy);


    // ************************************* GENERATE DATA ************************************* //

    policy->precomputeViews();


    // *************************************** SAVE DATA *************************************** //

    policy->save(fpath);
    std::cout << "\nThe generated data was successfully saved at:\n\t"
              << std::filesystem::absolute(fpath) << std::endl;


    parser.setArgs("--file " + fpath.string());
    auto policy2 = forge_scan::policies::Normal::create(reconstruction, parser);

    return 0;
}
