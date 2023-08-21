#ifndef FORGE_SCAN_COMMON_DEFINITIONS_HPP
#define FORGE_SCAN_COMMON_DEFINITIONS_HPP


// Ensures that M_PI is defined even if the compiler implementation has not provided it.
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

// Ensures that INFINITY is defined even if the compiler implementation has not provided it.
#ifndef INFINITY
    #include <numeric>
    #define INFINITY std::numeric_limits<float>::infinity()
#endif

// Ensures that NEGATIVE_INFINITY is defined based on INFINITY.
// Unlike INFINITY, this usually not provided by the compiler implementation.
#ifndef NEGATIVE_INFINITY
    #define NEGATIVE_INFINITY -1 * INFINITY
#endif

// Reserved channel name prefix for Policies.
#define FS_POLICY_CHANNEL_PREFIX "Policy"

// Reserved channel name prefix for Metrics.
#define FS_METRIC_CHANNEL_PREFIX "Metric"

// C string length of the reserved channel name prefix for Policies.
#define FS_POLICY_CHANNEL_PREFIX_C_STR_LEN 7

// C string length of the reserved channel name prefix for Metrics.
#define FS_METRIC_CHANNEL_PREFIX_C_STR_LEN 7

// Define the HDF5 file extension for this header file only.
#define FS_HDF5_FILE_EXTENSION ".h5"

// Define the XDMF file extension for this header file only.
#define FS_XDMF_FILE_EXTENSION ".xdmf"

/// Group name where all Reconstruction data is stored in an HDF5 file.
#define FS_HDF5_RECONSTRUCTION_GROUP "Reconstruction"

/// Group name where all Policy data is stored in an HDF5 file.
#define FS_HDF5_POLICY_GROUP "Policy"

/// Group name where all Metric data is stored in an HDF5 file.
#define FS_HDF5_METRIC_GROUP "Metric"


#endif // FORGE_SCAN_COMMON_DEFINITIONS_HPP
