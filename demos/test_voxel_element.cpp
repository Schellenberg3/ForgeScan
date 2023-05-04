#include <iostream>
#include <bitset>

#include <ForgeScan/voxel_element.h>


/// @brief Displays memory size and numeric limits for VoxelElement class and its
///        (possible) constituents. 
void display_voxel_element_memory_size_and_limits();

/// @brief Displays memory size for the Eigen3 Vector3 types.
void display_vector_memory_size_and_limits();


int main()
{
    VoxelElement t;
    
    VoxelUpdate u0(10);
    VoxelUpdate u1(5);
    VoxelUpdate u2(-5);

    t.update(u0);
    t.update(u1);
    t.update(u2);
    ++t.views;

    std::cout << "Target:\n";    
    std::cout << t.min << " min" << std::endl;
    std::cout << t.avg << " avg" << std::endl;
    std::cout << t.var << " var" << std::endl;


    std::cout << std::bitset<16>(t.views) << std::endl;
    resetViewUpdateFlag(t.views);

    std::cout << std::bitset<16>(t.views) << std::endl;
    return 0;
}


void display_voxel_element_memory_size_and_limits()
{
    std::cout << "\nSize of VoxelElement:\t" << sizeof(VoxelElement);
    std::cout << "\nSize of float:\t" << sizeof(float);
    std::cout << "\nSize of uint:\t" << sizeof(unsigned int);
    std::cout << "\nSize of ushort:\t" << sizeof(unsigned short);
    std::cout << "\nSize of size_t:\t" << sizeof(size_t);
    std::cout << "\nSize of uchar:\t" << sizeof(unsigned char);

    std::cout << "\n\nMax of uint:\t" << std::numeric_limits<unsigned int>::max();
    std::cout << "\nMax of ushort:\t" << std::numeric_limits<unsigned short>::max();
    std::cout << "\nMax of size_t:\t" << std::numeric_limits<size_t>::max();
    std::cout << "\nMax of uchar:\t" << (int)std::numeric_limits<unsigned char>::max();

    std::cout << "\n\nSize of double:\t" << sizeof(double);
    // the fact we would need to cast to int each time is not good. uchar is too small.

    std::cout << std::endl;
}


void display_vector_memory_size_and_limits()
{
    std::cout << "\nSize of Eigen Vectors:";
    std::cout << "\n\tSize of Vector3d:\t"  << sizeof(Vector3d);
    std::cout << "\n\tSize of Vector3ui:\t" << sizeof(Vector3ui);
    std::cout << "\n\tSize of Vector3i:\t"  << sizeof(Eigen::Vector3i);
    // Proves to me that we do not need a custom class to minimize the size of our
    // vectors. But do we need something better to reduce function call overhead?

    std::cout << std::endl;
}
