#include <ForgeScan/voxel_element.h>

#include <iostream>


/// @brief Displays memory size and numeric limits for VoxelElement class and its
///        (possible) constituents. 
void display_memory_size_and_limits();

int main()
{
    VoxelElement ve0(0, 0,0,0,0);
    VoxelElement ve1;


    display_memory_size_and_limits();


    auto c0 = ve0.get_centrality();
    auto c1 = ve1.get_centrality();


    voxel_distance d = 1;
    VoxelElementUpdate updates(&d);

    update_voxel_element(ve0, updates);

    d = -2;
    update_voxel_element(ve0, updates);

    return 0;
}


void display_memory_size_and_limits()
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
    // the fact we would need to cast to int each time is not good. uchar is too small.

    std::cout << std::endl;
}

