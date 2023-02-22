#include <iostream>
#include <string>

#include <ForgeScan/voxel_grid.h>

/// @brief Simple script for manually adding points to a VoxelGrid within [-1,-1,-1] and [+1,+1,+1].
/// @details  Demonstrates the VoxelGrid `*.HDF5` format and ability to add linearly spaced points.
int main(int argc, char** argv)
{ 
    Eigen::Vector3d lower, upper;
    lower << -1.0, -1.0, -1.0;
    upper << 1.0, 1.0, 1.0;

    double res = 0.02;

    // 2m x 2m x 2m cube with 0.02 m resolution
    VoxelGrid grid(res, lower, upper, 0, false);
    std::cout << "Initialized the VoxelGrid" << std::endl;

    double x, y, z, q = 0;
    Vector3d xyz, v2s, g2s;
    Vector3ui s2g, v2g;
    size_t s2v, g2v;

    int rs2g, rs2v, rg2v, rv2g, rv2s, rg2s, n_result, n_count, n_valid;
    std::string n_string;
    std::vector<Vector3ui> neighbors;


    do
    {
        std::cout << "enter the next point in cartesian space: ";
        std::cin >> x >> y >> z >> q;
        xyz << x, y, z;

        rs2g = grid.gidx(xyz, s2g);
        rs2v = grid.vidx(xyz, s2v);

        rg2v = grid.vidx(s2g, g2v);
        rv2g = grid.gidx(g2v, v2g);

        rv2s = grid.sidx(g2v, v2s);
        rg2s = grid.sidx(s2g, g2s);

        std::cout << "\n " << rs2g << " s->g = " << s2g.transpose();
        std::cout << "\n " << rs2v << " s->v = " << s2v;

        std::cout << "\n " << rg2v << " g->v = " << g2v;
        std::cout << "\n " << rv2g << " v->g = " << v2g.transpose();

        std::cout << "\n " << rv2s << " v->s = " << v2s.transpose();
        std::cout << "\n " << rg2s << " g->s = " << g2s.transpose() << "\n" << std::endl;

        n_result = grid.get_6(s2g, neighbors);

        n_count = 0;
        n_string = (n_result == 0) ? "yes" : "no";
        std::cout << "\nFound " << neighbors.size() << " neighbors. All valid? " << n_string << ". They are:";
        for (auto n = neighbors.begin(); n != neighbors.end(); ++n)
        {
            ++n_count;
            n_valid = grid.valid(*n);
            std::cout << "\n\t[" << n_count << "] " << n_valid - 1 << " " << n->transpose();
        }
        std::cout << std::endl;

    } while (q == 0);


    return 0;
}
