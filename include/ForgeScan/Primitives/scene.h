#ifndef FORGESCAN_SHAPE_PRIMITIVES_SCENE_H
#define FORGESCAN_SHAPE_PRIMITIVES_SCENE_H


#include "ForgeScan/types.h"
#include "ForgeScan/Primitives/primative.h"

namespace ForgeScan  {
namespace Primitives {

/// @brief A collection of Primitive objects which are imaged together in the same scene.
typedef std::vector<Primitive*> Scene;



} // Primitives
} // ForgeScan

#endif // FORGESCAN_SHAPE_PRIMITIVES_SCENE_H
