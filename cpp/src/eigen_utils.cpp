#include "eigen_utils.hpp"
#include "delta_constants.hpp"
#include <cmath>

namespace delta_robot {
namespace eigen_utils {

TriangleProperties analyzeTriangle(const Eigen::Vector3d& A, const Eigen::Vector3d& B, const Eigen::Vector3d& C) {
    TriangleProperties props;
    
    // Calculate side vectors and lengths
    Eigen::Vector3d AB = B - A;
    Eigen::Vector3d BC = C - B;
    Eigen::Vector3d CA = A - C;
    
    props.side_lengths[0] = BC.norm();  // a (opposite to A)
    props.side_lengths[1] = CA.norm();  // b (opposite to B)  
    props.side_lengths[2] = AB.norm();  // c (opposite to C)
    
    // Check for degenerate triangle
    double min_side = std::min({props.side_lengths[0], props.side_lengths[1], props.side_lengths[2]});
    double max_side = std::max({props.side_lengths[0], props.side_lengths[1], props.side_lengths[2]});
    props.is_degenerate = (min_side < constants::DEGENERATE_TRIANGLE_TOLERANCE) || 
                         (min_side / max_side < constants::DEGENERATE_TRIANGLE_TOLERANCE);
    
    if (props.is_degenerate) {
        // Set default values for degenerate case
        props.angles = {M_PI/3.0, M_PI/3.0, M_PI/3.0};  // Equilateral fallback
        props.centroid = (A + B + C) / 3.0;
        props.normal = Eigen::Vector3d::UnitZ();
        props.area = 0.0;
        return props;
    }
    
    // Calculate angles using safe operations (remove unused normalized vectors)
    props.angles[0] = safeAngleBetween(-CA, AB);    // Angle at A
    props.angles[1] = safeAngleBetween(-AB, BC);    // Angle at B
    props.angles[2] = safeAngleBetween(-BC, CA);    // Angle at C
    
    // Calculate centroid
    props.centroid = (A + B + C) / 3.0;
    
    // Calculate normal using safe cross product
    props.normal = safeCrossProduct(AB, -CA);  // AB × AC
    props.normal = safeNormalized(props.normal);
    
    // Ensure normal points upward
    if (props.normal.z() < 0) {
        props.normal = -props.normal;
    }
    
    // Calculate area using cross product magnitude
    props.area = 0.5 * AB.cross(-CA).norm();
    
    return props;
}

} // namespace eigen_utils
} // namespace delta_robot