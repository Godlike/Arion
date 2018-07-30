/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef ARION_BOUNDING_VOLUMES_HPP
#define ARION_BOUNDING_VOLUMES_HPP

#include <Arion/Shape.hpp>
#include <Arion/Intersection.hpp>

#include <vector>
#include <set>

namespace arion
{
namespace volume
{
/**
 * @brief Represents an instance of mesh data
 */
struct Mesh
{
    /**
     * @brief Constructs mesh data object
     * @param vertices vertex buffer data
     * @param indices index buffer data
     */
    Mesh(std::vector<glm::vec3> const& vertices, std::vector<glm::u64vec3> const& indices);

    std::vector<glm::vec3> const& vertices;
    std::vector<glm::u64vec3> const& indices;
};

/**
 * @brief Calculates average value of a vertex for the given set of indices
 * @param mesh vertex and index data
 * @param indices set of indices for calculation
 * @return average vertex value
 */
glm::vec3 CalculateMeanVertex(
    volume::Mesh const& mesh, std::set<std::size_t> const& indices
);

/**
 * @brief Calculates covariance matrix for the given set of vertices
 * @param mesh vertex and index data
 * @param indices set of indices for calculation
 * @param mean average vertex value for the given set
 * @return covariance matrix
 */
glm::mat3 CalculateCovarianceMatrix(
    volume::Mesh const& mesh, std::set<std::size_t> const& indices, glm::vec3 const& mean
);

/**
 * @brief Calculates extremal vertices for the given set in the given directions
 * @param basis direction vectors
 * @param mesh vertex and index data
 * @param indices set of indices for calculation
 * @return extremal vertices
 */
glm::mat3 CalculateExtremalVertices(
    glm::mat3 const& basis, volume::Mesh const& mesh, std::set<std::size_t> const& indices
);

namespace obb
{
/**
 * @brief Oriented bounding box calculation algorithm
 */
class OrientedBoundingBox
{
public:
    /**
     * @brief Stores calculation data
     */
    struct Box
    {
        glm::vec3 mean;
        glm::mat3 covariance;
        glm::mat3 eigenVectors;
        glm::mat3 eigenVectorsNormalized;
        glm::mat3 extremalVertices;
        glm::mat3 boxAxes;
    };

    /**
     * @brief Constructs OBB for the given set of vertices in the given mesh
     * @param mesh vertex and index data
     * @param indices set of indices for calculation
     */
    OrientedBoundingBox(Mesh const& mesh, std::set<std::size_t> const& indices);

    /**
     * @brief Returns pre-calculated OBB shape
     * @return obb shape
     */
    arion::Box GetVolume() const;

private:
    arion::Box m_boxShape;
    Box m_box;
    Mesh const& m_shape;
    std::set<std::size_t> const& m_indices;
};
} // namespace obb

namespace aabb
{
/**
 * @brief Axis aligned bounding box calculation algorithm
 */
class AxisAlignedBoundingBox
{
public:
    /**
     * @brief Stores representations of AABB
     */
    struct Box
    {
        glm::vec3 xMin;
        glm::vec3 xMax;
        glm::vec3 yMin;
        glm::vec3 yMax;
        glm::vec3 zMin;
        glm::vec3 zMax;
        glm::vec3 extremalMean;
        glm::vec3 xAxis;
        glm::vec3 yAxis;
        glm::vec3 zAxis;
    };

    /**
     * @brief Constructs AABB for the given set of vertices in the given mesh
     * @param mesh vertex and index data
     * @param indices set of indices for calculation
     */
    AxisAlignedBoundingBox(Mesh const& mesh, std::set<std::size_t> const& indices);

    /**
     * @brief Returns pre-calculated AABB shape
     * @return aabb shape
     */
    arion::Box GetVolume() const;

private:
    arion::Box m_boxShape;
    Box m_box;
    Mesh const& m_shape;
    std::set<std::size_t> const& m_indices;

    /**
     * @brief Calculates extremal vertices from the given set along the x,y and z axes
     * @param[in] mesh vertex and index data
     * @param[in] indices set of indices for calculation
     * @param[out] box aabb data
     */
    static void CalculateExtremalVetices(Mesh const& mesh, std::set<std::size_t> const& indices, Box& box);

    /**
     * @brief Calculates average box vertex
     * @param[out] box aabb data
     */
    static void CalculateMean(Box& box);

    /**
     * @brief Creates shape representation of AABB
     * @param[in,out] box aabb data
     */
    void CreateBox(Box& box);
};
} // namespace aabb

namespace sphere
{
/**
 * @brief Bounding sphere calculation algorithm
 */
class BoundingSphere
{
public:
    /**
     * @brief Stores representation of Bounding sphere
     */
    struct Sphere
    {
        glm::vec3 mean;
        glm::mat3 covariance;
        glm::vec3 eigenValues;
        glm::mat3 eigenVectors;
        glm::mat3 eigenVectorsNormalized;
    };

    /**
     * @brief Calculates bounding sphere for the given set of vertices in the given mesh
     *
     * The implementation of the algorithm accounts for the dispersion and density of the
     * vertex data by calculating covariance matrix and refining initial sphere in the direction
     * of the maximum spread.
     * @param mesh vertex and index data
     * @param indices set of indices for calculation
     */
    BoundingSphere(Mesh const& mesh, std::set<std::size_t> const& indices);

    /**
    * @brief Returns pre-calculated Bounding sphere shape
    * @return bounding sphere shape
    */
    arion::Sphere GetVolume() const;

private:
    arion::Sphere m_sphereShape;
    Sphere m_sphere;
    Mesh const& m_shape;
    std::set<std::size_t> const& m_indices;

    /**
     * @brief Calculates initial bounding sphere
     *
     * @note: This method requires calculation of the covariance matrix
     * for the given set of vertices in order to construct initial guess more efficiently.
     * @param[in] eigenVectors eigenvetors of the covariance matrix
     * @param[in] eigenValues eigenvalues of the covariance matrix
     * @param[in] mesh vertex and index data
     * @param[in] indices set of indices for calculation
     * @return bounding sphere shape
     */
    static arion::Sphere CalculateInitialBoundingSphere(
        glm::mat3 const& eigenVectors, glm::vec3 const& eigenValues,
        Mesh const& mesh, std::set<std::size_t> const& indices
    );

    /**
     * @brief Iteratively refines sphere
     *
     * Refining happens by accounting for the points that are outside of the current bounding sphere
     * @param[in] sphere initial sphere shape
     * @param[in] mesh vertex and index data
     * @param[in] indices set of indices for calculation
     * @return bounding sphere shape
     */
    static arion::Sphere RefineSphere(
        arion::Sphere const& sphere, Mesh const& mesh, std::set<std::size_t> const& indices
    );
};
} // namespace sphere
} // namespace volumes
} // namespace arion
#endif // ARION_BOUNDING_VOLUMES_HPP
