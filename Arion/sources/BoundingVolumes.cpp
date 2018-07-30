/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <Arion/BoundingVolumes.hpp>
#include <Epona/JacobiEigenvalue.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>

#include <algorithm>
#include <functional>

using namespace arion;
using namespace volume;

volume::Mesh::Mesh(std::vector<glm::vec3> const& vertices, std::vector<glm::u64vec3> const& indices)
    : vertices(vertices)
    , indices(indices)
{
}

glm::vec3 volume::CalculateMeanVertex(volume::Mesh const& mesh, std::set<std::size_t> const& indices)
{
    glm::vec3 sum{0, 0, 0};
    for (auto index : indices)
    {
        auto const& face = mesh.indices[index];

        sum += mesh.vertices[face[0]];
        sum += mesh.vertices[face[1]];
        sum += mesh.vertices[face[2]];
    }

    glm::vec3 const mean = 1.0f / (3.0f * indices.size()) * sum;
    return mean;
}

glm::mat3 volume::CalculateCovarianceMatrix(
    volume::Mesh const& mesh, std::set<std::size_t> const& indices, glm::vec3 const& mean)
{
    glm::mat3 c(0.0);

    for (auto index : indices)
    {
        auto const& face = mesh.indices[index];

        glm::vec3 const p = mesh.vertices[face[0]] - mean;
        glm::vec3 const q = mesh.vertices[face[1]] - mean;
        glm::vec3 const r = mesh.vertices[face[2]] - mean;

        for (uint8_t j = 0; j < 3; ++j)
        {
            for (uint8_t k = 0; k < 3; ++k)
            {
                float const v = p[j] * p[k] + q[j] * q[k] + r[j] * r[k];
                c[j][k] += v;
            }
        }
    }

    float const a = 1.0f / (3.0f * indices.size());
    for (uint8_t j = 0; j < 3; ++j)
    {
        for (uint8_t k = 0; k < 3; ++k)
        {
            c[j][k] *= a;
        }
    }

    return c;
}

glm::mat3 volume::CalculateExtremalVertices(
    glm::mat3 const& basis, volume::Mesh const& mesh, std::set<std::size_t> const& indices)
{
    std::vector<glm::vec3 const *> vertices;
    for (auto index : indices)
    {
        auto const& face = mesh.indices[index];
        vertices.insert(vertices.end(), {&mesh.vertices[face[0]], &mesh.vertices[face[1]], &mesh.vertices[face[2]]});
    }

    glm::mat3 extremal(0);
    for (uint8_t i = 0; i < 3; ++i)
    {
        auto maxVertex = *std::max_element(vertices.begin(), vertices.end(), [&basis, i](auto a, auto b) -> bool {
           return glm::abs(glm::dot(*a, basis[i])) < glm::abs(glm::dot(*b, basis[i]));
        });
        extremal[i] = *maxVertex;
    }

    return extremal;
}

obb::OrientedBoundingBox::OrientedBoundingBox(volume::Mesh const& mesh, std::set<std::size_t> const& indices)
    : m_boxShape({}, {}, {}, {}, {})
    , m_shape(mesh)
    , m_indices(indices)
{
    //Todo: implement convex hull step
    m_box.mean = volume::CalculateMeanVertex(m_shape, m_indices);
    m_box.covariance = volume::CalculateCovarianceMatrix(m_shape, m_indices, m_box.mean);

    epona::JacobiEigenvalue jacobiEigen(m_box.covariance);
    m_box.eigenVectors = jacobiEigen.GetEigenvectors();
    m_box.extremalVertices = volume::CalculateExtremalVertices(m_box.eigenVectors, m_shape, m_indices);

    for (uint8_t i = 0; i < 3; ++i) {
        m_box.eigenVectorsNormalized[i] = glm::normalize(m_box.eigenVectors[i]);
    }

    m_box.boxAxes[0] = m_box.eigenVectorsNormalized[0] * glm::dot(m_box.extremalVertices[0] - m_box.mean, m_box.eigenVectorsNormalized[0]);
    m_box.boxAxes[1] = m_box.eigenVectorsNormalized[1] * glm::dot(m_box.extremalVertices[1] - m_box.mean, m_box.eigenVectorsNormalized[1]);
    m_box.boxAxes[2] = m_box.eigenVectorsNormalized[2] * glm::dot(m_box.extremalVertices[2] - m_box.mean, m_box.eigenVectorsNormalized[2]);

    m_boxShape = arion::Box(m_box.mean, {}, m_box.boxAxes[0], m_box.boxAxes[1], m_box.boxAxes[2]);
}

arion::Box obb::OrientedBoundingBox::GetVolume() const
{
    return m_boxShape;
}

aabb::AxisAlignedBoundingBox::AxisAlignedBoundingBox(
    const volume::Mesh& mesh, std::set<std::size_t> const& indices)
    : m_boxShape({}, {}, {}, {}, {})
    , m_shape(mesh)
    , m_indices(indices)
{
    //ToDo: Calculate extremal vertices from a convex hull
    CalculateExtremalVetices(m_shape, m_indices, m_box);
    CalculateMean(m_box);
    CreateBox(m_box);
}

arion::Box aabb::AxisAlignedBoundingBox::GetVolume() const
{
    return m_boxShape;
}

void aabb::AxisAlignedBoundingBox::CalculateExtremalVetices(
    volume::Mesh const& mesh, std::set<std::size_t> const& indices, aabb::AxisAlignedBoundingBox::Box& box)
{
    using namespace std::placeholders;

    //ToDo: optimize it, currently O(n), should be possible to do O(log(n))
    std::vector<glm::vec3 const *> validVertices;
    validVertices.reserve(indices.size() * 3);
    for (auto faceIndex : indices)
    {
        for (uint8_t i = 0; i < 3; ++i)
        {
            validVertices.push_back(&mesh.vertices[mesh.indices[faceIndex][i]]);
        }
    }

    static auto axisCompareVertices = [] (uint32_t axis, glm::vec3 const* a, glm::vec3 const* b) -> bool {
        return (*a)[axis] < (*b)[axis];
    };
    static auto xCompareVertices = std::bind(axisCompareVertices, 0, _1, _2);
    static auto yCompareVertices = std::bind(axisCompareVertices, 1, _1, _2);
    static auto zCompareVertices = std::bind(axisCompareVertices, 2, _1, _2);

    std::sort(validVertices.begin(), validVertices.end(), xCompareVertices);
    box.xMin = *validVertices.front();
    box.xMax = *validVertices.back();

    std::sort(validVertices.begin(), validVertices.end(), yCompareVertices);
    box.yMin = *validVertices.front();
    box.yMax = *validVertices.back();

    std::sort(validVertices.begin(), validVertices.end(), zCompareVertices);
    box.zMin = *validVertices.front();
    box.zMax = *validVertices.back();
}

void aabb::AxisAlignedBoundingBox::CalculateMean(aabb::AxisAlignedBoundingBox::Box& box)
{
    box.extremalMean[0] = (box.xMax[0] + box.xMin[0]) / 2.0f;
    box.extremalMean[1] = (box.yMax[1] + box.yMin[1]) / 2.0f;
    box.extremalMean[2] = (box.zMax[2] + box.zMin[2]) / 2.0f;
    box.xAxis = box.xMax - box.extremalMean;
    box.yAxis = box.yMax - box.extremalMean;
    box.zAxis = box.zMax - box.extremalMean;
}

void aabb::AxisAlignedBoundingBox::CreateBox(aabb::AxisAlignedBoundingBox::Box& box)
{
    m_boxShape = arion::Box(
        box.extremalMean,
        {},
        {box.xAxis[0], 0, 0},
        {0, box.yAxis[1], 0},
        {0, 0, box.zAxis[2]}
    );
}

sphere::BoundingSphere::BoundingSphere(volume::Mesh const& mesh, std::set<std::size_t> const& indices)
    : m_sphereShape({}, {}, 0)
    , m_shape(mesh)
    , m_indices(indices)
{
    m_sphere.mean = volume::CalculateMeanVertex(mesh, indices);
    m_sphere.covariance = volume::CalculateCovarianceMatrix(mesh, indices, m_sphere.mean);

    epona::JacobiEigenvalue jacobiEigen(m_sphere.covariance);
    m_sphere.eigenValues = jacobiEigen.GetEigenvalues();
    m_sphere.eigenVectors = jacobiEigen.GetEigenvectors();

    for (uint8_t i = 0; i < 3; ++i) {
        m_sphere.eigenVectorsNormalized[i] = glm::normalize(m_sphere.eigenVectors[i]);
    }
    m_sphereShape = CalculateInitialBoundingSphere(
        m_sphere.eigenVectorsNormalized, m_sphere.eigenValues, m_shape, m_indices);
    m_sphereShape = RefineSphere(m_sphereShape, mesh, indices);
}

arion::Sphere sphere::BoundingSphere::GetVolume() const
{
    return m_sphereShape;
}

arion::Sphere sphere::BoundingSphere::CalculateInitialBoundingSphere(
    glm::mat3 const& eigenVectors, glm::vec3 const& eigenValues,
    volume::Mesh const& mesh, std::set<std::size_t> const& indices
)
{
    //Find max dispersion axis
    uint32_t maxEigenValueIndex = (eigenValues[0] < eigenValues[1] ? 1 : 0);
    if (eigenValues[maxEigenValueIndex] < eigenValues[2])
    {
        maxEigenValueIndex = 2;
    }
    glm::vec3 maxDispersionAxis = glm::normalize(eigenVectors[maxEigenValueIndex]);

    //Find extremal points on it
    size_t minVertexIndex = 0;
    size_t maxVertexIndex = 0;
    for (auto face_index : indices)
    {
        for (uint8_t i = 0; i < 3; ++i)
        {
            uint64_t const vertex_index = mesh.indices[face_index][i];
            auto const currentVertexProjection = glm::dot(mesh.vertices[vertex_index], maxDispersionAxis);
            if (currentVertexProjection > glm::dot(mesh.vertices[maxVertexIndex], maxDispersionAxis))
            {
                maxVertexIndex = vertex_index;
            }
            if (currentVertexProjection < glm::dot(mesh.vertices[minVertexIndex], maxDispersionAxis))
            {
                minVertexIndex = vertex_index;
            }
        }
    }

    //Calculate sphere
    auto const diameter = glm::length(mesh.vertices[maxVertexIndex] - mesh.vertices[minVertexIndex]);
    auto const radius = diameter / 2.0f;
    auto const center = (mesh.vertices[maxVertexIndex] - mesh.vertices[minVertexIndex]) / 2.0f
            + mesh.vertices[minVertexIndex];

    return arion::Sphere(center, {}, radius);
}

arion::Sphere sphere::BoundingSphere::RefineSphere(
    arion::Sphere const& sphere, volume::Mesh const& mesh, std::set<std::size_t> const& indices
)
{
    float sphereRadius = sphere.radius;
    glm::vec3 sphereCenter = sphere.centerOfMass;

    //Find point outside of the sphere and resize sphere
    for (auto faceIndex : indices)
    {
        for (uint8_t i = 0; i < 3; ++i)
        {
            uint64_t const vertexIndex = mesh.indices[faceIndex][i];
            glm::vec3 const & vertex = mesh.vertices[vertexIndex];
            glm::vec3 const sphereVertexVec = vertex - sphereCenter;
            float const sphereVertexNorm = glm::length(sphereVertexVec);

            if (sphereVertexNorm > sphereRadius)
            {
                glm::vec3 oppositeSphereVertex = glm::normalize(sphereVertexVec) * -1.0f * sphereRadius + sphereCenter;
                sphereCenter = (oppositeSphereVertex - vertex) / 2.0f + vertex;
                sphereRadius = glm::length(sphereCenter - oppositeSphereVertex);
            }
        }
    }

    return arion::Sphere(sphereCenter, {}, sphereRadius);
}
