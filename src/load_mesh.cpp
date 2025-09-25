/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*   * ...
*********************************************************************/

#include <cstdio>
#include <cmath>
#include <algorithm>
#include <set>

// ROS 2
#include <rclcpp/rclcpp.hpp>               
#include <resource_retriever/retriever.hpp>  
#include <tinyxml2.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/IOStream.hpp>
#include <assimp/IOSystem.hpp>


// TF2
#include <tf2/LinearMath/Vector3.h>       // <-- changed
#include <tf2/LinearMath/Matrix3x3.h>

// Your shapes header
#include "robot_self_filter/shapes.h"

// \author Ioan Sucan ; based on stl_to_mesh 

namespace robot_self_filter
{
namespace shapes
{

// A logger for this file or library (if you have a dedicated node, you might pass a node's logger in)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("robot_self_filter.shapes.load_mesh");

// ------------------------------------------------------------------------------------------------
// Assimp I/O wrappers adapted for ROS 2: ResourceIOStream, ResourceIOSystem
// ------------------------------------------------------------------------------------------------

class ResourceIOStream : public Assimp::IOStream
{
public:
  ResourceIOStream(const resource_retriever::MemoryResource& res)
  : res_(res),
    pos_(res.data.get())
  {
  }

  ~ResourceIOStream() override = default;

  size_t Read(void* buffer, size_t size, size_t count) override
  {
    size_t to_read = size * count;
    if (pos_ + to_read > res_.data.get() + res_.size)
    {
      to_read = res_.size - (pos_ - res_.data.get());
    }
    memcpy(buffer, pos_, to_read);
    pos_ += to_read;
    return to_read;
  }

  size_t Write(const void* /*buffer*/, size_t /*size*/, size_t /*count*/) override
  {
    // ROS 1 used ROS_BREAK(). In ROS 2, throw or log a fatal error:
    RCLCPP_FATAL(LOGGER, "Attempting to write to a read-only resource!");
    return 0;
  }

  aiReturn Seek(size_t offset, aiOrigin origin) override
  {
    uint8_t* new_pos = nullptr;
    switch (origin)
    {
      case aiOrigin_SET:
        new_pos = res_.data.get() + offset;
        break;
      case aiOrigin_CUR:
        new_pos = pos_ + offset;  
        break;
      case aiOrigin_END:
        new_pos = res_.data.get() + res_.size - offset;
        break;
      default:
        // ROS 1 used ROS_BREAK(). In ROS 2, throw or log a fatal error:
        RCLCPP_FATAL(LOGGER, "Invalid origin in Seek()");
        return aiReturn_FAILURE;
    }

    if (new_pos < res_.data.get() || new_pos > res_.data.get() + res_.size)
    {
      return aiReturn_FAILURE;
    }

    pos_ = new_pos;
    return aiReturn_SUCCESS;
  }

  size_t Tell() const override
  {
    return pos_ - res_.data.get();
  }

  size_t FileSize() const override
  {
    return res_.size;
  }

  void Flush() override
  {
    // read-only, do nothing
  }

private:
  resource_retriever::MemoryResource res_;
  uint8_t* pos_;
};

// ------------------------------------------------------------------------------------------------

class ResourceIOSystem : public Assimp::IOSystem
{
public:
  ResourceIOSystem() = default;
  ~ResourceIOSystem() override = default;

  bool Exists(const char* file) const override
  {
    // We attempt to retrieve the file to see if it exists
    resource_retriever::MemoryResource res;
    try
    {
      res = retriever_.get(file);
    }
    catch (resource_retriever::Exception& /*e*/)
    {
      return false;
    }
    return true;
  }

  char getOsSeparator() const override
  {
    return '/';
  }

  Assimp::IOStream* Open(const char* file, const char* /*mode*/="rb") override
  {
    resource_retriever::MemoryResource res;
    try
    {
      res = retriever_.get(file);
    }
    catch (resource_retriever::Exception& /*e*/)
    {
      return nullptr;
    }
    return new ResourceIOStream(res);
  }

  void Close(Assimp::IOStream* stream) override
  {
    delete stream;
  }

private:
  mutable resource_retriever::Retriever retriever_;
};

// ------------------------------------------------------------------------------------------------
// Utility to fetch a "meter" scaling factor from Collada files
// ------------------------------------------------------------------------------------------------
float getMeshUnitRescale(const std::string& resource_path)
{
  float unit_scale(1.0f);
  // Attempt to read from cache, if you implement one:
  //  static std::map<std::string, float> rescale_cache;

  resource_retriever::Retriever retriever;
  resource_retriever::MemoryResource res;
  try
  {
    res = retriever.get(resource_path);
  }
  catch (resource_retriever::Exception &e)
  {
    RCLCPP_ERROR(LOGGER, "%s", e.what());
    return unit_scale;
  }

  if (res.size == 0)
    return unit_scale;

  tinyxml2::XMLDocument xml_doc;
  const char* data = reinterpret_cast<const char*>(res.data.get());
  const auto error = xml_doc.Parse(data, res.size);

  if (error == tinyxml2::XML_SUCCESS)
  {
    const auto* collada_xml = xml_doc.FirstChildElement("COLLADA");
    if (collada_xml != nullptr)
    {
      const auto* asset_xml = collada_xml->FirstChildElement("asset");
      if (asset_xml != nullptr)
      {
        const auto* unit_xml = asset_xml->FirstChildElement("unit");
        if (unit_xml != nullptr && unit_xml->FindAttribute("meter") != nullptr)
        {
          if (unit_xml->QueryFloatAttribute("meter", &unit_scale) != tinyxml2::XML_SUCCESS)
          {
            RCLCPP_WARN(LOGGER,
                        "Failed to convert unit element meter attribute for [%s]. Using default scale=1.0",
                        resource_path.c_str());
          }
        }
      }
    }
  }
  return unit_scale;
}

// ------------------------------------------------------------------------------------------------
// Recursively collect vertices from the Assimp scene node hierarchy
// ------------------------------------------------------------------------------------------------
std::vector<tf2::Vector3> getVerticesFromAssimpNode(const aiScene* scene, const aiNode* node, float scale)
{
  std::vector<tf2::Vector3> vertices;
  if (!node)
  {
    return vertices;
  }

  aiMatrix4x4 transform = node->mTransformation;
  aiNode* pnode = node->mParent;
  while (pnode)
  {
    // Don't convert to y-up, skip the root correction:
    if (pnode->mParent != nullptr)
      transform = pnode->mTransformation * transform;
    pnode = pnode->mParent;
  }

  for (uint32_t i = 0; i < node->mNumMeshes; i++)
  {
    aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];
    // Add the vertices
    for (uint32_t j = 0; j < input_mesh->mNumVertices; j++)
    {
      aiVector3D p = input_mesh->mVertices[j];
      p *= transform;
      p *= scale;
      tf2::Vector3 v(p.x, p.y, p.z);
      vertices.push_back(v);
    }
  }

  // Recursively process children
  for (uint32_t i = 0; i < node->mNumChildren; ++i)
  {
    auto sub_vertices = getVerticesFromAssimpNode(scene, node->mChildren[i], scale);
    vertices.insert(vertices.end(), sub_vertices.begin(), sub_vertices.end());
  }
  return vertices;
}

// ------------------------------------------------------------------------------------------------
// Build a Mesh* from an Assimp scene
// ------------------------------------------------------------------------------------------------
shapes::Mesh* meshFromAssimpScene(const std::string& name, const aiScene* scene)
{
  if (!scene->HasMeshes())
  {
    RCLCPP_ERROR(LOGGER, "No meshes found in file [%s]", name.c_str());
    return nullptr;
  }

  float scale = getMeshUnitRescale(name);
  std::vector<tf2::Vector3> vertices = getVerticesFromAssimpNode(scene, scene->mRootNode, scale);
  return createMeshFromVertices(vertices);
}

// ------------------------------------------------------------------------------------------------
// Mesh creation functions
// ------------------------------------------------------------------------------------------------

namespace detail
{
  struct myVertex
  {
    tf2::Vector3 point;
    unsigned int index;
  };

  struct ltVertexValue
  {
    bool operator()(const myVertex &p1, const myVertex &p2) const
    {
      const auto &v1 = p1.point;
      const auto &v2 = p2.point;
      if (v1.x() < v2.x()) return true;
      if (v1.x() > v2.x()) return false;
      if (v1.y() < v2.y()) return true;
      if (v1.y() > v2.y()) return false;
      if (v1.z() < v2.z()) return true;
      return false;
    }
  };

  struct ltVertexIndex
  {
    bool operator()(const myVertex &p1, const myVertex &p2) const
    {
      return p1.index < p2.index;
    }
  };
}  // namespace detail

shapes::Mesh* createMeshFromVertices(const std::vector<tf2::Vector3> &vertices,
                                     const std::vector<unsigned int> &triangles)
{
  unsigned int nt = triangles.size() / 3;
  shapes::Mesh* mesh = new shapes::Mesh(vertices.size(), nt);
  for (unsigned int i = 0; i < vertices.size(); ++i)
  {
    mesh->vertices[3 * i    ] = vertices[i].x();
    mesh->vertices[3 * i + 1] = vertices[i].y();
    mesh->vertices[3 * i + 2] = vertices[i].z();
  }

  std::copy(triangles.begin(), triangles.end(), mesh->triangles);

  // compute normals
  for (unsigned int i = 0; i < nt; ++i)
  {
    tf2::Vector3 s1 = vertices[triangles[3*i    ]] - vertices[triangles[3*i + 1]];
    tf2::Vector3 s2 = vertices[triangles[3*i + 1]] - vertices[triangles[3*i + 2]];
    tf2::Vector3 normal = s1.cross(s2);
    normal.normalize();
    mesh->normals[3 * i    ] = normal.x();
    mesh->normals[3 * i + 1] = normal.y();
    mesh->normals[3 * i + 2] = normal.z();
  }
  return mesh;
}

shapes::Mesh* createMeshFromVertices(const std::vector<tf2::Vector3> &source)
{
  if (source.size() < 3)
    return nullptr;

  std::set<detail::myVertex, detail::ltVertexValue> vertices;
  std::vector<unsigned int> triangles;

  // Each triple in 'source' is a triangle
  for (size_t i = 0; i < source.size() / 3; ++i)
  {
    // For each of the 3 points
    detail::myVertex vt;

    vt.point = source[3*i];
    auto p1 = vertices.find(vt);
    if (p1 == vertices.end())
    {
      vt.index = vertices.size();
      vertices.insert(vt);
    }
    else
    {
      vt.index = p1->index;
    }
    triangles.push_back(vt.index);

    vt.point = source[3*i + 1];
    auto p2 = vertices.find(vt);
    if (p2 == vertices.end())
    {
      vt.index = vertices.size();
      vertices.insert(vt);
    }
    else
    {
      vt.index = p2->index;
    }
    triangles.push_back(vt.index);

    vt.point = source[3*i + 2];
    auto p3 = vertices.find(vt);
    if (p3 == vertices.end())
    {
      vt.index = vertices.size();
      vertices.insert(vt);
    }
    else
    {
      vt.index = p3->index;
    }
    triangles.push_back(vt.index);
  }

  // Sort vertices by index
  std::vector<detail::myVertex> vt;
  vt.insert(vt.begin(), vertices.begin(), vertices.end());
  std::sort(vt.begin(), vt.end(), detail::ltVertexIndex());

  // Build the actual Mesh
  unsigned int nt = triangles.size() / 3;
  shapes::Mesh* mesh = new shapes::Mesh(vt.size(), nt);

  for (unsigned int i = 0; i < vt.size(); ++i)
  {
    mesh->vertices[3 * i    ] = vt[i].point.x();
    mesh->vertices[3 * i + 1] = vt[i].point.y();
    mesh->vertices[3 * i + 2] = vt[i].point.z();
  }
  std::copy(triangles.begin(), triangles.end(), mesh->triangles);

  // compute normals
  for (unsigned int i = 0; i < nt; ++i)
  {
    tf2::Vector3 s1 = vt[triangles[3*i    ]].point - vt[triangles[3*i + 1]].point;
    tf2::Vector3 s2 = vt[triangles[3*i + 1]].point - vt[triangles[3*i + 2]].point;
    tf2::Vector3 normal = s1.cross(s2);
    normal.normalize();
    mesh->normals[3 * i    ] = normal.x();
    mesh->normals[3 * i + 1] = normal.y();
    mesh->normals[3 * i + 2] = normal.z();
  }
  return mesh;
}

shapes::Mesh* createMeshFromBinaryStlData(const char* data, unsigned int size)
{
  const char* pos = data;
  pos += 80;  // skip 80-byte header
  unsigned int numTriangles = *(reinterpret_cast<const unsigned int*>(pos));
  pos += 4;

  if (static_cast<long>(50 * numTriangles + 84) <= static_cast<long>(size))
  {
    std::vector<tf2::Vector3> vertices;
    vertices.reserve(numTriangles * 3);

    for (unsigned int currentTriangle = 0; currentTriangle < numTriangles; ++currentTriangle)
    {
      // skip the normal
      pos += 12;

      // read vertices
      tf2::Vector3 v1, v2, v3;
      float* floats = (float*)pos;
      v1.setX(floats[0]);
      v1.setY(floats[1]);
      v1.setZ(floats[2]);
      v2.setX(floats[3]);
      v2.setY(floats[4]);
      v2.setZ(floats[5]);
      v3.setX(floats[6]);
      v3.setY(floats[7]);
      v3.setZ(floats[8]);

      pos += 36; // 3 vertices * 3 floats each * 4 bytes
      // skip attribute
      pos += 2;

      vertices.push_back(v1);
      vertices.push_back(v2);
      vertices.push_back(v3);
    }
    return createMeshFromVertices(vertices);
  }
  return nullptr;
}

shapes::Mesh* createMeshFromBinaryDAE(const char* filename)
{
  std::string resource_path(filename);
  Assimp::Importer importer;
  importer.SetIOHandler(new ResourceIOSystem());

  const aiScene* scene = importer.ReadFile(resource_path,
    aiProcess_SortByPType
    | aiProcess_GenNormals
    | aiProcess_Triangulate
    | aiProcess_GenUVCoords
    | aiProcess_FlipUVs);

  if (!scene)
  {
    RCLCPP_ERROR(LOGGER, "Could not load resource [%s]: %s",
                 resource_path.c_str(), importer.GetErrorString());
    return nullptr;
  }
  return meshFromAssimpScene(resource_path, scene);
}

shapes::Mesh* createMeshFromBinaryStl(const char* filename)
{
  FILE* input = fopen(filename, "rb");
  if (!input)
    return nullptr;

  fseek(input, 0, SEEK_END);
  long fileSize = ftell(input);
  fseek(input, 0, SEEK_SET);

  char* buffer = new char[fileSize];
  size_t rd = fread(buffer, fileSize, 1, input);
  fclose(input);

  shapes::Mesh* result = nullptr;
  if (rd == 1)
    result = createMeshFromBinaryStlData(buffer, fileSize);

  delete[] buffer;
  return result;
}

}  // namespace shapes
}  // namespace robot_self_filter
