/*
 * Copyright 2012 Nate Koenig
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef COLLADALOADER_HH
#define COLLADALOADER_HH

#include <map>
#include <string>
#include <vector>

#include "common/MeshLoader.hh"
#include "math/MathTypes.hh"

class TiXmlElement;

namespace gazebo
{
  namespace common
  {
    class Material;

    /// \addtogroup gazebo_common Common
    /// \{

    /// \class ColladaLoader ColladaLoader.hh common/common.hh
    /// \brief Class used to load Collada mesh files
    class ColladaLoader : public MeshLoader
    {
      /// \brief Constructor
      public: ColladaLoader();

      /// \brief Destructor
      public: virtual ~ColladaLoader();

      /// \brief Load a mesh
      /// \param[in] _filename Collada file to load
      /// \return Pointer to a new Mesh
      public: virtual Mesh *Load(const std::string &_filename);

      /// \brief Load a controller instance
      /// \param[in] _contrXml Pointer to the control XML instance
      /// \param[in] _skelXml Pointer the skeleton xml instance
      /// \param[in] _transform A tranform to apply
      /// \param[in,out] _mesh The mesh being loaded
      private: void LoadController(TiXmlElement *_contrXml,
          TiXmlElement *_skelXml, const math::Matrix4 _transform, Mesh *_mesh);

      /// \brief Load animations for a skeleton
      /// \param[in] _xml Animation XML instance
      /// \param[in,out] _skel Pointer to the skeleton
      private: void LoadAnimations(TiXmlElement *_xml, Skeleton *_skel);

      /// \brief Load a set of animations for a skeleton
      /// \param[in] _xml Pointer to the animation set XML instance
      /// \param[in,out] _skel Pointer to the skeleton
      private: void LoadAnimationSet(TiXmlElement *_xml, Skeleton *_skel);

      /// \brief Load skeleton nodes
      /// \param[in] _xml Pointer to the XML instance
      /// \param[in,out] _xml Pointer to the Skeleton node parent
      private: SkeletonNode* LoadSkeletonNodes(TiXmlElement *_xml,
                                               SkeletonNode *_parent);

      /// \brief Set the tranform for a skeleton node
      /// \param[in] _xml Pointer to the XML instance
      /// \param[in,out] _node The skeleton node
      private: void SetSkeletonNodeTransform(TiXmlElement *_elem,
                                             SkeletonNode *_node);

      /// \brief Load geometry elements
      /// \param[in] _xml Pointer to the XML instance
      /// \param[in] _tranform Transform to apply to the loaded geometry
      /// \param[in,out] _mesh Pointer to the mesh currently being loaded
      private: void LoadGeometry(TiXmlElement *_xml,
                                 const math::Matrix4 &_transform, Mesh *_mesh);

      /// \brief Get an XML element by ID
      /// \param[in] _parent The parent element
      /// \param[in] _name String name of the element
      /// \param[out] _id String ID of the element
      private: TiXmlElement *GetElementId(TiXmlElement *_parent,
                                          const std::string &_name,
                                          const std::string &_id);

      /// \brief Get an XML element by ID
      /// \param[in] _name String name of the element
      /// \param[out] _id String ID of the element
      private: TiXmlElement *GetElementId(const std::string &_name,
                                           const std::string &_id);

      /// \brief Load a node
      /// \param[in] _elem Pointer to the node XML instance
      /// \param[in,out] _mesh Pointer to the current mesh
      /// \param[out] _transform Transform to apply to the node
      private: void LoadNode(TiXmlElement *_elem, Mesh *_mesh,
                             const math::Matrix4 &_transform);

      /// \brief Load a transform
      /// \param[in] _elem Pointer to the transform XML instance
      /// \return A Matrix4 transform
      private: math::Matrix4 LoadNodeTransform(TiXmlElement *_elem);

      /// \brief Load vertices
      /// \param[in] _id String id of the vertices XML node
      /// \param[in] _transform Transform to apply to all vertices
      /// \param[out] _verts Holds the resulting vertices
      /// \param[out] _norms Holds the resulting normals
      private: void LoadVertices(const std::string &_id,
                                 const math::Matrix4 &_transform,
                                 std::vector<math::Vector3> &_verts,
                                 std::vector<math::Vector3> &_norms);

      /// \brief Load positions
      /// \param[in] _id String id of the XML node
      /// \param[in] _transform Transform to apply to all positions
      /// \param[out] _values Holds the resulting position values
      private: void LoadPositions(const std::string &_id,
                                  const math::Matrix4 &_transform,
                                  std::vector<math::Vector3> &_values);

      /// \brief Load normals
      /// \param[in] _id String id of the XML node
      /// \param[in] _transform Transform to apply to all normals
      /// \param[out] _values Holds the resulting normal values
      private: void LoadNormals(const std::string &_id,
                                const math::Matrix4 &_transform,
                                std::vector<math::Vector3> &_values);

      /// \brief Load texture coordinates
      /// \param[in] _id String id of the XML node
      /// \param[out] _values Holds the resulting normal values
      private: void LoadTexCoords(const std::string &_id,
                                 std::vector<math::Vector2d> &_values);

      /// \brief Load a material
      /// \param _name Name of the material XML element
      /// \return A pointer to the new material
      private: Material *LoadMaterial(const std::string &_name);

      /// \brief Load a color or texture
      /// \param[in] _eleme Pointer to the XML element
      /// \param[in] _type One of {diffuse, ambient, emission}
      /// \param[out] _mat Material to load the texture or color into
      private: void LoadColorOrTexture(TiXmlElement *_elem,
                                       const std::string &_type,
                                       Material *_mat);

      /// \brief Load triangles
      /// \param[in] _trianglesXml Pointer the triangles XML instance
      /// \param[in] _transform Transform to apply to all triangles
      /// \param[out] _mesh Mesh that is currently being loaded
      private: void LoadTriangles(TiXmlElement *_trianglesXml,
                                   const math::Matrix4 &_transform,
                                   Mesh *_mesh);

      /// \brief Load a polygon list
      /// \param[in] _polylistXml Pointer to the XML element
      /// \param[in] _transform Transform to apply to each polygon
      /// \param[out] _mesh Mesh that is currently being loaded
      private: void LoadPolylist(TiXmlElement *_polylistXml,
                                   const math::Matrix4 &_transform,
                                   Mesh *_mesh);

      /// \brief Load lines
      /// \param[in] _xml Pointer to the XML element
      /// \param[in] _transform Transform to apply
      /// \param[out] _mesh Mesh that is currently being loaded
      private: void LoadLines(TiXmlElement *_xml,
                               const math::Matrix4 &_transform,
                               Mesh *_mesh);

      /// \brief Load an entire scene
      /// \param[out] _mesh Mesh that is currently being loaded
      private: void LoadScene(Mesh *_mesh);

      /// \brief Load a float value
      /// \param[out] _elem Pointer to the XML element
      /// \return The float value
      private: float LoadFloat(TiXmlElement *_elem);

      /// \brief Load a transparent material. NOT IMPLEMENTED
      /// \param[in] _elem Pointer to the XML element
      /// \param[out] _mat Material to hold the transparent properties
      private: void LoadTransparent(TiXmlElement *_elem, Material *_mat);

      /// \brief scaling factor
      private: double meter;

      /// \brief COLLADA file name
      private: std::string filename;

      /// \brief material dictionary indexed by name
      private: std::map<std::string, std::string> materialMap;

      /// \brief root xml element of COLLADA data
      private: TiXmlElement *colladaXml;

      /// \brief directory of COLLADA file name
      private: std::string path;
    };
    /// \}
  }
}
#endif
