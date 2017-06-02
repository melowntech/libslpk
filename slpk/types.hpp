/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef slpk_types_hpp_included_
#define slpk_types_hpp_included_

#include <cstddef>

// Please, don't ask...
#ifdef __GNUC__
#  ifdef major
#     undef major
#  endif
#  ifdef minor
#     undef minor
#  endif
#endif

#include <boost/optional.hpp>

#include "utility/enum-io.hpp"

#include "geo/srsdef.hpp"

namespace slpk {

UTILITY_GENERATE_ENUM(FolderPattern,
                      ((basic)("BASIC"))
                      ((extended)("EXTENDED"))
                      )

UTILITY_GENERATE_ENUM(ArchiveCompressionType,
                      ((store)("STORE"))
                      ((deflate64)("DEFLATE64"))
                      ((deflate)("DEFLATE"))
                      )

UTILITY_GENERATE_ENUM(ResourceCompressionType,
                      ((none)("NONE"))
                      ((gzip)("GZIP"))
                      )

struct Version {
    int major;
    int minor;

    Version(int major = 0, int minor = 0) : major(major), minor(minor) {}
};

struct Metadata {
    FolderPattern folderPattern;
    ArchiveCompressionType archiveCompressionType;
    ResourceCompressionType resourceCompressionType;
    Version version;
    std::size_t nodeCount;

    Metadata()
        : folderPattern(FolderPattern::basic)
        , archiveCompressionType(ArchiveCompressionType::store)
        , resourceCompressionType(ResourceCompressionType::gzip)
        , version({1, 6})
        , nodeCount()
    {}
};

UTILITY_GENERATE_ENUM(LayerType,
                      ((point)("Point"))
                      ((line)("Line"))
                      ((polygon)("Polygon"))
                      ((object)("3DObject"))
                      ((integratedMesh)("IntegratedMesh"))
                      ((pointcloud)("Pointcloud"))
                      )

struct SpatialReference {
    int wkid;
    int latestWkid;
    int vcsWkid;
    int latestVcsWkid;
    std::string wkt;

    SpatialReference() : wkid(), latestWkid(), vcsWkid(), latestVcsWkid() {}

    geo::SrsDefinition srs() const;
};

UTILITY_GENERATE_ENUM(HeightModel,
                      ((orthometric))
                      ((ellipsoidal))
                      )

struct HeightModelInfo {
    HeightModel heightModel;
    std::string ellipsoid;
    std::string heightUnit;

    HeightModelInfo() : heightModel(HeightModel::orthometric) {}
};

UTILITY_GENERATE_ENUM(Profile,
                      ((meshes))
                      ((polygons))
                      ((points))
                      ((lines))
                      ((analytics))
                      ((meshpyramids))
                      ((pointclouds))
                      ((symbols))
                      )

UTILITY_GENERATE_ENUM(ResourcePattern,
                      ((nodeIndexDocument)("3dNodeIndexDocument"))
                      ((sharedResource)("SharedResource"))
                      ((featureData)("FeatureData"))
                      ((geometry)("Geometry"))
                      ((texture)("Texture"))
                      )

UTILITY_GENERATE_ENUM(NormalReferenceFrame,
                      ((eastNorthUp)("east-north-up"))
                      ((earthCentered)("earth-centered"))
                      ((vertexReferenceFrame)("vertex-reference-frame"))
                      )

UTILITY_GENERATE_ENUM(LodType,
                      ((meshPyramid)("MeshPyramid"))
                      ((thinning)("Thinning"))
                      ((clustering)("Clustering"))
                      ((generalizing)("Generalizing"))
                      )

UTILITY_GENERATE_ENUM(LodModel,
                      ((nodeSwitching)("node-switching"))
                      ((none))
                      )

struct Cardinality {
    int min;
    int max;

    Cardinality(int min = 0, int max = 0) : min(min), max(max) {}
};

UTILITY_GENERATE_ENUM(IndexSchemeName,
                      ((esriRTree)("esriRTree"))
                      ((quadTree)("QuadTree"))
                      ((agolTilingScheme)("AGOLTilingScheme"))
                      )

struct IndexScheme {
    IndexSchemeName name;
    bool inclusive;
    int dimensionality;
    Cardinality childrenCardinality;
    Cardinality neighborCardinality;

    IndexScheme()
        : name(IndexSchemeName::esriRTree), inclusive(true), dimensionality(0)
    {}
};

UTILITY_GENERATE_ENUM(GeometryType,
                      ((triangles))
                      ((lines))
                      ((points))
                      )

UTILITY_GENERATE_ENUM(Topology,
                      ((perAttributeArray)("PerAttributeArray"))
                      ((indexed)("Indexed"))
                      )

UTILITY_GENERATE_ENUM(DataType,
                      ((uint8)("UInt8"))
                      ((uint16)("UInt16"))
                      ((uint32)("UInt32"))
                      ((uint64)("UInt64"))
                      ((int8)("Int8"))
                      ((int16)("Int16"))
                      ((int32)("Int32"))
                      ((int64)("Int64"))
                      ((float32)("Float32"))
                      ((float64)("Float64"))
                      )

struct HeaderAttribute {
    std::string property;
    DataType type;

    typedef std::vector<HeaderAttribute> list;

    HeaderAttribute(const std::string &property = ""
                    , DataType type = DataType())
        : property(property), type(type)
    {}
};

struct GeometryAttribute {
    // key from original dictionary
    std::string key;
    // TODO: byteOffset
    // TODO: count
    DataType valueType;
    std::uint16_t valuesPerElement;
    // TODO: values
    std::vector<int> componentIndices;

    GeometryAttribute(const std::string &key)
        : key(key), valueType(), valuesPerElement()
    {}

    typedef std::vector<GeometryAttribute> list;
};

struct GeometrySchema {
    GeometryType geometryType;
    Topology topology;
    HeaderAttribute::list header;

    /** Sorted by `ordering`. Ordering is not present here.
     */
    GeometryAttribute::list vertexAttributes;

    // TODO: faces

    /** Sorted by `featureAttributeOrder`. Ordering is not present here.
     */
    GeometryAttribute::list featureAttributes;

    GeometrySchema()
        : geometryType(), topology(Topology::perAttributeArray)
    {}
};

struct Store {
    std::string id;
    Profile profile;

    std::vector<ResourcePattern> resourcePattern;
    std::string rootNode;
    std::string version;
    math::Extents2 extents;
    std::string indexCRS;
    std::string vertexCRS;
    NormalReferenceFrame normalReferenceFrame;
    std::string nidEncoding;
    std::string featureEncoding;
    std::string geometryEncoding;
    std::vector<std::string> textureEncoding;
    LodType lodType;
    LodModel lodModel;
    IndexScheme indexingScheme;
    boost::optional<GeometrySchema> defaultGeometrySchema;
    // TODO: defaultTextureDefinition
    // TODO: defaultMaterialDefinition

    Store()
        : profile(Profile::meshpyramids)
        , normalReferenceFrame(NormalReferenceFrame::earthCentered)
        , lodType(LodType::meshPyramid)
        , lodModel(LodModel::nodeSwitching)
    {}
};

struct SceneLayerInfo {
    int id;
    LayerType layerType;
    SpatialReference spatialReference;
    HeightModelInfo heightModelInfo;
    std::string version;
    std::string name;
    boost::optional<std::string> alias;
    boost::optional<std::string> description;
    boost::optional<std::string> copyrightText;

    Store store;

    // capabilities
    // cachedDrawingInfo
    // drawingInfo
    // fields
    // attributeStorageInfo
};

} // namespace slpk

#endif // slpk_types_hpp_included_
