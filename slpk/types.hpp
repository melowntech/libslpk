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
#include <map>
#include <memory>
#include <functional>

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

const std::string MainFile("metadata.json");

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
    int wkid; // defaults to WGS84
    int latestWkid;
    int vcsWkid; // defaults to EGM96
    int latestVcsWkid;
    std::string wkt;

    SpatialReference()
        : wkid(4326), latestWkid(), vcsWkid(5773), latestVcsWkid()
    {}

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
                      ((attributes)("Attributes"))
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
                      ((interleavedArray)("InterleavedArray"))
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
    std::size_t byteOffset;
    std::size_t count;
    DataType valueType;
    std::uint16_t valuesPerElement;
    // TODO: values
    std::vector<int> componentIndices;

    GeometryAttribute(const std::string &key)
        : key(key), byteOffset(), count(), valueType(), valuesPerElement()
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

    /** Sorted by `ordering`. Ordering is not present here.
     */
    GeometryAttribute::list faces;

    /** Sorted by `featureAttributeOrder`. Ordering is not present here.
     */
    GeometryAttribute::list featureAttributes;

    GeometrySchema()
        : geometryType(GeometryType::triangles)
        , topology(Topology::perAttributeArray)
    {}
};

typedef std::function<math::Size2
                      (const roarchive::IStream::pointer&)> Size2Function;

struct Encoding {
    // mime type
    std::string mime;
    // filename extension
    std::string ext;
    // encdings with higher preference value are more preferred, negative value
    // means type is unsupported
    int preference;

    /** Measures size2 of files of this encoding. Available only for (some)
     *  image formats.
     */
    Size2Function size2;

    typedef std::vector<Encoding> list;

    Encoding() : preference(0) {}
};

struct PreferredEncoding {
    const Encoding *encoding;
    int index;

    PreferredEncoding(const Encoding *encoding = nullptr, int index = 0)
        : encoding(encoding), index(index)
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
    Encoding nidEncoding;
    Encoding featureEncoding;
    Encoding geometryEncoding;
    Encoding::list textureEncoding;
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

    void finish(const std::string &cwd = "");

    const PreferredEncoding& preferredTextureEncoding() const {
        return preferredTextureEncoding_;
    }

    typedef std::shared_ptr<Store> pointer;

private:
    PreferredEncoding preferredTextureEncoding_;
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

    Store::pointer store;

    // capabilities
    // cachedDrawingInfo
    // drawingInfo
    // fields
    // attributeStorageInfo

    SceneLayerInfo() : id() , layerType(LayerType::object) {}

    void finish(const std::string &cwd = "");
};

struct MeanBoundingSphere {
    double x;
    double y;
    double z;
    double r;
};

struct NodeReference {
    std::string id;
    MeanBoundingSphere mbs;
    std::string href;
    std::string version;
    int featureCount;

    typedef std::vector<NodeReference> list;
};

struct FeatureRange {
    int min;
    int max;

    FeatureRange(int min = 0, int max = 0) : min(min), max(max) {}
};

struct Resource {
    std::string href;
    const Encoding *encoding;
    std::vector<std::string> layerContent;
    FeatureRange featureRange;
    bool multiTextureBundle;
    int vertexElements;
    int faceElements;

    Resource()
        : encoding(), multiTextureBundle(), vertexElements(), faceElements() {}

    typedef std::vector<Resource> list;
};

UTILITY_GENERATE_ENUM(MetricType,
                      ((maxScreenThreshold))
                      ((screenSpaceRelative))
                      ((distanceRangeFromDefaultCamera))
                      )

struct LodSelection {
    MetricType metricType;
    double maxValue;
    double avgValue;
    double minValue;

    LodSelection() : maxValue(), avgValue(), minValue() {}
};

struct Feature {
    int id;
    MeanBoundingSphere mbs;
    int lodChildFeatures;
    std::vector<std::string> lodChildNodes;
    int rank;
    std::string rootFeature;

    typedef std::vector<Feature> list;

    Feature() : id(), lodChildFeatures(), rank() {}
};

struct Node {
    std::string id;
    int level;
    std::string version;
    MeanBoundingSphere mbs;
    boost::optional<std::time_t> created;
    boost::optional<std::time_t> expires;
    boost::optional<math::Matrix4> transform;
    boost::optional<NodeReference> parentNode;
    NodeReference::list children;
    NodeReference::list neighbors;

    boost::optional<Resource> sharedResource;
    Resource::list featureData;
    Resource::list geometryData;
    Resource::list textureData;
    boost::optional<LodSelection> lodSelection;
    Feature::list features;

    Node(const Store::pointer &store) : store_(store) {}
    const Store& store() const { return *store_; }

    bool hasGeometry() const { return !geometryData.empty(); }

    typedef std::map<std::string, Node> map;

private:
    /** Reference to shared store.
     */
    Store::pointer store_;
};

struct Tree {
    std::string rootNodeId;

    Node::map nodes;

    const Node* find(const std::string &id) const {
        auto fnodes(nodes.find(id));
        if (fnodes == nodes.end()) { return nullptr; }
        return &fnodes->second;
    }
};

} // namespace slpk

#endif // slpk_types_hpp_included_
