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
#include <set>
#include <memory>
#include <functional>
#include <algorithm>

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

#include "math/geometry_core.hpp"

#include "geo/srsdef.hpp"

#include "roarchive/roarchive.hpp"

#include "detail/files.hpp"

namespace slpk {

const std::string MainFile(detail::constants::MetadataName);

UTILITY_GENERATE_ENUM_CI(FolderPattern,
                         ((basic)("BASIC"))
                         ((extended)("EXTENDED"))
                         )

UTILITY_GENERATE_ENUM_CI(ArchiveCompressionType,
                         ((store)("STORE"))
                         ((deflate64)("DEFLATE64"))
                         ((deflate)("DEFLATE"))
                         )

UTILITY_GENERATE_ENUM_CI(ResourceCompressionType,
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
        , version({1, 7})
        , nodeCount()
    {}
};

UTILITY_GENERATE_ENUM_CI(LayerType,
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

    void setSrs(const geo::SrsDefinition &srs);

    geo::SrsDefinition srs() const;
};

UTILITY_GENERATE_ENUM_CI(HeightModel,
                         ((orthometric)("gravity_related_height")
                          ("orthometric"))
                         ((ellipsoidal))
                         )

struct HeightModelInfo {
    HeightModel heightModel;
    std::string ellipsoid;
    std::string heightUnit;

    HeightModelInfo() : heightModel(HeightModel::orthometric) {}

    HeightModelInfo(const geo::SrsDefinition &srs);
};

UTILITY_GENERATE_ENUM_CI(Profile,
                         ((meshes))
                         ((polygons))
                         ((points))
                         ((lines))
                         ((analytics))
                         ((meshpyramids))
                         ((pointclouds)("pointclouds")("pointcloud"))
                         ((symbols))
                         )

UTILITY_GENERATE_ENUM_CI(ResourcePattern,
                         ((nodeIndexDocument)("3dNodeIndexDocument"))
                         ((sharedResource)("SharedResource"))
                         ((featureData)("FeatureData"))
                         ((geometry)("Geometry"))
                         ((texture)("Texture"))
                         ((attributes)("Attributes"))
                         )

typedef std::vector<ResourcePattern> ResourcePatterns;

UTILITY_GENERATE_ENUM_CI(NormalReferenceFrame,
                         ((eastNorthUp)("east-north-up"))
                         ((earthCentered)("earth-centered"))
                         ((vertexReferenceFrame)("vertex-reference-frame"))
                         )

UTILITY_GENERATE_ENUM_CI(LodType,
                         ((meshPyramid)("MeshPyramid"))
                         ((thinning)("Thinning"))
                         ((clustering)("Clustering"))
                         ((generalizing)("Generalizing"))
                         )

UTILITY_GENERATE_ENUM_CI(LodModel,
                         ((nodeSwitching)("node-switching"))
                         ((none))
                         )

struct Cardinality {
    int min;
    int max;

    Cardinality(int min = 0, int max = 0) : min(min), max(max) {}
};

UTILITY_GENERATE_ENUM_CI(IndexSchemeName,
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

UTILITY_GENERATE_ENUM_CI(GeometryClass,
                         ((ArrayBufferView))
                         ((GeometryReference))
                         ((SharedResourceReference))
                         ((Embedded))
                         )

UTILITY_GENERATE_ENUM_CI(GeometryType,
                         ((triangles))
                         ((lines))
                         ((points))
                         )

UTILITY_GENERATE_ENUM_CI(Topology,
                         ((perAttributeArray)("PerAttributeArray"))
                         ((interleavedArray)("InterleavedArray"))
                         ((indexed)("Indexed"))
                         )

UTILITY_GENERATE_ENUM_CI(DataType,
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

bool has(const GeometryAttribute::list &gal, const std::string &name);

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

    typedef std::vector<GeometrySchema> list;
};

UTILITY_GENERATE_ENUM_CI(MetricType,
                         ((maxScreenThreshold))
                         ((maxScreenThresholdSQ))
                         ((screenSpaceRelative))
                         ((distanceRangeFromDefaultCamera))
                         ((effectiveDensity))
                         )

namespace v17 {

struct GeometryBuffer16 {
    // attribute list; for compatibility reasons re-using geometry attributes
    GeometryAttribute::list attributes;
    std::size_t offset = 0;
};

struct GeometryBuffer {
    // TODO: implement me
};

struct GeometryDefinition {
    GeometryBuffer16 geometryBuffer16;
    std::optional<GeometryBuffer> geometryBuffer;

    using list = std::vector<GeometryDefinition>;
};

struct MeshGeometry {
    std::size_t definition = 0;
    std::size_t resource = 0;
    std::size_t vertexCount = 0;
    std::size_t featureCount = 0;
};

struct MeshMaterial {
    std::size_t definition = 0;
    std::size_t resource = 0;
    std::size_t texelCountHint = 0;
};

struct Mesh {
    MeshMaterial material;
    MeshGeometry geometry;
};

struct NodePageDefinition {
    // cannot be set to anything else
    MetricType lodSelectionMetricType = MetricType::maxScreenThresholdSQ;

    int rootIndex = 0;
    int nodesPerPage = 0;
};

} // namespace v17

typedef std::function<math::Size2
                      (const roarchive::IStream::pointer&)> Size2Function;

struct Encoding {
    // mime type
    std::string mime;
    // filename extension
    std::string ext;
    // encodings with higher preference value are more preferred, negative value
    // means type is unsupported
    int preference;

    /** Measures size2 of files of this encoding. Available only for (some)
     *  image formats.
     */
    Size2Function size2;

    typedef std::vector<Encoding> list;

    Encoding() : preference(0) {}

    Encoding(const std::string &mime) : mime(mime) {}
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

    ResourcePatterns resourcePattern;
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
        , extents(math::InvalidExtents{})
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

UTILITY_GENERATE_ENUM_CI(Capability,
                         ((view)("View"))
                         ((query)("Query"))
                         ((edit)("Edit"))
                         )

typedef std::set<Capability> Capabilities;

struct SceneLayerInfo {
    int id;
    std::string href;
    LayerType layerType;
    SpatialReference spatialReference;
    HeightModelInfo heightModelInfo;
    std::string version;
    std::string name;
    boost::optional<std::string> alias;
    boost::optional<std::string> description;
    boost::optional<std::string> copyrightText;

    Capabilities capabilities;

    Store::pointer store;

    /** V 1.7
     */
    v17::GeometryDefinition::list geometryDefinitions;

    /** Mandatory in V 1.7
     */
    boost::optional<v17::NodePageDefinition> nodePages;

    // cachedDrawingInfo
    // drawingInfo
    // fields
    // attributeStorageInfo

    SceneLayerInfo()
        : id() , layerType(LayerType::object)
        , store(std::make_shared<Store>())
    {}

    void finish(const std::string &cwd = "");
};

struct MinimumBoundingSphere {
    math::Point3 center;
    double r;

    MinimumBoundingSphere() : r() {}
};

struct NodeReference {
    std::string id;
    MinimumBoundingSphere mbs;
    std::string href;
    std::string version;
    int featureCount;

    typedef std::vector<NodeReference> list;

    NodeReference() : featureCount() {}
};

struct FeatureRange {
    int min;
    int max;

    FeatureRange(int min = 0, int max = 0) : min(min), max(max) {}

    bool valid() const { return max >= min; }
};

struct Resource {
    std::string href;
    const Encoding *encoding;
    std::vector<std::string> layerContent;
    FeatureRange featureRange;
    bool multiTextureBundle;
    int vertexElements;
    int faceElements;

    Resource(const std::string &href = "")
        : href(href), encoding(), featureRange(0, -1)
        , multiTextureBundle(), vertexElements(), faceElements()
    {}

    typedef std::vector<Resource> list;
};

struct LodSelection {
    MetricType metricType;
    double maxError;

    LodSelection()
        : metricType(MetricType::maxScreenThreshold)
        , maxError()
    {}

    typedef std::vector<LodSelection> list;
};

struct Feature {
    int id;
    MinimumBoundingSphere mbs;
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
    MinimumBoundingSphere mbs;
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
    LodSelection::list lodSelection;
    Feature::list features;
    v17::GeometryDefinition geometryDefinition;

    Node(const Store::pointer &store = Store::pointer()) : store_(store) {}
    const Store& store() const { return *store_; }

    bool hasGeometry() const { return !geometryData.empty(); }

    NodeReference reference() const {
        NodeReference nr;
        nr.id = id;
        nr.version = version;
        nr.mbs = mbs;
        return nr;
    }

    bool hasSharedResource() {
        // TODO: more fields?
        return sharedResource && !geometryData.empty();
    }

    typedef std::map<std::string, Node> map;

private:
    /** Reference to shared store.
     */
    Store::pointer store_;
};

UTILITY_GENERATE_ENUM_CI(MaterialType,
                         ((standard))
                         ((water))
                         ((billboard))
                         ((leafcard))
                         ((reference))
                         )

UTILITY_GENERATE_ENUM_CI(RenderMode,
                         ((textured))
                         ((solid))
                         ((untextured))
                         ((wireframe))
                         )

UTILITY_GENERATE_ENUM_CI(CullFace,
                         ((none))
                         ((front))
                         ((back))
                         )

UTILITY_GENERATE_ENUM_CI(Wrap,
                         ((none))
                         ((repeat))
                         ((mirror))
                         )

typedef std::array<double, 3> Color;

struct Material {
    typedef std::vector<Material> list;
    std::string key; // key in serialized object

    std::string name;
    MaterialType type;
    std::string ref;

    struct Params {
        bool vertexRegions;
        bool vertexColors;
        bool useVertexColorAlpha;
        double transparency;
        double reflectivity;
        double shininess;
        Color ambient;
        Color difuse;
        Color specular;
        RenderMode renderMode;
        bool castShadows;
        bool receiveShadows;
        CullFace cullFace;

        Params()
            : vertexRegions(), vertexColors(), useVertexColorAlpha()
            , transparency(), reflectivity(), shininess()
            , ambient{{ 0.0, 0.0, 0.0 }}
            , difuse{{ 1.0, 1.0, 1.0 }}
            , specular{{ 0.0, 0.0, 0.0 }}
            , renderMode(RenderMode::textured)
            , castShadows(), receiveShadows()
            , cullFace(CullFace::none)
        {}
    };

    Params params;

    Material(const std::string &key)
        : key(key), type(MaterialType::standard)
    {}
};

struct Image {
    std::string id;
    std::size_t size;
    double pixelInWorldUnits;

    struct Version {
        typedef std::vector<Version> list;

        std::string href;
        std::size_t byteOffset;
        std::size_t length;

        Version(std::string href)
            : href(std::move(href)), byteOffset(), length() {}
    };

    Version::list versions;

    Image() : size(), pixelInWorldUnits() {}
    typedef std::vector<Image> list;
};

struct Texture {
    std::string key; // key in serialized object
    typedef std::vector<Texture> list;

    Encoding::list encoding;
    std::array<Wrap, 2> wrap;
    bool atlas;
    std::string uvSet;
    std::string channels;

    Image::list images;

    Texture(const std::string &key)
        : key(key), wrap{{ Wrap::none, Wrap::none }}, atlas(false)
    {}
};

struct SharedResource {
    Material::list materialDefinitions;
    Texture::list textureDefinitions;

    using optional = boost::optional<SharedResource>;
};

/** Geometry reference
 */
struct GeometryReference {
    std::string ref;
    FeatureRange faceRange;
    bool lodGeometry;

    GeometryReference() : lodGeometry(false) {}

    typedef std::vector<GeometryReference> list;
};

/** Array buffer view geometry type.
 */
struct ArrayBufferView {
    int id;
    math::Matrix4 transformation;

    GeometryType type;
    Topology topology;
    std::string material;
    std::string texture;
    GeometryAttribute::list vertexAttributes;
    GeometryAttribute::list faces;

    ArrayBufferView(int id = 0)
        : id(id)
        , transformation(boost::numeric::ublas::identity_matrix<double>(4))
        , type(), topology()
    {}
    typedef std::vector<ArrayBufferView> list;
};

/** Simpliefied FeatureData file content. Only for file generation.
 */
struct FeatureData {
    /** Feature, only usable for referenced geometries.
     */
    struct Feature {
        int id;
        math::Point3 position;
        math::Point3 pivotOffset;
        math::Extents3 mbb;
        std::string layer;
        GeometryReference::list geometries;

        Feature(int id = 0) : id(id) {}
        typedef std::vector<Feature> list;
    };

    Feature::list featureData;
    ArrayBufferView::list geometryData;
};

/** Expanded node info.
 */
struct NodeInfo {
    std::string href;
    std::string fullpath;
    Node node;

    NodeInfo(std::string href, std::string fullpath, Node &&node)
        : href(std::move(href)), fullpath(std::move(fullpath))
        , node(std::move(node))
    {}

    typedef std::vector<NodeInfo> list;
};

struct TreeNode {
    Node node;
    SharedResource::optional sharedResource;

    TreeNode(Node &&node, SharedResource::optional &&sharedResource)
        : node(std::move(node)), sharedResource(std::move(sharedResource))
    {}

    using map = std::map<std::string, TreeNode>;
};

struct Tree {
    std::string rootNodeId;

    TreeNode::map nodes;

    const Node* node(const std::string &id) const {
        if (auto *tn = find(id)) {
            return &tn->node;
        }
        return nullptr;
    }

    const SharedResource* sharedResource(const std::string &id) const {
        if (auto *tn = find(id)) {
            if (tn->sharedResource) { return &*tn->sharedResource; }
        }
        return nullptr;
    }

    const TreeNode* find(const std::string &id) const {
        auto fnodes(nodes.find(id));
        if (fnodes == nodes.end()) { return nullptr; }
        return &fnodes->second;
    }

};

// inlines

inline bool has(const GeometryAttribute::list &gal, const std::string &name)
{
    return (std::find_if(gal.begin(), gal.end()
                         , [&](const GeometryAttribute &ga)
                         {
                             return (ga.key == name);
                         })
            != gal.end());
}

} // namespace slpk

#endif // slpk_types_hpp_included_
