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

#include <map>
#include <queue>
#include <string>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/copy.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/path.hpp"
#include "utility/streams.hpp"
#include "utility/path.hpp"
#include "utility/uri.hpp"
#include "utility/binaryio.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "./reader.hpp"

namespace fs = boost::filesystem;
namespace bio = boost::iostreams;
namespace bin = utility::binaryio;

namespace slpk {

namespace {

namespace constants {
const std::string MetadataName("metadata.json");
const std::string SceneLayer("3dSceneLayer.json");
const std::string NodeIndex("3dNodeIndexDocument.json");
const fs::path gzExt(".gz");
} // namespace constants

typedef std::map<std::string, std::string> MIMEMapping;

const MIMEMapping mimeMapping {
    { "application/octet-stream", ".bin" }
    , { "image/jpeg", ".jpg" }
    , { "image/vnd-ms.dds", ".bin.dds" }
};

// must stay empty
const std::string unknownMimeExt;

const std::string &extFromMime(const std::string &mime) {
    auto process([](const std::string &mime) -> const std::string&
    {
        auto fmimeMapping(mimeMapping.find(mime));
        if (fmimeMapping == mimeMapping.end()) {
            return unknownMimeExt;
        }
        return fmimeMapping->second;
    });

    auto sc(mime.find(';'));
    if (sc == std::string::npos) {
        return process(mime);
    }
    return process(mime.substr(0, sc));
}

std::string joinPaths(const std::string &a, const std::string &b)
{
    if (a.empty()) { return utility::Uri::removeDotSegments(b); }
    return utility::Uri::joinAndRemoveDotSegments(a, b);
}

std::string makeDir(const std::string &path)
{
    if (path.empty()) { return path; }
    if (path[path.size() - 1] == '/') { return path; }
    return path + "/";
}

void makeDirInplace(std::string &path)
{
    if (!path.empty() && (path[path.size() - 1] != '/')) {
        path.push_back('/');
    }
}

Metadata loadMetadata(std::istream &in, const fs::path &path)
{
    LOG(info1) << "Loading SLPK metadata from " << path  << ".";

    // load json
    const auto value(Json::read(in, path, "SLPK metadata"));

    Metadata metadata;
    try {
        Json::getOpt(metadata.folderPattern, value, "folderPattern");
        Json::getOpt(metadata.archiveCompressionType, value
                     , "ArchiveCompressionType");
        Json::getOpt(metadata.resourceCompressionType, value
                     , "ResourceCompressionType");

        std::string versionString;
        if (Json::getOpt(versionString, value, "I3SVersion")) {
            Version version;
            std::istringstream is(versionString);
            is >> version.major >> utility::expect('.') >> version.minor;
            metadata.version = version;
        }

        Json::get(metadata.nodeCount, value, "nodeCount");
    } catch (const std::ios_base::failure &e) {
        LOGTHROW(err1, std::runtime_error)
            << "Invalid SLPK metadata format (" << e.what()
            << "); Unable to work with this metadata (file: " << path << ").";
    } catch (const Json::Error &e) {
        LOGTHROW(err1, std::runtime_error)
            << "Invalid SLPK metadata format (" << e.what()
            << "); Unable to work with this metadata (file: " << path << ").";
    }

    // done
    return metadata;
}

Metadata loadMetadata(const roarchive::IStream::pointer &in)
{
    return loadMetadata(in->get(), in->path());
}

void parse(SpatialReference &srs, const Json::Value &value)
{
    Json::getOpt(srs.wkid, value, "wkid");
    Json::getOpt(srs.latestWkid, value, "latestWkid");
    Json::getOpt(srs.vcsWkid, value, "vcsWkid");
    Json::getOpt(srs.latestVcsWkid, value, "latestVcsWkid");
    Json::getOpt(srs.wkt, value, "wkt");
}

void parse(HeightModelInfo &hmi, const Json::Value &value)
{
    Json::get(hmi.heightModel, value, "heightModel");
    Json::get(hmi.ellipsoid, value, "ellipsoid");
    Json::get(hmi.heightUnit, value, "heightUnit");
}

void parse(Cardinality &c, const Json::Value &value, const char *name)
{
    get(c.min, value, 0, name);
    get(c.max, value, 1, name);
}

void parse(IndexScheme &is, const Json::Value &value)
{
    Json::get(is.name, value, "name");
    Json::get(is.inclusive, value, "inclusive");
    Json::get(is.dimensionality, value, "dimensionality");
    parse(is.childrenCardinality
          , Json::check(value["childrenCardinality"]
                        , Json::arrayValue, "childrenCardinality")
          , "childrenCardinality");
    if (value.isMember("neighborCardinality")) {
        parse(is.neighborCardinality
              , Json::check(value["neighborCardinality"]
                            , Json::arrayValue, "neighborCardinality")
              ,  "neighborCardinality");
    }
}

void parse(HeaderAttribute::list &hal, const Json::Value &value)
{
    for (const auto &item : value) {
        hal.emplace_back();
        auto &ha(hal.back());
        Json::get(ha.property, item, "property");
        Json::get(ha.type, item, "type");
    }
}

void parse(GeometryAttribute &ga, const Json::Value &value)
{
    Json::getOpt(ga.byteOffset, value, "byteOffset");
    Json::getOpt(ga.count, value, "count");
    Json::get(ga.valueType, value, "valueType");
    Json::get(ga.valuesPerElement, value, "valuesPerElement");
    // TODO: values
    // TODO: componentIndices
}

void parse(GeometryAttribute::list &gal
           , const std::string &attrName, const std::string &orderName
           , const Json::Value &attrs, const Json::Value &order)
{
    // build indexing map
    gal.clear();
    gal.reserve(order.size());
    std::map<std::string, GeometryAttribute*> attrMap;

    for (const auto &key : order) {
        gal.emplace_back(key.asString());
        auto &ga(gal.back());
        attrMap[ga.key] = &ga;
    }

    // process all keys from attributes
    for (const auto &key : attrs.getMemberNames()) {
        auto fattrMap(attrMap.find(key));
        if (fattrMap == attrMap.end()) {
            LOG(warn2)
                << "Format inconsistency: unexpected attribute \""
                << key << "\" in \"" << attrName
                << "\" which is not present in \""
                << orderName << "\"; skipping.";
            continue;
        }

        parse(*fattrMap->second
              , Json::check(attrs[key], Json::objectValue, key.c_str()));
    }
}

void parse(GeometryAttribute::list &gal, const Json::Value &value
           , const std::string &attrName, const std::string &orderName)
{
    if (!value.isMember(attrName)) { return; }
    if (!value.isMember(orderName)) {
        LOGTHROW(err1, Json::RuntimeError)
            << "Missing \"" << orderName << "\" for \"" << attrName << "\".";
    }

    parse(gal, attrName, orderName
          , Json::check(value[attrName], Json::objectValue, attrName.c_str())
          , Json::check(value[orderName], Json::arrayValue, orderName.c_str())
          );
}

void parse(GeometrySchema &gs, const Json::Value &value)
{
    Json::get(gs.geometryType, value, "geometryType");
    Json::get(gs.topology, value, "topology");

    if (value.isMember("header")) {
        parse(gs.header
              , Json::check(value["header"], Json::arrayValue, "header"));
    }

    parse(gs.vertexAttributes, value
          , "vertexAttributes", "ordering");

    parse(gs.faces, value
          , "faces", "ordering");

    parse(gs.featureAttributes, value
          , "featureAttributes", "featureAttributeOrder");
}

void parse(Encoding &encoding, const Json::Value &value, const char *key)
{
    if (value.isMember(key)) {
        Json::get(encoding.mime, value, key);
        encoding.ext = extFromMime(encoding.mime);
    }
}

void parse(Encoding::list &el, const Json::Value &value
           , const char *key)
{
    if (!value.isMember(key)) { return; }
    for (const auto &item : Json::check(value[key], Json::arrayValue, key)) {
        el.emplace_back();
        auto &encoding(el.back());
        encoding.mime = item.asString();
        encoding.ext = extFromMime(encoding.mime);
    }
}

void parse(Store &s, const Json::Value &value)
{
    Json::get(s.id, value, "id");
    Json::get(s.profile, value, "profile");
    Json::get(s.resourcePattern, value, "resourcePattern");
    Json::get(s.rootNode, value, "rootNode");
    Json::get(s.version, value, "version");

    Json::get(s.extents.ll(0), value, "extent", 0);
    Json::get(s.extents.ll(1), value, "extent", 1);
    Json::get(s.extents.ur(0), value, "extent", 2);
    Json::get(s.extents.ur(1), value, "extent", 3);

    Json::get(s.indexCRS, value, "indexCRS");
    Json::get(s.vertexCRS, value, "vertexCRS");
    Json::getOpt(s.normalReferenceFrame, value, "normalReferenceFrame");

    parse(s.nidEncoding, value, "nidEncoding");
    parse(s.featureEncoding, value, "featureEncoding");
    parse(s.geometryEncoding, value, "geometryEncoding");
    parse(s.textureEncoding, value, "textureEncoding");

    Json::getOpt(s.lodType, value, "lodType");
    Json::getOpt(s.lodModel, value, "lodModel");

    parse(s.indexingScheme
          , Json::check(value["indexingScheme"]
                        , Json::objectValue, "indexingScheme"));

    if (value.isMember("defaultGeometrySchema")) {
        s.defaultGeometrySchema = boost::in_place();
        parse(*s.defaultGeometrySchema
              , Json::check(value["defaultGeometrySchema"]
                            , Json::objectValue, "defaultGeometrySchema"));
    }

    // TODO: defaultTextureDefinition
    // TODO: defaultMaterialDefinition
}

SceneLayerInfo loadSceneLayerInfo(std::istream &in, const fs::path &path)
{
    LOG(info1) << "Loading SLPK 3d scene layer info from " << path  << ".";

    const auto value(Json::read(in, path, "SLPK 3d scene layer info"));

    SceneLayerInfo sli;

    Json::get(sli.id, value, "id");
    Json::get(sli.layerType, value, "layerType");

    // spatial reference:
    parse(sli.spatialReference
          , Json::check(value["spatialReference"]
                        , Json::objectValue, "spatialReference"));

    if (value.isMember("heightModelInfo")) {
        parse(sli.heightModelInfo
              , Json::check(value["heightModelInfo"]
                            , Json::objectValue, "heightModelInfo"));
    }

    sli.store = std::make_shared<Store>();
    parse(*sli.store
          , Json::check(value["store"], Json::objectValue, "store"));

    return sli;
}

SceneLayerInfo loadSceneLayerInfo(const roarchive::IStream::pointer &in)
{
    return loadSceneLayerInfo(in->get(), in->path());
}

void parse(MeanBoundingSphere &mbs, const Json::Value &value, const char *name)
{
    Json::get(mbs.x, value, 0, name);
    Json::get(mbs.y, value, 1, name);
    Json::get(mbs.z, value, 2, name);
    Json::get(mbs.r, value, 3, name);
}

void parse(NodeReference &nr, const Json::Value &value, const std::string &dir)
{
    Json::get(nr.id, value, "id");
    parse(nr.mbs, value["mbs"], "mbs");
    Json::get(nr.href, value, "href");
    nr.href = joinPaths(dir, makeDir(nr.href));
    // version
    // featureCount
}

void parse(NodeReference::list &nrl, const Json::Value &value
           , const std::string &dir)
{
    for (const auto &item : value) {
        nrl.emplace_back();
        parse(nrl.back(), item, dir);
    }
}

void parse(Resource &r, const Json::Value &value, const std::string &dir
           , const Encoding &encoding)
{
    Json::get(r.href, value, "href");
    r.href = joinPaths(dir, r.href + encoding.ext);
    r.encoding = &encoding;
    // layerContent
    // featureRange
    Json::getOpt(r.multiTextureBundle, value, "multiTextureBundle");
    Json::getOpt(r.vertexElements, value, "vertexElements");
    Json::getOpt(r.faceElements, value, "faceElements");
}

void parse(Resource::list &rl, const Json::Value &value
           , const std::string &dir
           , const Encoding::list &encodings)
{
    // TODO: check encodings size
    auto iencodings(encodings.begin());
    for (const auto &item : value) {
        rl.emplace_back();
        parse(rl.back(), item, dir, *iencodings++);
    }
}

void parse(Resource::list &rl, const Json::Value &value
           , const std::string &dir
           , const Encoding &encoding)
{
    for (const auto &item : value) {
        rl.emplace_back();
        parse(rl.back(), item, dir, encoding);
    }
}

Node loadNodeIndex(std::istream &in, const fs::path &path
                   , const std::string &dir, const Store::pointer &store)
{
    LOG(info1) << "Loading SLPK 3d node index document from " << path  << ".";
    const auto value(Json::read(in, path, "SLPK 3d Node Index Document"));

    Node ni(store);
    Json::get(ni.id, value, "id");
    Json::get(ni.level, value, "level");
    Json::getOpt(ni.version, value, "version");

    parse(ni.mbs, value["mbs"], "mbs");

    if (value.isMember("parentNode")) {
        ni.parentNode = boost::in_place();
        parse(*ni.parentNode
              , Json::check(value["parentNode"], Json::objectValue
                            , "parentNode")
              , dir);
    }

    if (value.isMember("children")) {
        parse(ni.children
              , Json::check(value["children"], Json::arrayValue, "children")
              , dir);
    }

    if (value.isMember("neighbors")) {
        parse(ni.neighbors
              , Json::check(value["neighbors"], Json::arrayValue, "neighbors")
              , dir);
    }

    if (value.isMember("featureData")) {
        parse(ni.featureData
              , Json::check(value["featureData"], Json::arrayValue
                            , "featureData")
              , dir, store->featureEncoding);
    }

    if (value.isMember("geometryData")) {
        parse(ni.geometryData
              , Json::check(value["geometryData"], Json::arrayValue
                            , "geometryData")
              , dir, store->geometryEncoding);
    }

    if (value.isMember("textureData")) {
        parse(ni.textureData
              , Json::check(value["textureData"], Json::arrayValue
                            , "textureData")
              , dir, store->textureEncoding);
    }

    return ni;
}

Node loadNodeIndex(const roarchive::IStream::pointer &istream
                   , const std::string &dir, const Store::pointer &store)
{
    return loadNodeIndex(istream->get(), istream->path(), dir, store);
}

template <typename T>
void read(std::istream &in, DataType type, T &out)
{
#define READ_DATATYPE(ENUM, TYPE)                               \
    case DataType::ENUM: out = T(bin::read<TYPE>(in)); return

    switch (type) {
        READ_DATATYPE(uint8, std::uint8_t);
        READ_DATATYPE(uint16, std::uint16_t);
        READ_DATATYPE(uint32, std::uint32_t);
        READ_DATATYPE(uint64, std::uint64_t);

        READ_DATATYPE(int8, std::int8_t);
        READ_DATATYPE(int16, std::int16_t);
        READ_DATATYPE(int32, std::int32_t);
        READ_DATATYPE(int64, std::int64_t);

        READ_DATATYPE(float32, float);
        READ_DATATYPE(float64, double);
    }
#undef READ_DATATYPE

    LOGTHROW(err1, std::logic_error)
        << "Invalid datatype (int code="
        << static_cast<int>(type) << ").";
    throw;
}

std::size_t byteCount(const DataType &type)
{
#define MEASURE_DATATYPE(ENUM, TYPE)            \
    case DataType::ENUM: return sizeof(TYPE)

    switch (type) {
        MEASURE_DATATYPE(uint8, std::uint8_t);
        MEASURE_DATATYPE(uint16, std::uint16_t);
        MEASURE_DATATYPE(uint32, std::uint32_t);
        MEASURE_DATATYPE(uint64, std::uint64_t);

        MEASURE_DATATYPE(int8, std::int8_t);
        MEASURE_DATATYPE(int16, std::int16_t);
        MEASURE_DATATYPE(int32, std::int32_t);
        MEASURE_DATATYPE(int64, std::int64_t);

        MEASURE_DATATYPE(float32, float);
        MEASURE_DATATYPE(float64, double);
    }
#undef MEASURE_DATATYPE

LOGTHROW(err1, std::logic_error) << "Invalid datatype.";
    throw;
}

std::size_t byteCount(const GeometryAttribute &ga)
{
    return ga.valuesPerElement * byteCount(ga.valueType);
}

struct Header {
    std::size_t vertexCount;
    std::size_t featureCount;

    Header() : vertexCount(), featureCount() {}
};

Header loadHeader(std::istream &in, const HeaderAttribute::list &has)
{
    Header h;

    for (const auto &ha : has) {
        if (ha.property == "vertexCount") {
            read(in, ha.type, h.vertexCount);
        } else if (ha.property == "featureCount") {
            read(in, ha.type, h.featureCount);
        } else {
            in.ignore(byteCount(ha.type));
        }
    }

    return h;
}

void loadPerAttributeArray(geometry::Mesh &mesh, std::istream &in
                           , const Node &node, const Header &header
                           , const GeometrySchema &schema)
{
    // empty mesh?
    if (!header.vertexCount) { return; }

    if (header.vertexCount % 3) {
        LOGTHROW(err1, std::runtime_error)
            << "Invalid number of vertices in PerAttributeArray layout: "
            "number of vertices (" << header.vertexCount
            << ") not divisible by 3.";
    }

    auto loadVertex([&](const GeometryAttribute &ga, math::Point3 &v)
    {
        read(in, ga.valueType, v(0)); v[0] += node.mbs.x;
        read(in, ga.valueType, v(1)); v[1] += node.mbs.y;
        read(in, ga.valueType, v(2)); v[2] += node.mbs.z;
    });

    auto loadTxCoord([&](const GeometryAttribute &ga, math::Point2 &t)
    {
        read(in, ga.valueType, t(0));
        read(in, ga.valueType, t(1));
    });

    for (const auto &ga : schema.vertexAttributes) {
        if (ga.key == "position") {
            if (ga.valuesPerElement != 3) {
                LOGTHROW(err1, std::runtime_error)
                    << "Number of vertex elements must be 3 not "
                    << ga.valuesPerElement << ".";
            }

            LOG(debug) << "Loading data for vertices.";
            mesh.vertices.resize(header.vertexCount);
            for (auto &v : mesh.vertices) {
                loadVertex(ga, v);
            }
        } else if (ga.key == "uv0") {
            if (ga.valuesPerElement != 2) {
                LOGTHROW(err1, std::runtime_error)
                    << "Number of UV elements must be 2 not "
                    << ga.valuesPerElement << ".";
            }

            LOG(debug) << "Loading data for texture coordinates.";
            mesh.tCoords.resize(header.vertexCount);
            for (auto &t : mesh.tCoords) {
                loadTxCoord(ga, t);
            }
        } else {
            // ignore everything else
            LOG(debug)
                << "Ignoring data for unsupported vertex attribute <"
                << ga.key << "> (" << (header.vertexCount * byteCount(ga))
                << " bytes).";
            in.ignore(header.vertexCount * byteCount(ga));
        }
    }

    if (mesh.vertices.empty()) {
        LOGTHROW(err1, std::runtime_error)
            << "No vertex coordinates defined.";
    }

    // 3 vertices -> face
    mesh.faces.resize(header.vertexCount / 3);
    bool hasTx(!mesh.tCoords.empty());

    std::size_t vi(0);
    for (auto &f : mesh.faces) {
        f.a = vi;
        f.b = vi + 1;
        f.c = vi + 2;
        if (hasTx) {
            f.ta = vi;
            f.tb = vi + 1;
            f.tc = vi + 2;
        }
        vi += 3;
    }
}

geometry::Mesh loadMesh(const Node &node, const Resource &resource
                        , std::istream &in, const fs::path &path)
{
    LOG(info1) << "Loading geometry from " << path  << ".";

    const auto &store(node.store());
    if (!store.defaultGeometrySchema) {
        LOGTHROW(err1, std::runtime_error)
            << "Cannot load mesh since there is no default geometry "
            "schema present in the store.";
    }

    const auto &schema(*store.defaultGeometrySchema);

    if (schema.geometryType != GeometryType::triangles) {
        LOGTHROW(err1, std::runtime_error)
            << "Only triangles are supported while loading mesh.";
    }

    geometry::Mesh mesh;

    const auto header(loadHeader(in, schema.header));
    LOG(info1) << "Loaded header: vertexCount=" << header.vertexCount
               << ", featureCount=" << header.featureCount << ".";

    switch (schema.topology) {
    case Topology::perAttributeArray:
        loadPerAttributeArray(mesh, in, node, header, schema);
        break;

    case Topology::interleavedArray:
        LOGTHROW(err1, std::runtime_error)
            << "Cannot read mesh from " << path << "with indexed topology: "
            "unsupported.";
        break;

    case Topology::indexed:
        LOGTHROW(err1, std::runtime_error)
            << "Cannot read mesh from " << path << " with indexed topology: "
            "not implemented yet.";
        break;
    }

    return mesh;

    (void) resource;
}

geometry::Mesh loadMesh(const Node &node, const Resource &resource
                        , const roarchive::IStream::pointer &istream)
{
    return loadMesh(node, resource, istream->get(), istream->path());
}

} // namespace

geo::SrsDefinition SpatialReference::srs() const
{
    // use WKT if available
    if (!wkt.empty()) {
        return geo::SrsDefinition(wkt, geo::SrsDefinition::Type::wkt);
    }

    // construct from wkid
    if (!vcsWkid) {
        // just EPSG id
        return geo::SrsDefinition(wkid);
    }

    // combined
    return geo::SrsDefinition(wkid, vcsWkid);
}

void Store::absolutize(const std::string &cwd)
{
    rootNode = joinPaths(cwd, makeDir(rootNode));
}

void SceneLayerInfo::absolutize(const std::string &cwd)
{
    store->absolutize(cwd);
}

Archive::Archive(const fs::path &root)
    : archive_(root, constants::MetadataName)
    , metadata_(loadMetadata(archive_.istream(constants::MetadataName)))
    , sli_(loadSceneLayerInfo(istream(constants::SceneLayer)))
{
    sli_.absolutize();
}

roarchive::IStream::pointer Archive::istream(const fs::path &path) const
{
    switch (metadata_.resourceCompressionType) {
    case ResourceCompressionType::none:
        return archive_.istream(path);

    case ResourceCompressionType::gzip: {
        const auto gzPath(utility::addExtension(path, constants::gzExt));
        if (archive_.exists(gzPath)) {
            // gz path exists
            return archive_.istream
                (gzPath, [](bio::filtering_istream &fis) {
#if 1
                    // use raw zlib decompressor, tell zlib to autodetect gzip
                    // header
                    bio::zlib_params p;
                    p.window_bits |= 16;
                    fis.push(bio::zlib_decompressor(p));
#else
                    fis.push(bio::gzip_decompressor());
#endif
                });
        }

        return archive_.istream(path);
    } break;
    }

    LOGTHROW(err1, std::runtime_error)
        << "Invalid ResourceCompressionType in metadata.";
    throw;
}

Node Archive::loadNodeIndex(const fs::path &dir) const
{
#if 0
    LOG(info4) << "loading: " << dir;
    std::ostringstream os;
    bio::copy
        (istream(joinPaths(dir.string(), constants::NodeIndex))->get(), os);
    LOG(info4) << os.str();
#endif

    return slpk::loadNodeIndex
        (istream(joinPaths(dir.string(), constants::NodeIndex))
         , dir.string(), sli_.store);
}

Node Archive::loadRootNodeIndex() const
{
    return loadNodeIndex(sli_.store->rootNode);
}

Node::map Archive::loadTree() const
{
    Node::map nodes;

    std::queue<const NodeReference*> queue;
    auto add([&](Node node)
    {
        auto res(nodes.insert(Node::map::value_type
                              (node.id, std::move(node))));
        for (const auto &child : res.first->second.children) {
            queue.push(&child);
        }
    });

    add(loadRootNodeIndex());

    while (!queue.empty()) {
        add(loadNodeIndex(queue.front()->href));
        queue.pop();
    }

    return nodes;
}

geometry::Mesh::list Archive::loadGeometry(const Node &node)
{
    geometry::Mesh::list meshes;
    for (const auto &resource : node.geometryData) {
        meshes.push_back(loadMesh(node, resource, istream(resource.href)));
    }
    return meshes;
}

} // namespace slpk
