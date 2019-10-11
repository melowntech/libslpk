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

#include <ogr_spatialref.h>

#include <map>
#include <queue>
#include <string>
#include <tuple>
#include <fstream>
#include <algorithm>

#include <boost/utility/in_place_factory.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/path.hpp"
#include "utility/streams.hpp"
#include "utility/path.hpp"
#include "utility/uri.hpp"
#include "utility/binaryio.hpp"

#include "imgproc/readimage.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "reader.hpp"
#include "restapi.hpp"
#include "detail/files.hpp"

namespace fs = boost::filesystem;
namespace ba = boost::algorithm;
namespace bio = boost::iostreams;
namespace bin = utility::binaryio;

namespace slpk {

namespace {

struct ExtInfo {
    std::string extension;
    int preference;

    ExtInfo(const std::string &extension = "", int preference = -1)
        : extension(extension), preference(preference)
    {}
};

typedef std::map<std::string, ExtInfo> MIMEMapping;

const MIMEMapping mimeMapping {
    { "application/octet-stream", { ".bin", 0 } }
    , { "image/tiff", { ".tiff", 5 } }
    , { "image/jpeg", { ".jpg", 30 } }
    , { "image/png", { ".png", 20 } }
    , { "image/vnd-ms.dds", { ".bin.dds", -1 } }
};

const std::string defaultGeometryEncoding("application/octet-stream");

// must stay empty
const ExtInfo unknownMimeExt;

const ExtInfo& extFromMime(const std::string &mime) {
    auto process([](const std::string &mime) -> const ExtInfo&
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

PreferredEncoding
preferredTextureEncoding(const Encoding::list &textureEncoding)
{
    PreferredEncoding pe;

    int index = 0;
    for (const auto &encoding : textureEncoding) {
        if (!pe.encoding || (encoding.preference > pe.encoding->preference)) {
            pe.encoding = &encoding;
            pe.index = index;
        }

        ++index;
    }

    return pe;
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
    Json::getOpt(hmi.ellipsoid, value, "ellipsoid");
    Json::getOpt(hmi.heightUnit, value, "heightUnit");
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
    Json::getOpt(gs.geometryType, value, "geometryType");
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

/** Parse geometryBuffer like old geometry attributes
 */
void parse(v17::GeometryDefinition &gd, const Json::Value &value)
{
    const auto &getDefinition([&](const char *in, const char *out)
    {
        if (!value.isMember(in)) { return; }
        const auto &src(value[in]);

        gd.attributes.emplace_back(out);
        auto &ga(gd.attributes.back());

        Json::get(ga.valueType, src, "type");
        Json::get(ga.valuesPerElement, src, "component");
    });

    getDefinition("position", "position");
    getDefinition("normal", "normal");
    getDefinition("uv0", "uv0");
    getDefinition("uv1", "uv1");
    getDefinition("color", "color");
    getDefinition("uvRegion", "region");
}

void parse(v17::GeometryDefinition::list &gds, const Json::Value &value)
{
    for (const auto &item : value) {
        // TODO: check presence of geometryBuffers

        const auto &buffers
            (Json::check(item["geometryBuffers"]
                         , Json::arrayValue, "geometryBuffers"));

        if (buffers.size() < 1) {
            LOGTHROW(err1, std::runtime_error)
                << "There must be at least one geometry buffer.";
        }

        const auto &buffer
            (Json::check(buffers[0]
                         , Json::objectValue, "geometryBuffers[0]"));

        gds.emplace_back();
        parse(gds.back(), buffer);
    }
}

void parse(Encoding &encoding, const Json::Value &value, const char *key
           , const boost::optional<std::string> &dfltMime = boost::none)
{
    if (value.isMember(key)) {
        Json::get(encoding.mime, value, key);
    } else if (dfltMime) {
        encoding.mime = *dfltMime;
    } else {
        return;
    }

    const auto &ei(extFromMime(encoding.mime));
    encoding.ext = ei.extension;
    encoding.preference = ei.preference;
}

void parse(Encoding::list &el, const Json::Value &value
           , const char *key)
{
    if (!value.isMember(key)) { return; }
    for (const auto &item : Json::check(value[key], Json::arrayValue, key)) {
        el.emplace_back();
        auto &encoding(el.back());
        encoding.mime = item.asString();
        const auto &ei(extFromMime(encoding.mime));
        encoding.ext = ei.extension;
        encoding.preference = ei.preference;
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
    parse(s.geometryEncoding, value, "geometryEncoding"
          , defaultGeometryEncoding);
    parse(s.textureEncoding, value, "textureEncoding");

    Json::getOpt(s.lodType, value, "lodType");
    Json::getOpt(s.lodModel, value, "lodModel");

    if (value.isMember("indexingScheme")) {
        parse(s.indexingScheme
              , Json::check(value["indexingScheme"]
                            , Json::objectValue, "indexingScheme"));
    }

    if (value.isMember("defaultGeometrySchema")) {
        s.defaultGeometrySchema = boost::in_place();
        parse(*s.defaultGeometrySchema
              , Json::check(value["defaultGeometrySchema"]
                            , Json::objectValue, "defaultGeometrySchema"));
    }

    // TODO: defaultTextureDefinition
    // TODO: defaultMaterialDefinition
}

std::pair<SceneLayerInfo, boost::any>
loadSceneLayerInfo(std::istream &in, const fs::path &path)
{
    LOG(info1) << "Loading SLPK 3d scene layer info from " << path  << ".";


    const auto value(Json::read(in, path, "SLPK 3d scene layer info"));

    std::pair<SceneLayerInfo, boost::any> out;
    out.second = value;
    auto &sli(out.first);

    Json::get(sli.id, value, "id");
    Json::get(sli.layerType, value, "layerType");

    if (!Json::getOpt(sli.href, value, "href")) {
        // use default
        sli.href = "./layers/0";
    }

    // spatial reference:
    if (value.isMember("spatialReference")) {
        parse(sli.spatialReference
              , Json::check(value["spatialReference"]
                             , Json::objectValue, "spatialReference"));
    }

    if (value.isMember("heightModelInfo")) {
        parse(sli.heightModelInfo
              , Json::check(value["heightModelInfo"]
                            , Json::objectValue, "heightModelInfo"));
    }

    parse(*sli.store
          , Json::check(value["store"], Json::objectValue, "store"));

    // v1.7
    if (value.isMember("geometryDefinitions")) {
        parse(sli.geometryDefinitions
              , Json::check(value["geometryDefinitions"]
                            , Json::arrayValue, "geometryDefinitions"));
    }

    return out;
}

std::pair<SceneLayerInfo, boost::any>
loadSceneLayerInfo(const roarchive::IStream::pointer &in)
{
    return loadSceneLayerInfo(in->get(), in->path());
}

void parse(MinimumBoundingSphere &mbs, const Json::Value &value
           , const char *name)
{
    Json::get(mbs.center(0), value, 0, name);
    Json::get(mbs.center(1), value, 1, name);
    Json::get(mbs.center(2), value, 2, name);
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

void parse(boost::optional<NodeReference> &nr, const Json::Value &value
           , const std::string &dir)
{
    if (value.isNull()) { return; }

    nr = boost::in_place();
    parse(*nr, value, dir);
}

void parse(NodeReference::list &nrl, const Json::Value &value
           , const std::string &dir)
{
    if (value.isNull()) { return; }

    for (const auto &item : value) {
        nrl.emplace_back();
        parse(nrl.back(), item, dir);
    }
}

void parse(Resource &r, const Json::Value &value, const std::string &dir
           , const Encoding *encoding = nullptr
           , bool isDir = false)
{
    Json::get(r.href, value, "href");

    r.href = joinPaths(dir, r.href);
    if (isDir) { makeDirInplace(r.href); }

    if (encoding) { r.encoding = encoding; }

    // layerContent
    // featureRange
    Json::getOpt(r.multiTextureBundle, value, "multiTextureBundle");
    Json::getOpt(r.vertexElements, value, "vertexElements");
    Json::getOpt(r.faceElements, value, "faceElements");
}

void parse(boost::optional<Resource> &r, const Json::Value &value
           , const std::string &dir, const Encoding *encoding = nullptr
           , bool isDir = false)
{
    if (value.isNull()) { return; }

    r = boost::in_place();
    parse(*r, value, dir, encoding, isDir);
}

void parse(Resource::list &rl, const Json::Value &value
           , const std::string &dir
           , const Encoding::list &encodings
           , bool isDir = false)
{
    if (value.isNull()) { return; }

    if (value.size() > encodings.size()) {
        LOGTHROW(err1, std::runtime_error)
            << "Unexpected number of resources ("
            << value.size() << " but expected " << encodings.size()
            << ".";
    }

    if (value.size() < encodings.size()) {
        LOG(warn1)
            << "Unexpected number of resources ("
            << value.size() << " but expected " << encodings.size()
            << ". Ingoring.";
    }

    auto iencodings(encodings.begin());
    for (const auto &item : value) {
        rl.emplace_back();
        parse(rl.back(), item, dir, &*iencodings++, isDir);
    }
}

void parse(Resource::list &rl, const Json::Value &value
           , const std::string &dir
           , const Encoding &encoding, bool isDir = false)
{
    for (const auto &item : value) {
        rl.emplace_back();
        parse(rl.back(), item, dir, &encoding, isDir);
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

    parse(ni.parentNode, Json::check(Json::Null
                                     , value["parentNode"], Json::objectValue
                                     , "parentNode")
          , dir);

    parse(ni.children
          , Json::check(Json::Null, value["children"]
                        , Json::arrayValue, "children")
          , dir);

    parse(ni.neighbors
          , Json::check(Json::Null, value["neighbors"]
                        , Json::arrayValue, "neighbors")
          , dir);

    parse(ni.sharedResource
          , Json::check(Json::Null, value["sharedResource"]
                        , Json::objectValue, "sharedResource")
          , dir, nullptr, true);

    parse(ni.featureData
          , Json::check(Json::Null, value["featureData"]
                        , Json::arrayValue, "featureData")
          , dir, store->featureEncoding);

    parse(ni.geometryData
          , Json::check(Json::Null, value["geometryData"]
                        , Json::arrayValue, "geometryData")
          , dir, store->geometryEncoding);

    parse(ni.textureData
          , Json::check(Json::Null, value["textureData"]
                        , Json::arrayValue, "textureData")
          , dir, store->textureEncoding);

    return ni;
}

Node loadNodeIndex(const roarchive::IStream::pointer &istream
                   , const std::string &dir, const Store::pointer &store)
{
    return loadNodeIndex(istream->get(), istream->path(), dir, store);
}

void parse(Material::Params &params, const Json::Value &value)
{
    Json::get(params.renderMode, value, "renderMode");
    Json::getOpt(params.vertexRegions, value, "vertexRegions");
    Json::getOpt(params.vertexColors, value, "vertexColors");
}

void parse(Material &material, const Json::Value &value)
{
    Json::get(material.name, value, "name");
    Json::getOpt(material.type, value, "type");

    if (value.isMember("params")) {
        parse(material.params, value["params"]);
    }
}

void parse(Material::list &materials, const Json::Value &value)
{
    for (const auto &name : value.getMemberNames()) {
        materials.emplace_back(name);
        parse(materials.back(), value[name]);
    }
}

SharedResource loadSharedResource(std::istream &in, const fs::path &path)
{
    LOG(info1) << "Loading SLPK Shared Rsource from " << path  << ".";
    const auto value(Json::read(in, path, "SLPK Shared Resource"));

    SharedResource sr;

    parse(sr.materialDefinitions
          , Json::check(Json::Null, value["materialDefinitions"]
                        , Json::objectValue, "materialDefinitions"));

    return sr;
}

SharedResource loadSharedResource(const roarchive::IStream::pointer &istream)
{
    return loadSharedResource(istream->get(), istream->path());
}

template <bool normalize> struct Normalize {};

template <>
struct Normalize<true> {
    template <typename DstT, typename SrcT>
    static DstT apply(SrcT value) {
        return DstT(value) / std::numeric_limits<SrcT>::max();
    }
};

template <>
struct Normalize<false> {
    template <typename DstT, typename SrcT>
    static DstT apply(SrcT value) {
        return DstT(value);
    }
};

template <typename T, bool normalize = false>
void read(std::istream &in, DataType type, T &out)
{
#define READ_DATATYPE(ENUM, TYPE)                                           \
    case DataType::ENUM:                                                    \
        out = Normalize<normalize>::template apply<T>(bin::read<TYPE>(in)); \
        return

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

template <typename T, bool normalize = false>
T read(std::istream &in, DataType type)
{
    T out;
    read<T, normalize>(in, type, out);
    return out;
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
    std::size_t faceCount;
    std::size_t featureCount;

    Header() : vertexCount(), faceCount(), featureCount() {}
};

Header loadHeader(std::istream &in, const HeaderAttribute::list &has)
{
    Header h;

    for (const auto &ha : has) {
        if (ha.property == "vertexCount") {
            read(in, ha.type, h.vertexCount);
        } else if (ha.property == "faceCount") {
            read(in, ha.type, h.faceCount);
        } else if (ha.property == "featureCount") {
            read(in, ha.type, h.featureCount);
        } else {
            in.ignore(byteCount(ha.type));
        }
    }

    return h;
}

struct MeshFeatures {
    bool hasRegions = false;
    bool hasColor = false;
};

namespace paa {

struct CompareRegion {
    bool operator()(const Region &r1, const Region &r2) {
        if (r1.ll < r2.ll) { return true; }
        if (r2.ll < r1.ll) { return false; }
        return r1.ur < r2.ur;
    }
};

template <typename AddValueType, typename T>
void callAddValue(MeshLoader &loader, const AddValueType &addValue
                  , const T &value)
{
    (loader.*addValue)(value);
}

template <typename AddValueType, typename T1, typename T2>
void callAddValue(MeshLoader &loader, const AddValueType &addValue
                  , const std::tuple<T1, T2> &value)
{
    (loader.*addValue)(std::get<0>(value));
}

class Fuser {
public:
    Fuser(std::istream &in, MeshLoader &loader, const Node &node
          , const Header &header, const MeshFeatures &features)
        : in_(in), loader_(loader), node_(node)
        , header_(header), features_(features)
        , verticesLoaded_(false)
        , faces_(header.vertexCount / 3)
        , facesNormal_(header.vertexCount / 3)
        , facesTc_(header.vertexCount / 3)
    {
        if (header.vertexCount % 3) {
            LOGTHROW(err1, std::runtime_error)
                << "Invalid number of vertices in PerAttributeArray layout: "
                "number of vertices (" << header.vertexCount
                << ") not divisible by 3.";
        }
    }

    virtual ~Fuser() {}

    void operator()(const GeometrySchema &schema) {
        for (const auto &ga : schema.vertexAttributes) {
            if (ga.key == "position") {
                loadFaces(ga);
                verticesLoaded_ = true;
            } else if (ga.key == "uv0") {
                loadTcFaces(ga);
            } else if (ga.key == "normal") {
                loadNormals(ga);
                //ignore(ga);
            } else if (ga.key == "region") {
                loadTxRegions(ga);
            } else {
                ignore(ga);
            }
        }

        if (!verticesLoaded_) {
            LOGTHROW(err1, std::runtime_error)
                << "No vertex coordinates defined.";
        }

#if 0
        // do we need features?
        for (const auto &fa : schema.featureAttributes) {
            if (fa.key == "id") {
                for (std::size_t i(0); i < header_.featureCount; ++i) {
                    LOG(info4)
                        << "id: " << read<std::size_t>(in_, fa.valueType);
                }
            } else if (fa.key == "faceRange") {
                for (std::size_t i(0); i < header_.featureCount; ++i) {
                    auto rmin(read<std::size_t>(in_, fa.valueType));
                    auto rmax(read<std::size_t>(in_, fa.valueType));
                    LOG(info4)
                        << "range: " << rmin << ", " << rmax;
                }
            } else {
                ignore(fa);
            }
        }
#endif

        // finalize lod
        finalize();

        // feed loader with faces
        auto ifacesTc(facesTc_.begin());
        auto ifacesNormal(facesNormal_.begin());
        for (const auto &face : faces_) {
            loader_.addFace(face, *ifacesTc++, *ifacesNormal++);
        }
    }

protected:
    /** Adds new value to an index. Returns position of exiting entry if value
     *  was already added. Otherwise inserts new entery and returns its
     *  position in the index.
     *
     *  Position is computed from size of index or via provided counter.
     */
    template <typename Index, typename AddValueType, typename T>
    int add(Index &index, const AddValueType &addValue
            , const T &value, std::size_t *counter = nullptr)
    {
        auto findex(index.find(value));
        if (findex != index.end()) { return findex->second; }
        const auto i(counter ? (*counter)++ : index.size());
        index.insert(typename Index::value_type(value, i));
        callAddValue(loader_, addValue, value);
        return i;
    }

    int vertex(const GeometryAttribute &ga) {
        math::Point3d point;
        read(in_, ga.valueType, point(0)); point(0) += node_.mbs.center(0);
        read(in_, ga.valueType, point(1)); point(1) += node_.mbs.center(1);
        read(in_, ga.valueType, point(2)); point(2) += node_.mbs.center(2);
        return add(vertices_, &MeshLoader::addVertex, point);
    }

    int normal(const GeometryAttribute &ga) {
        math::Point3d point;
        read(in_, ga.valueType, point(0));
        read(in_, ga.valueType, point(1));
        read(in_, ga.valueType, point(2));
        return add(normals_, &MeshLoader::addNormal, point);
    }

    void loadFaces(const GeometryAttribute &ga) {
        if (ga.valuesPerElement != 3) {
            LOGTHROW(err1, std::runtime_error)
                << "Number of vertex elements must be 3, not "
                << ga.valuesPerElement << ".";
        }

        LOG(debug) << "Loading data for vertices.";
        for (auto &face : faces_) {
            face(0) = vertex(ga);
            face(1) = vertex(ga);
            face(2) = vertex(ga);
        }
    }

    void loadNormals(const GeometryAttribute &ga) {
        if (ga.valuesPerElement != 3) {
            LOGTHROW(err1, std::runtime_error)
                << "Number of normal elements must be 3, not "
                << ga.valuesPerElement << ".";
        }

        LOG(debug) << "Loading data for normals.";
        for (auto &face : facesNormal_) {
            face(0) = normal(ga);
            face(1) = normal(ga);
            face(2) = normal(ga);
        }
    }

    virtual void loadTcFaces(const GeometryAttribute &ga) = 0;
    virtual void loadTxRegions(const GeometryAttribute &ga) { ignore(ga); }
    virtual void finalize() = 0;

    void ignore(const GeometryAttribute &ga) {
        LOG(debug)
            << "Ignoring data for unsupported vertex attribute <"
            << ga.key << "> (" << (header_.vertexCount * byteCount(ga))
            << " bytes).";
        in_.ignore(header_.vertexCount * byteCount(ga));
    }

    std::istream &in_;
    MeshLoader &loader_;
    const Node &node_;
    const Header &header_;
    const MeshFeatures &features_;
    bool verticesLoaded_;

    typedef std::map<math::Point3d, int> VertexMap;
    typedef std::map<math::Point2d, int> TextureMap;
    typedef std::vector<TextureMap> TextureMaps;
    typedef std::map<Region, int, CompareRegion> RegionMap;

    Faces faces_;
    Faces facesNormal_;
    FacesTc facesTc_;
    VertexMap vertices_;
    VertexMap normals_;
};

class SimpleFuser : public Fuser {
public:
    SimpleFuser(std::istream &in, MeshLoader &loader, const Node &node
                , const Header &header, const MeshFeatures &features)
        : Fuser(in, loader, node, header, features)
    {}

private:
    void loadTcFaces(const GeometryAttribute &ga) {
        if (ga.valuesPerElement != 2) {
            LOGTHROW(err1, std::runtime_error)
                << "Number of UV elements must be 2, not "
                << ga.valuesPerElement << ".";
        }

        LOG(debug) << "Loading data for texture coordinates.";
        for (auto &face : facesTc_) {
            face(0) = tc(ga);
            face(1) = tc(ga);
            face(2) = tc(ga);
        }
    }

    void finalize() {}

    int tc(const GeometryAttribute &ga)  {
        math::Point2d point;
        read(in_, ga.valueType, point(0));
        read(in_, ga.valueType, point(1));

        // flip Y coord
        point(1) = 1.0 - point(1);
        return add(tc_, &MeshLoader::addTexture, point);
    }

    TextureMap tc_;
};

class RegionFuser : public Fuser {
public:
    RegionFuser(std::istream &in, MeshLoader &loader, const Node &node
                , const Header &header, const MeshFeatures &features)
        : Fuser(in, loader, node, header, features)
        , regions_(header.vertexCount)
    {}

private:
    void loadTcFaces(const GeometryAttribute &ga) {
        if (ga.valuesPerElement != 2) {
            LOGTHROW(err1, std::runtime_error)
                << "Number of UV elements must be 2, not "
                << ga.valuesPerElement << ".";
        }

        LOG(debug) << "Loading data for texture coordinates.";
        for (auto &face : facesTc_) {
            face(0) = tc(ga);
            face(1) = tc(ga);
            face(2) = tc(ga);
        }
    }

    int tc(const GeometryAttribute &ga)  {
        const auto index(tc_.size());
        tc_.emplace_back();
        auto &point(tc_.back());

        read(in_, ga.valueType, point(0));
        read(in_, ga.valueType, point(1));
        // flip Y coord
        point(1) = 1.0 - point(1);

        return index;
    }

    void loadTxRegions(const GeometryAttribute &ga) {
        if (ga.valuesPerElement != 4) {
            LOGTHROW(err1, std::runtime_error)
                << "Number of region elements must be 4, not "
                << ga.valuesPerElement << ".";
        }

        LOG(debug) << "Loading data for texture regions.";
        for (auto &regionIndex : regions_) {
            Region region;
            region.ll(0) = read<double, true>(in_, ga.valueType);
            region.ll(1) = read<double, true>(in_, ga.valueType);
            region.ur(0) = read<double, true>(in_, ga.valueType);
            region.ur(1) = read<double, true>(in_, ga.valueType);

            regionIndex = add(regionMap_, &MeshLoader::addTxRegion, region);
        }
#if 0
        for (const auto &item : regionMap_) {
            LOG(info4) << item.first << " -> " << item.second;
        }
#endif
    }

    void finalize() {
        // generate texture coordinates

        // multiple texturing maps, one per region
        TextureMaps tcMaps(regionMap_.size());

        auto itc(tc_.begin());
        auto iregions(regions_.begin());
        std::size_t counter(0);
        for (auto &face : facesTc_) {
            const auto &region(*iregions++);
            const auto &region2(*iregions++);
            const auto &region3(*iregions++);

            auto &tcMap(tcMaps[region]);
            face.region = region;
            face(0) = add(tcMap, &MeshLoader::addTexture, *itc++, &counter);
            face(1) = add(tcMap, &MeshLoader::addTexture, *itc++, &counter);
            face(2) = add(tcMap, &MeshLoader::addTexture, *itc++, &counter);

            if ((region != region2) || (region2 != region3)
                || (region3 != region))
            {
                LOG(warn1)
                    << "Face (" << face(0) << ", " << face(1)
                    << ", " << face(2) << ") references texture coordinates "
                    << "inside different texture subregions ("
                    << region << ", " << region2 << ", " << region3 << ").";
            }
        }
    }

    math::Points2d tc_;
    RegionMap regionMap_;
    std::vector<int> regions_;
};

void load(MeshLoader &loader, std::istream &in
          , const Node &node, const Header &header
          , const MeshFeatures &features
          , const GeometrySchema &schema)
{
    // empty mesh?
    if (!header.vertexCount) { return; }

    if (features.hasRegions && has(schema.vertexAttributes, "reions")) {
        // we need to take UV (sub) regions into account
        RegionFuser(in, loader, node, header, features)(schema);
    } else {
        // good old textured mesh
        SimpleFuser(in, loader, node, header, features)(schema);
    }
}

} // namespace paa

void loadMesh(MeshLoader &loader, const Node &node
              , const MeshFeatures &features
              , const Resource &
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

    const auto header(loadHeader(in, schema.header));
    LOG(info1) << "Loaded header: vertexCount=" << header.vertexCount
               << ", featureCount=" << header.featureCount << ".";

    switch (schema.topology) {
    case Topology::perAttributeArray:
        paa::load(loader, in, node, header, features, schema);
        break;

    case Topology::interleavedArray:
        LOGTHROW(err1, std::runtime_error)
            << "Cannot read mesh from " << path
            << " with interleaved array topology: unsupported.";
        break;

    case Topology::indexed:
        LOGTHROW(err1, std::runtime_error)
            << "Cannot read mesh from " << path << " with indexed topology: "
            "not implemented yet.";
        break;
    }
}

void loadMesh(MeshLoader &loader, const Node &node
              , const MeshFeatures &features
              , const Resource &resource
              , const roarchive::IStream::pointer &istream)
{
    loadMesh(loader, node, features, resource
             , istream->get(), istream->path());
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

void Store::finish(const std::string &cwd)
{
    rootNode = joinPaths(cwd, makeDir(rootNode));
    preferredTextureEncoding_
        = slpk::preferredTextureEncoding(textureEncoding);
}

void SceneLayerInfo::finish(const std::string &cwd)
{
    store->finish(cwd);
}

Archive::Archive(const fs::path &root, const std::string &mime)
    : archive_
      (root, roarchive::OpenOptions().setHint(detail::constants::MetadataName)
       .setMime(mime))
    , metadata_(loadMetadata(archive_.istream
                             (detail::constants::MetadataName)))
{
    std::tie(sli_, rawSli_)
        = loadSceneLayerInfo(istream(detail::constants::SceneLayer));
    sli_.finish();
}

Archive::Archive(roarchive::RoArchive &archive)
    : archive_(archive.applyHint(detail::constants::MetadataName))
    , metadata_(loadMetadata(archive_.istream
                             (detail::constants::MetadataName)))
{
    std::tie(sli_, rawSli_)
        = loadSceneLayerInfo(istream(detail::constants::SceneLayer));
    sli_.finish();
}

roarchive::IStream::pointer Archive::istream(const fs::path &path) const
{
    switch (metadata_.resourceCompressionType) {
    case ResourceCompressionType::none:
        return archive_.istream(path);

    case ResourceCompressionType::gzip: {
        const auto gzPath
            (utility::addExtension(path, detail::constants::ext::gz));
        if (archive_.exists(gzPath)) {
            return archive_.istream
                (gzPath, [](bio::filtering_istream &fis) {
                    // use raw zlib decompressor, tell zlib to autodetect gzip
                    // header
                    bio::zlib_params p;
                    p.window_bits |= 16;
                    fis.push(bio::zlib_decompressor(p));
                });
        }
        return archive_.istream(path);
    } break;
    }

    LOGTHROW(err1, std::runtime_error)
        << "Invalid ResourceCompressionType in metadata.";
    throw;
}

roarchive::IStream::pointer
Archive::istream(const fs::path &path
                 , const std::initializer_list<const char*> &extensions) const
{
    std::size_t left(extensions.size());
    if (!left) { return istream(path); }

    for (const auto &extension : extensions) {
        --left;
        const auto ePath(utility::addExtension(path, extension));

        switch (metadata_.resourceCompressionType) {
        case ResourceCompressionType::none:
            if (left && !archive_.exists(ePath)) { continue; }
            return archive_.istream(path);

        case ResourceCompressionType::gzip: {
            const auto gzPath
                (utility::addExtension(ePath, detail::constants::ext::gz));
            if (archive_.exists(ePath)) {
                return archive_.istream(ePath);
            } else if (left && !archive_.exists(gzPath)) { continue; }

            return archive_.istream
                (gzPath, [](bio::filtering_istream &fis) {
                    // use raw zlib decompressor, tell zlib to autodetect gzip
                    // header
                    bio::zlib_params p;
                    p.window_bits |= 16;
                    fis.push(bio::zlib_decompressor(p));
                });
        } break;
        }

        LOGTHROW(err1, std::runtime_error)
            << "Invalid ResourceCompressionType in metadata.";
        throw;
    }

    LOGTHROW(err1, std::logic_error)
        << "Error obtaining stream.";
    throw;
}

fs::path Archive::realPath(const boost::filesystem::path &path)
{
    switch (metadata_.resourceCompressionType) {
    case ResourceCompressionType::none:
        return path;

    case ResourceCompressionType::gzip: {
        const auto gzPath
            (utility::addExtension(path, detail::constants::ext::gz));
        if (archive_.exists(gzPath)) { return gzPath; }
        return path;
    } break;
    }

    LOGTHROW(err1, std::runtime_error)
        << "Invalid ResourceCompressionType in metadata.";
    throw;
}

roarchive::IStream::pointer Archive::rawistream(const fs::path &path) const
{
    return archive_.istream(path);
}

Node Archive::loadNodeIndex(const fs::path &dir
                            , boost::filesystem::path *path) const
{
    auto is(istream(joinPaths(dir.string(), detail::constants::NodeIndex)));
    if (path) { *path = is->index(); }
    return slpk::loadNodeIndex(is, dir.string(), sli_.store);
}

SharedResource Archive::loadSharedResource(const boost::filesystem::path &dir)
    const
{
    auto is(istream
            (joinPaths(dir.string(), detail::constants::SharedResource)));
    return slpk::loadSharedResource(is);
}

Node Archive::loadRootNodeIndex(boost::filesystem::path *path) const
{
    return loadNodeIndex(sli_.store->rootNode, path);
}

Tree Archive::loadTree() const
{
    Tree tree;

    std::queue<const NodeReference*> queue;
    auto add([&](Node &&node)
    {
        SharedResource::optional sharedResource;
        if (node.hasSharedResource()) {
            sharedResource = loadSharedResource
                (node.sharedResource->href);
        }

        auto res(tree.nodes.emplace
                 (std::piecewise_construct
                  , std::forward_as_tuple(node.id)
                  , std::forward_as_tuple(std::move(node)
                                          , std::move(sharedResource))));
        for (const auto &child : res.first->second.node.children) {
            queue.push(&child);
        }
    });

    // load root and remember id
    {
        fs::path path;
        auto root(loadRootNodeIndex(&path));
        tree.rootNodeId = root.id;
        add(std::move(root));
    }

    while (!queue.empty()) {
        fs::path path;
        auto node(loadNodeIndex(queue.front()->href, &path));
        add(std::move(node));
        queue.pop();
    }

    return tree;
}

NodeInfo::list Archive::loadNodes() const
{
    NodeInfo::list nodes;

    std::queue<std::string> queue;
    queue.push(sli_.store->rootNode);

    boost::filesystem::path path;
    while (!queue.empty()) {
        const auto &href(queue.front());
        auto node(loadNodeIndex(href, &path));
        nodes.emplace_back(href, path.string(), std::move(node));
        for (const auto &child : nodes.back().node.children) {
            queue.push(child.href);
        }
        queue.pop();
    }

    return nodes;
}

void Archive::loadGeometry(GeometryLoader &loader, const Node &node
                           , const SharedResource::optional &sharedResource)
    const
{
    // TODO: features should come from features but... let's wait for v1.7
    MeshFeatures features;
    if (sharedResource) {
        if (!sharedResource->materialDefinitions.empty()) {
            // use first material
            const auto &material
                (*sharedResource->materialDefinitions.begin());
            features.hasRegions = material.params.vertexRegions;
        }
    }

    for (const auto &resource : node.geometryData) {
        loadMesh(loader.next(), node, features, resource
                 , istream(resource.href + ".bin"));
    }
}

namespace {

class SimpleMeshLoader
    : public GeometryLoader
    , public MeshLoader
{
public:
    SimpleMeshLoader(std::size_t count)
        : current_(nullptr)
    {
        mesh_.submeshes.resize(count);
    }

    virtual MeshLoader& next() {
        if (!current_) {
            current_ = mesh_.submeshes.data();
        } else {
            ++current_;
        }
        return *this;
    }

    Mesh&& moveout() {
        return std::move(mesh_);
    }

    virtual void addVertex(const math::Point3d &v) {
        current_->mesh.vertices.push_back(v);
    }

    virtual void addTexture(const math::Point2d &t) {
        current_->mesh.tCoords.push_back(t);
    }

    virtual void addFace(const Face &mesh, const FaceTc &tc, const Face&)
    {
        current_->mesh.faces.emplace_back(mesh(0), mesh(1), mesh(2)
                                          , tc(0), tc(1), tc(2)
                                          , tc.region);
    }

    virtual void addTxRegion(const Region &r) {
        current_->regions.push_back(r);
    }

private:
    virtual void addNormal(const math::Point3d&) {}

    Mesh mesh_;
    SubMesh* current_;
};

} // namespace

Mesh Archive::loadGeometry(const Node &node
                           , const SharedResource::optional &sharedResource)
    const
{
    SimpleMeshLoader loader(node.geometryData.size());
    loadGeometry(loader, node, sharedResource);
    return loader.moveout();
}

roarchive::IStream::pointer Archive::texture(const Node &node, int index) const
{
    const auto& pe(node.store().preferredTextureEncoding());

    if (!pe.encoding) {
        LOGTHROW(err1, std::runtime_error)
            << "No supported texture available for node <" << node.id << ">.";
    }

    // calculate index in provided textures
    const auto i(index * node.store().textureEncoding.size() + pe.index);

    if (i >= node.textureData.size()) {
        LOGTHROW(err1, std::runtime_error)
            << "Not enough data to get texture of type <"
            << pe.encoding->mime << "> from node <" <<  node.id << ">.";
    }

    // return stream
    return istream
        (node.textureData[i].href, { ".bin", pe.encoding->ext.c_str() });
}

math::Size2 Archive::textureSize(const Node &node, int index) const
{
    const auto& pe(node.store().preferredTextureEncoding());

    if (!pe.encoding) {
        LOGTHROW(err1, std::runtime_error)
            << "No supported texture available for node <" << node.id << ">.";
    }

    // calculate index in provided textures
    const auto i(index * node.store().textureEncoding.size() + pe.index);

    if (i >= node.textureData.size()) {
        LOGTHROW(err1, std::runtime_error)
            << "Not enough data to get texture of type <"
            << pe.encoding->mime << "> from node <" <<  node.id << ">.";
    }

    // get file stream
    auto is(istream(node.textureData[i].href
                    , { ".bin", pe.encoding->ext.c_str() }));

    // try to measure the image
    return imgproc::imageSize(*is, is->path());
}

geo::SrsDefinition Archive::srs() const
{
    return sli_.spatialReference.srs();
}

roarchive::Files Archive::fileList() const
{
    return archive_.list();
}

RestApi::RestApi(Archive &&archive)
    : archive_(std::move(archive))
{
    const auto add([&](const std::string &path, const ApiFile &af)
    {
        files_.insert(ApiFile::map::value_type(path, af));
    });

    const auto addSlashed([&](const fs::path &path, const ApiFile &af)
    {
        auto spath(path.string());
        add(spath, af);
        if (spath.back() == '/') {
            spath.resize(spath.size() - 1);
        } else {
            spath.push_back('/');
        }
        add(spath, af);
    });

    // build SceneServer
    {
        std::ostringstream os;

        Json::Value config(Json::objectValue);
        config["serviceName"] = "SceneService";
        config["serviceVersion"] = "1.4";
        (config["supportedBindings"] = Json::arrayValue).append("REST");
        (config["supportedOperations"] = Json::arrayValue).append("BASE");

        auto &layers(config["layers"] = Json::arrayValue);
        layers.append(boost::any_cast<const Json::Value&>
                      (archive_.rawSceneLayerInfo()));

        Json::write(os, config, false);

        ApiFile af;
        af.contentType = "text/plain;charset=utf-8";
        af.content = os.str();
        addSlashed(constants::SceneServer, af);
    }

    const auto &sli(archive_.sceneLayerInfo());

    // build layer prefix
    const fs::path layerPrefix
        = utility::Uri::joinAndRemoveDotSegments
        ("/" + constants::SceneServer + "/", sli.href)
        .substr(1);

    const auto buildApiFile([&](ApiFile af) -> ApiFile
    {
        auto path(af.path);
        auto ext(path.extension());
        bool gzipped(ext == detail::constants::ext::gz);

        if (gzipped) {
            af.transferEncoding = "gzip";
            path.replace_extension();
            ext = path.extension();
        }

        if (ext == detail::constants::ext::json) {
            af.contentType = "text/plain;charset=utf-8";
        }

        return af;
    });

    addSlashed
        (layerPrefix, buildApiFile
         (archive_.realPath(detail::constants::SceneLayer)));

    typedef std::map<std::string, fs::path> BasePathMap;
    BasePathMap basePathMap;
    for (const auto &path : archive_.fileList()) {
        auto fname(path.filename().string());
        auto dot(fname.find('.'));
        if (dot == std::string::npos) {
            // no dot, as is
            basePathMap.insert(BasePathMap::value_type(path.string(), path));
            continue;
        }

        // trim all extensions
        basePathMap.insert
            (BasePathMap::value_type
             ((path.parent_path() / fname.substr(0, dot)).string(), path));
    }

    const auto addResource([&](const Resource &resource) -> void
    {
        const auto href(resource.href);
        auto fbasePathMap(basePathMap.find(href));
        if (fbasePathMap == basePathMap.end()) { return; }

        // compose API file
        ApiFile af(fbasePathMap->second);
        af.contentType = resource.encoding->mime;
        add((layerPrefix / fbasePathMap->first).string(), buildApiFile(af));
    });

    const auto addResources([&](const Resource::list &resources)
    {
        for (const auto &resource : resources) { addResource(resource); }
    });

    for (const auto &ni : archive_.loadNodes()) {
        addSlashed(layerPrefix / ni.href, buildApiFile(fs::path(ni.fullpath)));

        if (ni.node.sharedResource) {
            const fs::path path(ni.node.sharedResource->href);
            addSlashed
                (layerPrefix / path, buildApiFile
                 (archive_.realPath
                  (path / detail::constants::SharedResource)));
        }
        addResources(ni.node.featureData);
        addResources(ni.node.geometryData);
        addResources(ni.node.textureData);
        // TODO: geometry, store, etc
    }
}

std::pair<roarchive::IStream::pointer, const ApiFile*>
RestApi::file(const boost::filesystem::path &path) const
{
    // try to find file
    auto ffiles(files_.find(path.string()));

    if (ffiles == files_.end()) {
        LOGTHROW(err1, roarchive::NoSuchFile)
            << "File " << path << " not found in the SLPK archive.";
    }

    std::pair<roarchive::IStream::pointer, const ApiFile*>
        result(roarchive::IStream::pointer(), &ffiles->second);
    if (result.second->content.empty()) {
        result.first = std::move(archive_.rawistream(result.second->path));
    }
    return result;
}

bool Archive::changed() const
{
    return archive_.changed();
}

bool RestApi::changed() const
{
    return archive_.changed();
}

HeightModelInfo::HeightModelInfo(const geo::SrsDefinition &srs)
    : heightModel(HeightModel::ellipsoidal)
    , ellipsoid("unnamed"), heightUnit("meter")
{
    const auto reference(srs.reference());

    const auto *root(reference.GetRoot());
    if (!root) { return; }

    if (const auto *datum = root->GetNode("DATUM")) {
        const auto *spheroid(datum->GetNode("SPHEROID"));
        if (const auto *spheroid0 = spheroid->GetChild(0)) {
            ellipsoid = spheroid0->GetValue();
        }
    }

    if (const auto *vertCs = root->GetNode("VERT_CS")) {
        if (const auto *vertDatum = vertCs->GetNode("VERT_DATUM")) {
            if (const auto *vertDatum0 = vertDatum->GetChild(0)) {
                if (ba::icontains(vertDatum0->GetValue(), "geoid")) {
                    heightModel = HeightModel::orthometric;
                }
            }
        }

        if (const auto *unit = vertCs->GetNode("UNIT")) {
            if (const auto *unit0 = unit->GetChild(0)) {
                heightUnit = unit0->GetValue();
                ba::to_lower(heightUnit);
                // arggis.com doesn't like British English...
                if (heightUnit == "metre") { heightUnit = "meter"; }
            }
        }
    }

}

} // namespace slpk
