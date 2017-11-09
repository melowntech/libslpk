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
#include <tuple>
#include <fstream>
#include <algorithm>
#include <atomic>
#include <mutex>
#include <utility>

#include <boost/utility/in_place_factory.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/path.hpp"
#include "utility/streams.hpp"
#include "utility/path.hpp"
#include "utility/uri.hpp"
#include "utility/binaryio.hpp"
#include "utility/format.hpp"
#include "utility/stl-helpers.hpp"

#include "imgproc/readimage.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "./writer.hpp"
#include "./restapi.hpp"
#include "./detail/files.hpp"

namespace fs = boost::filesystem;
namespace bio = boost::iostreams;
namespace bin = utility::binaryio;

namespace slpk {

TextureSaver::~TextureSaver() {}
MeshSaver::~MeshSaver() {}

namespace {

template <typename T>
std::string asString(const T &value)
{
    return boost::lexical_cast<std::string>(value);
}

// fwd
template <typename T>
void build(Json::Value &value, const std::vector<T> &array);

// builders

void build(Json::Value &value, const Capabilities &capabilities)
{
    value = Json::arrayValue;
    for (auto capability : capabilities) {
        value.append(asString(capability));
    }
}

void build(Json::Value &value, const ResourcePatterns &resourcepatterns)
{
    value = Json::arrayValue;
    for (auto resourcepattern : resourcepatterns) {
        value.append(asString(resourcepattern));
    }
}

void build(Json::Value &value, const SpatialReference &srs)
{
    value = Json::objectValue;
    if (srs.wkid) {
        value["wkid"] = srs.wkid;
        if (!srs.latestWkid) { value["latestWkid"] = srs.wkid; }
    }
    if (srs.latestWkid) { value["latestWkid"] = srs.latestWkid; }

    if (srs.vcsWkid) {
        value["vcsWkid"] = srs.vcsWkid;
        if (!srs.latestVcsWkid) { value["latestVcsWkid"] = srs.vcsWkid; }
    }
    if (srs.latestVcsWkid) { value["latestVcsWkid"] = srs.latestVcsWkid; }

    if (!srs.wkt.empty()) {
        value["wkt"] = srs.wkt;
    } else if (srs.wkid) {
        value["wkt"] = srs.srs().as(geo::SrsDefinition::Type::wkt).srs;
    }
}

void build(Json::Value &value, const HeightModelInfo &hmi)
{
    value = Json::objectValue;
    value["heightModel"] = asString(hmi.heightModel);
    value["ellipsoid"] = hmi.ellipsoid;
    value["heightUnit"] = hmi.heightUnit;
}

void build(Json::Value &value, const FeatureRange &range)
{
    value = Json::arrayValue;
    value.append(range.min);
    value.append(range.max);
}

void build(Json::Value &value, const math::Point3 &p)
{
    value = Json::arrayValue;
    value.append(p(0));
    value.append(p(1));
    value.append(p(2));
}

void build(Json::Value &value, const math::Extents2 &extents)
{
    value = Json::arrayValue;
    value.append(extents.ll(0));
    value.append(extents.ll(1));
    value.append(extents.ur(0));
    value.append(extents.ur(1));
}

void build(Json::Value &value, const math::Extents3 &extents)
{
    value = Json::arrayValue;
    value.append(extents.ll(0));
    value.append(extents.ll(1));
    value.append(extents.ll(2));
    value.append(extents.ur(0));
    value.append(extents.ur(1));
    value.append(extents.ur(2));
}

void build(Json::Value &value, const math::Matrix4 &m)
{
    value = Json::arrayValue;
    for (int j(0); j < 4; ++j) {
        for (int i(0); i < 4; ++i) {
            value.append(m(j, i));
        }
    }
}

void build(Json::Value &value, const Encoding &encoding)
{
    value = encoding.mime;
}

void build(Json::Value &value, const Encoding::list &encodings)
{
    value = Json::arrayValue;
    for (const auto encoding : encodings) { value.append(encoding.mime); }
}

void build(Json::Value &value, const Cardinality &cardinality)
{
    value = Json::arrayValue;
    value.append(cardinality.min);
    value.append(cardinality.max);
}

void build(Json::Value &value, const IndexScheme &indexingScheme)
{
    value = Json::objectValue;
    value["name"] = asString(indexingScheme.name);
    value["inclusive"] = indexingScheme.inclusive;
    value["dimensionality"] = indexingScheme.dimensionality;
    build(value["childrenCardinality"], indexingScheme.childrenCardinality);
    build(value["neighborCardinality"], indexingScheme.neighborCardinality);
}

void build(Json::Value &value, const HeaderAttribute &header)
{
    value = Json::objectValue;
    value["property"] = header.property;
    value["type"] = asString(header.type);
}

void build(Json::Value &value, const GeometryAttribute &attr)
{
    value = Json::objectValue;
    if (attr.byteOffset) { value["byteOffset"] = Json::Int(attr.byteOffset); }
    if (attr.count) { value["count"] = Json::Int(attr.count); }
    value["valueType"] = asString(attr.valueType);
    value["valuesPerElement"] = Json::Int(attr.valuesPerElement);

    // TODO: values?
    if (!attr.componentIndices.empty()) {
        auto jcomponentIndices(value["componentIndices"] = Json::arrayValue);
        for (auto ci : attr.componentIndices) { jcomponentIndices.append(ci); }
    }
}

void build(Json::Value &jattrs, Json::Value &jordering
           , const GeometryAttribute::list &attrs)
{
    jattrs = Json::objectValue;
    jordering = Json::arrayValue;
    for (const auto &attr : attrs) {
        build(jattrs[attr.key], attr);
        jordering.append(attr.key);
    }
}

void build(Json::Value &value, const GeometryAttribute::list &attrs)
{
    value = Json::objectValue;
    for (const auto &attr : attrs) {
        build(value[attr.key], attr);
    }
}

void build(Json::Value &value, const GeometrySchema &geometrySchema)
{
    value = Json::objectValue;
    value["geometryType"] = asString(geometrySchema.geometryType);
    value["topology"] = asString(geometrySchema.topology);
    build(value["header"], geometrySchema.header);

    if (!geometrySchema.vertexAttributes.empty()) {
        build(value["vertexAttributes"], value["ordering"]
              , geometrySchema.vertexAttributes);
    }

    if (!geometrySchema.faces.empty()) {
        build(value["faces"], geometrySchema.faces);
    }

    if (!geometrySchema.featureAttributes.empty()) {
        build(value["featureAttributes"], value["featureAttributeOrder"]
              , geometrySchema.featureAttributes);
    }
}

void build(Json::Value &value, const Store &store, const Metadata &metadata)
{
    value = Json::objectValue;
    value["id"] = store.id;
    value["profile"] = asString(store.profile);
    build(value["resourcePattern"], store.resourcePattern);

    value["rootNode"] = store.rootNode;
    value["version"] = store.version;
    build(value["extent"], store.extents);

    value["normalReferenceFrame"] = asString(store.normalReferenceFrame);

    {
        const auto gzipped
            (metadata.resourceCompressionType
             == ResourceCompressionType::gzip);
        std::string jsonEncoding
            (str(boost::format
                 ("application/vnd.esri.I3S.json%s; version=%d.%d")
                 % (gzipped ? "+gzip" : "")
                 % metadata.version.major
                 % metadata.version.minor));

        value["nidEncoding"] = jsonEncoding;
        value["featureEncoding"] = jsonEncoding;
        value["geometryEncoding"] = jsonEncoding;
    }

    build(value["textureEncoding"], store.textureEncoding);
    value["lodType"] = asString(store.lodType);
    value["lodModel"] = asString(store.lodModel);

    build(value["indexingScheme"], store.indexingScheme);

    if (store.defaultGeometrySchema) {
        build(value["defaultGeometrySchema"], *store.defaultGeometrySchema);
    }
}

void build(Json::Value &value, const Version &version)
{
    value = utility::format("%d.%d", version.major, version.minor);
}

void build(Json::Value &value, const SceneLayerInfo &sli
           , const Metadata &metadata)
{
    value = Json::objectValue;
    value["id"] = sli.id;
    value["layerType"] = asString(sli.layerType);
    value["href"] = sli.href;
    value["name"] = sli.name;
    if (sli.alias) { value["alias"] = *sli.alias; }
    if (sli.description) { value["description"] = *sli.description; }
    if (sli.copyrightText) { value["copyrightText"] = *sli.copyrightText; }

    build(value["capabilities"], sli.capabilities);

    build(value["spatialReference"], sli.spatialReference);

    build(value["heightModelInfo"], sli.heightModelInfo);

    if (sli.store) {
        auto &store(value["store"]);
        build(store, *sli.store, metadata);
        auto crs(str(boost::format("http://www.opengis.net/def/crs/EPSG/0/%d")
                     % sli.spatialReference.wkid));
        store["indexCRS"] = crs;
        store["vertexCRS"] = crs;
        build(store["version"], metadata.version);
    }
}

void build(Json::Value &value, const Metadata &metadata)
{
    value = Json::objectValue;
    value["folderPattern"] = asString(metadata.folderPattern);
    value["ArchiveCompressionType"]
        = asString(metadata.archiveCompressionType);
    value["ResourceCompressionType"]
        = asString(metadata.resourceCompressionType);
    build(value["I3SVersion"], metadata.version);
    value["nodeCount"] = Json::UInt64(metadata.nodeCount);
}

void build(Json::Value &value, const MinimumBoundingSphere &mbs)
{
    value = Json::arrayValue;
    value.append(mbs.center(0));
    value.append(mbs.center(1));
    value.append(mbs.center(2));
    value.append(mbs.r);
}

void build(Json::Value &value, const NodeReference &nodeReference)
{
    value = Json::objectValue;
    value["id"] = nodeReference.id;
    build(value["mbs"], nodeReference.mbs);
    value["href"] = nodeReference.href;
    if (!nodeReference.version.empty()) {
        value["version"] = nodeReference.version;
    }
    if (nodeReference.featureCount) {
        value["featureCount"] = nodeReference.featureCount;
    }
}

void build(Json::Value &value, const Resource &resource)
{
    value = Json::objectValue;
    value["href"] = resource.href;
    // TODO: rest
}

void build(Json::Value &value, const LodSelection &lodSelection)
{
    value = Json::objectValue;
    value["metricType"] = asString(lodSelection.metricType);
    if (lodSelection.maxValue) { value["maxValue"] = lodSelection.maxValue; }
    if (lodSelection.avgValue) { value["avgValue"] = lodSelection.avgValue; }
    if (lodSelection.minValue) { value["minValue"] = lodSelection.minValue; }
    if (lodSelection.maxError) { value["maxError"] = lodSelection.maxError; }
}

void build(Json::Value &value, const Node &node
           , bool standardSharedResource = false)
{
    value = Json::objectValue;
    value["id"] = node.id;
    value["level"] = node.level;
    if (!node.version.empty()) { value["version"] = node.version; }
    build(value["mbs"], node.mbs);

    if (node.parentNode) { build(value["parentNode"], *node.parentNode); }

    if (!node.children.empty()) { build(value["children"], node.children); }
    if (!node.neighbors.empty()) { build(value["neighbors"], node.neighbors); }

    if (node.sharedResource) {
        build(value["sharedResource"], *node.sharedResource);
    } else if (standardSharedResource) {
        build(value["sharedResource"], Resource("./shared"));
    }

    if (!node.featureData.empty()) {
        build(value["featureData"], node.featureData);
    }
    if (!node.geometryData.empty()) {
        build(value["geometryData"], node.geometryData);
    }
    if (!node.textureData.empty()) {
        build(value["textureData"], node.textureData);
    }

    if (!node.lodSelection.empty()) {
        build(value["lodSelection"], node.lodSelection);
    }
}

void build(Json::Value &value, const Color &color)
{
    value = Json::arrayValue;

    value.append(color[0]);
    value.append(color[1]);
    value.append(color[2]);
}

void build(Json::Value &value, const Material &material)
{
    value = Json::objectValue;

    value["name"] = material.name;
    value["type"] = asString(material.type);
    if (!material.ref.empty()) { value["$ref"] = material.ref; }

    auto &params(value["params"] = Json::objectValue);
    params["vertexRegions"] = material.params.vertexRegions;
    params["vertexColors"] = material.params.vertexColors;
    params["useVertexColorAlpha"] = material.params.useVertexColorAlpha;
    params["transparency"] = material.params.transparency;
    params["reflectivity"] = material.params.reflectivity;
    params["shininess"] = material.params.shininess;
    build(params["ambient"], material.params.ambient);
    build(params["difuse"], material.params.difuse);
    build(params["specular"], material.params.specular);
    params["renderMode"] = asString(material.params.renderMode);
    params["castShadows"] = material.params.castShadows;
    params["receiveShadows"] = material.params.receiveShadows;
    params["cullFace"] = asString(material.params.cullFace);
}

void build(Json::Value &value, const Image &image)
{
    value = Json::objectValue;

    value["id"] = image.id;
    value["size"] = Json::UInt64(image.size);
    value["pixelInWorldUnits"] = image.pixelInWorldUnits;

    auto &href(value["href"] = Json::arrayValue);
    auto &byteOffset(value["byteOffset"] = Json::arrayValue);
    auto &length(value["length"] = Json::arrayValue);

    for (const auto &version : image.versions) {
        href.append(version.href);
        byteOffset.append(Json::UInt64(version.byteOffset));
        length.append(Json::UInt64(version.length));
    }
}

void build(Json::Value &value, const Texture &texture)
{
    value = Json::objectValue;

    auto &encoding(value["encoding"] = Json::arrayValue);
    for (const auto &e : texture.encoding) { encoding.append(e.mime); }

    auto &wrap(value["wrap"] = Json::arrayValue);
    wrap.append(asString(texture.wrap[0]));
    wrap.append(asString(texture.wrap[1]));

    value["atlas"] = texture.atlas;
    value["uvSet"] = texture.uvSet;
    value["channels"] = texture.channels;

    auto &images(value["images"] = Json::arrayValue);
    for (const auto &image : texture.images) {
        build(images.append({}), image);
    }
}

void build(Json::Value &value, const SharedResource &sr)
{
    if (!sr.materialDefinitions.empty()) {
        auto &md(value["materialDefinitions"] = Json::objectValue);
        for (const auto &materialDefinition : sr.materialDefinitions) {
            build(md[materialDefinition.key], materialDefinition);
        }
    }

    if (!sr.textureDefinitions.empty()) {
        auto &md(value["textureDefinitions"] = Json::objectValue);
        for (const auto &textureDefinition : sr.textureDefinitions) {
            build(md[textureDefinition.key], textureDefinition);
        }
    }
}

void build(Json::Value &value, const GeometryReference &geometry)
{
    value = Json::objectValue;
    value["type"] = asString(GeometryClass::GeometryReference);

    value["$ref"] = geometry.ref;
    build(value["faceRange"], geometry.faceRange);
    value["lodGeometry"] = geometry.lodGeometry;
}

void build(Json::Value &value, GeometryReference::list &geometries)
{
    value = Json::arrayValue;
    for (const auto geometry : geometries) {
        build(value.append({}), geometry);
    }
}

void build(Json::Value &value, const FeatureData::Feature &feature)
{
    value = Json::objectValue;

    value["id"] = feature.id;
    build(value["position"], feature.position);
    build(value["pivotOffset"], feature.pivotOffset);
    build(value["mbb"], feature.mbb);
    value["layer"] = feature.layer;

    build(value["geometries"], feature.geometries);
}

void build(Json::Value &value, const ArrayBufferView &geometry)
{
    value = Json::objectValue;
    value["type"] = asString(GeometryClass::ArrayBufferView);

    value["id"] = geometry.id;
    build(value["transformation"], geometry.transformation);

    auto &params(value["params"]);
    params["type"] = asString(geometry.type);
    params["topology"] = asString(geometry.topology);
    params["material"] = geometry.material;
    params["texture"] = geometry.texture;

    if (!geometry.vertexAttributes.empty()) {
        build(params["vertexAttributes"], geometry.vertexAttributes);
    }
    if (!geometry.faces.empty()) {
        build(params["faces"], geometry.faces);
    }
}

void build(Json::Value &value, const FeatureData::Feature::list &features)
{
    value = Json::arrayValue;
    for (const auto feature : features) {
        build(value.append({}), feature);
    }
}

void build(Json::Value &value, const ArrayBufferView::list &geometries)
{
    value = Json::arrayValue;
    for (const auto geometry : geometries) {
        build(value.append({}), geometry);
    }
}

void build(Json::Value &value, const FeatureData &fd)
{
    value = Json::objectValue;

    build(value["featureData"], fd.featureData);
    build(value["geometryData"], fd.geometryData);
}

// implementation must be here to see other build functions
template <typename T>
void build(Json::Value &value, const std::vector<T> &array)
{
    value = Json::arrayValue;
    for (const auto &item : array) {
        build(value.append({}), item);
    }
}

template <typename T1, typename T2>
void build(Json::Value &value, const std::tuple<T1, T2> &tupple)
{
    build(value, std::get<0>(tupple), std::get<1>(tupple));
}

template <typename T>
void write(std::ostream &out, DataType type, const T &value)
{
#define WRITE_DATATYPE(ENUM, TYPE)                               \
    case DataType::ENUM: bin::write(out, TYPE(value)); return

    switch (type) {
        WRITE_DATATYPE(uint8, std::uint8_t);
        WRITE_DATATYPE(uint16, std::uint16_t);
        WRITE_DATATYPE(uint32, std::uint32_t);
        WRITE_DATATYPE(uint64, std::uint64_t);

        WRITE_DATATYPE(int8, std::int8_t);
        WRITE_DATATYPE(int16, std::int16_t);
        WRITE_DATATYPE(int32, std::int32_t);
        WRITE_DATATYPE(int64, std::int64_t);

        WRITE_DATATYPE(float32, float);
        WRITE_DATATYPE(float64, double);
    }
#undef WRITE_DATATYPE

    LOGTHROW(err1, std::logic_error)
        << "Invalid datatype (int code="
        << static_cast<int>(type) << ").";
    throw;
}

void write(std::ostream &out, const GeometryAttribute &ga
           , const math::Point3 &p)
{
    write(out, ga.valueType, p(0));
    write(out, ga.valueType, p(1));
    write(out, ga.valueType, p(2));
}

void write(std::ostream &out, const GeometryAttribute &ga
           , const math::Point2 &p)
{
    write(out, ga.valueType, p(0));
    write(out, ga.valueType, p(1));
}

class SavePerAttributeArray {
public:
    SavePerAttributeArray(std::ostream &os, const Node &node
                          , const MeshSaver &meshSaver
                          , const GeometrySchema &gs
                          , FeatureData::Feature &feature
                          , ArrayBufferView &arrayBufferView)
        : os_(os), node_(node), meshSaver_(meshSaver), gs_(gs)
        , feature_(feature), arrayBufferView_(arrayBufferView)
        , properties_(meshSaver_.properties())
        , vertexCount_(3 * properties_.faceCount)
    {
        // store array buffer view info
        arrayBufferView_.type = gs.geometryType;
        arrayBufferView_.topology = gs.topology;

        // save header
        for (const auto &header : gs_.header) {
            if (header.property == "vertexCount") {
                // vertex count
                write(os, header.type, vertexCount_);
            } else if (header.property == "featureCount") {
                // feature count: whole mesh is a single feature
                write(os, header.type, 1);
            } else {
                LOGTHROW(err2, std::runtime_error)
                    << "Header element <" << header.property
                    << "> not supported.";
            }
        }

        // save mesh data
        for (const auto &ga : gs_.vertexAttributes) {
            if (ga.key == "position") {
                auto &oga(utility::append
                          (arrayBufferView_.vertexAttributes, ga));
                oga.byteOffset = os.tellp();
                oga.count = vertexCount_;
                saveFaces(ga);
            } else if (ga.key == "uv0") {
                auto &oga(utility::append
                          (arrayBufferView_.vertexAttributes, ga));
                oga.byteOffset = os.tellp();
                oga.count = vertexCount_;
                saveFacesTc(ga);
            } else {
                LOGTHROW(err2, std::runtime_error)
                    << "Geometry attribute <" << ga.key << "> not supported.";
            }
        }

        // save features
        for (const auto &fa : gs_.featureAttributes) {
            if (fa.key == "id") {
                if (fa.valuesPerElement != 1) {
                    LOGTHROW(err1, std::runtime_error)
                        << "Number of feaure.id elements must be 1 not "
                        << fa.valuesPerElement << ".";
                }

                // ID = 0
                write(os, fa.valueType, 0);

            } else if (fa.key == "faceRange") {
                if (fa.valuesPerElement != 2) {
                    LOGTHROW(err1, std::runtime_error)
                        << "Number of feaure.faceRanage  elements must be "
                        "2 not " << fa.valuesPerElement << ".";
                }

                // whole mesh; doc says inclusive range -> faceCount - 1
                write(os, fa.valueType, 0);
                write(os, fa.valueType, (properties_.faceCount - 1));
            } else {
                LOGTHROW(err2, std::runtime_error)
                    << "Feature attribute <" << fa.key << "> not supported.";
            }
        }

        auto &geometry(utility::append(feature_.geometries));
        geometry.ref = "/geometryData/1";
        geometry.faceRange.min = 0;
        geometry.faceRange.max = (properties_.faceCount - 1);
        geometry.lodGeometry = true;
    }

private:
    void saveFaces(const GeometryAttribute &ga) {
        if (ga.valuesPerElement != 3) {
            LOGTHROW(err1, std::runtime_error)
                << "Number of vertex elements must be 3 not "
                << ga.valuesPerElement << ".";
        }

        feature_.mbb = math::Extents3(math::InvalidExtents{});
        for (std::size_t i(0), e(properties_.faceCount); i != e; ++i) {
            const auto &face(meshSaver_.face(i));
            for (const auto &point : face) {
                // update mesh extents
                math::update(feature_.mbb, point);
                // write localized point
                write(os_, ga, math::Point3(point - node_.mbs.center));
            }
        }
    }

    void saveFacesTc(const GeometryAttribute &ga) {
        if (ga.valuesPerElement != 2) {
            LOGTHROW(err1, std::runtime_error)
                << "Number of UV elements must be 2 not "
                << ga.valuesPerElement << ".";
        }

        for (std::size_t i(0), e(properties_.faceCount); i != e; ++i) {
            const auto &face(meshSaver_.faceTc(i));
            for (const auto &point : face) {
                write(os_, ga, point);
            }
        }
    }

    std::ostream &os_;
    const Node &node_;
    const MeshSaver &meshSaver_;
    const GeometrySchema &gs_;
    FeatureData::Feature &feature_;
    ArrayBufferView &arrayBufferView_;

    const MeshSaver::Properties properties_;

    std::size_t vertexCount_;
};

void saveMesh(std::ostream &os, const Node &node
              , const MeshSaver &meshSaver
              , const GeometrySchema &gs
              , FeatureData::Feature &feature
              , ArrayBufferView &arrayBufferView)
{
    switch (gs.topology) {
    case Topology::perAttributeArray:
        SavePerAttributeArray(os, node, meshSaver, gs
                              , feature, arrayBufferView);
        return;

    default:
        LOGTHROW(err2, std::runtime_error)
            << "Unsupported geomety topology " << gs.topology << ".";
    }
    throw;
}

const GeometrySchema& getGeometrySchema(const SceneLayerInfo &sli)
{
    if (!sli.store || !sli.store->defaultGeometrySchema) {
        LOGTHROW(err2, std::runtime_error)
            << "No default geometry schema present.";
    }

    const auto &gs(*sli.store->defaultGeometrySchema);

    if (gs.geometryType != GeometryType::triangles) {
        LOGTHROW(err2, std::runtime_error)
            << "Cannot store nothing else then trianges geometries.";
    }

    if (gs.topology != Topology::perAttributeArray) {
        LOGTHROW(err2, std::runtime_error)
            << "Cannot store nothing else then PerAttributeArray geometries.";
    }

    return gs;
}

} // namespace

struct Writer::Detail {
    Detail(const boost::filesystem::path &path
           , const Metadata &metadata, const SceneLayerInfo &sli
           , bool overwrite)
        : sli(sli), gs(getGeometrySchema(this->sli)), zip(path, overwrite)
        , metadata(metadata), nodeCount(), textureCount()
    {}

    void flush(const SceneLayerInfoCallback &callback);

    utility::zip::Writer::OStream::pointer
    ostream(const fs::path &path, bool raw = false);

    void write(Node &node, SharedResource &sharedResource
               , const MeshSaver &meshSaver
               , const TextureSaver &textureSaver);

    template <typename T>
    void store(const T &value, const fs::path &path, bool raw = false);

    SceneLayerInfo sli;
    const GeometrySchema &gs;
    utility::zip::Writer zip;
    std::mutex mutex;
    Metadata metadata;

    std::atomic<std::size_t> nodeCount;
    std::atomic<std::size_t> textureCount;
};

template <typename T>
void Writer::Detail::store(const T &value, const fs::path &path, bool raw)
{
    Json::Value jValue;
    build(jValue, value);

    std::unique_lock<std::mutex> lock(mutex);
    auto os(ostream( path, raw));
    Json::write(os->get(), jValue, false);
    os->close();
}

void Writer::Detail::flush(const SceneLayerInfoCallback &callback) {
    // update and store scene layer
    if (callback) { callback(sli); }
    store(std::make_tuple(std::cref(sli), std::cref(metadata))
          , detail::constants::SceneLayer);

    // update and save metadata
    metadata.nodeCount = nodeCount;
    store(metadata, detail::constants::MetadataName, true);

        // done
    zip.close();
}

utility::zip::Writer::OStream::pointer
Writer::Detail::ostream(const fs::path &path, bool raw)
{
    const utility::zip::Compression compression
        ((!raw && (metadata.archiveCompressionType
                   != ArchiveCompressionType::store))
         ? utility::zip::Compression::deflate
         : utility::zip::Compression::store);

    if (!raw && (metadata.resourceCompressionType
                 == ResourceCompressionType::gzip))
    {
        // add .gz extension and push gzip compressor at the top of this
        // filter stack
        return zip.ostream(utility::addExtension
                           (path, detail::constants::ext::gz)
                           , compression
                           , [](bio::filtering_ostream &fos) {
                               bio::zlib_params p;
                               p.window_bits |= 16;
                               fos.push(bio::zlib_compressor(p));
                           });
    }

    return zip.ostream(path, compression);
}

std::uint64_t buildId(std::uint64_t id, const math::Size2 &size
                      , unsigned int l, unsigned int al)
{
    std::uint64_t l_al(std::uint64_t(al) << 60);
    std::uint64_t l_l(std::uint64_t(l) << 56);
    std::uint64_t l_w (std::uint64_t(size.width - 1) << 44);
    std::uint64_t l_h(std::uint64_t(size.height - 1) << 32);
    return  l_al + l_l + l_w + l_h + std::uint64_t(id);
}

void Writer::Detail::write(Node &node, SharedResource &sharedResource
                           , const MeshSaver &meshSaver
                           , const TextureSaver &textureSaver)
{
    if (sharedResource.materialDefinitions.empty()) {
        LOGTHROW(err2, std::runtime_error)
            << "No material defined in shared resource.";
    }

    auto &textures(sharedResource.textureDefinitions);

    if (!textures.empty()) {
        LOGTHROW(err2, std::runtime_error)
            << "Multi texture bundle not supported.";
    }

    const auto txId(textureCount++);

    // add texture
    textures.emplace_back(utility::format("tex%d", txId));
    auto &texture(textures.back());

    // setup texture
    texture.atlas = true;
    texture.uvSet = "uv0";
    texture.channels = "rgb";

    auto &image(utility::append(texture.images));
    const auto imageSize(textureSaver.imageSize());
    image.size = imageSize.width;

    // TODO: what are these levels?
    image.id = asString(buildId(txId, imageSize, 0, 0));

    const auto index(node.geometryData.size());

    // write textures
    int txi(0);
    for (const auto &encoding : sli.store->textureEncoding) {
        // node
        const auto href
            (utility::format("textures/0_%d", txi++));
        auto &td(utility::append(node.textureData, "./" + href));
        td.multiTextureBundle = false;

        // texture
        texture.encoding.push_back(encoding);

        auto &imageVersion(utility::append(image.versions, "../" + href));

        const fs::path texturePath
            (detail::constants::Nodes / node.id / (href + ".bin"));

        std::unique_lock<std::mutex> lock(mutex);
        // TODO: do not report DDS as raw (if ever used)
        auto os(ostream(texturePath, true));
        textureSaver.save(os->get(), encoding.mime);

        const auto stat(os->close());
        imageVersion.length = stat.uncompressedSize;
    }

    // write meshes and features
    const auto mhref(utility::format("geometries/%d", index));
    node.geometryData.emplace_back("./" + mhref);
    const fs::path geometryPath
        (detail::constants::Nodes / node.id / (mhref + ".bin"));

    const auto fhref(utility::format("features/%d", index));
    node.featureData.emplace_back("./" + fhref);
    const fs::path featurePath
        (detail::constants::Nodes / node.id / (fhref + ".json"));

    // save mesh to temporary stream
    std::stringstream tmp;

    // feature stuff
    FeatureData featureData;
    {
        auto &fd(utility::append(featureData.featureData, 0));
        fd.position = node.mbs.center;
        fd.layer = "3D model";

        auto &gd(utility::append(featureData.geometryData, 0));

        // only per-attribute array geometry type
        saveMesh(tmp, node, meshSaver, gs, fd, gd);

        // store references to material and textures
        gd.material = ("/materialDefinitions/"
                       + sharedResource.materialDefinitions.back().key);
        gd.texture = "/textureDefinitions/" + texture.key;
    }

    {
        std::unique_lock<std::mutex> lock(mutex);
        auto os(ostream(geometryPath));
        os->get() << tmp.rdbuf();
        os->close();
    }

    // store feature data
    store(featureData, featurePath);
}

Writer::Writer(const boost::filesystem::path &path
               , const Metadata &metadata, const SceneLayerInfo &sli
               , bool overwrite)
    : detail_(std::make_shared<Detail>(path, metadata, sli, overwrite))
{}

void Writer::write(const Node &node, const SharedResource *sharedResource)
{
    const auto dir(detail::constants::Nodes / node.id);
    detail_->store(std::make_tuple(std::cref(node), bool(sharedResource))
                   , dir / detail::constants::NodeIndex);

    if (sharedResource) {
        detail_->store(*sharedResource
                       , (dir / detail::constants::Shared
                          / detail::constants::SharedResource));
    }

    ++detail_->nodeCount;
}

void Writer::write(Node &node, SharedResource &sharedResource
                   , const MeshSaver &meshSaver
                   , const TextureSaver &textureSaver)
{
    return detail_->write(node, sharedResource, meshSaver, textureSaver);
}

void Writer::flush(const SceneLayerInfoCallback &callback)
{
    detail_->flush(callback);
}

} // namespace slpk
