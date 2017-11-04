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
    if (srs.wkid) { value["wkid"] = srs.wkid; }
    if (srs.latestWkid) { value["vcsWkid"] = srs.vcsWkid; }
    if (srs.vcsWkid) { value["vcsWkid"] = srs.vcsWkid; }
    if (srs.latestVcsWkid) { value["latestVcsWkid"] = srs.latestVcsWkid; }
    if (!srs.wkt.empty()) { value["wkt"] = srs.wkt; }
}

void build(Json::Value &value, const HeightModelInfo &hmi)
{
    value = Json::objectValue;
    value["heightModel"] = asString(hmi.heightModel);
    value["ellipsoid"] = hmi.ellipsoid;
    value["heightUnit"] = hmi.heightUnit;
}

void build(Json::Value &value, const math::Extents2 &extents)
{
    value = Json::arrayValue;
    value.append(extents.ll(0));
    value.append(extents.ll(1));
    value.append(extents.ur(0));
    value.append(extents.ur(1));
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
    build(value["extents"], store.extents);

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

    // TODO: fill me in pls, thx
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
        value["indexCRS"] = crs;
        value["vertexCRS"] = crs;
    }
}

void build(Json::Value &value, const Version &version)
{
    value = str(boost::format("%d.%d") % version.major % version.minor);
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
    value.append(mbs.x);
    value.append(mbs.y);
    value.append(mbs.z);
    value.append(mbs.r);
}

void build(Json::Value &value, const NodeReference &nodeReference)
{
    value = Json::objectValue;
    value["id"] = nodeReference.id;
    build(value["mbs"], nodeReference.mbs);
    value["href"] = nodeReference.href;
    value["version"] = nodeReference.version;
    value["featureCount"] = nodeReference.featureCount;
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

void build(Json::Value &value, const Node &node)
{
    value = Json::objectValue;
    value["id"] = node.id;
    value["level"] = node.level;
    value["version"] = node.version;
    build(value["mbs"], node.mbs);

    if (node.parentNode) { build(value["parentNode"], *node.parentNode); }

    build(value["children"], node.children);
    build(value["neighbors"], node.neighbors);

    if (node.sharedResource) {
        build(value["sharedResource"], *node.sharedResource);
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

void saveMesh(std::ostream &os, const MeshSaver &meshSaver)
{
    // TODO: check layout
    (void) os;
    (void) meshSaver;
}

} // namespace

struct Writer::Detail {
    Detail(const boost::filesystem::path &path
           , const Metadata &metadata, const SceneLayerInfo &sli
           , bool overwrite)
        : zip(path, overwrite), metadata(metadata), sli(sli)
    {}

    void flush(const SceneLayerInfoCallback &callback) {
        // update and store scene layer
        if (callback) { callback(sli); }
        store(std::make_tuple(sli, metadata), detail::constants::SceneLayer);

        // update and save metadata
        metadata.nodeCount = nodeCount;
        store(metadata, detail::constants::MetadataName);

        // done
        zip.close();
    }

    utility::zip::Writer::OStream::pointer
    ostream(const fs::path &path, bool image = false)
    {
        const utility::zip::Compression compression
            ((!image && (metadata.archiveCompressionType
                         != ArchiveCompressionType::store))
             ? utility::zip::Compression::deflate
             : utility::zip::Compression::store);

        if (!image && (metadata.resourceCompressionType
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

    template <typename T>
    void store(const T &value, const fs::path &path
               , bool image = false)
    {
        Json::Value jValue;
        build(jValue, value);

        std::unique_lock<std::mutex> lock(mutex);
        auto os(ostream( path, image));
        // Json::write(os->get(), jValue, false);
        Json::write(os->get(), jValue, true);
        os->close();
    }

    void write(Node &node, const TextureSaver &textureSaver
               , const MeshSaver &meshSaver);

    utility::zip::Writer zip;
    std::mutex mutex;
    Metadata metadata;
    SceneLayerInfo sli;

    std::atomic<std::size_t> nodeCount;
};

void Writer::Detail::write(Node &node, const TextureSaver &textureSaver
                           , const MeshSaver &meshSaver)
{
    const auto index(node.geometryData.size());

    // write textures
    int txi(0);
    for (const auto &encoding : sli.store->textureEncoding) {
        const auto href
            (utility::format("textures/%d_%d", index, txi++));
        node.textureData.emplace_back("./" + href);
        const fs::path texturePath
            (fs::path("nodes") / node.id / (href + ".bin"));

        std::unique_lock<std::mutex> lock(mutex);
        // TODO: do not report DDS as image (if ever used)
        auto os(ostream(texturePath, true));
        textureSaver.save(os->get(), encoding.mime);
        os->close();
    }

    // write meshes
    const auto href(utility::format("geometries/%d", index));
    node.geometryData.emplace_back("./" + href);
    const fs::path geometryPath
        (fs::path("nodes") / node.id / (href + ".bin"));

    // TODO: write without lock to temporary stream
    {
        std::unique_lock<std::mutex> lock(mutex);
        auto os(ostream(geometryPath));
        saveMesh(os->get(), meshSaver);
        os->close();
    }
}

Writer::Writer(const boost::filesystem::path &path
               , const Metadata &metadata, const SceneLayerInfo &sli
               , bool overwrite)
    : detail_(std::make_shared<Detail>(path, metadata, sli, overwrite))
{}

void Writer::write(const Node &node)
{
    detail_->store(node, (fs::path("nodes")
                          / node.id
                          / detail::constants::NodeIndex));
    ++detail_->nodeCount;
}

void Writer::write(Node &node, const TextureSaver &textureSaver
                   , const MeshSaver &meshSaver)
{
    detail_->write(node, textureSaver, meshSaver);
}

void Writer::flush(const SceneLayerInfoCallback &callback)
{
    detail_->flush(callback);
}

} // namespace slpk
