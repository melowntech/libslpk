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

#include <boost/utility/in_place_factory.hpp>
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

namespace {

template <typename T>
std::string asString(const T &value)
{
    return boost::lexical_cast<std::string>(value);
}

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

void build(Json::Value &value, const Store &store)
{
    value = Json::objectValue;
    value["id"] = store.id;
    value["profile"] = asString(store.profile);
    build(value["resourcePattern"], store.resourcePattern);

    // TODO: fill me in pls, thx
}

void build(Json::Value &value, const SceneLayerInfo &sli)
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

    if (sli.store) { build(value["store"], *sli.store); }
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
}

// implementation must be here
template <typename T>
void build(Json::Value &value, const std::vector<T> &array)
{
    value = Json::arrayValue;
    for (const auto &item : array) {
        build(value.append({}), item);
    }
}

} // namespace

struct Writer::Detail {
    Detail(const boost::filesystem::path &path
           , const Metadata &metadata, bool overwrite)
        : zip(path, overwrite), metadata(metadata)
    {}

    void flush() {
        metadata.nodeCount = nodeCount;
        store(metadata, detail::constants::MetadataName);
        zip.close();
    }

    utility::zip::Writer::OStream::pointer
    ostream(fs::path path, bool image = false)
    {
        const utility::zip::Compression compression
            ((!image && (metadata.archiveCompressionType
                         != ArchiveCompressionType::store))
             ? utility::zip::Compression::deflate
             : utility::zip::Compression::store);

        if (!image && (metadata.resourceCompressionType
                       == ResourceCompressionType::gzip))
        {
            path = utility::addExtension(path, detail::constants::ext::gz);
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

    utility::zip::Writer zip;
    std::mutex mutex;
    Metadata metadata;

    std::atomic<std::size_t> nodeCount;
};

Writer::Writer(const boost::filesystem::path &path
               , const Metadata &metadata, bool overwrite)
    : detail_(std::make_shared<Detail>(path, metadata, overwrite))
{}

void Writer::write(const SceneLayerInfo &sli)
{
    detail_->store(sli, detail::constants::SceneLayer);
}

void Writer::write(const Node &node)
{
    detail_->store(node, (fs::path("nodes")
                          / node.id
                          / detail::constants::NodeIndex));
    ++detail_->nodeCount;
}

void Writer::flush()
{
    detail_->flush();
}

} // namespace slpk
