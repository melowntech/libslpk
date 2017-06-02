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
#include <string>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/path.hpp"
#include "utility/streams.hpp"
#include "utility/path.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "./reader.hpp"

namespace fs = boost::filesystem;
namespace bio = boost::iostreams;

namespace slpk {

namespace {

namespace constants {
const std::string MetadataName("metadata.json");
const std::string SceneLayer("3dSceneLayer.json");
const boost::filesystem::path gzExt(".gz");
} // namespace constants

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
    Json::getOpt(srs.wkid, value, "layerType");
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
    // TODO: byteOffset
    // TODO: count
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

    // load ordering
    parse(gs.vertexAttributes, value
          , "vertexAttributes", "ordering");

    // TODO: faces

    parse(gs.vertexAttributes, value
          , "featureAttributes", "featureAttributeOrder");
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
    Json::getOpt(s.nidEncoding, value, "nidEncoding");
    Json::getOpt(s.featureEncoding, value, "featureEncoding");
    Json::getOpt(s.geometryEncoding, value, "geometryEncoding");
    Json::get(s.textureEncoding, value, "textureEncoding");
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

    parse(sli.store
          , Json::check(value["store"], Json::objectValue, "store"));

    return sli;
}

SceneLayerInfo loadSceneLayerInfo(const roarchive::IStream::pointer &in)
{
    return loadSceneLayerInfo(in->get(), in->path());
}

} // namespace

Archive::Archive(const fs::path &root)
    : archive_(root, constants::MetadataName)
    , metadata_(loadMetadata(archive_.istream(constants::MetadataName)))
    , sli_(loadSceneLayerInfo(istream(constants::SceneLayer)))
{}

roarchive::IStream::pointer
Archive::istream(const boost::filesystem::path &path)
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
                    fis.push(bio::gzip_decompressor());
                });
        }

        return archive_.istream(path);
    } break;
    }

    LOGTHROW(err1, std::runtime_error)
        << "Invalid ResourceCompressionType in metadata.";
    throw;
}

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

} // namespace slpk
