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
                      ((Pointcloud)("pointcloud"))
                      )

struct SpatialReference {
    int wkid;
    int latestWkid;
    int vcsWkid;
    int latestVcsWkid;
    std::string wkt;
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

    // capabilities
    // cachedDrawingInfo
    // drawingInfo
    // fields
    // attributeStorageInfo
};

} // namespace slpk

#endif // slpk_types_hpp_included_
