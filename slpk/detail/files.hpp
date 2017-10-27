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

#ifndef slpk_detail_files_hpp_included_
#define slpk_detail_files_hpp_included_

#include <string>
#include <vector>

#include <boost/filesystem/path.hpp>

namespace slpk { namespace detail {

namespace constants {
    const std::string MetadataName("metadata.json");
    const std::string SceneLayer("3dSceneLayer.json");
    const std::string NodeIndex("3dNodeIndexDocument.json");
    const std::string SharedResource("sharedResource.json");

    namespace ext {
        const boost::filesystem::path gz(".gz");
        const boost::filesystem::path json(".json");
        const boost::filesystem::path bin(".bin");
    } // namespace ext
} // namespace constants

typedef std::pair<std::string, bool> SpecialFile;

const std::vector<SpecialFile> specialFiles = {{
        { constants::SceneLayer, true, }
        , { constants::NodeIndex, true }
        , { constants::SharedResource, true }
        , { constants::MetadataName, false }
    }};

} } // namespace slpk::detail

#endif // slpk_detail_files_hpp_included_
