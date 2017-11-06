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

#ifndef slpk_writer_hpp_included_
#define slpk_writer_hpp_included_

#include <ostream>

#include "utility/zip.hpp"

#include "math/geometry.hpp"

#include "./types.hpp"

namespace slpk {

class TextureSaver {
public:
    virtual ~TextureSaver();
    virtual math::Size2 imageSize() const = 0;
    virtual void save(std::ostream &os, const std::string &mimeType) const = 0;
};

class MeshSaver {
public:
    struct Properties {
        std::size_t faceCount;

        Properties() : faceCount() {}
    };

    virtual ~MeshSaver();

    virtual Properties properties() const = 0;
    virtual math::Triangle3d face(std::size_t index) const = 0;
    virtual math::Triangle2d faceTc(std::size_t index) const = 0;
};

class Writer {
public:
    /** Creates SLPK file writer.
     *
     *  metadata are used to configure output but node count is computed
     *  dynamically
     */
    Writer(const boost::filesystem::path &path
           , const Metadata &metadata
           , const SceneLayerInfo &sli
           , bool overwrite = false);

    void write(const SceneLayerInfo &sli);

    /** Writes node and its (optional) shared resource definition at their
     *  respective proper places in the archive.
     *
     * Increments node count in metadata.
     */
    void write(const Node &node
               , const SharedResource *sharedResource = nullptr);

    /** Write mesh and texture(s). Update node.
     */
    void write(Node &node, Texture::list &textures
               , const MeshSaver &meshSaver
               , const TextureSaver &textureSaver);

    typedef std::function<void(SceneLayerInfo&)> SceneLayerInfoCallback;

    /** Saves metadata, 3dSceneLayerInfo and flushes output archive.
     *  Callback is used to modify 3d scene layer info before final flush.
     */
    void flush(const SceneLayerInfoCallback &callback
               = SceneLayerInfoCallback());

    struct Detail;

private:
    std::shared_ptr<Detail> detail_;
};

} // namespace slpk

#endif // slpk_writer_hpp_included_
