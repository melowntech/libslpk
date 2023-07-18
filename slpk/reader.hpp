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

#ifndef slpk_reader_hpp_included_
#define slpk_reader_hpp_included_

#include <initializer_list>
#include <vector>
#include <map>

#include <boost/any.hpp>

#include "geometry/mesh.hpp"

#include "geo/srsdef.hpp"

#include "roarchive/roarchive.hpp"

#include "types.hpp"

namespace slpk {

typedef math::Point3_<unsigned int> Face;
typedef std::vector<Face> Faces;
typedef math::Extents2 Region;
typedef std::vector<Region> Regions;

/** Face in texture coordinates. Enhanced with texture coordinates.
 */
struct FaceTc : Face {
    unsigned int region;

    FaceTc() : Face(), region() {}
    FaceTc(Face &&f) : Face(std::move(f)), region() {}
    FaceTc(Face &&f, unsigned int region)
        : Face(std::move(f)), region(region) {}

    FaceTc(unsigned int t1, unsigned int t2, unsigned int t3
           , unsigned int region)
        : Face(t1, t2, t3), region(region) {}
};

typedef std::vector<FaceTc> FacesTc;

class MeshLoader {
public:
    typedef slpk::Face Face;
    typedef slpk::FaceTc FaceTc;
    typedef slpk::Region Region;
    typedef slpk::Regions Regions;

    virtual ~MeshLoader() {}
    virtual void addVertex(const math::Point3d&) = 0;
    virtual void addTexture(const math::Point2d&) = 0;
    virtual void addNormal(const math::Point3d&) = 0;
    virtual void addFace(const Face &mesh, const FaceTc &tc = FaceTc()
                         , const Face &normal = Face()) = 0;
    virtual void addTxRegion(const Region &region) = 0;
};

/** Abstract geometry loader.
 */
class GeometryLoader {
public:
    virtual ~GeometryLoader() {}

    /** Get reference to next mesh loader.
     */
    virtual MeshLoader& next() = 0;
};

/** Sub mesh for provided simple geometry loader.
 */
struct SubMesh {
    geometry::Mesh mesh;
    Regions regions;

    typedef std::vector<SubMesh> list;
};

/** Mesh for provided simple geometry loader.
 */
struct Mesh {
    SubMesh::list submeshes;
};

/** Scene scervice file mapping.
 */

/** SLPK archive reader
 */
class Archive {
public:
    /** Open SLPK archive at given path with optional archive MIME type (from
     *  magic library).
     *
     * \param root path to archive/archive's root directory
     * \param mime root's MIME type or empty if unknown
     */
    Archive(const boost::filesystem::path &root, const std::string &mime = "");
    Archive(roarchive::RoArchive &archive);

    /** Generic I/O.
     */
    roarchive::IStream::pointer
    istream(const boost::filesystem::path &path) const;

    /** Generic I/O. Tries all extensions.
     */
    roarchive::IStream::pointer
    istream(const boost::filesystem::path &path
            , const std::initializer_list<const char*> &extensions) const;

    /** Generic I/O. Does not ungzip gzipped files. Usable only for HTTP
     *  adapters.
     */
    roarchive::IStream::pointer
    rawistream(const boost::filesystem::path &path) const;

    /** Returns real path to resource.
     */
    boost::filesystem::path realPath(const boost::filesystem::path &path);

    /** Returns loaded scene layer info.
     */
    const SceneLayerInfo& sceneLayerInfo() const { return sli_; }

    geo::SrsDefinition srs() const;

    /** Loads node index from given path inside the archive.
     *
     * \param dir node directory
     * \param path real path to node file (filled when non-null);
     * \return loaded node
     */
    Node loadNodeIndex(const boost::filesystem::path &dir
                       , boost::filesystem::path *path = nullptr) const;

    /** Loads shared resource inside the archive.
     *
     * \param shared resource dir
     * \return loaded shared resource
     */
    SharedResource loadSharedResource(const boost::filesystem::path &dir)
        const;

    /** Load root node (from path stated in scene layer info).
     */
    Node loadRootNodeIndex(boost::filesystem::path *path = nullptr) const;

    /** Loads whole node tree.
     */
    Tree loadTree() const;

    /** Loads whole node tree as a list of extended node info.
     */
    NodeInfo::list loadNodes() const;

    /** Loads node geometry. Possibly more meshes than just one.
     */
    Mesh loadGeometry(const Node &node
                      , const SharedResource::optional &sharedResource) const;

    /** Generic mesh load interface.
     */
    void loadGeometry(GeometryLoader &loader, const Node &node
                      , const SharedResource::optional &sharedResource) const;

    /** Opens texture file for given geometry mesh. If there are more version of
     *  the same texture the returns PNG or JPEG. DDS is ignored.
     */
    roarchive::IStream::pointer texture(const Node &node, int index = 0)
        const;

    /** Measures texture image size.
     */
    math::Size2 textureSize(const Node &node, int index = 0) const;

    /** Get raw scene layer info as decoded from file.
     */
    const boost::any& rawSceneLayerInfo() const { return rawSli_; }

    /** Get list of all files.
     */
    roarchive::Files fileList() const;

    /** Has the underlying archive been changed.
     */
    bool changed() const;

    /** Get metadata
     */
    const Metadata& metadata() const { return metadata_; }

private:
    roarchive::RoArchive archive_;
    Metadata metadata_;
    boost::any rawSli_;
    SceneLayerInfo sli_;
};

} // namespace slpk

#endif // slpk_reader_hpp_included_
