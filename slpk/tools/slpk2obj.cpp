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

#include <set>

#include <boost/optional.hpp>
#include <boost/utility/in_place_factory.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "utility/buildsys.hpp"
#include "utility/gccversion.hpp"
#include "utility/limits.hpp"
#include "utility/path.hpp"
#include "utility/openmp.hpp"

#include "service/cmdline.hpp"

#include "geometry/meshop.hpp"

#include "imgproc/readimage.cpp"
#include "imgproc/texturing.cpp"

#include "geo/csconvertor.hpp"

#include "slpk/reader.hpp"

namespace po = boost::program_options;
namespace bio = boost::iostreams;
namespace fs = boost::filesystem;
namespace ublas = boost::numeric::ublas;

namespace {

typedef std::vector<std::string> NodeIdList;
typedef std::set<std::string> NodeIdSet;

bool notPicked(const std::string &nodeId
               , const boost::optional<NodeIdSet> &picked)
{
    if (!picked) { return false; }
    return (picked->find(nodeId) == picked->end());
}


class Slpk2Obj : public service::Cmdline
{
public:
    Slpk2Obj()
        : service::Cmdline("slpk2obj", BUILD_TARGET_VERSION)
        , overwrite_(false), srs_(3857)
    {}

private:
    virtual void configuration(po::options_description &cmdline
                               , po::options_description &config
                               , po::positional_options_description &pd)
        UTILITY_OVERRIDE;

    virtual void configure(const po::variables_map &vars)
        UTILITY_OVERRIDE;

    virtual bool help(std::ostream &out, const std::string &what) const
        UTILITY_OVERRIDE;

    virtual int run() UTILITY_OVERRIDE;

    fs::path output_;
    fs::path input_;
    bool overwrite_;
    geo::SrsDefinition srs_;

    boost::optional<NodeIdSet> nodes_;
};

void Slpk2Obj::configuration(po::options_description &cmdline
                             , po::options_description &config
                             , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("output", po::value(&output_)->required()
         , "Path to output converted input.")
        ("input", po::value(&input_)->required()
         , "Path to input SLPK archive.")
        ("overwrite", "Generate output even if output directory exists.")
        ("srs", po::value(&srs_)->default_value(srs_)->required()
         , "Destination SRS of converted meshes.")
        ("nodes", po::value<NodeIdList>()
         , "Limit output to listed nodes.")
        ;

    pd
        .add("input", 1)
        .add("output", 1)
        .add("nodes", -1);

    (void) config;
}

void Slpk2Obj::configure(const po::variables_map &vars)
{
    overwrite_ = vars.count("overwrite");

    if (vars.count("nodes")) {
        const auto &raw(vars["nodes"].as<NodeIdList>());
        nodes_ = boost::in_place(raw.begin(), raw.end());
    }
}

bool Slpk2Obj::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(slpk2obj

    Converts SLPK archive into textured meshes in OBJ format.

usage
    slpk2obj INPUT OUTPUT [OPTIONS]
)RAW";
    }
    return false;
}

void writeMtl(const fs::path &path, const std::string &name)
{
    LOG(info1) << "Writing " << path;
    std::ofstream f(path.string());

    f << "newmtl 0\n"
      << "map_Kd " << name
      << "\n";
}

class DeepCopyCsCovertor {
public:
    DeepCopyCsCovertor(geo::SrsDefinition src, geo::SrsDefinition dst)
        : src_(src), dst_(dst), conv_(src_, dst_)
    {}

    DeepCopyCsCovertor(const DeepCopyCsCovertor &c)
        : src_(c.src_), dst_(c.dst_), conv_(src_, dst_)
    {}

    DeepCopyCsCovertor& operator=(const DeepCopyCsCovertor &c) {
        src_ = c.src_;
        dst_ = c.dst_;
        conv_ = geo::CsConvertor(src_, dst_);
        return *this;
    }

    operator const geo::CsConvertor&() const { return conv_; }

    template <typename T> T operator()(const T &p) const { return conv_(p); }

private:
    geo::SrsDefinition src_;
    geo::SrsDefinition dst_;

    geo::CsConvertor conv_;
};

/** Loads SLPK geometry as a list of submeshes.
 */
class MeasureMesh
    : public slpk::GeometryLoader
    , public slpk::MeshLoader
{
public:
    MeasureMesh(const geo::CsConvertor &conv, math::Extents2 &extents)
        : conv_(conv), extents_(extents)
    {}

    virtual slpk::MeshLoader& next() { return *this; }

    virtual void addVertex(const math::Point3d &v) {
        math::update(extents_, conv_(v));
    }

private:
    virtual void addTexture(const math::Point2d&) {}
    virtual void addFace(const Face&, const FaceTc&, const Face&) {}
    virtual void addNormal(const math::Point3d&) {}
    virtual void addTxRegion(const Region&) {}

    const geo::CsConvertor &conv_;
    math::Extents2 &extents_;
};

math::Extents2 measureMesh(const slpk::Tree &tree
                           , const slpk::Archive &input
                           , DeepCopyCsCovertor conv
                           , const boost::optional<NodeIdSet> &pickedNodes)
{
    // find topLevel
    auto topLevel(std::numeric_limits<int>::max());
    for (const auto item : tree.nodes) {
        if (notPicked(item.first, pickedNodes)) { continue; }
        const auto &node(item.second.node);
        if (node.hasGeometry()) {
            topLevel = std::min(topLevel, node.level);
        }
    }

    // collect nodes for OpenMP
    std::vector<const slpk::TreeNode*> treeNodes;
    for (const auto &item : tree.nodes) {
        if (notPicked(item.first, pickedNodes)) { continue; }
        const auto &node(item.second.node);
        if ((node.level == topLevel) && (node.hasGeometry())) {
            treeNodes.push_back(&item.second);
        }
    }

    math::Extents2 extents(math::InvalidExtents{});

    UTILITY_OMP(parallel for firstprivate(conv), shared(extents, treeNodes))
    for (std::size_t i = 0; i < treeNodes.size(); ++i) {
        const auto &treeNode(*treeNodes[i]);
        const auto &node(treeNode.node);

        // load geometry
        math::Extents2 e(math::InvalidExtents{});
        MeasureMesh loader(conv, e);
        input.loadGeometry(loader, node, treeNode.sharedResource);

        UTILITY_OMP(critical(slpk2obj_measureMesh))
        {
            math::update(extents, e.ll);
            math::update(extents, e.ur);
        }
    }

    return extents;
}

cv::Mat stream2mat(const roarchive::IStream::pointer &txStream)
{
    const auto buf(txStream->read());
    const auto image(cv::imdecode(buf, cv::IMREAD_COLOR));

    if (!image.data) {
        LOGTHROW(err1, std::runtime_error)
            << "Cannot decode image from " << txStream->path() << ".";
    }
    return image;
}

inline math::Extents2 remap(const math::Size2 &size
                            , const slpk::Region &region)
{
    return {
        size.width * region.ll(0), size.height * region.ll(1)
        , size.width * region.ur(0), size.height *  region.ur(1)
    };
}

inline math::Point2d& denormalize(const math::Size2f &rsize, math::Point2d &tc)
{
    tc(0) *= rsize.width;
    tc(1) = (1.0 - tc(1)) * rsize.height;
    return tc;
}

void rebuild(slpk::SubMesh &submesh
             , const roarchive::IStream::pointer &txStream
             , const fs::path &texPath)
{
    const auto tx(stream2mat(txStream));

    const math::Size2 txSize(tx.cols, tx.rows);

    auto &mesh(submesh.mesh);

    std::vector<math::Extents2> regions;
    imgproc::tx::Patch::Rect::list uvRects;
    for (auto &region : submesh.regions) {
        regions.push_back(remap(txSize, region));
        uvRects.emplace_back(imgproc::tx::UvPatch(regions.back()));
    }

    // UV patch per region
    imgproc::tx::UvPatch::list uvPatches(submesh.regions.size());

    // create expanded patches
    std::vector<std::uint8_t> seen(mesh.tCoords.size(), false);
    const auto expand([&](imgproc::tx::UvPatch &uvPatch
                          , const math::Size2f &rsize, int index)
    {
        // skip mapped tc
        auto &iseen(seen[index]);
        if (iseen) { return; }

        uvPatch.update(denormalize(rsize, mesh.tCoords[index]));

        iseen = true;
    });

    for (const auto &face : mesh.faces) {
        const auto &region(regions[face.imageId]);
        const auto &rsize(math::size(region));
        auto &uvPatch(uvPatches[face.imageId]);

        expand(uvPatch, rsize, face.ta);
        expand(uvPatch, rsize, face.tb);
        expand(uvPatch, rsize, face.tc);
    }

    std::vector<imgproc::tx::Patch>
        patches(uvPatches.begin(), uvPatches.end());

    const auto size(imgproc::tx::pack(patches.begin(), patches.end()));

    // map texture coordinates to new texture

    seen.assign(mesh.tCoords.size(), false);
    const auto map([&](const imgproc::tx::Patch &patch, int index) -> void
    {
        // skip mapped tc
        auto &iseen(seen[index]);
        if (iseen) { return; }

        auto &tc(mesh.tCoords[index]);
        // map
        patch.map({}, tc);

        // and normalize back again
        tc(0) /= size.width;
        tc(1) /= size.height;
        tc(1) = 1.0 - tc(1);

        iseen = true;
    });

    for (auto &face : mesh.faces) {
        auto &patch(patches[face.imageId]);
        map(patch, face.ta);
        map(patch, face.tb);
        map(patch, face.tc);

        // reset image ID
        face.imageId = 0;
    }

    // generate new texture
    cv::Mat_<cv::Vec3b> otx(size.height, size.width, cv::Vec3b());

    const auto wrap([&](int pos, int origin, int size) -> int
    {
        auto mod(pos % size);
        if (mod < 0) { mod += size; }
        return origin + mod;
    });

    auto iuvRects(uvRects.begin());
    for (const auto &patch : patches) {
        const auto &uvRect(*iuvRects++);
        const auto &dstRect(patch.dst());
        const auto &srcRect(patch.src());
        const math::Point2i diff(uvRect.point - srcRect.point);

        // copy data
        for (int j(0), je(dstRect.size.height); j != je; ++j) {
            const auto jsrc(wrap(j - diff(1), uvRect.point(1)
                                 , uvRect.size.height));

            if ((jsrc < 0) || (jsrc >= txSize.height)) { continue; }

            for (int i(0), ie(dstRect.size.width); i != ie; ++i) {
                const auto isrc(wrap(i - diff(0), uvRect.point(0)
                                     , uvRect.size.width));

                if ((isrc < 0) || (isrc >= txSize.width)) { continue; }

                otx(dstRect.point(1) + j, dstRect.point(0) + i)
                    = tx.at<cv::Vec3b>(jsrc, isrc);
            }
        }
    }

    cv::imwrite(texPath.string(), otx
                , { cv::IMWRITE_JPEG_QUALITY, 85
                    , cv::IMWRITE_PNG_COMPRESSION, 9 });
}

void write(const slpk::Archive &input, fs::path &output
           , const geo::SrsDefinition &srs
           , const boost::optional<NodeIdSet> &pickedNodes)
{
    DeepCopyCsCovertor conv
        (input.sceneLayerInfo().spatialReference.srs(), srs);

    const auto tree(input.loadTree());

    // find extents in destination SRS to localize mesh
    const auto extents(measureMesh(tree, input, conv, pickedNodes));
    const auto center(math::center(extents));

    // collect nodes for OpenMP
    std::vector<const slpk::TreeNode*> treeNodes;
    for (const auto &item : tree.nodes) {
        if (notPicked(item.first, pickedNodes)) { continue; }
        treeNodes.push_back(&item.second);
    }

    UTILITY_OMP(parallel for firstprivate(conv), shared(treeNodes))
    for (std::size_t i = 0; i < treeNodes.size(); ++i) {
        const auto &treeNode(*treeNodes[i]);
        const auto &node(treeNode.node);

        LOG(info3) << "Converting <" << node.id << ">.";

        auto geometry(input.loadGeometry(node, treeNode.sharedResource));

        auto igd(node.geometryData.begin());
        int meshIndex(0);
        for (auto &submesh : geometry.submeshes) {
            auto &mesh(submesh.mesh);
            for (auto &v : mesh.vertices) {
                v = conv(v);
                v(0) -= center(0);
                v(1) -= center(1);
            }

            const fs::path path(output / (*igd++).href);
            const auto meshPath(utility::addExtension(path, ".obj"));
            create_directories(meshPath.parent_path());

            auto texture(input.texture(node, meshIndex));

            // detect extension
            const auto mtlPath(utility::addExtension(path, ".mtl"));

            fs::path texPath;

            if (submesh.regions.empty()) {
                // copy texture as-is
                texPath  = utility::addExtension
                    (path, imgproc::imageType
                     (*texture, texture->path()));
                copy(texture, texPath);
            } else {
                // texture atlas, need to repack/unpack texture
                texPath = utility::addExtension(path, ".jpg");
                rebuild(submesh, texture, texPath);
            }

            {
                // save mesh
                utility::ofstreambuf os(meshPath.string());
                os.precision(12);
                saveAsObj(mesh, os, mtlPath.filename().string());
                os.flush();
            }

            writeMtl(mtlPath, texPath.filename().string());
            ++meshIndex;
        }
    }
}

int Slpk2Obj::run()
{
    LOG(info4) << "Opening SLPK archive at " << input_ << ".";
    slpk::Archive archive(input_);
    LOG(info4) << "Generating textured meshes at " << output_ << ".";
    write(archive, output_, srs_, nodes_);
    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    utility::unlimitedCoredump();
    return Slpk2Obj()(argc, argv);
}
