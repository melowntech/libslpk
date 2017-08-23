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

#include "geo/csconvertor.hpp"

#include "slpk/reader.hpp"

namespace po = boost::program_options;
namespace bio = boost::iostreams;
namespace fs = boost::filesystem;
namespace ublas = boost::numeric::ublas;

namespace {

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
        ;

    pd
        .add("input", 1)
        .add("output", 1);

    (void) config;
}

void Slpk2Obj::configure(const po::variables_map &vars)
{
    overwrite_ = vars.count("overwrite");
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
    virtual void addFace(const Face&, const Face&, const Face&) {}
    virtual void addNormal(const math::Point3d&) {}
    virtual void addTxRegion(const Region&) {}

    const geo::CsConvertor &conv_;
    math::Extents2 &extents_;
};

math::Extents2 measureMesh(const slpk::Tree &tree
                           , const slpk::Archive &input
                           , geo::CsConvertor conv)
{
    // find topLevel
    auto topLevel(std::numeric_limits<int>::max());

    for (const auto item : tree.nodes) {
        const auto &node(item.second);
        if (node.hasGeometry()) {
            topLevel = std::min(topLevel, node.level);
        }
    }

    // collect nodes for OpenMP
    std::vector<const slpk::Node*> nodes;
    for (const auto &item : tree.nodes) {
        const auto &node(item.second);
        if ((node.level == topLevel) && (node.hasGeometry())) {
            nodes.push_back(&node);
        }
    }

    const auto *pnodes(&nodes);

    math::Extents2 extents(math::InvalidExtents{});
    auto *pextents(&extents);

    UTILITY_OMP(parallel for)
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        const auto &node(*(*pnodes)[i]);

        // load geometry
        math::Extents2 e(math::InvalidExtents{});
        MeasureMesh loader(conv, e);
        input.loadGeometry(loader, node);

        UTILITY_OMP(critical(slpk2obj_measureMesh))
        {
            math::update(*pextents, e.ll);
            math::update(*pextents, e.ur);
        }
    }

    return extents;
}

void write(const slpk::Archive &input, fs::path &output
           , const geo::SrsDefinition &srs)
{
    geo::CsConvertor conv
        (input.sceneLayerInfo().spatialReference.srs(), srs);

    const auto tree(input.loadTree());

    // find extents in destination SRS to localize mesh
    const auto extents(measureMesh(tree, input, conv));
    const auto center(math::center(extents));
    LOG(info4) << std::fixed << "extents: " << extents
               << ", center: " << center;

    // collect nodes for OpenMP
    std::vector<const slpk::Node*> nodes;
    for (const auto &item : tree.nodes) {
        nodes.push_back(&item.second);
    }

    const auto *pnodes(&nodes);
    const auto *pcenter(&center);

    UTILITY_OMP(parallel for)
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        const auto &node(*(*pnodes)[i]);
        const auto &center(*pcenter);

        LOG(info4) << "Exporting <" << node.id << ">.";

        auto geometry(input.loadGeometry(node));

        auto igd(node.geometryData.begin());
        int meshIndex(0);
        for (auto &mesh : geometry) {
            for (auto &v : mesh.vertices) {
                v = conv(v) - center;
            }

            const fs::path path(output / (*igd++).href);
            const auto meshPath(utility::addExtension(path, ".obj"));
            create_directories(meshPath.parent_path());

            auto texture(input.texture(node, meshIndex));

            // detect extension
            const auto texPath(utility::addExtension
                               (path, imgproc::imageType
                                (*texture, texture->path())));
            const auto mtlPath(utility::addExtension(path, ".mtl"));

            // save mesh
            {
                utility::ofstreambuf os(meshPath.string());
                os.precision(12);
                saveAsObj(mesh, os, mtlPath.filename().string());
                os.flush();
            }

            // copy texture as-is
            copy(input.texture(node, meshIndex), texPath);
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
    write(archive, output_, srs_);
    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    utility::unlimitedCoredump();
    return Slpk2Obj()(argc, argv);
}
