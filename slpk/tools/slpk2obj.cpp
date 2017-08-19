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
        , overwrite_(false)
    {
    }

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
};

void Slpk2Obj::configuration(po::options_description &cmdline
                             , po::options_description &config
                             , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("output", po::value(&output_)->required()
         , "Path to output (obj) tile set.")
        ("input", po::value(&input_)->required()
         , "Path to input SLPK archive.")
        ("overwrite", "Existing tile set gets overwritten if set.")
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

void localize(geometry::Mesh &mesh)
{
    math::Extents3 extents(math::InvalidExtents{});
    for (const auto &v: mesh.vertices) {
        math::update(extents, v);
    }

    const auto c(math::center(extents));
    for (auto &v: mesh.vertices) {
        v -= c;
    }
}

void write(const slpk::Archive &input, fs::path &output)
{
    const geo::CsConvertor conv(input.sceneLayerInfo().spatialReference.srs()
                                , geo::SrsDefinition(3857));

    const auto tree(input.loadTree());
    for (const auto &n : tree.nodes) {
        const auto &node(n.second);
        LOG(info4) << n.first;
        LOG(info4) << "    geometry:";
        for (const auto &r : node.geometryData) {
            LOG(info4) << "        " << r.href << " " << r.encoding->mime;
        }
        LOG(info4) << "    texture:";
        for (const auto &r : node.textureData) {
            LOG(info4) << "        " << r.href << " " << r.encoding->mime;
        }

        auto geometry(input.loadGeometry(node));

        {
            auto igd(node.geometryData.begin());
            int meshIndex(0);
            for (auto &mesh : geometry) {
                for (auto &v : mesh.vertices) { v = conv(v); }
                const fs::path path(output / (*igd++).href);
                const auto meshPath(utility::addExtension(path, ".obj"));
                create_directories(meshPath.parent_path());

                // TODO get extension from internals
                const auto texPath(utility::addExtension(path, ".jpg"));
                const auto mtlPath(utility::addExtension(path, ".mtl"));

                localize(mesh);

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
}

int Slpk2Obj::run()
{
    write(slpk::Archive(input_), output_);
    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    utility::unlimitedCoredump();
    return Slpk2Obj()(argc, argv);
}
