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

#ifndef slpk_restapi_hpp_included_
#define slpk_restapi_hpp_included_

#include "reader.hpp"

namespace slpk {

namespace constants {
const std::string SceneServer("SceneServer");
} // namespace constants

struct ApiFile {
    /** Stored file's path
     */
    boost::filesystem::path path;

    /** File's content type.
     */
    std::string contentType;

    /** Transfer encoding (gzip, deflate, or empty if not encoded at all);
     */
    std::string transferEncoding;

    /** File content. Nonempty only for service files.a
     */
    std::string content;

    typedef std::map<std::string, ApiFile> map;

    ApiFile(const boost::filesystem::path &path = "") : path(path) {}
};

/** SLPK archive reader -- REST API adapter
 */
class RestApi {
public:
    /** Adapts open SLPK archive as a REST API provider.
     *
     *  Reader is owned by this adapter.
     *
     * \param reader reader to adapt
     */
    RestApi(Archive &&archive);

    /** Get stream and file info for given path.
     *
     *  If stream is null then api-file containts data to stream.
     */
    std::pair<roarchive::IStream::pointer, const ApiFile*>
    file(const boost::filesystem::path &path) const;

private:
    Archive archive_;
    ApiFile::map files_;
};

} // namespace slpk

#endif // slpk_restapi_hpp_included_
