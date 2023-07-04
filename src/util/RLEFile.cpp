#include "RLEFile.h"
#include <filesystem>
#include <numeric>

RLEFile RLEFile::openNext(const std::string &mediapath)
{
    // Collect path and size of all RLE files in mediapath
    struct FileInfo
    {
        std::filesystem::path path;
        std::uintmax_t size;
    };

    std::map<int, FileInfo> files;

    for(const auto& entry : std::filesystem::directory_iterator(mediapath))
    {
        if(!entry.is_regular_file())
            continue;

        const auto& path = entry.path();
        if(path.extension() != ".rle")
            continue;

        try
        {
            int fileNr = std::stoi(path.stem());
            files[fileNr].path = path;
            files[fileNr].size = entry.file_size();
        }
        catch(std::invalid_argument& e)
        {
            continue;
        }
        catch(std::filesystem::filesystem_error& e)
        {
            continue;
        }
    }

    // Determine next RLE file ID
    int nextFileId = 0;
    if(!files.empty())
        nextFileId = files.rbegin()->first + 1;

    // Limit RLE usage to 50% of partition size
    auto spaceInfo = std::filesystem::space(mediapath);
    std::uintmax_t filesizeLimit = spaceInfo.capacity / 2;

    // Remove files as long as the usage is exceeded
    auto getAccumulatedFilesize = [&](){ return std::accumulate(files.begin(), files.end(), (std::uintmax_t)0, [](const std::uintmax_t& s, const auto& b){ return s + b.second.size; }); };

    while(getAccumulatedFilesize() > filesizeLimit && !files.empty())
    {
        std::filesystem::remove(files.begin()->second.path);
        files.erase(files.begin());
    }

    return RLEFile(mediapath + "/" + std::to_string(nextFileId) + ".rle");
}

RLEFile::RLEFile(const std::string& path): file_(path, std::ios::binary | std::ios::in | std::ios::out | std::ios::app)
{}

void RLEFile::append(const RLEFrame& frame)
{
    const auto& data = frame.getData();
    const uint32_t length = data.size();
    file_.write(reinterpret_cast<const char *>(&length), 4);
    file_.write(reinterpret_cast<const char *>(data.data()), (std::streamsize)length);
}

bool RLEFile::readNext(RLEFrame& frame)
{
    int32_t length;
    file_.read(reinterpret_cast<char*>(&length), 4);
    if(file_.gcount() != 4)
        return false;

    std::vector<uint8_t> data(length);
    file_.read(reinterpret_cast<char*>(data.data()), length);
    frame = RLEFrame(data.data(), (uint32_t)file_.gcount());
    return true;
}
