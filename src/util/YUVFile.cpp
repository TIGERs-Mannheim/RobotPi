#include "YUVFile.h"

YUVFile::YUVFile(const std::string& path)
{
    auto file = fopen(path.c_str(), "rb");

    FrameMetadata metadata;

    fread(&metadata.timestampUs, 8, 1, file);
    fread(&metadata.width, 2, 1, file);
    fread(&metadata.height, 2, 1, file);

    uint32_t size = metadata.width * metadata.height;
    uint32_t halfSize = (metadata.width / 2) * (metadata.height / 2);

    y_.resize(size);
    u_.resize(halfSize);
    v_.resize(halfSize);

    fread(y_.data(), size, 1, file);
    fread(u_.data(), halfSize, 1, file);
    fread(v_.data(), halfSize, 1, file);

    fclose(file);

    frame_ = std::make_unique<FrameYUV420>(y_.data(), u_.data(), v_.data(), metadata);
}

void YUVFile::write(const std::string& path, const FrameYUV420* pFrame)
{
    const auto& metadata = pFrame->getMetadata();
    uint32_t width = metadata.width;
    uint32_t height = metadata.height;

    auto file = fopen(path.c_str(), "wb");
    fwrite(&metadata.timestampUs, 8, 1, file);
    fwrite(&width, 2, 1, file);
    fwrite(&height, 2, 1, file);
    fwrite(pFrame->getYData(), width, height, file);
    fwrite(pFrame->getUData(), width/2, height/2, file);
    fwrite(pFrame->getVData(), width/2, height/2, file);
    fclose(file);
}
