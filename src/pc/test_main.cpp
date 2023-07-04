#include "util/YUVFile.h"
#include "interface/RLEFrame.h"
#include "util/UDPSocket.h"
#include "steps/YPresummer.h"
#include "steps/CameraCalibration.h"

#include <chrono>
#include <iostream>

int main(int argc, const char **argv)
{
    if(argc < 2)
    {
        std::cout << "Usage: robotpi [path/to/test_image.yuv] ..." << std::endl;
        return 1;
    }

    State state;
    state.pComm = std::make_unique<CommandTransceiver>();

    UDPSocket socket("224.0.23.182", 28575);
    YPresummer presummer;
    ColorClassifierYUV classifier;
    classifier.setDebugLevel(1);
    RegionExtractor extractor;
    CameraCalibration detector;
    detector.setDebugLevel(1);

    for(int i = 1; i < argc; i++)
    {
        std::string imgName(argv[i]);
        std::cout << "Current image: " << imgName << std::endl;

        YUVFile yuvFile(imgName);
        state.pFrame = yuvFile.getFrame();
        const FrameMetadata& metadata = state.pFrame->getMetadata();
        state.uvWidth = metadata.width/2;
        state.uvHeight = metadata.height/2;

        auto startTime = std::chrono::high_resolution_clock::now();

        presummer.execute(state);
        classifier.execute(state);
        extractor.execute(state);
        detector.execute(state);

        std::cout << "main " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - startTime).count() / 1000.0 << " ms" << std::endl;

        RLEFrame rle(metadata, state.frameColoredRuns);
        socket.send(rle.getData());
        YUVFile::write(imgName.substr(0, imgName.size()-4).append(".out.yuv"), state.pFrame);
    }

    return 0;
}
