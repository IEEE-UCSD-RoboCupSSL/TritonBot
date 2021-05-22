#include "ManualTest/CoreModuleTests/ConversionTest.hpp"
#include "CoreModules/CommandProcessorModule/Conversion.hpp"

bool ConversionTest::test(ThreadPool& threadPool) {
    
    
    BotData data;
    data.frame = ReferenceFrame::WorldFrame;
    data.pos = {5800.00f, -3760.0f};
    data.ang = -128.0f;
    arma::vec testPoint = {29.5f, 6.3f};

    auto resultA = transformWorldToBodyFrame(data, testPoint);
    auto resultB = DEPRECATED_transformWorldToBodyFrame(data.pos, data.ang, testPoint); 

    std::cout << resultA << std::endl;
    std::cout << resultB << std::endl;
    return arma::norm(arma::abs(resultA - resultB));
}