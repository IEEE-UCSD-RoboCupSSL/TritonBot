#include "ManualTest/CoreModuleTests/ConversionTest.hpp"
#include "CoreModules/Conversion.hpp"

bool ConversionTest::test(ThreadPool& threadPool) {
    
    
    BotData data;
    data.frame = ReferenceFrame::WorldFrame;
    data.pos = {5800.00f, -3760.0f};
    data.ang = -128.0f;


    arma::vec2 testPoint = {29.5f, 6.3f};
    auto resultA = transformWorldToBodyFrame(data.pos, data.ang, testPoint);
    auto resultB = DEPRECATED_transformWorldToBodyFrame(data.pos, data.ang, testPoint); 
    std::cout << "========================================" << std::endl;   
    std::cout << resultA << std::endl;
    std::cout << resultB << std::endl;

    testPoint = {321.0f, 123.0f};
    resultA = transformWorldToBodyFrame(data.pos, data.ang, testPoint);
    resultB = DEPRECATED_transformWorldToBodyFrame(data.pos, data.ang, testPoint); 
    std::cout << "========================================" << std::endl;   
    std::cout << resultA << std::endl;
    std::cout << resultB << std::endl;

    testPoint = {-321.0f, 123.0f};
    resultA = transformWorldToBodyFrame(data.pos, data.ang, testPoint);
    resultB = DEPRECATED_transformWorldToBodyFrame(data.pos, data.ang, testPoint); 
    std::cout << "========================================" << std::endl;   
    std::cout << resultA << std::endl;
    std::cout << resultB << std::endl;

    testPoint = {-321.0f, -123.0f};
    resultA = transformWorldToBodyFrame(data.pos, data.ang, testPoint);
    resultB = DEPRECATED_transformWorldToBodyFrame(data.pos, data.ang, testPoint); 
    std::cout << "========================================" << std::endl;   
    std::cout << resultA << std::endl;
    std::cout << resultB << std::endl;

    testPoint = {321.0f, -123.0f};
    resultA = transformWorldToBodyFrame(data.pos, data.ang, testPoint);
    resultB = DEPRECATED_transformWorldToBodyFrame(data.pos, data.ang, testPoint); 
    std::cout << "========================================" << std::endl;   
    std::cout << resultA << std::endl;
    std::cout << resultB << std::endl;





    return true;
}