#include "GlobalFrontierBagAnalyzer.hpp"

#include <iostream>

int main(int argc, char ** argv)
{
    if(argc != 3 && argc != 4) {
        std::cerr << "usage: global_frontier_bag_analyzer BAG_URI STATISTICS_CSV "
                     "[TIMING_CSV]\n";
        return 2;
    }
    SwarmController::Test::BagAnalyzerOptions options;
    options.bag_uri = argv[1];
    options.statistics_csv = argv[2];
    if(argc == 4) {
        options.timing_csv = argv[3];
    }
    const auto result = SwarmController::Test::analyzeGlobalFrontierBag(options);
    if(!result.succeeded()) {
        std::cerr << "analysis failed after " << result.frames_analyzed
                  << " frames: " << result.reason << '\n';
        return 1;
    }
    std::cout << "analyzed " << result.frames_analyzed << " /global_map frames\n";
    return 0;
}
