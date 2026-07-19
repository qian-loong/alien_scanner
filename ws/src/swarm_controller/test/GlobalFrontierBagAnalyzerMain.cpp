#include "GlobalFrontierBagAnalyzer.hpp"

#include <charconv>
#include <cstddef>
#include <iostream>
#include <string>
#include <system_error>

namespace {

    void printUsage()
    {
        std::cerr
                << "usage: global_frontier_bag_analyzer BAG_URI STATISTICS_CSV "
                   "[TIMING_CSV] [--component-csv PATH "
                   "--component-membership-csv PATH] "
                   "[--max-trace-geometry-elements COUNT]\n";
    }

    bool parsePositiveSize(const std::string & text, std::size_t & value)
    {
        std::size_t parsed {};
        const auto [end, error] =
                std::from_chars(text.data(), text.data() + text.size(), parsed);
        if(error != std::errc {} || end != text.data() + text.size()
           || parsed == 0U)
        {
            return false;
        }
        value = parsed;
        return true;
    }

}// namespace

int main(int argc, char ** argv)
{
    if(argc < 3) {
        printUsage();
        return 2;
    }
    SwarmController::Test::BagAnalyzerOptions options;
    options.bag_uri = argv[1];
    options.statistics_csv = argv[2];
    int index = 3;
    if(index < argc && std::string(argv[index]).rfind("--", 0U) != 0U) {
        options.timing_csv = argv[index++];
    }
    while(index < argc) {
        const std::string flag = argv[index++];
        if(index >= argc) {
            printUsage();
            return 2;
        }
        const std::string value = argv[index++];
        if(flag == "--component-csv") {
            options.component_csv = value;
        } else if(flag == "--component-membership-csv") {
            options.component_membership_csv = value;
        } else if(flag == "--max-trace-geometry-elements") {
            if(!parsePositiveSize(
                       value,
                       options.detector_config.max_trace_geometry_elements))
            {
                printUsage();
                return 2;
            }
        } else {
            printUsage();
            return 2;
        }
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
