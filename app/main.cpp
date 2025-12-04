//
// Created by zhuge on 2025/11/19.
//


#include <iostream>
#include <string>
#include <filesystem> // C++17
#include "ASGBuilder.h"

// Use raw string literal for cleaner path handling

//const std::string STEP_FILE_PATH = R"(F:/2025-Now_NUAA_PhD/Storage/ProjectCodeStore/OCCTProjectWorkspace/TestModels/The Baseline_Ass.stp)";
//const std::string STEP_FILE_PATH = R"(F:/2025-Now_NUAA_PhD/Storage/ProjectCodeStore/OCCTProjectWorkspace/TestModels/Face2Face_Ass.stp)";
const std::string STEP_FILE_PATH = R"(F:/2025-Now_NUAA_PhD/Storage/ProjectCodeStore/OCCTProjectWorkspace/TestModels/tests.step)";
//const std::string STEP_FILE_PATH = R"(F:/2025-Now_NUAA_PhD/Storage/ProjectCodeStore/OCCTProjectWorkspace/TestModels/USlot-Key_Ass.stp)";
// const std::string STEP_FILE_PATH =
//     R"(F:/2025-Now_NUAA_PhD/Storage/ProjectCodeStore/OCCTProjectWorkspace/TestModels/The Logic Check_Ass.stp)";
//const std::string STEP_FILE_PATH = R"(F:/2025-Now_NUAA_PhD/Storage/ProjectCodeStore/OCCTProjectWorkspace/TestModels/Shaft_nostep.stp)";
const std::string JSON_OUTPUT_PATH = R"(F:/2025-Now_NUAA_PhD/Storage/ProjectCodeStore/OCCTProjectWorkspace/TestModels/output_result.json)";

int main()
{
    std::cout << "==============================================" << std::endl;
    std::cout << "   3D Geometry Analysis Platform (Refactored) " << std::endl;
    std::cout << "==============================================" << std::endl;

    // 1. Initialize Builder / 初始化构建器
    ASG::ASGBuilder builder;

    // 2. Load File / 加载文件
    // Using std::filesystem to check existence first is a good modern practice
    if (!std::filesystem::exists(STEP_FILE_PATH))
    {
        std::cerr << "[Error] File not found: " << STEP_FILE_PATH << std::endl;
        std::cerr << "Please update STEP_FILE_PATH in main.cpp." << std::endl;
        return 1;
    }

    if (!builder.LoadAssemblyFromSTEP(STEP_FILE_PATH))
    {
        std::cerr << "[Error] Failed to load STEP file." << std::endl;
        return 1;
    }

    // 3. Execute Pipeline / 执行分析流程
    try
    {
        // A. Parse Structure
        builder.ParseAssemblyTree();

        // B. Classify Atomic Features (and Merge Fragments)
        builder.ClassifyAtomicFeatures();

        // C. Recognize Composite Features
        builder.RecognizeCompositeFeatures();

        // D. Build Assembly Constraint Graph
        builder.BuildAssemblyConstraintGraph();

        // E. Output Results
        builder.PrintStatistics();

        if (builder.ExportToJSON(JSON_OUTPUT_PATH))
        {
            std::cout << "[Success] Analysis results saved to: " << JSON_OUTPUT_PATH << std::endl;
        }

        // ============================================================
        // TEST: GetGraphDataForPart (Tensor Export Check)
        // ============================================================
        // std::cout << "\n----------------------------------------------" << std::endl;
        // std::cout << "[TEST] Verifying Graph Data Export for GNN..." << std::endl;
        //
        // // 测试零件 ID (根据你的日志，应该是 "HolePlane" 或 "Shaft")
        // std::string testPartID = "HolePlane";
        //
        // ASG::DeepLearningGraphData graphData = builder.GetGraphDataForPart(testPartID);
        //
        // if (!graphData.IsEmpty())
        // {
        //     size_t numNodes = graphData.nodeTypes.size();
        //     size_t numEdges = graphData.edgeSource.size();
        //
        //     std::cout << "Target Part: " << testPartID << std::endl;
        //     std::cout << "  > Nodes: " << numNodes << std::endl;
        //     std::cout << "  > Edges: " << numEdges << std::endl;
        //
        //     // 打印前 5 个节点信息
        //     std::cout << "  > Node Sample (First 5):" << std::endl;
        //     for(size_t i=0; i < std::min(static_cast<size_t>(5), numNodes); ++i) {
        //         std::cout << "    [" << i << "] Type: " << graphData.nodeTypes[i]
        //                   << ", Area: " << graphData.nodeAreas[i]
        //                   << ", Curv: " << graphData.nodeCurvatures[i] << std::endl;
        //     }
        //
        //     // 打印前 5 条边信息 (检查二面角)
        //     std::cout << "  > Edge Sample (First 5):" << std::endl;
        //     for(size_t i=0; i < std::min(static_cast<size_t>(5), numEdges); ++i) {
        //         std::cout << "    [" << i << "] " << graphData.edgeSource[i] << " -> " << graphData.edgeTarget[i]
        //                   << " | Angle: " << graphData.edgeAngles[i] << " rad ("
        //                   << (graphData.edgeAngles[i] * 180.0 / 3.14159265359) << " deg)" << std::endl;
        //     }
        // }
        // else
        // {
        //     std::cout << "[TEST] Failed to get graph data for " << testPartID << " (Is the ID correct?)" << std::endl;
        // }
        // std::cout << "----------------------------------------------\n" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[Exception] " << e.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cerr << "[Exception] Unknown error occurred." << std::endl;
        return 1;
    }

    std::cout << "==============================================" << std::endl;
    std::cout << "   Analysis Finished Successfully.            " << std::endl;
    std::cout << "==============================================" << std::endl;

    return 0;
}
