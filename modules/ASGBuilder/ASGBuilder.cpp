//
// Created by zhuge on 2025/11/19.
//

#include "ASGBuilder.h"

// Standard Library
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>

// OpenCASCADE STEP & XDE Headers
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <TDocStd_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDataStd_Name.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <TCollection_AsciiString.hxx>
#include <gp_Lin.hxx>

namespace ASG
{
    // ============================================================================
    // Constructor & Destructor
    // ============================================================================

    ASGBuilder::ASGBuilder()
    {
        // Create XDE Document
        Handle(TDocStd_Application) app = new TDocStd_Application();
        app->NewDocument("MDTV-XCAF", doc_);
    }

    ASGBuilder::~ASGBuilder()
    {
        if (!doc_.IsNull())
        {
            doc_->Close();
        }
    }

    // ============================================================================
    // Step 1: CAD Data Parsing
    // ============================================================================

    bool ASGBuilder::LoadAssemblyFromSTEP(const std::string& filePath)
    {
        std::cout << "[Loading STEP File] " << filePath << std::endl;

        STEPCAFControl_Reader reader;
        reader.SetColorMode(true);
        reader.SetNameMode(true);
        reader.SetLayerMode(true);

        if (const IFSelect_ReturnStatus status = reader.ReadFile(filePath.c_str()); status != IFSelect_RetDone)
        {
            std::cerr << "Error: Cannot read STEP file." << std::endl;
            return false;
        }

        if (!reader.Transfer(doc_))
        {
            std::cerr << "Error: Cannot transfer STEP data to XDE document." << std::endl;
            return false;
        }

        shapeTool_ = XCAFDoc_DocumentTool::ShapeTool(doc_->Main());
        if (shapeTool_.IsNull())
        {
            std::cerr << "Error: Cannot retrieve ShapeTool." << std::endl;
            return false;
        }

        std::cout << "STEP file loaded successfully." << std::endl;
        return true;
    }

    void ASGBuilder::ParseAssemblyTree()
    {
        std::cout << "[Parsing Assembly Tree]" << std::endl;

        TDF_LabelSequence topLabels;
        shapeTool_->GetFreeShapes(topLabels);

        std::cout << "  Found " << topLabels.Length() << " top-level entities." << std::endl;

        for (int i = 1; i <= topLabels.Length(); i++)
        {
            gp_Trsf identityTransform;
            TDF_Label label = topLabels.Value(i);
            ExtractPartsFromLabel(label, identityTransform);
        }

        std::cout << "Assembly parsing complete. Extracted " << partNodes_.size() << " parts." << std::endl;
    }

    void ASGBuilder::ExtractPartsFromLabel(const TDF_Label& label, const gp_Trsf& transformation)
    {
        if (XCAFDoc_ShapeTool::IsAssembly(label))
        {
            TDF_LabelSequence components;
            XCAFDoc_ShapeTool::GetComponents(label, components);

            for (int i = 1; i <= components.Length(); i++)
            {
                TDF_Label compLabel = components.Value(i);
                TopLoc_Location loc = XCAFDoc_ShapeTool::GetLocation(compLabel);
                gp_Trsf compTransform = loc.Transformation();
                gp_Trsf accumulatedTransform = transformation * compTransform;

                TDF_Label refLabel;
                if (XCAFDoc_ShapeTool::GetReferredShape(compLabel, refLabel))
                {
                    ExtractPartsFromLabel(refLabel, accumulatedTransform);
                }
            }
        }
        else if (XCAFDoc_ShapeTool::IsSimpleShape(label))
        {
            TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(label);
            if (shape.IsNull() || shape.ShapeType() != TopAbs_SOLID)
            {
                return;
            }

            PartNode partNode;
            partNode.brepShape = shape;
            partNode.transformation = transformation;

            // Generate Part ID (UTF-8 Support)
            if (Handle(TDataStd_Name) nameAttr; label.FindAttribute(TDataStd_Name::GetID(), nameAttr))
            {
                const TCollection_ExtendedString extName = nameAttr->Get();

                // Modern OCCT way to convert ExtendedString to UTF-8
                // 现代 OCCT 转换 UTF-8 的方式
                auto buffer = new char[extName.LengthOfCString() + 1];
                extName.ToUTF8CString(buffer);
                partNode.partID = std::string(buffer);
                delete[] buffer;
            }

            if (partNode.partID.empty())
            {
                partNode.partID = "Part_" + std::to_string(partNodes_.size() + 1);
            }

            std::cout << "  |- Part Found: " << partNode.partID << std::endl;
            partNodes_.push_back(partNode);
        }
    }

    // ============================================================================
    // Step 2: Atomic Geometric Feature Classification
    // ============================================================================

    void ASGBuilder::ClassifyAtomicFeatures()
    {
        std::cout << "[Classifying Atomic Features]" << std::endl;

        for (auto& partNode : partNodes_)
        {
            ClassifyPartAtomicFeatures(partNode);
            std::cout << "  |- " << partNode.partID << ": " << partNode.atomicFeatures.size() << " atomic features." <<
                std::endl;
        }

        std::cout << "[Merging Fragmented Features]" << std::endl;
        MergeFragmentedFeatures();
        std::cout << "Feature classification and merging complete." << std::endl;
    }

    void ASGBuilder::ClassifyPartAtomicFeatures(PartNode& partNode)
    {
        const TopoDS_Shape& solid = partNode.brepShape;
        partNode.atomicFeatures.clear();
        faceIDMap_.clear();

        int faceIndex = 0;

        // 1. Create and Identify Features
        for (TopExp_Explorer faceExplorer(solid, TopAbs_FACE); faceExplorer.More(); faceExplorer.Next())
        {
            TopoDS_Face face = TopoDS::Face(faceExplorer.Current());

            auto feature = std::make_shared<AtomicFeature>();
            feature->faceID = partNode.partID + "_Face_" + std::to_string(faceIndex++);
            feature->brepFace = face;
            faceIDMap_[face] = feature->faceID;

            auto [type, params] = IdentifyGeometryType(face);
            feature->atomType = type;
            feature->geometricParams = params;

            if (type == AtomType::CYLINDER)
            {
                // Corrected: use locationPoint
                feature->geometricParams.height = ComputeCylinderHeight(face, params.locationPoint, params.axisVector);
            }

            feature->formType = DetermineConcavity(face, type, params, solid);
            feature->area = ComputeFaceArea(face);
            if (feature->area < 0.001) feature->isFunctional = false;

            partNode.atomicFeatures.push_back(feature);
        }

        // 2. Analyze Adjacency
        for (const auto& feature : partNode.atomicFeatures)
        {
            feature->adjacencyList = AnalyzeTopologicalAdjacency(feature->brepFace, feature->faceID, solid);
        }
    }

    // ============================================================================
    // Step 2-B: Fragment Feature Merging
    // ============================================================================

    void ASGBuilder::MergeFragmentedFeatures() const
    {
        int logicalFeatureCounter = 0;

        for (auto& partNode : partNodes_)
        {
            std::vector<std::shared_ptr<AtomicFeature>> candidates;
            for (auto& f : partNode.atomicFeatures)
            {
                if (!f->isFragment && (
                    f->atomType == AtomType::CYLINDER ||
                    f->atomType == AtomType::CONE ||
                    f->atomType == AtomType::SPHERE ||
                    f->atomType == AtomType::TORUS))
                {
                    candidates.push_back(f);
                }
            }

            if (candidates.empty()) continue;

            for (size_t i = 0; i < candidates.size(); ++i)
            {
                const auto& mainFeat = candidates[i];
                if (mainFeat->isFragment) continue;

                std::vector<std::shared_ptr<AtomicFeature>> group;
                group.push_back(mainFeat);

                for (size_t j = i + 1; j < candidates.size(); ++j)
                {
                    const auto& otherFeat = candidates[j];
                    if (otherFeat->isFragment) continue;

                    if (mainFeat->atomType != otherFeat->atomType || mainFeat->formType != otherFeat->formType)
                        continue
                            ;

                    bool match = false;
                    const auto& p1 = mainFeat->geometricParams;
                    const auto& p2 = otherFeat->geometricParams;

                    if (mainFeat->atomType == AtomType::CYLINDER)
                    {
                        if (std::abs(p1.radius - p2.radius) < Constants::DistanceTolerance &&
                            p1.axisVector.IsParallel(p2.axisVector, Constants::AngleTolerance))
                        {
                            if (gp_Lin axisLine(p1.locationPoint, p1.axisVector); axisLine.Distance(p2.locationPoint) <
                                Constants::DistanceTolerance)
                            {
                                match = true;
                            }
                        }
                    }
                    // Extend other types here...

                    if (match)
                    {
                        group.push_back(otherFeat);
                    }
                }

                if (group.size() > 1)
                {
                    const std::string logicalID = partNode.partID + "_Logical_" + std::to_string(logicalFeatureCounter++);
                    double totalArea = 0.0;
                    std::vector<std::string> ids;

                    for (auto& f : group)
                    {
                        f->isFragment = true;
                        f->logicalFeatureID = logicalID;
                        totalArea += f->area;
                        ids.push_back(f->faceID);
                    }

                    mainFeat->isMainFragment = true;
                    mainFeat->mergedArea = totalArea;
                    mainFeat->fragmentFaceIDs = ids;
                }
            }
        }
    }

    // ============================================================================
    // Step 3: Composite Feature Recognition
    // ============================================================================

    void ASGBuilder::RecognizeCompositeFeatures()
    {
        std::cout << "[Recognizing Composite Features]" << std::endl;

        for (auto& partNode : partNodes_)
        {
            std::ranges::sort(partNode.atomicFeatures,
                              [](const std::shared_ptr<AtomicFeature>& a, const std::shared_ptr<AtomicFeature>& b) {
                                  return a->area > b->area; // <--- Descending (降序)
                              });
            FeatureMap featureMap;
            for (const auto& f : partNode.atomicFeatures)
            {
                featureMap[f->faceID] = f;
            }

            for (auto& f : partNode.atomicFeatures)
            {
                if (f->isConsumed) continue;
                if (f->isFragment && !f->isMainFragment) continue;

                if (RecognizeHoleFeature(partNode, f, featureMap)) continue;
                if (RecognizeShaftFeature(partNode, f, featureMap)) continue;
                if (RecognizeSlotFeature(partNode, f, featureMap)) continue;
                if (RecognizeTongueFeature(partNode, f, featureMap)) continue;
                if (RecognizeStepPlaneFeature(partNode, f, featureMap)) continue;
                if (RecognizeFunctionalPlaneFeature(partNode, f, featureMap)) continue;
            }
        }
    }

    // ============================================================================
    // Step 4: Assembly Relationship Recognition
    // ============================================================================

    void ASGBuilder::BuildAssemblyConstraintGraph()
    {
        std::cout << "-----------------------------------------------" << std::endl;
        std::cout << "[Step 4] Building Assembly Constraint Graph..." << std::endl;
        constraints_.clear();

        for (size_t i = 0; i < partNodes_.size(); ++i)
        {
            for (size_t j = i + 1; j < partNodes_.size(); ++j)
            {
                const auto& nodeA = partNodes_[i];
                const auto& nodeB = partNodes_[j];

                if (!CheckBoundingBoxCollision(nodeA, nodeB))
                {
                    continue;
                }

                MatchPartPair(nodeA, nodeB);
            }
        }

        std::cout << "Constraint Graph Built. Total Constraints: " << constraints_.size() << std::endl;
        for (const auto& c : constraints_)
        {
            std::cout << "  -> " << c.ToString() << std::endl;
        }
        std::cout << std::endl;
    }


    // ============================================================================
    // Interface Stub
    // ============================================================================

    void ASGBuilder::runTest()
    {
        // Intentionally left empty or simple check.
        // Logic is now driven by main.cpp.
        std::cout << "ASGBuilder linked successfully. Ready for external commands." << std::endl;
    }
} // namespace ASG
