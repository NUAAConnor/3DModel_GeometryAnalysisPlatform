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
#include <algorithm>
#include <ranges>

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
#include <gp_Lin.hxx>

namespace ASG
{
    // ============================================================================
    // Constructor & Destructor
    // ============================================================================

    ASGBuilder::ASGBuilder()
    {
        // a. Create a new XDE application instance for managing CAD documents
        Handle(TDocStd_Application) app = new TDocStd_Application();

        // b. Allocate a new document with the "MDTV-XCAF" format for assembly storage
        app->NewDocument("MDTV-XCAF", doc_);
    }

    ASGBuilder::~ASGBuilder()
    {
        // a. Close the XDE document if it is still open to release resources
        if (!doc_.IsNull())
        {
            doc_->Close();
        }
    }

    // ============================================================================
    // STEP 1: CAD Data Parsing - STEP File Import
    // ============================================================================

    bool ASGBuilder::LoadAssemblyFromSTEP(const std::string& filePath)
    {
        std::cout << "[Loading STEP File] " << filePath << std::endl;

        // a. Initialize the STEP reader with CAF (Color and Assembly Framework) support
        STEPCAFControl_Reader reader;
        reader.SetColorMode(true);
        reader.SetNameMode(true);
        reader.SetLayerMode(true);

        // b. Attempt to read the STEP file from disk
        if (const IFSelect_ReturnStatus status = reader.ReadFile(filePath.c_str()); status != IFSelect_RetDone)
        {
            std::cerr << "Error: Cannot read STEP file." << std::endl;
            return false;
        }

        // c. Transfer the STEP data into the XDE document structure
        if (!reader.Transfer(doc_))
        {
            std::cerr << "Error: Cannot transfer STEP data to XDE document." << std::endl;
            return false;
        }

        // d. Retrieve the shape tool for navigating the assembly hierarchy
        shapeTool_ = XCAFDoc_DocumentTool::ShapeTool(doc_->Main());
        if (shapeTool_.IsNull())
        {
            std::cerr << "Error: Cannot retrieve ShapeTool." << std::endl;
            return false;
        }

        std::cout << "STEP file loaded successfully." << std::endl;
        return true;
    }

    // ============================================================================
    // STEP 1: CAD Data Parsing - Assembly Tree Traversal
    // ============================================================================

    void ASGBuilder::ParseAssemblyTree()
    {
        std::cout << "[Parsing Assembly Tree]" << std::endl;

        // a. Retrieve all free shapes (root-level components) from the document
        TDF_LabelSequence topLabels;
        shapeTool_->GetFreeShapes(topLabels);

        std::cout << "  Found " << topLabels.Length() << " top-level entities." << std::endl;

        // b. Recursively extract parts from each root label with identity transform
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
        // a. Check if the current label represents an assembly node
        if (XCAFDoc_ShapeTool::IsAssembly(label))
        {
            // b. Retrieve all child components of this assembly
            TDF_LabelSequence components;
            XCAFDoc_ShapeTool::GetComponents(label, components);

            // c. Process each child component recursively
            for (int i = 1; i <= components.Length(); i++)
            {
                TDF_Label compLabel = components.Value(i);

                // d. Extract the local transformation for this component
                TopLoc_Location loc = XCAFDoc_ShapeTool::GetLocation(compLabel);
                gp_Trsf compTransform = loc.Transformation();

                // e. Accumulate transformations: parent * local
                gp_Trsf accumulatedTransform = transformation * compTransform;

                // f. Get the referenced shape and recurse
                TDF_Label refLabel;
                if (XCAFDoc_ShapeTool::GetReferredShape(compLabel, refLabel))
                {
                    ExtractPartsFromLabel(refLabel, accumulatedTransform);
                }
            }
        }
        // g. If the label is a simple shape (leaf node), extract it as a part
        else if (XCAFDoc_ShapeTool::IsSimpleShape(label))
        {
            // h. Retrieve the geometric shape from the label
            TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(label);

            // i. Only accept solid shapes (skip surfaces, edges, etc.)
            if (shape.IsNull() || shape.ShapeType() != TopAbs_SOLID)
            {
                return;
            }

            // j. Create a new part node to store this solid
            PartNode partNode;
            partNode.brepShape = shape;
            partNode.transformation = transformation;

            // k. Compute world-space bounding box for collision detection
            BRepBndLib::Add(partNode.brepShape, partNode.worldBoundingBox);
            partNode.worldBoundingBox.SetGap(Constants::DistanceTolerance);
            partNode.worldBoundingBox = partNode.worldBoundingBox.Transformed(transformation);

            // l. Extract part name from XDE label (UTF-8 support)
            if (Handle(TDataStd_Name) nameAttr; label.FindAttribute(TDataStd_Name::GetID(), nameAttr))
            {
                const TCollection_ExtendedString extName = nameAttr->Get();

                // m. Convert ExtendedString to UTF-8 (modern OCCT approach)
                auto buffer = new char[extName.LengthOfCString() + 1];
                extName.ToUTF8CString(buffer);
                partNode.partID = std::string(buffer);
                delete[] buffer;
            }

            // n. Generate a default ID if no name is found
            if (partNode.partID.empty())
            {
                partNode.partID = "Part_" + std::to_string(partNodes_.size() + 1);
            }

            std::cout << "  |- Part Found: " << partNode.partID << std::endl;

            // o. Store the part node in the global collection
            partNodes_.push_back(partNode);
        }
    }

    // ============================================================================
    // STEP 2: Atomic Feature Classification - Main Entry Point
    // ============================================================================

    void ASGBuilder::ClassifyAtomicFeatures()
    {
        std::cout << "[Classifying Atomic Features]" << std::endl;

        // a. Classify atomic features for each part in the assembly
        for (auto& partNode : partNodes_)
        {
            ClassifyPartAtomicFeatures(partNode);
            std::cout << "  |- " << partNode.partID << ": " << partNode.atomicFeatures.size() << " atomic features." <<
                std::endl;
        }

        // b. Merge fragmented features that belong to the same geometric surface
        std::cout << "[Merging Fragmented Features]" << std::endl;
        MergeFragmentedFeatures();
        std::cout << "Feature classification and merging complete." << std::endl;
    }

    void ASGBuilder::ClassifyPartAtomicFeatures(PartNode& partNode)
    {
        // a. Extract the solid shape from the part node
        const TopoDS_Shape& solid = partNode.brepShape;

        // b. Clear any existing atomic features and face ID mappings
        partNode.atomicFeatures.clear();
        faceIDMap_.clear();

        // c. Build a global edge-to-faces map for adjacency analysis
        TopTools_IndexedDataMapOfShapeListOfShape edgeToFacesMap;
        TopExp::MapShapesAndAncestors(solid, TopAbs_EDGE, TopAbs_FACE, edgeToFacesMap);

        int faceIndex = 0;

        // d. Iterate over all faces and create atomic feature records
        for (TopExp_Explorer faceExplorer(solid, TopAbs_FACE); faceExplorer.More(); faceExplorer.Next())
        {
            TopoDS_Face face = TopoDS::Face(faceExplorer.Current());

            // e. Create atomic feature with unique ID
            auto feature = std::make_shared<AtomicFeature>();
            feature->faceID = partNode.partID + "_Face_" + std::to_string(faceIndex++);
            feature->brepFace = face;

            // f. Register face in the global face-to-ID map
            faceIDMap_[face] = feature->faceID;

            // g. Identify the geometric type and extract parameters
            auto [type, params] = IdentifyGeometryType(face);
            feature->atomType = type;
            feature->geometricParams = params;

            // h. For cylindrical faces, compute height along the axis
            if (type == AtomType::CYLINDER)
            {
                feature->geometricParams.height = ComputeCylinderHeight(face, params.locationPoint, params.axisVector);
            }

            // i. Determine concavity (convex/concave/neutral)
            feature->formType = DetermineConcavity(face, type, params, solid);

            // j. Compute surface area
            feature->area = ComputeFaceArea(face);

            // k. Mark as non-functional if area is below threshold
            if (feature->area < Constants::FaceAreaThreshold) feature->isFunctional = false;

            // l. Store the atomic feature in the part node
            partNode.atomicFeatures.push_back(feature);
        }

        // m. Analyze topological adjacency relationships for all faces
        for (const auto& feature : partNode.atomicFeatures)
        {
            feature->adjacencyList = AnalyzeTopologicalAdjacency(feature->brepFace, feature->faceID, solid, edgeToFacesMap);
        }
    }

    // ============================================================================
    // STEP 2: Atomic Feature Classification - Fragment Merging
    // ============================================================================

    void ASGBuilder::MergeFragmentedFeatures() const
    {
        int logicalFeatureCounter = 0;

        // a. Process each part independently
        for (auto& partNode : partNodes_)
        {
            // b. Collect candidate features for merging (curved surfaces only)
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

            // c. Compare all candidate pairs to find geometric matches
            for (size_t i = 0; i < candidates.size(); ++i)
            {
                const auto& mainFeat = candidates[i];
                if (mainFeat->isFragment) continue;

                // d. Start a new group with the main feature
                std::vector<std::shared_ptr<AtomicFeature>> group;
                group.push_back(mainFeat);

                // e. Search for matching features to merge
                for (size_t j = i + 1; j < candidates.size(); ++j)
                {
                    const auto& otherFeat = candidates[j];
                    if (otherFeat->isFragment) continue;

                    // f. Skip if type or form doesn't match
                    if (mainFeat->atomType != otherFeat->atomType || mainFeat->formType != otherFeat->formType)
                        continue;

                    bool match = false;
                    const auto& p1 = mainFeat->geometricParams;
                    const auto& p2 = otherFeat->geometricParams;

                    // g. For cylinders, check radius, axis parallelism, and co-linearity
                    if (mainFeat->atomType == AtomType::CYLINDER)
                    {
                        if (std::abs(p1.radius - p2.radius) < Constants::DistanceTolerance &&
                            p1.axisVector.IsParallel(p2.axisVector, Constants::AngleTolerance))
                        {
                            // h. Verify axes are co-linear by computing distance between them
                            if (gp_Lin axisLine(p1.locationPoint, p1.axisVector); axisLine.Distance(p2.locationPoint) <
                                Constants::DistanceTolerance)
                            {
                                match = true;
                            }
                        }
                    }
                    // Additional geometry types can be extended here

                    // i. Add matching feature to the group
                    if (match)
                    {
                        group.push_back(otherFeat);
                    }
                }

                // j. If multiple features were grouped, mark them as fragments
                if (group.size() > 1)
                {
                    // k. Generate a unique logical feature ID
                    const std::string logicalID = partNode.partID + "_Logical_" + std::to_string(logicalFeatureCounter++);
                    double totalArea = 0.0;
                    std::vector<std::string> ids;

                    // l. Mark all features in the group as fragments
                    for (auto& f : group)
                    {
                        f->isFragment = true;
                        f->logicalFeatureID = logicalID;
                        totalArea += f->area;
                        ids.push_back(f->faceID);
                    }

                    // m. Designate the first feature as the main representative
                    mainFeat->isMainFragment = true;
                    mainFeat->mergedArea = totalArea;
                    mainFeat->fragmentFaceIDs = ids;
                }
            }
        }
    }

    // ============================================================================
    // STEP 3: Composite Feature Recognition - Main Entry Point
    // ============================================================================

    void ASGBuilder::RecognizeCompositeFeatures()
    {
        std::cout << "[Recognizing Composite Features]" << std::endl;

        // a. Process each part independently
        for (auto& partNode : partNodes_)
        {
            // b. Sort atomic features by area (largest first) for better recognition
            std::ranges::sort(partNode.atomicFeatures,
                              [](const std::shared_ptr<AtomicFeature>& a, const std::shared_ptr<AtomicFeature>& b)
                              {
                                  return a->area > b->area;
                              });

            // c. Build a lookup map for quick access to atomic features
            FeatureMap featureMap;
            for (const auto& f : partNode.atomicFeatures)
            {
                featureMap[f->faceID] = f;
            }

            // d. Apply recognition rules in priority order
            for (auto& f : partNode.atomicFeatures)
            {
                // e. Skip features that have already been consumed
                if (f->isConsumed) continue;

                // f. Skip non-main fragments (only process the representative)
                if (f->isFragment && !f->isMainFragment) continue;

                // g. Attempt recognition in order of priority
                //    Rule 1: Recognize holes (concave cylinders)
                if (RecognizeHoleFeature(partNode, f, featureMap)) continue;

                //    Rule 2: Recognize shafts (convex cylinders)
                if (RecognizeShaftFeature(partNode, f, featureMap)) continue;

                //    Rule 3: Recognize step planes (planes adjacent to cylinders)
                if (RecognizeStepPlaneFeature(partNode, f, featureMap)) continue;

                //    Rule 4: Recognize functional planes (catch-all for large planes)
                if (RecognizeFunctionalPlaneFeature(partNode, f, featureMap)) continue;
            }
        }
    }

    // ============================================================================
    // STEP 4: Assembly Constraint Recognition - Main Entry Point
    // ============================================================================

    void ASGBuilder::BuildAssemblyConstraintGraph()
    {
        std::cout << "-----------------------------------------------" << std::endl;
        std::cout << "[Step 4] Building Assembly Constraint Graph..." << std::endl;

        // a. Clear any previously discovered constraints
        constraints_.clear();

        // b. Perform pairwise comparison between all parts
        for (size_t i = 0; i < partNodes_.size(); ++i)
        {
            for (size_t j = i + 1; j < partNodes_.size(); ++j)
            {
                const auto& nodeA = partNodes_[i];
                const auto& nodeB = partNodes_[j];

                // c. Broad-phase collision detection: check bounding box overlap
                if (!CheckBoundingBoxCollision(nodeA, nodeB))
                {
                    continue;
                }

                // d. Narrow-phase: match features between the two parts
                MatchPartPair(nodeA, nodeB);
            }
        }

        // e. Display discovered constraints
        std::cout << "Constraint Graph Built. Total Constraints: " << constraints_.size() << std::endl;
        for (const auto& c : constraints_)
        {
            std::cout << "  -> " << c.ToString() << std::endl;
        }
        std::cout << std::endl;
    }

    // ============================================================================
    // Test Function (Legacy Interface)
    // ============================================================================

    void ASGBuilder::runTest()
    {
        // a. Simple linkage verification function
        //    Logic is now driven by main.cpp
        std::cout << "ASGBuilder linked successfully. Ready for external commands." << std::endl;
    }

    // ============================================================================
    // Deep Learning Data Export - Graph Data Generation
    // ============================================================================

    std::vector<std::string> ASGBuilder::GetAllPartIDs() const
    {
        std::vector<std::string> ids;
        ids.reserve(partNodes_.size());

        for (const auto& node : partNodes_)
        {
            if (!node.partID.empty())
            {
                ids.push_back(node.partID);
            }
        }
        return ids;
    }

    bool ASGBuilder::CheckMaterialBetween(const std::string& partID, const std::vector<double>& p1_coords, const std::vector<double>& p2_coords) const
    {
        if (p1_coords.size() != 3 || p2_coords.size() != 3) {
            std::cerr << "[Error] CheckMaterialBetween expects 3D coordinates (x, y, z)." << std::endl;
            return false;
        }

        const PartNode* targetNode = nullptr;
        for (const auto& node : partNodes_) {
            if (node.partID == partID) {
                targetNode = &node;
                break;
            }
        }

        if (!targetNode) {
            std::cerr << "[Warning] PartID '" << partID << "' not found." << std::endl;
            return false;
        }


        gp_Pnt P1(p1_coords[0], p1_coords[1], p1_coords[2]);
        gp_Pnt P2(p2_coords[0], p2_coords[1], p2_coords[2]);

        gp_Trsf invTrsf = targetNode->transformation.Inverted();
        P1.Transform(invTrsf);
        P2.Transform(invTrsf);

        return IsMaterialBetween(P1, P2, targetNode->brepShape);
    }

    DeepLearningGraphData ASGBuilder::GetGraphDataForPart(const std::string& partID) const
    {
        DeepLearningGraphData data = {};

        // a. Find the target part node by ID
        const PartNode* targetPart = nullptr;
        for (const auto& node : partNodes_)
        {
            if (node.partID == partID)
            {
                targetPart = &node;
                break;
            }
        }

        // b. Return empty data if part is not found
        if (!targetPart) return data;

        // c. Build a mapping from face ID to numerical index (0, 1, 2...)
        std::map<std::string, int> faceIndexMap;
        int idx = 0;

        // d. Populate node features from atomic features
        for (const auto& feat : targetPart->atomicFeatures)
        {
            // e. Assign a unique index to each face
            faceIndexMap[feat->faceID] = idx;

            // f. Node features: AtomType, Area, Curvature
            data.nodeTypes.push_back(static_cast<int>(feat->atomType));
            data.nodeAreas.push_back(feat->area);
            data.nodeCurvatures.push_back(feat->geometricParams.curvature);

            // g. Placeholder for centroid coordinates (flattened x, y, z)
            //    For real GNN usage, implement proper centroid computation
            gp_Pnt center = GetFaceSamplePoint(feat->brepFace); // 或者使用 geometricParams.locationPoint
            data.nodeCentroids.push_back(center.X());
            data.nodeCentroids.push_back(center.Y());
            data.nodeCentroids.push_back(center.Z());

            idx++;
        }

        // h. Build edge list from adjacency relationships
        for (const auto& feat : targetPart->atomicFeatures)
        {
            // i. Get the source node index
            int srcIdx = faceIndexMap[feat->faceID];

            // j. Add edges to all adjacent faces
            for (const auto& [neighborFaceID, continuityType, dihedralAngle] : feat->adjacencyList)
            {
                // k. Verify that the neighbor exists in our feature set
                if (faceIndexMap.contains(neighborFaceID))
                {
                    int tgtIdx = faceIndexMap[neighborFaceID];

                    // l. Store edge indices
                    data.edgeSource.push_back(srcIdx);
                    data.edgeTarget.push_back(tgtIdx);

                    // m. Store edge attributes: dihedral angle and continuity type
                    data.edgeAngles.push_back(dihedralAngle);
                    data.edgeContinuity.push_back(static_cast<int>(continuityType));
                }
            }
        }

        // n. Optional: Export supervision labels if available (currently empty)
        //    Ready for future label integration

        return data;
    }
} // namespace ASG
