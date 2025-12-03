//
// Created by zhuge on 2025/11/19.
//

#pragma once

// ============================================================================
// ASGBuilder.h
// Refactored for Modern C++ (C++20) & OpenCASCADE 7.9
// ============================================================================

#include <string>
#include <vector>
#include <map>
#include <memory> // smart pointers
#include <iostream>
#include <optional> // C++17/20
#include <climits>

// ============== OpenCASCADE Core Headers ==============
#include <Standard_Handle.hxx>
#include <Standard_Transient.hxx>

// Topological Data Structures
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopLoc_Location.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <Bnd_Box.hxx>

// Geometry & Math
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax1.hxx>
#include <gp_Pln.hxx>
#include <Geom_Surface.hxx>

// XDE / STEP Import
#include <TDocStd_Document.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TDF_Label.hxx>

namespace ASG
{
    // ============================================================================
    // Constants
    // ============================================================================

    namespace Constants
    {
        constexpr double DistanceTolerance = 1e-6; // Distance tolerance used for radius comparisons and bounding-box enlargement
        constexpr double AngleTolerance = 1e-6; // Angular tolerance (radians) for axis, normal, and direction comparisons
        constexpr double FaceAreaThreshold = 0.001; // Minimum area to be considered functional
        constexpr double CoaxialRadiusDiffTolerance = 0.5; // Max radius difference for coaxial fit
        constexpr double CoincidentOverlapTolerance = 1.0; // Bounding box overlap tolerance for planes
        constexpr double ConcavityOffsetRatio = 0.5; // Ratio to move probe point
        constexpr double C0_Continuity_Threshold = 0.1; // Radians (~5.7 deg)
        constexpr double C1_Continuity_Threshold = 0.01; // Radians (~0.57 deg)
    }

    // ============================================================================
    // Enumerations
    // ============================================================================

    /**
     * @brief Enumerates supported atomic surface primitives
     * @details Used during surface classification; determines which geometric parameters are populated
     */
    enum class AtomType
    {
        PLANE,
        CYLINDER,
        CONE,
        SPHERE,
        TORUS,
        BSPLINE,
        OTHER
    };

    /**
     * @brief Indicates whether a surface is convex, concave, or indeterminate relative to its solid
     * @details Derived via curvature-center test and used to distinguish shafts vs. holes
     */
    enum class FormType
    {
        CONVEX,
        CONCAVE,
        NEUTRAL
    };

    /**
     * @brief Classifies continuity between neighboring faces
     * @details Encodes whether surfaces meet sharply (C0), tangentially (C1), smoothly (C2), or cannot be determined
     */
    enum class ContinuityType
    {
        C0, //sharply
        C1, //tangentially
        C2, //smoothly
        UNKNOWN //cannot be determined
    };

    /**
     * @brief Identifies high-level engineering features composed from atomic surfaces
     * @details Used by composite recognition to tag holes, shafts, planar mates, and other semantic groups
     */
    enum class CompositeFeatureType
    {
        UNKNOWN,
        HOLE,
        SHAFT,
        FUNCTIONAL_PLANE,
        STEP_PLANE,
    };

    /**
     * @brief Differentiates through-holes from blind holes
     * @details Derived during hole recognition to support downstream tolerance reasoning
     */
    enum class HoleType
    {
        THROUGH,
        BLIND,
        UNKNOWN
    };

    /**
     * @brief Enumerates supported assembly constraint relationships
     * @details Produced by constraint matching and consumed by downstream solvers
     */
    enum class ConstraintType
    {
        COAXIAL, // hole-shaft/hole-hole
        COINCIDENT, // face2face
        OFFSET // face2face with distance(not used currently)
    };

    // ==========================================================================
    // Data Structures
    // ==========================================================================

    /**
     * @brief Captures a single assembly constraint between two composite features
     * @details Provides part/feature identifiers plus type-specific numeric value for later graph export
     */
    struct AssemblyConstraint
    {
        ConstraintType type;

        std::string partID_A;
        std::string featureID_A;

        std::string partID_B;
        std::string featureID_B;

        double value = 0.0;

        [[nodiscard]] std::string ToString() const
        {
            std::string typeStr;
            switch (type)
            {
            case ConstraintType::COAXIAL: typeStr = "COAXIAL";
                break;
            case ConstraintType::COINCIDENT: typeStr = "COINCIDENT";
                break;
            case ConstraintType::OFFSET: typeStr = "OFFSET";
                break;
            }
            return "[" + typeStr + "] " + partID_A + ":" + featureID_A + " <--> " + partID_B + ":" + featureID_B;
        }
    };

    /**
     * @brief Container for geometry parameters shared across all atomic feature types
     * @details Holds axis, radii, height, and curvature values derived during IdentifyGeometryType
     */
    struct GeometricParams
    {
        gp_Dir axisVector; // Normal for Plane, Axis for Cylinder/Cone
        gp_Pnt locationPoint; // Location for Plane, Axis Point for Cylinder
        double radius = 0.0; // Radius (Cylinder, Cone, Sphere)
        double height = 0.0; // Height/Depth along axis
        double semiAngle = 0.0; // Cone semi-angle
        double majorRadius = 0.0; // Torus
        double minorRadius = 0.0; // Torus
        double curvature = 0.0; // Pre-computed curvature feature (Mean Curvature approximation)
    };

    /**
     * @brief Records adjacency metadata for a single neighboring face
     * @details Stores neighbor identifier, continuity classification, and measured dihedral angle
     */
    struct AdjacencyInfo
    {
        std::string neighborFaceID;
        ContinuityType continuityType = ContinuityType::UNKNOWN;
        double dihedralAngle = 0.0;
    };

    /**
     * @brief Describes a single atomic surface along with derived attributes
     * @details Contains the OCC face handle, geometry classification, adjacency list, and fragment metadata
     */
    struct AtomicFeature
    {
        std::string faceID;
        TopoDS_Face brepFace;
        AtomType atomType = AtomType::OTHER;
        GeometricParams geometricParams;
        FormType formType = FormType::NEUTRAL;
        std::vector<AdjacencyInfo> adjacencyList;
        double area = 0.0;
        bool isFunctional = true;
        bool isFragment = false;
        bool isMainFragment = false;
        std::string logicalFeatureID;
        double mergedArea = 0.0;
        std::vector<std::string> fragmentFaceIDs;
        bool isConsumed = false;
    };

    /**
     * @brief Describes an engineering composite feature constructed from atomic surfaces
     * @details Stores identifiers, geometry summaries, and child face relationships for graph export
     */
    struct CompositeFeature
    {
        std::string compositeID;
        CompositeFeatureType type = CompositeFeatureType::UNKNOWN;
        std::string partID;
        std::vector<std::string> childAtomicFeatureIDs;
        gp_Ax1 axis;
        double nominalRadius = 0.0;
        double height = 0.0;
        gp_Dir planeNormal;
        gp_Pnt planeLocation;
        double planeArea = 0.0;
        double width = 0.0;
        HoleType holeSubType = HoleType::UNKNOWN;
    };

    /**
     * @brief Key used to cluster fragmented faces that share the same underlying surface
     * @details Combines the OCC surface handle with concavity state for use in ordered containers
     */
    struct MergeKey
    {
        Handle(Geom_Surface) surface;
        FormType formType;

        bool operator<(const MergeKey& other) const
        {
            if (surface.get() != other.surface.get())
            {
                return surface.get() < other.surface.get();
            }
            return static_cast<int>(formType) < static_cast<int>(other.formType);
        }
    };

    /**
     * @brief Represents a single part including raw shape data and derived features
     * @details Holds the local B-Rep, transformation to world space, atomic features, and recognized composites
     */
    struct PartNode
    {
        std::string partID;
        TopoDS_Shape brepShape;
        gp_Trsf transformation;
        Bnd_Box worldBoundingBox;

        // Use shared_ptr to prevent pointer invalidation when vector resizes
        std::vector<std::shared_ptr<AtomicFeature>> atomicFeatures;

        std::vector<CompositeFeature> compositeFeatures;

        PartNode() = default;
    };

    /**
     * @brief Flattened graph structure consumed by Python/GNN pipelines
     * @details Captures node features, edge indices, and supervision labels exported via GetGraphDataForPart
     */
    struct DeepLearningGraphData
    {
        // Node feature channels
        std::vector<int> nodeTypes;
        std::vector<double> nodeAreas;
        std::vector<double> nodeCurvatures;
        std::vector<double> nodeCentroids;

        // Edge index storage
        std::vector<int> edgeSource;
        std::vector<int> edgeTarget;

        // Edge attributes
        std::vector<double> edgeAngles;
        std::vector<int> edgeContinuity;

        // Supervision labels
        std::vector<int> nodeLabelTypes;
        std::vector<int> nodeLabelIndices;

        [[nodiscard]] bool IsEmpty() const { return nodeTypes.empty(); }
    };


    // ============================================================================
    // Main Class: ASGBuilder
    // ============================================================================

    class ASGBuilder
    {
    public:
        /**
         * @brief Construct ASGBuilder and allocate an XDE document for downstream import
         * @details Instantiates the OCCT application context and prepares an empty document handle
         * @return Initialized ASGBuilder instance ready for STEP loading
         */
        ASGBuilder();

        /**
         * @brief Destroy ASGBuilder and release document resources
         * @details Closes the XDE document if allocated to avoid handle leaks
         */
        ~ASGBuilder();

        /**
         * @brief Load a STEP assembly into the internal XDE document
         * @details Configures STEPCAF reader, transfers data, and populates shapeTool_
         * @param filePath input: absolute or relative path to a STEP file
         * @return output: true when reading/transferring succeed, otherwise false
         */
        [[nodiscard]] bool LoadAssemblyFromSTEP(const std::string& filePath);

        /**
         * @brief Traverse the assembly tree and collect PartNode entries
         * @details Uses shapeTool_ to find free shapes, recursing through assembly/component labels
         */
        void ParseAssemblyTree();

        /**
         * @brief Identify atomic features for every part in partNodes_
         * @details Classifies each face, captures adjacency, and merges fragments prior to composite recognition
         */
        void ClassifyAtomicFeatures();

        /**
         * @brief Apply rule-based recognition to convert atomic features into composite features
         * @details Detects holes, shafts, step planes, and functional planes by inspecting adjacency patterns
         */
        void RecognizeCompositeFeatures();

        /**
         * @brief Build the assembly constraint graph between recognized composite features
         * @details Checks bounding boxes, matches coaxial/planar pairs, and stores resulting constraints
         */
        void BuildAssemblyConstraintGraph();

        /**
         * @brief Print part/feature counts to the console for quick diagnostics
         */
        void PrintStatistics() const;

        /**
         * @brief Serialize current analysis data to a JSON file
         * @details Writes part IDs and composite feature summaries in a lightweight interchange format
         * @param filePath input: destination JSON path
         * @return output: true when file writing succeeds, false otherwise
         */
        [[nodiscard]] bool ExportToJSON(const std::string& filePath) const;

        /**
         * @brief Simple linkage smoke test invoked from legacy entry points
         * @details Currently emits a readiness message to stdout
         */
        static void runTest();

        /**
         * @brief Check whether the midpoint of a segment between two points lies inside the solid
         * @details Used to distinguish slot vs. tongue by probing material occupancy
         * @param p1 input: first sample point
         * @param p2 input: second sample point
         * @param solid input: solid used for inside-outside test
         * @return output: true if midpoint is inside the solid, otherwise false
         */
        static bool IsMaterialBetween(const gp_Pnt& p1, const gp_Pnt& p2, const TopoDS_Shape& solid);

        /**
         * @brief Retrieve the current list of inferred assembly constraints
         * @return output: copy of the internal constraint vector
         */
        [[nodiscard]] std::vector<AssemblyConstraint> GetAssemblyConstraints() const { return constraints_; }

        /**
         * @brief Retrieve a list of all Part IDs found in the assembly
         * @details Used by Python scripts to iterate over parts for batch processing
         * @return Vector of Part ID strings
         */
        [[nodiscard]] std::vector<std::string> GetAllPartIDs() const;

        /**
         * @brief Generate flattened graph data for a specific part ID
         * @details Converts atomic features and adjacency information into arrays consumable by PyG
         * @param partID input: identifier of the part to export
         * @return output: populated DeepLearningGraphData structure (empty if part not found)
         */
        [[nodiscard]] DeepLearningGraphData GetGraphDataForPart(const std::string& partID) const;

        /**
         * @brief Python-friendly wrapper for material check
         * @param partID Input Part ID string
         * @param p1_coords List of 3 doubles [x, y, z]
         * @param p2_coords List of 3 doubles [x, y, z]
         * @return true if material exists between points
         */
        [[nodiscard]] bool CheckMaterialBetween(const std::string& partID, const std::vector<double>& p1_coords,
                                                const std::vector<double>& p2_coords) const;

    private:
        // Internal logic helpers (Step 1)
        /**
         * @brief Recursively extract PartNode entries from an XDE label
         * @details Handles both assembly and simple-shape labels while accumulating the applied transformation
         * @param label input: XDE label under inspection
         * @param transformation input: cumulative transformation from parent assemblies
         */
        void ExtractPartsFromLabel(const TDF_Label& label, const gp_Trsf& transformation);

        // Internal logic helpers (Step 2)
        /**
         * @brief Classify every face in the given part into AtomicFeature records
         * @details Computes geometry, concavity, area, adjacency, and fragment metadata for downstream processing
         * @param partNode input/output: part structure receiving freshly computed atomicFeatures
         */
        void ClassifyPartAtomicFeatures(PartNode& partNode);

        /**
         * @brief Merge atomic features that belong to the same analytical surface
         * @details Collapses curved fragments into logical groups, assigning shared IDs and accumulated areas
         */
        void MergeFragmentedFeatures() const;

        /**
         * @brief Inspect the OCC surface backing a face and return its AtomType plus parameters
         * @param face input: face whose geometry is analyzed
         * @return output: pair of AtomType and filled GeometricParams
         */
        static std::pair<AtomType, GeometricParams> IdentifyGeometryType(const TopoDS_Face& face);

        /**
         * @brief Determine whether a classified face is convex, concave, or neutral
         * @param face input: target face
         * @param atomType input: previously detected surface type
         * @param geomParams input: geometry parameters including center point
         * @param parentSolid input: solid used for inside/outside testing
         * @return output: FormType result describing concavity
         */
        static FormType DetermineConcavity(
            const TopoDS_Face& face, const AtomType& atomType,
            const GeometricParams& geomParams, const TopoDS_Shape& parentSolid);

        /**
         * @brief Measure the dihedral angle between two faces sharing the same edge
         * @details Evaluates normals at the edge midpoint and returns the angle in radians
         */
        static double ComputeEdgeDihedralAngle(const TopoDS_Edge& edge, const TopoDS_Face& f1, const TopoDS_Face& f2);

        /**
         * @brief Build an adjacency list for the supplied face within a parent solid
         * @details Iterates over incident edges, locates neighboring faces, and stores continuity and angles
         * @param face input: face under analysis
         * @param faceID input: identifier associated with the face
         * @param parentSolid input: solid needed for edge-to-face mapping
         * @param edgeToFacesMap input: pre-computed map of edges to incident faces
         * @return output: vector of AdjacencyInfo entries
         */
        std::vector<AdjacencyInfo> AnalyzeTopologicalAdjacency(
            const TopoDS_Face& face, const std::string& faceID,
            const TopoDS_Shape& parentSolid,
            const TopTools_IndexedDataMapOfShapeListOfShape& edgeToFacesMap);

        // Internal logic helpers (Step 3)
        // Using shared_ptr for map values to prevent pointer invalidation
        using FeatureMap = std::map<std::string, std::shared_ptr<AtomicFeature>>;

        /**
         * @brief Recognize hole features composed of concave cylindrical surfaces
         * @details Accepts candidate concave cylinders, optionally merges fragments, and records blind/through metadata
         * @param partNode input/output: part that receives the new composite feature
         * @param feature input: candidate atomic cylinder
         * @param featureMap input/output: lookup table used to mark faces as consumed
         * @return output: true if a hole feature was recognized, otherwise false
         */
        static bool RecognizeHoleFeature(PartNode& partNode, const std::shared_ptr<AtomicFeature>& feature, FeatureMap& featureMap);

        /**
         * @brief Recognize shaft features composed of convex cylindrical surfaces
         * @details Mirrors the hole logic but for protruding geometry used in mating conditions
         * @param partNode input/output: part that receives the new composite feature
         * @param feature input: candidate atomic cylinder
         * @param featureMap input/output: lookup table used to mark faces as consumed
         * @return output: true if a shaft feature was recognized, otherwise false
         */
        static bool RecognizeShaftFeature(PartNode& partNode, const std::shared_ptr<AtomicFeature>& feature, FeatureMap& featureMap);

        /**
         * @brief Recognize step planes adjacent to cylindrical features via sharp (C0) edges
         * @details Promotes neutral planes that neighbor cylinders into STEP_PLANE composites
         * @param partNode input/output: owning part
         * @param feature input: candidate atomic plane
         * @param featureMap input/output: map used to find adjacent faces
         * @return output: true if a step plane was recorded, otherwise false
         */
        static bool RecognizeStepPlaneFeature(PartNode& partNode, const std::shared_ptr<AtomicFeature>& feature, FeatureMap& featureMap);

        /**
         * @brief Recognize functional planes that remain unconsumed yet exceed area/importance thresholds
         * @details Acts as a catch-all to ensure significant planar surfaces are captured for mating relations
         * @param partNode input/output: owning part
         * @param feature input: candidate atomic plane
         * @param featureMap input/output: map used for state tracking
         * @return output: true if the plane was promoted to a composite feature, otherwise false
         */
        static bool RecognizeFunctionalPlaneFeature(PartNode& partNode, const std::shared_ptr<AtomicFeature>& feature,
                                                    FeatureMap& featureMap);

        /**
         * @brief Compute the surface area of a given face
         * @details Uses BRepGProp to calculate the precise surface area via mass properties
         * @param face input: face whose area is measured
         * @return output: surface area in square units (typically mm2)
         */
        static double ComputeFaceArea(const TopoDS_Face& face);

        /**
         * @brief Sample a representative point on the face surface
         * @details Evaluates the face at the midpoint of its UV parameter domain
         * @param face input: face from which to extract a sample point
         * @return output: 3D point in world coordinates
         */
        static gp_Pnt GetFaceSamplePoint(const TopoDS_Face& face);

        /**
         * @brief Measure the height of a cylindrical face along its axis
         * @details Projects all edge vertices onto the axis and computes the span distance
         * @param face input: cylindrical face to measure
         * @param axisPoint input: reference point on the cylinder axis
         * @param axisVector input: direction vector of the cylinder axis
         * @return output: height measured along the axis direction
         */
        static double ComputeCylinderHeight(const TopoDS_Face& face, const gp_Pnt& axisPoint, const gp_Dir& axisVector);

        /**
         * @brief Evaluate all feature pairs between two parts and record compatible constraints
         * @details Iterates over composite features in both parts and delegates to coaxial/planar matchers
         * @param nodeA input: first part in the pair
         * @param nodeB input: second part in the pair
         */
        void MatchPartPair(const PartNode& nodeA, const PartNode& nodeB);

        /**
         * @brief Test whether two cylindrical features form a valid coaxial constraint
         * @details Checks axis parallelism, distance from axis, and radius compatibility
         * @param nodeA input: part containing the first feature
         * @param featA input: first cylindrical feature (hole or shaft)
         * @param nodeB input: part containing the second feature
         * @param featB input: second cylindrical feature (hole or shaft)
         */
        void MatchCoaxial(const PartNode& nodeA, const CompositeFeature& featA,
                          const PartNode& nodeB, const CompositeFeature& featB);

        /**
         * @brief Test whether two planar features form a valid coincident constraint
         * @details Verifies that normals are opposite, planes are co-planar, and bounding boxes overlap
         * @param nodeA input: part containing the first feature
         * @param featA input: first planar feature (functional or step plane)
         * @param nodeB input: part containing the second feature
         * @param featB input: second planar feature (functional or step plane)
         */
        void MatchCoincident(const PartNode& nodeA, const CompositeFeature& featA,
                             const PartNode& nodeB, const CompositeFeature& featB);

        /**
         * @brief Transform a local axis to world coordinates using the provided transformation
         * @details Applies the gp_Trsf to both the axis location and direction
         * @param localAxis input: axis in part-local coordinates
         * @param trsf input: transformation matrix from local to world space
         * @return output: transformed axis in world coordinates
         */
        static gp_Ax1 TransformAxis(const gp_Ax1& localAxis, const gp_Trsf& trsf);

        /**
         * @brief Transform a local point to world coordinates using the provided transformation
         * @details Applies the gp_Trsf to the point coordinates
         * @param localPnt input: point in part-local coordinates
         * @param trsf input: transformation matrix from local to world space
         * @return output: transformed point in world coordinates
         */
        static gp_Pnt TransformPoint(const gp_Pnt& localPnt, const gp_Trsf& trsf);

        /**
         * @brief Transform a local direction vector to world coordinates using the provided transformation
         * @details Applies the gp_Trsf to the direction components (ignores translation)
         * @param localDir input: direction in part-local coordinates
         * @param trsf input: transformation matrix from local to world space
         * @return output: transformed direction in world coordinates
         */
        static gp_Dir TransformDir(const gp_Dir& localDir, const gp_Trsf& trsf);

        /**
         * @brief Perform broad-phase collision detection between two parts using axis-aligned bounding boxes
         * @details Computes bounding boxes in world space, enlarges by tolerance, and tests for overlap
         * @param nodeA input: first part to test
         * @param nodeB input: second part to test
         * @return output: true if bounding boxes overlap (potential contact), otherwise false
         */
        static bool CheckBoundingBoxCollision(const PartNode& nodeA, const PartNode& nodeB);

        // Member Data
        Handle(TDocStd_Document) doc_; // XDE document holding the imported STEP assembly
        Handle(XCAFDoc_ShapeTool) shapeTool_; // XDE shape tool for navigating the assembly hierarchy

        std::vector<PartNode> partNodes_; // Collection of all extracted parts with their features

        // Custom comparator for using TopoDS_Face as a map key
        struct FaceComparator
        {
            bool operator()(const TopoDS_Face& f1, const TopoDS_Face& f2) const
            {
                // 1. Compare TShape pointers
                if (f1.TShape().get() != f2.TShape().get())
                {
                    return f1.TShape().get() < f2.TShape().get();
                }

                // 2. Compare Location Hash
                const int h1 = static_cast<int>(f1.Location().HashCode());

                if (const int h2 = static_cast<int>(f2.Location().HashCode()); h1 != h2)
                {
                    return h1 < h2;
                }

                // 3. Compare Orientation
                return f1.Orientation() < f2.Orientation();
            }
        };

        std::map<TopoDS_Face, std::string, FaceComparator> faceIDMap_; // Bidirectional lookup: face to unique ID

        /**
         * @brief Store the list of assembly constraints discovered during graph construction
         * @details Populated by BuildAssemblyConstraintGraph and consumed by downstream clients
         */
        std::vector<AssemblyConstraint> constraints_;
    };
} // namespace ASG
