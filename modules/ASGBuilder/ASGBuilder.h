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
#include <set>
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
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS.hxx>
#include <TopLoc_Location.hxx>
#include <TopExp_Explorer.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>

// Geometry & Math
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pln.hxx>
#include <gp_Cylinder.hxx>
#include <Geom_Surface.hxx>
#include <Geom_Curve.hxx>

// XDE / STEP Import
#include <TDocStd_Document.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TDF_Label.hxx>

namespace ASG
{
    // ============================================================================
    // Constants / 常量定义
    // ============================================================================

    namespace Constants
    {
        /**
         * @brief Tolerance for distance/radius comparison (mm)
         * 距离和半径比较的容差 (1e-6 mm)
         */
        constexpr double DistanceTolerance = 1e-6;

        /**
         * @brief Tolerance for angle comparison (radians)
         * 角度和矢量比较的容差 (rad)
         */
        constexpr double AngleTolerance = 1e-6;
    }

    // ============================================================================
    // Enums / 枚举类型
    // ============================================================================

    /**
     * @brief Atomic Geometry Type
     * 原子几何类型枚举
     */
    enum class AtomType
    {
        PLANE, // 平面
        CYLINDER, // 圆柱面
        CONE, // 圆锥面
        SPHERE, // 球面
        TORUS, // 环面
        BSPLINE, // B样条曲面
        OTHER // 其他
    };

    /**
     * @brief Form Type (Concavity)
     * 形态类型枚举（凹/凸）
     */
    enum class FormType
    {
        CONVEX, // 凸出（外部表面）
        CONCAVE, // 凹陷（内部特征）
        NEUTRAL // 中性（无法判断）
    };

    /**
     * @brief Geometric Continuity Type
     * 几何连续性类型
     */
    enum class ContinuityType
    {
        C0, // 位置连续（尖锐边）
        C1, // 相切连续（圆角）
        C2, // 曲率连续
        UNKNOWN // 未知
    };

    /**
     * @brief Composite Feature Type
     * 复合特征类型枚举 (有工程意义的特征)
     */
    enum class CompositeFeatureType
    {
        UNKNOWN,
        HOLE, // 孔 (凹陷圆柱)
        SHAFT, // 轴 (凸出圆柱)
        FUNCTIONAL_PLANE, // 功能平面 (装配贴合面)
        STEP_PLANE, // 台阶面 (与孔/轴相邻的平面)
        SLOT, // 槽 (由平面组成的凹槽)
        TONGUE, // 榫 (由平面组成的凸出部分)
    };

    /**
     * @brief Hole Type
     * 孔类型枚举
     */
    enum class HoleType
    {
        THROUGH, // 通孔
        BLIND, // 盲孔
        UNKNOWN
    };

    /**
     * @brief Assembly Constraint Type
     * 装配约束类型
     */
    enum class ConstraintType
    {
        COAXIAL, // 同轴 (孔-轴)
        COINCIDENT, // 贴合 (面-面)
        PRISMATIC, // 棱柱配合 (键-槽)
        OFFSET // 距离 (面-面有间距)
    };

    /**
     * @brief Assembly Constraint Structure
     * 装配约束结构：描述两个零件特征之间的关系
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
            case ConstraintType::PRISMATIC: typeStr = "PRISMATIC";
                break;
            case ConstraintType::OFFSET: typeStr = "OFFSET";
                break;
            }
            return "[" + typeStr + "] " + partID_A + ":" + featureID_A + " <--> " + partID_B + ":" + featureID_B;
        }
    };


    // ============================================================================
    // Data Structures / 数据结构
    // ============================================================================

    /**
     * @brief Geometric Parameters for various shapes
     * 通用几何参数结构
     */
    struct GeometricParams
    {
        // Common
        gp_Dir axisVector; // Normal for Plane, Axis for Cylinder/Cone
        gp_Pnt locationPoint; // Location for Plane, Axis Point for Cylinder

        // Dimensions
        double radius = 0.0; // Radius (Cylinder, Cone, Sphere)
        double height = 0.0; // Height/Depth along axis
        double semiAngle = 0.0; // Cone semi-angle
        double majorRadius = 0.0; // Torus
        double minorRadius = 0.0; // Torus
    };

    /**
     * @brief Adjacency Information
     * 邻接信息结构
     */
    struct AdjacencyInfo
    {
        std::string neighborFaceID;
        ContinuityType continuityType = ContinuityType::UNKNOWN;
        double dihedralAngle = 0.0;     //二面角 (弧度制)，范围 [0, PI]
    };

    /**
     * @brief Atomic Feature Structure
     * 原子特征结构：存储一个面的完整几何与拓扑信息
     */
    struct AtomicFeature
    {
        std::string faceID;
        TopoDS_Face brepFace; // TopoDS_Face is a value-type handle (lightweight)

        // Step 2 Results
        AtomType atomType = AtomType::OTHER;
        GeometricParams geometricParams;
        FormType formType = FormType::NEUTRAL;
        std::vector<AdjacencyInfo> adjacencyList;

        // Metadata
        double area = 0.0;
        bool isFunctional = true;

        // Fragment Merging (Step 2.5)
        bool isFragment = false; // Is this a split face? / 是否为碎片
        bool isMainFragment = false; // Is this the representative? / 是否为主特征
        std::string logicalFeatureID; // Shared ID for merged features / 逻辑特征ID
        double mergedArea = 0.0;
        std::vector<std::string> fragmentFaceIDs;

        // Step 3 Status
        bool isConsumed = false; // Already used in a composite feature? / 是否已被消耗

        AtomicFeature() = default;
    };

    /**
     * @brief Composite Feature Structure
     * 复合特征结构：工程语义层面的特征
     */
    struct CompositeFeature
    {
        std::string compositeID;
        CompositeFeatureType type = CompositeFeatureType::UNKNOWN;
        std::string partID;

        // Links to atomic features / 关联的原子特征ID
        std::vector<std::string> childAtomicFeatureIDs;

        // Geometric Summary (Cached for quick access) / 几何简报
        gp_Ax1 axis; // For rotational features
        double nominalRadius = 0.0;
        double height = 0.0;

        gp_Dir planeNormal; // For planar features
        gp_Pnt planeLocation;
        double planeArea = 0.0;
        double width = 0.0; // For slots/tongues

        HoleType holeSubType = HoleType::UNKNOWN; // For holes-subType

        CompositeFeature() = default;
    };

    /**
     * @brief Merge Key for fragment identification
     * 合并键：用于识别属于同一几何面的碎片
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
     * @brief Part Node Structure
     * 零件节点结构
     */
    struct PartNode
    {
        std::string partID;
        TopoDS_Shape brepShape;
        gp_Trsf transformation;

        // Use shared_ptr to prevent pointer invalidation when vector resizes
        // 使用 shared_ptr 防止 vector 扩容时指针失效 (关键修改)
        std::vector<std::shared_ptr<AtomicFeature>> atomicFeatures;

        std::vector<CompositeFeature> compositeFeatures;

        PartNode() = default;
    };

    // [新增] 专门用于传输给 Python/GNN 的扁平化图数据结构
    // [NEW] Flattened Graph Data Structure for Python/GNN Bridge
    struct DeepLearningGraphData {
        // Node Features (N x D)
        std::vector<int> nodeTypes;        // One-hot encoding source
        std::vector<double> nodeAreas;     // Normalized area
        std::vector<double> nodeCurvatures; // Approx. curvature value
        std::vector<double> nodeCentroids;  // x, y, z flattened

        // Edge Index (2 x E) - COO format for PyTorch Geometric
        std::vector<int> edgeSource;
        std::vector<int> edgeTarget;

        // Edge Attributes (E x F)
        std::vector<double> edgeAngles;    // Dihedral angles (rad)
        std::vector<int> edgeContinuity;   // C0, C1, C2 encoded
    };



    // ============================================================================
    // Main Class: ASGBuilder
    // ============================================================================

    class ASGBuilder
    {
    public:
        /**
         * @brief Constructor for ASGBuilder
         * ASGBuilder 构造函数
         * @details Creates XDE document for CAD data management
         * 创建用于 CAD 数据管理的 XDE 文档
         * @input 无输入参数
         * @output 初始化的 ASGBuilder 对象
         */
        ASGBuilder();

        /**
         * @brief Destructor for ASGBuilder
         * ASGBuilder 析构函数
         * @details Closes XDE document and releases resources
         * 关闭 XDE 文档并释放资源
         * @input 无输入参数
         * @output 无返回值
         */
        ~ASGBuilder();

        // --- Step 1: CAD Data Parsing / 数据解析 ---

        /**
         * @brief Load assembly from STEP file
         * 从 STEP 文件加载装配体
         * @details Reads STEP file and transfers data to internal XDE document structure
         * 读取 STEP 文件并将数据传输到内部 XDE 文档结构
         * @param filePath 输入的 STEP 文件路径 / Input STEP file path
         * @return 加载成功返回 true，失败返回 false / Returns true if successful, false otherwise
         */
        [[nodiscard]] bool LoadAssemblyFromSTEP(const std::string& filePath);

        /**
         * @brief Parse assembly tree structure
         * 解析装配树结构
         * @details Recursively traverses XDE assembly tree and extracts all parts with their transformations
         * 递归遍历 XDE 装配树并提取所有零件及其变换
         * @input 无显式输入参数（使用内部 shapeTool_）/ No explicit input (uses internal shapeTool_)
         * @output 填充 partNodes_ 向量 / Populates partNodes_ vector
         */
        void ParseAssemblyTree();

        // --- Step 2: Atomic Classification / 原子特征分类 ---

        /**
         * @brief Classify atomic geometric features for all parts
         * 对所有零件进行原子几何特征分类
         * @details Analyzes each face in every part to identify geometry type, form type (concave/convex),
         *          and topological adjacency. Also merges fragmented features that belong to same geometric surface.
         * 分析每个零件中的每个面，识别几何类型、形态类型（凹/凸）和拓扑邻接关系。同时合并属于同一几何面的碎片特征。
         * @input 无显式输入参数（使用内部 partNodes_）/ No explicit input (uses internal partNodes_)
         * @output 填充每个零件的 atomicFeatures 向量 / Populates atomicFeatures vector for each part
         */
        void ClassifyAtomicFeatures();

        // --- Step 3: Composite Recognition / 复合特征识别 ---

        /**
         * @brief Recognize engineering composite features from atomic features
         * 从原子特征中识别工程复合特征
         * @details Applies recognition rules to identify holes, shafts, slots, tongues, step planes,
         *          and functional planes based on atomic feature combinations and adjacency relationships.
         * 应用识别规则识别孔、轴、槽、榫、台阶面和功能平面，基于原子特征组合和邻接关系。
         * @input 无显式输入参数（使用内部 partNodes_）/ No explicit input (uses internal partNodes_)
         * @output 填充每个零件的 compositeFeatures 向量 / Populates compositeFeatures vector for each part
         */
        void RecognizeCompositeFeatures();

        // --- Step 4: Assembly Relationship Recognition ---
        void BuildAssemblyConstraintGraph();

        // --- Output & Debug / 输出与调试 ---

        /**
         * @brief Print statistical information to console
         * 向控制台打印统计信息
         * @details Outputs the total number of parts and composite features recognized
         * 输出识别的零件总数和复合特征总数
         * @input 无输入参数 / No input parameters
         * @output 向标准输出打印统计信息 / Prints statistics to standard output
         */
        void PrintStatistics() const;

        /**
         * @brief Export analysis results to JSON file
         * 将分析结果导出到 JSON 文件
         * @details Serializes part information and composite features to JSON format
         * 将零件信息和复合特征序列化为 JSON 格式
         * @param filePath 输出 JSON 文件路径 / Output JSON file path
         * @return 导出成功返回 true，失败返回 false / Returns true if export succeeds, false otherwise
         */
        [[nodiscard]] bool ExportToJSON(const std::string& filePath) const;

        // --- Public Test Helper (Matches previous main.cpp) ---
        /**
         * @brief Public test helper function
         * 公开测试辅助函数
         * @details Simple test function for verifying ASGBuilder linkage
         * 用于验证 ASGBuilder 链接的简单测试函数
         * @input 无输入参数 / No input parameters
         * @output 向标准输出打印测试消息 / Prints test message to standard output
         */
        static void runTest();

        /**
        * @brief Check whether the midpoint of a segment between two points lies inside the solid
        * 检查两点连线中点是否位于实体内部
        * @details Used to distinguish slot vs. tongue by probing material occupancy
        * 通过采样中点判断材料分布，用于区分槽/榫特征
        * @param p1 输入：第一采样点 / First sample point
        * @param p2 输入：第二采样点 / Second sample point
        * @param solid 输入：待测试的实体 / Solid used for inside-outside test
        * @return 输出：位于实体内部返回 true，否则返回 false / true if midpoint is inside the solid
        */
        static bool IsMaterialBetween(const gp_Pnt& p1, const gp_Pnt& p2, const TopoDS_Shape& solid);

        // [新增] 获取用于深度学习的图数据
        // [NEW] Get graph data formatted for PyTorch Geometric
        [[nodiscard]] DeepLearningGraphData GetGraphDataForPart(const std::string& partID) const;

    private:
        // Internal logic helpers (Step 1)
        /**
         * @brief Recursively extract parts from XDE label
         * 递归从 XDE 标签提取零件
         * @details Traverses assembly hierarchy, accumulates transformations, and extracts solid parts
         * 遍历装配层次结构，累积变换，提取实体零件
         * @param label XDE 标签节点 / XDE label node to process
         * @param transformation 累积的变换矩阵 / Accumulated transformation matrix
         * @output 向 partNodes_ 向量添加零件节点 / Adds part nodes to partNodes_ vector
         */
        void ExtractPartsFromLabel(const TDF_Label& label, const gp_Trsf& transformation);

        // Internal logic helpers (Step 2)
        /**
         * @brief Classify atomic features for a single part
         * 对单个零件进行原子特征分类
         * @details Iterates through all faces, identifies geometry type, determines concavity, and analyzes adjacency
         * 遍历所有面，识别几何类型，确定凹凸性，分析邻接关系
         * @param partNode 要分类的零件节点（引用）/ Part node to classify (by reference)
         * @output 填充 partNode.atomicFeatures 和 faceIDMap_ / Populates partNode.atomicFeatures and faceIDMap_
         */
        void ClassifyPartAtomicFeatures(PartNode& partNode);

        /**
         * @brief Merge fragmented features belonging to same geometric surface
         * 合并属于同一几何面的碎片特征
         * @details Groups faces split by trim curves or seams into logical features
         * 将因修剪曲线或缝合线分割的面组合成逻辑特征
         * @input 无显式输入参数（使用内部 partNodes_）/ No explicit input (uses internal partNodes_)
         * @output 标记碎片特征并设置逻辑特征 ID / Marks fragment features and sets logical feature IDs
         */
        void MergeFragmentedFeatures() const;

        // /**
        //  * @brief Create atomic feature from a face (deprecated/unused helper)
        //  * 从面创建原子特征（已弃用/未使用的辅助函数）
        //  * @details This function signature exists but is not currently used in the implementation
        //  * 此函数签名存在但当前实现中未使用
        //  * @param face 面对象 / Face object
        //  * @param faceIndex 面索引 / Face index
        //  * @param parentSolid 父实体 / Parent solid
        //  * @param partID 零件 ID / Part ID
        //  * @return 原子特征共享指针 / Shared pointer to atomic feature
        //  */
        // std::shared_ptr<AtomicFeature> CreateAtomicFeature(
        //     const TopoDS_Face& face, int faceIndex,
        //     const TopoDS_Shape& parentSolid, const std::string& partID);

        /**
         * @brief Identify geometry type and extract geometric parameters
         * 识别几何类型并提取几何参数
         * @details Analyzes underlying surface to classify as plane, cylinder, cone, sphere, torus, or BSpline
         * 分析底层曲面，分类为平面、圆柱、圆锥、球面、环面或 B 样条
         * @param face 要分析的面 / Face to analyze
         * @return 几何类型和参数的配对 / Pair of atom type and geometric parameters
         */
        static std::pair<AtomType, GeometricParams> IdentifyGeometryType(const TopoDS_Face& face);

        /**
         * @brief Determine concavity (form type) of a face
         * 确定面的凹凸性（形态类型）
         * @details Uses curvature center test: if center is inside solid, surface is convex; if outside, concave
         * 使用曲率中心测试：中心在实体内则凸出，在外则凹陷
         * @param face 要分析的面 / Face to analyze
         * @param atomType 几何类型 / Atom type
         * @param geomParams 几何参数 / Geometric parameters
         * @param parentSolid 父实体（用于内外判断）/ Parent solid (for inside/outside test)
         * @return 形态类型（CONVEX, CONCAVE, 或 NEUTRAL）/ Form type (CONVEX, CONCAVE, or NEUTRAL)
         */
        static FormType DetermineConcavity(
            const TopoDS_Face& face, const AtomType& atomType,
            const GeometricParams& geomParams, const TopoDS_Shape& parentSolid);


        /**
         * @brief Compute the dihedral angle (in radians) between two faces along a shared edge.
         * 计算两个面在共享边处的二面角（弧度制）。
         * Logic:
         * 1. Sample the midpoint of the edge.
         * 2. Get UV coordinates of this point on both faces.
         * 3. Compute surface normals at UV points.
         * 4. Correct normal direction based on face orientation.
         * 5. Calculate angle between normals.
         */
        static double ComputeEdgeDihedralAngle(const TopoDS_Edge& edge, const TopoDS_Face& f1, const TopoDS_Face& f2);


        /**
         * @brief Analyze topological adjacency relationships
         * 分析拓扑邻接关系
         * @details Finds neighboring faces through shared edges and determines continuity type (C0, C1, C2)
         * 通过共享边查找相邻面，确定连续性类型（C0、C1、C2）
         * @param face 要分析的面 / Face to analyze
         * @param faceID 面的 ID / Face ID
         * @param parentSolid 父实体 / Parent solid
         * @return 邻接信息向量 / Vector of adjacency information
         */
        std::vector<AdjacencyInfo> AnalyzeTopologicalAdjacency(
            const TopoDS_Face& face, const std::string& faceID,
            const TopoDS_Shape& parentSolid);

        // Internal logic helpers (Step 3)
        // Using shared_ptr for map values for safety
        // 使用 shared_ptr 确保 Map 中的指针安全
        using FeatureMap = std::map<std::string, std::shared_ptr<AtomicFeature>>;

        /**
         * @brief Recognize hole feature (concave cylinder)
         * 识别孔特征（凹陷圆柱）
         * @details Rule: Cylinder + Concave form type. Supports merged fragmented cylinders.
         * 规则：圆柱 + 凹陷形态。支持合并的碎片圆柱。
         * @param partNode 零件节点（引用）/ Part node (by reference)
         * @param feature 候选原子特征 / Candidate atomic feature
         * @param featureMap 特征 ID 到特征的映射 / Map from feature ID to feature
         * @return 识别成功返回 true，否则返回 false / Returns true if recognized, false otherwise
         */
        static bool RecognizeHoleFeature(PartNode& partNode, const std::shared_ptr<AtomicFeature>& feature, FeatureMap& featureMap);

        /**
         * @brief Recognize shaft feature (convex cylinder)
         * 识别轴特征（凸出圆柱）
         * @details Rule: Cylinder + Convex form type. Supports merged fragmented cylinders.
         * 规则：圆柱 + 凸出形态。支持合并的碎片圆柱。
         * @param partNode 零件节点（引用）/ Part node (by reference)
         * @param feature 候选原子特征 / Candidate atomic feature
         * @param featureMap 特征 ID 到特征的映射 / Map from feature ID to feature
         * @return 识别成功返回 true，否则返回 false / Returns true if recognized, false otherwise
         */
        static bool RecognizeShaftFeature(PartNode& partNode, const std::shared_ptr<AtomicFeature>& feature, FeatureMap& featureMap);


        /**
         * @brief Recognize slot feature (concave channel)
         * 识别槽特征（凹陷通道）
         * @details Rule: Base plane (concave) + 2+ perpendicular wall planes (concave) with parallel opposite walls
         * 规则：基准平面（凹陷）+ 2个以上垂直侧壁平面（凹陷），具有平行对立侧壁
         * @param partNode 输入：待分析的零件节点引用 / Part node reference to analyze
         * @param baseFeature 输入：槽基准原子特征 / Base atomic feature candidate for slot
         * @param featureMap 输入：特征映射，便于通过 ID 获取原子特征 / Feature map keyed by ID
         * @return 输出：识别成功返回 true，否则返回 false / true if slot recognized, false otherwise
         */
        static bool RecognizeSlotFeature(PartNode& partNode, const std::shared_ptr<AtomicFeature>& baseFeature, FeatureMap& featureMap);

        /**
         * @brief Recognize tongue feature (convex protrusion)
         * 识别榫特征（凸出突起）
         * @details Rule: Base plane (convex/neutral) + 2+ perpendicular wall planes (convex/neutral) with parallel opposite walls
         * 规则：基准平面（凸出/中性）+ 至少两组互相平行的侧壁平面（凸出/中性）
         * @param partNode 输入：目标零件节点引用 / Part node under inspection
         * @param baseFeature 输入：榫的基面原子特征 / Base atomic feature candidate for tongue
         * @param featureMap 输入：特征映射表 / Map providing lookup by feature ID
         * @return 输出：识别成功返回 true，否则返回 false / true when a tongue feature is created
         */
        static bool RecognizeTongueFeature(PartNode& partNode, const std::shared_ptr<AtomicFeature>& baseFeature, FeatureMap& featureMap);



        /**
         * @brief Recognize step plane feature (plane adjacent to hole/shaft)
         * 识别台阶面特征（与孔/轴相邻的平面）
         * @details Rule: Plane + Neutral form + Adjacent to cylinder via C0 edge
         * 规则：平面 + 中性形态 + 通过 C0 边与圆柱相邻
         * @param partNode 输入：零件节点引用 / Part node reference
         * @param feature 输入：候选原子特征 / Candidate atomic feature
         * @param featureMap 输入：特征映射表 / Feature ID map
         * @return 输出：识别成功返回 true，否则返回 false / true on successful step-plane recognition
         */
        static bool RecognizeStepPlaneFeature(PartNode& partNode, const std::shared_ptr<AtomicFeature>& feature, FeatureMap& featureMap);

        /**
         * @brief Recognize functional plane feature (significant mating surface)
         * 识别功能平面特征（重要配合面）
         * @details Catch-all rule: Plane + Neutral form + Functional (area > threshold) + Not consumed
         * 兜底规则：平面 + 中性形态 + 功能性（面积大于阈值）+ 未被其他特征使用
         * @param partNode 输入：零件节点引用 / Part node reference
         * @param feature 输入：候选原子特征 / Candidate atomic feature
         * @param featureMap 输入：特征映射表 / Feature ID map
         * @return 输出：识别成功返回 true，否则返回 false / true when plane promoted to functional feature
         */
        static bool RecognizeFunctionalPlaneFeature(PartNode& partNode, const std::shared_ptr<AtomicFeature>& feature,
                                                    FeatureMap& featureMap);

        static double ComputeFaceArea(const TopoDS_Face& face);
        static gp_Pnt GetFaceSamplePoint(const TopoDS_Face& face);
        static double ComputeCylinderHeight(const TopoDS_Face& face, const gp_Pnt& axisPoint, const gp_Dir& axisVector);

        // --- Step 4 Helpers / 步骤四辅助函数 ---
        void MatchPartPair(const PartNode& nodeA, const PartNode& nodeB);
        void MatchCoaxial(const PartNode& nodeA, const CompositeFeature& featA,
                          const PartNode& nodeB, const CompositeFeature& featB);
        void MatchCoincident(const PartNode& nodeA, const CompositeFeature& featA,
                             const PartNode& nodeB, const CompositeFeature& featB);
        void MatchPrismatic(const PartNode& nodeA, const CompositeFeature& featA,
                            const PartNode& nodeB, const CompositeFeature& featB);
        static gp_Ax1 TransformAxis(const gp_Ax1& localAxis, const gp_Trsf& trsf);
        static gp_Pnt TransformPoint(const gp_Pnt& localPnt, const gp_Trsf& trsf);
        static gp_Dir TransformDir(const gp_Dir& localDir, const gp_Trsf& trsf);
        static bool CheckBoundingBoxCollision(const PartNode& nodeA, const PartNode& nodeB);

        // Member Data
        Handle(TDocStd_Document) doc_;
        Handle(XCAFDoc_ShapeTool) shapeTool_;

        std::vector<PartNode> partNodes_;

        // [Fix] Updated Comparator
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
                // Using IntegerLast() from Standard_Integer.hxx instead of INT_MAX
                const int h1 = static_cast<int>(f1.Location().HashCode());

                if (const int h2 = static_cast<int>(f2.Location().HashCode()); h1 != h2)
                {
                    return h1 < h2;
                }

                // 3. Compare Orientation
                return f1.Orientation() < f2.Orientation();
            }
        };

        std::map<TopoDS_Face, std::string, FaceComparator> faceIDMap_;

        /**
         * @brief Store discovered assembly constraints
         * 存储识别到的装配约束
         * @details Accumulates constraints detected during BuildAssemblyConstraintGraph
         * 汇总 BuildAssemblyConstraint 过程中识别到的约束
         * @input 由匹配函数写入 / Filled by matcher functions
         * @output 向其他模块提供约束查询 / Available for downstream consumers
         */
        std::vector<AssemblyConstraint> constraints_;
    };
} // namespace ASG
