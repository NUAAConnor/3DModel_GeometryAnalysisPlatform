//
// Created by zhuge on 2025/11/19.
//

#include "ASGBuilder.h"

// OpenCASCADE Standard Headers
// OpenCASCADE 标准头文件
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <GeomAbs_Shape.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>

// Standard Library
// 标准库
#include <iomanip>
#include <sstream>
#include <cmath>
#include <limits>
#include <fstream>

namespace ASG
{
    // ============================================================================
    // Internal Helper Namespace (Local to this file)
    // 内部辅助命名空间 (仅本文件可见)
    // ============================================================================
    namespace
    {
        namespace JSONUtils
        {
            std::string EscapeJSONString(const std::string& str)
            {
                std::ostringstream oss;
                for (const char c : str)
                {
                    switch (c)
                    {
                    case '"': oss << "\\\"";
                        break;
                    case '\\': oss << "\\\\";
                        break;
                    case '\b': oss << "\\b";
                        break;
                    case '\f': oss << "\\f";
                        break;
                    case '\n': oss << "\\n";
                        break;
                    case '\r': oss << "\\r";
                        break;
                    case '\t': oss << "\\t";
                        break;
                    default:
                        if ('\x00' <= c && c <= '\x1f')
                        {
                            oss << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c);
                        }
                        else
                        {
                            oss << c;
                        }
                    }
                }
                return oss.str();
            }

            std::string FormatDouble(const double value, const int precision = 6)
            {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(precision) << value;
                return oss.str();
            }
        }
    }

    // ============================================================================
    // Step 2: Geometry Identification & Analysis
    // 步骤 2: 几何识别与分析
    // ============================================================================

    std::pair<AtomType, GeometricParams> ASGBuilder::IdentifyGeometryType(const TopoDS_Face& face)
    {
        AtomType atomType = AtomType::OTHER;
        GeometricParams params;

        // Get underlying geometric surface
        // 获取底层几何曲面
        Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
        if (surface.IsNull())
        {
            return {atomType, params};
        }

        bool isReversed = (face.Orientation() == TopAbs_REVERSED);

        // 1. Plane / 平面
        if (auto plane = Handle(Geom_Plane)::DownCast(surface))
        {
            atomType = AtomType::PLANE;
            gp_Pln gp_plane = plane->Pln();
            gp_Dir naturalNormal = gp_plane.Axis().Direction();
            params.axisVector = isReversed ? naturalNormal.Reversed() : naturalNormal;
            params.locationPoint = gp_plane.Location();
        }
        // 2. Cylinder / 圆柱面
        else if (auto cylinder = Handle(Geom_CylindricalSurface)::DownCast(surface))
        {
            atomType = AtomType::CYLINDER;
            gp_Cylinder gp_cyl = cylinder->Cylinder();
            params.axisVector = gp_cyl.Axis().Direction();
            params.locationPoint = gp_cyl.Axis().Location();
            params.radius = gp_cyl.Radius();
        }
        // 3. Cone / 圆锥面
        else if (auto cone = Handle(Geom_ConicalSurface)::DownCast(surface))
        {
            atomType = AtomType::CONE;
            gp_Cone gp_cone = cone->Cone();
            params.axisVector = gp_cone.Axis().Direction();
            params.locationPoint = gp_cone.Axis().Location();
            params.radius = gp_cone.RefRadius();
            params.semiAngle = gp_cone.SemiAngle();
        }
        // 4. Sphere / 球面
        else if (auto sphere = Handle(Geom_SphericalSurface)::DownCast(surface))
        {
            atomType = AtomType::SPHERE;
            gp_Sphere gp_sphere = sphere->Sphere();
            params.locationPoint = gp_sphere.Location();
            params.radius = gp_sphere.Radius();
        }
        // 5. Torus / 环面
        else if (auto torus = Handle(Geom_ToroidalSurface)::DownCast(surface))
        {
            atomType = AtomType::TORUS;
            gp_Torus gp_torus = torus->Torus();
            params.axisVector = gp_torus.Axis().Direction();
            params.locationPoint = gp_torus.Axis().Location();
            params.majorRadius = gp_torus.MajorRadius();
            params.minorRadius = gp_torus.MinorRadius();
        }
        // 6. BSpline / B样条
        else if (Handle(Geom_BSplineSurface)::DownCast(surface))
        {
            atomType = AtomType::BSPLINE;
        }

        return {atomType, params};
    }

    FormType ASGBuilder::DetermineConcavity(
        const TopoDS_Face& face,
        const AtomType& atomType,
        const GeometricParams& geomParams,
        const TopoDS_Shape& parentSolid)
    {
        // Skip non-curved surfaces
        // 跳过非曲面
        if (atomType == AtomType::PLANE || atomType == AtomType::BSPLINE || atomType == AtomType::OTHER)
        {
            return FormType::NEUTRAL;
        }

        // Curvature Center Test
        // 曲率中心测试法
        if (atomType == AtomType::CYLINDER || atomType == AtomType::CONE || atomType == AtomType::SPHERE)
        {
            const gp_Pnt centerOfCurvature = geomParams.locationPoint;

            // Use SolidClassifier to check if the center is inside the solid
            // 使用 SolidClassifier 检查曲率中心是否在实体内部
            const BRepClass3d_SolidClassifier classifier(parentSolid, centerOfCurvature, 1e-7);
            const TopAbs_State state = classifier.State();

            if (state == TopAbs_IN)
            {
                // Center inside solid -> Surface curves away -> Convex (e.g., Shaft)
                // 中心在实体内 -> 表面向外凸 -> 凸 (例如轴)
                return FormType::CONVEX;
            }
            else if (state == TopAbs_OUT)
            {
                // Center outside solid -> Surface curves inward -> Concave (e.g., Hole)
                // 中心在实体外 -> 表面向内凹 -> 凹 (例如孔)
                return FormType::CONCAVE;
            }
        }

        return FormType::NEUTRAL;
    }

    std::vector<AdjacencyInfo> ASGBuilder::AnalyzeTopologicalAdjacency(
        const TopoDS_Face& face,
        const std::string& faceID,
        const TopoDS_Shape& parentSolid)
    {
        std::vector<AdjacencyInfo> adjacencyList;

        // Map edges to faces to find neighbors
        // 映射边到面以查找邻居
        TopTools_IndexedDataMapOfShapeListOfShape edgeFaceMap;
        TopExp::MapShapesAndAncestors(parentSolid, TopAbs_EDGE, TopAbs_FACE, edgeFaceMap);

        TopExp_Explorer edgeExplorer(face, TopAbs_EDGE);
        for (; edgeExplorer.More(); edgeExplorer.Next())
        {
            const TopoDS_Edge& edge = TopoDS::Edge(edgeExplorer.Current());

            if (!edgeFaceMap.Contains(edge)) continue;

            const TopTools_ListOfShape& facesOnEdge = edgeFaceMap.FindFromKey(edge);

            for (TopTools_ListIteratorOfListOfShape faceIter(facesOnEdge); faceIter.More(); faceIter.Next())
            {
                TopoDS_Face neighborFace = TopoDS::Face(faceIter.Value());

                if (neighborFace.IsSame(face)) continue;

                AdjacencyInfo adjInfo;

                // Find Neighbor ID
                // 查找邻居 ID
                auto it = faceIDMap_.find(neighborFace);
                if (it != faceIDMap_.end())
                {
                    adjInfo.neighborFaceID = it->second;
                }
                else
                {
                    adjInfo.neighborFaceID = "Unknown";
                }

                // Determine Continuity
                // 确定连续性
                switch (GeomAbs_Shape continuity = BRep_Tool::Continuity(edge, face, neighborFace))
                {
                case GeomAbs_C0: adjInfo.continuityType = ContinuityType::C0;
                    break;
                case GeomAbs_G1:
                case GeomAbs_C1: adjInfo.continuityType = ContinuityType::C1;
                    break;
                case GeomAbs_G2:
                case GeomAbs_C2:
                case GeomAbs_CN: adjInfo.continuityType = ContinuityType::C2;
                    break;
                default: adjInfo.continuityType = ContinuityType::UNKNOWN;
                    break;
                }

                adjacencyList.push_back(adjInfo);
            }
        }

        return adjacencyList;
    }

    // ============================================================================
    // Step 3: Composite Feature Recognition
    // 步骤 3: 复合特征识别
    // ============================================================================

    bool ASGBuilder::RecognizeHoleFeature(
        PartNode& partNode,
        const std::shared_ptr<AtomicFeature>& feature,
        FeatureMap& featureMap)
    {
        // 基础检查
        if (feature->atomType != AtomType::CYLINDER || feature->formType != FormType::CONCAVE)
        {
            return false;
        }

        CompositeFeature holeFeature;
        holeFeature.partID = partNode.partID;
        holeFeature.type = CompositeFeatureType::HOLE;
        holeFeature.compositeID = partNode.partID + "_Hole_" + std::to_string(partNode.compositeFeatures.size());

        holeFeature.nominalRadius = feature->geometricParams.radius;
        holeFeature.height = feature->geometricParams.height;
        holeFeature.axis = gp_Ax1(feature->geometricParams.locationPoint, feature->geometricParams.axisVector);

        if (feature->isMainFragment)
        {
            holeFeature.childAtomicFeatureIDs = feature->fragmentFaceIDs;
        }
        else
        {
            holeFeature.childAtomicFeatureIDs.push_back(feature->faceID);
        }

        // --- 盲孔/通孔 检测 (简化版) ---
        bool hasBottom = false;
        const double holeArea = M_PI * std::pow(holeFeature.nominalRadius, 2);

        for (const auto& [neighborFaceID, continuityType] : feature->adjacencyList)
        {
            auto it = featureMap.find(neighborFaceID);
            if (it == featureMap.end()) continue;
            const auto neighbor = it->second;
            if (neighbor->isConsumed) continue;

            // 检查平底 (C0连接 + 面积近似)
            if (neighbor->atomType == AtomType::PLANE && continuityType == ContinuityType::C0)
            {
                if (std::abs(neighbor->area - holeArea) < 0.2 * holeArea)
                {
                    // 20% 容差
                    hasBottom = true;
                    holeFeature.childAtomicFeatureIDs.push_back(neighbor->faceID);
                    neighbor->isConsumed = true;
                }
            }
            // 检查锥底 (C0/C1连接 + 锥面)
            else if (neighbor->atomType == AtomType::CONE)
            {
                hasBottom = true; // 只要有个锥底，就是盲孔，不用管它是钻尖还是什么
                holeFeature.childAtomicFeatureIDs.push_back(neighbor->faceID);
                neighbor->isConsumed = true;
            }
        }

        // 设置类型
        holeFeature.holeSubType = hasBottom ? HoleType::BLIND : HoleType::THROUGH;

        // 标记孔壁为已消耗
        for (const auto& childID : holeFeature.childAtomicFeatureIDs)
        {
            if (auto it = featureMap.find(childID); it != featureMap.end()) it->second->isConsumed = true;
        }

        partNode.compositeFeatures.push_back(holeFeature);
        return true;
    }

    bool ASGBuilder::RecognizeShaftFeature(
        PartNode& partNode,
        const std::shared_ptr<AtomicFeature>& feature,
        FeatureMap& featureMap)
    {
        if (feature->atomType != AtomType::CYLINDER || feature->formType != FormType::CONVEX)
        {
            return false;
        }

        CompositeFeature shaftFeature;
        shaftFeature.partID = partNode.partID;
        shaftFeature.type = CompositeFeatureType::SHAFT;
        shaftFeature.compositeID = partNode.partID + "_Shaft_" + std::to_string(partNode.compositeFeatures.size());

        shaftFeature.nominalRadius = feature->geometricParams.radius;
        shaftFeature.height = feature->geometricParams.height;
        shaftFeature.axis = gp_Ax1(feature->geometricParams.locationPoint, feature->geometricParams.axisVector);

        if (feature->isMainFragment)
        {
            shaftFeature.childAtomicFeatureIDs = feature->fragmentFaceIDs;
        }
        else
        {
            shaftFeature.childAtomicFeatureIDs.push_back(feature->faceID);
        }

        for (const auto& childID : shaftFeature.childAtomicFeatureIDs)
        {
            if (auto it = featureMap.find(childID); it != featureMap.end())
            {
                it->second->isConsumed = true;
            }
        }

        partNode.compositeFeatures.push_back(shaftFeature);
        return true;
    }

    bool ASGBuilder::RecognizeStepPlaneFeature(
        PartNode& partNode,
        const std::shared_ptr<AtomicFeature>& feature,
        FeatureMap& featureMap)
    {
        // Rule: Plane + Neutral + Adjacent to Cylinder (via C0)
        // 规则: 平面 + 中性 + 邻接圆柱 (通过 C0 边)
        if (feature->atomType != AtomType::PLANE || feature->formType != FormType::NEUTRAL)
        {
            return false;
        }

        bool isAdjacentToCylinder = false;
        for (const auto& [neighborFaceID, continuityType] : feature->adjacencyList)
        {
            if (continuityType == ContinuityType::C0)
            {
                auto it = featureMap.find(neighborFaceID);
                if (it == featureMap.end()) continue;

                if (it->second->atomType == AtomType::CYLINDER)
                {
                    isAdjacentToCylinder = true;
                    break;
                }
            }
        }

        if (!isAdjacentToCylinder) return false;

        CompositeFeature stepFeature;
        stepFeature.partID = partNode.partID;
        stepFeature.type = CompositeFeatureType::STEP_PLANE;
        stepFeature.compositeID = partNode.partID + "_StepPlane_" + std::to_string(partNode.compositeFeatures.size());

        stepFeature.planeNormal = feature->geometricParams.axisVector; // Plane uses axisVector as normal
        stepFeature.planeLocation = feature->geometricParams.locationPoint;
        stepFeature.planeArea = feature->area;
        stepFeature.childAtomicFeatureIDs.push_back(feature->faceID);

        feature->isConsumed = true;
        partNode.compositeFeatures.push_back(stepFeature);
        return true;
    }

    bool ASGBuilder::RecognizeFunctionalPlaneFeature(
        PartNode& partNode,
        const std::shared_ptr<AtomicFeature>& feature,
        FeatureMap& featureMap)
    {
        // Catch-all for remaining significant planes
        // 兜底规则：收集剩余的重要平面
        if (feature->atomType != AtomType::PLANE || feature->formType != FormType::NEUTRAL)
        {
            return false;
        }
        if (!feature->isFunctional || feature->isConsumed)
        {
            return false;
        }

        CompositeFeature planeFeature;
        planeFeature.partID = partNode.partID;
        planeFeature.type = CompositeFeatureType::FUNCTIONAL_PLANE;
        planeFeature.compositeID = partNode.partID + "_FuncPlane_" + std::to_string(partNode.compositeFeatures.size());

        planeFeature.planeNormal = feature->geometricParams.axisVector;
        planeFeature.planeLocation = feature->geometricParams.locationPoint;
        planeFeature.planeArea = feature->area;
        planeFeature.childAtomicFeatureIDs.push_back(feature->faceID);

        feature->isConsumed = true;
        partNode.compositeFeatures.push_back(planeFeature);
        return true;
    }

    bool ASGBuilder::IsMaterialBetween(const gp_Pnt& p1, const gp_Pnt& p2, const TopoDS_Shape& solid)
    {
        const gp_Pnt midPoint((p1.X() + p2.X()) / 2.0,
                              (p1.Y() + p2.Y()) / 2.0,
                              (p1.Z() + p2.Z()) / 2.0);

        // 使用分类器检查中点状态 (使用较小的容差)
        const BRepClass3d_SolidClassifier classifier(solid, midPoint, 1e-7);

        // 如果中点在实体内部 (TopAbs_IN)，说明两壁之间是材料
        return (classifier.State() == TopAbs_IN);
    }

    bool ASGBuilder::RecognizeSlotFeature(
        PartNode& partNode,
        const std::shared_ptr<AtomicFeature>& baseFeature,
        FeatureMap& featureMap)
    {
        // 1. 基座检查：只要求是平面 (不再依赖 FormType)
        if (baseFeature->atomType != AtomType::PLANE) return false;

        std::vector<std::shared_ptr<AtomicFeature>> candidateWalls;
        const gp_Dir& baseNormal = baseFeature->geometricParams.axisVector;

        // 2. 寻找侧壁
        for (const auto& [neighborFaceID, continuityType] : baseFeature->adjacencyList)
        {
            if (continuityType != ContinuityType::C0) continue; // 必须是尖锐连接

            auto it = featureMap.find(neighborFaceID);
            if (it == featureMap.end()) continue;
            auto neighbor = it->second;

            if (neighbor->isConsumed) continue;
            if (neighbor->atomType != AtomType::PLANE) continue;

            // 2a. 几何检查：侧壁必须垂直于基座
            if (!neighbor->geometricParams.axisVector.IsNormal(baseNormal, Constants::AngleTolerance)) continue;

            // 2b. [关键] 基座连接性检查
            // 对于槽(Slot)，基座是"底面"，它与侧壁的夹角内应该是"空气" (凹连接)
            gp_Pnt pBase = GetFaceSamplePoint(baseFeature->brepFace);
            gp_Pnt pWall = GetFaceSamplePoint(neighbor->brepFace);
            if (IsMaterialBetween(pBase, pWall, partNode.brepShape))
            {
                continue; // 如果连接处是实心的，说明这是凸边(如榫的顶面)，不是槽底
            }

            candidateWalls.push_back(neighbor);
        }

        if (candidateWalls.size() < 2) return false;

        // 3. 验证侧壁对
        for (size_t i = 0; i < candidateWalls.size(); ++i)
        {
            for (size_t j = i + 1; j < candidateWalls.size(); ++j)
            {
                const auto& wallA = candidateWalls[i];
                const auto& wallB = candidateWalls[j];

                // 3a. 法线反向平行检查
                // 槽的两个对立侧壁，法线指向槽内空间，应该是相反的
                if (!wallA->geometricParams.axisVector.IsOpposite(wallB->geometricParams.axisVector,
                                                                  Constants::AngleTolerance))
                {
                    continue;
                }

                // 3b. [核心] 内部材质检测
                // 获取两个侧壁上的采样点
                gp_Pnt p1 = GetFaceSamplePoint(wallA->brepFace);
                gp_Pnt p2 = GetFaceSamplePoint(wallB->brepFace);

                // 如果两壁连线中点 *没有* 材料 (IsMaterial == false)，说明中间是空的 -> 这是一个槽
                if (!IsMaterialBetween(p1, p2, partNode.brepShape))
                {
                    // === 识别成功，创建槽特征 ===
                    CompositeFeature slotFeature;
                    slotFeature.partID = partNode.partID;
                    slotFeature.type = CompositeFeatureType::SLOT;
                    slotFeature.compositeID = partNode.partID + "_Slot_" + std::to_string(partNode.compositeFeatures.size());

                    // 填充几何信息
                    slotFeature.planeNormal = baseNormal;

                    // [计算真实几何中心]
                    gp_Pnt centerPnt = p1;
                    centerPnt.Translate(gp_Vec(p1, p2) * 0.5);
                    slotFeature.planeLocation = centerPnt; // 记录中心点

                    // 轴线方向：基座法线 x 侧壁法线 (即槽的延伸方向)
                    slotFeature.axis = gp_Ax1(centerPnt, baseNormal.Crossed(wallA->geometricParams.axisVector));

                    // 计算宽度
                    gp_Pln planeB(wallB->geometricParams.locationPoint, wallB->geometricParams.axisVector);
                    slotFeature.width = planeB.Distance(p1);

                    // 链接子特征
                    slotFeature.childAtomicFeatureIDs = {baseFeature->faceID, wallA->faceID, wallB->faceID};

                    // 标记消耗
                    baseFeature->isConsumed = true;
                    wallA->isConsumed = true;
                    wallB->isConsumed = true;

                    partNode.compositeFeatures.push_back(slotFeature);
                    return true;
                }
            }
        }
        return false;
    }

    bool ASGBuilder::RecognizeTongueFeature(
        PartNode& partNode,
        const std::shared_ptr<AtomicFeature>& baseFeature,
        FeatureMap& featureMap)
    {
        // 1. 基座检查：只要求是平面
        if (baseFeature->atomType != AtomType::PLANE) return false;

        std::vector<std::shared_ptr<AtomicFeature>> candidateWalls;
        const gp_Dir& baseNormal = baseFeature->geometricParams.axisVector;

        // 2. 寻找侧壁
        for (const auto& [neighborFaceID, continuityType] : baseFeature->adjacencyList)
        {
            if (continuityType != ContinuityType::C0) continue;

            auto it = featureMap.find(neighborFaceID);
            if (it == featureMap.end()) continue;
            auto neighbor = it->second;

            if (neighbor->isConsumed) continue;
            if (neighbor->atomType != AtomType::PLANE) continue;

            // 2a. 几何检查：垂直
            if (!neighbor->geometricParams.axisVector.IsNormal(baseNormal, Constants::AngleTolerance)) continue;

            // 2b. [关键] 基座连接性检查
            // 对于榫(Tongue)，基座是"顶面"，它与侧壁的夹角内应该是"实体" (凸连接)
            gp_Pnt pBase = GetFaceSamplePoint(baseFeature->brepFace);
            gp_Pnt pWall = GetFaceSamplePoint(neighbor->brepFace);

            // 如果连接处 *不是* 实心的 (是空气)，跳过
            if (!IsMaterialBetween(pBase, pWall, partNode.brepShape))
            {
                continue;
            }

            candidateWalls.push_back(neighbor);
        }

        if (candidateWalls.size() < 2) return false;

        // 3. 验证侧壁对
        for (size_t i = 0; i < candidateWalls.size(); ++i)
        {
            for (size_t j = i + 1; j < candidateWalls.size(); ++j)
            {
                const auto& wallA = candidateWalls[i];
                const auto& wallB = candidateWalls[j];

                // 3a. 法线反向平行
                if (!wallA->geometricParams.axisVector.IsOpposite(wallB->geometricParams.axisVector,
                                                                  Constants::AngleTolerance))
                {
                    continue;
                }

                // 3b. [核心] 内部材质检测
                gp_Pnt p1 = GetFaceSamplePoint(wallA->brepFace);
                gp_Pnt p2 = GetFaceSamplePoint(wallB->brepFace);

                // 如果两壁连线中点 *有* 材料 (IsMaterial == true)，说明中间是实心的 -> 这是一个榫
                if (IsMaterialBetween(p1, p2, partNode.brepShape))
                {
                    // [增强] 根部连接检测 (Root Check)
                    // 防止将整个零件基座识别为榫。真正的榫，其侧壁根部应连接到地板(凹连接)。
                    // 只要检测到侧壁有任何一个"凹"的邻居，就认为它是特征，而不是基体。
                    bool hasConcaveRoot = false;

                    // 检查 WallA 的所有邻居
                    for(const auto& [adjID, continuity] : wallA->adjacencyList) {
                        if(adjID == baseFeature->faceID) continue; // 忽略顶面
                        auto it = featureMap.find(adjID);
                        if(it != featureMap.end()) {
                            gp_Pnt pNeighbor = GetFaceSamplePoint(it->second->brepFace);
                            // 如果侧壁和某个邻居之间是空气(凹连接)，说明这是根部
                            if(!IsMaterialBetween(p1, pNeighbor, partNode.brepShape)) {
                                hasConcaveRoot = true;
                                break;
                            }
                        }
                    }
                    // 如果 WallA 没找到，查 WallB
                    if(!hasConcaveRoot) {
                        for(const auto& [adjID, continuity] : wallB->adjacencyList) {
                            if(adjID == baseFeature->faceID) continue;
                            auto it = featureMap.find(adjID);
                            if(it != featureMap.end()) {
                                gp_Pnt pNeighbor = GetFaceSamplePoint(it->second->brepFace);
                                if(!IsMaterialBetween(p2, pNeighbor, partNode.brepShape)) {
                                    hasConcaveRoot = true;
                                    break;
                                }
                            }
                        }
                    }

                    // 如果没有凹根部，说明这是一个孤立的凸多面体(如平板基座)，跳过，留给FunctionalPlane处理
                    if (!hasConcaveRoot) continue;

                    // === 识别成功，创建榫特征 ===
                    CompositeFeature tongueFeature;
                    tongueFeature.partID = partNode.partID;
                    tongueFeature.type = CompositeFeatureType::TONGUE;
                    tongueFeature.compositeID = partNode.partID + "_Tongue_" + std::to_string(partNode.compositeFeatures.size());

                    tongueFeature.planeNormal = baseNormal;

                    // [计算真实几何中心]
                    gp_Pnt centerPnt = p1;
                    centerPnt.Translate(gp_Vec(p1, p2) * 0.5);
                    tongueFeature.planeLocation = centerPnt;

                    tongueFeature.axis = gp_Ax1(centerPnt, baseNormal.Crossed(wallA->geometricParams.axisVector));

                    gp_Pln planeB(wallB->geometricParams.locationPoint, wallB->geometricParams.axisVector);
                    tongueFeature.width = planeB.Distance(p1);

                    tongueFeature.childAtomicFeatureIDs = {baseFeature->faceID, wallA->faceID, wallB->faceID};

                    baseFeature->isConsumed = true;
                    wallA->isConsumed = true;
                    wallB->isConsumed = true;

                    partNode.compositeFeatures.push_back(tongueFeature);
                    return true;
                }
            }
        }
        return false;
    }


    // ============================================================================
    // Utilities / 工具函数
    // ============================================================================

    double ASGBuilder::ComputeFaceArea(const TopoDS_Face& face)
    {
        GProp_GProps properties;
        BRepGProp::SurfaceProperties(face, properties);
        return properties.Mass();
    }

    gp_Pnt ASGBuilder::GetFaceSamplePoint(const TopoDS_Face& face)
    {
        Standard_Real uMin, uMax, vMin, vMax;
        BRepTools::UVBounds(face, uMin, uMax, vMin, vMax);

        const double uMid = (uMin + uMax) / 2.0;
        const double vMid = (vMin + vMax) / 2.0;

        const BRepAdaptor_Surface surfAdaptor(face);
        return surfAdaptor.Value(uMid, vMid);
    }

    double ASGBuilder::ComputeCylinderHeight(const TopoDS_Face& face, const gp_Pnt& axisPoint, const gp_Dir& axisVector)
    {
        double minParam = std::numeric_limits<double>::max();
        double maxParam = std::numeric_limits<double>::lowest();

        for (TopExp_Explorer edgeExplorer(face, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next())
        {
            const TopoDS_Edge& edge = TopoDS::Edge(edgeExplorer.Current());
            for (TopExp_Explorer vExp(edge, TopAbs_VERTEX); vExp.More(); vExp.Next())
            {
                gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(vExp.Current()));
                double proj = gp_Vec(axisPoint, p).Dot(gp_Vec(axisVector));
                minParam = std::min(minParam, proj);
                maxParam = std::max(maxParam, proj);
            }
        }

        double h = maxParam - minParam;
        if (h <= 0 || h > 1e6)
        {
            // Fallback to parameter space
            BRepAdaptor_Surface surf(face);
            h = std::abs(surf.LastVParameter() - surf.FirstVParameter());
        }
        return h;
    }

    bool ASGBuilder::ExportToJSON(const std::string& filePath) const
    {
        std::ofstream outFile(filePath);
        if (!outFile.is_open()) return false;

        outFile << "{\n";
        outFile << "  \"parts\": [\n";

        for (size_t i = 0; i < partNodes_.size(); ++i)
        {
            const auto& part = partNodes_[i];
            outFile << "    {\n";
            outFile << R"(      "partID": ")" << JSONUtils::EscapeJSONString(part.partID) << "\",\n";
            outFile << "      \"features\": [\n";

            for (size_t j = 0; j < part.compositeFeatures.size(); ++j)
            {
                const auto& feat = part.compositeFeatures[j];
                outFile << "        {\n";
                outFile << R"(          "id": ")" << JSONUtils::EscapeJSONString(feat.compositeID) << "\",\n";

                //  输出 type，注意这里暂时不换行，以便后续追加 subType
                outFile << R"(          "type": ")" << static_cast<int>(feat.type) << "\"";

                // 如果是孔特征，额外输出子类型 (subType)
                if (feat.type == CompositeFeatureType::HOLE)
                {
                    outFile << ",\n"; // 加逗号并换行
                    std::string subTypeStr;
                    switch (feat.holeSubType)
                    {
                    case HoleType::THROUGH: subTypeStr = "Through";
                        break;
                    case HoleType::BLIND: subTypeStr = "Blind";
                        break;
                    default: subTypeStr = "Unknown";
                        break;
                    }
                    outFile << R"(          "subType": ")" << subTypeStr << "\"";
                }

                outFile << "\n"; // 结束属性行的换行

                // 结束当前特征对象 (如果是最后一个特征，则不加逗号)
                outFile << "        }" << (j == part.compositeFeatures.size() - 1 ? "" : ",") << "\n";
            }

            outFile << "      ]\n";
            // 结束当前零件对象 (如果是最后一个零件，则不加逗号)
            outFile << "    }" << (i == partNodes_.size() - 1 ? "" : ",") << "\n";
        }

        outFile << "  ]\n";
        outFile << "}\n";
        outFile.close();
        return true;
    }

    void ASGBuilder::PrintStatistics() const
    {
        int totalFeatures = 0;
        for (const auto& p : partNodes_) totalFeatures += static_cast<int>(p.compositeFeatures.size());
        std::cout << "Total Parts: " << partNodes_.size() << ", Total Composite Features: " << totalFeatures <<
            std::endl;
    }

    // ============================================================================
    // Step 4 Helpers: Coordinate Transformations & Bounding Boxes
    // ============================================================================

    gp_Pnt ASGBuilder::TransformPoint(const gp_Pnt& localPnt, const gp_Trsf& trsf)
    {
        return localPnt.Transformed(trsf);
    }

    gp_Dir ASGBuilder::TransformDir(const gp_Dir& localDir, const gp_Trsf& trsf)
    {
        return localDir.Transformed(trsf);
    }

    gp_Ax1 ASGBuilder::TransformAxis(const gp_Ax1& localAxis, const gp_Trsf& trsf)
    {
        return localAxis.Transformed(trsf);
    }

    bool ASGBuilder::CheckBoundingBoxCollision(const PartNode& nodeA, const PartNode& nodeB)
    {
        Bnd_Box boxA;
        Bnd_Box boxB;

        const TopoDS_Shape shapeA = nodeA.brepShape.Moved(TopLoc_Location(nodeA.transformation));
        const TopoDS_Shape shapeB = nodeB.brepShape.Moved(TopLoc_Location(nodeB.transformation));

        BRepBndLib::Add(shapeA, boxA);
        BRepBndLib::Add(shapeB, boxB);

        boxA.Enlarge(0.1);
        boxB.Enlarge(0.1);

        return !boxA.IsOut(boxB);
    }

    void ASGBuilder::MatchPartPair(const PartNode& nodeA, const PartNode& nodeB)
    {
        for (const auto& featA : nodeA.compositeFeatures)
        {
            for (const auto& featB : nodeB.compositeFeatures)
            {
                if ((featA.type == CompositeFeatureType::HOLE && featB.type == CompositeFeatureType::SHAFT) ||
                    (featA.type == CompositeFeatureType::SHAFT && featB.type == CompositeFeatureType::HOLE))
                {
                    MatchCoaxial(nodeA, featA, nodeB, featB);
                }

                const bool isPlanarA = (featA.type == CompositeFeatureType::FUNCTIONAL_PLANE ||
                    featA.type == CompositeFeatureType::STEP_PLANE ||
                    featA.type == CompositeFeatureType::TONGUE ||
                    featA.type == CompositeFeatureType::SLOT);
                const bool isPlanarB = (featB.type == CompositeFeatureType::FUNCTIONAL_PLANE ||
                    featB.type == CompositeFeatureType::STEP_PLANE ||
                    featB.type == CompositeFeatureType::TONGUE ||
                    featB.type == CompositeFeatureType::SLOT);

                if (isPlanarA && isPlanarB)
                {
                    MatchCoincident(nodeA, featA, nodeB, featB);
                }

                if ((featA.type == CompositeFeatureType::SLOT && featB.type == CompositeFeatureType::TONGUE) ||
                    (featA.type == CompositeFeatureType::TONGUE && featB.type == CompositeFeatureType::SLOT))
                {
                    MatchPrismatic(nodeA, featA, nodeB, featB);
                }
            }
        }
    }

    void ASGBuilder::MatchCoaxial(const PartNode& nodeA, const CompositeFeature& featA,
                                  const PartNode& nodeB, const CompositeFeature& featB)
    {
        const gp_Ax1 axisA = TransformAxis(featA.axis, nodeA.transformation);
        const gp_Ax1 axisB = TransformAxis(featB.axis, nodeB.transformation);

        if (!axisA.Direction().IsParallel(axisB.Direction(), Constants::AngleTolerance))
        {
            return;
        }

        if (const gp_Lin lineA(axisA); lineA.Distance(axisB.Location()) > Constants::DistanceTolerance)
        {
            return;
        }

        if (std::abs(featA.nominalRadius - featB.nominalRadius) > 1.0)
        {
            return;
        }

        AssemblyConstraint constraint;
        constraint.type = ConstraintType::COAXIAL;
        constraint.partID_A = nodeA.partID;
        constraint.featureID_A = featA.compositeID;
        constraint.partID_B = nodeB.partID;
        constraint.featureID_B = featB.compositeID;
        constraints_.push_back(constraint);
    }

    void ASGBuilder::MatchCoincident(const PartNode& nodeA, const CompositeFeature& featA,
                                     const PartNode& nodeB, const CompositeFeature& featB)
    {
        // [鲁棒升级] 子特征遍历匹配
        // 不再只比较复合特征的摘要，而是深入比较每一个原子平面

        auto getAtomicFeature = [&](const PartNode& node, const std::string& faceID) -> std::shared_ptr<AtomicFeature>
        {
            for (const auto& af : node.atomicFeatures) {
                if (af->faceID == faceID) return af;
            }
            return nullptr;
        };

        // 双重循环遍历所有子特征
        for (const std::string& idA : featA.childAtomicFeatureIDs)
        {
            auto atomA = getAtomicFeature(nodeA, idA);
            if (!atomA || atomA->atomType != AtomType::PLANE) continue;

            // 获取原子特征的世界坐标几何
            gp_Dir localNormA = atomA->geometricParams.axisVector;
            gp_Pnt localLocA = atomA->geometricParams.locationPoint;
            gp_Dir globalNormA = TransformDir(localNormA, nodeA.transformation);
            gp_Pnt globalLocA = TransformPoint(localLocA, nodeA.transformation);

            for (const std::string& idB : featB.childAtomicFeatureIDs)
            {
                auto atomB = getAtomicFeature(nodeB, idB);
                if (!atomB || atomB->atomType != AtomType::PLANE) continue;

                gp_Dir localNormB = atomB->geometricParams.axisVector;
                gp_Pnt localLocB = atomB->geometricParams.locationPoint;
                gp_Dir globalNormB = TransformDir(localNormB, nodeB.transformation);
                gp_Pnt globalLocB = TransformPoint(localLocB, nodeB.transformation);

                // 1. 法线反向平行
                if (!globalNormA.IsOpposite(globalNormB, Constants::AngleTolerance))
                {
                    continue;
                }

                // 2. 共面距离
                gp_Pln planeB(globalLocB, globalNormB);
                if (double dist = planeB.Distance(globalLocA); dist < Constants::DistanceTolerance)
                {
                    // 3. 重叠检测
                    Bnd_Box boxA, boxB;
                    BRepBndLib::Add(atomA->brepFace.Moved(TopLoc_Location(nodeA.transformation)), boxA);
                    BRepBndLib::Add(atomB->brepFace.Moved(TopLoc_Location(nodeB.transformation)), boxB);
                    boxA.Enlarge(0.1); boxB.Enlarge(0.1);

                    if (!boxA.IsOut(boxB))
                    {
                        // 匹配成功
                        AssemblyConstraint constraint;
                        constraint.type = ConstraintType::COINCIDENT;
                        constraint.partID_A = nodeA.partID;
                        constraint.featureID_A = featA.compositeID;
                        constraint.partID_B = nodeB.partID;
                        constraint.featureID_B = featB.compositeID;

                        // 去重
                        bool exists = false;
                        for (const auto& c : constraints_) {
                            if (c.type == ConstraintType::COINCIDENT &&
                                c.featureID_A == constraint.featureID_A &&
                                c.featureID_B == constraint.featureID_B) {
                                exists = true; break;
                            }
                        }

                        if (!exists) {
                            constraints_.push_back(constraint);
                            goto NextPair; // 找到一个接触即可，跳出
                        }
                    }
                }
            }
        }
    NextPair:;
    }

    void ASGBuilder::MatchPrismatic(const PartNode& nodeA, const CompositeFeature& featA,
                                    const PartNode& nodeB, const CompositeFeature& featB)
    {
        const CompositeFeature* slot = (featA.type == CompositeFeatureType::SLOT) ? &featA : &featB;
        const CompositeFeature* tongue = (featA.type == CompositeFeatureType::SLOT) ? &featB : &featA;
        const PartNode* nodeSlot = (featA.type == CompositeFeatureType::SLOT) ? &nodeA : &nodeB;
        const PartNode* nodeTongue = (featA.type == CompositeFeatureType::SLOT) ? &nodeB : &nodeA;

        gp_Ax1 axisSlot = TransformAxis(slot->axis, nodeSlot->transformation);
        gp_Ax1 axisTongue = TransformAxis(tongue->axis, nodeTongue->transformation);

        // 1. 轴线平行检查
        if (!axisSlot.Direction().IsParallel(axisTongue.Direction(), Constants::AngleTolerance))
        {
            return;
        }

        // 2. [修复] 宽度匹配检查 (放宽容差)
        // 您的模型间隙总和为 1.0mm，浮点误差可能导致 > 1.0，这里放宽到 2.0mm 以确保安全
        if (std::abs(slot->width - tongue->width) > 2.0)
        {
            return;
        }

        gp_Dir normSlot = TransformDir(slot->planeNormal, nodeSlot->transformation);
        gp_Dir normTongue = TransformDir(tongue->planeNormal, nodeTongue->transformation);

        // 3. [修复] 法线检查：必须是平行 (IsParallel)
        // 槽底法线(朝外)和榫顶法线(朝外)在装配时方向是一致的
        if (!normSlot.IsParallel(normTongue, Constants::AngleTolerance))
        {
            return;
        }

        // 4. 中心对齐检查
        gp_Dir widthDirSlot = normSlot.Crossed(axisSlot.Direction());
        gp_Pln centerPlaneSlot(axisSlot.Location(), widthDirSlot); // 过槽中心，法线垂直侧壁

        // 检查榫的中心是否在槽的中心面上 (允许 1mm 偏心)
        if (centerPlaneSlot.Distance(axisTongue.Location()) > 1.0) {
            return;
        }

        AssemblyConstraint constraint;
        constraint.type = ConstraintType::PRISMATIC;
        constraint.partID_A = nodeA.partID;
        constraint.featureID_A = featA.compositeID;
        constraint.partID_B = nodeB.partID;
        constraint.featureID_B = featB.compositeID;
        constraints_.push_back(constraint);
    }
} // namespace ASG
