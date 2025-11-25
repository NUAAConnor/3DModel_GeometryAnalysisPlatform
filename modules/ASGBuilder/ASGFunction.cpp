//
// Created by zhuge on 2025/11/19.
//

#include "ASGBuilder.h"

// OpenCASCADE Standard Headers
// OpenCASCADE 标准头文件
#include <BRep_Tool.hxx>
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
        // 1. 基座检查：只要求是平面
        if (baseFeature->atomType != AtomType::PLANE) return false;

        std::vector<std::shared_ptr<AtomicFeature>> candidateWalls;
        const gp_Dir& baseNormal = baseFeature->geometricParams.axisVector; // 已修正为指向外部的法线

        // 2. 寻找侧壁
        for (const auto& [neighborFaceID, continuityType] : baseFeature->adjacencyList)
        {
            if (continuityType != ContinuityType::C0) continue;

            auto it = featureMap.find(neighborFaceID);
            if (it == featureMap.end()) continue;
            auto neighbor = it->second;

            if (neighbor->isConsumed) continue;
            if (neighbor->atomType != AtomType::PLANE) continue;

            // 几何检查：侧壁必须垂直于基座
            if (!neighbor->geometricParams.axisVector.IsNormal(baseNormal, Constants::AngleTolerance)) continue;

            // ====================================================================
            // 鲁棒性检查：基座有效性验证 (Edge Convexity Check)
            // 原理：基座与侧壁的连接必须是"凹"的 (中间是空气)。
            // 如果中间是实体，说明这是"凸"边，当前面是"顶面"而非"基座"，必须剔除。
            // ====================================================================
            gp_Pnt pBase = GetFaceSamplePoint(baseFeature->brepFace);
            gp_Pnt pWall = GetFaceSamplePoint(neighbor->brepFace);

            if (IsMaterialBetween(pBase, pWall, partNode.brepShape))
            {
                continue; // 连接处是实心的 -> 这是顶面，不是基座 -> 跳过
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

                // [检查 A] 法线反向平行
                // 既然法线都指向外部，槽的两个对立侧壁法线应该是相反的
                if (!wallA->geometricParams.axisVector.IsOpposite(wallB->geometricParams.axisVector,
                                                                  Constants::AngleTolerance))
                {
                    continue;
                }

                // [检查 B] 材质检测
                // 获取两个侧壁上的采样点
                gp_Pnt p1 = GetFaceSamplePoint(wallA->brepFace);
                gp_Pnt p2 = GetFaceSamplePoint(wallB->brepFace);

                // 如果两壁之间没有材料 (!IsMaterial)，那就是槽
                if (!IsMaterialBetween(p1, p2, partNode.brepShape))
                {
                    // 找到了槽，创建特征
                    CompositeFeature slotFeature;
                    slotFeature.partID = partNode.partID;
                    slotFeature.type = CompositeFeatureType::SLOT;
                    slotFeature.compositeID = partNode.partID + "_Slot_" + std::to_string(
                        partNode.compositeFeatures.size());

                    slotFeature.planeNormal = baseNormal;
                    slotFeature.axis = gp_Ax1(baseFeature->geometricParams.locationPoint,
                                              baseNormal.Crossed(wallA->geometricParams.axisVector));

                    // 使用采样点计算距离更安全
                    gp_Pln planeB(wallB->geometricParams.locationPoint, wallB->geometricParams.axisVector);
                    slotFeature.width = planeB.Distance(p1);

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
        // 1. 基座检查
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
            if (!neighbor->geometricParams.axisVector.IsNormal(baseNormal, Constants::AngleTolerance)) continue;

            // 基座有效性检查 (榫的顶面与侧壁应为凸连接/实体)
            gp_Pnt pBase = GetFaceSamplePoint(baseFeature->brepFace);
            gp_Pnt pWall = GetFaceSamplePoint(neighbor->brepFace);
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

                // A. 法线反向平行
                if (!wallA->geometricParams.axisVector.IsOpposite(wallB->geometricParams.axisVector,
                                                                  Constants::AngleTolerance))
                {
                    continue;
                }

                // B. 内部材质检测 (确保是实体)
                gp_Pnt pWallA = GetFaceSamplePoint(wallA->brepFace);
                gp_Pnt pWallB = GetFaceSamplePoint(wallB->brepFace);

                if (IsMaterialBetween(pWallA, pWallB, partNode.brepShape))
                {
                    // ============================================================
                    // [新增] 全局鲁棒性检查：根部连接检测 (Root Check)
                    // 目的：区分"真榫"(凸特征)和"平板"(凸实体)。
                    // 逻辑：真榫的侧壁必须至少有一个"凹"的邻居(连接到基体地板)。
                    //       如果侧壁的所有邻居都是"凸"连接，那它就是一个孤立的块。
                    // ============================================================

                    bool hasConcaveConnection = false;

                    // 检查 WallA 的邻居
                    for (const auto& [adjID, continuity] : wallA->adjacencyList)
                    {
                        if (adjID == baseFeature->faceID) continue; // 忽略顶面
                        auto it = featureMap.find(adjID);
                        if (it == featureMap.end()) continue;

                        // 采样并检查连接性质
                        gp_Pnt pNeighbor = GetFaceSamplePoint(it->second->brepFace);
                        // 如果两面中点是空气(false)，说明是凹连接 -> 真特征
                        if (!IsMaterialBetween(pWallA, pNeighbor, partNode.brepShape))
                        {
                            hasConcaveConnection = true;
                            break;
                        }
                    }

                    // 如果 WallA 没找到，再给 WallB 一次机会
                    if (!hasConcaveConnection)
                    {
                        for (const auto& [adjID, continuity] : wallB->adjacencyList)
                        {
                            if (adjID == baseFeature->faceID) continue;
                            auto it = featureMap.find(adjID);
                            if (it == featureMap.end()) continue;

                            gp_Pnt pNeighbor = GetFaceSamplePoint(it->second->brepFace);
                            if (!IsMaterialBetween(pWallB, pNeighbor, partNode.brepShape))
                            {
                                hasConcaveConnection = true;
                                break;
                            }
                        }
                    }

                    // [判决]
                    if (!hasConcaveConnection)
                    {
                        // 这是一个孤立的长方体，不是榫特征
                        // 应该跳过，留给后续的 FunctionalPlane 规则去处理
                        continue;
                    }

                    // ============================================================
                    // 匹配成功，创建特征
                    // ============================================================
                    CompositeFeature tongueFeature;
                    tongueFeature.partID = partNode.partID;
                    tongueFeature.type = CompositeFeatureType::TONGUE;
                    tongueFeature.compositeID = partNode.partID + "_Tongue_" + std::to_string(
                        partNode.compositeFeatures.size());

                    tongueFeature.planeNormal = baseNormal;
                    tongueFeature.planeLocation = baseFeature->geometricParams.locationPoint;
                    tongueFeature.axis = gp_Ax1(baseFeature->geometricParams.locationPoint,
                                                baseNormal.Crossed(wallA->geometricParams.axisVector));

                    gp_Pln planeB(wallB->geometricParams.locationPoint, wallB->geometricParams.axisVector);
                    tongueFeature.width = planeB.Distance(pWallA); // 使用采样点

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
        const BRepAdaptor_Surface surfAdaptor(face);
        const double uMid = (surfAdaptor.FirstUParameter() + surfAdaptor.LastUParameter()) / 2.0;
        const double vMid = (surfAdaptor.FirstVParameter() + surfAdaptor.LastVParameter()) / 2.0;
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
        // ========================================================================
        // 鲁棒性升级：子特征遍历匹配 (Sub-Feature Traversal)
        // 不再依赖复合特征的"摘要"信息(feat.planeNormal)，而是直接检查构成特征的每一个物理面。
        // 这样无论面是"基座"还是"侧壁"，只要发生了物理接触，都能被捕获。
        // ========================================================================

        // 辅助Lambda：根据ID查找原子特征
        // 注意：为了性能，这只是一个简单的查找。在生产环境中，可以使用更高效的索引。
        auto getAtomicFeature = [&](const PartNode& node, const std::string& faceID) -> std::shared_ptr<AtomicFeature>
        {
            for (const auto& af : node.atomicFeatures)
            {
                if (af->faceID == faceID) return af;
            }
            return nullptr;
        };

        // ========================================================================
        // 双重循环：遍历 A 的所有子面 vs B 的所有子面
        // ========================================================================
        for (const std::string& idA : featA.childAtomicFeatureIDs)
        {
            auto atomA = getAtomicFeature(nodeA, idA);
            // 只处理平面类型的原子特征
            if (!atomA || atomA->atomType != AtomType::PLANE) continue;

            // 1. 获取原子特征 A 的世界坐标系几何参数
            // [重要] 我们使用原子特征自己的几何参数，而不是复合特征的摘要
            gp_Dir localNormA = atomA->geometricParams.axisVector;
            gp_Pnt localLocA = atomA->geometricParams.locationPoint;

            gp_Dir globalNormA = TransformDir(localNormA, nodeA.transformation);
            gp_Pnt globalLocA = TransformPoint(localLocA, nodeA.transformation);

            for (const std::string& idB : featB.childAtomicFeatureIDs)
            {
                auto atomB = getAtomicFeature(nodeB, idB);
                // 只处理平面类型的原子特征
                if (!atomB || atomB->atomType != AtomType::PLANE) continue;

                // 2. 获取原子特征 B 的世界坐标系几何参数
                gp_Dir localNormB = atomB->geometricParams.axisVector;
                gp_Pnt localLocB = atomB->geometricParams.locationPoint;

                gp_Dir globalNormB = TransformDir(localNormB, nodeB.transformation);
                gp_Pnt globalLocB = TransformPoint(localLocB, nodeB.transformation);

                // ====================================================================
                // 3. 几何检查 (检查这两个具体的子面是否贴合)
                // ====================================================================

                // A. 法线反向平行 (IsOpposite)
                if (!globalNormA.IsOpposite(globalNormB, Constants::AngleTolerance))
                {
                    continue;
                }

                // B. 共面距离 (Coplanarity)
                // 计算点 A 到平面 B 的距离
                gp_Pln planeB(globalLocB, globalNormB);

                // 检查距离是否在容差范围内
                if (double dist = planeB.Distance(globalLocA); dist < Constants::DistanceTolerance)
                {
                    // C. [关键] 重叠检测 (Overlap Check)
                    // 防止两个共面但相距很远的面被误判。
                    // 我们计算这两个面的包围盒，看它们在空间中是否有交集。

                    Bnd_Box boxA;
                    BRepBndLib::Add(atomA->brepFace.Moved(TopLoc_Location(nodeA.transformation)), boxA);

                    Bnd_Box boxB;
                    BRepBndLib::Add(atomB->brepFace.Moved(TopLoc_Location(nodeB.transformation)), boxB);

                    // 稍微扩大包围盒以容忍边缘接触
                    boxA.Enlarge(0.1);
                    boxB.Enlarge(0.1);

                    if (!boxA.IsOut(boxB))
                    {
                        // ================================================================
                        // 匹配成功！生成约束
                        // ================================================================
                        AssemblyConstraint constraint;
                        constraint.type = ConstraintType::COINCIDENT;
                        constraint.partID_A = nodeA.partID;
                        constraint.featureID_A = featA.compositeID;
                        constraint.partID_B = nodeB.partID;
                        constraint.featureID_B = featB.compositeID;

                        // 4. 去重检查
                        // 防止同一个复合特征对生成多条重复约束 (例如 A的子面1和B的子面1匹配，A的子面2和B的子面2也匹配)
                        bool exists = false;
                        for (const auto& c : constraints_)
                        {
                            if (c.type == ConstraintType::COINCIDENT &&
                                c.featureID_A == constraint.featureID_A &&
                                c.featureID_B == constraint.featureID_B)
                            {
                                exists = true;
                                break;
                            }
                        }

                        if (!exists)
                        {
                            constraints_.push_back(constraint);
                            // 既然这对复合特征已经建立了一个接触约束，我们可以跳出循环，
                            // 避免为同一对特征生成重复的 COINCIDENT 记录。
                            // 使用 goto 跳出两层循环是最直接的方式。
                            goto NextPair;
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

        const gp_Ax1 axisSlot = TransformAxis(slot->axis, nodeSlot->transformation);
        const gp_Ax1 axisTongue = TransformAxis(tongue->axis, nodeTongue->transformation);

        if (!axisSlot.Direction().IsParallel(axisTongue.Direction(), Constants::AngleTolerance))
        {
            return;
        }

        if (std::abs(slot->width - tongue->width) > 1.0)
        {
            return;
        }

        const gp_Dir normSlot = TransformDir(slot->planeNormal, nodeSlot->transformation);
        const gp_Dir normTongue = TransformDir(tongue->planeNormal, nodeTongue->transformation);

        if (!normSlot.IsOpposite(normTongue, Constants::AngleTolerance))
        {
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
