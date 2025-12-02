//
// Created by zhuge on 2025/11/19.
//

#include "ASGBuilder.h"

// OpenCASCADE Standard Headers
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
#include <Geom2d_Curve.hxx>
#include <gp_Pnt2d.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <gp_Vec.hxx>
#include <ElCLib.hxx>

// Standard Library
#include <iomanip>
#include <sstream>
#include <cmath>
#include <limits>
#include <fstream>

namespace ASG
{
    // ============================================================================
    // Internal Helper Namespace (Local to this file)
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
        }
    }

    // ============================================================================
    // STEP 2: Atomic Feature Classification - Geometry Identification
    // ============================================================================

    std::pair<AtomType, GeometricParams> ASGBuilder::IdentifyGeometryType(const TopoDS_Face& face)
    {
        auto atomType = AtomType::OTHER;
        GeometricParams params;

        // a. Extract the underlying geometric surface from the topological face
        const Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
        if (surf.IsNull())
        {
            return {atomType, params};
        }

        // b. Attempt to cast the surface to each known analytical type
        //    Each successful cast populates params with type-specific data

        if (const Handle(Geom_Plane) plane = Handle(Geom_Plane)::DownCast(surf))
        {
            // c. Plane: extract location point and normal direction
            atomType = AtomType::PLANE;
            const gp_Pln pln = plane->Pln();
            params.locationPoint = pln.Location();
            params.axisVector = pln.Axis().Direction();
        }
        else if (const Handle(Geom_CylindricalSurface) cyl = Handle(Geom_CylindricalSurface)::DownCast(surf))
        {
            // d. Cylinder: extract axis, radius; height computed separately later
            atomType = AtomType::CYLINDER;
            const gp_Cylinder cylinder = cyl->Cylinder();
            params.locationPoint = cylinder.Location();
            params.axisVector = cylinder.Axis().Direction();
            params.radius = cylinder.Radius();
        }
        else if (const Handle(Geom_ConicalSurface) cone = Handle(Geom_ConicalSurface)::DownCast(surf))
        {
            // e. Cone: extract apex, axis, radius at reference location, and semi-angle
            atomType = AtomType::CONE;
            const gp_Cone gCone = cone->Cone();
            params.locationPoint = gCone.Location();
            params.axisVector = gCone.Axis().Direction();
            params.radius = gCone.RefRadius();
            params.semiAngle = gCone.SemiAngle();
        }
        else if (const Handle(Geom_SphericalSurface) sphere = Handle(Geom_SphericalSurface)::DownCast(surf))
        {
            // f. Sphere: extract center and radius
            atomType = AtomType::SPHERE;
            const gp_Sphere gSphere = sphere->Sphere();
            params.locationPoint = gSphere.Location();
            params.radius = gSphere.Radius();
        }
        else if (const Handle(Geom_ToroidalSurface) torus = Handle(Geom_ToroidalSurface)::DownCast(surf))
        {
            // g. Torus: extract axis, major/minor radii
            atomType = AtomType::TORUS;
            const gp_Torus gTorus = torus->Torus();
            params.locationPoint = gTorus.Location();
            params.axisVector = gTorus.Axis().Direction();
            params.majorRadius = gTorus.MajorRadius();
            params.minorRadius = gTorus.MinorRadius();
        }
        else if (Handle(Geom_BSplineSurface)::DownCast(surf))
        {
            // h. B-Spline surface: no simple parameters to extract
            atomType = AtomType::BSPLINE;
        }

        // i. Return the identified type plus extracted parameters
        return {atomType, params};
    }

    // ============================================================================
    // STEP 2: Atomic Feature Classification - Concavity Determination
    // ============================================================================

    FormType ASGBuilder::DetermineConcavity(
        const TopoDS_Face& face,
        const AtomType& atomType,
        const GeometricParams& geomParams,
        const TopoDS_Shape& parentSolid)
    {
        // a. Only curved surfaces (cylinder, cone, sphere, torus) have meaningful concavity
        if (atomType != AtomType::CYLINDER &&
            atomType != AtomType::CONE &&
            atomType != AtomType::SPHERE &&
            atomType != AtomType::TORUS)
        {
            return FormType::NEUTRAL;
        }

        gp_Pnt testPoint;
        const gp_Pnt samplePoint = GetFaceSamplePoint(face);

        // [FIX] Improved Logic for Cylinder and Cone
        if (atomType == AtomType::CYLINDER || atomType == AtomType::CONE)
        {
            // b. Project samplePoint onto the axis to get the correct center for this slice
            //    (Original code used geomParams.locationPoint which causes diagonal drift)
            gp_Vec axisVec(geomParams.axisVector);
            gp_Vec toSample(geomParams.locationPoint, samplePoint);

            // Projection length along axis
            double projection = toSample.Dot(axisVec);

            // The point on axis closest to samplePoint
            gp_Pnt axisProjPoint = geomParams.locationPoint.Translated(projection * axisVec);

            // Compute TRUE radial vector (perpendicular to axis)
            gp_Vec radialDir(axisProjPoint, samplePoint);
            double localRadius = radialDir.Magnitude();

            // c. Robust offset distance calculation
            //    For cones, localRadius is safer than geomParams.radius (RefRadius)
            if (localRadius > 1e-9)
            {
                radialDir.Normalize();
                // Move halfway towards the axis
                testPoint = samplePoint.Translated(-localRadius * Constants::ConcavityOffsetRatio * radialDir);
            }
            else
            {
                // Degenerate case (e.g. apex of cone), treat as neutral or skip
                return FormType::NEUTRAL;
            }
        }
        else if (atomType == AtomType::SPHERE)
        {
            // d. For sphere: offset toward the center
            const gp_Pnt center = geomParams.locationPoint;
            gp_Vec toCenter(samplePoint, center);
            if (toCenter.Magnitude() > 1e-9)
            {
                toCenter.Normalize();
                testPoint = samplePoint.Translated(geomParams.radius * Constants::ConcavityOffsetRatio * toCenter);
            }
            else
            {
                return FormType::NEUTRAL;
            }
        }
        else
        {
            // e. Torus: Compute direction to axis similarly to cylinder
            gp_Vec axisVec(geomParams.axisVector);
            gp_Vec toSample(geomParams.locationPoint, samplePoint);
            double projection = toSample.Dot(axisVec);
            gp_Pnt axisProjPoint = geomParams.locationPoint.Translated(projection * axisVec);
            gp_Vec radialDir(axisProjPoint, samplePoint);

            if (radialDir.Magnitude() > 1e-9) {
                radialDir.Normalize();
                // For torus, minor radius is the tube radius.
                // Move inward by a fraction of the minor radius.
                double offset = geomParams.minorRadius > 1e-6 ? geomParams.minorRadius * Constants::ConcavityOffsetRatio : 0.1;
                testPoint = samplePoint.Translated(-offset * radialDir);
            } else {
                return FormType::NEUTRAL;
            }
        }

        // f. Classify the test point
        //    IN  => Material exists towards axis => Shaft (CONVEX)
        //    OUT => Void exists towards axis     => Hole  (CONCAVE)
        const BRepClass3d_SolidClassifier classifier(parentSolid, testPoint, 1e-7);
        if (classifier.State() == TopAbs_IN)
        {
            return FormType::CONVEX;
        }
        if (classifier.State() == TopAbs_OUT)
        {
            return FormType::CONCAVE;
        }

        return FormType::NEUTRAL;
    }

    // ============================================================================
    // STEP 2: Atomic Feature Classification - Adjacency Analysis
    // ============================================================================

    double ASGBuilder::ComputeEdgeDihedralAngle(const TopoDS_Edge& edge, const TopoDS_Face& f1, const TopoDS_Face& f2)
    {
        // a. Sample the midpoint parameter on the edge curve
        double uMin, uMax;
        const Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, uMin, uMax);
        if (curve.IsNull())
        {
            return 0.0;
        }
        //const double uMid = (uMin + uMax) / 2.0;
        //const gp_Pnt edgePoint = curve->Value(uMid);

        // b. Retrieve the underlying surfaces for both faces
        const Handle(Geom_Surface) surf1 = BRep_Tool::Surface(f1);
        const Handle(Geom_Surface) surf2 = BRep_Tool::Surface(f2);
        if (surf1.IsNull() || surf2.IsNull())
        {
            return 0.0;
        }

        // c. Project the edge midpoint onto each surface to get UV coordinates
        double u1, v1, u2, v2;
        BRepAdaptor_Surface adapt1(f1);
        BRepAdaptor_Surface adapt2(f2);

        // d. Use a simple UV midpoint approximation (robust for most cases)
        u1 = (adapt1.FirstUParameter() + adapt1.LastUParameter()) / 2.0;
        v1 = (adapt1.FirstVParameter() + adapt1.LastVParameter()) / 2.0;
        u2 = (adapt2.FirstUParameter() + adapt2.LastUParameter()) / 2.0;
        v2 = (adapt2.FirstVParameter() + adapt2.LastVParameter()) / 2.0;

        // e. Evaluate surface normals at the UV points
        gp_Pnt p1, p2;
        gp_Vec d1u, d1v, d2u, d2v;
        surf1->D1(u1, v1, p1, d1u, d1v);
        surf2->D1(u2, v2, p2, d2u, d2v);

        // f. Compute normals via cross product and adjust for face orientation
        gp_Vec normal1 = d1u.Crossed(d1v);
        gp_Vec normal2 = d2u.Crossed(d2v);

        if (normal1.Magnitude() < 1e-9 || normal2.Magnitude() < 1e-9)
        {
            return 0.0;
        }

        normal1.Normalize();
        normal2.Normalize();

        // g. Reverse normal if face orientation is reversed
        if (f1.Orientation() == TopAbs_REVERSED)
        {
            normal1.Reverse();
        }
        if (f2.Orientation() == TopAbs_REVERSED)
        {
            normal2.Reverse();
        }

        // h. Compute the angle between normals (dihedral angle)
        const double dotProduct = normal1.Dot(normal2);
        const double angle = std::acos(std::clamp(dotProduct, -1.0, 1.0));

        return angle;
    }

    std::vector<AdjacencyInfo> ASGBuilder::AnalyzeTopologicalAdjacency(
        const TopoDS_Face& face,
        const std::string& faceID,
        const TopoDS_Shape& parentSolid,
        const TopTools_IndexedDataMapOfShapeListOfShape& edgeToFacesMap)
    {
        std::vector<AdjacencyInfo> adjacencyList;

        // a. Build a global edge-to-faces map for the entire solid
        //TopTools_IndexedDataMapOfShapeListOfShape edgeToFacesMap;
        //TopExp::MapShapesAndAncestors(parentSolid, TopAbs_EDGE, TopAbs_FACE, edgeToFacesMap);

        // b. Iterate over all edges belonging to the current face
        for (TopExp_Explorer edgeExplorer(face, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next())
        {
            const TopoDS_Edge edge = TopoDS::Edge(edgeExplorer.Current());

            // c. Retrieve the list of faces sharing this edge
            if (!edgeToFacesMap.Contains(edge))
            {
                continue;
            }
            const TopTools_ListOfShape& facesOnEdge = edgeToFacesMap.FindFromKey(edge);

            // d. Identify the neighbor face (the other face sharing the edge)
            TopoDS_Face neighborFace;
            bool foundNeighbor = false;
            for (const TopoDS_Shape& shapeOnEdge : facesOnEdge)
            {
                const TopoDS_Face candidateFace = TopoDS::Face(shapeOnEdge);
                if (!candidateFace.IsSame(face))
                {
                    neighborFace = candidateFace;
                    foundNeighbor = true;
                    break;
                }
            }

            if (!foundNeighbor) continue;


            // e. Look up the neighbor's ID from the global face map
            const auto it = faceIDMap_.find(neighborFace);
            if (it == faceIDMap_.end()) continue;
            const std::string neighborFaceID = it->second;

            // f. Skip self-references
            if (neighborFaceID == faceID) continue;

            // g. Compute the dihedral angle between the two faces
            const double dihedralAngle = ComputeEdgeDihedralAngle(edge, face, neighborFace);

            // h. Classify continuity based on the dihedral angle
            auto continuity = ContinuityType::UNKNOWN;
            if (dihedralAngle > Constants::C0_Continuity_Threshold)
            {
                continuity = ContinuityType::C0; // Sharp edge
            }
            else if (dihedralAngle > Constants::C1_Continuity_Threshold)
            {
                continuity = ContinuityType::C1; // Tangent blend
            }
            else
            {
                continuity = ContinuityType::C2; // Smooth curvature-continuous
            }

            // i. Record the adjacency information
            AdjacencyInfo adjInfo;
            adjInfo.neighborFaceID = neighborFaceID;
            adjInfo.continuityType = continuity;
            adjInfo.dihedralAngle = dihedralAngle;

            adjacencyList.push_back(adjInfo);
        }

        return adjacencyList;
    }

    // ============================================================================
    // STEP 2: Atomic Feature Classification - Utility Functions
    // ============================================================================

    double ASGBuilder::ComputeFaceArea(const TopoDS_Face& face)
    {
        // a. Use OpenCASCADE's mass properties to compute surface area
        GProp_GProps props;
        BRepGProp::SurfaceProperties(face, props);
        return props.Mass();
    }

    gp_Pnt ASGBuilder::GetFaceSamplePoint(const TopoDS_Face& face)
    {
        // a. Retrieve the underlying parametric surface
        const Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
        double uMin, uMax, vMin, vMax;
        surf->Bounds(uMin, uMax, vMin, vMax);

        // b. Evaluate at the midpoint of the parameter domain
        const double uMid = (uMin + uMax) / 2.0;
        const double vMid = (vMin + vMax) / 2.0;

        return surf->Value(uMid, vMid);
    }

    double ASGBuilder::ComputeCylinderHeight(const TopoDS_Face& face, const gp_Pnt& axisPoint, const gp_Dir& axisVector)
    {
        double minProj = std::numeric_limits<double>::max();
        double maxProj = std::numeric_limits<double>::lowest();

        // a. Iterate over all edges of the cylindrical face
        for (TopExp_Explorer edgeExplorer(face, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next())
        {
            const TopoDS_Edge edge = TopoDS::Edge(edgeExplorer.Current());

            // b. For each edge, examine its vertices
            for (TopExp_Explorer vertexExplorer(edge, TopAbs_VERTEX); vertexExplorer.More(); vertexExplorer.Next())
            {
                const TopoDS_Vertex vertex = TopoDS::Vertex(vertexExplorer.Current());
                const gp_Pnt point = BRep_Tool::Pnt(vertex);

                // c. Project the vertex onto the cylinder axis
                const gp_Vec toPoint(axisPoint, point);
                const double projection = toPoint.Dot(gp_Vec(axisVector));

                // d. Track min and max projections
                if (projection < minProj) minProj = projection;
                if (projection > maxProj) maxProj = projection;
            }
        }

        // e. Height is the span along the axis
        return maxProj > minProj ? maxProj - minProj : 0.0;
    }

    // ============================================================================
    // STEP 3: Composite Feature Recognition - Hole Features
    // ============================================================================

    bool ASGBuilder::RecognizeHoleFeature(
        PartNode& partNode,
        const std::shared_ptr<AtomicFeature>& feature,
        FeatureMap& featureMap)
    {
        // a. Validate that the candidate is a concave cylinder
        if (feature->atomType != AtomType::CYLINDER || feature->formType != FormType::CONCAVE)
        {
            return false;
        }

        // b. Initialize the composite hole feature metadata
        CompositeFeature holeFeature;
        holeFeature.partID = partNode.partID;
        holeFeature.type = CompositeFeatureType::HOLE;
        holeFeature.compositeID = partNode.partID + "_Hole_" + std::to_string(partNode.compositeFeatures.size());
        holeFeature.nominalRadius = feature->geometricParams.radius;
        holeFeature.height = feature->geometricParams.height;
        holeFeature.axis = gp_Ax1(feature->geometricParams.locationPoint, feature->geometricParams.axisVector);

        // c. Attach all fragments belonging to the cylinder wall
        if (feature->isMainFragment)
        {
            holeFeature.childAtomicFeatureIDs = feature->fragmentFaceIDs;
        }
        else
        {
            holeFeature.childAtomicFeatureIDs.push_back(feature->faceID);
        }

        // d. Search for a bottom surface (plane or cone) to determine blind vs. through
        bool hasBottom = false;
        const double holeArea = M_PI * std::pow(holeFeature.nominalRadius, 2);

        for (const auto& [neighborFaceID, continuityType, dihedralAngle] : feature->adjacencyList)
        {
            auto it = featureMap.find(neighborFaceID);
            if (it == featureMap.end()) continue;
            const auto neighbor = it->second;
            if (neighbor->isConsumed) continue;

            // e. Check for planar bottom with similar area to the hole cross-section
            if (neighbor->atomType == AtomType::PLANE && continuityType == ContinuityType::C0)
            {
                if (std::abs(neighbor->area - holeArea) < 0.2 * holeArea)
                {
                    hasBottom = true;
                    holeFeature.childAtomicFeatureIDs.push_back(neighbor->faceID);
                    neighbor->isConsumed = true;
                }
            }
            // f. Conical bottoms also indicate blind holes
            else if (neighbor->atomType == AtomType::CONE)
            {
                hasBottom = true;
                holeFeature.childAtomicFeatureIDs.push_back(neighbor->faceID);
                neighbor->isConsumed = true;
            }
        }

        // g. Set subtype based on the presence of a bottom surface
        holeFeature.holeSubType = hasBottom ? HoleType::BLIND : HoleType::THROUGH;

        // h. Mark all constituent faces as consumed
        for (const auto& childID : holeFeature.childAtomicFeatureIDs)
        {
            if (auto it = featureMap.find(childID); it != featureMap.end())
            {
                it->second->isConsumed = true;
            }
        }

        // i. Register the hole feature with the part
        partNode.compositeFeatures.push_back(holeFeature);
        return true;
    }

    // ============================================================================
    // STEP 3: Composite Feature Recognition - Shaft Features
    // ============================================================================

    bool ASGBuilder::RecognizeShaftFeature(
        PartNode& partNode,
        const std::shared_ptr<AtomicFeature>& feature,
        FeatureMap& featureMap)
    {
        // a. Validate that the candidate is a convex cylinder
        if (feature->atomType != AtomType::CYLINDER || feature->formType != FormType::CONVEX)
        {
            return false;
        }

        // b. Initialize the composite shaft feature metadata
        CompositeFeature shaftFeature;
        shaftFeature.partID = partNode.partID;
        shaftFeature.type = CompositeFeatureType::SHAFT;
        shaftFeature.compositeID = partNode.partID + "_Shaft_" + std::to_string(partNode.compositeFeatures.size());
        shaftFeature.nominalRadius = feature->geometricParams.radius;
        shaftFeature.height = feature->geometricParams.height;
        shaftFeature.axis = gp_Ax1(feature->geometricParams.locationPoint, feature->geometricParams.axisVector);

        // c. Attach fragments or the single face
        if (feature->isMainFragment)
        {
            shaftFeature.childAtomicFeatureIDs = feature->fragmentFaceIDs;
        }
        else
        {
            shaftFeature.childAtomicFeatureIDs.push_back(feature->faceID);
        }

        // d. Mark all constituent faces as consumed
        for (const auto& childID : shaftFeature.childAtomicFeatureIDs)
        {
            if (auto it = featureMap.find(childID); it != featureMap.end())
            {
                it->second->isConsumed = true;
            }
        }

        // e. Register the shaft feature with the part
        partNode.compositeFeatures.push_back(shaftFeature);
        return true;
    }

    // ============================================================================
    // STEP 3: Composite Feature Recognition - Step Plane Features
    // ============================================================================

    bool ASGBuilder::RecognizeStepPlaneFeature(
        PartNode& partNode,
        const std::shared_ptr<AtomicFeature>& feature,
        FeatureMap& featureMap)
    {
        // a. Validate that the candidate is a neutral plane
        if (feature->atomType != AtomType::PLANE || feature->formType != FormType::NEUTRAL)
        {
            return false;
        }

        // b. Check for sharp (C0) adjacency to at least one cylinder
        bool isAdjacentToCylinder = false;
        for (const auto& [neighborFaceID, continuityType, dihedralAngle] : feature->adjacencyList)
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

        if (!isAdjacentToCylinder)
        {
            return false;
        }

        // c. Promote the plane to a step plane composite feature
        CompositeFeature stepFeature;
        stepFeature.partID = partNode.partID;
        stepFeature.type = CompositeFeatureType::STEP_PLANE;
        stepFeature.compositeID = partNode.partID + "_StepPlane_" + std::to_string(partNode.compositeFeatures.size());
        stepFeature.planeNormal = feature->geometricParams.axisVector;
        stepFeature.planeLocation = feature->geometricParams.locationPoint;
        stepFeature.planeArea = feature->area;
        stepFeature.childAtomicFeatureIDs.push_back(feature->faceID);

        // d. Mark as consumed and register
        feature->isConsumed = true;
        partNode.compositeFeatures.push_back(stepFeature);
        return true;
    }

    // ============================================================================
    // STEP 3: Composite Feature Recognition - Functional Plane Features
    // ============================================================================

    bool ASGBuilder::RecognizeFunctionalPlaneFeature(
        PartNode& partNode,
        const std::shared_ptr<AtomicFeature>& feature,
        FeatureMap& featureMap)
    {
        // a. Validate that the candidate is a neutral plane and not yet consumed
        if (feature->atomType != AtomType::PLANE || feature->formType != FormType::NEUTRAL)
        {
            return false;
        }
        if (!feature->isFunctional || feature->isConsumed)
        {
            return false;
        }

        // b. Promote the plane to a functional plane composite feature
        CompositeFeature planeFeature;
        planeFeature.partID = partNode.partID;
        planeFeature.type = CompositeFeatureType::FUNCTIONAL_PLANE;
        planeFeature.compositeID = partNode.partID + "_FuncPlane_" + std::to_string(partNode.compositeFeatures.size());
        planeFeature.planeNormal = feature->geometricParams.axisVector;
        planeFeature.planeLocation = feature->geometricParams.locationPoint;
        planeFeature.planeArea = feature->area;
        planeFeature.childAtomicFeatureIDs.push_back(feature->faceID);

        // c. Mark as consumed and register
        feature->isConsumed = true;
        partNode.compositeFeatures.push_back(planeFeature);
        return true;
    }

    // ============================================================================
    // STEP 3: Composite Feature Recognition - Utility Function
    // ============================================================================

    bool ASGBuilder::IsMaterialBetween(const gp_Pnt& p1, const gp_Pnt& p2, const TopoDS_Shape& solid)
    {
        // a. Compute the midpoint between the two sample positions
        const gp_Pnt midPoint((p1.X() + p2.X()) / 2.0,
                              (p1.Y() + p2.Y()) / 2.0,
                              (p1.Z() + p2.Z()) / 2.0);

        // b. Classify the midpoint against the solid volume with tight tolerance
        const BRepClass3d_SolidClassifier classifier(solid, midPoint, 1e-7);

        // c. Return true if the midpoint lies inside the material
        return classifier.State() == TopAbs_IN;
    }

    // ============================================================================
    // STEP 4: Assembly Constraint Recognition - Transformation Utilities
    // ============================================================================

    gp_Pnt ASGBuilder::TransformPoint(const gp_Pnt& localPnt, const gp_Trsf& trsf)
    {
        // a. Apply the transformation matrix to the point coordinates
        return localPnt.Transformed(trsf);
    }

    gp_Dir ASGBuilder::TransformDir(const gp_Dir& localDir, const gp_Trsf& trsf)
    {
        // a. Apply the transformation to the direction (ignores translation)
        return localDir.Transformed(trsf);
    }

    gp_Ax1 ASGBuilder::TransformAxis(const gp_Ax1& localAxis, const gp_Trsf& trsf)
    {
        // a. Transform both the axis location and direction
        const gp_Pnt transformedLoc = localAxis.Location().Transformed(trsf);
        const gp_Dir transformedDir = localAxis.Direction().Transformed(trsf);
        return {transformedLoc, transformedDir};
    }

    // ============================================================================
    // STEP 4: Assembly Constraint Recognition - Broad-Phase Collision Detection
    // ============================================================================

    bool ASGBuilder::CheckBoundingBoxCollision(const PartNode& nodeA, const PartNode& nodeB)
    {
        return !nodeA.worldBoundingBox.IsOut(nodeB.worldBoundingBox);
    }

    // ============================================================================
    // STEP 4: Assembly Constraint Recognition - Part-Pair Matching
    // ============================================================================

    void ASGBuilder::MatchPartPair(const PartNode& nodeA, const PartNode& nodeB)
    {
        // a. Iterate over all composite features in part A
        for (const auto& featA : nodeA.compositeFeatures)
        {
            // b. Iterate over all composite features in part B
            for (const auto& featB : nodeB.compositeFeatures)
            {
                // c. Test for coaxial constraints (hole-shaft, hole-hole, shaft-shaft)
                if ((featA.type == CompositeFeatureType::HOLE || featA.type == CompositeFeatureType::SHAFT) &&
                    (featB.type == CompositeFeatureType::HOLE || featB.type == CompositeFeatureType::SHAFT))
                {
                    MatchCoaxial(nodeA, featA, nodeB, featB);
                }

                // d. Test for planar coincident constraints (functional plane to functional plane)
                if ((featA.type == CompositeFeatureType::FUNCTIONAL_PLANE || featA.type == CompositeFeatureType::STEP_PLANE) &&
                    (featB.type == CompositeFeatureType::FUNCTIONAL_PLANE || featB.type == CompositeFeatureType::STEP_PLANE))
                {
                    MatchCoincident(nodeA, featA, nodeB, featB);
                }
            }
        }
    }

    // ============================================================================
    // STEP 4: Assembly Constraint Recognition - Coaxial Constraint Matching
    // ============================================================================

    void ASGBuilder::MatchCoaxial(const PartNode& nodeA, const CompositeFeature& featA,
                                  const PartNode& nodeB, const CompositeFeature& featB)
    {
        // a. Transform both axes to world coordinates
        const gp_Ax1 axisA_world = TransformAxis(featA.axis, nodeA.transformation);
        const gp_Ax1 axisB_world = TransformAxis(featB.axis, nodeB.transformation);

        // b. Check if the axes are parallel within angular tolerance
        if (!axisA_world.Direction().IsParallel(axisB_world.Direction(), Constants::AngleTolerance))
        {
            return;
        }

        // c. Compute the distance between the two axis lines
        const gp_Pnt locA = axisA_world.Location();
        const gp_Pnt locB = axisB_world.Location();
        const gp_Vec AB(locA, locB);
        const gp_Vec axisVec(axisA_world.Direction());
        const double distance = AB.Crossed(axisVec).Magnitude();

        // d. Verify that the axes are co-linear (distance near zero)
        if (distance > Constants::DistanceTolerance)
        {
            return;
        }

        // e. Check radius compatibility (hole should be slightly larger than shaft)
        const double radiusA = featA.nominalRadius;
        const double radiusB = featB.nominalRadius;
        const double radiusDiff = std::abs(radiusA - radiusB);

        // f. Allow small clearance for coaxial fit (typically 0.01 to 0.5 mm)
        if (radiusDiff > Constants::CoaxialRadiusDiffTolerance)
        {
            return;
        }

        // g. Record the coaxial constraint
        AssemblyConstraint constraint;
        constraint.type = ConstraintType::COAXIAL;
        constraint.partID_A = nodeA.partID;
        constraint.featureID_A = featA.compositeID;
        constraint.partID_B = nodeB.partID;
        constraint.featureID_B = featB.compositeID;
        constraint.value = radiusDiff;

        constraints_.push_back(constraint);
    }

    // ============================================================================
    // STEP 4: Assembly Constraint Recognition - Coincident Constraint Matching
    // ============================================================================

    void ASGBuilder::MatchCoincident(const PartNode& nodeA, const CompositeFeature& featA,
                                     const PartNode& nodeB, const CompositeFeature& featB)
    {
        // a. Transform plane normals and locations to world coordinates
        const gp_Dir normalA_world = TransformDir(featA.planeNormal, nodeA.transformation);
        const gp_Dir normalB_world = TransformDir(featB.planeNormal, nodeB.transformation);
        const gp_Pnt locA_world = TransformPoint(featA.planeLocation, nodeA.transformation);
        const gp_Pnt locB_world = TransformPoint(featB.planeLocation, nodeB.transformation);

        // b. Check if normals are opposite (mating planes face each other)
        const double dotProduct = normalA_world.Dot(normalB_world);
        if (dotProduct > -0.9)
        {
            return; // Normals are not sufficiently opposite
        }

        // c. Compute the distance between the two planes
        const gp_Vec AB(locA_world, locB_world);
        const double distance = std::abs(AB.Dot(gp_Vec(normalA_world)));

        // d. Verify that planes are co-planar within tolerance
        if (distance > Constants::DistanceTolerance)
        {
            return;
        }

        // e. Perform a simple bounding box overlap check in the plane projection
        //    (This is a simplified test; a full polygon intersection would be more accurate)
        //constexpr double overlapTolerance = 1.0; // mm

        // f. Record the coincident constraint
        AssemblyConstraint constraint;
        constraint.type = ConstraintType::COINCIDENT;
        constraint.partID_A = nodeA.partID;
        constraint.featureID_A = featA.compositeID;
        constraint.partID_B = nodeB.partID;
        constraint.featureID_B = featB.compositeID;
        constraint.value = distance;

        constraints_.push_back(constraint);
    }

    // ============================================================================
    // Output & Debugging - Export to JSON
    // ============================================================================

    bool ASGBuilder::ExportToJSON(const std::string& filePath) const
    {
        // a. Open the output file stream
        std::ofstream outFile(filePath);
        if (!outFile.is_open())
        {
            std::cerr << "Error: Cannot open file for writing: " << filePath << std::endl;
            return false;
        }

        // b. Begin JSON structure
        outFile << "{\n";
        outFile << "  \"assembly\": {\n";
        outFile << "    \"parts\": [\n";

        // c. Serialize each part and its composite features
        for (size_t i = 0; i < partNodes_.size(); ++i)
        {
            const auto& part = partNodes_[i];
            outFile << "      {\n";
            outFile << R"(        "partID": ")" << JSONUtils::EscapeJSONString(part.partID) << "\",\n";
            outFile << "        \"compositeFeatures\": [\n";

            for (size_t j = 0; j < part.compositeFeatures.size(); ++j)
            {
                const auto& feat = part.compositeFeatures[j];
                outFile << "          {\n";
                outFile << R"(            "featureID": ")" << JSONUtils::EscapeJSONString(feat.compositeID) << "\",\n";
                outFile << R"(            "type": ")";

                switch (feat.type)
                {
                case CompositeFeatureType::HOLE: outFile << "HOLE";
                    break;
                case CompositeFeatureType::SHAFT: outFile << "SHAFT";
                    break;
                case CompositeFeatureType::FUNCTIONAL_PLANE: outFile << "FUNCTIONAL_PLANE";
                    break;
                case CompositeFeatureType::STEP_PLANE: outFile << "STEP_PLANE";
                    break;
                default: outFile << "UNKNOWN";
                }

                outFile << "\"\n";
                outFile << "          }";
                if (j < part.compositeFeatures.size() - 1) outFile << ",";
                outFile << "\n";
            }

            outFile << "        ]\n";
            outFile << "      }";
            if (i < partNodes_.size() - 1) outFile << ",";
            outFile << "\n";
        }

        outFile << "    ]\n";
        outFile << "  }\n";
        outFile << "}\n";

        // d. Close the file and confirm success
        outFile.close();
        return true;
    }

    // ============================================================================
    // Output & Debugging - Print Statistics
    // ============================================================================

    void ASGBuilder::PrintStatistics() const
    {
        // a. Count total composite features across all parts
        int totalCompositeFeatures = 0;
        for (const auto& part : partNodes_)
        {
            totalCompositeFeatures += static_cast<int>(part.compositeFeatures.size());
        }

        // b. Display aggregate statistics to console
        std::cout << "========================================" << std::endl;
        std::cout << "Assembly Statistics:" << std::endl;
        std::cout << "  Total Parts: " << partNodes_.size() << std::endl;
        std::cout << "  Total Composite Features: " << totalCompositeFeatures << std::endl;
        std::cout << "========================================" << std::endl;
    }
} // namespace ASG
