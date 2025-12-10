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
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <Bnd_Box.hxx>
#include <gp_Vec.hxx>
#include <BRepLProp_SLprops.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <gp_Pnt2d.hxx>

#include <BRepLProp_SLprops.hxx>
#include <BRepTopAdaptor_FClass2d.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <gp_Pnt2d.hxx>
#include <TopAbs.hxx>

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
    // STEP 1: Geometry Sampling Utilities
    // ============================================================================

    double ASGBuilder::ComputeFaceArea(const TopoDS_Face& face)
    {
        // a. Accumulate precise surface properties using OpenCASCADE mass properties API
        GProp_GProps props;
        BRepGProp::SurfaceProperties(face, props);
        // b. Return the computed area (stored in the mass field for surfaces)
        return props.Mass();
    }

    gp_Pnt ASGBuilder::GetFaceSamplePoint(const TopoDS_Face& face)
    {
        // a. Access the underlying parametric surface and query its UV bounds
        const Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
        double uMin, uMax, vMin, vMax;
        surf->Bounds(uMin, uMax, vMin, vMax);
        // b. Evaluate the surface at the midpoint of its parameter domain to obtain a representative point
        const double uMid = (uMin + uMax) / 2.0;
        const double vMid = (vMin + vMax) / 2.0;
        // c. Return the sampled world-space point for downstream feature metrics
        return surf->Value(uMid, vMid);
    }

    double ASGBuilder::ComputeCylinderHeight(const TopoDS_Face& face, const gp_Pnt& axisPoint, const gp_Dir& axisVector)
    {
        double minProj = std::numeric_limits<double>::max();
        double maxProj = std::numeric_limits<double>::lowest();

        // a. Traverse every topological edge and vertex belonging to the cylindrical face
        for (TopExp_Explorer edgeExplorer(face, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next())
        {
            const TopoDS_Edge edge = TopoDS::Edge(edgeExplorer.Current());

            for (TopExp_Explorer vertexExplorer(edge, TopAbs_VERTEX); vertexExplorer.More(); vertexExplorer.Next())
            {
                const TopoDS_Vertex vertex = TopoDS::Vertex(vertexExplorer.Current());
                const gp_Pnt point = BRep_Tool::Pnt(vertex);

                // b. Project each vertex onto the axis to determine its signed distance along the axis direction
                const gp_Vec toPoint(axisPoint, point);
                const double projection = toPoint.Dot(gp_Vec(axisVector));

                // c. Track the minimum and maximum projections to capture the axial span
                if (projection < minProj) minProj = projection;
                if (projection > maxProj) maxProj = projection;
            }
        }

        // d. Compute the net height by subtracting the extremes, guarding against degenerate spans
        return maxProj > minProj ? maxProj - minProj : 0.0;
    }

    // ============================================================================
    // STEP 2: Atomic Feature Classification Helpers
    // ============================================================================

    std::pair<AtomType, GeometricParams> ASGBuilder::IdentifyGeometryType(const TopoDS_Face& face)
    {
        AtomType atomType;
        GeometricParams params;

        // a. Extract the analytic surface pointer backing the topological face
        BRepAdaptor_Surface adaptor(face);
        GeomAbs_SurfaceType surfType = adaptor.GetType();
        bool isReversed = face.Orientation() == TopAbs_REVERSED;

        // b. Attempt successive DownCast operations to known analytical surface categories
        switch (surfType)
        {
        case GeomAbs_Plane:
            {
                // c. Plane: capture reference point and oriented normal, flipping if the face is reversed
                atomType = AtomType::PLANE;
                gp_Pln gp_plane = adaptor.Plane();

                gp_Dir naturalNormal = gp_plane.Axis().Direction();
                params.axisVector = isReversed ? naturalNormal.Reversed() : naturalNormal;
                params.locationPoint = gp_plane.Location();
                break;
            }
        case GeomAbs_Cylinder:
            {
                // d. Cylinder: store axis direction, reference point, and radius
                atomType = AtomType::CYLINDER;
                gp_Cylinder gp_cyl = adaptor.Cylinder(); // 直接获取解析圆柱

                params.axisVector = gp_cyl.Axis().Direction();
                params.locationPoint = gp_cyl.Axis().Location();
                params.radius = gp_cyl.Radius();
                break;
            }
        case GeomAbs_Cone:
            {
                // e. Cone: capture apex, axis, reference radius, and semi-angle for later taper checks
                atomType = AtomType::CONE;
                gp_Cone gp_cone = adaptor.Cone();

                params.axisVector = gp_cone.Axis().Direction();
                params.locationPoint = gp_cone.Axis().Location();
                params.radius = gp_cone.RefRadius();
                params.semiAngle = gp_cone.SemiAngle();
                break;
            }
        case GeomAbs_Sphere:
            {
                // f. Sphere: store center and radius for curvature-driven reasoning
                atomType = AtomType::SPHERE;
                gp_Sphere gp_sphere = adaptor.Sphere();

                params.locationPoint = gp_sphere.Location();
                params.radius = gp_sphere.Radius();
                break;
            }
        case GeomAbs_Torus:
            {
                // g. Torus: record donut center, axis, and both radii for later matching
                atomType = AtomType::TORUS;
                gp_Torus gp_torus = adaptor.Torus();

                params.axisVector = gp_torus.Axis().Direction();
                params.locationPoint = gp_torus.Axis().Location();
                params.majorRadius = gp_torus.MajorRadius();
                params.minorRadius = gp_torus.MinorRadius();
                break;
            }
        case GeomAbs_BezierSurface:
        case GeomAbs_BSplineSurface:
            {
                atomType = AtomType::BSPLINE;
                break;
            }
        default:
            {
                atomType = AtomType::OTHER;
                break;
            }
        }
        // i. Return the detected atom type along with any populated geometric parameters
        return {atomType, params};
    }

    FormType ASGBuilder::DetermineConcavity(
        const TopoDS_Face& face,
        const AtomType& atomType,
        const GeometricParams& geomParams,
        const TopoDS_Shape& parentSolid)
    {
        // a. Curvature-based concavity only applies to curved primitives; others default to neutral
        if (atomType != AtomType::CYLINDER &&
            atomType != AtomType::CONE &&
            atomType != AtomType::SPHERE &&
            atomType != AtomType::TORUS)
        {
            return FormType::NEUTRAL;
        }

        gp_Pnt testPoint;
        const gp_Pnt samplePoint = GetFaceSamplePoint(face);

        if (atomType == AtomType::CYLINDER || atomType == AtomType::CONE)
        {
            // b. Project the sampled point to the axis to find the local center used for radial offset
            gp_Vec axisVec(geomParams.axisVector);
            gp_Vec toSample(geomParams.locationPoint, samplePoint);
            const double projection = toSample.Dot(axisVec);
            gp_Pnt axisProjPoint = geomParams.locationPoint.Translated(projection * axisVec);

            gp_Vec radialDir(axisProjPoint, samplePoint);
            const double localRadius = radialDir.Magnitude();

            // c. Normalize the radial vector and march toward the axis to probe material occupancy
            if (localRadius > 1e-9)
            {
                radialDir.Normalize();
                testPoint = samplePoint.Translated(-localRadius * Constants::ConcavityOffsetRatio * radialDir);
            }
            else
            {
                return FormType::NEUTRAL;
            }
        }
        else if (atomType == AtomType::SPHERE)
        {
            // d. For spheres, move toward the center along the radial direction
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
            // e. For tori, treat the tube similarly to a cylinder using the local axis projection
            gp_Vec axisVec(geomParams.axisVector);
            gp_Vec toSample(geomParams.locationPoint, samplePoint);
            const double projection = toSample.Dot(axisVec);
            gp_Pnt axisProjPoint = geomParams.locationPoint.Translated(projection * axisVec);
            gp_Vec radialDir(axisProjPoint, samplePoint);

            if (radialDir.Magnitude() > 1e-9)
            {
                radialDir.Normalize();
                const double offset = geomParams.minorRadius > 1e-6 ? geomParams.minorRadius * Constants::ConcavityOffsetRatio : 0.1;
                testPoint = samplePoint.Translated(-offset * radialDir);
            }
            else
            {
                return FormType::NEUTRAL;
            }
        }

        // f. Classify the offset point relative to the parent solid to infer concavity
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

    double ASGBuilder::ComputeEdgeDihedralAngle(const TopoDS_Edge& edge, const TopoDS_Face& f1, const TopoDS_Face& f2)
    {
        // a. Retrieve the shared edge curve to validate availability (parameter midpoint unused but ensures curve presence)
        double uMin, uMax;
        const Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, uMin, uMax);
        if (curve.IsNull())
        {
            return 0.0;
        }

        // b. Access both supporting surfaces for normal evaluation
        const Handle(Geom_Surface) surf1 = BRep_Tool::Surface(f1);
        const Handle(Geom_Surface) surf2 = BRep_Tool::Surface(f2);
        if (surf1.IsNull() || surf2.IsNull())
        {
            return 0.0;
        }

        // c. Approximate UV parameters at mid-domain locations for each surface
        double u1, v1, u2, v2;
        BRepAdaptor_Surface adapt1(f1);
        BRepAdaptor_Surface adapt2(f2);
        u1 = (adapt1.FirstUParameter() + adapt1.LastUParameter()) / 2.0;
        v1 = (adapt1.FirstVParameter() + adapt1.LastVParameter()) / 2.0;
        u2 = (adapt2.FirstUParameter() + adapt2.LastUParameter()) / 2.0;
        v2 = (adapt2.FirstVParameter() + adapt2.LastVParameter()) / 2.0;

        // d. Compute surface tangents and normals at the chosen UV samples
        gp_Pnt p1, p2;
        gp_Vec d1u, d1v, d2u, d2v;
        surf1->D1(u1, v1, p1, d1u, d1v);
        surf2->D1(u2, v2, p2, d2u, d2v);

        gp_Vec normal1 = d1u.Crossed(d1v);
        gp_Vec normal2 = d2u.Crossed(d2v);
        if (normal1.Magnitude() < 1e-9 || normal2.Magnitude() < 1e-9)
        {
            return 0.0;
        }

        // e. Normalize normals and respect face orientation to ensure consistent direction
        normal1.Normalize();
        normal2.Normalize();
        if (f1.Orientation() == TopAbs_REVERSED)
        {
            normal1.Reverse();
        }
        if (f2.Orientation() == TopAbs_REVERSED)
        {
            normal2.Reverse();
        }

        // f. Return the dihedral angle derived from the dot product between oriented normals
        const double dotProduct = normal1.Dot(normal2);
        return std::acos(std::clamp(dotProduct, -1.0, 1.0));
    }

    std::vector<AdjacencyInfo> ASGBuilder::AnalyzeTopologicalAdjacency(
        const TopoDS_Face& face,
        const std::string& faceID,
        const TopoDS_Shape& parentSolid,
        const TopTools_IndexedDataMapOfShapeListOfShape& edgeToFacesMap)
    {
        std::vector<AdjacencyInfo> adjacencyList;

        // a. Iterate over every edge belonging to the current face
        for (TopExp_Explorer edgeExplorer(face, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next())
        {
            const TopoDS_Edge edge = TopoDS::Edge(edgeExplorer.Current());

            // b. Skip edges missing from the global map (e.g., open boundaries)
            if (!edgeToFacesMap.Contains(edge))
            {
                continue;
            }
            const TopTools_ListOfShape& facesOnEdge = edgeToFacesMap.FindFromKey(edge);

            // c. Locate the neighboring face sharing this edge
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
            if (!foundNeighbor)
            {
                continue;
            }

            // d. Resolve the neighbor's face ID from the global lookup and avoid self references
            const auto it = faceIDMap_.find(neighborFace);
            if (it == faceIDMap_.end())
            {
                continue;
            }
            const std::string neighborFaceID = it->second;
            if (neighborFaceID == faceID)
            {
                continue;
            }

            // e. Compute dihedral angles and deduce continuity classification thresholds
            const double dihedralAngle = ComputeEdgeDihedralAngle(edge, face, neighborFace);
            ContinuityType continuity;
            if (dihedralAngle > Constants::C0_Continuity_Threshold)
            {
                continuity = ContinuityType::C0;
            }
            else if (dihedralAngle > Constants::C1_Continuity_Threshold)
            {
                continuity = ContinuityType::C1;
            }
            else
            {
                continuity = ContinuityType::C2;
            }

            // f. Store adjacency metadata for downstream graph construction
            AdjacencyInfo adjInfo;
            adjInfo.neighborFaceID = neighborFaceID;
            adjInfo.continuityType = continuity;
            adjInfo.dihedralAngle = dihedralAngle;
            adjacencyList.push_back(adjInfo);
        }

        return adjacencyList;
    }

    bool ASGBuilder::IsMaterialBetween(const gp_Pnt& p1, const gp_Pnt& p2, const TopoDS_Shape& solid)
    {
        // a. Evaluate the midpoint of the probing segment connecting the two supplied points
        const gp_Pnt midPoint((p1.X() + p2.X()) / 2.0,
                              (p1.Y() + p2.Y()) / 2.0,
                              (p1.Z() + p2.Z()) / 2.0);
        // b. Classify the midpoint against the solid volume using a tight tolerance
        const BRepClass3d_SolidClassifier classifier(solid, midPoint, 1e-7);
        // c. Report true only when the midpoint resides inside the material volume
        return classifier.State() == TopAbs_IN;
    }

    // ============================================================================
    // STEP 3: Composite Feature Recognition Helpers
    // ============================================================================

    bool ASGBuilder::RecognizeHoleFeature(
        PartNode& partNode,
        const std::shared_ptr<AtomicFeature>& feature,
        FeatureMap& featureMap)
    {
        // a. Guard against non-concave cylinders because only those can form hole walls
        if (feature->atomType != AtomType::CYLINDER || feature->formType != FormType::CONCAVE)
        {
            return false;
        }

        // b. Initialize hole metadata including identifiers and geometric references
        CompositeFeature holeFeature;
        holeFeature.partID = partNode.partID;
        holeFeature.type = CompositeFeatureType::HOLE;
        holeFeature.compositeID = partNode.partID + "_Hole_" + std::to_string(partNode.compositeFeatures.size());
        holeFeature.nominalRadius = feature->geometricParams.radius;
        holeFeature.height = feature->geometricParams.height;
        holeFeature.axis = gp_Ax1(feature->geometricParams.locationPoint, feature->geometricParams.axisVector);

        // c. Attach all fragments representing the cylindrical wall surface
        if (feature->isMainFragment)
        {
            holeFeature.childAtomicFeatureIDs = feature->fragmentFaceIDs;
        }
        else
        {
            holeFeature.childAtomicFeatureIDs.push_back(feature->faceID);
        }

        // d. Search adjacent faces for planar or conical bottoms to classify blind holes
        bool hasBottom = false;
        const double holeArea = M_PI * std::pow(holeFeature.nominalRadius, 2);
        for (const auto& [neighborFaceID, continuityType, dihedralAngle] : feature->adjacencyList)
        {
            auto it = featureMap.find(neighborFaceID);
            if (it == featureMap.end())
            {
                continue;
            }
            const auto neighbor = it->second;
            if (neighbor->isConsumed)
            {
                continue;
            }

            if (neighbor->atomType == AtomType::PLANE && continuityType == ContinuityType::C0)
            {
                // e. Accept planar bottoms whose area matches the circular cross-section within tolerance
                if (std::abs(neighbor->area - holeArea) < 0.2 * holeArea)
                {
                    hasBottom = true;
                    holeFeature.childAtomicFeatureIDs.push_back(neighbor->faceID);
                    neighbor->isConsumed = true;
                }
            }
            else if (neighbor->atomType == AtomType::CONE)
            {
                // f. Conical neighbors also signify blind terminations
                hasBottom = true;
                holeFeature.childAtomicFeatureIDs.push_back(neighbor->faceID);
                neighbor->isConsumed = true;
            }
        }

        // g. Label hole subtype based on whether a bottom surface was attached
        holeFeature.holeSubType = hasBottom ? HoleType::BLIND : HoleType::THROUGH;

        // h. Mark all contributing faces as consumed to avoid double counting
        for (const auto& childID : holeFeature.childAtomicFeatureIDs)
        {
            if (auto it = featureMap.find(childID); it != featureMap.end())
            {
                it->second->isConsumed = true;
            }
        }

        // i. Persist the newly recognized hole feature in the part registry
        partNode.compositeFeatures.push_back(holeFeature);
        return true;
    }

    bool ASGBuilder::RecognizeShaftFeature(
        PartNode& partNode,
        const std::shared_ptr<AtomicFeature>& feature,
        FeatureMap& featureMap)
    {
        // a. Require convex cylinders because shafts protrude instead of recessing
        if (feature->atomType != AtomType::CYLINDER || feature->formType != FormType::CONVEX)
        {
            return false;
        }

        // b. Initialize shaft metadata mirroring the hole structure
        CompositeFeature shaftFeature;
        shaftFeature.partID = partNode.partID;
        shaftFeature.type = CompositeFeatureType::SHAFT;
        shaftFeature.compositeID = partNode.partID + "_Shaft_" + std::to_string(partNode.compositeFeatures.size());
        shaftFeature.nominalRadius = feature->geometricParams.radius;
        shaftFeature.height = feature->geometricParams.height;
        shaftFeature.axis = gp_Ax1(feature->geometricParams.locationPoint, feature->geometricParams.axisVector);

        // c. Track all participating fragments or the lone face
        if (feature->isMainFragment)
        {
            shaftFeature.childAtomicFeatureIDs = feature->fragmentFaceIDs;
        }
        else
        {
            shaftFeature.childAtomicFeatureIDs.push_back(feature->faceID);
        }

        // d. Mark consumed faces to prevent reuse by other composites
        for (const auto& childID : shaftFeature.childAtomicFeatureIDs)
        {
            if (auto it = featureMap.find(childID); it != featureMap.end())
            {
                it->second->isConsumed = true;
            }
        }

        // e. Register the shaft composite
        partNode.compositeFeatures.push_back(shaftFeature);
        return true;
    }

    bool ASGBuilder::RecognizeStepPlaneFeature(
        PartNode& partNode,
        const std::shared_ptr<AtomicFeature>& feature,
        FeatureMap& featureMap)
    {
        // a. Accept only neutral planar faces since step planes are neither convex nor concave
        if (feature->atomType != AtomType::PLANE || feature->formType != FormType::NEUTRAL)
        {
            return false;
        }

        // b. Require at least one sharp adjacency to a cylindrical surface to indicate a functional step
        bool isAdjacentToCylinder = false;
        for (const auto& [neighborFaceID, continuityType, dihedralAngle] : feature->adjacencyList)
        {
            if (continuityType == ContinuityType::C0)
            {
                auto it = featureMap.find(neighborFaceID);
                if (it != featureMap.end() && it->second->atomType == AtomType::CYLINDER)
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

        // c. Promote the plane into a composite step surface and capture its metrics
        CompositeFeature stepFeature;
        stepFeature.partID = partNode.partID;
        stepFeature.type = CompositeFeatureType::STEP_PLANE;
        stepFeature.compositeID = partNode.partID + "_StepPlane_" + std::to_string(partNode.compositeFeatures.size());
        stepFeature.planeNormal = feature->geometricParams.axisVector;
        stepFeature.planeLocation = feature->geometricParams.locationPoint;
        stepFeature.planeArea = feature->area;
        stepFeature.childAtomicFeatureIDs.push_back(feature->faceID);

        // d. Consume the plane and store the composite
        feature->isConsumed = true;
        partNode.compositeFeatures.push_back(stepFeature);
        return true;
    }

    bool ASGBuilder::RecognizeFunctionalPlaneFeature(
        PartNode& partNode,
        const std::shared_ptr<AtomicFeature>& feature,
        FeatureMap& featureMap)
    {
        // a. Only neutral, functional, and yet-to-be-consumed planes qualify
        if (feature->atomType != AtomType::PLANE || feature->formType != FormType::NEUTRAL)
        {
            return false;
        }
        if (!feature->isFunctional || feature->isConsumed)
        {
            return false;
        }

        // b. Cultivate a composite functional plane with stored orientation and area
        CompositeFeature planeFeature;
        planeFeature.partID = partNode.partID;
        planeFeature.type = CompositeFeatureType::FUNCTIONAL_PLANE;
        planeFeature.compositeID = partNode.partID + "_FuncPlane_" + std::to_string(partNode.compositeFeatures.size());
        planeFeature.planeNormal = feature->geometricParams.axisVector;
        planeFeature.planeLocation = feature->geometricParams.locationPoint;
        planeFeature.planeArea = feature->area;
        planeFeature.childAtomicFeatureIDs.push_back(feature->faceID);

        // c. Mark the plane so it is not reused and persist the composite
        feature->isConsumed = true;
        partNode.compositeFeatures.push_back(planeFeature);
        return true;
    }

    // ============================================================================
    // STEP 4: Assembly Constraint Recognition Utilities
    // ============================================================================

    gp_Pnt ASGBuilder::TransformPoint(const gp_Pnt& localPnt, const gp_Trsf& trsf)
    {
        // a. Apply the supplied transformation matrix to relocate the point into world coordinates
        return localPnt.Transformed(trsf);
    }

    gp_Dir ASGBuilder::TransformDir(const gp_Dir& localDir, const gp_Trsf& trsf)
    {
        // a. Transform the direction while ignoring the translational portion of the matrix
        return localDir.Transformed(trsf);
    }

    gp_Ax1 ASGBuilder::TransformAxis(const gp_Ax1& localAxis, const gp_Trsf& trsf)
    {
        // a. Transform both the axis base point and its direction vector
        const gp_Pnt transformedLoc = localAxis.Location().Transformed(trsf);
        const gp_Dir transformedDir = localAxis.Direction().Transformed(trsf);
        // b. Reconstruct an axis object in the target coordinate frame
        return {transformedLoc, transformedDir};
    }

    bool ASGBuilder::CheckBoundingBoxCollision(const PartNode& nodeA, const PartNode& nodeB)
    {
        // a. Use world-space bounding boxes to perform a broad-phase overlap test
        return !nodeA.worldBoundingBox.IsOut(nodeB.worldBoundingBox);
    }

    void ASGBuilder::MatchPartPair(const PartNode& nodeA, const PartNode& nodeB)
    {
        // a. Iterate through every composite feature pair drawn from both parts
        for (const auto& featA : nodeA.compositeFeatures)
        {
            for (const auto& featB : nodeB.compositeFeatures)
            {
                // b. Evaluate coaxial compatibility for cylindrical features (holes or shafts)
                if ((featA.type == CompositeFeatureType::HOLE || featA.type == CompositeFeatureType::SHAFT) &&
                    (featB.type == CompositeFeatureType::HOLE || featB.type == CompositeFeatureType::SHAFT))
                {
                    MatchCoaxial(nodeA, featA, nodeB, featB);
                }

                // c. Evaluate planar coincidence for functional or step planes
                if ((featA.type == CompositeFeatureType::FUNCTIONAL_PLANE || featA.type == CompositeFeatureType::STEP_PLANE) &&
                    (featB.type == CompositeFeatureType::FUNCTIONAL_PLANE || featB.type == CompositeFeatureType::STEP_PLANE))
                {
                    MatchCoincident(nodeA, featA, nodeB, featB);
                }
            }
        }
    }

    void ASGBuilder::MatchCoaxial(const PartNode& nodeA, const CompositeFeature& featA,
                                  const PartNode& nodeB, const CompositeFeature& featB)
    {
        // a. Transform both local feature axes into the shared world coordinate system
        const gp_Ax1 axisA_world = TransformAxis(featA.axis, nodeA.transformation);
        const gp_Ax1 axisB_world = TransformAxis(featB.axis, nodeB.transformation);

        // b. Reject pairs whose axis directions are not parallel within angular tolerance
        if (!axisA_world.Direction().IsParallel(axisB_world.Direction(), Constants::AngleTolerance))
        {
            return;
        }

        // c. Measure the shortest distance between the two axes to ensure co-linearity
        const gp_Pnt locA = axisA_world.Location();
        const gp_Pnt locB = axisB_world.Location();
        const gp_Vec AB(locA, locB);
        const gp_Vec axisVec(axisA_world.Direction());
        const double distance = AB.Crossed(axisVec).Magnitude();
        if (distance > Constants::DistanceTolerance)
        {
            return;
        }

        // d. Compare radii and allow only pairs within coaxial tolerance
        const double radiusA = featA.nominalRadius;
        const double radiusB = featB.nominalRadius;
        const double radiusDiff = std::abs(radiusA - radiusB);
        if (radiusDiff > Constants::CoaxialRadiusDiffTolerance)
        {
            return;
        }

        // e. Populate the constraint record and append it to the global list
        AssemblyConstraint constraint;
        constraint.type = ConstraintType::COAXIAL;
        constraint.partID_A = nodeA.partID;
        constraint.featureID_A = featA.compositeID;
        constraint.partID_B = nodeB.partID;
        constraint.featureID_B = featB.compositeID;
        constraint.value = radiusDiff;
        constraints_.push_back(constraint);
    }

    void ASGBuilder::MatchCoincident(const PartNode& nodeA, const CompositeFeature& featA,
                                     const PartNode& nodeB, const CompositeFeature& featB)
    {
        // a. Transform plane normals and origins into world coordinates
        const gp_Dir normalA_world = TransformDir(featA.planeNormal, nodeA.transformation);
        const gp_Dir normalB_world = TransformDir(featB.planeNormal, nodeB.transformation);
        const gp_Pnt locA_world = TransformPoint(featA.planeLocation, nodeA.transformation);
        const gp_Pnt locB_world = TransformPoint(featB.planeLocation, nodeB.transformation);

        // b. Ensure the planes face one another (normals nearly opposite)
        const double dotProduct = normalA_world.Dot(normalB_world);
        if (dotProduct > -0.99)
        {
            return;
        }

        // c. Compute signed distance between planes along the normal direction
        const gp_Vec AB(locA_world, locB_world);
        const double distance = std::abs(AB.Dot(gp_Vec(normalA_world)));
        if (distance > Constants::DistanceTolerance)
        {
            return;
        }

        // d. Capture the coincident constraint and persist it
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
    // STEP 5: Output and Diagnostics
    // ============================================================================

    bool ASGBuilder::ExportToJSON(const std::string& filePath) const
    {
        // a. Open the output stream and report failures early
        std::ofstream outFile(filePath);
        if (!outFile.is_open())
        {
            std::cerr << "Error: Cannot open file for writing: " << filePath << std::endl;
            return false;
        }

        // b. Emit assembly-level JSON scaffolding
        outFile << "{\n";
        outFile << "  \"assembly\": {\n";
        outFile << "    \"parts\": [\n";

        // c. Serialize each part and its composite features
        for (size_t i = 0; i < partNodes_.size(); ++i)
        {
            const auto& part = partNodes_[i];
            outFile << "      {\n";
            outFile << R"(        \"partID\": \")" << JSONUtils::EscapeJSONString(part.partID) << "\",\n";
            outFile << "        \"compositeFeatures\": [\n";

            for (size_t j = 0; j < part.compositeFeatures.size(); ++j)
            {
                const auto& feat = part.compositeFeatures[j];
                outFile << "          {\n";
                outFile << R"(            \"featureID\": \")" << JSONUtils::EscapeJSONString(feat.compositeID) << "\",\n";
                outFile << R"(            \"type\": \")";

                switch (feat.type)
                {
                case CompositeFeatureType::HOLE: outFile << "HOLE"; break;
                case CompositeFeatureType::SHAFT: outFile << "SHAFT"; break;
                case CompositeFeatureType::FUNCTIONAL_PLANE: outFile << "FUNCTIONAL_PLANE"; break;
                case CompositeFeatureType::STEP_PLANE: outFile << "STEP_PLANE"; break;
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

        // d. Close the file and signal success
        outFile.close();
        return true;
    }

    void ASGBuilder::PrintStatistics() const
    {
        // a. Aggregate the total number of composite features across all parts
        int totalCompositeFeatures = 0;
        for (const auto& part : partNodes_)
        {
            totalCompositeFeatures += static_cast<int>(part.compositeFeatures.size());
        }

        // b. Emit summary statistics to the console for quick diagnostics
        std::cout << "========================================" << std::endl;
        std::cout << "Assembly Statistics:" << std::endl;
        std::cout << "  Total Parts: " << partNodes_.size() << std::endl;
        std::cout << "  Total Composite Features: " << totalCompositeFeatures << std::endl;
        std::cout << "========================================" << std::endl;
    }

   UVGridData ASGBuilder::ComputeFaceUVGrid(const TopoDS_Face& face, const std::string& faceID, int resolution)
    {
        UVGridData data;
        data.faceID = faceID;
        data.resolution = resolution;
        data.channels = 6;
        data.flattenedData.resize(resolution * resolution * 6, 0.0);

        // 1. 准备适配器
        BRepAdaptor_Surface adaptor(face);
        double uMin = adaptor.FirstUParameter();
        double uMax = adaptor.LastUParameter();
        double vMin = adaptor.FirstVParameter();
        double vMax = adaptor.LastVParameter();

        // 2. 准备分类器 (Mask 计算)
        BRepTopAdaptor_FClass2d classifier(face, 1e-7);

        // 3. 准备局部属性计算器 (法向/曲率)
        BRepLProp_SLProps props(adaptor, 2, 1e-7);

        // 4. 网格遍历
        int idx = 0;
        for (int i = 0; i < resolution; ++i)
        {
            double u = uMin + static_cast<double>(i) / (resolution - 1) * (uMax - uMin);

            for (int j = 0; j < resolution; ++j)
            {
                double v = vMin + static_cast<double>(j) / (resolution - 1) * (vMax - vMin);

                // --- A. 计算 Mask (裁剪掩码) ---
                gp_Pnt2d uvPoint(u, v);

                // 执行分类
                TopAbs_State state = classifier.Perform(uvPoint);

                // [FIXED] TopAbs_IN 和 TopAbs_ON 需要包含 <TopAbs.hxx>
                bool isInside = state == TopAbs_IN || state == TopAbs_ON;
                double maskVal = isInside ? 1.0 : 0.0;

                // --- B. 计算几何属性 ---
                double nx = 0.0, ny = 0.0, nz = 0.0;
                double gaussCurv = 0.0, meanCurv = 0.0;

                if (isInside)
                {
                    props.SetParameters(u, v);

                    if (props.IsNormalDefined())
                    {
                        gp_Dir normal = props.Normal();
                        if (face.Orientation() == TopAbs_REVERSED)
                        {
                            normal.Reverse();
                        }
                        nx = normal.X();
                        ny = normal.Y();
                        nz = normal.Z();
                    }

                    if (props.IsCurvatureDefined())
                    {
                        // 使用 std::clamp 限制数值范围
                        gaussCurv = std::clamp(props.GaussianCurvature(), -100.0, 100.0);
                        meanCurv = std::clamp(props.MeanCurvature(), -100.0, 100.0);
                    }
                }

                // --- C. 填充数据 ---
                // Layout: [Nx, Ny, Nz, K, H, Mask]
                data.flattenedData[idx++] = nx;
                data.flattenedData[idx++] = ny;
                data.flattenedData[idx++] = nz;
                data.flattenedData[idx++] = gaussCurv;
                data.flattenedData[idx++] = meanCurv;
                data.flattenedData[idx++] = maskVal;
            }
        }

        return data;
    }




} // namespace ASG
