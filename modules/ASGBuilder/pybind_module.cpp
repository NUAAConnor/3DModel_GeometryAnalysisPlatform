//
// Created by zhuge on 2025/12/1.
//

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "ASGBuilder.h"

namespace py = pybind11;
using namespace ASG;


py::dict ConvertGraphDataToDict(const DeepLearningGraphData& data)
{
    py::dict d;
    d["node_types"] = data.nodeTypes;
    d["node_areas"] = data.nodeAreas;
    d["node_curvatures"] = data.nodeCurvatures;
    d["node_centroids"] = data.nodeCentroids;
    d["node_label_types"] = data.nodeLabelTypes;
    d["node_label_indices"] = data.nodeLabelIndices;
    d["edge_source"] = data.edgeSource;
    d["edge_target"] = data.edgeTarget;
    d["edge_angles"] = data.edgeAngles;
    d["edge_continuity"] = data.edgeContinuity;
    return d;
}

PYBIND11_MODULE(ASGCore, m)
{
    m.doc() = "ASGBuilder Python Interface";

    py::enum_<AtomType>(m, "AtomType")
        .value("PLANE", AtomType::PLANE).value("CYLINDER", AtomType::CYLINDER)
        .value("CONE", AtomType::CONE).value("SPHERE", AtomType::SPHERE)
        .value("TORUS", AtomType::TORUS).value("BSPLINE", AtomType::BSPLINE)
        .value("OTHER", AtomType::OTHER).export_values();

    py::enum_<ConstraintType>(m, "ConstraintType")
        .value("COAXIAL", ConstraintType::COAXIAL)
        .value("COINCIDENT", ConstraintType::COINCIDENT)
        .value("OFFSET", ConstraintType::OFFSET).export_values();

    py::enum_<CompositeFeatureType>(m, "FeatureType")
        .value("UNKNOWN", CompositeFeatureType::UNKNOWN)
        .value("HOLE", CompositeFeatureType::HOLE)
        .value("SHAFT", CompositeFeatureType::SHAFT)
        .value("FUNCTIONAL_PLANE", CompositeFeatureType::FUNCTIONAL_PLANE)
        .value("STEP_PLANE", CompositeFeatureType::STEP_PLANE).export_values();

    py::class_<AssemblyConstraint>(m, "AssemblyConstraint")
        .def_readonly("type", &AssemblyConstraint::type)
        .def_readonly("part_id_a", &AssemblyConstraint::partID_A)
        .def_readonly("feature_id_a", &AssemblyConstraint::featureID_A)
        .def_readonly("part_id_b", &AssemblyConstraint::partID_B)
        .def_readonly("feature_id_b", &AssemblyConstraint::featureID_B)
        .def_readonly("value", &AssemblyConstraint::value)
        .def("__repr__", &AssemblyConstraint::ToString);

    py::class_<UVGridData>(m, "UVGridData")
        .def_readonly("face_id", &UVGridData::faceID)
        .def_readonly("resolution", &UVGridData::resolution)
        .def_readonly("channels", &UVGridData::channels)
        .def_readonly("data", &UVGridData::flattenedData,
            "Flattened array of size resolution*resolution*channels. "
            "Layout: pixel0[nx,ny,nz,K,H,mask], pixel1[...]");

    py::class_<ASGBuilder>(m, "ASGBuilder")
        .def(py::init<>())
        .def("load_step_file", &ASGBuilder::LoadAssemblyFromSTEP)
        .def("parse_assembly", &ASGBuilder::ParseAssemblyTree)
        .def("classify_features", &ASGBuilder::ClassifyAtomicFeatures)
        .def("recognize_composites", &ASGBuilder::RecognizeCompositeFeatures)
        .def("build_constraint_graph", &ASGBuilder::BuildAssemblyConstraintGraph)
        .def("export_json", &ASGBuilder::ExportToJSON)
        .def("get_all_part_ids", &ASGBuilder::GetAllPartIDs)
        .def("get_graph_data", [](const ASGBuilder& self, const std::string& partID)
        {
            return ConvertGraphDataToDict(self.GetGraphDataForPart(partID));
        })
        .def("get_constraints", &ASGBuilder::GetAssemblyConstraints)
        .def("check_material_between", &ASGBuilder::CheckMaterialBetween,
             "Check if material exists between two points (World Coords) for a given Part ID",
             py::arg("part_id"), py::arg("p1"), py::arg("p2"))
        .def("get_part_uv_grids", &ASGBuilder::GetPartUVGrids,
             "Compute UV-Grid features (Normals, Curvature, Mask) for all faces in a part",
             py::arg("part_id"));
}
