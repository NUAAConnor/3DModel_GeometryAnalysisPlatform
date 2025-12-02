//
// Created by zhuge on 2025/12/1.
//

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "ASGBuilder.h"

namespace py = pybind11;
using namespace ASG;

// 辅助函数：将 C++ 图数据转为 Python 字典
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

PYBIND11_MODULE(ASGBuilder, m)
{
    m.doc() = "ASGBuilder Python Interface";

    // 1. 暴露枚举
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

    // 2. 暴露约束结构体
    py::class_<AssemblyConstraint>(m, "AssemblyConstraint")
        .def_readonly("type", &AssemblyConstraint::type)
        .def_readonly("part_id_a", &AssemblyConstraint::partID_A)
        .def_readonly("feature_id_a", &AssemblyConstraint::featureID_A)
        .def_readonly("part_id_b", &AssemblyConstraint::partID_B)
        .def_readonly("feature_id_b", &AssemblyConstraint::featureID_B)
        .def_readonly("value", &AssemblyConstraint::value)
        .def("__repr__", &AssemblyConstraint::ToString);

    // 3. 暴露主类
    py::class_<ASGBuilder>(m, "ASGBuilder")
        .def(py::init<>())
        .def("load_step_file", &ASGBuilder::LoadAssemblyFromSTEP)
        .def("parse_assembly", &ASGBuilder::ParseAssemblyTree)
        .def("classify_features", &ASGBuilder::ClassifyAtomicFeatures)
        .def("recognize_composites", &ASGBuilder::RecognizeCompositeFeatures)
        .def("build_constraint_graph", &ASGBuilder::BuildAssemblyConstraintGraph)
        .def("export_json", &ASGBuilder::ExportToJSON)

        // 核心 GNN 数据接口
        .def("get_graph_data", [](const ASGBuilder& self, const std::string& partID)
        {
            return ConvertGraphDataToDict(self.GetGraphDataForPart(partID));
        })
        // 核心 KG 构建接口
        .def("get_constraints", &ASGBuilder::GetAssemblyConstraints)

        // 物理验证接口
        .def_static("is_material_between", [](const py::object&, const py::object&)
        {
            // 暂时占位，未来需要处理 OCCT Shape 的 Python 传递
            return false;
        });
}
