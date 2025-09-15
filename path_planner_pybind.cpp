#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include "/home/daniel/path_planner/src/path_planner.h"

namespace py = pybind11;

PYBIND11_MODULE(path_planner, m) {
    py::class_<PathPlannerGraph>(m, "PathPlannerGraph")
        .def(py::init<std::tuple<double, double>, 
                      std::tuple<double, double>, 
                      std::vector<std::tuple<double, double, double>>,
                      int, 
                      double>(),
             py::arg("start"),
             py::arg("goal"),
             py::arg("obstacles"),
             py::arg("points_per_obstacle") = 8,
             py::arg("ellipse_factor") = 1.1)
        .def("generate_graph", &PathPlannerGraph::generate_graph)
        .def("get_graph", &PathPlannerGraph::get_graph);
}