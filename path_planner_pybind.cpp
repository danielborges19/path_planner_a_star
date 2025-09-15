#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include "/home/daniel/path_planner_a_star/src/path_planner.h"

namespace py = pybind11;

PYBIND11_MODULE(path_planner, m) {
    py::class_<PathPlannerGraph>(m, "PathPlannerGraph")
        .def(py::init<int, double>(),
             py::arg("points_per_obstacle") = 8,
             py::arg("ellipse_factor") = 1.1)
        .def("generateGraph", &PathPlannerGraph::generateGraph)
        .def("get_graph", &PathPlannerGraph::get_graph);
}