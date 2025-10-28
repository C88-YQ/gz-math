#ifndef GZ_MATH_PYTHON_FOURWIDSDRIVEODOMETRY_HH
#define GZ_MATH_PYTHON_FOURWIDSDRIVEODOMETRY_HH

#include <pybind11/chrono.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include <string>
namespace py = pybind11;

namespace gz
{
namespace math
{
namespace python
{
/// Define a pybind11 wrapper for an gz::math::FourwidsDriveOdometry
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
void defineMathFourwidsDriveOdometry(py::module &m, const std::string &typestr);
} // namespace python
} // namespace math
} // namespace gz

#endif // GZ_MATH_PYTHON_FOURWIDSDRIVEODOMETRY_HH