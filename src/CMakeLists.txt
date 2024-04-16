project(smooth_main)
set(CMAKE_CXX_STANDARD 17)

find_package(Python3 COMPONENTS Development NumPy)
find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${HEAD_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${OSQP_EIGEN_INCLUDE_DIRS}
    /usr/include/python3.8
    )

add_executable(main main.cpp)

target_link_libraries(main Python3::Python Python3::NumPy OsqpEigen::OsqpEigen)
