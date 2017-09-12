list(APPEND EIGEN_CHECK_INCLUDE_DIRS
        /usr/local/include/eigen3
        /usr/local/homebrew/include/eigen3
        /opt/local/var/macports/software/eigen3
        /opt/local/include/eigen3
        /usr/include/eigen3)
find_path(EIGEN_INCLUDE_DIR
        NAMES
        Eigen/Core
        PATHS
        ${EIGEN_INCLUDE_DIR}
        ${EIGEN_CHECK_INCLUDE_DIRS}
        )