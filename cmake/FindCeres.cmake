list(APPEND CERES_CHECK_INCLUDE_DIRS
        /usr/local/include
        /usr/local/homebrew/include
        /opt/local/var/macports/software
        /opt/local/include
        /usr/include)


list(APPEND CERES_CHECK_LIBRARY_DIRS
        /usr/local/lib
        /usr/local/homebrew/lib
        /opt/local/lib
        /usr/lib)

find_path(CERES_INCLUDE_DIRS
        NAMES
        ceres/ceres.h
        PATHS
        ${CERES_INCLUDE_DIR_HINTS}
        ${CERES_CHECK_INCLUDE_DIRS}
        )
find_library(CERES_LIBRARIES
        NAMES
        ceres
        libceres
        PATHS
        ${CERES_LIBRARY_DIR_HINTS}
        ${CERES_CHECK_LIBRARY_DIRS}
        )