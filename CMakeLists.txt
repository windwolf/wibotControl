cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)

process_src_dir(${CMAKE_CURRENT_LIST_DIR}/basic ${PROJECT_NAME})
process_src_dir(${CMAKE_CURRENT_LIST_DIR}/kf ${PROJECT_NAME})
process_src_dir(${CMAKE_CURRENT_LIST_DIR}/pid ${PROJECT_NAME})
process_src_dir(${CMAKE_CURRENT_LIST_DIR}/foc ${PROJECT_NAME})
process_src_dir(${CMAKE_CURRENT_LIST_DIR}/filter ${PROJECT_NAME})

if (${ENABLE_TEST})
    process_src_dir(${CMAKE_CURRENT_LIST_DIR}/test ${PROJECT_NAME})
endif ()
