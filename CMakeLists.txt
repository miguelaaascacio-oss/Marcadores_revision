cmake_minimum_required(VERSION 3.10)
project(Servidor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(THREADS_PREFER_PTHREAD_FLAG ON)

# ==================== RUTA DE ONNX RUNTIME ====================
set(ONNXRUNTIME_ROOT "/home/armas/onnxruntime-linux-x64-1.20.0")

if(NOT EXISTS ${ONNXRUNTIME_ROOT})
    message(FATAL_ERROR "No se encontró ONNX Runtime en: ${ONNXRUNTIME_ROOT}\n"
                        "Descárgalo y actualiza la ruta arriba.")
endif()

# ==================== Paquetes ====================
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

# ==================== Directorios ====================
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${ONNXRUNTIME_ROOT}/include)

link_directories(${ONNXRUNTIME_ROOT}/lib)

# ==================== Bibliotecas del proyecto ====================
add_library(triangles SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/marker/triangles.cc)
add_library(lambda SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/marker/lambdaMatrix.cc)
add_library(follow SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/marker/follow.cc)
add_library(reconoce SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/marker/reconoce.cc)
add_library(eigen SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/homography/eigen.cc)
add_library(homografia SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/homography/homografia.cc)
add_library(matrices SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/homography/matrices.cc)

add_library(procesa SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/procesa/procesa.cpp)
add_library(procesaRobot SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/procesa/procesaRobot.cpp)

# ==================== Ejecutable ====================
add_executable(Servidor Servidor.cpp)

# ==================== Enlazado ====================
target_link_libraries(Servidor
    Threads::Threads
    procesa
    procesaRobot
    ${OpenCV_LIBS}
    reconoce
    triangles
    lambda
    follow
    homografia
    eigen
    matrices
    onnxruntime
)

