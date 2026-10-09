#include "marker/reconoce.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
//Para obtener los puntos y el marcador
#include <stdio.h>
#include <vector>
#include <string>
#include <tuple>
#include <onnxruntime_cxx_api.h>
#include "marker/reconoce.h"
#include "homography/homografia.h"

#include <iterator>
#include <list>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <fstream>


using namespace cv;
using namespace std;

// ---- YOLO ----
struct YoloBBox { float x1, y1, x2, y2, confidence; bool valid; };
Ort::Session& yolo_session();
std::vector<YoloBBox> inferirYOLO(cv::Mat& frame, Ort::Session& session);
// proesamiento
int numero_marcador(int num_comp[2]);
bool search_marker_in_frame(Mat& IMG1, Mat& IMG, uint8_t um, int tiempo, const std::vector<YoloBBox>& candidatos);
int obtener_caracteristicas(Mat& IMG1, Mat& IMG, uint8_t um, int tiempo);
void cambia_coordenadasAumentadas(unsigned char buffer[55], int i, float coordenada);
void cambia_DatoAumentado(unsigned char buffer[55], int i, int ancho);
void enviar_caracteristicasTrans(int num_marcadores, int sock);
//Computadora
void recibir_caracteristicas(int num_marcadores, int sock);
float cambia_coordenadasRecibidas(int DivEnt, int ResEnt, int DivFlt, int ResFlt);
int cambia_DatoRecibido(int DivEnt, int ResEnt);
void imprimir_lista();
void imprimir_lista_leida();
void guardar_lista();
void abrir_lista();
//Opcionales
struct MarcadorHomografia {
  int id = 0;                         // modelo 1..4
  cv::Matx33d R;                      // rotación 3x3
  cv::Vec3d t;                        // traslación 3x1
  std::array<cv::Point2f, 6> puntos;  // los 6 puntos en el orden del modelo
};

std::vector<MarcadorHomografia> obtener_homografias(Mat& IMG1, Mat& IMG, uint8_t um,
                                                    Ort::Session& session, bool verbose = true);

