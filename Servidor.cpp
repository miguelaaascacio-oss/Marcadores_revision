#include <onnxruntime_cxx_api.h>
#include <opencv2/dnn.hpp> //necesario para dnn filtro de los candidatos similares
#include "procesa/procesa.h"
#include "marker/reconoce.h" 
#include <opencv2/opencv.hpp>
#include <iostream>
#include <atomic>
#include <algorithm>
#include <limits>
#include <vector>
#include <stdio.h>
#include <unistd.h>

std::mutex frame_mutex;
std::atomic<bool> running(true);
std::atomic<bool> ready_enviado(false);
std::atomic<bool> fueraZona(false);
std::atomic<bool> EnRuta(false);

cv::VideoCapture* cap = nullptr;
cv::Mat frame;


//* --------- ESTRUCTURA DE DATOS --------- /
struct DatosMarcador {
  int id;
  bool detectado;
  int etiqueta = 0;    
  std::vector<cv::Point2f> puntos;
  cv::Matx33d R;
  cv::Vec3d t;
  cv::Vec3d C;  
  double norma;
};

// Inversion de eje y para el marco y el triangulo
constexpr float IMG_H = 480.0f;
inline cv::Point2f invertirY(const cv::Point2f& p) {
    return cv::Point2f(p.x, (IMG_H - 1.0f) - p.y);
}

Ort::Session& yolo_session() {
    static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "yolo");
    static Ort::Session s(env, "../best.onnx", Ort::SessionOptions{});
    return s;
}

// --- YOLO ---
  
  //sesión de ONNX Runtime con el modelo ya cargado, imagen de la cámara 640x480, en formato BGR opencv

  std::vector<YoloBBox> inferirYOLO(cv::Mat& frame, Ort::Session& session) {
    cv::Mat IMG_yolo;
    cv::resize(frame, IMG_yolo, cv::Size(640, 640)); //rediimensionamos a 640x640 (estandar yolo)
    cv::Mat rgb;
    cv::cvtColor(IMG_yolo, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(rgb, CV_32F, 1.0f / 255.0f); //normalizamos 

// ------------- conversion de HWC a CHW ----------------------
    
    //OpenCV guarda la imagen entrelazada (HWC): los tres canales de cada píxel van juntos.
    //HWC (OpenCV):  R0 G0 B0 | R1 G1 B1 | R2 G2 B2 |
    
    //El modelo espera planar (CHW): primero todos los R, luego todos los G, luego todos los B.
    //CHW (ONNX):    R0 R1 R2 ... | G0 G1 G2 ... | B0 B1 B2

    std::vector<cv::Mat> ch(3);
    cv::split(rgb, ch); //separamos la imagen en 3 canales RGB 
    std::vector<float> input_data(3 * 640 * 640); 
    
    // Aplanamiento de canales
    // R desde el elemento 0, G desde el elemento 409,600, B desde el elemento 819,200 total 1,228,800 floats
    for (int c = 0; c < 3; c++)
        memcpy(input_data.data() + c * 640 * 640, ch[c].data, 640 * 640 * sizeof(float));

    std::array<int64_t, 4> shape = {1, 3, 640, 640}; //tensor de entrada 1 imagen 3 canales 640x640
    auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault); //aqui viven los datos 
    
    //se crea tensor de entrada contiene:
    //mem, el puntero a los datos, el número total de elementos, el puntero a la forma y cuántas dimensiones tiene
    auto tensor = Ort::Value::CreateTensor<float>(mem, input_data.data(), input_data.size(), shape.data(), shape.size());

    const char* input_names[]  = {"images"}; //Los nombres de las capas de entrada y salida de ONNX
    const char* output_names[] = {"output0"};//Los nombres de las capas de entrada y salida de ONNX
    
    auto outputs = session.Run(Ort::RunOptions{nullptr}, input_names, &tensor, 1, output_names, 1); //se realiza inferencia

    float* data = outputs[0].GetTensorMutableData<float>(); //Puntero al inicio del buffer de la salida
    int num_det = outputs[0].GetTensorTypeAndShapeInfo().GetShape()[1]; //informacion vector [imagen, detecciones, salidas]
    // data: [ cx0 cy0 w0 h0 conf0 clase0 | cx1 cy1 w1 h1 conf1 clase1 | solo hay una clase, un marcador

    std::vector<YoloBBox> candidatos;
    for (int i = 0; i < num_det; i++) {
        float conf = data[i * 6 + 4];
        if (conf < 0.9f) continue;
        float cx = data[i * 6 + 0], cy = data[i * 6 + 1], w = data[i * 6 + 2], h = data[i * 6 + 3];
        
        // como la salida en un vetor plano encontramos las componentes de esta manera
	//índice:  0   1   2   3   4     5      6   7   8   9   10    11    ...
	//data: [ a   b   c   d  conf  clase | a   b   c   d  conf  clase | ... ]
	//	└──────── fila 0 ────────┘ └──────── fila 1 ────────┘
        
        
        //calculamos el bounding box recortando si se sale de los limites y reescalando el boundingbox de 
        // una imagen de 640 x 640 a 640 x 480

        const float sx = frame.cols / 640.0f;   // 640x480 -> 1.0
        const float sy = frame.rows / 640.0f;   // 640x480 -> 0.75
        
        YoloBBox box;
        box.x1 = std::max(0.0f,   cx - (w * 1.2f) / 2) * sx;
	box.y1 = std::max(0.0f,   cy - (h * 1.2f) / 2) * sy;
	box.x2 = std::min(640.0f, cx + (w * 1.2f) / 2) * sx;
	box.y2 = std::min(640.0f, cy + (h * 1.2f) / 2) * sy;
        box.confidence = conf;
        box.valid = true;
        candidatos.push_back(box);
    }
    
    
    //return candidatos; //candidatos = [ box1, box2, ... ] que pasaron el umbral no tiene NMS                           (Non-Maximum Suppression y manda muchas cajas identicas usamos el de opencn dnn <opencv2/dnn.hpp>
    std::vector<cv::Rect> rects;
    std::vector<float> scores;
    
    for (const auto& c : candidatos) {
        rects.emplace_back(cv::Point(cvRound(c.x1), cvRound(c.y1)),
                       cv::Point(cvRound(c.x2), cvRound(c.y2)));
        scores.push_back(c.confidence);
    }

    std::vector<int> idx;
    cv::dnn::NMSBoxes(rects, scores, 0.9f , 0.5f, idx); // idx = posicion de candidato que sobreviven al NMS

    std::vector<YoloBBox> keep;
    for (int i : idx) keep.push_back(candidatos[i]);
        return keep;                 // aqui estan los candidatos no repetidos
   }


//* --------- PROYECCIÓN (2D HOMOGRAFÍA) --------- /

void proyectarMarcador(cv::Mat& img, const DatosMarcador& datos, std::atomic<bool>& zonaGlobal, std::atomic<bool>& enRuta) 
{
    if (!datos.detectado || datos.puntos.size() != 6) {
    return; 
    }

    constexpr float xmin = 30.0f;
    constexpr float xmax = 610.0f;

    constexpr float ymin = 30.0f;
    constexpr float ymax = 450.0f;
    
    constexpr float xmin2 = 200.0f;
    constexpr float xmax2 = 440.0f;

    constexpr float ymin2 = 120.0f;
    constexpr float ymax2 = 320.0f;
    
    //x ∈ [30 , 610]
    //y ∈ [30 , 450]

    bool esteFuera = false;
    bool marcadorEnAreaFoco = false;
        
    for (const auto& p : datos.puntos)
    {
       if (p.x < xmin || p.x > xmax ||
           p.y < ymin || p.y > ymax)
       {
           esteFuera = true;
       }
       if (p.x >= xmin2 && p.x <= xmax2 &&
           p.y >= ymin2 && p.y <= ymax2)
       {
           marcadorEnAreaFoco = true;
       }
 
    }
    
    if (marcadorEnAreaFoco)              
    	enRuta.store(true);
    if (marcadorEnAreaFoco && esteFuera) 
    	zonaGlobal.store(true);
    
    // Dibujar rectangulo de zona segura
	cv::rectangle(img,
		      cv::Point(30,30),
		      cv::Point(610,450),
		      zonaGlobal.load() ? cv::Scalar(0,0,255)
		                       : cv::Scalar(0,255,0),
		      2);
		      
    // Dibujar rectangulo de ruta
	cv::rectangle(img,
		      cv::Point(200,120),
		      cv::Point(440,320),
		      enRuta.load() ? cv::Scalar(0,255,0)
		                       : cv::Scalar(0,0,255),
		      2);

    std::vector<cv::Point2f> marcador = {
        {255, 0},       // P1 - Esquina superior derecha (Y invertido: 255-255=0)
        {0, 0},         // P2 - Esquina superior izquierda (Y invertido: 255-255=0)
        {90, 56},       // P3 - Punto medio izquierdo (Y invertido: 255-199=56)
        {165, 56},      // P4 - Punto medio derecho (Y invertido: 255-199=56)
        {0, 255},       // P5 - Esquina inferior izquierda (Y invertido: 255-0=255)
        {255, 255}      // P6 - Esquina inferior derecha (Y invertido: 255-0=255)
    };

    //std::vector<cv::Point2f> imagen = datos.puntos;
    std::vector<cv::Point2f> imagen = datos.puntos;
    cv::Mat H = cv::findHomography(marcador, imagen, 0);
    if (H.empty()) {
        return;  // Salir si no se pudo calcular la homografía
    }
    
    std::vector<cv::Point2f> contorno = {
        {255, 0},   // Esquina superior derecha
        {0, 0},     // Esquina superior izquierda
        {0, 255},   // Esquina inferior izquierda
        {255, 255}  // Esquina inferior derecha
    };

    std::vector<cv::Point2f> contorno_img;
    cv::perspectiveTransform(contorno, contorno_img, H);
    
    cv::Scalar colorContorno = enRuta.load() ? cv::Scalar(0, 255, 255) : 
                               (datos.id == 2 ? cv::Scalar(255, 0, 255) : cv::Scalar(0, 255, 0));

    for (int i = 0; i < 4; i++) {
    cv::Point2f p0 = invertirY(contorno_img[i]);
    cv::Point2f p1 = invertirY(contorno_img[(i+1)%4]);

    cv::line(img,
             p0,
             p1,
             colorContorno,
             2);
	}

    std::vector<cv::Point2f> tri = {
        {90, 56},           // Punto medio izquierdo
        {165, 56},          // Punto medio derecho
        {127.5f, 127.5f}    // Punto central superior (mitad de 255 = 127.5)
    };

    std::vector<cv::Point2f> tri_img;
    cv::perspectiveTransform(tri, tri_img, H);

	for (int i = 0; i < 3; i++) {
	    cv::Point2f p0 = invertirY(tri_img[i]);
	    cv::Point2f p1 = invertirY(tri_img[(i+1)%3]);

	    cv::line(img,
		     p0,
		     p1,
		     cv::Scalar(0,0,255),
		     2);
	}
	
	cv::Point2f centroide = invertirY(tri_img[2]); // Usamos la punta del triángulo como ancla
        cv::putText(img, "Marcador: " + std::to_string(datos.etiqueta), cv::Point(centroide.x - 20, centroide.y - 20),
        cv::FONT_HERSHEY_SIMPLEX, 0.7, colorContorno, 2);
}

//* ------------------ FUNCIÓN PRINCIPAL ------------------ /
int main(int /*argc*/, char** /*argv*/)
{

    //Abrir camara sin hilo
    
    cv::VideoCapture cap(2);
    int frame_id = 0;
    Ort::Session& session = yolo_session(); //inicia YOLO
    
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open the camera." << std::endl;
        return -1;
    }
    
    cv::Mat frame;
    std::cout << "Streaming camera... Press 'ESC' or 'q' to exit." << std::endl;

    while (running.load()) {
        cv::Mat frame_nativo;
        cap >> frame;
        
        if (frame.empty()) {
            std::cerr << "Error: Blank frame grabbed." << std::endl;
            break;
        }

        // ---------- Preprocesado ----------
        cv::Mat IMG, IMG_marcas, grayFull, frameProcesado;
        cv::resize(frame, IMG, cv::Size(640, 480));
        cv::cvtColor(IMG, grayFull, cv::COLOR_BGR2GRAY);
        IMG_marcas      = frame.clone();
        frameProcesado  = frame.clone();
        cv::Mat YoloVista = IMG.clone();

        // ---------- Detección  ----------
        // Aqui se llama a la red YOLO para detectar los marcadores
        std::vector<YoloBBox> cajas = inferirYOLO(IMG, session);  //se manda a llamar YOLO
         //cout << "YOLO cajas: " << candidatos.size() << endl; //ver las cajas encontradas arriba del umbral
      
        // Ver las caja de Yolo comentar todo el ciclo para ya no verlas 
        for (const auto& c : cajas) {
            cv::rectangle(YoloVista,
                cv::Point(cvRound(c.x1), cvRound(c.y1)),
                cv::Point(cvRound(c.x2), cvRound(c.y2)),
                cv::Scalar(0, 0, 255), 2);                    

            char txt[32];
            snprintf(txt, sizeof(txt), "%.2f", c.confidence);
            cv::putText(YoloVista, txt,
                cv::Point(cvRound(c.x1), std::max(15, cvRound(c.y1) - 5)),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
            }

        // para cada marcador detectado se llama a homografias en su caja contenedora
        std::vector<MarcadorHomografia> detectados;
        
        for (const auto& c : cajas) {
            std::array<int, 4> caja = { static_cast<int>(c.x1), static_cast<int>(c.y1),
                                        static_cast<int>(c.x2), static_cast<int>(c.y2) };
                                
            // c.x1 c.y1 y c.x2 c.y2 son las esquinas de la caja que contiene al marcador 
            
            std::vector<MarcadorHomografia> det =
                obtener_homografias(IMG_marcas, grayFull, caja, 70);

            detectados.insert(detectados.end(), det.begin(), det.end());
        }

        
        // ---------- Construcción de DatosMarcador ----------
        std::vector<DatosMarcador> marcadores_procesados;
        marcadores_procesados.reserve(detectados.size());

        for (const auto& m : detectados) {
            DatosMarcador d;
            d.id        = m.id;
            d.detectado = true;
            d.puntos.assign(m.puntos.begin(), m.puntos.end());
            
            d.R = m.R;
            d.t = m.t;
            //d.norma = cv::norm(m.t);
            
            //Camara centrado al marcador
            double tx = d.t[0];
            double ty = d.t[1];
            double tz = d.t[2];

            //Marcador centrado en la camara
            //d.C = -d.R.t() * d.t;
            
            //double cx = d.C[0];
            //double cy = d.C[1];
            //double cz = d.C[2];
            
            d.norma = std::sqrt((tx * tx) + (ty * ty) + (tz * tz));

            //d.Rt = cv::Mat::eye(4, 4, CV_64F);
            //for (int r = 0; r < 3; r++) {
              //  for (int c = 0; c < 3; c++) d.Rt.at<double>(r, c) = m.R(r, c);
                //d.Rt.at<double>(r, 3) = m.t[r];
            //}

            // Mover origen de la esquina (P4) al centro del marcador
            //cv::Mat T_local = (cv::Mat_<double>(4, 4) <<
                               // -1,  0, 0, 5,       //21.75 para marcador grande 5 para chico
                               // 0,  -1, 0, 5,
                               // 0,  0, 1, 0,
                               // 0,  0, 0, 1);
            //d.Rt = d.Rt * T_local;
            
            marcadores_procesados.push_back(d);
        }
        // ---------- Ordenación Y-X (primero el cercano al origen) ----------
        std::sort(marcadores_procesados.begin(), marcadores_procesados.end(),
            [](const DatosMarcador& a, const DatosMarcador& b) {
                auto prio = [](const std::vector<cv::Point2f>& pts) {
                    float min_y = std::numeric_limits<float>::max();
                    float min_x = std::numeric_limits<float>::max();
                    for (auto& p : pts) {
                        if (p.y < min_y) { min_y = p.y; min_x = p.x; }
                        else if (p.y == min_y && p.x < min_x) min_x = p.x;
                    }
                    return std::make_pair(min_y, min_x);
                };
                auto pa = prio(a.puntos), pb = prio(b.puntos);
                if (pa.first != pb.first) return pa.first > pb.first;
                return pa.second < pb.second;
            });

        // ---------- Iterar marcadores ----------
        int etiqueta_actual = 1;
        for (auto& mp : marcadores_procesados) {
            if (etiqueta_actual > 3) break; 
            mp.etiqueta = etiqueta_actual;

            proyectarMarcador(frameProcesado, mp, fueraZona, EnRuta);

            // >>> POSE <<<
            // t en cm -> m
            double tx = mp.t[0];
	    double ty = mp.t[1];
	    double tz = mp.t[2]; 
	    double norma = std::sqrt(tx*tx + ty*ty + tz*tz);

            // Texto AR
            int oy = (etiqueta_actual - 1) * 140;
            char buf[128];
            snprintf(buf, sizeof(buf), "E%d Profundidad: %.2f m", mp.etiqueta, -tz);
            cv::putText(frameProcesado, buf, {30, 30 + oy},
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, {0,255,255}, 2);
            snprintf(buf, sizeof(buf), "X:%.2f Y:%.2f", tx, ty);
            cv::putText(frameProcesado, buf, {30, 60 + oy},
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, {0,255,0}, 2);

            //printf("E%d -> x:%.2f y:%.2f z:%.2f | norma:%.2f\n",
                   //mp.etiqueta, tx, ty, tz, norma);

            etiqueta_actual++;
        }
        
        // ---------- Visualización ----------
        cv::imshow("Marcador Proyectado", frameProcesado);
        cv::imshow("Deteccion", IMG_marcas);   //Marcadores detectados de homografias
        cv::imshow("YOLO detecciones", YoloVista);   //detecciones YOLO

        char key = (char)cv::waitKey(1);
        if (key == 27 || key == 'q') running = false;

        frame_id++;
        usleep(30000); // ~33 FPS
    }

    running = false;
    cap.release();
    cv::destroyAllWindows();
    printf("\nPrograma finalizado.\n");
    return 0;
}
