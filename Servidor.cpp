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

        // ---------- Detección  ----------
        std::vector<MarcadorHomografia> detectados =	
        obtener_homografias(IMG_marcas, grayFull, 70, yolo_session()); 

        
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
        cv::imshow("Deteccion", IMG_marcas);          //ver candidatos YOLO y marcadores detectados 

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
