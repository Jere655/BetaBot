#pragma once

#include <thread>
#include <vector>
#include <opencv2/opencv.hpp>
#include <cscore.h>
#include <frc/CameraServer.h>
#include <frc/smartdashboard/SmartDashboard.h>

// Pipeline de vision : capture la caméra USB du roboRIO et détecte des objets
// par seuillage de couleur (HSV) + contours (OpenCV).
//
// Note : le roboRIO ne peut pas faire tourner un vrai réseau de neurones (IA/ML)
// en temps réel. Cette détection classique (couleur/forme) fonctionne sur le
// roboRIO. Pour une vraie IA (réseau entraîné), utiliser un coprocesseur
// (Limelight, PhotonVision/Raspberry Pi) et lire ses résultats ici.
class Vision {
public:
    Vision();
    ~Vision();

    void Start();
    void Stop();

private:
    void Run();

    std::thread m_thread;
    bool m_running = false;

    cs::UsbCamera m_camera;
    cs::CvSink m_cvSink;
    cs::CvSource m_output;
};
