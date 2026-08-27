#include "Vision.h"

Vision::Vision()
    : m_camera("usbcam", 0),
      m_cvSink(m_camera.GetVideo()),
      m_output(frc::CameraServer::PutVideo("Vision", 320, 240)) {
    m_camera.SetResolution(320, 240);
    m_camera.SetFPS(20);
}

Vision::~Vision() {
    Stop();
}

void Vision::Start() {
    if (m_running) return;
    m_running = true;
    m_thread = std::thread(&Vision::Run, this);
}

void Vision::Stop() {
    m_running = false;
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void Vision::Run() {
    cv::Mat frame, hsv, mask;
    cv::Scalar lower, upper;

    while (m_running) {
        if (m_cvSink.GrabFrame(frame) == 0) {
            continue;  // pas de frame disponible
        }

        // Plages HSV réglables depuis le SmartDashboard (ex. objet jaune/vert)
        int hMin = (int)frc::SmartDashboard::GetNumber("Vision H min", 50);
        int hMax = (int)frc::SmartDashboard::GetNumber("Vision H max", 90);
        int sMin = (int)frc::SmartDashboard::GetNumber("Vision S min", 100);
        int sMax = (int)frc::SmartDashboard::GetNumber("Vision S max", 255);
        int vMin = (int)frc::SmartDashboard::GetNumber("Vision V min", 100);
        int vMax = (int)frc::SmartDashboard::GetNumber("Vision V max", 255);
        lower = cv::Scalar(hMin, sMin, vMin);
        upper = cv::Scalar(hMax, sMax, vMax);

        // --- Détection d'objets ---
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, lower, upper, mask);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        int meilleur = -1;
        double meilleurAire = 0.0;
        for (size_t i = 0; i < contours.size(); i++) {
            double aire = cv::contourArea(contours[i]);
            if (aire > meilleurAire) {
                meilleurAire = aire;
                meilleur = static_cast<int>(i);
            }
        }

        double cibleX = -1.0;
        double cibleY = -1.0;
        if (meilleur >= 0 && meilleurAire > 50.0) {
            cv::Moments m = cv::moments(contours[meilleur]);
            cibleX = m.m10 / m.m00;
            cibleY = m.m01 / m.m00;
            cv::drawContours(frame, contours, meilleur, cv::Scalar(0, 255, 0), 2);
            cv::circle(frame, cv::Point(static_cast<int>(cibleX), static_cast<int>(cibleY)),
                       6, cv::Scalar(0, 0, 255), -1);
        }

        frc::SmartDashboard::PutNumber("Vision Nb objets", static_cast<double>(contours.size()));
        frc::SmartDashboard::PutNumber("Vision Cible X", cibleX);
        frc::SmartDashboard::PutNumber("Vision Cible Y", cibleY);
        frc::SmartDashboard::PutNumber("Vision Aire", meilleurAire);

        m_output.PutFrame(frame);
    }
}
