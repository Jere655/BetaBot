#include "Base.h"

// --- Paramètres du moteur d'intake unique (levage + rouleau) ---
constexpr double kHauteurHaut = 10.0;  // tours d'encodeur = position haute (tir)
constexpr double kTolPos      = 0.5;   // tolérance (tours) pour "à terre" / "à hauteur"
constexpr double kVitesseLift = 0.5;   // vitesse de levage progressive (0.0 - 1.0)

void Base::RobotInit() {
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_leftMotor);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_rightMotor);
    m_rightMotor.SetInverted(true);
    m_compressor.EnableDigital();

    // Curseurs SmartDashboard — valeurs par défaut
    frc::SmartDashboard::SetDefaultNumber("Vitesse Base (%)",   48.0);
    frc::SmartDashboard::SetDefaultNumber("Vitesse Intake (%)", 45.0);

    // Initialiser la position de l'encodeur du intake à 0
    m_intakeEncoder.SetPosition(0.0);
}

void Base::TeleopPeriodic() {
    // Lire les curseurs (0 à 100 → 0.0 à 1.0)
    double limiteBase   = frc::SmartDashboard::GetNumber("Vitesse Base (%)",   48.0) / 100.0;
    double limiteIntake = frc::SmartDashboard::GetNumber("Vitesse Intake (%)", 45.0) / 100.0;

    // Sécurité : limiter entre 0.0 et 1.0
    limiteBase   = std::clamp(limiteBase,   0.0, 1.0);
    limiteIntake = std::clamp(limiteIntake, 0.0, 1.0);

    // --- Base de propulsion ---
    double vitesse  = m_driverController.GetLeftY()  * limiteBase;
    double rotation = m_driverController.GetRightX() * limiteBase;

    if (std::abs(vitesse)  < 0.1)  vitesse  = 0;
    if (std::abs(rotation) < 0.15) rotation = 0;

    m_robotDrive.ArcadeDrive(vitesse, rotation);

    // --- Moteur d'intake unique : levage progressif + rouleau ---
    double pos = m_intakeEncoder.GetPosition();
    double rt = m_driverController.GetRightTriggerAxis();
    double lt = m_driverController.GetLeftTriggerAxis();

    bool aTerre   = pos <= kTolPos;                             // intake au sol (ramassage)
    bool aHauteur = pos >= (kHauteurHaut - kTolPos);            // intake en hauteur (tir)

    // Détection des appuis (front montant) sur A (monter) / B (descendre)
    bool aAppui = m_driverController.GetAButton() && !m_aPrecedent;
    bool bAppui = m_driverController.GetBButton() && !m_bPrecedent;
    m_aPrecedent = m_driverController.GetAButton();
    m_bPrecedent = m_driverController.GetBButton();

    // 1 appui = déclenche le mouvement automatique (sans maintenir)
    if (aAppui && !aHauteur) m_etatIntake = 1;   // monter
    if (bAppui && !aTerre)   m_etatIntake = 2;   // descendre

    // Le moteur fait SOIT le levage progressif, SOIT le rouleau (jamais en même temps)
    double cmd = 0.0;
    if (m_etatIntake == 1) {
        if (aHauteur) m_etatIntake = 0;            // arrivé en haut -> stop
        else cmd = kVitesseLift;                   // monte progressivement
    } else if (m_etatIntake == 2) {
        if (aTerre) m_etatIntake = 0;              // arrivé en bas -> stop
        else cmd = -kVitesseLift;                  // descend progressivement
    } else {
        // Repos : rouleau aux positions extrêmes
        if (rt > 0.1 && aHauteur) cmd = rt * limiteIntake;   // tire en haut
        else if (lt > 0.1 && aTerre) cmd = lt * limiteIntake; // ramasse en bas
    }
    m_intakeMotor.Set(cmd);

    // --- Déploiement pneumatique (toggle X) ---
    bool xAppuye = m_driverController.GetXButton();
    if (xAppuye && !m_xPrecedent) {
        m_intakeLeve = !m_intakeLeve;
    }
    m_xPrecedent = xAppuye;
    m_intake.Set(m_intakeLeve ? frc::DoubleSolenoid::Value::kForward
                              : frc::DoubleSolenoid::Value::kReverse);

    // Affichage des valeurs actives sur le dashboard
    frc::SmartDashboard::PutNumber("Vitesse Base active (%)",   limiteBase   * 100.0);
    frc::SmartDashboard::PutNumber("Vitesse Intake active (%)", limiteIntake * 100.0);
    frc::SmartDashboard::PutNumber("Intake Position (rot)",     pos);
    frc::SmartDashboard::PutBoolean("Intake A terre",           aTerre);
    frc::SmartDashboard::PutBoolean("Intake A hauteur",         aHauteur);
    frc::SmartDashboard::PutBoolean("Intake Deploye",           m_intakeLeve);
}