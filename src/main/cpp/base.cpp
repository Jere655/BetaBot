#include "Base.h"

// --- Paramètres du moteur d'intake unique (levage + rouleau) ---
constexpr double kHauteurHaut = 10.0;  // tours d'encodeur = position haute (tir)
constexpr double kTolPos      = 0.5;   // tolérance (tours) pour "à terre" / "à hauteur"
constexpr double kVitesseLift = 0.5;   // vitesse de levage progressive (0.0 - 1.0)

// --- Constantes autonomes ---
constexpr double kPi             = 3.14159265358979323846;
constexpr double kImageWidth     = 320.0;
constexpr double kImageCenterX   = kImageWidth / 2.0;
constexpr double kBallAreaMin    = 150.0;
constexpr double kBallAreaCollect = 500.0;
constexpr double kBallLostTimeout = 0.5;
constexpr double kIntakeRunTime   = 1.4;
constexpr double kSearchSpeed     = 0.35;
constexpr double kTurnSpeed       = 0.4;
constexpr double kReverseSpeed    = 0.25;
constexpr double kTrackWidth      = 0.7;
constexpr double kWheelDiameter   = 0.1524;
constexpr double kDriveGearRatio  = 1.0;
constexpr double kHeadingScale    = (2.0 * kPi * kWheelDiameter) / (kTrackWidth * kDriveGearRatio);

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

    // Curseurs de vision (plages HSV pour la détection d'objets)
    frc::SmartDashboard::SetDefaultNumber("Vision H min", 50.0);
    frc::SmartDashboard::SetDefaultNumber("Vision H max", 90.0);
    frc::SmartDashboard::SetDefaultNumber("Vision S min", 100.0);
    frc::SmartDashboard::SetDefaultNumber("Vision S max", 255.0);
    frc::SmartDashboard::SetDefaultNumber("Vision V min", 100.0);
    frc::SmartDashboard::SetDefaultNumber("Vision V max", 255.0);

    // Curseurs autonomes (réglables depuis le dashboard)
    frc::SmartDashboard::SetDefaultNumber("Auto Search Speed", kSearchSpeed);
    frc::SmartDashboard::SetDefaultNumber("Auto Turn Speed", kTurnSpeed);
    frc::SmartDashboard::SetDefaultNumber("Auto Reverse Speed", kReverseSpeed);
    frc::SmartDashboard::SetDefaultNumber("Auto Ball Area Min", kBallAreaMin);
    frc::SmartDashboard::SetDefaultNumber("Auto Ball Area Collect", kBallAreaCollect);
    frc::SmartDashboard::SetDefaultNumber("Auto Track Width (m)", kTrackWidth);
    frc::SmartDashboard::SetDefaultNumber("Auto Wheel Diameter (m)", kWheelDiameter);
    frc::SmartDashboard::SetDefaultNumber("Auto Gear Ratio", kDriveGearRatio);

    // Démarrer le pipeline de vision (caméra USB + détection)
    m_vision.Start();
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

    // Détection des appuis (front montant) sur A / B
    bool aAppui = m_driverController.GetAButton() && !m_aPrecedent;
    bool bAppui = m_driverController.GetBButton() && !m_bPrecedent;
    m_aPrecedent = m_driverController.GetAButton();
    m_bPrecedent = m_driverController.GetBButton();

    // --- DEBUG INTAKE ---
    bool xAppuye = m_driverController.GetXButton();
    if (xAppuye && !m_xPrecedent) {
        printf("[INTAKE DEBUG] X PRESSED\n");
        printf("[INTAKE DEBUG] X action: RETRACT (kReverse) -> intake monte/rentre\n");
        printf("[INTAKE DEBUG] solenoid module=3 channels=0/1 type=CTREPCM\n");
    }
    if (aAppui) {
        printf("[INTAKE DEBUG] A PRESSED\n");
        printf("[INTAKE DEBUG] A action: DEPLOY (kForward) -> intake descend/sort\n");
        printf("[INTAKE DEBUG] solenoid module=3 channels=0/1 type=CTREPCM\n");
    }
    if (bAppui) {
        printf("[INTAKE DEBUG] B PRESSED\n");
        printf("[INTAKE DEBUG] B action: motor lift UP (m_etatIntake=1)\n");
        printf("[INTAKE DEBUG] aHauteur=%d pos=%.2f\n", aHauteur, pos);
    }
    m_xPrecedent = xAppuye;

    // X = RÉTRACTER (monter/rentre)  ---  pneumatique direct
    // A = DÉPLOIER (descendre/sort) --- pneumatique direct
    if (xAppuye && !m_xPrecedent) {
        m_intake.Set(frc::DoubleSolenoid::Value::kReverse);  // RÉTRACTE
    }
    if (aAppui) {
        m_intake.Set(frc::DoubleSolenoid::Value::kForward);  // DÉPLOIE
    }

    // B = monter moteur (lift fin vers le haut)
    bool yAppui = m_driverController.GetYButton() && !m_yPrecedent;
    if (yAppui) {
        m_etatIntake = 2;
        printf("[INTAKE DEBUG] Y PRESSED -> lift fine DOWN\n");
    }
    m_yPrecedent = m_driverController.GetYButton();
    if (bAppui && !aHauteur) m_etatIntake = 1;   // monter

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

    // --- DEBUG : moteur intake ---
    if (cmd != 0.0 || m_etatIntake != 0) {
        printf("[INTAKE DEBUG] motor output=%.3f state=%d cmd=%.3f\n",
               m_intakeMotor.Get(), m_etatIntake, cmd);
    }

    // Mise à jour m_xPrecedent pour X (déjà fait plus haut via xAppuye)
    // NOTE: m_xPrecedent n'est plus utilisé pour toggle, X est maintenant momentané direct kReverse

    // Affichage des valeurs actives sur le dashboard
    frc::SmartDashboard::PutNumber("Vitesse Base active (%)",   limiteBase   * 100.0);
    frc::SmartDashboard::PutNumber("Vitesse Intake active (%)", limiteIntake * 100.0);
    frc::SmartDashboard::PutNumber("Intake Position (rot)",     pos);
    frc::SmartDashboard::PutBoolean("Intake A terre",           aTerre);
    frc::SmartDashboard::PutBoolean("Intake A hauteur",         aHauteur);
    frc::SmartDashboard::PutBoolean("Intake Deploye",           m_intakeLeve);
}

void Base::AutonomousInit() {
    m_autoState = AutoState::SEARCH;
    m_autoTimer = 0.0;
    m_searchDirection = 1.0;
    m_collectionHeading = 0.0;
    m_ballLostTimer = 0.0;
    m_heading = 0.0;
    m_prevLeftPos = m_leftMotor.GetPosition();
    m_prevRightPos = m_rightMotor.GetPosition();
    m_intakeLeve = false;
    m_xPrecedent = false;
    m_etatIntake = 0;
    m_aPrecedent = false;
    m_bPrecedent = false;
    m_intakeEncoder.SetPosition(0.0);
    m_intake.Set(frc::DoubleSolenoid::Value::kReverse);
    m_leftMotor.ResetPosition();
    m_rightMotor.ResetPosition();
}

void Base::AutonomousPeriodic() {
    AutoUpdateOdometry();

    double ballArea = AutoBallArea();
    bool ballDetected = AutoBallDetected();
    int nbObjets = (int)frc::SmartDashboard::GetNumber("Vision Nb objets", 0);

    switch (m_autoState) {
        case AutoState::SEARCH: {
            m_autoTimer += 0.02;
            double searchSpeed = frc::SmartDashboard::GetNumber("Auto Search Speed", kSearchSpeed);
            double turnAmp = 0.35;
            double turn = turnAmp * m_searchDirection * std::sin(m_autoTimer * 1.8);
            AutoDrive(searchSpeed, turn);

            if (ballDetected && ballArea > frc::SmartDashboard::GetNumber("Auto Ball Area Min", kBallAreaMin)) {
                m_autoState = AutoState::APPROACH;
                m_autoTimer = 0.0;
                m_ballLostTimer = 0.0;
            }

            if (m_autoTimer > 5.0) {
                m_searchDirection *= -1.0;
                m_autoTimer = 0.0;
            }
            break;
        }

        case AutoState::APPROACH: {
            AutoDrive(0.0, 0.0);
            m_collectionHeading = m_heading;
            m_intakeLeve = true;
            m_xPrecedent = false;
            m_intake.Set(frc::DoubleSolenoid::Value::kForward);
            m_autoState = AutoState::DEPLOY_INTAKE;
            m_autoTimer = 0.0;
            break;
        }

        case AutoState::DEPLOY_INTAKE: {
            AutoDrive(0.0, 0.0);
            m_autoTimer += 0.02;

            bool deploye = m_intakeLeve;
            if (!deploye && m_autoTimer > 0.5) {
                m_intakeLeve = true;
                m_intake.Set(frc::DoubleSolenoid::Value::kForward);
            }

            if (m_autoTimer > 1.0) {
                m_autoState = AutoState::ALIGN_BACK;
                m_autoTimer = 0.0;
                m_ballLostTimer = 0.0;
            }
            break;
        }

        case AutoState::ALIGN_BACK: {
            double targetHeading = m_collectionHeading + kPi;
            while (targetHeading > kPi) targetHeading -= 2.0 * kPi;
            while (targetHeading < -kPi) targetHeading += 2.0 * kPi;

            double headingError = targetHeading - m_heading;
            while (headingError > kPi) headingError -= 2.0 * kPi;
            while (headingError < -kPi) headingError += 2.0 * kPi;

            double turnSpeed = frc::SmartDashboard::GetNumber("Auto Turn Speed", kTurnSpeed);

            if (std::abs(headingError) < 0.25) {
                m_autoState = AutoState::REVERSE_COLLECT;
                m_autoTimer = 0.0;
                m_ballLostTimer = 0.0;
                break;
            }

            if (ballArea > 1500.0) {
                m_autoState = AutoState::OBSTACLE_AVOID;
                m_autoTimer = 0.0;
                break;
            }

            if (ballArea > 800.0 || nbObjets > 1) {
                turnSpeed *= 0.3;
            }

            double ballCenterX = AutoBallCenterX();
            double centerOffset = 0.0;
            if (ballCenterX >= 0.0) {
                centerOffset = (ballCenterX - kImageCenterX) / kImageCenterX;
            }
            double visionAdjust = centerOffset * 0.15;

            double turnDir = headingError > 0 ? 1.0 : -1.0;
            AutoDrive(0.0, turnDir * turnSpeed + visionAdjust);
            break;
        }

        case AutoState::REVERSE_COLLECT: {
            if (!ballDetected || ballArea < frc::SmartDashboard::GetNumber("Auto Ball Area Min", kBallAreaMin)) {
                m_ballLostTimer += 0.02;
                if (m_ballLostTimer > kBallLostTimeout) {
                    m_autoState = AutoState::RETURN_SEARCH;
                    m_autoTimer = 0.0;
                    break;
                }
            } else {
                m_ballLostTimer = 0.0;
            }

            double reverseSpeed = frc::SmartDashboard::GetNumber("Auto Reverse Speed", kReverseSpeed);
            double ballCenterX = AutoBallCenterX();
            double centerOffset = 0.0;
            if (ballCenterX >= 0.0) {
                centerOffset = (ballCenterX - kImageCenterX) / kImageCenterX;
            }

            double areaThreshold = frc::SmartDashboard::GetNumber("Auto Ball Area Collect", kBallAreaCollect);
            double speedFactor = 1.0;
            if (ballArea > areaThreshold * 0.5) speedFactor = 0.5;
            if (ballArea > areaThreshold * 0.8) speedFactor = 0.2;

            if (ballArea > areaThreshold * 1.1) {
                AutoDrive(0.0, 0.0);
                m_autoState = AutoState::COLLECTING;
                m_autoTimer = 0.0;
                break;
            }

            double rotation = centerOffset * 0.4;
            AutoDrive(-reverseSpeed * speedFactor, rotation);
            break;
        }

        case AutoState::COLLECTING: {
            AutoDrive(0.0, 0.0);
            m_autoTimer += 0.02;

            double pos = m_intakeEncoder.GetPosition();
            bool aTerre = pos <= kTolPos;

            if (m_autoTimer < 0.6) {
                AutoLowerIntake();
            } else if (m_autoTimer < 1.0) {
                if (aTerre) {
                    AutoStopIntake();
                    AutoRunIntakeRoller(-0.6);
                } else {
                    AutoLowerIntake();
                }
            } else {
                AutoStopIntake();
                AutoRaiseIntake();
                bool aHauteur = pos >= (kHauteurHaut - kTolPos);
                if (aHauteur || m_autoTimer > 2.0) {
                    m_autoState = AutoState::RETURN_SEARCH;
                    m_autoTimer = 0.0;
                }
            }
            break;
        }

        case AutoState::RETURN_SEARCH: {
            m_autoTimer += 0.02;

            if (m_autoTimer < 0.3) {
                m_intakeLeve = false;
                m_intake.Set(frc::DoubleSolenoid::Value::kReverse);
            }

            double headingError = m_collectionHeading - m_heading;
            while (headingError > kPi) headingError -= 2.0 * kPi;
            while (headingError < -kPi) headingError += 2.0 * kPi;

            if (std::abs(headingError) > 0.3 && m_autoTimer < 2.0) {
                double turnSpeed = frc::SmartDashboard::GetNumber("Auto Turn Speed", kTurnSpeed);
                double turnDir = headingError > 0 ? 1.0 : -1.0;
                AutoDrive(0.0, turnDir * turnSpeed);
            } else if (m_autoTimer < 3.0) {
                double searchSpeed = frc::SmartDashboard::GetNumber("Auto Search Speed", kSearchSpeed);
                double turn = 0.2 * m_searchDirection * std::sin(m_autoTimer * 2.5);
                AutoDrive(searchSpeed, turn);
            } else {
                m_autoState = AutoState::SEARCH;
                m_autoTimer = 0.0;
                m_searchDirection = 1.0;
            }
            break;
        }

        case AutoState::OBSTACLE_AVOID: {
            AutoDrive(0.0, 0.0);
            m_autoTimer += 0.02;
            if (m_autoTimer > 1.0) {
                m_autoState = AutoState::SEARCH;
                m_autoTimer = 0.0;
            }
            break;
        }
    }

    frc::SmartDashboard::PutNumber("Auto State", static_cast<double>(m_autoState));
    frc::SmartDashboard::PutNumber("Auto Heading (rad)", m_heading);
    frc::SmartDashboard::PutNumber("Auto Prev Left", m_prevLeftPos);
    frc::SmartDashboard::PutNumber("Auto Prev Right", m_prevRightPos);
}

void Base::AutoUpdateOdometry() {
    double leftPos = m_leftMotor.GetPosition();
    double rightPos = m_rightMotor.GetPosition();
    double deltaLeft = leftPos - m_prevLeftPos;
    double deltaRight = rightPos - m_prevRightPos;

    m_heading += (deltaRight - deltaLeft) * kHeadingScale;

    while (m_heading > kPi) m_heading -= 2.0 * kPi;
    while (m_heading < -kPi) m_heading += 2.0 * kPi;

    m_prevLeftPos = leftPos;
    m_prevRightPos = rightPos;
}

bool Base::AutoBallDetected() const {
    double nbObjets = frc::SmartDashboard::GetNumber("Vision Nb objets", 0.0);
    return nbObjets >= 1.0;
}

double Base::AutoBallCenterX() const {
    return frc::SmartDashboard::GetNumber("Vision Cible X", -1.0);
}

double Base::AutoBallArea() const {
    return frc::SmartDashboard::GetNumber("Vision Aire", 0.0);
}

void Base::AutoDrive(double speed, double rotation) {
    speed = std::clamp(speed, -1.0, 1.0);
    rotation = std::clamp(rotation, -1.0, 1.0);
    m_robotDrive.ArcadeDrive(speed, rotation);
}

void Base::AutoLowerIntake() {
    m_intakeMotor.Set(-kVitesseLift);
}

void Base::AutoRaiseIntake() {
    m_intakeMotor.Set(kVitesseLift);
}

void Base::AutoRunIntakeRoller(double speed) {
    m_intakeMotor.Set(std::clamp(speed, -1.0, 1.0));
}

void Base::AutoStopIntake() {
    m_intakeMotor.Set(0.0);
}