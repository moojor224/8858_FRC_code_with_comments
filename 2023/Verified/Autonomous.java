My Drive
package frc.robot;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.RelativeEncoder;

public class Autonomous {
    enum Auton {
        DO_NOTHING,
        SCORE_AND_STAY,
        SCORE_AND_COMMUNITY,
        SCORE_AND_BALANCE
    }

    Auton m_selectedAuton;
    Timer m_timer;
    CANSparkMax m_armMotor;
    CANSparkMax m_intakeMotor;
    DifferentialDrive m_differentialDrive;
    ADIS16470_IMU m_imu;
    RelativeEncoder m_armEncoder;
    LinearFilter m_accelFilter = LinearFilter.movingAverage(5);
    double m_filteredAccel = 0.0;

    boolean armIsZero = false;
    double lastPosition;
    int m_state = 0;
    double Threshold = 10.0;

    public Autonomous(String autonName, CANSparkMax armMotor, CANSparkMax intakeMotor,
            DifferentialDrive differentialDrive, ADIS16470_IMU imu) {
        switch (autonName) {
            case "Do Nothing":
                m_selectedAuton = Auton.DO_NOTHING;
                break;
            case "Score and Stay":
                m_selectedAuton = Auton.SCORE_AND_STAY;
                break;
            case "Score and Community":
                m_selectedAuton = Auton.SCORE_AND_COMMUNITY;
                break;
            case "Score and Balance":
                m_selectedAuton = Auton.SCORE_AND_BALANCE;
                break;
            default:
                m_selectedAuton = Auton.SCORE_AND_STAY;
        }
        m_armMotor = armMotor;
        m_intakeMotor = intakeMotor;
        m_differentialDrive = differentialDrive;
        m_imu = imu;
        m_armEncoder = m_armMotor.getEncoder();
        m_timer = new Timer();
        m_timer.reset();
        m_timer.start();
    }

    public void execute() {
        SmartDashboard.putNumber("autoState", m_state);
        m_filteredAccel = m_accelFilter.calculate(m_imu.getAccelY());

        if (/* m_timer.get() < 6.0 && */ !armIsZero) {
            armIsZero = zeroArm();
            System.out.println("arm is Zeroing" + "timer:" + m_timer.get());
            if (armIsZero) {
                m_timer.reset();
                m_armEncoder.setPosition(0.0);
                System.out.println("Info: arm is zero");
            }
        } else {
            System.out.println("Info: State = " + m_state);
            switch (m_selectedAuton) {
                case DO_NOTHING:
                    break;
                case SCORE_AND_STAY:
                    m_differentialDrive.arcadeDrive(0.0, 0.0);
                    if (m_state == 0) {
                        m_armMotor.set(-0.2);
                        if (m_timer.get() >= .5 && m_armEncoder.getPosition() >= lastPosition) {
                            m_state += 1;
                            m_timer.reset();
                        }
                    } else if (m_state == 1) {
                        m_intakeMotor.set(-0.75);
                        if (m_timer.get() >= 1.0) {
                            m_state += 1;
                            m_timer.reset();
                        }
                    } else if (m_state == 2) {
                        m_intakeMotor.set(0.0);
                        m_armMotor.set(0.2);
                        if (m_timer.get() >= 2.0) {
                            m_state += 1;
                            m_timer.reset();
                        }
                    } else if (m_state == 3) {
                        m_armMotor.set(0.0);
                    }
                    break;
                case SCORE_AND_COMMUNITY:
                    // 0.6 speed for 3 seconds then...
                    // 0.2 speed for 2 seconds
                    break;
                case SCORE_AND_BALANCE:

                    if (m_state == 0) {
                        m_armMotor.set(-0.3);
                        if (m_timer.get() >= .5 && m_armEncoder.getPosition() >= lastPosition) {
                            m_state += 1;
                            m_timer.reset();
                        }
                    } else if (m_state == 1) {
                        m_intakeMotor.set(-0.75);
                        if (m_timer.get() >= .45) {
                            m_state += 1;
                            m_timer.reset();
                        }
                    } else if (m_state == 2) {
                        m_intakeMotor.set(0.0);
                        m_armMotor.set(0.2);
                        if (m_timer.get() >= 2.0) {
                            m_state += 1;
                            m_timer.reset();
                        }
                    } else if (m_state == 3) {
                        m_armMotor.set(0.0);
                        m_differentialDrive.arcadeDrive(-0.7, 0);
                        if (m_timer.get() >= 2.5) {
                            m_state += 1;
                            m_timer.reset();
                        }
                    }
                    // leaning back
                    else if (m_state == 4) {
                        m_armMotor.set(0.0);
                        if (m_imu.getYComplementaryAngle() <= Threshold) {
                            m_state = 5;
                            m_timer.reset();
                        } else {
                            m_differentialDrive.arcadeDrive(-0.57, 0);
                        }
                    }
                    else if (m_state == 5) {
                        m_armMotor.set(0.0);
                        if (m_timer.get() >= 0.3) {
                            m_state = 8;
                            m_timer.reset();
                            m_differentialDrive.arcadeDrive(.001, 0);
                        } else {
                            m_differentialDrive.arcadeDrive(0.4, 0);
                        }
                    }
                    // leaning forward
                    else if (m_state == 6) {
                        m_armMotor.set(0.0);
                        if (m_imu.getYComplementaryAngle() >= -Threshold) {
                            m_state = 7;
                            m_timer.reset();
                        } else {
                            m_differentialDrive.arcadeDrive(0.57, 0);
                        }
                    }
                    else if (m_state == 7) {
                        m_armMotor.set(0.0);
                        if (m_timer.get() >= 0.3) {
                            m_state = 8;
                            m_timer.reset();
                            m_differentialDrive.arcadeDrive(.001, 0);
                        } else {
                            m_differentialDrive.arcadeDrive(-0.4, 0);
                        }
                    }
                    // balanced
                    else if (m_state == 8) {
                        m_armMotor.set(0.0);
                        if (m_imu.getYComplementaryAngle() <= -Threshold) {
                            m_state = 6;
                            m_timer.reset();
                        } else if (m_imu.getYComplementaryAngle() >= Threshold) {
                            m_state = 4;
                            m_timer.reset();
                        }
                        else{
                            m_armMotor.set(0.0);
                            m_differentialDrive.arcadeDrive(-0.001, 0);
                        }
                    }
                    break;
                default:
                    break;
            }
            lastPosition = m_armEncoder.getPosition();
        }
    }

    public static void populateList() {
        String[] autonNames = {
                "Do Nothing",
                "Score and Stay",
                "Score and Community",
                "Score and Balance"
        };
        SmartDashboard.putStringArray("Auto List", autonNames);
    }

    public boolean zeroArm() {
        if (m_timer.get() >= 1.0) {
            if (m_armEncoder.getPosition() <= lastPosition) {
                m_armMotor.set(0.0);
                return true;
            } else {
                m_armMotor.set(0.1);
            }
        } else {
            m_armMotor.set(0.1);
        }
        lastPosition = m_armEncoder.getPosition();
        return false;
    }
}
