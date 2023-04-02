
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

    // 'Score and...' arm variables
    double SCORING_ARM_SPEED    = 0.3;  // speed at which the arm reaches out to score
    double LOWER_ARM_SPEED      = 0.2;  // speed at which the arm retracts after scoring
    double LOWER_ARM_TIME       = 2.0;  // time that the arm gets lowered
    double INTAKE_OUTPUT_SPEED  = 0.75; // speed that the intake turns to output in autonomous
    double INTAKE_OUTPUT_TIME   = 0.45; // amount of time intake runs for in autonomous to

    // 'Score and Community' variables
    double BACKUP_SPEED         = 0.6;  // set motors to this speed to leave community
    double BACKUP_TIME          = 4.0;  // drive the motors for this duration of time to leave community

    // 'Score and Ballance' variables
    double THRESHOLD            = 10.0; // in degrees, threshold angle at which kickback is started
    double BLIND_DRIVE_SPEED    = 0.7;  // set motor to this power to get to the charge-station
    double BLIND_DRIVE_TIME     = 2.5;  // drive at BLIND_DRIVE_SPEED for this much time to get to charge station
    double CLIMB_SPEED          = 0.57; // speed to climb charge station in steady manner
    double KICKBACK_SPEED       = 0.4;  // set motor to this power during 'kickback' maneuver which should leave the robot balanced
    double KICKBACK_TIME        = 0.3;  // amount of time to apply KICKBACK_SPEED

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
                        m_armMotor.set(-SCORING_ARM_SPEED);
                        if (m_timer.get() >= .5 && m_armEncoder.getPosition() >= lastPosition) {
                            m_state += 1;
                            m_timer.reset();
                        }
                    } else if (m_state == 1) {
                        m_intakeMotor.set(-INTAKE_OUTPUT_SPEED);
                        if (m_timer.get() >= INTAKE_OUTPUT_TIME) {
                            m_state += 1;
                            m_timer.reset();
                        }
                    } else if (m_state == 2) {
                        m_intakeMotor.set(0.0);
                        m_armMotor.set(LOWER_ARM_SPEED);
                        if (m_timer.get() >= LOWER_ARM_TIME) {
                            m_state += 1;
                            m_timer.reset();
                        }
                    } else if (m_state == 3) {
                        m_armMotor.set(0.0);
                    }
                    break;
                case SCORE_AND_COMMUNITY:
                    if (m_state == 0) {
                        m_armMotor.set(-SCORING_ARM_SPEED);
                        if (m_timer.get() >= .5 && m_armEncoder.getPosition() >= lastPosition) {
                            m_state += 1;
                            m_timer.reset();
                        }
                    } else if (m_state == 1) {
                        m_intakeMotor.set(-INTAKE_OUTPUT_SPEED);
                        if (m_timer.get() >= INTAKE_OUTPUT_TIME) {
                            m_state += 1;
                            m_timer.reset();
                        }
                    } else if (m_state == 2) {
                        m_intakeMotor.set(0.0);
                        m_armMotor.set(LOWER_ARM_SPEED);
                        if (m_timer.get() >= LOWER_ARM_TIME) {
                            m_state += 1;
                            m_timer.reset();
                        }
                    } else if (m_state == 3) {
                        m_armMotor.set(0.0);
                        m_differentialDrive.arcadeDrive(-BACKUP_SPEED, 0);
                        if (m_timer.get() >= BACKUP_TIME) {
                            m_state += 1;
                            m_timer.reset();
                        }
                    } else if (m_state == 4) {
                        m_armMotor.set(0.0);
                        m_differentialDrive.arcadeDrive(0, 0);
                    }
                    break;
                case SCORE_AND_BALANCE:

                    if (m_state == 0) {
                        m_armMotor.set(-SCORING_ARM_SPEED);
                        if (m_timer.get() >= .5 && m_armEncoder.getPosition() >= lastPosition) {
                            m_state += 1;
                            m_timer.reset();
                        }
                    } else if (m_state == 1) {
                        m_intakeMotor.set(-INTAKE_OUTPUT_SPEED);
                        if (m_timer.get() >= INTAKE_OUTPUT_TIME) {
                            m_state += 1;
                            m_timer.reset();
                        }
                    } else if (m_state == 2) {
                        m_intakeMotor.set(0.0);
                        m_armMotor.set(LOWER_ARM_SPEED);
                        if (m_timer.get() >= LOWER_ARM_TIME) {
                            m_state += 1;
                            m_timer.reset();
                        }
                    } else if (m_state == 3) {
                        m_armMotor.set(0.0);
                        m_differentialDrive.arcadeDrive(-BLIND_DRIVE_SPEED, 0);
                        if (m_timer.get() >= BLIND_DRIVE_TIME) {
                            m_state += 1;
                            m_timer.reset();
                        }
                    }
                    // leaning back correction
                    else if (m_state == 4) {
                        m_armMotor.set(0.0);
                        if (m_imu.getYComplementaryAngle() <= THRESHOLD) {
                            m_state += 1;
                            m_timer.reset();
                        } else {
                            m_differentialDrive.arcadeDrive(-CLIMB_SPEED, 0);
                        }
                    }
                    else if (m_state == 5) {
                        m_armMotor.set(0.0);
                        if (m_timer.get() >= KICKBACK_TIME) {
                            m_state = 8; // check if we're balanced
                            m_timer.reset();
                            m_differentialDrive.arcadeDrive(.001, 0);
                        } else {
                            m_differentialDrive.arcadeDrive(KICKBACK_SPEED, 0);
                        }
                    }
                    // leaning forward correction
                    else if (m_state == 6) {
                        m_armMotor.set(0.0);
                        if (m_imu.getYComplementaryAngle() >= -THRESHOLD) {
                            m_state += 1;
                            m_timer.reset();
                        } else {
                            m_differentialDrive.arcadeDrive(CLIMB_SPEED, 0);
                        }
                    }
                    else if (m_state == 7) {
                        m_armMotor.set(0.0);
                        if (m_timer.get() >= KICKBACK_TIME) {
                            m_state = 8; // check if we're balanced
                            m_timer.reset();
                            m_differentialDrive.arcadeDrive(.001, 0);
                        } else {
                            m_differentialDrive.arcadeDrive(-KICKBACK_SPEED, 0);
                        }
                    }
                    // balanced
                    else if (m_state == 8) {
                        m_armMotor.set(0.0);
                        if (m_imu.getYComplementaryAngle() <= -THRESHOLD) {
                            m_state = 6; // jump to the 'leaning forward' state
                            m_timer.reset();
                        } else if (m_imu.getYComplementaryAngle() >= THRESHOLD) {
                            m_state = 4; // jump to the 'leaning backward' state
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
