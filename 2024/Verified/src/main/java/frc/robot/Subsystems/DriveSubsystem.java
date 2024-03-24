// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class DriveSubsystem extends SubsystemBase {

    // Defines Each of the motors with a spark ID
    public static CANSparkMax leftFrontMotor = new CANSparkMax(Constants.kDrivetrainFLMotorId, MotorType.kBrushless) {
        {
            setIdleMode(IdleMode.kCoast);
            set(0);
            setInverted(false);
        }
    };
    public static CANSparkMax leftBackMotor = new CANSparkMax(Constants.kDrivetrainBLMotorId, MotorType.kBrushless) {
        {
            setIdleMode(IdleMode.kCoast);
            set(0);
        }
    };
    public static CANSparkMax rightFrontMotor = new CANSparkMax(Constants.kDrivetrainFRMotorId, MotorType.kBrushless) {
        {
            setIdleMode(IdleMode.kCoast);
            set(0);
        }
    };
    public static CANSparkMax rightBackMotor = new CANSparkMax(Constants.kDrivetrainBRMotorId, MotorType.kBrushless) {
        {
            setIdleMode(IdleMode.kCoast);
            set(0);
        }
    };

    public static final DifferentialDrive m_robotDrive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);
    public static final DifferentialDrive m_robotDrive2 = new DifferentialDrive(leftBackMotor, rightBackMotor);

    public DriveSubsystem() {
    }

    // Sets the Motor Speeds with the parameters ySpeed and xSpeed
    public static void setMotors(double ySpeed, double xSpeed) {
        m_robotDrive.arcadeDrive(ySpeed, xSpeed);
        m_robotDrive2.arcadeDrive(ySpeed, xSpeed);
    }

}
