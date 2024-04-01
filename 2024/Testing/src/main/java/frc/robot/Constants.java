// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    // Drivetrain
    public static final int kDrivetrainFLMotorId = 1;
    public static final int kDrivetrainBLMotorId = 2;
    public static final int kDrivetrainFRMotorId = 3;
    public static final int kDrivetrainBRMotorId = 4;

    public static final double kDriveLimiter = .7;
    public static final double kTurnLimiter = .7;

    public static final double autonDriveSpeed = .4;

    // Hanging
    public static final int kHangerLeftMotorId = 5;
    public static final int kHangerRightMotorId = 6;

    public static final double kHangerSpeed = .5;

    public static final double kHangLowerLimit = 0;
    public static final double kHangUpperLimit = 52;

    public static final double kHangerCurrInitLimit = 29.0;
    public static final double kHangerInitTime = 3000;
    public static final double khangerTargetTolerance = 1;

    // Arm
    public static final int kArmMotor = 7;

    public static final double kArmDefaultSpeed = 0.4; // use this when arm is far away from target position
    public static final double kArmSafetySpeed = 0.15; // use this when arm is approaching target position

    public static final double kArmStowEncPos   = 0.420;
    public static final double kArmLaunchEncPos = 0.715;
    public static final double kArmFloorEncPos  = 0.125;
    public static final double kArmEncMargin    = 0.01;


    public static final double kArmLimiter = .5;

    public static final double armP = .12;
    public static final double armI = 0;
    public static final double armD = .001;

    public static final double kUpperLimit = 2.5;
    public static final double kLowerLimit = 2;

    // Intake
    public static final int kIntakeMotorId = 8;

    public static final double kIntakeSpeed = 0.5;

    // Launcher
    public static final int kLaunchMotorRId = 9;
    public static final int kLaunchMotorLId = 10;

    public static final double kSpeakerLaunchSpeed = .9;

    // Speaker Speed Defines
    public static final double kTargetSpeakerVelocity = 4400;
    public static final double kSpeakerShooterSpeed = 0.9 * kTargetSpeakerVelocity / 4400;

    // Amp Speed Defines
    public static final double kTargetAmpVelocity = 700;
    public static final double kAmpShooterSpeed = 0.9 * kTargetAmpVelocity / 4400;

    // YEET Speed defines Shooter
    public static final double kTargetYeetVelocity = 5000;
    public static final double kYeetShooterSpeed = 0.9 * kTargetYeetVelocity / 4400;
}
