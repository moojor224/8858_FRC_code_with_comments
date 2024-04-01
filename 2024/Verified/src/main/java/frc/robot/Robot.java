// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Autonomous.AutonomousMain;
import frc.robot.Controls.ArmControls;
import frc.robot.Controls.HangerControls;
import frc.robot.Controls.LauncherControls;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.HangingSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LauncherSubsystem;
import frc.robot.TaskList.TaskList;

import org.opencv.core.Mat;

import com.revrobotics.CANSparkBase.IdleMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    SlewRateLimiter m_RateLimiterLeft = new SlewRateLimiter(2);//Jeremy 2.2, Wyatt 1.5, Jordan 1.25
    SlewRateLimiter m_RateLimiterRight = new SlewRateLimiter(2);

    Thread m_visionThread;

    final XboxController m_Controller = new XboxController(OperatorConstants.kDriverControllerPort);

    public static DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(1);
    boolean leftZeroDone_flag = false;
    boolean rightZeroDone_flag = false;

    public static double getArmEncoderPosition() {
        double offset = 0.3;
        double position = m_armEncoder.getAbsolutePosition() - offset;
        if (position < 0) {
            position += 1;
        }
        return position;
    }

    double TeleopStartTime = 0;
    double endTime = 0;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        AutonomousMain.populateList();
        m_visionThread = new Thread(() -> {
            // Get the UsbCamera from CameraServer
            UsbCamera camera = CameraServer.startAutomaticCapture();
            // Set the resolution
            camera.setResolution(160, 120);

            // Get a CvSink. This will capture Mats from the camera
            CvSink cvSink = CameraServer.getVideo();
            // Setup a CvSource. This will send images back to the Dashboard
            CvSource outputStream = CameraServer.putVideo("Rectangle", 160, 120);

            // Mats are very memory expensive. Lets reuse this Mat.
            Mat mat = new Mat();

            // This cannot be 'true'. The program will never exit if it is. This
            // lets the robot stop this thread when restarting robot code or
            // deploying.
            while (!Thread.interrupted()) {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat. If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 0) {
                    // Send the output the error.
                    outputStream.notifyError(cvSink.getError());
                    // skip the rest of the current iteration
                    continue;
                }
                // Put a rectangle on the image
                // Imgproc.rectangle(
                // mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
                // Give the output stream a new image to display
                outputStream.putFrame(mat);
            }
        });
        m_visionThread.setDaemon(true);
        m_visionThread.start();

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // m_allSubsystems.forEach(subsystem -> subsystem.periodic());
        // m_allSubsystems.forEach(subsystem -> subsystem.writePeriodicOutputs());
        // m_allSubsystems.forEach(subsystem -> subsystem.outputTelemetry());
        // m_allSubsystems.forEach(subsystem -> subsystem.writeToLog());

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    /** This function is called periodically while the robot is disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        AutonomousMain.list = new TaskList();
        AutonomousMain.list.clear();
        DriveSubsystem.leftFrontMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightFrontMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.leftBackMotor.setIdleMode(IdleMode.kBrake);
        DriveSubsystem.rightBackMotor.setIdleMode(IdleMode.kBrake);
        try {
            AutonomousMain.init(SmartDashboard.getString("Auto Selector", ""));
        } catch (InterruptedException e) {
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        if (DriverStation.isAutonomousEnabled()) { // only run auton tasks if auton is enabled
            AutonomousMain.execute();
        }
    }

    @Override
    public void teleopInit() {
        LauncherControls.activateLaunchSequence = false;
        AutonomousMain.list = new TaskList();
        AutonomousMain.list.clear();
        leftZeroDone_flag = false;
        rightZeroDone_flag = false;

        ArmSubsystem.m_encoder.setPosition(0);

        TeleopStartTime = System.currentTimeMillis();
        // endTime = TeleopStartTime += Constants.kHangerInitTime;
        // m_arm.m_encoder.setPosition(0);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        //Moves and sets hangers to zero position
        if (System.currentTimeMillis() < TeleopStartTime + Constants.kHangerInitTime) {
            double leftCurrent = HangingSubsystem.hangMotorLeft.getOutputCurrent();
            double rightCurrent = HangingSubsystem.hangMotorRight.getOutputCurrent();
            if (leftZeroDone_flag == false) {
                HangingSubsystem.hangMotorLeft.set(-Constants.kHangerSpeed);
                SmartDashboard.putNumber("Arm Position", getArmEncoderPosition());
                // check current limit reached
                if (leftCurrent >= Constants.kHangerCurrInitLimit) {
                    leftZeroDone_flag = true;
                    HangingSubsystem.m_LeftEncoder.setPosition(0);
                    HangingSubsystem.hangMotorLeft.set(0);
                    System.out.println("left Zero");
                }
            }
            if (rightZeroDone_flag == false) {
                HangingSubsystem.hangMotorRight.set(-Constants.kHangerSpeed);
                // check current limit reached
                if (rightCurrent >= Constants.kHangerCurrInitLimit) {
                    rightZeroDone_flag = true;
                    HangingSubsystem.m_RightEncoder.setPosition(0);
                    HangingSubsystem.hangMotorRight.set(0);
                    System.out.println("right Zero");
                }
            }
        } else {
            HangingSubsystem.setMotors(HangerControls.leftHangSpeed, HangerControls.rightHangSpeed);
        }

        // get controller inputs for all subsystems
        ArmControls.armInput();
        HangerControls.hangerInput();
        LauncherControls.launcherInput();

        // move hangers to target position

        // read controller joysticks
        double xSpeed = m_Controller.getRightX();
        double ySpeed = m_Controller.getLeftY();

        // add deadzone to center of joysticks
        xSpeed = MathUtil.applyDeadband(xSpeed, 0.1) * 0.85; // slow down turn speed to help with precision
        ySpeed = MathUtil.applyDeadband(ySpeed, 0.1);

        // apply acceleration limiter to joystick input
        xSpeed = m_RateLimiterRight.calculate(xSpeed);
        ySpeed = m_RateLimiterLeft.calculate(ySpeed);

        // Setting the motor speeds across all subsystems
        // set drive speed
        DriveSubsystem.setMotors(xSpeed, ySpeed);
        HangingSubsystem.setMotors(HangerControls.leftHangSpeed, HangerControls.rightHangSpeed);
        ArmSubsystem.setMotor(ArmControls.limitedArmSpeed * Constants.kArmLimiter);
        IntakeSubsystem.setMotor(ArmControls.intakeSpeed);
        LauncherSubsystem.setMotors(LauncherControls.launchSpeed);

        // System.out.println(m_arm.m_encoder.getPosition());

    }
}
