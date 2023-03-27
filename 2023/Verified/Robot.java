My Drive
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.ModuleLayer.Controller;

import javax.management.relation.RelationException;

import org.photonvision.PhotonCamera;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class Robot extends TimedRobot {

  // These 6 lines are what define our motors. First, we must declare what kind of
  // controllers we use.
  // In this scenario, we are using the CAN bus of our Spark Max controllers.
  // Next, we give each
  // controller a name. To make reading and debugging our code easier, we use "m_"
  // in the beginning
  // of each motor name. Next, we use new to state that we are creating a new
  // object. Then, we redefine what
  // kind of controller we use. Next, we state the CAN ID of our controller, and
  // state what type of motor
  // we would like to control.

  CANSparkMax m_leftFrontMotor = new CANSparkMax(1, MotorType.kBrushed);
  CANSparkMax m_leftBackMotor = new CANSparkMax(2, MotorType.kBrushed);
  CANSparkMax m_rightFrontMotor = new CANSparkMax(3, MotorType.kBrushed);
  CANSparkMax m_rightBackMotor = new CANSparkMax(4, MotorType.kBrushed);
  CANSparkMax m_armMotor = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax m_intakeMotor = new CANSparkMax(6, MotorType.kBrushless);

  SlewRateLimiter m_rateLimiter = new SlewRateLimiter(3.5);

  // By grouping up each motor controller, we can simplify the programming and
  // debugging process
  MotorControllerGroup m_leftDrive = new MotorControllerGroup(m_leftFrontMotor, m_leftBackMotor);
  MotorControllerGroup m_rightDrive = new MotorControllerGroup(m_rightFrontMotor, m_rightBackMotor);

  PIDController pidController = new PIDController(.5, 0, 0);

  RelativeEncoder armEncoder = m_armMotor.getEncoder();

  PhotonCamera intakeCam = new PhotonCamera("photonvision");

  private final DifferentialDrive m_differentialDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);

  private final XboxController m_stick = new XboxController(0);

  Accelerometer accelerometer = new BuiltInAccelerometer();

  // Edit modifiers to change the sensitivity/aggressiveness of the joysticks. A
  // higher value will result
  // in a gradual ramp up. This allows for ease of control when moving slower. A
  // higher value will help
  // A lower value will allow for a more aggressive feel for the joystick. Values
  // must be no greater than
  // 1, and no lesser than -0.5.

  double modif1 = 1;
  double modif2 = 0.5;

  double armError;

  // In order to prevent any damage on the intake from the shear power of the
  // motors, we limit the
  // amount of amperage going into the motors. This lowers both the power draw and
  // the torque output

  double MOTOR_SPEED_LIMIT_DEFAULT = .9;
  double MOTOR_SPEED_LIMIT_SLOW = 0.6;
  double MOTOR_SPEED_LIMIT = MOTOR_SPEED_LIMIT_DEFAULT;
  double DEADZONE_PARAM = 0.1; // used for software deadzone control

  // initialize motor joystick readings to zero
  double alteredX = 0;
  double alteredY = 0;

  // IMPORTANT: Current Limits are handled directly in the SparkMAX, not this code
  int armAmperageLimit = 20;
  int intakeAmperageLimit = 25;

  double armOutputPower = 0.4;

  int intakeHoldAmperageLimit = 5;
  double intakeOutputPower = 0.75;
  double intakeHoldPower = 0.07;

  double armZeroPower = 0.15;
  double calibratedEncoder;

  int ticksPerRotation = 1680;

  // Enable the toggle-speed button, otherwise default speed is the only speed
  boolean toggle_speed_functionality = true;
  boolean toggle_brake_functionality = true;

  // boolean variables used to support the speed-toggle functionality
  boolean speed_button_pressed = false;
  boolean fast = true;
  boolean slow = false;
  boolean speedToggle = fast;

  // switched to true by zero-ing the arm at start of autonomous
  boolean armIsZero = false;

  // capture this value once Arm is Zeroed, use as reference for relative Arm
  // positions
  double armZeroPosition;

  // capture this value when the furthest position is found in autonomous, use as
  // reference for relative Arm positions
  double armExtendedPosition;

  // subtract armZeroPosition from armExtendedPosition to get this value in
  // autonomous
  double ARM_TOTAL_RANGE;

  // decode values for the object currently in the intake
  int PARAM_CUBE = 1;
  int PARAM_CONE = 2;
  int currentObj = 0;

  double armPosition = 0;
  double desiredPosition = 0;
  double armTolerance = 0.02;

  boolean intakeRunning = false;

  double armDifference = 0;
  double kp = .4;
  double ki = 8;
  double integral = 1;

  // AUTONOMOUS PARAMETERS
  int AUTON_HEIGHT_MEDIUM = 1;
  int AUTON_HEIGHT_HIGH = 2;
  int AUTON_HEIGHT = AUTON_HEIGHT_HIGH;
  int arm_raise_time = 500;
  double autonArmSpeed = -0.2; // define the arm speed to be used for autonomous (negative is forward)
  int autonomousPiece = PARAM_CUBE; // define which game piece is being scored
  boolean touch_charge_station = true; // set to true if starting in middle position and want to backup up to charge
                                        // station after scoring game piece
  double ballance_sample_rate = 600;
  boolean leave_community = false; // set to true if leaving community after scoring game piece
  boolean maxHeightReached = false; // set this flag once the arm is fully extended
  boolean autonomousScored = false; // set this flag after scoring the game piece in autonomous

  ADIS16470_IMU m_imu;
  Autonomous m_auton;
  @Override
  public void autonomousInit() {
    m_leftBackMotor.setIdleMode(IdleMode.kBrake);
    m_leftFrontMotor.setIdleMode(IdleMode.kBrake);
    m_rightBackMotor.setIdleMode(IdleMode.kBrake);
    m_rightFrontMotor.setIdleMode(IdleMode.kBrake);
    m_armMotor.setIdleMode(IdleMode.kBrake);
    double armExtensionTime = 2.0;

    // for our autonomous, start against the shelf, and extend arm
    // release payload
    // retract arm
    // maybe drive backwards for a few seconds

    armIsZero = false;

    m_auton = new Autonomous(SmartDashboard.getString("Auto Selector", ""), m_armMotor, m_intakeMotor, m_differentialDrive, m_imu);
  }

  @Override
  public void autonomousPeriodic() {
    m_auton.execute();
    SmartDashboard.putNumber("accelY", m_imu.getAccelY());
    SmartDashboard.putNumber("angleY", m_imu.getYComplementaryAngle());
  }

  @Override
  public void robotInit() {

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftFrontMotor.setInverted(true);
    m_leftBackMotor.setInverted(true);
    m_armMotor.setInverted(true);

    m_leftBackMotor.setIdleMode(IdleMode.kCoast);
    m_leftFrontMotor.setIdleMode(IdleMode.kCoast);
    m_rightBackMotor.setIdleMode(IdleMode.kCoast);
    m_rightFrontMotor.setIdleMode(IdleMode.kCoast);

    m_intakeMotor.setIdleMode(IdleMode.kBrake);
    m_armMotor.setIdleMode(IdleMode.kBrake);

    armIsZero = false;
    Autonomous.populateList();

    m_imu = new ADIS16470_IMU();
  }

  @Override
  public void teleopPeriodic() {

    if((m_stick.getLeftTriggerAxis() >= 0.1) || (m_stick.getRightTriggerAxis() >= 0.1)){
      m_armMotor.set((m_stick.getLeftTriggerAxis() - m_stick.getRightTriggerAxis()) * armOutputPower);
    } else {
      m_armMotor.set(0.07);
    }

    if (m_stick.getLeftBumper()) {
      m_intakeMotor.set(intakeOutputPower);
    } else if (m_stick.getRightBumper()) {
      m_intakeMotor.set(-intakeOutputPower);
    }

    else {
      m_intakeMotor.set(0.0);
    }

    // intake a Cube
    if (m_stick.getXButton()) {
      currentObj = PARAM_CUBE;
      m_intakeMotor.set(intakeOutputPower);
    }

    // intake a Cone
    if (m_stick.getYButton()) {
      currentObj = PARAM_CONE;
      m_intakeMotor.set(-intakeOutputPower);
    }

    // dump out whatever you just sucked up
    if (m_stick.getBButton()) {
      if (currentObj == PARAM_CUBE) {
        m_intakeMotor.set(-intakeOutputPower);
      } else if (currentObj == PARAM_CONE) {
        m_intakeMotor.set(intakeOutputPower);
      }
    }

    if (toggle_speed_functionality) {

      if (m_stick.getAButtonPressed()) {
        if (speedToggle == fast) {
          System.out.println("Toggling to Slow Slow");
          speedToggle = slow;
          MOTOR_SPEED_LIMIT = MOTOR_SPEED_LIMIT_SLOW;
        } else {
          System.out.println("Toggling to Default Mode");
          speedToggle = fast;
          MOTOR_SPEED_LIMIT = MOTOR_SPEED_LIMIT_DEFAULT;
        }
      }

      // detect first edge of a speed button press
      // if (speed_button_pressed == false){
      // if (m_stick.getStartButton()) {
      // speed_button_pressed = true;
      // if (speedToggle == fast) {
      // speedToggle = slow;
      // System.out.println("Toggling to Slow Slow");
      // }
      // else {
      // speedToggle = fast;
      // System.out.println("Toggling to Default Mode");
      // }
      // } else {
      // speed_button_pressed = false;
      // }
      // }

      // Watchdog way to look for button release
      // if (m_stick.getStartButton()){
      // speed_button_pressed = true;
      // } else{
      // speed_button_pressed = false;
      // }

      // Fast Speed Limits
      // if (speedToggle == fast) {
      // MOTOR_SPEED_LIMIT = MOTOR_SPEED_LIMIT_DEFAULT;
      // }

      // // Slow Speed Limits
      // if (speedToggle == slow) {
      // MOTOR_SPEED_LIMIT = MOTOR_SPEED_LIMIT_SLOW;
      // }
    }
    // */

    // if you're pressing the "START" button, robot is braking.
    if (m_stick.getStartButtonPressed()) {
      if (toggle_brake_functionality == false) {
        System.out.println("INFO : Brake Enabled");
        m_leftBackMotor.setIdleMode(IdleMode.kBrake);
        m_leftFrontMotor.setIdleMode(IdleMode.kBrake);
        m_rightBackMotor.setIdleMode(IdleMode.kBrake);
        m_rightFrontMotor.setIdleMode(IdleMode.kBrake);
        toggle_brake_functionality = true;
      } else {
        System.out.println("INFO : Brake Disabled");
        m_leftBackMotor.setIdleMode(IdleMode.kCoast);
        m_leftFrontMotor.setIdleMode(IdleMode.kCoast);
        m_rightBackMotor.setIdleMode(IdleMode.kCoast);
        m_rightFrontMotor.setIdleMode(IdleMode.kCoast);
        toggle_brake_functionality = false;
      }
    }
    /*
     * if(toggle_brake_functionality){
     * if(m_stick.getBackButtonPressed()){
     * System.out.println("INFO : Brake Enabled");
     * m_leftBackMotor.setIdleMode(IdleMode.kBrake);
     * m_leftFrontMotor.setIdleMode(IdleMode.kBrake);
     * m_rightBackMotor.setIdleMode(IdleMode.kBrake);
     * m_rightFrontMotor.setIdleMode(IdleMode.kBrake);
     * }
     *
     * if(m_stick.getAButtonReleased()){
     * System.out.println("INFO : Brake Disabled");
     * m_leftBackMotor.setIdleMode(IdleMode.kCoast);
     * m_leftFrontMotor.setIdleMode(IdleMode.kCoast);
     * m_rightBackMotor.setIdleMode(IdleMode.kCoast);
     * m_rightFrontMotor.setIdleMode(IdleMode.kCoast);
     * }
     * }
     */

    /////////////////////////////// MOBILITY OPTION 1
    /////////////////////////////// ///////////////////////////////
    /*
     * math here breaks down to:
     * altered = (modif)*X^3 - (modif)*X + X
     * where X represents the stick position
     * therefore -1 <= X <= 1
     *
     * Summary of behavior:
     * for 0 < modif < 1:
     * implements artificial 'deadzones'
     * very 'touchy' at further stick distances
     * can change exponent to be higher value to increase effect so long as
     * odd-symetry is preserved
     *
     * for -0.5 < modif < 0:
     * altered starts out 'thouchy' then flattens out towards further stick
     * deadzones
     *
     * not recommended modif > 1 or modif < -0.5 as these can return altered values
     * > 1 or < -1
     *
     */
    alteredX = modif2 * Math.pow(m_stick.getRightX(), 3) - ((modif2 - 1) * m_stick.getRightX());
    alteredY = modif1 * Math.pow(m_stick.getLeftY(), 3) - ((modif1 - 1) * m_stick.getLeftY());
    ///////////////////////////// END MOBILITY OPTION 1
    ///////////////////////////// /////////////////////////////

    // alteredX = modif2 * m_stick.getRightX();
    // alteredY = modif1 * m_stick.getLeftY();

    alteredY = m_rateLimiter.calculate(alteredY);


    m_differentialDrive.arcadeDrive(-alteredY * MOTOR_SPEED_LIMIT, -alteredX * MOTOR_SPEED_LIMIT);

    armEncoder.setMeasurementPeriod(64);
    double maxLimit = armZeroPosition + 9.5;
  }

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putNumber("accelY", m_imu.getAccelY());
    SmartDashboard.putNumber("angleY", m_imu.getYComplementaryAngle());

  }
}
