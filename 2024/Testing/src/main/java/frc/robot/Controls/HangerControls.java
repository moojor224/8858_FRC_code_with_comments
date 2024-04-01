package frc.robot.Controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.HangingSubsystem;

public class HangerControls extends SubsystemBase {
    static final XboxController m_Controller = new XboxController(OperatorConstants.kDriverControllerPort);

    public static double rightHangSpeed;
    public static double leftHangSpeed;
    public static double targetHanger_pos = Constants.kHangLowerLimit;

    public static double BackLimit = 0;
    public static double frontLimit = 0;

    public static double leftPosition;
    public static double rightPosition;
    public static double hangSpeed;

    public HangerControls() {
    }

    // sets controls for hangers
    public static void hangerInput() {
        // poll the encoders
        leftPosition = HangingSubsystem.m_LeftEncoder.getPosition();
        rightPosition = HangingSubsystem.m_RightEncoder.getPosition();

        // change the target position to the lower limit
        if (m_Controller.getAButtonPressed()) {
            targetHanger_pos = Constants.kHangLowerLimit;
        }

        // change the target position to the upper limit
        if (m_Controller.getYButtonPressed()) {
            targetHanger_pos = Constants.kHangUpperLimit;
        }

        // handle edge case where target position may be outside of the range
        targetHanger_pos = MathUtil.clamp(targetHanger_pos, Constants.kHangLowerLimit, Constants.kHangUpperLimit);

        // Left Hanger controls based on current position VS target position
        if (leftPosition < targetHanger_pos - Constants.khangerTargetTolerance) {
            leftHangSpeed = Constants.kHangerSpeed;
        } else if (leftPosition > targetHanger_pos + Constants.khangerTargetTolerance) {
            leftHangSpeed = -Constants.kHangerSpeed;
        } else {
            leftHangSpeed = 0;
        }

        // Right Hanger controls based on current position VS target position
        if (rightPosition < targetHanger_pos - Constants.khangerTargetTolerance) {
            rightHangSpeed = Constants.kHangerSpeed;
        } else if (leftPosition > targetHanger_pos + Constants.khangerTargetTolerance) {
            rightHangSpeed = -Constants.kHangerSpeed;
        } else {
            rightHangSpeed = 0;
        }


        // if (m_Controller.getBackButton()){
        //     HangingSubsystem.setMotors(Constants.kHangerSpeed, 0);
        //     HangingSubsystem.m_LeftEncoder.setPosition(0);
        // }
        // if (m_Controller.getStartButton()){
        //     HangingSubsystem.setMotors(0, Constants.kHangerSpeed);
        //     HangingSubsystem.m_LeftEncoder.setPosition(0);
        // }
        // else if(m_Controller.getStartButtonReleased() || m_Controller.getBackButtonReleased()){
        //     rightHangSpeed = 0;
        //     leftHangSpeed = 0;
        // }
       // Left Hanger controls based on current position VS target position
        // if (m_Controller.getAButton() && leftPosition >= Constants.kHangLowerLimit) {
        //     leftHangSpeed = -Constants.kHangerSpeed;
        //     System.out.println("left Hanger Position" + leftPosition);
        //     System.out.println("Right Hanger Position" + rightPosition);
        // } else if (m_Controller.getYButton() && leftPosition <= Constants.kHangUpperLimit) {
        //     leftHangSpeed = Constants.kHangerSpeed;
        //     System.out.println("left Hanger Position" + leftPosition);
        //     System.out.println("Right Hanger Position" + rightPosition);
        // } else if (m_Controller.getBackButton()){
        //     leftHangSpeed = -Constants.kHangerSpeed;
        //     HangingSubsystem.m_LeftEncoder.setPosition(0);
        // }else {
        //     leftHangSpeed = 0;
        //    }
        //  if (m_Controller.getAButton() && rightPosition >= Constants.kHangLowerLimit) {
        //     rightHangSpeed = -Constants.kHangerSpeed;
        // } else if (m_Controller.getYButton() && leftPosition <= Constants.kHangUpperLimit) {
        //     rightHangSpeed = Constants.kHangerSpeed;
        // }else if (m_Controller.getStartButton()){
        //     rightHangSpeed = -Constants.kHangerSpeed;
        //     HangingSubsystem.m_RightEncoder.setPosition(0);
        // } else {
        //     rightHangSpeed = 0;
        // }
        

    }

    public static void runEncoder() {
        double leftPosition = HangingSubsystem.m_LeftEncoder.getPosition();
        double rightPosition = HangingSubsystem.m_RightEncoder.getPosition();

        double leftCurrent = HangingSubsystem.hangMotorLeft.getOutputCurrent();
        double rightCurrent = HangingSubsystem.hangMotorRight.getOutputCurrent();

        System.out.println("left Hanger Position" + leftPosition);
        System.out.println("Right Hanger Position" + rightPosition);
        System.out.println("left Hanger Amperage" + leftCurrent);
        System.out.println("Right Hanger Amperage" + rightCurrent);
    }

}
