package frc.robot.Controls;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class DriveControls extends SubsystemBase {
    private static DriveControls mInstance;
    // private final DriveSubsystem m_drive = DriveSubsystem.getInstance();

    final XboxController m_Controller = new XboxController(OperatorConstants.kDriverControllerPort);

    public double xAxis;
    public double yAxis;

    public double xSpeed;
    public double ySpeed;

    // This is Used to create an intance of this class whitin other files
    public static DriveControls getInstance() {
        if (mInstance == null) {
            mInstance = new DriveControls();
        }
        return mInstance;
    }

    // SlewRateLimiter m_RateLimiterLeft = new SlewRateLimiter(5);
    // SlewRateLimiter m_RateLimiterRight = new SlewRateLimiter(5);

    public DriveControls() {
    }

    // sets controls for driving
    // public void DriveInput(double xSpeed, double ySpeed) {
    //     xAxis = m_Controller.getRightX();
    //     yAxis = m_Controller.getRightY();

    //     xSpeed = m_RateLimiterRight.calculate(xAxis) * 0.85;
    //     ySpeed = m_RateLimiterLeft.calculate(yAxis);
    // }
}
