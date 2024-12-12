// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class SwerveSubsystem extends SubsystemBase {
    /** Creates a new Subsystem. */

    File directory = new File(Filesystem.getDeployDirectory(), "swerve");
    SwerveDrive swerveDrive;

    public SwerveSubsystem() {
        try{
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED);
        } catch (Exception e){
            throw new RuntimeException(e);
        }
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
            /* one-time action goes here */
            }
        );
    }

    /**
     * Command to drive the robot using translative values and heading as angular velocity.
     *
     * from https://docs.yagsl.com/configuring-yagsl/code-setup
     *
     * @param translationX     Translation in the X direction.
     * @param translationY     Translation in the Y direction.
     * @param angularRotationX Angular velocity of the robot to set.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
    {
        return run(
            () -> {
                // Make the robot move
                swerveDrive.drive(
                    SwerveMath.scaleTranslation(
                        new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumVelocity()
                        ),
                        0.8
                    ),
                    angularRotationX.getAsDouble()* swerveDrive.getMaximumAngularVelocity(),
                    true,
                    false
                );
            }
        );
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
