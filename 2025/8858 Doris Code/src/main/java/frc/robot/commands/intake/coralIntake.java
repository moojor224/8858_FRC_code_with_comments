package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.CoralIntakeSubsystem;

public class coralIntake extends Command {
    private final CoralIntakeSubsystem coralIntakeSubsystem;
    private final double speed;

    public coralIntake(CoralIntakeSubsystem coralIntakeSubsystem, double speed) {
        this.coralIntakeSubsystem = coralIntakeSubsystem;
        this.speed = speed;
        addRequirements(coralIntakeSubsystem);
    }

    @Override
    public void execute() {
        coralIntakeSubsystem.coralIntake(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        coralIntakeSubsystem.coralIntake(0);
    }
}