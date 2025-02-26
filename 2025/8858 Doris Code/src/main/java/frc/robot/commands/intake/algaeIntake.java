package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.AlgaeSubsystem;

public class algaeIntake extends Command {
    private final AlgaeSubsystem algaeSubsystem;
    private final double speed;

    public algaeIntake(AlgaeSubsystem algaeSubsystem, double speed) {
        this.algaeSubsystem = algaeSubsystem;
        this.speed = speed;
        addRequirements(algaeSubsystem); // add requirement so that multiple commands using the same subsystem don't run at the same time
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        algaeSubsystem.algaeIntake(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (speed < 0) {
            algaeSubsystem.algaeIntake(-0.05);
        } else {
            algaeSubsystem.algaeIntake(0);
        }
    }
}