package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.IntakeSubsystem;

public class algaeIntake extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double speed;

    public algaeIntake(IntakeSubsystem intakeSubsystem, double speed) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
        intakeSubsystem.resetPID();
    }

    @Override
    public void execute(){
        intakeSubsystem.algaeIntake(speed);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.algaeIntake(-0.1);
    }
}