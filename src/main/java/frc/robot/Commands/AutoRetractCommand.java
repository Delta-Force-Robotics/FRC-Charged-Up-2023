package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Constants.IntakeConstants;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

public class AutoRetractCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private ElevatorSubsystem elevatorSubsystem;

    private double lastCurrent = 0;

    public AutoRetractCommand (IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(this.intakeSubsystem, this.elevatorSubsystem);
    }

    @Override
    public void execute() {
        if(intakeSubsystem.getOutputCurrent() > 30 && lastCurrent <= 30) {
            intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
            elevatorSubsystem.setSetpoint(0);
        }

        lastCurrent = intakeSubsystem.getOutputCurrent();
    }

    
}
