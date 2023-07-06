package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Constants.IntakeConstants;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.IntakeSubsystem.WheelDirection;

public class AutoRetractCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private LEDSubsystem ledSubsystem;

    private double lastCurrent = 0;

    public AutoRetractCommand (IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, LEDSubsystem ledSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.ledSubsystem = ledSubsystem;

        addRequirements(this.intakeSubsystem, this.elevatorSubsystem, this.ledSubsystem);
    }

    @Override
    public void execute() {
        if(intakeSubsystem.getOutputCurrent() > 38 && lastCurrent <= 38) {
            intakeSubsystem.setSetpoint(IntakeConstants.kPivotAngleRadHome);
            elevatorSubsystem.setSetpoint(0);
            intakeSubsystem.setWheelDirection(WheelDirection.OFF);

            ElevatorSubsystem.isHome = true;
            ElevatorSubsystem.isScore = false;
            ledSubsystem.setIsElementInside(true);
        }

        lastCurrent = intakeSubsystem.getOutputCurrent();

    } 
}
