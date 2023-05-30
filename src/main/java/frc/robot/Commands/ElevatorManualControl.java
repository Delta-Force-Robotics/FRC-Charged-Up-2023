package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ElevatorSubsystem;

public class ElevatorManualControl extends CommandBase {
    private ElevatorSubsystem elevatorSubsystem;
    private DoubleSupplier rightTrigger;
    private DoubleSupplier leftTrigger;


    public ElevatorManualControl(ElevatorSubsystem elevatorSubsystem, DoubleSupplier rightTrigger, DoubleSupplier leftTrigger) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.rightTrigger = rightTrigger;
        this.leftTrigger = leftTrigger;

        this.addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        if(Math.abs(rightTrigger.getAsDouble() - leftTrigger.getAsDouble()) >= 0.1) {
            elevatorSubsystem.setMotorPower(rightTrigger.getAsDouble() - leftTrigger.getAsDouble());
        }
        else {
            elevatorSubsystem.setMotorPower(0);
        }
    }
}
