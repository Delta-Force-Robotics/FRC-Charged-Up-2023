package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ElevatorSubsystem;

public class ElevatorVoltageCommand extends CommandBase{
            private ElevatorSubsystem elevatorSubsystem;
            private double motorVoltageElevator = 0;

            public ElevatorVoltageCommand( ElevatorSubsystem elevatorSubsystem) {
                this.elevatorSubsystem = elevatorSubsystem;
                addRequirements(elevatorSubsystem);
            }
            
            @Override
            public void execute() {
            motorVoltageElevator = Math.min(motorVoltageElevator + 0.1, 1);

            double zeroTimeElevator = Timer.getFPGATimestamp();
            while(Timer.getFPGATimestamp() - zeroTimeElevator <= 1) {
            }
            
            elevatorSubsystem.setMotorPower(motorVoltageElevator);
        }
}
