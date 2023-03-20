package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ArmSubsystem;

public class ArmManualControl extends CommandBase {
    private ArmSubsystem armSubsystem;
    private DoubleSupplier rightTrigger;
    private DoubleSupplier leftTrigger;


    public ArmManualControl(ArmSubsystem armSubsystem, DoubleSupplier rightTrigger, DoubleSupplier leftTrigger) {
        this.armSubsystem = armSubsystem;
        this.rightTrigger = rightTrigger;
        this.leftTrigger = leftTrigger;

        this.addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        if(Math.abs(rightTrigger.getAsDouble() - leftTrigger.getAsDouble()) >= 0.15) {
            armSubsystem.setMotorPower(rightTrigger.getAsDouble() - leftTrigger.getAsDouble());
        }
        else {
            armSubsystem.setMotorPower(0);
        }
    }
}
