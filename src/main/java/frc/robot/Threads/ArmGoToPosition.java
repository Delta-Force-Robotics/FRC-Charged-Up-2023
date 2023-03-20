package frc.robot.Threads;

import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.ClawSubsystem;
import frc.robot.Subsystems.ArmSubsystem.ArmPosition;

public class ArmGoToPosition extends Thread {
    private ArmSubsystem armSubsystem;
    public double armPosition = 0;
    public ArmPosition armState;

    public ArmGoToPosition(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    public void run() {
        try {
            armSubsystem.goToPosition(armPosition);
        } catch (InterruptedException e) {
            armSubsystem.setCurrArmState(armState);
        }
        armSubsystem.setCurrArmState(armState);
    }
}