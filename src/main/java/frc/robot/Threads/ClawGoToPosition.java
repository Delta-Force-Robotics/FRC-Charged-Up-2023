package frc.robot.Threads;

import frc.robot.Subsystems.ClawSubsystem;

public class ClawGoToPosition extends Thread {
    private ClawSubsystem clawSubsystem;
    public double clawPosition = 0;

    public ClawGoToPosition(ClawSubsystem clawSubsystem, double clawToPosition) {
        this.clawSubsystem = clawSubsystem;
        this.clawPosition = clawToPosition;
    }

    public void run() {
        clawSubsystem.goToPosition(clawPosition);
    }
}