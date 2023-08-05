package frc.robot.Subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.LimelightSubsystem.LimelightOptions.CamMode;
import frc.robot.Subsystems.LimelightSubsystem.LimelightOptions.LEDState;

public class LimelightSubsystem extends SubsystemBase {
    public static class LimelightOptions {
        public static enum LEDState {
            PIPELINE,
            OFF,
            BLINK,
            ON;
        }

        public static enum CamMode {
            VISION_PROCESSOR,
            DRIVER_CAMERA;
        }
    }

    public CamMode currCamMode = CamMode.VISION_PROCESSOR;
    public LEDState currLedState = LEDState.PIPELINE;
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");  
    ArrayList<NetworkTableEntry> collectedEntries = new ArrayList<>();

    public LimelightSubsystem(String... entries) {
        for(String entry : entries) {
            collectedEntries.add(limelightTable.getEntry(entry));
        }
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("tx", getEntryAsDouble("tx"));
        SmartDashboard.putNumber("ta", getEntryAsDouble("ta"));

        /*for(NetworkTableEntry entry : collectedEntries) {
            SmartDashboard.putNumber(entry.getName(), entry.getDouble(0.0));
        }*/
    }

    public void setPipeline(double pipelineIndex) {
        writeNetworkTableEntry("pipeline", pipelineIndex);
    }

    public void setLedMode(LEDState lState) {
        double ledValue = 0;
        currLedState = lState;

        if(lState == LEDState.OFF) {
            ledValue = 1;
        }
        else if(lState == LEDState.BLINK) {
            ledValue = 2;
        }
        else if(lState == LEDState.ON) {
            ledValue = 3;
        }

        writeNetworkTableEntry("ledMode", ledValue);
    }

    public void setCamMode(CamMode cMode) {
        double camValue = 0;
        currCamMode = cMode;

        if(cMode == CamMode.DRIVER_CAMERA) {
            camValue = 1;
        }

        writeNetworkTableEntry("camMode", camValue);
    }

    public void writeNetworkTableEntry(String entry, double value) {
        limelightTable.putValue(entry, NetworkTableValue.makeDouble(value));
    }

    public NetworkTableEntry getNetworkTableEntry(String entry) {
        return limelightTable.getEntry(entry);
    }

    public double getEntryAsDouble(String entry) {
        return limelightTable.getEntry(entry).getDouble(0.0);
    }

    public ArrayList<NetworkTableEntry> getCollectedEntries() {
        return collectedEntries;
    }
}
