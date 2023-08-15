package frc.robot.Commands;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Constants.CameraConstants;
import frc.robot.Constants.Constants.DriveConstants;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Util.VisionTarget;

public class AutoFollowingCommand extends CommandBase { 
    private SwerveSubsystem swerveSubsystem;
    private LimelightSubsystem limelightSubsystem;
    private VisionTarget visionTarget;
    private PIDController headingController;
    private PIDController xController;
    private PIDController yController;
    //private PIDController distanceController;

    /*private double DIAG_FOV = CameraConstants.DIAG_FOV;
    private double HORIZ_FOV = CameraConstants.HORIZ_FOV;
    private double VERT_FOV = CameraConstants.VERT_FOV;

    private double alpha = Math.cos(Math.toRadians(CameraConstants.HORIZ_FOV));
    private double beta = Math.cos(Math.toRadians(CameraConstants.VERT_FOV));
    private double theta = Math.cos(Math.toRadians(CameraConstants.DIAG_FOV));*/

    public AutoFollowingCommand(SwerveSubsystem swerveSubsystem, LimelightSubsystem limelightSubsystem /*, PIDController distanceController */ , PIDController headingController, PIDController xController, PIDController yController, VisionTarget visionTarget) {
        this.swerveSubsystem = swerveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.visionTarget = visionTarget;
        this.headingController = headingController;
        this.xController = xController;
        this.yController = yController;
    }

    @Override
    public void execute() {
        double tx = limelightSubsystem.getEntryAsDouble("tx");
        double ty = limelightSubsystem.getEntryAsDouble("ty");
        double ta = limelightSubsystem.getEntryAsDouble("ta");
        double[] aprilTagPose = limelightSubsystem.getNetworkTableEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

        /*double tau = Math.sqrt(tx*tx + ty*ty);
        double deltaY = Math.sqrt( //the other version named it "distance"
            Math.sqrt(
                (visionTarget.getArea() * 100 * theta) / (4 * ta * (1 - alpha) * (1 - beta))
            )
        ); // Distance to the geometric plane defined by the vision target*/

        double deltaY = Math.cos(Math.toRadians(aprilTagPose[4])) * aprilTagPose[2];
        double deltaX = Math.sin(Math.toRadians(aprilTagPose[4])) * aprilTagPose[2];

        SmartDashboard.putNumber("deltaX", deltaX);
        SmartDashboard.putNumber("deltaY", deltaY);

        //ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xController.calculate(deltaX), yController.calculate(deltaY), headingController.calculate(tx));
        //SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        //swerveSubsystem.setModuleStatesAuton(moduleStates);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
