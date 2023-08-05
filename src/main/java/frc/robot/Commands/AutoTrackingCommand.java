package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Constants.DriveConstants;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;

public class AutoTrackingCommand extends CommandBase {
    private SwerveSubsystem swerveSubsystem;
    private LimelightSubsystem limelightSubsystem = new LimelightSubsystem("tx");

    public AutoTrackingCommand(SwerveSubsystem swerveSubsystem, LimelightSubsystem limnLimelightSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        //addRequirements(swerveSubsystem, limelightSubsystem);
    }

    @Override
    public void execute() {
    if(limelightSubsystem.getEntryAsDouble("tx") != 1) {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 3 * (Math.toRadians(limelightSubsystem.getEntryAsDouble("tx") / 1)), swerveSubsystem.getRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModulesStates(moduleStates);
    }
    
    else {
        swerveSubsystem.stopModules();
    }

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
