package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Constants.DriveConstants;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;

public class AutoFollowingCommand extends CommandBase{
    private SwerveSubsystem swerveSubsystem;
    private LimelightSubsystem limelightSubsystem;

    public AutoFollowingCommand(SwerveSubsystem swerveSubsystem, LimelightSubsystem limelightSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
    }

    @Override
    public void execute() {
        if(limelightSubsystem.getEntryAsDouble("ta") > 1.1 && limelightSubsystem.getEntryAsDouble("tx") != 1) {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-1 * (limelightSubsystem.getEntryAsDouble("ta") / 5), 0, 3 * (Math.toRadians(limelightSubsystem.getEntryAsDouble("tx") / 1)), Rotation2d.fromDegrees(0));
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModulesStates(moduleStates);
    }

    else if(limelightSubsystem.getEntryAsDouble("ta") < 0.9 && limelightSubsystem.getEntryAsDouble("ta") > 0 && limelightSubsystem.getEntryAsDouble("tx") != 1) {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(1 * ((1 - limelightSubsystem.getEntryAsDouble("ta")) / 3), 0, 3 * (Math.toRadians(limelightSubsystem.getEntryAsDouble("tx") / 1)), Rotation2d.fromDegrees(0));
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModulesStates(moduleStates);
    }

    else {
        swerveSubsystem.stopModules();
    }}

    @Override
    public boolean isFinished() {
        return false;
    }
}
