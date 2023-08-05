package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Constants.DriveConstants;
import frc.robot.Subsystems.SwerveSubsystem;

public class ParkCommand extends CommandBase {
    private SwerveSubsystem swerveSubsystem;

    public ParkCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override 
    public void execute() {
        if(Math.abs(swerveSubsystem.getInclination()) >= 2) {
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.35*(swerveSubsystem.getInclination()/15), 0, 0, swerveSubsystem.getRotation2d());
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
