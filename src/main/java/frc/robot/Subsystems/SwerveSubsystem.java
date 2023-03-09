package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModuleCoaxial frontLeft = new SwerveModuleCoaxial(
        DriveConstants.kFrontLeftDriveMotorPort, 
        DriveConstants.kFrontLeftTurnMotorPort, 
        DriveConstants.kFrontLeftAbsoluteEncoderReversed, 
        DriveConstants.kFrontLeftAbsoluteEncoderOffset, 
        DriveConstants.kFrontLeftDriveMotorReversed, 
        DriveConstants.kFrontLeftTurnMotorReversed, 
        DriveConstants.kFrontLeftAbsoluteEncoderPort
    );
    
    private final SwerveModuleCoaxial frontRight = new SwerveModuleCoaxial(
        DriveConstants.kFrontRightDriveMotorPort, 
        DriveConstants.kFrontRightTurnMotorPort, 
        DriveConstants.kFrontRightAbsoluteEncoderReversed, 
        DriveConstants.kFrontRightAbsoluteEncoderOffset, 
        DriveConstants.kFrontRightDriveMotorReversed, 
        DriveConstants.kFrontRightTurnMotorReversed, 
        DriveConstants.kFrontRightAbsoluteEncoderPort
    );

    private final SwerveModuleCoaxial backLeft = new SwerveModuleCoaxial(
        DriveConstants.kBackLeftDriveMotorPort, 
        DriveConstants.kBackLeftTurnMotorPort, 
        DriveConstants.kBackLeftAbsoluteEncoderReversed, 
        DriveConstants.kBackLeftAbsoluteEncoderOffset, 
        DriveConstants.kBackLeftDriveMotorReversed, 
        DriveConstants.kBackLeftTurnMotorReversed, 
        DriveConstants.kBackLeftAbsoluteEncoderPort
    );

    private final SwerveModuleCoaxial backRight = new SwerveModuleCoaxial(
        DriveConstants.kBackRightDriveMotorPort, 
        DriveConstants.kBackRightTurnMotorPort, 
        DriveConstants.kBackRightAbsoluteEncoderReversed, 
        DriveConstants.kBackRightAbsoluteEncoderOffset, 
        DriveConstants.kBackRightDriveMotorReversed, 
        DriveConstants.kBackRightTurnMotorReversed, 
        DriveConstants.kBackRightAbsoluteEncoderPort
    );

    private final AHRS navX = new AHRS(SerialPort.Port.kMXP);

    private SwerveDriveOdometry kSwerveDriveOdometry;

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
                kSwerveDriveOdometry = new SwerveDriveOdometry(Constants.DriveConstants.kDriveKinematics, getRotation2d(), getModulePositions());
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        navX.reset();
    }

    public double getHeading() {
        return navX.getYaw();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose2d() {
        return kSwerveDriveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        kSwerveDriveOdometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        kSwerveDriveOdometry.update(getRotation2d(), getModulePositions());

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Front Left Module Velocity", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("Front Right Module Velocity", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("Back Left Module Velocity", backLeft.getDriveVelocity());
        SmartDashboard.putNumber("Back Right Module Velocity", backRight.getDriveVelocity());
        SmartDashboard.putString("Robot Location", getPose2d().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModulesStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }    

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurnPosition())),
            new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurnPosition())),
            new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurnPosition())),
            new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurnPosition()))
        };
    }
}