package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
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

    private SwerveDriveOdometry kSwerveDriveOdometry = new SwerveDriveOdometry(Constants.DriveConstants.kDriveKinematics, new Rotation2d(), getModulePositions());;

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(10000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        navX.zeroYaw();
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
        SmartDashboard.putNumber("Front Left Position", frontLeft.turnEncoder.getPosition());
        SmartDashboard.putNumber("Front Right Position", frontRight.turnEncoder.getPosition());
        SmartDashboard.putNumber("Back Left Position", backLeft.turnEncoder.getPosition());
        SmartDashboard.putNumber("Back Right Position", backRight.turnEncoder.getPosition());   
        SmartDashboard.putString("Front Left MagField", frontLeft.getMagnetStrenght().name());
        SmartDashboard.putString("Front Right MagField", frontRight.getMagnetStrenght().name());
        SmartDashboard.putString("Back Left MagField", backLeft.getMagnetStrenght().name());
        SmartDashboard.putString("Back Right MagField", backRight.getMagnetStrenght().name()); 
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

    public void setOdometryPosition(Pose2d startLocation) {
        kSwerveDriveOdometry.resetPosition(getRotation2d(), getModulePositions(), startLocation);
    }
}