package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class SwerveModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / 21.4285;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kDriveMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static double kPTurning = 0.5;

        public static final double kCPRAbsoluteEncoder = 4096;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(21.805);
        public static final double kWheelBase = Units.inchesToMeters(22.909);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2)
        );

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;

        public static final int kFrontLeftDriveMotorPort = 0;
        public static final int kFrontRightDriveMotorPort = 2;
        public static final int kBackLeftDriveMotorPort = 4;
        public static final int kBackRightDriveMotorPort = 6; 

        public static final int kFrontLeftTurnMotorPort = 1;
        public static final int kFrontRightTurnMotorPort = 3;
        public static final int kBackLeftTurnMotorPort = 5;
        public static final int kBackRightTurnMotorPort = 7;

        public static final boolean kFrontLeftAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftAbsoluteEncoderReversed = false;
        public static final boolean kBackRightAbsoluteEncoderReversed = false;

        public static final double kFrontLeftAbsoluteEncoderOffset = 0;
        public static final double kFrontRightAbsoluteEncoderOffset = 0;
        public static final double kBackLeftAbsoluteEncoderOffset = 0;
        public static final double kBackRightAbsoluteEncoderOffset = 0;

        public static final boolean kFrontLeftDriveMotorReversed = false;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kBackLeftDriveMotorReversed = false;
        public static final boolean kBackRightDriveMotorReversed = false;

        public static final boolean kFrontLeftTurnMotorReversed = false;
        public static final boolean kFrontRightTurnMotorReversed = false;
        public static final boolean kBackLeftTurnMotorReversed = false;
        public static final boolean kBackRightTurnMotorReversed = false;
        
        public static final int kFrontLeftAbsoluteEncoderPort = 0;
        public static final int kFrontRightAbsoluteEncoderPort = 0;
        public static final int kBackLeftAbsoluteEncoderPort = 0;
        public static final int kBackRightAbsoluteEncoderPort = 0;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = 5;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 5;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = Math.toRadians(360);
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = Math.toRadians(360);

        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kS = 0;
        public static final double kPVel = 0;
        public static final double kIVel = 0;
        public static final double kDVel = 0;
    }

    public static final class ArmConstants {
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kCos = 0;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kArmMotorGearRatio = 0;
        public static final double kArmMotorTicksToRadians = kArmMotorGearRatio * Math.PI * 2;
        public static final double kArmMotorRPMToRadiansPerSec = kArmMotorTicksToRadians / 60;
        public static final double kArmEncoderOffset = 0;

        public static final double kArmFeedBackPositionTolerance = Math.toRadians(1); // in Radians
        public static final double kArmFeedBackVelocityTolerance = Math.toRadians(0); // in Radians
    }

    public static final class CANConstants {
        public static int kLeftArmMotor = 8;
        public static int kRightArmMotor = 9;
        public static int kClawPivotMotor = 10;
        public static int kClawWheelMotor = 11;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverXAxis = 0;
        public static final int kDriverYAxis = 1;
        public static final int kDriverRotAxis = 2;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.1;
    }
}
