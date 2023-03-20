package frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class AutoConstants {
        public static final Pose2d kLeftStartPose = new Pose2d(new Translation2d(-1, -1), new Rotation2d(Math.toRadians(-1)));
        public static final Pose2d kMidStartPose = new Pose2d(new Translation2d(-1, -1), new Rotation2d(Math.toRadians(-1)));
        public static final Pose2d kRightStartPose = new Pose2d(new Translation2d(-1, -1), new Rotation2d(Math.toRadians(-1)));
    }

    public static final class SwerveModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / 21.4285;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static double kPTurning = 0.3;

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

        public static final double kPhysicalMaxSpeedMetersPerSecond = 2;
        public static final double kPhysicalMaxAccelMetersPerSecondSq = 2;
        public static final double kPhysicalMaxAngSpeedRadPerSecond = Math.toRadians(180);
        public static final double kPhysicalMaxAngAccelRadPerSecondSq = Math.toRadians(180);

        public static final int kFrontLeftDriveMotorPort = 11;
        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kBackLeftDriveMotorPort = 8;
        public static final int kBackRightDriveMotorPort = 5; 

        public static final int kFrontLeftTurnMotorPort = 10;
        public static final int kFrontRightTurnMotorPort = 2;
        public static final int kBackLeftTurnMotorPort = 9;
        public static final int kBackRightTurnMotorPort = 4;

        public static final boolean kFrontLeftAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftAbsoluteEncoderReversed = false;
        public static final boolean kBackRightAbsoluteEncoderReversed = false;

        public static final double kFrontLeftAbsoluteEncoderOffset = -Math.toRadians(121.641);
        public static final double kFrontRightAbsoluteEncoderOffset = -Math.toRadians(31.232);
        public static final double kBackLeftAbsoluteEncoderOffset = Math.toRadians(37.354);
        public static final double kBackRightAbsoluteEncoderOffset = Math.toRadians(60.382);

        public static final boolean kFrontLeftDriveMotorReversed = false;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kBackLeftDriveMotorReversed = false;
        public static final boolean kBackRightDriveMotorReversed = false;

        public static final boolean kFrontLeftTurnMotorReversed = false;
        public static final boolean kFrontRightTurnMotorReversed = false;
        public static final boolean kBackLeftTurnMotorReversed = false;
        public static final boolean kBackRightTurnMotorReversed = false;
        
        public static final int kFrontLeftAbsoluteEncoderPort = 15;
        public static final int kFrontRightAbsoluteEncoderPort = 14;
        public static final int kBackLeftAbsoluteEncoderPort = 16;
        public static final int kBackRightAbsoluteEncoderPort = 13;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = 5;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 5;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = Math.toRadians(360);
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = Math.toRadians(360);

        public static final double kV = 2.6284;
        public static final double kA = 0.1677;
        public static final double kS = 0.19357;
        public static final double kPVel = 4;
        public static final double kIVel = 0;
        public static final double kDVel = 0;

        public static final double kPTheta = 4;
        public static final double kPX = 4;
        public static final double kPY = 4;
    }

    public static final class ArmConstants {
        public static final double kS = 0.57721;
        public static final double kV = 0.76381;
        public static final double kA = 0.10614;
        public static final double kCos = 0.15614;

        public static final double kP = 0.94539;
        public static final double kI = 0;
        public static final double kD = 0.24637;

        public static final double kArmMotorGearRatio = 1.0 / 80.0;
        public static final double kArmMotorTicksToRadians = kArmMotorGearRatio * Math.PI * 2;
        public static final double kArmMotorRPMToRadiansPerSec = kArmMotorTicksToRadians / 60;
        public static final double kArmEncoderOffset = Math.toRadians(-90);

        public static final double kArmFeedBackPositionTolerance = Math.toRadians(1); // in Radians
        public static final double kArmFeedBackVelocityTolerance = Math.toRadians(1); // in Radians

        public static final double armAngleRadConeLow = Math.toRadians(-70);
        public static final double armAngleRadCubeLow = Math.toRadians(-70);

        public static final double armAngleRadConeMid = Math.toRadians(-30);
        public static final double armAngleRadCubeMid = Math.toRadians(-30);
        
        public static final double armAngleRadConeHigh = Math.toRadians(-12);
        public static final double armAngleRadCubeHigh = Math.toRadians(-10);

        public static final double armAngleRadIntake = 0;
        public static final double armAngleRadHome = kArmEncoderOffset;

        public static boolean isHome = true;
        public static boolean isScore = false;
    }

    public static final class ClawConstants {
        public static final double kP = 0.3;
        public static final double kI = 0;
        public static final double kD = 0.05;

        public static final double kS = 0.1;
        public static final double kV = 0.44151;
        public static final double kA = 0.040624;
        public static final double kCos = 1.8869;

        public static final double kPivotMotorGearRatio = 1 / 20.0;
        public static final double kPivotMotorTicksToRadians = kPivotMotorGearRatio * Math.PI * 2;
        public static final double kPivotMotorRPMToRadiansPerSecond = kPivotMotorTicksToRadians / 60;
        public static final double kPivotEncoderOffset = Math.toRadians(45);

        public static final double kPivotFeedbackPositionTolerance = Math.toRadians(4);
        public static final double kPivotFeedbackVelocityTolerance = Math.toRadians(4);

        public static final double clawAngleRadScoreCube = Math.toRadians(30);
        public static final double clawAngleRadScoreCone = Math.toRadians(45);
        public static final double clawAngleRadIntake = Math.toRadians(0);

        public static final double kAngleRadHome = kPivotEncoderOffset;
    }

    public static final class CANConstants {
        public static int kLeftArmMotor = 7;
        public static int kRightArmMotor = 6;
        public static int kClawPivotMotor = 17;
        public static int kClawWheelMotor = 12;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kDriver2ControllerPort = 1;

        public static final int kDriverXAxis = 0;
        public static final int kDriverYAxis = 1;
        public static final int kDriverRotAxis = 2;
        public static final int kDriverFieldOrientedButtonIdx = 4;

        public static final double kDeadband = 0.1;
    }
}
