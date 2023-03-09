
package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.Constants.DriveConstants;
import frc.robot.Constants.Constants.SwerveModuleConstants;
import frc.robot.Util.PIDFController;

public class SwerveModuleCoaxial {

    public final CANSparkMax driveMotor;
    public final CANSparkMax turnMotor;

    public final RelativeEncoder driveEncoder;
    public final RelativeEncoder turnEncoder;

    private final PIDFController turningPidController;
    private final PIDFController velocityController;
    private final SimpleMotorFeedforward motorFeedforward;

    public final CANCoder absoluteEncoder;
    public final boolean absoluteEncoderReversed;
    public final double absoluteEncoderOffsetRad;

    public SwerveModuleCoaxial (int driveMotorID, int turnMotorID, boolean absoluteEncoderReversed, double absoluteEncoderOffset, 
                boolean driveMotorReversed, boolean turnMotorReversed, int absoluteEncoderID) {
        
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset / SwerveModuleConstants.kCPRAbsoluteEncoder  * 2.0 * Math.PI;
        absoluteEncoder = new CANCoder(absoluteEncoderID);
        
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(SwerveModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveEncoderRPM2MeterPerSec);

        turnEncoder.setPositionConversionFactor(SwerveModuleConstants.kTurningEncoderRot2Rad);
        turnEncoder.setVelocityConversionFactor(SwerveModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDFController(SwerveModuleConstants.kPTurning, 0, 0);
        turningPidController.setContinuous();
        turningPidController.setInputRange(-Math.PI, Math.PI);

        velocityController = new PIDFController(DriveConstants.kPVel, DriveConstants.kIVel, DriveConstants.kDVel);
        motorFeedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

        resetEncoders();
    }

    public double getDrivePosition () {
        return driveEncoder.getPosition();
    }

    public double getTurnPosition () {
        return turnEncoder.getPosition();
    }

    public double getDriveVelocity () {
        return driveEncoder.getVelocity();
    }

    public double getTurnVelocity () {
        return turnEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad(){
        double angle = absoluteEncoder.getAbsolutePosition() / SwerveModuleConstants.kCPRAbsoluteEncoder;
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;

        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if(state.speedMetersPerSecond < 0.001) {
            stop();
        }

        state = SwerveModuleState.optimize(state, getState().angle);

        driveMotor.setVoltage(motorFeedforward.calculate(state.speedMetersPerSecond) + velocityController.calculate(getDriveVelocity(), state.speedMetersPerSecond));

        turningPidController.setSetpoint(state.angle.getRadians());
        turnMotor.set(turningPidController.calculate(getTurnPosition()));
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}