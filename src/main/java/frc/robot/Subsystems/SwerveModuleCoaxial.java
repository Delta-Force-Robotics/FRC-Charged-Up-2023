
package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Constants.DriveConstants;
import frc.robot.Constants.Constants.SwerveModuleConstants;
import frc.robot.Util.PIDFController;

public class SwerveModuleCoaxial {

    public final CANSparkMax driveMotor;
    public final CANSparkMax turnMotor;

    public final RelativeEncoder driveEncoder;
    public final RelativeEncoder turnEncoder;

    private final PIDController turningPidController;
    private final PIDFController velocityController;
    private final SimpleMotorFeedforward motorFeedforward;

    public final CANCoder absoluteEncoder;
    public final boolean absoluteEncoderReversed;
    public double absoluteEncoderOffsetRad;

    public SwerveModuleCoaxial (int driveMotorID, int turnMotorID, boolean absoluteEncoderReversed, double absoluteEncoderOffset, 
                boolean driveMotorReversed, boolean turnMotorReversed, int absoluteEncoderID) {
        
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
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

        turningPidController = new PIDController(SwerveModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        
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
        double angle = Math.toRadians(absoluteEncoder.getAbsolutePosition()) - absoluteEncoderOffsetRad;

        if(angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        else if (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }

        return angle * (absoluteEncoderReversed ? -1 : 1);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if(Math.abs(state.speedMetersPerSecond) < 0.05) {
            stop();
        }

        state = SwerveModuleState.optimize(state, getState().angle);

        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        turnMotor.set(turningPidController.calculate(getTurnPosition(), state.angle.getRadians()));
    }

    public void setDesiredStateAuton(SwerveModuleState state) {
        if(Math.abs(state.speedMetersPerSecond) < 0.1) {
            stop();
        }

        state = SwerveModuleState.optimize(state, getState().angle);

        driveMotor.setVoltage(motorFeedforward.calculate(state.speedMetersPerSecond) + velocityController.calculate(getDriveVelocity(), state.speedMetersPerSecond));

        turnMotor.set(turningPidController.calculate(getTurnPosition(), state.angle.getRadians()));
    }

    public MagnetFieldStrength getMagnetStrenght() {
        return absoluteEncoder.getMagnetFieldStrength();
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}