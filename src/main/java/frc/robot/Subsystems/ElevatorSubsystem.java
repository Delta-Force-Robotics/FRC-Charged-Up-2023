package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    private CANSparkMax leftMotor = new CANSparkMax(Constants.CANConstants.kLeftElevatorMotor, MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(Constants.CANConstants.kRightElevatorMotor, MotorType.kBrushless);
    private RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private RelativeEncoder rightEncoder = rightMotor.getEncoder();

    private ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kCos, ElevatorConstants.kV, ElevatorConstants.kA);

    private PIDController leftElevatorFeedback = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    private PIDController rightElevatorFeedback = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

    private BooleanSupplier isInterrupted;

    private double setPoint = 0;

    private ElevatorPosition desiredElevatorPosition = ElevatorPosition.HOME;
    private ElevatorPosition currElevatorPosition = ElevatorPosition.HOME;

    public ElevatorSubsystem() {
        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

        leftEncoder.setPositionConversionFactor(ElevatorConstants.kElevatorMotorTicksToCm);
        rightEncoder.setPositionConversionFactor(ElevatorConstants.kElevatorMotorTicksToCm);

        leftEncoder.setVelocityConversionFactor(ElevatorConstants.kElevatorMotorRPMToCmPerSec);
        rightEncoder.setVelocityConversionFactor(ElevatorConstants.kElevatorMotorRPMToCmPerSec);

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        rightElevatorFeedback.setIntegratorRange(-1, 1);
        leftElevatorFeedback.setIntegratorRange(-1, 1);

        rightElevatorFeedback.setTolerance(ElevatorConstants.kElevatorFeedBackPositionTolerance,
                ElevatorConstants.kElevatorFeedBackVelocityTolerance);
        leftElevatorFeedback.setTolerance(ElevatorConstants.kElevatorFeedBackPositionTolerance,
                ElevatorConstants.kElevatorFeedBackVelocityTolerance);
    }

    public void setInterruptedCondition(BooleanSupplier isInterrupted) {
        this.isInterrupted = isInterrupted;
    }

    @Override
    public void periodic() {

        setMotorVoltageByPosition(setPoint);
    }

    public double[] getPosition() {
        return new double[] { leftEncoder.getPosition(), rightEncoder.getPosition() };
    }

    public double getSetPoint() {
        return setPoint;
    }

    public void setSetpoint(double elevatorPosMeters) {
        if(setPoint - elevatorPosMeters <= 0) {
            leftElevatorFeedback.setP(ElevatorConstants.kP);
            rightElevatorFeedback.setP(ElevatorConstants.kP);

            leftElevatorFeedback.setI(ElevatorConstants.kI);
            rightElevatorFeedback.setI(ElevatorConstants.kI);

            leftElevatorFeedback.setD(ElevatorConstants.kD);
            rightElevatorFeedback.setD(ElevatorConstants.kD);
        } else {
            leftElevatorFeedback.setP(ElevatorConstants.kRetractP);
            rightElevatorFeedback.setP(ElevatorConstants.kRetractP);

            leftElevatorFeedback.setI(ElevatorConstants.kRetractI);
            rightElevatorFeedback.setI(ElevatorConstants.kRetractI);

            leftElevatorFeedback.setD(ElevatorConstants.kRetractD);
            rightElevatorFeedback.setD(ElevatorConstants.kRetractD);
        }
        setPoint = elevatorPosMeters;
    }

    public void setMotorVoltageByPosition(double elevatorVelocitySetPoint) {
        leftMotor.setVoltage(
                elevatorFeedforward.calculate(leftEncoder.getVelocity()) 
                + leftElevatorFeedback.calculate(leftEncoder.getPosition(), elevatorVelocitySetPoint));
        rightMotor.setVoltage(
                elevatorFeedforward.calculate(rightEncoder.getVelocity())
                + rightElevatorFeedback.calculate(rightEncoder.getPosition(), elevatorVelocitySetPoint));
    } 

    public double getElevatorPosition() {
        return leftEncoder.getPosition();
    }

    public double getFeedForwardPower() {
        return elevatorFeedforward.calculate(leftEncoder.getVelocity());
    }

    public void setMotorPower(double motorPower) {
        leftMotor.set(motorPower);
        rightMotor.set(motorPower);
    }

    public void setMotorVoltage(double motorVoltage) {
        leftMotor.setVoltage(motorVoltage);
        rightMotor.setVoltage(motorVoltage);
    }

    public ElevatorPosition getDesiredElevatorPosition() {
        return desiredElevatorPosition;
    }

    public void setCurrElevatorPosition(ElevatorPosition elevatorPosition) {
        currElevatorPosition = elevatorPosition;
    }

    public void setDesiredElevatorPosition(ElevatorPosition elevatorPosition) {
        desiredElevatorPosition = elevatorPosition;
    }

    public ElevatorPosition getCurrElevatorPosition() {
        return currElevatorPosition;
    }

    public static enum ElevatorPosition {
        HOME,
        INTAKE,
        LOW,
        MID,
        HIGH;
    }
    
}
