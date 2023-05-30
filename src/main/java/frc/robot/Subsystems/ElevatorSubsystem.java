package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    private CANSparkMax leftMotor = new CANSparkMax(Constants.CANConstants.kLeftElevatorMotor, MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(Constants.CANConstants.kRightElevatorMotor, MotorType.kBrushless);
    private RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private AbsoluteEncoder absoluteEncoder = rightMotor.getAbsoluteEncoder(Type.kDutyCycle);

    private double motionStartTime = Timer.getFPGATimestamp();
    private TrapezoidProfile motionProfile = getTrapezoidProfile(0);
    private ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
    private PIDController leftElevatorFeedback = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    private PIDController rightElevatorFeedback = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

    private double setPoint = 0;

    private ElevatorPosition desiredElevatorPosition = ElevatorPosition.HOME;
    private ElevatorPosition currElevatorPosition = ElevatorPosition.HOME;

    public IntakeGameElement desiredIntakeGameElement = IntakeGameElement.GROUND_CONE_UP;

    public boolean isElementInside = false;

    public ElevatorSubsystem() {
        leftMotor.setInverted(false);
        rightMotor.setInverted(false);

        leftMotor.setSmartCurrentLimit(40);
        rightMotor.setSmartCurrentLimit(40);

        absoluteEncoder.setInverted(false);

        absoluteEncoder.setPositionConversionFactor(ElevatorConstants.kElevatorMotorTicksToM);
        absoluteEncoder.setVelocityConversionFactor(ElevatorConstants.kElevatorMotorRPMToMPerSec);

        rightElevatorFeedback.setIntegratorRange(-1, 1);
        leftElevatorFeedback.setIntegratorRange(-1, 1);

        rightElevatorFeedback.setTolerance(ElevatorConstants.kElevatorFeedBackPositionTolerance,
                ElevatorConstants.kElevatorFeedBackVelocityTolerance);
        leftElevatorFeedback.setTolerance(ElevatorConstants.kElevatorFeedBackPositionTolerance,
                ElevatorConstants.kElevatorFeedBackVelocityTolerance);
    }

    @Override
    public void periodic() {
        refreshControlLoop();
        SmartDashboard.putNumber("elvatorMotorLeft", leftEncoder.getPosition());
        SmartDashboard.putNumber("elvatorMotorRight", rightEncoder.getPosition());
        SmartDashboard.putNumber("absolute elevator", absoluteEncoder.getPosition());
    } 

    public double getPosition() {
        return absoluteEncoder.getPosition();
    }

    public double getVelocity() {
        return absoluteEncoder.getVelocity();
    }

    public void refreshControlLoop() {
        State motState = motionProfile.calculate(Timer.getFPGATimestamp() - motionStartTime);
        
        leftMotor.setVoltage(
            elevatorFeedforward.calculate(motState.velocity, ElevatorConstants.kElevatorMaxAccel)
            + leftElevatorFeedback.calculate(getPosition(), setPoint)
        );

        rightMotor.setVoltage(
            elevatorFeedforward.calculate(motState.velocity, ElevatorConstants.kElevatorMaxAccel)
            + rightElevatorFeedback.calculate(getPosition(), setPoint)
        );
    }

    public double getSetPoint() {
        return setPoint;
    }

    public void setSetpoint(double elevatorPosM) {
        setPoint = elevatorPosM;
        motionStartTime = Timer.getFPGATimestamp();

        motionProfile = getTrapezoidProfile(elevatorPosM);
    }

    public TrapezoidProfile getTrapezoidProfile(double position) {
        Constraints motionConstraints = new Constraints(ElevatorConstants.kElevatorMaxVel, ElevatorConstants.kElevatorMaxAccel);

        return new TrapezoidProfile(motionConstraints, new State(position, 0), new State(position, getVelocity()));
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

    public IntakeGameElement getDesiredIntakeGameElement(){
        return desiredIntakeGameElement;
    }

    public void setDesiredIntakeGameElement(IntakeGameElement intakeGameElement) {
        desiredIntakeGameElement = intakeGameElement;
    }

    public static enum ElevatorPosition {
        HOME,
        INTAKE,
        LOW,
        MID,
        HIGH;
    }

    public static enum IntakeGameElement {
        GROUND_CONE_UP,
        GROUND_CONE_TIPPED,
        CONE_SUBSTATION,
        GROUND_CUBE;
    }
    
}
