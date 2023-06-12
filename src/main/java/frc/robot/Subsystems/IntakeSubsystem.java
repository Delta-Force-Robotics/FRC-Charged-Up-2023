package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.CANConstants;
import frc.robot.Constants.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax pivotMotor = new CANSparkMax(CANConstants.kIntakePivotMotor, MotorType.kBrushless);
    private CANSparkMax wheelMotor = new CANSparkMax(CANConstants.kIntakeWheelMotor, MotorType.kBrushless);
    private RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
    private PIDController pivotFeedBack = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
    private ArmFeedforward pivotFeedForward = new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kCos, IntakeConstants.kV, IntakeConstants.kA);
    private AbsoluteEncoder absoluteEncoderPivot = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle); 

    private BooleanSupplier isInterrupted;

    private double setPoint = IntakeConstants.kPivotAngleRadHome; 
    private TrapezoidProfile motionProfile = getTrapezoidProfile(IntakeConstants.kPivotAngleRadHome);
    private double motionStartTime = Timer.getFPGATimestamp();
    private double lastVelTime = 0;
    private double lastVel = 0;

    public WheelDirection currWheelDirection = WheelDirection.OFF;
    private GameElement currGameElement = GameElement.CONE;

    public IntakeSubsystem() {
        pivotMotor.setInverted(true); 
        wheelMotor.setOpenLoopRampRate(0.5);   

        pivotEncoder.setPositionConversionFactor(IntakeConstants.kPivotMotorTicksToRadians);        
        pivotEncoder.setVelocityConversionFactor(IntakeConstants.kPivotMotorRPMToRadiansPerSecond);

        absoluteEncoderPivot.setInverted(false);

        pivotEncoder.setPosition(wrapAroundEncoder(absoluteEncoderPivot.getPosition() / 0.28 * 2 * Math.PI));

        pivotFeedBack.setIntegratorRange(-1, 1);
    }

    public void setInterruptedCondition(BooleanSupplier isInterrupted) {
        this.isInterrupted = isInterrupted;
    }

    public double wrapAroundEncoder(double encoderPosition) {
        if (encoderPosition >= Math.PI) {
            encoderPosition -= 2 * Math.PI;
        }

        return encoderPosition; 
    }

    @Override
    public void periodic() {
        /*SmartDashboard.putNumber("Pivot", pivotEncoder.getPosition());        //SmartDashboard.putNumber("pivot motor", pivotEncoder.getPosition());
        SmartDashboard.putNumber("Absolut", absoluteEncoderPivot.getPosition());
        SmartDashboard.putNumber("feedforward", pivotFeedForward.calculate(IntakeConstants.kPivotAngleRadIntakeConeSubstation, 0));
        SmartDashboard.putNumber("feedback", pivotFeedBack.calculate(pivotEncoder.getPosition(), IntakeConstants.kPivotAngleRadIntakeConeSubstation));
        SmartDashboard.putNumber("feedforwardback", pivotFeedForward.calculate(IntakeConstants.kPivotAngleRadIntakeConeSubstation, 0) + pivotFeedBack.calculate(pivotEncoder.getPosition(), IntakeConstants.kPivotAngleRadIntakeConeSubstation));
        SmartDashboard.putNumber("offset", absoluteEncoderPivot.getZeroOffset());
        SmartDashboard.putNumber("output current", wheelMotor.getOutputCurrent());
        SmartDashboard.putNumber("applied output", wheelMotor.getAppliedOutput());
        SmartDashboard.putNumber("setpoint", setPoint);
        SmartDashboard.putNumber("veloc?", pivotEncoder.getVelocity());
        SmartDashboard.putNumber("pos?", pivotEncoder.getPosition());*/

        updateWheelPower();
        refreshControlLoop();

    }

    public void refreshControlLoop() {
        State motState = motionProfile.calculate(Timer.getFPGATimestamp() - motionStartTime);

        double velocity = motState.velocity;
        double accel = (velocity - lastVel) / (Timer.getFPGATimestamp() - lastVelTime);

        SmartDashboard.putNumber("pos", motState.position);
        SmartDashboard.putNumber("velocity", velocity);
        SmartDashboard.putNumber("1", motionProfile.totalTime());

        pivotMotor.setVoltage(pivotFeedBack.calculate(pivotEncoder.getPosition(), motState.position) + pivotFeedForward.calculate(pivotEncoder.getPosition(), velocity, accel));

        lastVel = velocity;
        lastVelTime = Timer.getFPGATimestamp();
    }
    
    public void setPivotPower(double pivotPower) {
        pivotMotor.set(pivotPower);
    }

    public double getOutputCurrent() {
        return wheelMotor.getOutputCurrent();
    }

    public void updateWheelPower() {
        double wheelPower = 0;

        if(currWheelDirection == WheelDirection.INTAKE) {
            wheelPower = 0.5;
        }
        else if(currWheelDirection == WheelDirection.OUTTAKE) {
            wheelPower = -0.5;
        }

        wheelMotor.set(wheelPower);
    }

    public TrapezoidProfile getTrapezoidProfile(double position) {
        Constraints motionConstraints = new Constraints(IntakeConstants.kIntakeMaxVel, IntakeConstants.kIntakeMaxAccel);

        return new TrapezoidProfile(motionConstraints, new State(position, 0), new State(pivotEncoder.getPosition(), 0));
    }

    public void setWheelDirection(WheelDirection wDirection) {
        currWheelDirection = wDirection;
    }

    public WheelDirection getWheelDirection() {
        return currWheelDirection;
    }

    public void setCurrGameElement(GameElement currGameElement) {
        this.currGameElement = currGameElement;
    }

    public GameElement getCurrGameElement() {
        return currGameElement;
    }

    public void setSetpoint(double intakeAngleRad) {
        motionStartTime = Timer.getFPGATimestamp();
        setPoint = intakeAngleRad;

        motionProfile = getTrapezoidProfile(intakeAngleRad);
    }

    public void setRampRate(double rampRate) {
        wheelMotor.setOpenLoopRampRate(rampRate);
    }

    public static enum GameElement {
        CUBE,
        CONE
    };

    public static enum WheelDirection {
        INTAKE,
        OUTTAKE,
        OFF
    }
}