package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.ArmConstants;
import frc.robot.Constants.Constants.CANConstants;
import frc.robot.Constants.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
    private CANSparkMax pivotMotor = new CANSparkMax(CANConstants.kClawPivotMotor, MotorType.kBrushless);
    private CANSparkMax wheelMotor = new CANSparkMax(CANConstants.kClawWheelMotor, MotorType.kBrushless);
    private RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
    private PIDController pivotFeedback = new PIDController(ClawConstants.maxkP, ClawConstants.kI, ClawConstants.kD);
    private ArmFeedforward pivotFeedforward = new ArmFeedforward(ClawConstants.kS, ClawConstants.kCos, ClawConstants.kV, ClawConstants.kA);
    private DigitalInput intakeLimitSwitch = new DigitalInput(1);
    private DutyCycleEncoder throughBoreEncoder = new DutyCycleEncoder(0);
    private BooleanSupplier isInterrupted;
    private GameElement currGameElement = GameElement.CONE;
    private double setPoint = ClawConstants.kPivotEncoderOffset; 
    public WheelDirection currWheelDirection = WheelDirection.OFF;

    public ClawSubsystem() {
        pivotMotor.setInverted(false);
        pivotEncoder.setPositionConversionFactor(ClawConstants.kPivotMotorTicksToRadians);        
        pivotEncoder.setVelocityConversionFactor(ClawConstants.kPivotMotorRPMToRadiansPerSecond);

        pivotEncoder.setPosition(ClawConstants.kPivotEncoderOffset);

        pivotFeedback.setIntegratorRange(-1, 1);

        throughBoreEncoder.setPositionOffset(0.32);        
        throughBoreEncoder.setDistancePerRotation(2 * Math.PI);
    }

    public void setInterruptedCondition(BooleanSupplier isInterrupted) {
        this.isInterrupted = isInterrupted;
    }

    public double getPosition() {
        return wrapAroundEncoder((- throughBoreEncoder.getAbsolutePosition() + throughBoreEncoder.getPositionOffset()) * throughBoreEncoder.getDistancePerRotation());
    }

    public double wrapAroundEncoder(double encoderPosition) {
        if(encoderPosition > 0) {
            return encoderPosition;
        }

        return throughBoreEncoder.getDistancePerRotation() + encoderPosition;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Claw Pivot Motor Position", Math.toDegrees(getPosition()));
        SmartDashboard.putNumber("Claw Pivot Target Position", Math.toDegrees(setPoint));
        SmartDashboard.putNumber("Feedforward Voltage", pivotFeedforward.calculate(setPoint, 0));
        SmartDashboard.putNumber("PID Angle", Math.toDegrees(Math.PI - setPoint + ArmConstants.armAngle));
        SmartDashboard.putBoolean("Limit Switch", intakeLimitSwitch.get());

        updateWheelPower();

        scheduleGains(setPoint);
        setMotorVoltageByPosition(setPoint); //keep PIDF running forever
    }

    public void scheduleGains(double clawPosition) {
        pivotFeedback.setP(Math.abs(Math.cos(clawPosition)) * ClawConstants.maxkP);
    }

    public void setMotorVoltageByPosition(double pivotAngleRad) {
        pivotMotor.setVoltage(pivotFeedforward.calculate(pivotAngleRad, 0) + pivotFeedback.calculate(getPosition(), Math.PI - pivotAngleRad + ArmConstants.armAngle));
    }

    public void setPivotPower(double pivotPower) {
        pivotMotor.set(pivotPower);
    }

    public void updateWheelPower() {
        double wheelPower = 0;

        if(currWheelDirection == WheelDirection.INTAKE) {
            wheelPower = (intakeLimitSwitch.get() == false) ? 1 : 0.01;
        }
        else if(currWheelDirection == WheelDirection.OUTTAKE) {
            wheelPower = -0.2;
        }
        else {
            wheelPower = (intakeLimitSwitch.get() == false) ? 0 : 0.01;
        }

        wheelMotor.set(wheelPower);
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

    public double getSetPoint() {
        return setPoint;
    }

    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
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
