package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.CANConstants;
import frc.robot.Constants.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
    private CANSparkMax pivotMotor = new CANSparkMax(CANConstants.kClawPivotMotor, MotorType.kBrushless);
    private CANSparkMax wheelMotor = new CANSparkMax(CANConstants.kClawWheelMotor, MotorType.kBrushless);
    private RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
    private PIDController pivotFeedback = new PIDController(ClawConstants.kP, ClawConstants.kI, ClawConstants.kD);
    private ArmFeedforward pivotFeedforward = new ArmFeedforward(ClawConstants.kS, ClawConstants.kCos, ClawConstants.kV, ClawConstants.kA);
    private ColorSensorV3 colorSensorV3 = new ColorSensorV3(I2C.Port.kMXP);
    private ColorMatch colorMatcher = new ColorMatch();
    private BooleanSupplier isInterrupted;
    private GameElement currGameElement = GameElement.CONE;
    public WheelDirection currWheelDirection = WheelDirection.OFF;

    public ClawSubsystem() {
        pivotEncoder.setPositionConversionFactor(ClawConstants.kPivotMotorTicksToRadians);        
        pivotEncoder.setVelocityConversionFactor(ClawConstants.kPivotMotorTicksToRadians);

        pivotEncoder.setPosition(ClawConstants.kPivotEncoderOffset);

        pivotFeedback.setIntegratorRange(-1, 1);
        pivotFeedback.setTolerance(ClawConstants.kPivotFeedbackPositionTolerance, ClawConstants.kPivotFeedbackVelocityTolerance);
        
        colorSensorV3.configureProximitySensor(ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate6ms);
    }

    public void setInterruptedCondition(BooleanSupplier isInterrupted) {
        this.isInterrupted = isInterrupted;
    }

    @Override
    public void periodic() {
        Color sensorColor = colorSensorV3.getColor();
        ColorMatchResult mColorMatchResult = colorMatcher.matchClosestColor(sensorColor);

        SmartDashboard.putNumber("Claw Pivot Motor Position", pivotEncoder.getPosition());
        SmartDashboard.putNumber("Claw Pivot Target Position", pivotEncoder.getPosition());
        SmartDashboard.putNumber("Color Sensor Blue", sensorColor.blue);
        SmartDashboard.putNumber("Color Sensor Red", sensorColor.red);
        SmartDashboard.putNumber("Color Sensor Green", sensorColor.green);
        SmartDashboard.putString("Color Sensor Likely Color", mColorMatchResult.color.toString());
        SmartDashboard.putNumber("Color Sensor Confidence", mColorMatchResult.confidence);

        if(colorSensorV3.getProximity() <= 100) {
            if(mColorMatchResult.color == Color.kYellow) {
                this.currGameElement = GameElement.CONE;
            }
            else if(mColorMatchResult.color == Color.kPurple) {
                this.currGameElement = GameElement.CUBE;
            }
        }

        updateWheelPower();
    }

    public void setPosition(double clawPosition) {
        pivotFeedback.setSetpoint(clawPosition);
    }

    public void setMotorVoltageByPosition(double pivotAngleRad) {
        pivotMotor.setVoltage(pivotFeedforward.calculate(pivotAngleRad, 0) + pivotFeedback.calculate(pivotEncoder.getPosition()));
    }

    public void setPivotPower(double pivotPower) {
        pivotMotor.set(pivotPower);
    }

    public void updateWheelPower() {
        double wheelPower = 0;

        if(currWheelDirection == WheelDirection.INTAKE) {
            wheelPower = colorSensorV3.getProximity() / 2047;
        }
        else if(currWheelDirection == WheelDirection.OUTTAKE) {
            wheelPower = -1;
        }

        wheelMotor.set(wheelPower);
    }

    public void setWheelDirection(WheelDirection wDirection) {
        currWheelDirection = wDirection;
    }

    public WheelDirection getWheelDirection() {
        return currWheelDirection;
    }

    public void goToPosition(double pivotAngleRad) {
        setPosition(pivotAngleRad);

        while(!pivotFeedback.atSetpoint()) {
            setMotorVoltageByPosition(pivotAngleRad);

            //to not hog CPU cycles
            try {
                Thread.sleep(15);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        pivotMotor.setVoltage(pivotFeedforward.calculate(pivotAngleRad, 0));
    }

    public void setCurrGameElement(GameElement currGameElement) {
        this.currGameElement = currGameElement;
    }

    public GameElement getCurrGameElement() {
        return currGameElement;
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
