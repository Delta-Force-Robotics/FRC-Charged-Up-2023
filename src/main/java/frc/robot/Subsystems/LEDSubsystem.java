package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED LEDStrip;
    private AddressableLEDBuffer LEDBuffer;
    private double startTime = 0;
    private boolean greenLEDs = false;
    private Colors LEDColour = Colors.ORANGE;
    private boolean isElementInside = false;

    public LEDSubsystem() {
        LEDStrip = new AddressableLED(0);
    
        LEDBuffer = new AddressableLEDBuffer(74);

        LEDStrip.setLength(LEDBuffer.getLength());

        LEDStrip.start();
    }

    @Override
    public void periodic() {
        checkLEDs();
    }

    public boolean getIsElementInside() {
        return isElementInside;
    }

    public void setIsElementInside(boolean isElemInside) {
        isElementInside = isElemInside;
        startTime = Timer.getFPGATimestamp();
    }

    public void checkLEDs() {
        if(Timer.getFPGATimestamp() - startTime < 1) {
            setColor(0, 255, 0);
        }
        else {
            isElementInside = false;
            setColor(LEDColour.r, LEDColour.g, LEDColour.b);
        }
    }

    public void setLEDColour(Colors colour) {
        LEDColour = colour;
    }

    public Colors getLEDColour() {
        return LEDColour;
    }
    
    public void setColor(int r, int g, int b) {
        for(int i = 0; i<LEDBuffer.getLength(); i++) {
            LEDBuffer.setRGB(i, r, g, b);
        }

        LEDStrip.setData(LEDBuffer);
    }

    public static enum Colors {
        ORANGE(255,102,0),
        PURPLE(160, 32, 240),
        GREEN(0,255,0);

        public int r,g,b;

        private Colors(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }

    }
}
