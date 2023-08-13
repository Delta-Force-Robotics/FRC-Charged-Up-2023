package frc.robot.Util;

public class VisionTarget {
    private double width, height; //meters
    private double area; //meters_sq

    /**
     * <p>Creates a vision target with width and height in meters for use with Limelight.</p>
     * @param width width in meters
     * @param height height in meters
     */
    public VisionTarget(double width, double height) {
        this.width = width;
        this.height = height;
        this.area = width * height;
    }

    public double getWidth() {
        return width;
    }

    public double getHeight() {
        return height;
    }

    public double getArea() {
        return area;
    }
}
