package triangle;

import java.util.function.Function;

public class QualityOptions {

    private double maxAngle;
    private double minAngle;
    private double maxArea;
    private boolean variableArea;
    private int steinerPoints;
    private Function<Tuple<ITriangle, Double>, Boolean> usertest;

    public double getMaxAngle() {
        return maxAngle;
    }

    public void setMaxAngle(double angle) {
        maxAngle = angle;
    }

    public double getMinAngle() {
        return minAngle;
    }

    public void setMinAngle(double angle) {
        minAngle = angle;
    }

    public double getMaxArea() {
        return maxArea;
    }

    public void setMaxArea(double area) {
        maxArea = area;
    }

    public boolean isVariableArea() {
        return variableArea;
    }

    public void setVariableArea(boolean bool) {
        variableArea = bool;
    }

    public int getSteinerPoints() {
        return steinerPoints;
    }

    public void setSteinerPoints(int steinerPoints) {
        this.steinerPoints = steinerPoints;
    }

    public Function<Tuple<ITriangle, Double>, Boolean> getUsertest() {
        return usertest;
    }

    public void setUsertest(Function<Tuple<ITriangle, Double>, Boolean> usertest) {
        this.usertest = usertest;
    }
}
