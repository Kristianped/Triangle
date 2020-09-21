package triangle;

import java.util.function.Function;

public class QualityOptions {

    private int maxAngle;
    private int minAngle;
    private int maxArea;
    private boolean variableArea;
    private int steinerPoints;
    private Function<Tuple<ITriangle, Double>, Boolean> usertest;

    public int getMaxAngle() {
        return maxAngle;
    }

    public void setMaxAngle(int angle) {
        maxAngle = angle;
    }

    public int getMinAngle() {
        return minAngle;
    }

    public void setMinAngle(int angle) {
        minAngle = angle;
    }

    public int getMaxArea() {
        return maxArea;
    }

    public void setMaxArea(int area) {
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
