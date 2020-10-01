package triangle;

import triangle.tools.Tuple;

import java.util.function.Function;

/**
 * Controls the behavior of the meshing software
 */
public class Behavior {

    public static boolean NoExact;

    boolean poly = false;
    boolean quality = false;
    boolean varArea = false;
    boolean convex = false;
    boolean jettison = false;
    boolean boundaryMarkers = true;
    boolean noHoles = false;
    boolean conformDel = false;

    Function<Tuple<ITriangle, Double>, Boolean> usertest;

    int noBisect = 0;

    double minAngle = 0.0;
    double maxAngle = 0.0;
    double maxArea = -1.0;

    boolean fixedArea = false;
    boolean useSegments = true;
    boolean useRegions = false;
    double goodAngle = 0.0;
    double maxGoodAngle = 0.0;
    double offconstant = 0.0;

    public Behavior() {
        this(false, 20);
    }

    /**
     * Creates an instance of the Behavior class
     */
    public Behavior(boolean quality, double minAngle) {
        if (quality) {
            this.quality = true;
            this.minAngle = minAngle;
            update();
        }
    }

    private void update() {
        quality = true;

        if (this.minAngle < 0 || this.minAngle > 60) {
            minAngle = 0;
            quality = false;
            System.out.println("Invalid quality options (minimum angle)");
        }

        if ((this.maxAngle != 0.0) && (this.maxAngle < 60 || this.maxAngle > 180)) {
            maxAngle = 0;
            quality = false;
            System.out.println("Invalid quality options (maximum angle)");
        }

        this.useSegments = this.poly || this.quality || this.convex;
        this.goodAngle = Math.cos(this.minAngle * Math.PI / 180.0);
        this.maxGoodAngle = Math.cos(this.maxAngle * Math.PI / 180.0);

        if (this.goodAngle == 1.0)
        {
            this.offconstant = 0.0;
        }
        else
        {
            this.offconstant = 0.475 * Math.sqrt((1.0 + this.goodAngle) / (1.0 - this.goodAngle));
        }

        this.goodAngle *= this.goodAngle;
    }

    public boolean isQuality() {
        return quality;
    }

    public void setQuality(boolean value) {
        quality = value;

        if (quality)
            update();
    }

    public double getMinAngle() {
        return minAngle;
    }

    public void setMinAngle(double angle) {
        minAngle = angle;
        update();
    }

    public double getMaxAngle() {
        return maxAngle;
    }

    public void setMaxAngle(double angle) {
        maxAngle = angle;
        update();
    }

    public double getMaxArea() {
        return maxArea;
    }

    public void setMaxArea(double area) {
        maxArea = area;
        fixedArea = area > 0;
    }

    public boolean isVarArea() {
        return varArea;
    }

    public void setVarArea(boolean value) {
        varArea = value;
    }

    public boolean isPoly() {
        return poly;
    }

    public void setPoly(boolean value) {
        poly = value;
    }

    public Function<Tuple<ITriangle, Double>, Boolean> getUsertest() {
        return usertest;
    }

    public void setUsertest(Function<Tuple<ITriangle, Double>, Boolean> test) {
        usertest = test;
    }

    public boolean isConvex() {
        return convex;
    }

    public void setConvex(boolean value) {
        convex = value;
    }

    public boolean isConformingDelaunay() {
        return conformDel;
    }

    public void setConforminDelaunay(boolean value) {
        conformDel = value;
    }

    public int getNoBisect() {
        return noBisect;
    }

    public void setNoBisect(int value) {
        noBisect = value;

        if (noBisect < 0 || noBisect > 2)
            noBisect = 0;
    }

    public boolean useBoundaryMarkers() {
        return boundaryMarkers;
    }

    public void setBoundaryMarkers(boolean value) {
        boundaryMarkers = value;
    }

    public boolean isNoHoles() {
        return noHoles;
    }

    public void setNoHoles(boolean value) {
        noHoles = value;
    }

    public boolean isJettison() {
        return jettison;
    }

    public void setJettison(boolean value) {
        jettison = value;
    }

    public boolean useRegions() {
        return useRegions;
    }

    public void setUseRegions(boolean useRegions) {
        this.useRegions = useRegions;
    }

    public boolean useSegments() {
        return useSegments;
    }

    public void setUseSegments(boolean useSegments) {
        this.useSegments = useSegments;
    }

    public double getGoodAngle() {
        return goodAngle;
    }

    public double getMaxGoodAngle() {
        return maxGoodAngle;
    }

    public boolean isFixedArea() {
        return fixedArea;
    }

    public double getOffconstant() {
        return offconstant;
    }
}
