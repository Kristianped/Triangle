package triangle.tools;

import triangle.Mesh;

public class Statistic {

    /// Number of incircle tests performed.
    public static long InCircleCount = 0;
    public static long InCircleAdaptCount = 0;

    /// Number of counterclockwise tests performed.
    public static long CounterClockwiseCount = 0;
    public static long CounterClockwiseAdaptCount = 0;

    /// Number of 3D orientation tests performed.
    public static long Orient3dCount = 0;

    /// Number of right-of-hyperbola tests performed.
    public static long HyperbolaCount = 0;

    /// // Number of circumcenter calculations performed.
    public static long CircumcenterCount = 0;

    /// Number of circle top calculations performed.
    public static long CircleTopCount = 0;

    /// Number of vertex relocations.
    public static long RelocationCount = 0;

    double minEdge = 0;
    double maxEdge = 0;
    double minAspect = 0;
    double maxAspect = 0;
    double minArea = 0;
    double maxArea = 0;
    double minAngle = 0;
    double maxAngle = 0;
    int[] angleTable;
    int[] minAngles;
    int[] maxAngles;

    public double ShortestEdge() { return minEdge; }
    public double LongestEdge() { return maxEdge; }
    public double ShortestAltitude() { return minAspect; }
    public double LargestAspectRatio() { return maxAspect; }
    public double SmallestArea() { return minArea; }
    public double LargestArea() { return maxArea; }
    public double SmallestAngle() { return minAngle; }
    public double LargestAngle() { return maxAngle; }
    public int[] AngleHistogram() { return angleTable; }
    public int[] MinAngleHistogram() { return minAngles; }
    public int[] MaxAngleHistogram() {return maxAngles; }

    private void getAspectHistogram(Mesh mesh) {

    }

}
