package triangle.tools;

import triangle.*;

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

    final static int[] plus1Mod3 = { 1, 2, 0 };
    final static int[] minus1Mod3 = { 2, 0, 1 };

    private void getAspectHistogram(Mesh mesh) {
        int[] aspecttable;
        double[] ratiotable;

        aspecttable = new int[16];
        ratiotable = new double[] {
                1.5, 2.0, 2.5, 3.0, 4.0, 6.0, 10.0, 15.0, 25.0, 50.0,
                100.0, 300.0, 1000.0, 10000.0, 100000.0, 0.0 };


        Otri tri = new Otri();
        Vertex[] p = new Vertex[3];
        double[] dx = new double[3], dy = new double[3];
        double[] edgelength = new double[3];
        double triarea;
        double trilongest2;
        double triminaltitude2;
        double triaspect2;

        int aspectindex;
        int i, j, k;

        tri.orient = 0;
        for (var t : mesh.getTriangles()) {
            tri.tri = t;
            p[0] = tri.org();
            p[1] = tri.dest();
            p[2] = tri.apex();
            trilongest2 = 0.0;

            for (i = 0; i < 3; i++) {
                j = plus1Mod3[i];
                k = minus1Mod3[i];
                dx[i] = p[j].getX() - p[k].getX();
                dy[i] = p[j].getY() - p[k].getY();
                edgelength[i] = dx[i] * dx[i] + dy[i] * dy[i];

                if (edgelength[i] > trilongest2)
                    trilongest2 = edgelength[i];
            }

            //triarea = Primitives.CounterClockwise(p[0], p[1], p[2]);
            triarea = Math.abs((p[2].getX() - p[0].getX()) * (p[1].getY() - p[0].getY()) -
                    (p[1].getX() - p[0].getX()) * (p[2].getY() - p[0].getY())) / 2.0;

            triminaltitude2 = triarea * triarea / trilongest2;

            triaspect2 = trilongest2 / triminaltitude2;

            aspectindex = 0;

            while ((triaspect2 > ratiotable[aspectindex] * ratiotable[aspectindex]) && (aspectindex < 15))
                aspectindex++;

            aspecttable[aspectindex]++;
        }
    }

    /**
     * Update statistics about the quality of the mesh.
     * @param mesh The mesh to update the statistics on
     */
    public void update(Mesh mesh) {
        Point[] p = new Point[3];

        int k1, k2;
        int degreeStep;

        //sampleDegrees = 36; // sample every 5 degrees
        //sampleDegrees = 45; // sample every 4 degrees
        int sampleDegrees = 60; // sample every 3 degrees

        double[] cosSquareTable = new double[sampleDegrees / 2 - 1];
        double[] dx = new double[3];
        double[] dy = new double[3];
        double[] edgeLength = new double[3];
        double dotProduct;
        double cosSquare;
        double triArea;
        double triLongest2;
        double triMinAltitude2;
        double triAspect2;

        double radconst = Math.PI / sampleDegrees;
        double degconst = 180.0 / Math.PI;

        // New angle table
        angleTable = new int[sampleDegrees];
        minAngles = new int[sampleDegrees];
        maxAngles = new int[sampleDegrees];

        for (int i = 0; i < sampleDegrees / 2 - 1; i++) {
            cosSquareTable[i] = Math.cos(radconst * (i + 1));
            cosSquareTable[i] = cosSquareTable[i] * cosSquareTable[i];
        }

        for (int i = 0; i < sampleDegrees; i++)
            angleTable[i] = 0;


        minAspect = mesh.getBounds().width() + mesh.getBounds().height();
        minAspect = minAspect * minAspect;
        maxAspect = 0.0;
        minEdge = minAspect;
        maxEdge = 0.0;
        minArea = minAspect;
        maxArea = 0.0;
        minAngle = 0.0;
        maxAngle = 2.0;

        boolean acuteBiggest = true;
        boolean acuteBiggestTri = true;

        double triMinAngle, triMaxAngle = 1;

        for (var tri : mesh.getTriangles()) {
            triMinAngle = 0; // Min angle:  0 < a <  60 degress
            triMaxAngle = 1; // Max angle: 60 < a < 180 degress

            p[0] = tri.getVertex(0);
            p[1] = tri.getVertex(1);
            p[2] = tri.getVertex(2);

            triLongest2 = 0.0;

            for (int i = 0; i < 3; i++) {
                k1 = plus1Mod3[i];
                k2 = minus1Mod3[i];

                dx[i] = p[k1].getX() - p[k2].getX();
                dy[i] = p[k1].getY() - p[k2].getY();

                edgeLength[i] = dx[i] * dx[i] + dy[i] * dy[i];

                if (edgeLength[i] > triLongest2)
                    triLongest2 = edgeLength[i];

                if (edgeLength[i] > maxEdge)
                    maxEdge = edgeLength[i];

                if (edgeLength[i] < minEdge)
                    minEdge = edgeLength[i];
            }

            //triarea = Primitives.CounterClockwise(p[0], p[1], p[2]);
            triArea = Math.abs((p[2].getX() - p[0].getX()) * (p[1].getY() - p[0].getY()) -
                    (p[1].getX() - p[0].getX()) * (p[2].getY() - p[0].getY()));

            if (triArea < minArea)
                minArea = triArea;

            if (triArea > maxArea)
                maxArea = triArea;

            triMinAltitude2 = triArea * triArea / triLongest2;

            if (triMinAltitude2 < minAspect)
                minAspect = triMinAltitude2;

            triAspect2 = triLongest2 / triMinAltitude2;

            if (triAspect2 > maxAspect)
                maxAspect = triAspect2;

            for (int i = 0; i < 3; i++) {
                k1 = plus1Mod3[i];
                k2 = minus1Mod3[i];

                dotProduct = dx[k1] * dx[k2] + dy[k1] * dy[k2];
                cosSquare = dotProduct * dotProduct / (edgeLength[k1] * edgeLength[k2]);
                degreeStep = sampleDegrees / 2 - 1;

                for (int j = degreeStep - 1; j >= 0; j--)
                    if (cosSquare > cosSquareTable[j])
                        degreeStep = j;

                if (dotProduct <= 0.0) {
                    angleTable[degreeStep]++;

                    if (cosSquare > minAngle)
                        minAngle = cosSquare;

                    if (acuteBiggest && (cosSquare < maxAngle))
                        maxAngle = cosSquare;

                    // Update min/max angle per triangle
                    if (cosSquare > triMinAngle)
                        triMinAngle = cosSquare;

                    if (acuteBiggestTri && (cosSquare < triMaxAngle))
                        triMaxAngle = cosSquare;
                } else {
                    angleTable[sampleDegrees - degreeStep - 1]++;

                    if (acuteBiggest || (cosSquare > maxAngle)) {
                        maxAngle = cosSquare;
                        acuteBiggest = false;
                    }

                    // Update max angle for (possibly non-acute) triangle
                    if (acuteBiggestTri || (cosSquare > triMaxAngle)) {
                        triMaxAngle = cosSquare;
                        acuteBiggestTri = false;
                    }
                }
            }

            // Update min angle histogram
            degreeStep = sampleDegrees / 2 - 1;

            for (int j = degreeStep - 1; j >= 0; j--)
                if (triMinAngle > cosSquareTable[j])
                    degreeStep = j;

            minAngles[degreeStep]++;

            // Update max angle histogram
            degreeStep = sampleDegrees / 2 - 1;

            for (int j = degreeStep - 1; j >= 0; j--)
                if (triMaxAngle > cosSquareTable[j])
                    degreeStep = j;

            if (acuteBiggestTri)
                maxAngles[degreeStep]++;
            else
                maxAngles[sampleDegrees - degreeStep - 1]++;

            acuteBiggestTri = true;
        }

        minEdge = Math.sqrt(minEdge);
        maxEdge = Math.sqrt(maxEdge);
        minAspect = Math.sqrt(minAspect);
        maxAspect = Math.sqrt(maxAspect);
        minArea *= 0.5;
        maxArea *= 0.5;

        if (minAngle >= 1.0)
            minAngle = 0.0;
        else
            minAngle = degconst * Math.acos(Math.sqrt(minAngle));

        if (maxAngle >= 1.0) {
            maxAngle = 180.0;
        } else {
            if (acuteBiggest)
                maxAngle = degconst * Math.acos(Math.sqrt(maxAngle));
            else
                maxAngle = 180.0 - degconst * Math.acos(Math.sqrt(maxAngle));
        }
    }

    /**
     * Compute angle information for given triangle.
     * <br><br>
     * On return, the squared cosines of the minimum and maximum angle will
     * be stored at position data[0] and data[1] respectively.
     * If the triangle was obtuse, data[2] will be set to -1 and maximum angle
     * is computed as (pi - acos(sqrt(data[1]))).
     * @param triangle The triangle to check
     * @param data Array of doubles (length 6)
     */
    public static void ComputeAngles(ITriangle triangle, double[] data) {
        double min = 0.0;
        double max = 1.0;

        var va = triangle.getVertex(0);
        var vb = triangle.getVertex(1);
        var vc = triangle.getVertex(2);

        double dxa = vb.getX() - vc.getX();
        double dya = vb.getY() - vc.getY();
        double lena = dxa * dxa + dya * dya;

        double dxb = vc.getX() - va.getX();
        double dyb = vc.getY() - va.getY();
        double lenb = dxb * dxb + dyb * dyb;

        double dxc = va.getX() - vb.getX();
        double dyc = va.getY() - vb.getY();
        double lenc = dxc * dxc + dyc * dyc;

        // Dot products.
        double dota = data[0] = dxb * dxc + dyb * dyc;
        double dotb = data[1] = dxc * dxa + dyc * dya;
        double dotc = data[2] = dxa * dxb + dya * dyb;

        // Squared cosines.
        data[3] = (dota * dota) / (lenb * lenc);
        data[4] = (dotb * dotb) / (lenc * lena);
        data[5] = (dotc * dotc) / (lena * lenb);

        // The sign of the dot product will tell us, if the angle is
        // acute (value < 0) or obtuse (value > 0).

        boolean acute = true;

        double cos, dot;

        for (int i = 0; i < 3; i++) {
            dot = data[i];
            cos = data[3 + i];

            if (dot <= 0.0) {
                if (cos > min)
                    min = cos;

                if (acute && (cos < max))
                    max = cos;
            } else {
                // Update max angle for (possibly non-acute) triangle
                if (acute || (cos > max)) {
                    max = cos;
                    acute = false;
                }
            }
        }

        data[0] = min;
        data[1] = max;
        data[2] = acute ? 1.0 : -1.0;
    }
}
