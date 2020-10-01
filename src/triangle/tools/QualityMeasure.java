package triangle.tools;

import triangle.Mesh;
import triangle.geometry.Point;

/**
 * Provides mesh quality information.
 * <br><br>
 * Given a triangle abc with points A (ax, ay), B (bx, by), C (cx, cy).
 * <br><br>
 * The side lengths are given as<br>
 *   a = sqrt((cx - bx)^2 + (cy - by)^2) -- side BC opposite of A<br>
 *   b = sqrt((cx - ax)^2 + (cy - ay)^2) -- side CA opposite of B<br>
 *   c = sqrt((ax - bx)^2 + (ay - by)^2) -- side AB opposite of C
 *   <br><br>
 * The angles are given as<br>
 *   ang_a = acos((b^2 + c^2 - a^2)  / (2 * b * c)) -- angle at A<br>
 *   ang_b = acos((c^2 + a^2 - b^2)  / (2 * c * a)) -- angle at B<br>
 *   ang_c = acos((a^2 + b^2 - c^2)  / (2 * a * b)) -- angle at C
 *   <br><br>
 * The semiperimeter is given as<br>
 *   s = (a + b + c) / 2
 *   <br><br>
 * The area is given as<br>
 *   D = abs(ax * (by - cy) + bx * (cy - ay) + cx * (ay - by)) / 2<br>
 *     = sqrt(s * (s - a) * (s - b) * (s - c))
 *      <br><br>
 * The inradius is given as<br>
 *   r = D / s
 *   <br><br>
 * The circumradius is given as<br>
 *   R = a * b * c / (4 * D)
 * <br><br>
 * The altitudes are given as<br>
 *   alt_a = 2 * D / a -- altitude above side a<br>
 *   alt_b = 2 * D / b -- altitude above side b<br>
 *   alt_c = 2 * D / c -- altitude above side c
 * <br><br>
 * The aspect ratio may be given as the ratio of the longest to the
 * shortest edge or, more commonly as the ratio of the circumradius
 * to twice the inradius<br>
 *   ar = R / (2 * r)<br>
 *      = a * b * c / (8 * (s - a) * (s - b) * (s - c))<br>
 *      = a * b * c / ((b + c - a) * (c + a - b) * (a + b - c))
 */
public class QualityMeasure {

    AreaMeasure areaMeasure;
    AlphaMeasure alphaMeasure;
    Q_Measure qMeasure;

    Mesh mesh;

    public QualityMeasure() {
        areaMeasure = new AreaMeasure();
        alphaMeasure = new AlphaMeasure();
        qMeasure = new Q_Measure();
    }

    public void update(Mesh mesh) {
        this.mesh = mesh;

        // Reset all measures
        areaMeasure.reset();
        alphaMeasure.reset();
        qMeasure.reset();

        compute();
    }

    private void compute() {
        Point a, b, c;
        double ab, bc, ca;
        double lx, ly;
        double area;

        int n = 0;

        for (var tri : mesh.getTriangles()) {
            n++;

            a = tri.getVertex(0);
            b = tri.getVertex(1);
            c = tri.getVertex(2);

            lx = a.getX() - b.getX();
            ly = a.getY() - b.getY();
            ab = Math.sqrt(lx * lx + ly * ly);
            lx = b.getX() - c.getX();
            ly = b.getY() - c.getY();
            bc = Math.sqrt(lx * lx + ly * ly);
            lx = c.getX() - a.getX();
            ly = c.getY() - a.getY();
            ca = Math.sqrt(lx * lx + ly * ly);

            area = areaMeasure.measure(a, b, c);
            alphaMeasure.measure(ab, bc, ca, area);
            qMeasure.measure(ab, bc, ca, area);
        }

        // Normalize measures
        alphaMeasure.normalize(n, areaMeasure.area_total);
        qMeasure.normalize(n, areaMeasure.area_total);
    }

    /**
     * Determines the bandwidth of the coefficient matrix.
     * <br><br>
     * The quantity computed here is the "geometric" bandwidth determined
     * by the finite element mesh alone.
     *<br><br>
     * If a single finite element variable is associated with each node
     * of the mesh, and if the nodes and variables are numbered in the
     * same way, then the geometric bandwidth is the same as the bandwidth
     * of a typical finite element matrix.
     *<br><br>
     * The bandwidth M is defined in terms of the lower and upper bandwidths:
     *<br>
     *   M = ML + 1 + MU
     * where
     *<br>
     *   ML = maximum distance from any diagonal entry to a nonzero
     *   entry in the same row, but earlier column,
     *<br>
     *   MU = maximum distance from any diagonal entry to a nonzero
     *   entry in the same row, but later column.
     *<br>
     * Because the finite element node adjacency relationship is symmetric,
     * we are guaranteed that ML = MU.
     * @return Bandwidth of the coefficient matrix
     */
    public int bandwidth() {
        if (mesh == null) return 0;

        // Lower and upper bandwidth of the matrix
        int ml = 0, mu = 0;

        int gi, gj;

        for (var tri : mesh.getTriangles()) {
            for (int j = 0; j < 3; j++) {
                gi = tri.getVertex(j).getId();

                for (int k = 0; k < 3; k++) {
                    gj = tri.getVertex(k).getId();

                    mu = Math.max(mu, gj - gi);
                    ml = Math.max(ml, gi - gj);
                }
            }
        }

        return ml + 1 + mu;
    }

    public double getAreaMinimmum() {
        return areaMeasure.area_min;
    }

    public double getAreaMaximum() {
        return areaMeasure.area_max;
    }

    public double getAreaRatio() {
        return areaMeasure.area_max / areaMeasure.area_min;
    }

    public double getAlphaMinimum() {
        return alphaMeasure.alpha_min;
    }

    public double getAlphaMaximum() {
        return alphaMeasure.alpha_max;
    }

    public double getAlphaAverage() {
        return alphaMeasure.alpha_ave;
    }

    public double getAlphaArea() {
        return alphaMeasure.alpha_area;
    }

    public double getQMinimum() {
        return qMeasure.q_min;
    }

    public double getQMaximum() {
        return qMeasure.q_max;
    }

    public double getQAverage() {
        return qMeasure.q_ave;
    }

    public double getQArea() {
        return qMeasure.q_area;
    }

}

class AreaMeasure {

    // Minimum area
    public double area_min = Double.MAX_VALUE;
    // Maximum area
    public double area_max = -Double.MAX_VALUE;
    // Total area of geometry
    public double area_total = 0;
    // Nmber of triangles with zero area
    public int area_zero = 0;

    /**
     * Reset all values
     */
    public void reset() {
        area_min = Double.MAX_VALUE;
        area_max = -Double.MAX_VALUE;
        area_total = 0;
        area_zero = 0;
    }

    /**
     * Computes the area of given triangle
     * @param a Triangle corner a
     * @param b Triangle corner b
     * @param c Triangle corner c
     * @return Triangle area
     */
    public double measure(Point a, Point b, Point c) {
        double area = 0.5 * Math.abs(a.getX() * (b.getY() - c.getY()) + b.getX() * (c.getY() - a.getY()) + c.getX() * (a.getY() - b.getY()));

        area_min = Math.min(area_min, area);
        area_max = Math.max(area_max, area);
        area_total += area;

        if (area == 0.0)
            area_zero = area_zero + 1;

        return area;
    }
}

class AlphaMeasure {

    // Minimum value over all triangles
    public double alpha_min;
    // Maximum value over all triangles
    public double alpha_max;
    // Value averaged over all triangles
    public double alpha_ave;
    // Value averaged over all triangles and weighted by area
    public double alpha_area;

    public void reset() {
        alpha_min = Double.MAX_VALUE;
        alpha_max = -Double.MAX_VALUE;
        alpha_ave = 0;
        alpha_area = 0;
    }

    double acos(double c) {
        if (c <= -1.0)
            return Math.PI;
        else if (1.0 <= c)
            return 0.0;
        else
            return Math.acos(c);
    }

    /**
     * Calculate q of given triangle
     * @param ab Side length ab
     * @param bc Side length bc
     * @param ca Side length ca
     * @param area Triangle area
     */
    public double measure(double ab, double bc, double ca, double area) {
        double alpha = Double.MAX_VALUE;

        double ab2 = ab * ab;
        double bc2 = bc * bc;
        double ca2 = ca * ca;

        double a_angle;
        double b_angle;
        double c_angle;

        // Take care of a ridiculous special case.
        if (ab == 0.0 && bc == 0.0 && ca == 0.0) {
            a_angle = 2.0 * Math.PI / 3.0;
            b_angle = 2.0 * Math.PI / 3.0;
            c_angle = 2.0 * Math.PI / 3.0;
        } else {
            if (ca == 0.0 || ab == 0.0)
                a_angle = Math.PI;
            else
                a_angle = acos((ca2 + ab2 - bc2) / (2.0 * ca * ab));

            if (ab == 0.0 || bc == 0.0)
                b_angle = Math.PI;
            else
                b_angle = acos((ab2 + bc2 - ca2) / (2.0 * ab * bc));

            if (bc == 0.0 || ca == 0.0)
                c_angle = Math.PI;
            else
                c_angle = acos((bc2 + ca2 - ab2) / (2.0 * bc * ca));
        }

        alpha = Math.min(alpha, a_angle);
        alpha = Math.min(alpha, b_angle);
        alpha = Math.min(alpha, c_angle);

        // Normalize angle from [0,pi/3] radians into qualities in [0,1].
        alpha = alpha * 3.0 / Math.PI;

        alpha_ave += alpha;
        alpha_area += area * alpha;

        alpha_min = Math.min(alpha, alpha_min);
        alpha_max = Math.max(alpha, alpha_max);

        return alpha;
    }


    public void normalize(int n, double area_total) {
        if (n > 0)
            alpha_ave /= n;
        else
            alpha_ave = 0.0;

        if (0.0 < area_total)
            alpha_area /= area_total;
        else
            alpha_area = 0.0;
    }
}

class Q_Measure {

    // Minimum value over all triangles
    public double q_min;
    // Maximum value over all triangles
    public double q_max;
    // Average value
    public double q_ave;
    // Average value weighted by the area of each triangle
    public double q_area;

    /**
     * Resets all values
     */
    public void reset() {
        q_min = Double.MAX_VALUE;
        q_max = -Double.MAX_VALUE;
        q_ave = 0;
        q_area = 0;
    }

    /**
     * Compute q value of given triangle.
     * @param ab Side length ab
     * @param bc Side length bc
     * @param ca Side length ca
     * @param area Triangle area
     */
    public double measure(double ab, double bc, double ca, double area) {
        double q = (bc + ca - ab) * (ca + ab - bc) * (ab + bc - ca) / (ab * bc * ca);

        q_min = Math.min(q_min, q);
        q_max = Math.max(q_max, q);

        q_ave += q;
        q_area += q * area;

        return q;
    }

    /**
     * Normalize values
     */
    public void normalize(int n, double area_total) {
        if (n > 0)
            q_ave /= n;
        else
            q_ave = 0.0;

        if (area_total > 0.0)
            q_area /= area_total;
        else
            q_area = 0.0;
    }
}