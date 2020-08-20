package triangle;

import triangle.geometry.Point;

public class RobustPredicates implements IPredicates {

    private static final Object creationLock = new Object();
    private static RobustPredicates _default;

    private static double epsilon, splitter, resulterrbound;
    private static double ccwerrboundA, ccwerrboundB, ccwerrboundC;
    private static double iccerrboundA, iccerrboundB, iccerrboundC;

    public static RobustPredicates getDefault() {
        if (_default == null) {
            synchronized (creationLock) {
                if (_default == null)
                    _default = new RobustPredicates();
            }
        }

        return _default;
    }

    static {
        double half;
        double check, lastcheck;
        boolean every_other;

        every_other = true;
        half = 0.5;
        epsilon = 1.0;
        splitter = 1.0;
        check = 1.0;
        // Repeatedly divide 'epsilon' by two until it is too small to add to
        // one without causing roundoff.  (Also check if the sum is equal to
        // the previous sum, for machines that round up instead of using exact
        // rounding.  Not that these routines will work on such machines.)
        do
        {
            lastcheck = check;
            epsilon *= half;
            if (every_other)
            {
                splitter *= 2.0;
            }
            every_other = !every_other;
            check = 1.0 + epsilon;
        } while ((check != 1.0) && (check != lastcheck));
        splitter += 1.0;
        // Error bounds for orientation and incircle tests.
        resulterrbound = (3.0 + 8.0 * epsilon) * epsilon;
        ccwerrboundA = (3.0 + 16.0 * epsilon) * epsilon;
        ccwerrboundB = (2.0 + 12.0 * epsilon) * epsilon;
        ccwerrboundC = (9.0 + 64.0 * epsilon) * epsilon * epsilon;
        iccerrboundA = (10.0 + 96.0 * epsilon) * epsilon;
        iccerrboundB = (4.0 + 48.0 * epsilon) * epsilon;
        iccerrboundC = (44.0 + 576.0 * epsilon) * epsilon * epsilon;
    }

    public RobustPredicates() {
        allocateWorkspace();
    }

    @Override
    public double counterClockwise(Point a, Point b, Point c) {
        return 0;
    }

    @Override
    public double inCircle(Point a, Point b, Point c, Point d) {
        return 0;
    }

    @Override
    public Point findCircumcenter(Point org, Point dest, Point apex, double xi, double eta) {
        return null;
    }

    @Override
    public Point findCircumcenter(Point org, Point dest, Point apex, double xi, double eta, double offconstant) {
        return null;
    }

    // InCircleAdapt workspace:
    double[] fin1, fin2, abdet;

    double[] axbc, axxbc, aybc, ayybc, adet;
    double[] bxca, bxxca, byca, byyca, bdet;
    double[] cxab, cxxab, cyab, cyyab, cdet;

    double[] temp8, temp16a, temp16b, temp16c;
    double[] temp32a, temp32b, temp48, temp64;

    private void allocateWorkspace()
    {
        fin1 = new double[1152];
        fin2 = new double[1152];
        abdet = new double[64];

        axbc = new double[8];
        axxbc = new double[16];
        aybc = new double[8];
        ayybc = new double[16];
        adet = new double[32];

        bxca = new double[8];
        bxxca = new double[16];
        byca = new double[8];
        byyca = new double[16];
        bdet = new double[32];

        cxab = new double[8];
        cxxab = new double[16];
        cyab = new double[8];
        cyyab = new double[16];
        cdet = new double[32];

        temp8 = new double[8];
        temp16a = new double[16];
        temp16b = new double[16];
        temp16c = new double[16];

        temp32a = new double[32];
        temp32b = new double[32];
        temp48 = new double[48];
        temp64 = new double[64];
    }
}
