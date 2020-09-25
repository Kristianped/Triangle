package triangle;

public class TriangleHelper {

    private TriangleHelper() {

    }

    /**
     * Test whether a given point lies inside a triangle or not.
     * @param triangle Triangle to test
     * @param p Point to locate
     * @return True if point is inside or on the edge of this triangle
     */
    public static boolean contains(ITriangle triangle, Point p) {
        return contains(triangle, p.x, p.y);
    }

    /**
     * Test whether a given point lies inside a triangle or not.
     * @param triangle Triangle to test
     * @param x X coordinate of point
     * @param y Y coordinate of point
     * @return True if point is inside or on the edge of this triangle
     */
    public static boolean contains(ITriangle triangle, double x, double y) {
        var t0 = triangle.getVertex(0);
        var t1 = triangle.getVertex(1);
        var t2 = triangle.getVertex(2);

        // TODO: no need to create new Point instances here
        Point d0 = new Point(t1.x - t0.x, t1.y - t0.y);
        Point d1 = new Point(t2.x - t0.x, t2.y - t0.y);
        Point d2 = new Point(x - t0.x, y - t0.y);

        // crossproduct of (0, 0, 1) and d0
        Point c0 = new Point(-d0.y, d0.x);

        // crossproduct of (0, 0, 1) and d1
        Point c1 = new Point(-d1.y, d1.x);

        // Linear combination d2 = s * d0 + v * d1.
        //
        // Multiply both sides of the equation with c0 and c1
        // and solve for s and v respectively
        //
        // s = d2 * c1 / d0 * c1
        // v = d2 * c0 / d1 * c0

        double s = dotProduct(d2, c1) / dotProduct(d0, c1);
        double v = dotProduct(d2, c0) / dotProduct(d1, c0);

        if (s >= 0 && v >= 0 && ((s + v) <= 1))
        {
            // Point is inside or on the edge of this triangle.
            return true;
        }

        return false;
    }

    /**
     * Gets the bounds of a triangle
     * @param triangle Triangle to find bounds of
     * @return A rectangle with the bounds
     */
    public static Rectangle getBounds(ITriangle triangle) {
        var bounds = new Rectangle();

        for (int i = 0; i < 3; i++)
            bounds.expand(triangle.getVertex(i));

        return bounds;
    }

    private static double dotProduct(Point p, Point q) {
        return p.x * q.x + p.y * q.y;
    }
}
