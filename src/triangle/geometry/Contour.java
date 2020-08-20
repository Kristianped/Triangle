package triangle.geometry;

import triangle.RobustPredicates;

import java.util.ArrayList;
import java.util.List;

public class Contour {

    int marker;
    boolean convex;
    List<Vertex> points;

    /**
     * Initializes a new instance of the Contour class
     * @param points - The points that make up the contour
     */
    public Contour(Iterable<Vertex> points) {
        this(points, 0);
    }

    /**
     * Initializes a new instance of the Contour class
     * @param points - The points that make up the contour
     * @param marker - Contour marker
     */
    public Contour(Iterable<Vertex> points, int marker) {
        this(points, marker, false);
    }

    /**
     * Initializes a new instance of the Contour class
     * @param points - The points that make up the contour
     * @param marker - Contour marker
     * @param convex - The hole is convex or not
     */
    public Contour(Iterable<Vertex> points, int marker, boolean convex) {
        addPoints(points);
        this.marker = marker;
        this.convex = convex;
    }

    public List<Vertex> getPoints() {
        return points;
    }

    public void setPoints(List<Vertex> points) {
        this.points = points;
    }

    public List<ISegment> getSegments() {
        List<ISegment> segments = new ArrayList<>();
        var p = this.points;
        int count = p.size() - 1;

        for (int i = 0; i < count; i++)
            segments.add(new Segment(p.get(i), p.get(i + 1), marker));

        // Close the segment
        segments.add(new Segment(p.get(count), p.get(0), marker));

        return segments;
    }

    /**
     * Try to find a point inside the contour.
     * Note: For each corner (index i) of the contour, the 3 points with indices i-1, i and i+1
     * are considered and a search on the line through the corner vertex is started (either
     * on the bisecting line, or, if {@link triangle.IPredicates#counterClockwise} is less than
     * eps, on the perpendicular line.
     * A given number of points will be tested (limit), while the distance to the contour boundary
     * will be reduced in each iteration (with a factor 1 / 2î, i = 1 ... limit).
     * @return - Point inside the contour
     * @throws Exception - If no point could be found
     */
    public Point findInteriorPoint() throws Exception {
        return findInteriorPoint(5, 2e-5);
    }

    /**
     * Try to find a point inside the contour.
     * Note: For each corner (index i) of the contour, the 3 points with indices i-1, i and i+1
     * are considered and a search on the line through the corner vertex is started (either
     * on the bisecting line, or, if {@link triangle.IPredicates#counterClockwise} is less than
     * eps, on the perpendicular line.
     * A given number of points will be tested (limit), while the distance to the contour boundary
     * will be reduced in each iteration (with a factor 1 / 2î, i = 1 ... limit).
     * @param limit - The number of iterations on each segment
     * @return - Point inside the contour
     * @throws Exception - If no point could be found
     */
    public Point findInteriorPoint(int limit) throws Exception {
        return findInteriorPoint(limit, 2e-5);
    }

    /**
     * Try to find a point inside the contour.
     * Note: For each corner (index i) of the contour, the 3 points with indices i-1, i and i+1
     * are considered and a search on the line through the corner vertex is started (either
     * on the bisecting line, or, if {@link triangle.IPredicates#counterClockwise} is less than
     * eps, on the perpendicular line.
     * A given number of points will be tested (limit), while the distance to the contour boundary
     * will be reduced in each iteration (with a factor 1 / 2î, i = 1 ... limit).
     * @param eps - Threshold for co-linear points
     * @return - Point inside the contour
     * @throws Exception - If no point could be found
     */
    public Point findInteriorPoint(double eps) throws Exception {
        return findInteriorPoint(5, eps);
    }

    /**
     * Try to find a point inside the contour.
     * Note: For each corner (index i) of the contour, the 3 points with indices i-1, i and i+1
     * are considered and a search on the line through the corner vertex is started (either
     * on the bisecting line, or, if {@link triangle.IPredicates#counterClockwise} is less than
     * eps, on the perpendicular line.
     * A given number of points will be tested (limit), while the distance to the contour boundary
     * will be reduced in each iteration (with a factor 1 / 2î, i = 1 ... limit).
     * @param limit - The number of iterations on each segment
     * @param eps - Threshold for co-linear points
     * @return - Point inside the contour
     * @throws Exception - If no point could be found
     */
    public Point findInteriorPoint(int limit, double eps) throws Exception {
        if (convex) {
            int count = this.points.size();
            var point = new Point(0, 0);

            for (int i = 0; i < count; i++) {
                point.x += this.points.get(i).x;
                point.y += this.points.get(i).y;
            }

            // If the contour is convex, use its centroid
            point.x /= count;
            point.y /= count;

            return point;
        }

        return findPointInPolygon(this.points, limit, eps);
    }

    private static Point findPointInPolygon(List<Vertex> contour, int limit, double eps) throws Exception {
        var bounds = new Rectangle();

        for (Vertex v : contour)
            bounds.expand(v);

        int length = contour.size();
        var test = new Point();

        Point a, b, c;  // Current corner points

        double bx, by;
        double dx, dy;
        double h;

        var predicates = new RobustPredicates();

        a = contour.get(0);
        b = contour.get(1);

        for (int i = 0; i < length; i++) {
            c = contour.get((i + 2) % length);

            // Corner point
            bx = b.x;
            by = b.y;

            // NOTE: if we knew the contour points were in counterclockwise order,
            // we could skip concave corners and search only in one direction.

            h = predicates.counterClockwise(a, b, c);

            if (Math.abs(h) < eps) {
                // Points are nearly co-linear. Use perpendicular direction
                dx = (c.y - a.y) / 2;
                dy = (a.x - c.x) / 2;
            } else {
                // Direction [midpoint(a-c) -> corner point
                dx = (a.x + c.x) / 2 - bx;
                dy = (a.y + c.y) / 2 - by;
            }

            // Move around the corner
            a = b;
            b = c;
            h = 1;

            for (int j = 0; j < limit; j++) {
                // Search in direction
                test.x = bx + dx * h;
                test.y = by + dy * h;

                if (bounds.contains(test) && isPointInPolygon(test, contour))
                    return test;

                // Search in opposite direction
                test.x = bx - dx * h;
                test.y = by - dy * h;

                if (bounds.contains(test) && isPointInPolygon(test, contour))
                    return test;

                h = h / 2;
            }
        }

        throw new Exception("Point not found");
    }

    private static boolean isPointInPolygon(Point p, List<Vertex> poly) {
        boolean inside = false;
        int count = poly.size();
        double x = p.x;
        double y = p.y;

        // Jordan curve theorem
        for (int i = 0, j = count - 1; i < count; j = i++) {
            if ((poly.get(i).y > y) != (poly.get(j).y > y) &&
                    x < (poly.get(j).x - poly.get(i).x) * (y - poly.get(i).y) / (poly.get(j).y - poly.get(i).y) + poly.get(i).x)
                inside = !inside;
        }

        return inside;
    }


    private void addPoints(Iterable<Vertex> points) {
        this.points = new ArrayList<>();

        for (Vertex v : points)
            this.points.add(v);

        int count = this.points.size();

        // Check if first vertex is equal to last vertex
        if (this.points.get(0) == this.points.get(count))
            this.points.remove(count);
    }
}
