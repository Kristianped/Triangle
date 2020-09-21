package triangle;

import java.util.HashSet;
import java.util.Set;

public class PolygonValidator {

    public static boolean isConsistent(IPolygon poly) {
        var points = poly.getPoints();

        int horrors = 0;

        int i = 0;
        int count = points.size();

        if (count < 3) {
            System.err.println("Polygon must have at least 3 vertices: PolygonValidator.IsConsistent()");
            return false;
        }

        for (var p : points) {
            if (p == null) {
                horrors++;
                String s = String.format("Point {0} is null: PolygonValidator.IsConsistent()", i);
                System.err.println(s);
            }
            else if (Double.isNaN(p.x) || Double.isNaN(p.y)) {
                horrors++;
                System.err.println(String.format("Point {0} has invalid coordinates: PolygonValidator.IsConsistent()", i));
            }
            else if (Double.isInfinite(p.x) || Double.isInfinite(p.y)) {
                horrors++;
                System.err.println(String.format("Point {0} has invalid coordinates: PolygonValidator.IsConsistent()", i));
            }

            i++;
        }

        i = 0;

        for (var seg : poly.getSegments()) {
            if (seg == null) {
                horrors++;
                System.err.println(String.format("Segment {0} is null: PolygonValidator.IsConsistent()", i));

                // Always abort if a NULL-segment is found.
                return false;
            }

            var p = seg.getVertex(0);
            var q = seg.getVertex(1);

            if ((p.x == q.x) && (p.y == q.y)) {
                horrors++;
                System.err.println(String.format("Endpoints of segment {0} are coincident (IDs {1} / {2}): PolygonValidator.IsConsistent()", i, p.id, q.id));
            }

            i++;
        }

        if (points.get(0).id == points.get(1).id)
            horrors += checkVertexIDs(poly, count);
        else
            horrors += checkDuplicateIDs(poly);

        return horrors == 0;
    }

    public static boolean hasDuplicateVertices(IPolygon poly) {
        int horrors = 0;

        var points = poly.getPoints().toArray(new Vertex[0]);

        VertexSorter.sort(points);

        for (int i = 1; i < points.length; i++) {
            if (points[i - 1] == points[i]) {
                horrors++;
                System.err.println(String.format("Found duplicate point {0}: PolygonValidator.HasDuplicateVertices()", points[i]));
            }
        }

        return horrors > 0;
    }

    public static boolean hasBadAngles(IPolygon poly) {
        return hasBadAngles(poly, 2e-12);
    }

    public static boolean hasBadAngles(IPolygon poly, double threshold) {
        int horrors = 0;
        int i = 0;

        Point p0 = null, p1 = null;
        Point q0, q1;

        int count = poly.getPoints().size();

        for (var seg : poly.getSegments()) {
            q0 = p0; // Previous segment start point.
            q1 = p1; // Previous segment end point.

            p0 = seg.getVertex(0); // Current segment start point.
            p1 = seg.getVertex(1); // Current segment end point.

            if (p0 == p1 || q0 == q1) {
                // Ignore zero-length segments.
                continue;
            }

            if (q0 != null && q1 != null) {
                // The two segments are connected.
                if (p0 == q1 && p1 != null) {
                    if (isBadAngle(q0, p0, p1,threshold)) {
                        horrors++;
                        System.err.println(String.format("Bad segment angle found at index {0}: PolygonValidator.HasBadAngles()", i));
                    }
                }
            }

            i++;
        }

        return horrors > 0;
    }

    private static boolean isBadAngle(Point a, Point b, Point c, double threshold) {
        double x = dotProduct(a, b, c);
        double y = crossProductLength(a, b, c);

        return Math.abs(Math.atan2(y, x)) <= threshold;
    }

    private static double dotProduct(Point a, Point b, Point c) {
        //  Calculate the dot product.
        return (a.x - b.x) * (c.x - b.x) + (a.y - b.y) * (c.y - b.y);
    }

    private static double crossProductLength(Point a, Point b, Point c) {
        //  Calculate the Z coordinate of the cross product.
        return (a.x - b.x) * (c.y - b.y) - (a.y - b.y) * (c.x - b.x);
    }

    private static int checkVertexIDs(IPolygon poly, int count) {
        int horrors = 0;

        int i = 0;

        Vertex p, q;

        for (var seg : poly.getSegments()) {
            p = seg.getVertex(0);
            q = seg.getVertex(1);

            if (p.id < 0 || p.id >= count) {
                horrors++;
                System.err.println(String.format("Segment {0} has invalid startpoint: PolygonValidator.CheckVertexIDs()", i));
            }

            if (q.id < 0 || q.id >= count) {
                horrors++;
                System.err.println(String.format("Segment {0} has invalid endpoint: PolygonValidator.CheckVertexIDs()", i));
            }

            i++;
        }

        return horrors;
    }

    private static int checkDuplicateIDs(IPolygon poly) {
        Set<Integer> ids = new HashSet<>();

        // Check for duplicate ids.
        for (var p : poly.getPoints()) {
            if (!ids.add(p.id)) {
                System.err.println("Found duplicate vertex ids: PolygonValidator.CheckDuplicateIDs");
                return 1;
            }
        }

        return 0;
    }
}
