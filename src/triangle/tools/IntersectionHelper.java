package triangle.tools;

import triangle.Point;
import triangle.Rectangle;

public class IntersectionHelper {

    /**
     * Compute intersection of two segments.
     * <br><br>
     * This is a special case of segment intersection. Since the calling algorithm assures
     * that a valid intersection exists, there's no need to check for any special cases.
     * @param p0 Segment 1 start point
     * @param p1 Segement 1 end point
     * @param q0 Segement 2 start point
     * @param q1 Segment 2 end point
     * @param c0 The intersection point
     */
    public static void intersectSegments(Point p0, Point p1, Point q0, Point q1, Point c0) {
        double ux = p1.getX() - p0.getX();
        double uy = p1.getY() - p0.getY();
        double vx = q1.getX() - q0.getX();
        double vy = q1.getY() - q0.getY();
        double wx = p0.getX() - q0.getX();
        double wy = p0.getY() - q0.getY();

        double d = (ux * vy - uy * vx);
        double s = (vx * wy - vy * wx) / d;

        // Intersection point
        c0.setX(p0.getX() + s * ux);
        c0.setY(p0.getY() + s * uy);
    }

    /**
     * Intersect segment with a bounding box.
     * <br><br>
     * Based on Liang-Barsky function by Daniel White:
     * http://www.skytopia.com/project/articles/compsci/clipping.html
     * @param rect The clip rectangle
     * @param p0 Segment endpoint
     * @param p1 Segment endpoint
     * @param c0 The new location of p0
     * @param c1 The new location of p1
     * @return Returns true if point found
     */
    public static boolean liangBarsky(Rectangle rect, Point p0, Point p1, Point c0, Point c1) {
        // Define the x/y clipping values for the border.
        double xmin = rect.left();
        double xmax = rect.right();
        double ymin = rect.bottom();
        double ymax = rect.top();

        // Define the start and end points of the line.
        double x0 = p0.getX();
        double y0 = p0.getY();
        double x1 = p1.getX();
        double y1 = p1.getY();

        double t0 = 0.0;
        double t1 = 1.0;

        double dx = x1 - x0;
        double dy = y1 - y0;

        double p = 0.0, q = 0.0, r;

        for (int edge = 0; edge < 4; edge++) {
            // Traverse through left, right, bottom, top edges.
            if (edge == 0) { p = -dx; q = -(xmin - x0); }
            if (edge == 1) { p = dx; q = (xmax - x0); }
            if (edge == 2) { p = -dy; q = -(ymin - y0); }
            if (edge == 3) { p = dy; q = (ymax - y0); }
            r = q / p;
            if (p == 0 && q < 0) return false; // Don't draw line at all. (parallel line outside)

            if (p < 0) {
                if (r > t1) return false; // Don't draw line at all.
                else if (r > t0) t0 = r; // Line is clipped!
            } else if (p > 0) {
                if (r < t0) return false; // Don't draw line at all.
                else if (r < t1) t1 = r; // Line is clipped!
            }
        }

        c0.setX(x0 + t0 * dx);
        c0.setY(y0 + t0 * dy);
        c1.setX(x0 + t1 * dx);
        c1.setY(y0 + t1 * dy);

        return true; // (clipped) line is drawn
    }

    /**
     * Intersect a ray with a bounding box
     * @param rect The clip rectangle
     * @param p0 The ray startpoint
     * @param p1 Any point in ray direction (NOT the direction vector)
     * @param c1 The intersection point
     * @return Returns false, if startpoint is outside the box
     */
    public static boolean boxRayIntersection(Rectangle rect, Point p0, Point p1, Point c1)
    {
        return boxRayIntersection(rect, p0, p1.getX() - p0.getX(), p1.getY() - p0.getY(), c1);
    }

    /**
     * Intersect a ray with a bounding box
     * @param rect The clip rectangle
     * @param p The ray startpoint
     * @param dx X direction
     * @param dy Y direction
     * @return Returns intersection point if found, or null if not found
     */
    public static Point boxRayIntersection(Rectangle rect, Point p, double dx, double dy) {
        var intersection = new Point();

        if (boxRayIntersection(rect, p, dx, dy, intersection))
            return intersection;

        return null;
    }

    /**
     * Intersect a ray with a bounding box
     * @param rect The clip rectangle
     * @param p The ray startpoint
     * @param dx X direction
     * @param dy Y direction
     * @param c The intersection point
     * @return Returns false, if startpoint is outside the box
     */
    public static boolean boxRayIntersection(Rectangle rect, Point p, double dx, double dy, Point c) {
        double x = p.getX();
        double y = p.getY();

        double t1, x1, y1, t2, x2, y2;

        // Bounding box
        double xmin = rect.left();
        double xmax = rect.right();
        double ymin = rect.bottom();
        double ymax = rect.top();

        // Check if point is inside the bounds
        if (x < xmin || x > xmax || y < ymin || y > ymax)
            return false;

        // Calculate the cut through the vertical boundaries
        if (dx < 0) {
            // Line going to the left: intersect with x = minX
            t1 = (xmin - x) / dx;
            x1 = xmin;
            y1 = y + t1 * dy;
        } else if (dx > 0) {
            // Line going to the right: intersect with x = maxX
            t1 = (xmax - x) / dx;
            x1 = xmax;
            y1 = y + t1 * dy;
        } else {
            // Line going straight up or down: no intersection possible
            t1 = Double.MAX_VALUE;
            x1 = y1 = 0;
        }

        // Calculate the cut through upper and lower boundaries
        if (dy < 0) {
            // Line going downwards: intersect with y = minY
            t2 = (ymin - y) / dy;
            x2 = x + t2 * dx;
            y2 = ymin;
        } else if (dy > 0) {
            // Line going upwards: intersect with y = maxY
            t2 = (ymax - y) / dy;
            x2 = x + t2 * dx;
            y2 = ymax;
        } else {
            // Horizontal line: no intersection possible
            t2 = Double.MAX_VALUE;
            x2 = y2 = 0;
        }

        if (t1 < t2) {
            c.setX(x1);
            c.setY(y1);
        }
        else {
            c.setX(x2);
            c.setY(y2);
        }

        return true;
    }
}
