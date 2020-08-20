package triangle.geometry;

import org.w3c.dom.css.Rect;
import triangle.topology.dcel.Vertex;

import java.util.List;

public class Rectangle {

    double xmin;
    double ymin;
    double xmax;
    double ymax;

    public Rectangle() {
        xmin = ymin = Double.MAX_VALUE;
        xmax = ymax = -Double.MAX_VALUE;
    }

    public Rectangle(Rectangle other) {
        this(other.left(), other.bottom(), other.width(), other.height());
    }

    public Rectangle(double x, double y, double width, double height) {
        xmin = x;
        ymin = y;
        xmax = x + width;
        ymax = y + height;
    }

    /**
     * Update bounds
     * @param dx - Add dx to the left and right bounds
     * @param dy - Add dy to the top and bottom bounds
     */
    public void resize(double dx, double dy) {
       xmin -= dx;
       xmax += dx;
       ymin -= dy;
       ymax += dy;
    }

    /**
     * Expand rectangle to include point
     */
    public void expand(Point p) {
       xmin = Math.min(xmin, p.x);
       ymin = Math.min(ymin, p.y);
       xmax = Math.max(xmax, p.x);
       ymax = Math.max(ymax, p.y);
    }

    /**
     * Expand rectangle to include a list of points
     */
    public void expand(List<Point> points) {
        for (Point p : points)
            expand(p);
    }

    /**
     * Expand rectangle to include given rectangle
     */
    public void expand(Rectangle other) {
       xmin = Math.min(xmin, other.xmin);
       ymin = Math.min(ymin, other.ymin);
       xmax = Math.max(xmax, other.xmax);
       ymax = Math.max(ymax, other.ymax);
    }

    /**
     * Check if given point is inside rectangle
     * @param x - X-coordinate of point to check
     * @param y - Y-coordinate of point to check
     * @return - Returns true if rectangle contains given point
     */
    public boolean contains(double x, double y) {
        return x >= xmin && x <= xmax && y >= ymin && y <= ymax;
    }

    /**
     * Check if given point is inside rectangle
     * @param p - Point to check
     * @return - Returns true if rectangle contains given point
     */
    public boolean contains(Point p) {
        return contains(p.x, p.y);
    }

    /**
     * Check if this rectangle contains other rectangle
     * @param other - Rectangle to check
     * @return - Returns true if this rectangle contains other rectangle
     */
    public boolean contains(Rectangle other) {
        return xmin <= other.left() && other.right() <= xmax
            && ymin <= other.bottom() && other.top() <= ymax;
    }

    /**
     * Check if this rectangle intersects other triangle
     * @param other - Rectangle to check
     * @return - Returns true if this rectangle intersects other rectangle
     */
    public boolean intersects(Rectangle other) {
        return other.left() < xmax && xmin < other.right()
            && other.bottom() < ymax && ymin < other.top();
    }

    public double left() {
        return xmin;
    }

    public double right() {
        return xmax;
    }

    public double bottom() {
        return ymin;
    }

    public double top() {
        return ymax;
    }

    public double width() {
        return xmax - xmin;
    }

    public double height() {
        return ymax - ymin;
    }
}
