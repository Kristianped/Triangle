package triangle.geometry;

public class RegionPointer {

    Point point;
    int id;
    double area;

    public RegionPointer(double x, double y, int id) {
        this(x, y, id, 0);
    }

    public RegionPointer(double x, double y, int id, double area) {
        this.point = new Point(x, y);
        this.id = id;
        this.area = area;
    }

    public Point getPoint() {
        return point;
    }

    public double getArea() {
        return area;
    }

    public int getId() {
        return id;
    }

    public void setArea(double area) {
        if (area < 0.0)
            throw new IllegalArgumentException("Area constraints must be positive");

        this.area = area;
    }
}
