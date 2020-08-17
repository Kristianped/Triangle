package triangle.geometry;

public class Point implements Comparable<Point> {

    protected int id;
    protected int label;

    protected double x;
    protected double y;

    public Point() {
        this(0, 0, 0);
    }

    public Point(double x, double y) {
        this(x, y, 0);
    }

    public Point(double x, double y, int label) {
        this.setX(x);
        this.setY(y);
        this.setLabel(label);
    }

    @Override
    public int hashCode() {
        int hash = 19;
        hash = hash * 31 + Double.hashCode(x);
        hash = hash * 31 + Double.hashCode(y);

        return hash;
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof Point))
            return false;

        if (obj == null)
            return false;

        return getX() == ((Point) obj).getX() && getY() == ((Point) obj).getY();
    }

    @Override
    public int compareTo(Point o) {
        if (getX() == o.getX() && getY() == o.getY())
            return 0;

        return (x < o.getX() || (x == o.getX() && y < o.getY())) ? -1 : 1;
    }

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }

    public int getLabel() {
        return label;
    }

    public void setLabel(int label) {
        this.label = label;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }
}
