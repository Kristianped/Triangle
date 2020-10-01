package triangle;

import triangle.tools.MutableDouble;

public interface IPredicates {

    public double counterClockwise(Point a, Point b, Point c);

    public double inCircle(Point a, Point b, Point c, Point d);

    public Point findCircumcenter(Point org, Point dest, Point apex, MutableDouble xi, MutableDouble eta);

    public Point findCircumcenter(Point org, Point dest, Point apex, MutableDouble xi, MutableDouble eta, double offconstant);
}
