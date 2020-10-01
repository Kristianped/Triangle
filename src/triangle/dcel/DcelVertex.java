package triangle.dcel;

import triangle.geometry.Point;

public class DcelVertex extends Point {

    protected HalfEdge leaving;

    public DcelVertex(double x, double y) {
        super(x, y);
    }

    public DcelVertex(double x, double y, HalfEdge leaving) {
        super(x, y);
        this.leaving = leaving;
    }

    @Override
    public String toString() {
        return String.format("V-ID {0}", super.getId());
    }

    public HalfEdge getLeaving() {
        return leaving;
    }

    public void setLeaving(HalfEdge leaving) {
        this.leaving = leaving;
    }
}
