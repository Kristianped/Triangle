package triangle.topology.dcel;

import triangle.geometry.Point;

public class Face {

    public static final Face Empty;

    static {
        Empty = new Face(null);
        Empty.setId(-1);
    }

    protected int id;
    protected int mark;

    protected Point generator;
    protected HalfEdge edge;
    protected boolean bounded;

    public Face(Point generator) {
        this(generator, null);
    }

    public Face(Point generator, HalfEdge edge) {
        this.generator = generator;
        this.setEdge(edge);
        this.setBounded(true);
    }

    @Override
    public String toString() {
        return String.format("F-ID {0}", id);
    }

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }

    public HalfEdge getEdge() {
        return edge;
    }

    public void setEdge(HalfEdge edge) {
        this.edge = edge;
    }

    public boolean isBounded() {
        return bounded;
    }

    public void setBounded(boolean bounded) {
        this.bounded = bounded;
    }
}
