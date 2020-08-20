package triangle.geometry;

public class Segment implements ISegment {

    Vertex v0;
    Vertex v1;
    int label;

    public Segment(Vertex v0, Vertex v1) {
        this(v0, v1, 0);
    }

    public Segment(Vertex v0, Vertex v1, int label) {
        this.v0 = v0;
        this.v1 = v1;
        this.label = label;
    }

    @Override
    public Vertex getVertex(int index) {
        if (index == 0)
            return v0;
        else if (index == 1)
            return v1;
        else
            throw new IndexOutOfBoundsException("Vertex-index out of bounds");
    }

    @Override
    public ITriangle getTriangle(int index) {
        throw new UnsupportedOperationException("Not supported yet");
    }

    @Override
    public int getP0() {
        return v0.id;
    }

    @Override
    public int getP1() {
        return v1.id;
    }

    @Override
    public int getLabel() {
        return label;
    }

    public void setLabel(int value) {
        this.label = value;
    }
}
