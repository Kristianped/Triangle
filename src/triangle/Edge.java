package triangle;

public class Edge implements IEdge {

    private int P0;
    private int P1;
    private int label;

    public Edge(int p0, int p1) {
        this(p0, p1, 0);
    }

    public Edge(int p0, int p1, int label) {
        this.P0 = p0;
        this.P1 = p1;
        this.label = label;
    }

    @Override
    public int getP0() {
        return P0;
    }

    @Override
    public int getP1() {
        return P1;
    }

    @Override
    public int getLabel() {
        return label;
    }
}
