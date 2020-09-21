package triangle;

public class InputTriangle implements ITriangle {

    int[] vertices;
    int label;
    double area;

    public InputTriangle(int p0, int p1, int p2) {
        this.vertices = new int[] { p0, p1, p2 };
    }

    @Override
    public int getID() {
        return 0;
    }

    @Override
    public void setID(int id) {

    }

    @Override
    public int getLabel() {
        return label;
    }

    @Override
    public void setLabel(int label) {
        this.label = label;
    }

    @Override
    public double getArea() {
        return area;
    }

    @Override
    public void setArea(double area) {
        this.area = area;
    }

    @Override
    public Vertex getVertex(int index) {
        return null;
    }

    @Override
    public int getVertexID(int index) {
        return vertices[index];
    }

    @Override
    public ITriangle getNeighbor(int index) {
        return null;
    }

    @Override
    public int getNeighborID(int index) {
        return -1;
    }

    @Override
    public ISegment getSegment(int index) {
        return null;
    }
}
