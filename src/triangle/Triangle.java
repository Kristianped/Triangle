package triangle;

public class Triangle implements ITriangle {

    // Hash for dictionary. Will be set by mesh instance.
    int hash;

    // The ID is only used for mesh output.
    int id;

    Otri[] neighbors;
    Vertex[] vertices;
    Osub[] subsegs;
    int label;
    double area;
    boolean infected;

    public Triangle() {
        // Three null vertices
        vertices = new Vertex[3];

        // Initialises the three adjoining subsegments to be the omnipresent subsegment
        subsegs = new Osub[3];

        // Initialises the three adjoining triangles to be "outer space"
        neighbors = new Otri[3];

        reset();
    }

    public void reset() {
        for (int i = 0; i < 3; i++) {
            vertices[i] = null;
            subsegs[i] = new Osub();
            neighbors[i] = new Otri();
        }
    }

    public Otri[] getNeighbors() {
        return neighbors;
    }

    public Osub[] getSubsegs() {
        return subsegs;
    }

    public void setHash(int hash) {
        this.hash = hash;
    }

    public boolean isInfected() {
        return infected;
    }

    public void setInfected(boolean infected) {
        this.infected = infected;
    }

    public void setVertex(int index, Vertex v) {
        vertices[index] = v;
    }

    @Override
    public int getID() {
        return id;
    }

    @Override
    public void setID(int id) {
        this.id = id;
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
        return vertices[index];
    }

    @Override
    public int getVertexID(int index) {
        return vertices[index].id;
    }

    @Override
    public ITriangle getNeighbor(int index) {
        return neighbors[index].tri.hash == Mesh.DUMMY ? null : neighbors[index].tri;
    }

    @Override
    public int getNeighborID(int index) {
        return neighbors[index].tri.hash == Mesh.DUMMY ? -1 : neighbors[index].tri.id;
    }

    @Override
    public ISegment getSegment(int index) {
        return subsegs[index].seg.hash == Mesh.DUMMY ? null : subsegs[index].seg;
    }

    @Override
    public int hashCode() {
        return this.hash;
    }

    @Override
    public String toString() {
        return String.format("{TID %d}", hash);
    }
}
