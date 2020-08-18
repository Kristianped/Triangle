package triangle.topology;

import triangle.Mesh;
import triangle.geometry.ISegment;
import triangle.geometry.ITriangle;
import triangle.topology.dcel.Vertex;

public class Triangle implements ITriangle {

    // Hash for dictionary. Will be set by mesh instance.
    protected int hash;

    // The ID is only used for mesh output.
    protected int id;

    protected Otri[] neighbors;
    protected Vertex[] vertices;
    protected Osub[] subsegs;
    protected int label;
    protected double area;
    protected boolean infected;

    public Triangle() {
        // Three null vertices
        vertices = new Vertex[3];

        // Initialises the three adjoining subsegments to be the omnipresent subsegment
        subsegs = new Osub[3];

        // Initialises the three adjoining triangles to be "outer space"
        neighbors = new Otri[3];
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
        return vertices[index].getId();
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
        return String.format("TID {0}", hash);
    }
}
