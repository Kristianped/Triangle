package triangle;

import triangle.geometry.ISegment;
import triangle.geometry.ITriangle;
import triangle.geometry.Vertex;

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

    // This is used for 6-nodes triangles. Not used in calculation of the mesh, just storing the vertices
    // sideVertex[0] is between vertices[0] and vertices[1]
    // sideVertex[1] is between vertices[1] and vertices[2]
    // sideVertex[2] is between vertices[2] and vertices[0]
    private Vertex[] sideVertices;

    public Triangle() {
        // Three null vertices
        vertices = new Vertex[3];

        // Three null side vertices
        sideVertices = new Vertex[3];

        // Initialises the three adjoining subsegments to be the omnipresent subsegment
        subsegs = new Osub[3];

        // Initialises the three adjoining triangles to be "outer space"
        neighbors = new Otri[3];

        reset();
    }

    public void reset() {
        for (int i = 0; i < 3; i++) {
            vertices[i] = null;
            sideVertices[i] = null;
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
        updateSideVertices(index);
    }

    /**
     * Calculates and returns the area of the crossection this triangle makes
     * @return Crossectional area of the triangle
     */
    public double getCrossectionArea() {
        double ax = vertices[0].getX(), ay = vertices[0].getY();
        double bx = vertices[1].getX(), by = vertices[1].getY();
        double cx = vertices[2].getX(), cy = vertices[2].getY();

        double a = (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by)) / 2.0;

        return Math.abs(a);
    }

    /**
     * Gets a vertex on the side for 6-noded triangles
     * @param index Index 0 is side between main vertex 0 and 1.
     *          <br>Index 1 is side between main vertex 1 and 2.
     *          <br>Index 2 is side between main vertex 2 and 0.
     * @return The side vertex of the index
     */
    public Vertex getSideVertex(int index) {
        return sideVertices[index];
    }

    private void updateSideVertices(int index) {
        Vertex a = vertices[0];
        Vertex b = vertices[1];
        Vertex c = vertices[2];

        if (index == 0) {            // Update sides 0->1 and 2->0
            if (a != null && b != null)
                sideVertices[0] = getHalfway(a, b);

            if (a != null && c != null)
                sideVertices[2] = getHalfway(c, a);
        } else if (index == 1) {    // Update sides 0->1 and 1->2
            if (a != null && b != null)
                sideVertices[0] = getHalfway(a, b);

            if (b != null && c != null)
                sideVertices[1] = getHalfway(b, c);
        } else if (index == 2) {    // Update siden 1->2 and 2->0
            if (b != null && c != null)
                sideVertices[1] = getHalfway(b, c);

            if (a != null && c != null)
                sideVertices[2] = getHalfway(c, a);
        }
    }

    private Vertex getHalfway(Vertex a, Vertex b) {
        double x = (a.getX() + b.getX()) / 2.0;
        double y = (a.getY() + b.getY()) / 2.0;

        return new Vertex(x, y);
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
        return String.format("{TID %d}", hash);
    }
}
