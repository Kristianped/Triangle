package triangle;

public class SubSegment implements ISegment {

    int hash;

    Osub[] subsegs;
    Vertex[] vertices;
    Otri[] triangles;
    int boundary;

    public SubSegment() {
        // Four NULL vertices
        vertices = new Vertex[4];

        // Set the boundary marker to zero
        boundary = 0;

        // Initialize the two adjoining subsegments to be omnipresent subsegment
        subsegs = new Osub[2];
        subsegs[0] = new Osub();
        subsegs[1] = new Osub();


        // Initialize the two adjoining triangles to be "outer space"
        triangles = new Otri[2];
        triangles[0] = new Otri();
        triangles[1] = new Otri();
    }

    @Override
    public String toString() {
        return String.format("SID %d", hash);
    }

    @Override
    public int hashCode() {
        return hash;
    }

    @Override
    public Vertex getVertex(int index) {
        return vertices[index];
    }

    @Override
    public ITriangle getTriangle(int index) {
        return triangles[index].tri.hash == Mesh.DUMMY ? null : triangles[index].tri;
    }

    @Override
    public int getP0() {
        return vertices[0].getId();
    }

    @Override
    public int getP1() {
        return vertices[1].getId();
    }

    @Override
    public int getLabel() {
        return boundary;
    }
}
