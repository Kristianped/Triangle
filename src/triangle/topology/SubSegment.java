package triangle.topology;

import triangle.Mesh;
import triangle.geometry.ISegment;
import triangle.geometry.ITriangle;
import triangle.geometry.Vertex;

public class SubSegment implements ISegment {

    protected int hash;

    protected Osub[] subsegs;
    protected Vertex[] vertices;
    protected Otri[] triangles;
    protected int boundary;

    public SubSegment() {
        // Four NULL vertices
        vertices = new Vertex[4];

        // Set the boundary marker to zero
        boundary = 0;

        // Initialize the two adjoining subsegments to be omnipresent subsegment
        subsegs = new Osub[2];

        // Initialize the two adjoining triangles to be "outer space"
        triangles = new Otri[2];
    }

    @Override
    public String toString() {
        return String.format("SID {0}", hash);
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
