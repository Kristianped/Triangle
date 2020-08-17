package triangle.geometry;

import triangle.topology.dcel.Vertex;

public interface ISegment extends IEdge {

    public Vertex getVertex(int index);

    public ITriangle getTriangle(int index);
}
