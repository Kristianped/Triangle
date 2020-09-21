package triangle;

public interface ISegment extends IEdge {

    public Vertex getVertex(int index);

    public ITriangle getTriangle(int index);
}
