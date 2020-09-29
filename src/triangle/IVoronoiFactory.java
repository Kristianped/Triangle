package triangle;

import triangle.Vertex;
import triangle.Face;
import triangle.HalfEdge;
import triangle.DcelVertex;

public interface IVoronoiFactory {

    void initialize(int vertexCount, int edgeCount, int faceCount);

    void reset();

    DcelVertex createVertex(double x, double y);

    HalfEdge createHalfEdge(DcelVertex origin, Face face);

    Face createFace(Vertex vertex);
}
