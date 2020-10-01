package triangle.voronoi;

import triangle.geometry.Vertex;
import triangle.dcel.Face;
import triangle.dcel.HalfEdge;
import triangle.dcel.DcelVertex;

public interface IVoronoiFactory {

    void initialize(int vertexCount, int edgeCount, int faceCount);

    void reset();

    DcelVertex createVertex(double x, double y);

    HalfEdge createHalfEdge(DcelVertex origin, Face face);

    Face createFace(Vertex vertex);
}
