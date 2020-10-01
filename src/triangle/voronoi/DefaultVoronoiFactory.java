package triangle.voronoi;

import triangle.geometry.Vertex;
import triangle.dcel.DcelVertex;
import triangle.dcel.Face;
import triangle.dcel.HalfEdge;

public class DefaultVoronoiFactory implements IVoronoiFactory {
    @Override
    public void initialize(int vertexCount, int edgeCount, int faceCount) {

    }

    @Override
    public void reset() {

    }

    @Override
    public DcelVertex createVertex(double x, double y) {
        return new DcelVertex(x, y);
    }

    @Override
    public HalfEdge createHalfEdge(DcelVertex origin, Face face) {
        return new HalfEdge(origin, face);
    }

    @Override
    public Face createFace(Vertex vertex) {
        return new Face(vertex);
    }
}
