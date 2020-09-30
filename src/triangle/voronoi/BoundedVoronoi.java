package triangle.voronoi;

import triangle.*;
import triangle.dcel.DcelVertex;
import triangle.dcel.HalfEdge;

import java.util.ArrayList;

public class BoundedVoronoi extends VoronoiBase {

    int offset;

    public BoundedVoronoi(Mesh mesh) {
        this(mesh, new DefaultVoronoiFactory(), RobustPredicates.Default());
    }

    public BoundedVoronoi(Mesh mesh, IVoronoiFactory factory, IPredicates predicates) {
        super(mesh, factory, predicates, true);

        // We explicitly told the base constructor to call the Generate method, so
        // at this point the basic Voronoi diagram is already created.
        offset = this.vertices.size();

        // Each vertex of the hull will be part of a Voronoi cell.
        ((ArrayList<DcelVertex>) this.vertices).ensureCapacity(offset + mesh.getHullsize());

        // Create bounded Voronoi diagram.
        postProcess();

        resolveBoundaryEdges();
    }

    private void postProcess() {
        for (var edge : rays)
        {
            var twin = edge.getTwin();

            var v1 = (Vertex) edge.getFace().getGenerator();
            var v2 = (Vertex) twin.getFace().getGenerator();

            double dir = predicates.counterClockwise(v1, v2, edge.getOrigin());

            if (dir <= 0)
            {
                handleCase1(edge, v1, v2);
            }
            else
            {
                handleCase2(edge, v1, v2);
            }
        }
    }

    private void handleCase1(HalfEdge edge, Vertex v1, Vertex v2) {
        //int mark = GetBoundaryMark(v1);

        // The infinite vertex.
        var v = (Point) edge.getTwin().getOrigin();

        // The half-edge is the bisector of v1 and v2, so the projection onto the
        // boundary segment is actually its midpoint.
        v.setX((v1.getX() + v2.getX()) / 2.0);
        v.setY((v1.getY() + v2.getY()) / 2.0);

        // Close the cell connected to edge.
        var gen = factory.createVertex(v1.getX(), v1.getY());

        var h1 = factory.createHalfEdge(edge.getTwin().getOrigin(), edge.getFace());
        var h2 = factory.createHalfEdge(gen, edge.getFace());

        edge.setNext(h1);
        h1.setNext(h2);
        h2.setNext(edge.getFace().getEdge());

        gen.setLeaving(h2);

        // Let the face edge point to the edge leaving at generator.
        edge.getFace().setEdge(h2);

        this.edges.add(h1);
        this.edges.add(h2);

        int count = this.edges.size();

        h1.setId(count);
        h2.setId(count + 1);

        gen.setId(offset++);
        this.vertices.add(gen);
    }

    private void handleCase2(HalfEdge edge, Vertex v1, Vertex v2) {
        // The vertices of the infinite edge.
        var p1 = (Point)edge.getOrigin();
        var p2 = (Point)edge.getTwin().getOrigin();

        // The two edges leaving p1, pointing into the mesh.
        var e1 = edge.getTwin().getNext();
        var e2 = e1.getTwin().getNext();

        // Find the two intersections with boundary edge.
        IntersectionHelper.intersectSegments(v1, v2, e1.getOrigin(), e1.getTwin().getOrigin(), p2);
        IntersectionHelper.intersectSegments(v1, v2, e2.getOrigin(), e2.getTwin().getOrigin(), p1);

        // The infinite edge will now lie on the boundary. Update pointers:
        e1.getTwin().setNext(edge.getTwin());
        edge.getTwin().setNext(e2);
        edge.getTwin().setFace(e2.getFace());

        e1.setOrigin(edge.getTwin().getOrigin());

        edge.getTwin().setTwin(null);
        edge.setTwin(null);

        // Close the cell.
        var gen = factory.createVertex(v1.getX(), v1.getY());
        var he = factory.createHalfEdge(gen, edge.getFace());

        edge.setNext(he);
        he.setNext(edge.getFace().getEdge());

        // Let the face edge point to the edge leaving at generator.
        edge.getFace().setEdge(he);

        this.edges.add(he);

        he.setId(this.edges.size());

        gen.setId(offset++);
        this.vertices.add(gen);
    }
}
