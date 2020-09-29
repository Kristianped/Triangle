package triangle;

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
            var twin = edge.twin;

            var v1 = (Vertex) edge.face.generator;
            var v2 = (Vertex) twin.face.generator;

            double dir = predicates.counterClockwise(v1, v2, edge.origin);

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
        var v = (Point) edge.twin.origin;

        // The half-edge is the bisector of v1 and v2, so the projection onto the
        // boundary segment is actually its midpoint.
        v.x = (v1.x + v2.x) / 2.0;
        v.y = (v1.y + v2.y) / 2.0;

        // Close the cell connected to edge.
        var gen = factory.createVertex(v1.x, v1.y);

        var h1 = factory.createHalfEdge(edge.twin.origin, edge.face);
        var h2 = factory.createHalfEdge(gen, edge.face);

        edge.next = h1;
        h1.next = h2;
        h2.next = edge.face.edge;

        gen.leaving = h2;

        // Let the face edge point to the edge leaving at generator.
        edge.face.edge = h2;

        this.edges.add(h1);
        this.edges.add(h2);

        int count = this.edges.size();

        h1.id = count;
        h2.id = count + 1;

        gen.id = offset++;
        this.vertices.add(gen);
    }

    private void handleCase2(HalfEdge edge, Vertex v1, Vertex v2) {
        // The vertices of the infinite edge.
        var p1 = (Point)edge.origin;
        var p2 = (Point)edge.twin.origin;

        // The two edges leaving p1, pointing into the mesh.
        var e1 = edge.twin.next;
        var e2 = e1.twin.next;

        // Find the two intersections with boundary edge.
        IntersectionHelper.intersectSegments(v1, v2, e1.origin, e1.twin.origin, p2);
        IntersectionHelper.intersectSegments(v1, v2, e2.origin, e2.twin.origin, p1);

        // The infinite edge will now lie on the boundary. Update pointers:
        e1.twin.next = edge.twin;
        edge.twin.next = e2;
        edge.twin.face = e2.face;

        e1.origin = edge.twin.origin;

        edge.twin.twin = null;
        edge.twin = null;

        // Close the cell.
        var gen = factory.createVertex(v1.x, v1.y);
        var he = factory.createHalfEdge(gen, edge.face);

        edge.next = he;
        he.next = edge.face.edge;

        // Let the face edge point to the edge leaving at generator.
        edge.face.edge = he;

        this.edges.add(he);

        he.id = this.edges.size();

        gen.id = offset++;
        this.vertices.add(gen);
    }
}
