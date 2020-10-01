package triangle.voronoi;

import triangle.*;
import triangle.dcel.DcelMesh;
import triangle.dcel.DcelVertex;
import triangle.dcel.Face;
import triangle.dcel.HalfEdge;
import triangle.geometry.Edge;
import triangle.geometry.IEdge;
import triangle.geometry.Point;
import triangle.geometry.Vertex;
import triangle.tools.MutableDouble;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public abstract class VoronoiBase extends DcelMesh {

    protected IPredicates predicates;

    protected IVoronoiFactory factory;

    // List of infinite half-edges, i.e. half-edges that start at circumcenters of triangles
    // which lie on the domain boundary.
    protected List<HalfEdge> rays;

    protected VoronoiBase(Mesh mesh, IVoronoiFactory factory, IPredicates predicates, boolean generate) {
        super(false);

        this.factory = factory;
        this.predicates = predicates;

        if (generate)
            generate(mesh);
    }

    protected void generate(Mesh mesh) {
        mesh.renumber();

        this.edges = new ArrayList<HalfEdge>();
        this.rays = new ArrayList<HalfEdge>();

        // Allocate space for Voronoi diagram.
        var vertices = new DcelVertex[mesh.getTriangles().size() + mesh.getHullsize()];
        var faces = new Face[mesh.getVertices().size()];

        if (factory == null)
        {
            factory = new DefaultVoronoiFactory();
        }

        factory.initialize(vertices.length, 2 * mesh.getNumberOfEdges(), faces.length);

        // Compute triangles circumcenters.
        var map = computeVertices(mesh, vertices);

        // Create all Voronoi faces.
        for (var vertex : mesh.getVertices())
        {
            faces[vertex.getId()] = factory.createFace(vertex);
        }

        computeEdges(mesh, vertices, faces, map);

        // At this point all edges are computed, but the (edge.next) pointers aren't set.
        connectEdges(map);

        this.vertices = new ArrayList<>(Arrays.asList(vertices));

        this.faces = new ArrayList<>(Arrays.asList(faces));
    }

    protected List<HalfEdge>[] computeVertices(Mesh mesh, DcelVertex[] vertices) {
        Otri tri = new Otri();
        MutableDouble xi = new MutableDouble(), eta = new MutableDouble();
        DcelVertex vertex;
        Point pt;
        int id;

        // Maps all vertices to a list of leaving edges.
        List<HalfEdge>[] map = new List[mesh.getTriangles().size()];

        // Compue triangle circumcenters
        for (var t : mesh.getTriangles())
        {
            id = t.getID();
            tri.tri = t;

            pt = predicates.findCircumcenter(tri.org(), tri.dest(), tri.apex(), xi, eta);

            vertex = factory.createVertex(pt.getX(), pt.getY());
            vertex.setId(id);

            vertices[id] = vertex;
            map[id] = new ArrayList<HalfEdge>();
        }

        return map;
    }

    protected void computeEdges(Mesh mesh, DcelVertex[] vertices, Face[] faces, List<HalfEdge>[] map) {
        Otri tri = new Otri(), neighbor = new Otri();
        Vertex org, dest;

        double px, py;
        int id, nid, count = mesh.getTriangles().size();

        Face face, neighborFace;
        HalfEdge edge, twin;
        DcelVertex vertex, end;

        // Count infinte edges (vertex id for their endpoints).
        int j = 0;

        // Count half-edges (edge ids).
        int k = 0;

        // To loop over the set of edges, loop over all triangles, and look at the
        // three edges of each triangle.  If there isn't another triangle adjacent
        // to the edge, operate on the edge. If there is another adjacent triangle,
        // operate on the edge only if the current triangle has a smaller id than
        // its neighbor. This way, each edge is considered only once.
        for (var t : mesh.getTriangles())
        {
            id = t.getID();
            tri.tri = t;

            for (int i = 0; i < 3; i++)
            {
                tri.orient = i;
                tri.sym(neighbor);

                nid = neighbor.tri.getID();

                if (id < nid || nid < 0)
                {
                    // Get the endpoints of the current triangle edge.
                    org = tri.org();
                    dest = tri.dest();

                    face = faces[org.getId()];
                    neighborFace = faces[dest.getId()];

                    vertex = vertices[id];

                    // For each edge in the triangle mesh, there's a corresponding edge
                    // in the Voronoi diagram, i.e. two half-edges will be created.
                    if (nid < 0)
                    {
                        // Unbounded edge, direction perpendicular to the boundary edge,
                        // pointing outwards.
                        px = dest.getY() - org.getY();
                        py = org.getX() - dest.getX();

                        end = factory.createVertex(vertex.getX() + px, vertex.getY() + py);
                        end.setId(count + j++);

                        vertices[end.getId()] = end;

                        edge = factory.createHalfEdge(end, face);
                        twin = factory.createHalfEdge(vertex, neighborFace);

                        // Make (face.edge) always point to an edge that starts at an infinite
                        // vertex. This will allow traversing of unbounded faces.
                        face.setEdge(edge);
                        face.setBounded(false);

                        map[id].add(twin);

                        rays.add(twin);
                    }
                    else
                    {
                        end = vertices[nid];

                        // Create half-edges.
                        edge = factory.createHalfEdge(end, face);
                        twin = factory.createHalfEdge(vertex, neighborFace);

                        // Add to vertex map.
                        map[nid].add(edge);
                        map[id].add(twin);
                    }

                    vertex.setLeaving(twin);
                    end.setLeaving(edge);

                    edge.setTwin(twin);
                    twin.setTwin(edge);

                    edge.setId(k++);
                    twin.setId(k++);

                    this.edges.add(edge);
                    this.edges.add(twin);
                }
            }
        }
    }

    protected void connectEdges(List<HalfEdge>[] map) {
        int length = map.length;

        // For each half-edge, find its successor in the connected face.
        for (var edge : this.edges)
        {
            var face = edge.getFace().getGenerator().getId();

            // The id of the dest vertex of current edge.
            int id = edge.getTwin().getOrigin().getId();

            // The edge origin can also be an infinite vertex. Sort them out
            // by checking the id.
            if (id < length)
            {
                // Look for the edge that is connected to the current face. Each
                // Voronoi vertex has degree 3, so this loop is actually O(1).
                for (var next : map[id])
                {
                    if (next.getFace().getGenerator().getId() == face)
                    {
                        edge.setNext(next);
                        break;
                    }
                }
            }
        }
    }

    @Override
    protected Iterable<IEdge> enumerateEdges() {
        var edges = new ArrayList<IEdge>(this.edges.size() / 2);

        for (var edge : this.edges)
        {
            var twin = edge.getTwin();

            // Report edge only once.
            if (twin == null)
            {
                edges.add(new Edge(edge.getOrigin().getId(), edge.getNext().getOrigin().getId()));
            }
            else if (edge.getId() < twin.getId())
            {
                edges.add(new Edge(edge.getOrigin().getId(), twin.getOrigin().getId()));
            }
        }

        return edges;
    }
}
