package triangle.topology.dcel;

import triangle.Edge;
import triangle.IEdge;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class DcelMesh {

    protected List<Vertex> vertices;
    protected List<HalfEdge> edges;
    protected List<Face> faces;

    public DcelMesh() {
        this(true);
    }

    public DcelMesh(boolean initialise) {
        if (initialise) {
            vertices = new ArrayList<>();
            edges = new ArrayList<>();
            faces = new ArrayList<>();
        }
    }

    public boolean isConsistent() {
        return isConsistent(true, 0);
    }

    public boolean isConsistent(boolean closed, int depth) {
        for (var vertex : vertices) {
            if (vertex.getId() < 0)
                continue;

            if (vertex.leaving == null)
                return false;

            if (vertex.leaving.origin.getId() != vertex.getId())
                return false;
        }

        for (var face : faces) {
            if (face.id < 0)
                continue;

            if (face.edge == null)
                return false;

            if (face.id != face.edge.face.id)
                return false;
        }

        for (var edge : edges) {
            if (edge.id < 0)
                continue;

            if (edge.twin == null)
                return false;

            if (edge.origin == null)
                return false;

            if (edge.face == null)
                return false;

            if (closed && edge.next == null)
                return false;
        }

        for (var edge : edges) {
            if (edge.id < 0)
                continue;

            var twin = edge.twin;
            var next = edge.next;

            if (edge.id != twin.twin.id)
                return false;

            if (closed) {
                if (next.origin.getId() != twin.origin.getId())
                    return false;

                if (next.twin.next.origin.getId() != edge.twin.origin.getId())
                    return false;
            }
        }

        if (closed && depth > 0) {
            for (var face : faces) {
                if (face.id < 0)
                    continue;

                var edge = face.edge;
                var next = edge.next;

                int id = edge.id;
                int k = 0;

                while (next.id != id && k < depth) {
                    next = next.next;
                    k++;
                }

                if (next.id != id)
                    return false;
            }
        }

        return true;
    }

    public void resolveBoundaryEdges() {
        Map<Integer, HalfEdge> map = new HashMap<>();

        for (var edge : edges) {
            if (edge.twin == null) {
                var twin = edge.twin = new HalfEdge(edge.next.origin, Face.Empty);
                twin.twin = edge;
                map.put(twin.origin.getId(), twin);
            }
        }

        int j = edges.size();

        for (var edge : map.values()) {
            edge.id = j++;
            edge.next = map.get(edge.twin.origin.getId());
            edges.add(edge);
        }
    }

    /**
     * Enumerates all the edges of the DCEL.
     * This method assumes each half-edge has a twin
     * @return - List containing all the edges.
     */
    protected Iterable<IEdge> enumerateEdges() {
        List<IEdge> edges = new ArrayList<>(this.edges.size() / 2);

        for (var edge : this.edges) {
            var twin = edge.twin;

            if (edge.id < twin.id)
                edges.add(new Edge(edge.origin.getId(), twin.origin.getId()));
        }

        return edges;
    }

    public Iterable<IEdge> getEdges() {
        return enumerateEdges();
    }

    public List<Vertex> getVertices() {
        return vertices;
    }

    public List<HalfEdge> getHalfEdges() {
        return edges;
    }

    public List<Face> getFaces() {
        return faces;
    }
}
