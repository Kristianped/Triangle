package triangle;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class DcelMesh {

    protected List<DcelVertex> vertices;
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
        // Check vertices for null pointers.
        for (var vertex : vertices)
        {
            if (vertex.id < 0)
            {
                continue;
            }

            if (vertex.leaving == null)
            {
                return false;
            }

            if (vertex.leaving.origin.id != vertex.id)
            {
                return false;
            }
        }

        // Check faces for null pointers.
        for (var face : faces)
        {
            if (face.id < 0)
            {
                continue;
            }

            if (face.edge == null)
            {
                return false;
            }

            if (face.id != face.edge.face.id)
            {
                return false;
            }
        }

        // Check half-edges for null pointers.
        for (var edge : edges)
        {
            if (edge.id < 0)
            {
                continue;
            }

            if (edge.twin == null)
            {
                return false;
            }

            if (edge.origin == null)
            {
                return false;
            }

            if (edge.face == null)
            {
                return false;
            }

            if (closed && edge.next == null)
            {
                return false;
            }
        }

        // Check half-edges (topology).
        for (var edge : edges)
        {
            if (edge.id < 0)
            {
                continue;
            }

            var twin = edge.twin;
            var next = edge.next;

            if (edge.id != twin.twin.id)
            {
                return false;
            }

            if (closed)
            {
                if (next.origin.id != twin.origin.id)
                {
                    return false;
                }

                if (next.twin.next.origin.id != edge.twin.origin.id)
                {
                    return false;
                }
            }
        }

        if (closed && depth > 0)
        {
            // Check if faces are closed.
            for (var face : faces)
            {
                if (face.id < 0)
                {
                    continue;
                }

                var edge = face.edge;
                var next = edge.next;

                int id = edge.id;
                int k = 0;

                while (next.id != id && k < depth)
                {
                    next = next.next;
                    k++;
                }

                if (next.id != id)
                {
                    return false;
                }
            }
        }

        return true;
    }

    public void resolveBoundaryEdges() {
        // Maps vertices to leaving boundary edge.
        var map = new HashMap<Integer, HalfEdge>();

        // TODO: parallel?
        for (var edge : this.edges)
        {
            if (edge.twin == null)
            {
                var twin = edge.twin = new HalfEdge(edge.next.origin, Face.Empty);
                twin.twin = edge;

                map.put(twin.origin.id, twin);
            }
        }

        int j = edges.size();

        for (var edge : map.values())
        {
            edge.id = j++;
            edge.next = map.get(edge.twin.origin.id);

            this.edges.add(edge);
        }
    }

    /**
     * Enumerates all the edges of the DCEL.
     * This method assumes each half-edge has a twin
     * @return - List containing all the edges.
     */
    protected Iterable<IEdge> enumerateEdges() {
        var edges = new ArrayList<IEdge>(this.edges.size() / 2);

        for (var edge : this.edges)
        {
            var twin = edge.twin;

            // Report edge only once.
            if (edge.id < twin.id)
            {
                edges.add(new Edge(edge.origin.id, twin.origin.id));
            }
        }

        return edges;
    }

    public Iterable<IEdge> getEdges() {
        return enumerateEdges();
    }

    public List<DcelVertex> getVertices() {
        return vertices;
    }

    public List<HalfEdge> getHalfEdges() {
        return edges;
    }

    public List<Face> getFaces() {
        return faces;
    }
}
