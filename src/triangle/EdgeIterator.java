package triangle;

import java.util.Iterator;

public class EdgeIterator implements Iterator<Edge> {

    Iterator<Triangle> triangles;
    Otri tri = new Otri();
    Otri neighbor = new Otri();
    Osub sub = new Osub();
    Edge current;
    Vertex p1, p2;

    public EdgeIterator(Mesh mesh) {
        mesh.triangles.iterator();
        triangles.hasNext();

        tri.tri = triangles.next();
        tri.orient = 0;
    }

    @Override
    public boolean hasNext() {
        if (tri.tri == null)
            return false;

        current = null;

        while (current == null) {
            if (tri.orient == 3) {
                if (triangles.hasNext()) {
                    tri.tri = triangles.next();
                    tri.orient = 0;
                } else {
                    // Finally no more triangles
                    return false;
                }
            }

            tri.sym(neighbor);

            if (tri.tri.id < neighbor.tri.id || neighbor.tri.id == Mesh.DUMMY) {
                p1 = tri.org();
                p2 = tri.dest();

                tri.pivot(sub);

                // Boundary mark of dummysub is 0, so we don't need to worry about that
                current = new Edge(p1.id, p2.id, sub.seg.boundary);
            }

            tri.orient = tri.orient + 1;
        }

        return true;
    }

    @Override
    public Edge next() {
        return current;
    }
}
