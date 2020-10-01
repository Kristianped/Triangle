package triangle.meshing.iterators;

import triangle.*;

import java.util.Iterator;

public class EdgeIterator implements Iterator<Edge> {

    Iterator<Triangle> triangles;
    Otri tri = new Otri();
    Otri neighbor = new Otri();
    Osub sub = new Osub();
    Edge current;
    Vertex p1, p2;

    public EdgeIterator(Mesh mesh) {
        triangles = mesh.getTrianglePool().iterator();
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

            if (tri.tri.getID() < neighbor.tri.getID() || neighbor.tri.getID() == Mesh.DUMMY) {
                p1 = tri.org();
                p2 = tri.dest();

                sub = tri.pivot();

                // Boundary mark of dummysub is 0, so we don't need to worry about that
                current = new Edge(p1.getId(), p2.getId(), sub.seg.getLabel());
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
