package triangle.voronoi;

import triangle.*;
import triangle.tools.IntersectionHelper;

public class StandardVoronoi extends VoronoiBase {

    public StandardVoronoi(Mesh mesh) {
        this(mesh, mesh.getBounds());
    }

    public StandardVoronoi(Mesh mesh, Rectangle box) {
        this(mesh, box, new DefaultVoronoiFactory(), RobustPredicates.Default());
    }

    public StandardVoronoi(Mesh mesh, Rectangle box, IVoronoiFactory factory, IPredicates predicates) {
        super(mesh, factory, predicates, true);

        // We assume the box to be at least as large as the mesh.
        box.expand(mesh.getBounds());

        // We explicitly told the base constructor to call the Generate method, so
        // at this point the basic Voronoi diagram is already created.
        postProcess(box);
    }

    private void postProcess(Rectangle box) {
        for (var edge : rays) {
            // The vertices of the infinite edge.
            var v1 = (Point)edge.getOrigin();
            var v2 = (Point)edge.getTwin().getOrigin();

            if (box.contains(v1) || box.contains(v2)) {
                // Move infinite vertex v2 onto the box boundary.
                IntersectionHelper.boxRayIntersection(box, v1, v2, v2);
            } else {
                // There is actually no easy way to handle the second case. The two edges
                // leaving v1, pointing towards the mesh, don't have to intersect the box
                // (the could join with edges of other cells outside the box).

                // A general intersection algorithm (DCEL <-> Rectangle) is needed, which
                // computes intersections with all edges and discards objects outside the
                // box.
            }
        }
    }
}
