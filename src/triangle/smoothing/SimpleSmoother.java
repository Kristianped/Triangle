package triangle.smoothing;

import triangle.*;
import triangle.dcel.Face;
import triangle.meshing.ConstraintOptions;
import triangle.meshing.GenericMesher;
import triangle.meshing.IMesh;
import triangle.voronoi.BoundedVoronoi;
import triangle.voronoi.IVoronoiFactory;

public class SimpleSmoother implements ISmoother {

    TrianglePool pool;
    Configuration config;
    IVoronoiFactory factory;
    ConstraintOptions options;

    public SimpleSmoother() {
        this(new VoronoiFactory());
    }

    public SimpleSmoother(IVoronoiFactory factory) {
        this.pool = new TrianglePool();
        this.config = new Configuration(() -> RobustPredicates.Default(), () -> pool.restart());
        this.factory = factory;
        this.options = new ConstraintOptions();
        options.setConformingDelaunay(true);
    }

    @Override
    public void smooth(IMesh mesh) {
        smooth(mesh, 10);
    }

    @Override
    public void smooth(IMesh mesh, int limit) {
        var smoothedMesh = (Mesh)mesh;

        var mesher = new GenericMesher(config);
        var predicates = config.getPredicates().get();

        // The smoother should respect the mesh segment splitting behavior.
        this.options.setSegmentSplitting(smoothedMesh.getBehavior().getNoBisect());

        // Take a few smoothing rounds (Lloyd's algorithm).
        for (int i = 0; i < limit; i++)
        {
            step(smoothedMesh, factory, predicates);

            // Actually, we only want to rebuild, if the mesh is no longer
            // Delaunay. Flipping edges could be the right choice instead
            // of re-triangulating...
            smoothedMesh = (Mesh)mesher.triangulate(rebuild(smoothedMesh), options);

            factory.reset();
        }

        smoothedMesh.copyTo((Mesh)mesh);
    }

    private void step(Mesh mesh, IVoronoiFactory factory, IPredicates predicates) {
        var voronoi = new BoundedVoronoi(mesh, factory, predicates);

        MutableDouble x = new MutableDouble();
        MutableDouble y = new MutableDouble();

        for (var face : voronoi.getFaces())
        {
            if (face.getGenerator().getLabel() == 0)
            {
                centroid(face, x, y);

                face.getGenerator().setX(x.getValue());
                face.getGenerator().setY(y.getValue());
            }
        }
    }

    private void centroid(Face face, MutableDouble x, MutableDouble y) {
        double ai, atmp = 0, xtmp = 0, ytmp = 0;

        var edge = face.getEdge();
        var first = edge.getNext().getId();

        Point p, q;

        do
        {
            p = edge.getOrigin();
            q = edge.getTwin().getOrigin();

            ai = p.getX() * q.getY() - q.getX() * p.getY();
            atmp += ai;
            xtmp += (q.getX() + p.getX()) * ai;
            ytmp += (q.getY() + p.getY()) * ai;

            edge = edge.getNext();

        } while (edge.getNext().getId() != first);

        x.setValue(xtmp / (3 * atmp));
        y.setValue(ytmp / (3 * atmp));

        //area = atmp / 2;
    }

    private Polygon rebuild(Mesh mesh) {
        var data = new Polygon(mesh.getVertices().size());

        for (var v : mesh.getVertices())
        {
            // Reset to input vertex.
            v.setType(Enums.VertexType.InputVertex);
            data.getPoints().add(v);
        }

        data.getSegments().addAll(mesh.getSegments());

        data.getHoles().addAll(mesh.getHoles());
        data.getRegions().addAll(mesh.getRegions());

        return data;
    }
}
