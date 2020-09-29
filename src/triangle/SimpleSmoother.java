package triangle;

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
        var predicates = config.predicates.get();

        // The smoother should respect the mesh segment splitting behavior.
        this.options.segmentSplitting = smoothedMesh.behavior.noBisect;

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

        for (var face : voronoi.faces)
        {
            if (face.generator.label == 0)
            {
                centroid(face, x, y);

                face.generator.x = x.getValue();
                face.generator.y = y.getValue();
            }
        }
    }

    private void centroid(Face face, MutableDouble x, MutableDouble y) {
        double ai, atmp = 0, xtmp = 0, ytmp = 0;

        var edge = face.edge;
        var first = edge.next.id;

        Point p, q;

        do
        {
            p = edge.origin;
            q = edge.twin.origin;

            ai = p.x * q.y - q.x * p.y;
            atmp += ai;
            xtmp += (q.x + p.x) * ai;
            ytmp += (q.y + p.y) * ai;

            edge = edge.next;

        } while (edge.next.id != first);

        x.setValue(xtmp / (3 * atmp));
        y.setValue(ytmp / (3 * atmp));

        //area = atmp / 2;
    }

    private Polygon rebuild(Mesh mesh) {
        var data = new Polygon(mesh.vertices.size());

        for (var v : mesh.vertices.values())
        {
            // Reset to input vertex.
            v.type = Enums.VertexType.InputVertex;

            data.points.add(v);
        }

        data.segments.addAll(mesh.subsegs.values());

        data.holes.addAll(mesh.holes);
        data.regions.addAll(mesh.regions);

        return data;
    }
}
