package triangle;

import triangle.dcel.Face;

public class SimpleSmoother implements ISmoother {

    TrianglePool pool;
    Configuration config;
    ConstraintOptions options;

    public SimpleSmoother() {
        this.pool = new TrianglePool();
        this.config = new Configuration(RobustPredicates.Default(), pool.restart());
        this.options = new ConstraintOptions();
        options.setConformingDelaunay(true);
    }

    @Override
    public void smooth(IMesh mesh) {
        smooth(mesh, 10);
    }

    @Override
    public void smooth(IMesh mesh, int limit) {

    }

    private void Step(Mesh mesh, IPredicates predicates) {

    }

    private void Centroid(Face face, MutableDouble x, MutableDouble y) {
        double ai, atmp = 0, xtmp = 0, ytmp = 0;

        var edge = face.getEdge();
        var first = edge.getNext().getId();

        Point p, q;

        do {
            p = edge.getOrigin();
            q = edge.getTwin().getOrigin();

            ai = p.x * q.y - q.x * p.y;
            atmp += ai;
            xtmp += (q.x + p.x) * ai;
            ytmp += (q.y + p.y) * ai;

            edge = edge.getNext();

        } while (edge.getNext().getId() != first);

        x = new MutableDouble(xtmp / (3 * atmp));
        y = new MutableDouble(ytmp / (3 * atmp));
    }
}
