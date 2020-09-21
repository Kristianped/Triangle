package triangle;

public class Configuration {

    IPredicates predicates;
    TrianglePool trianglePool;

    public Configuration() {
        this(RobustPredicates.Default(), new TrianglePool());
    }

    public Configuration(IPredicates predicates, TrianglePool trianglePool) {
        this.predicates = predicates;
        this.trianglePool = trianglePool;
    }

    public IPredicates getPredicates() {
        return predicates;
    }

    public void setPredicates(IPredicates predicates) {
        this.predicates = predicates;
    }

    public TrianglePool getTrianglePool() {
        return trianglePool;
    }

    public void setTrianglePool(TrianglePool trianglePool) {
        this.trianglePool = trianglePool;
    }
}
