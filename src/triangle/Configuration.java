package triangle;

import java.util.function.Function;
import java.util.function.Supplier;

public class Configuration {

    private final static int dummy = 1;

    Supplier<IPredicates> predicates;
    Supplier<TrianglePool> trianglePool;

    public Configuration() {
        this(() -> RobustPredicates.Default(), () -> new TrianglePool());
    }

    public Configuration(Supplier<IPredicates> predicates, Supplier<TrianglePool> trianglePool) {
        this.predicates = predicates;
        this.trianglePool = trianglePool;
    }

    public Supplier<IPredicates>  getPredicates() {
        return predicates;
    }

    public void setPredicates(Supplier<IPredicates> predicates) {
        this.predicates = predicates;
    }

    public Supplier<TrianglePool> getTrianglePool() {
        return trianglePool;
    }

    public void setTrianglePool(Supplier<TrianglePool> trianglePool) {
        this.trianglePool = trianglePool;
    }
}
