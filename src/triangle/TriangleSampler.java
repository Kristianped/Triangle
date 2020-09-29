package triangle;

import java.util.Iterator;
import java.util.Random;

public class TriangleSampler implements Iterable<Triangle> {

    private final int RANDOM_SEED = 110503;

    private Mesh mesh;
    private Random random;

    private int samples = 1;

    private int triangleCount = 1;

    public TriangleSampler(Mesh mesh) {
        this.mesh = mesh;
        this.random = new Random(RANDOM_SEED);
    }

    @Override
    public Iterator<Triangle> iterator() {
        return mesh.triangles.sample(samples, random).iterator();
    }

    /**
     * Resets the sampler
     */
    public void reset() {
        this.samples = 1;
        this.triangleCount = 0;
    }

    /**
     * Update sampling parameters if mesh changed
     */
    public void update() {
        int count = mesh.triangles.size();

        if (triangleCount != count) {
            triangleCount = count;

            // The number of random samples taken is proportional to the cube root
            // of the number of triangles in the mesh. The next bit of code assumes
            // that the number of triangles increases monotonically ( or at least
            // doesn't decrease enough to matter)

            while (11 * samples * samples * samples < count)
                samples++;
        }
    }
}
