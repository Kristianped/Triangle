package triangle;

import java.util.*;
import java.util.concurrent.ThreadLocalRandom;

public class TrianglePool implements Collection<Triangle> {

    // Determines the size of each block in the pool
    private final static int BLOCKSIZE = 1024;

    // The total number of currently allocated triangles
    int size;

    // The number of triangles currently used
    int count;

    // The pool
    Triangle[][] pool;

    // A stack of free triangles
    Deque<Triangle> stack;

    public TrianglePool() {
        size = 0;

        // On startup, the pool should be able to hold 2^16 triangles
        int n = Math.max(1, 65536 / BLOCKSIZE);

        pool = new Triangle[n][];
        pool[0] = new Triangle[BLOCKSIZE];

        stack = new ArrayDeque<>(BLOCKSIZE);
    }

    /**
     * Gets a triangle from the pool
     */
    public Triangle get() {
        Triangle triangle;

        if (stack.size() > 0) {
            triangle = stack.pop();
            triangle.hash = -triangle.hash - 1;

            cleanup(triangle);
        } else if (count < size) {
            triangle = pool[count / BLOCKSIZE][count % BLOCKSIZE];
            triangle.id = triangle.hash;

            cleanup(triangle);

            count++;
        } else {
            triangle = new Triangle();
            triangle.hash = size;
            triangle.id = triangle.hash;

            int block = size / BLOCKSIZE;

            if (pool[block] == null) {
                pool[block] = new Triangle[BLOCKSIZE];

                if (block + 1 == pool.length)
                    pool = Arrays.copyOf(pool, 2 * pool.length);
            }

            pool[block][size % BLOCKSIZE] = triangle;
            count = ++size;
        }

        return triangle;
    }

    public void release(Triangle triangle) {
        stack.push(triangle);

        // Mark triangle as free
        triangle.hash = -triangle.hash - 1;
    }

    /**
     * Restart the triangle pool
     */
    public TrianglePool restart() {
        for (Triangle t : stack)
            t.hash = -t.hash - 1;

        stack.clear();
        count = 0;

        return this;
    }

    /**
     * Samples a number of triangles from the pool
     * @param k - The number of triangles to sample
     */
    Iterable<Triangle> sample(int k, Random random) {
        int i;
        int count = this.size();

        if (k > count)
            k = count;

        List<Triangle> list = new ArrayList<>();
        Triangle t;

        while (k > 0) {
            i = random.nextInt(count);
            t = pool[i / BLOCKSIZE][i % BLOCKSIZE];

            if (t.hash >= 0) {
                k--;
                list.add(t);
            }
        }

        return list;
    }

    private void cleanup(Triangle triangle) {
        triangle.label = 0;
        triangle.area = 0.0;
        triangle.infected = false;

        triangle.reset();
    }

    public void copyTo(Triangle[] array, int index) {
        var enumerator = iterator();

        while (enumerator.hasNext()) {
            array[index] = enumerator.next();
            index++;
        }
    }

    @Override
    public int size() {
        return count - stack.size();
    }

    @Override
    public boolean isEmpty() {
        return count == 0;
    }

    @Override
    public boolean contains(Object o) {
        if (!(o instanceof  Triangle))
            return false;

        Triangle item = (Triangle) o;
        int i = item.hash;

        if (i < 0 || i > size)
            return false;

        return pool[i / BLOCKSIZE][i % BLOCKSIZE].hash >= 0;
    }

    @Override
    public Iterator<Triangle> iterator() {
        return new Enumerator(this);
    }

    @Override
    public Object[] toArray() {
        Object[] array = new Object[size()];
        int i = 0;
        Iterator<Triangle> iterator = iterator();

        while (iterator.hasNext())
            array[i++] = iterator.next();

        return array;
    }

    @Override
    public boolean add(Triangle o) {
        throw new UnsupportedOperationException("Not implemented");
    }

    @Override
    public boolean remove(Object o) {
        throw new UnsupportedOperationException("Not implemented");
    }

    @Override
    public boolean addAll(Collection c) {
        throw new UnsupportedOperationException("Not implemented");
    }

    @Override
    public void clear() {
        stack.clear();

        int blocks = (size / BLOCKSIZE) + 1;

        for (int i = 0; i < blocks; i++) {
            var block = pool[i];

            // Number of triangles in current block
            int length = (size - i * BLOCKSIZE) % BLOCKSIZE;

            for (int j = 0; j < length; j++)
                block[j] = null;
        }

        size = count = 0;
    }

    @Override
    public boolean retainAll(Collection c) {
        throw new UnsupportedOperationException("Not implemented");
    }

    @Override
    public boolean removeAll(Collection c) {
        throw new UnsupportedOperationException("Not implemented");
    }

    @Override
    public boolean containsAll(Collection c) {
        for (Object o : c)
            if (!contains(o))
                return false;

        return true;
    }

    @Override
    public Object[] toArray(Object[] a) {
        throw new UnsupportedOperationException("Not implemented");
    }

    class Enumerator implements Iterator<Triangle> {

        int count;
        Triangle[][] pool;
        Triangle current;
        int index, offset;

        public Enumerator(TrianglePool pool) {
            this.count = pool.size();
            this.pool = pool.pool;
            index = 0;
            offset = 0;
        }

        @Override
        public boolean hasNext() {
            while (index < count) {
                current = pool[offset / BLOCKSIZE][offset % BLOCKSIZE];

                offset++;

                if (current.hashCode() >= 0) {
                    index++;
                    return true;
                }
            }

            return false;
        }

        public void reset() {
            index = 0;
            offset = 0;
        }

        @Override
        public Triangle next() {
            return current;
        }
    }
}
