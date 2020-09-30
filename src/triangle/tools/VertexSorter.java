package triangle.tools;

import triangle.Vertex;

import java.util.Random;

public class VertexSorter {

    private static final int RANDOM_SEED = 57113;

    Random rand;
    Vertex[] points;

    VertexSorter(Vertex[] points, int seed) {
        this.points = points;
        this.rand = new Random(seed);
    }

    public static void sort(Vertex[] array) {
        sort(array, RANDOM_SEED);
    }

    public static void sort(Vertex[] array, int seed) {
        var qs = new VertexSorter(array, seed);
        qs.quickSort(0, array.length - 1);
    }

    public static void alternateSort(Vertex[] array, int length) {
        alternateSort(array, length, RANDOM_SEED);
    }

    public static void alternateSort(Vertex[] array, int length, int seed) {
        var qs = new VertexSorter(array, seed);
        int divider = length >> 1;

        // Re-sort the array of vertices to accommodate alternating cuts.
        if (length - divider >= 2) {
            if (divider > 2)
                qs.alternateAxes(0, divider - 1, 1);

            qs.alternateAxes(divider, length - 1, 1);
        }
    }

    /**
     * Sort an array of vertices by x-coordinate, using the y-coordinate as a secondary key.
     * <br><br>
     * Uses quicksort. Randomized O(n log n) time. No, I did not make any of
     * the usual quicksort mistakes.
     * @param left Left
     * @param right Right
     */
    private void quickSort(int left, int right) {
        int oringialLeft = left;
        int originalRight = right;
        int arraySize = right - left + 1;
        int pivot;
        double pivotX;
        double pivotY;
        Vertex temp;

        var array = this.points;

        if (arraySize < 32) {
            // Insertion sort
            for (int i = left + 1; i <= right; i++) {
                var a = array[i];
                int j = i - 1;

                while (j >= left && (array[j].getX() > a.getX() || (array[j].getX() == a.getX() && array[j].getY() > a.getY()))) {
                    array[j + 1] = array[j];
                    j--;
                }
                array[j + 1] = a;
            }

            return;
        }

        // Choose a random pivot to split the array
        pivot = rand.nextInt(right - left) + left;
        pivotX = array[pivot].getX();
        pivotY = array[pivot].getY();

        // Split the array
        left--;
        right++;

        while (left < right) {
            // Search for a vertex whose x-coordinates is too large for the left
            do {
                left++;
            } while ((left <= right) && ((array[left].getX() < pivotX) || ((array[left].getX() == pivotX) && (array[left].getY() < pivotY))));

            // Search for a vertex whoose x-coordinate is too small for the right
            do {
                right--;
            } while ((left <= right) && ((array[right].getX() > pivotX) || ((array[right].getX() == pivotX) && (array[right].getY() > pivotY))));

            if (left < right) {
                // Swap the left and right vertices
                temp = array[left];
                array[left] = array[right];
                array[right] = temp;
            }
        }

        if (left > oringialLeft)
            quickSort(oringialLeft, left);  // recursively sort the left subset

        if (originalRight > right + 1)
            quickSort(right + 1, originalRight);    // recursively sort the right subset
    }

    /**
     * Sorts the vertices as appropriate for the divide-and-conquer algorithm with
     * alternating cuts.
     * <br><br>
     * Partitions by x-coordinate if axis == 0; by y-coordinate if axis == 1.
     * For the base case, subsets containing only two or three vertices are
     * always sorted by x-coordinate.
     */
    private void alternateAxes(int left, int right, int axis) {
        int size = right - left + 1;
        int divider = size >> 1;

        if (size <= 3) {
            // Recursive base case:  subsets of two or three vertices will be
            // handled specially, and should always be sorted by x-coordinate.
            axis = 0;
        }

        // Partition with a horizontal or vertical cut.
        if (axis == 0)
            vertexMedianX(left, right, left + divider);
        else
            vertexMedianY(left, right, left + divider);

        // Recursively partition the subsets with a cross cut.
        if (size - divider >= 2) {
            if (divider >= 2)
                alternateAxes(left, left + divider - 1, 1 - axis);

            alternateAxes(left + divider, right, 1 - axis);
        }
    }

    /**
     * An order statistic algorithm, almost. Shuffles an array of vertices so that the
     * first 'median' vertices occur lexicographically before the remaining vertices.
     * <br><br>
     * Uses the x-coordinate as the primary key. Very similar to the QuickSort()
     * procedure, but runs in randomized linear time.
     */
    private void vertexMedianX(int left, int right, int median) {
        int arraysize = right - left + 1;
        int oleft = left, oright = right;
        int pivot;
        double pivot1, pivot2;
        Vertex temp;

        var array = this.points;

        if (arraysize == 2)
        {
            // Recursive base case.
            if ((array[left].getX() > array[right].getX()) ||
                    ((array[left].getX() == array[right].getX()) &&
                            (array[left].getY() > array[right].getY()))) {
                temp = array[right];
                array[right] = array[left];
                array[left] = temp;
            }

            return;
        }

        // Choose a random pivot to split the array.
        pivot = rand.nextInt(right - left) + left;
        pivot1 = array[pivot].getX();
        pivot2 = array[pivot].getY();

        left--;
        right++;

        while (left < right)
        {
            // Search for a vertex whose x-coordinate is too large for the left.
            do {
                left++;
            } while ((left <= right) && ((array[left].getX() < pivot1) || ((array[left].getX() == pivot1) && (array[left].getY() < pivot2))));

            // Search for a vertex whose x-coordinate is too small for the right.
            do {
                right--;
            } while ((left <= right) && ((array[right].getX() > pivot1) || ((array[right].getX() == pivot1) && (array[right].getY() > pivot2))));

            if (left < right) {
                // Swap the left and right vertices.
                temp = array[left];
                array[left] = array[right];
                array[right] = temp;
            }
        }

        // Unlike in vertexsort(), at most one of the following conditionals is true.
        if (left > median) {
            // Recursively shuffle the left subset.
            vertexMedianX(oleft, left - 1, median);
        }

        if (right < median - 1) {
            // Recursively shuffle the right subset.
            vertexMedianX(right + 1, oright, median);
        }
    }

    /**
     * An order statistic algorithm, almost.  Shuffles an array of vertices so that
     * the first 'median' vertices occur lexicographically before the remaining vertices.
     * <br><br>
     * Uses the y-coordinate as the primary key. Very similar to the QuickSort()
     * procedure, but runs in randomized linear time.
     */
    private void vertexMedianY(int left, int right, int median) {
        int arraysize = right - left + 1;
        int oleft = left, oright = right;
        int pivot;
        double pivot1, pivot2;
        Vertex temp;

        var array = this.points;

        if (arraysize == 2) {
            // Recursive base case.
            if ((array[left].getY() > array[right].getY()) ||
                    ((array[left].getY() == array[right].getY()) &&
                            (array[left].getX() > array[right].getX()))) {
                temp = array[right];
                array[right] = array[left];
                array[left] = temp;
            }

            return;
        }

        // Choose a random pivot to split the array.
        pivot = rand.nextInt(right - left) + left;
        pivot1 = array[pivot].getY();
        pivot2 = array[pivot].getX();

        left--;
        right++;

        while (left < right) {
            // Search for a vertex whose x-coordinate is too large for the left.
            do {
                left++;
            } while ((left <= right) && ((array[left].getY() < pivot1) || ((array[left].getY() == pivot1) && (array[left].getX() < pivot2))));

            // Search for a vertex whose x-coordinate is too small for the right.
            do {
                right--;
            } while ((left <= right) && ((array[right].getY() > pivot1) || ((array[right].getY() == pivot1) && (array[right].getX() > pivot2))));

            if (left < right) {
                // Swap the left and right vertices.
                temp = array[left];
                array[left] = array[right];
                array[right] = temp;
            }
        }

        // Unlike in QuickSort(), at most one of the following conditionals is true.
        if (left > median) {
            // Recursively shuffle the left subset.
            vertexMedianY(oleft, left - 1, median);
        }

        if (right < median - 1) {
            // Recursively shuffle the right subset.
            vertexMedianY(right + 1, oright, median);
        }
    }
}
