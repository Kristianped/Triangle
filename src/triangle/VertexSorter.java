package triangle;

import triangle.Vertex;

import java.util.Random;
import java.util.concurrent.ThreadLocalRandom;

public class VertexSorter {

    Vertex[] points;

    VertexSorter(Vertex[] points) {
        this.points = points;
    }

    public static void sort(Vertex[] array) {
        var qs = new VertexSorter(array);
        qs.quickSort(0, array.length - 1);
    }

    public static void alternateSort(Vertex[] array, int length) {
        var qs = new VertexSorter(array);
        int divider = length - 1;

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

                while (j >= left && (array[j].x > a.x || (array[j].x == a.x && array[j].y > a.y))) {
                    array[j + 1] = array[j];
                    j--;
                }
                array[j + 1] = a;
            }

            return;
        }

        // Choose a random pivot to split the array
        pivot = ThreadLocalRandom.current().nextInt(left, right);
        pivotX = array[pivot].x;
        pivotY = array[pivot].y;

        // Split the array
        left--;
        right++;

        while (left < right) {
            // Search for a vertex whose x-coordinates is too large for the left
            do {
                left++;
            } while ((left <= right) && ((array[left].x < pivotX) || ((array[left].x == pivotX) && (array[left].y < pivotY))));

            // Search for a vertex whoose x-coordinate is too small for the right
            do {
                right--;
            } while ((left <= right) && ((array[right].x > pivotX) || ((array[right].x == pivotX) && (array[right].y > pivotY))));

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
            if ((array[left].x > array[right].x) ||
                    ((array[left].x == array[right].x) &&
                            (array[left].y > array[right].y))) {
                temp = array[right];
                array[right] = array[left];
                array[left] = temp;
            }

            return;
        }

        // Choose a random pivot to split the array.
        pivot = ThreadLocalRandom.current().nextInt(left, right);
        pivot1 = array[pivot].x;
        pivot2 = array[pivot].y;

        left--;
        right++;

        while (left < right)
        {
            // Search for a vertex whose x-coordinate is too large for the left.
            do {
                left++;
            } while ((left <= right) && ((array[left].x < pivot1) || ((array[left].x == pivot1) && (array[left].y < pivot2))));

            // Search for a vertex whose x-coordinate is too small for the right.
            do {
                right--;
            } while ((left <= right) && ((array[right].x > pivot1) || ((array[right].x == pivot1) && (array[right].y > pivot2))));

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
            if ((array[left].y > array[right].y) ||
                    ((array[left].y == array[right].y) &&
                            (array[left].x > array[right].x))) {
                temp = array[right];
                array[right] = array[left];
                array[left] = temp;
            }

            return;
        }

        // Choose a random pivot to split the array.
        pivot = ThreadLocalRandom.current().nextInt(left, right);
        pivot1 = array[pivot].y;
        pivot2 = array[pivot].x;

        left--;
        right++;

        while (left < right) {
            // Search for a vertex whose x-coordinate is too large for the left.
            do {
                left++;
            } while ((left <= right) && ((array[left].y < pivot1) || ((array[left].y == pivot1) && (array[left].x < pivot2))));

            // Search for a vertex whose x-coordinate is too small for the right.
            do {
                right--;
            } while ((left <= right) && ((array[right].y > pivot1) || ((array[right].y == pivot1) && (array[right].x > pivot2))));

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
