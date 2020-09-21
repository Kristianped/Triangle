package triangle;

/**
 * A priority queue for bad triangles<br><br>
 * Note: The queue is actually a set of 4096 queues.
 */
public class BadTriQueue {

    final double SQRT2 = 1.4142135623730950488016887242096980785696718753769480732;

    // Variables that maintain the bad triangle queues.  The queues are
    // ordered from 4095 (highest priority) to 0 (lowest priority).
    BadTriangle[] queueFront;
    BadTriangle[] queueTail;
    int[] nextnonemptyq;
    int firstnonemptyq;
    int count;

    public BadTriQueue() {
        queueFront = new BadTriangle[4096];
        queueTail = new BadTriangle[4096];
        nextnonemptyq = new int[4096];
        firstnonemptyq = -1;
        count = 0;
    }

    public int getCount() {
        return count;
    }

    /**
     * Add a bad triangle data structure to the end of the queue
     * @param badTriangle The bad triangle to enqueue
     */
    public void enqueue(BadTriangle badTriangle) {
        double length, multiplier;
        int exponent, expincrement;
        int queuenumber;
        int posexponent;
        int i;

        this.count++;

        // Determine the appropriate queue to put the bad triangle into.
        // Recall that the key is the square of its shortest edge length.
        if (badTriangle.key >= 1.0) {
            length = badTriangle.key;
            posexponent = 1;
        } else {
            // 'badtri.key' is 2.0 to a negative exponent, so we'll record that
            // fact and use the reciprocal of 'badtri.key', which is > 1.0.
            length = 1.0 / badTriangle.key;
            posexponent = 0;
        }

        // 'length' is approximately 2.0 to what exponent? The following code
        // determines the answer in time logarithmic in the exponent
        exponent = 0;

        while (length > 2) {
            // Find an approximation by repeated squaring of two
            expincrement = 1;
            multiplier = 0.5;

            while (length * multiplier * multiplier > 1.0) {
                expincrement *= 2;
                multiplier *= multiplier;
            }

            // Reduce the value of 'length', then iterate if needed
            exponent += expincrement;
            length *= multiplier;
        }

        // 'length'is approx squareroot(2.0) to what exponent?
        exponent = 2 * exponent + (length > SQRT2 ? 1 : 0);

        // 'exponent is now in the range 0...2047 for IEEE double precision.
        // Choose a queue in the range 0...4095. The shortest edges have the
        // highest priority

        if (posexponent > 0)
            queuenumber = 2047 - exponent;
        else
            queuenumber = 2048 + exponent;

        // Are we inserting into an empty queue?
        if (queueFront[queuenumber] == null) {
            // Yes we are inserting into an empty queue.
            // Will this become the highest-priority queue?
            if (queuenumber > firstnonemptyq) {
                // Yes, this is the highest-priority queue
                nextnonemptyq[queuenumber] = firstnonemptyq;
                firstnonemptyq = queuenumber;
            } else {
                // No, this is not the highest-priority queue
                // Find the queue with the next higher priority
                i = queuenumber + 1;

                while (queueFront[i] == null)
                    i++;

                // Mark the newly nonempty queue as following a higher-priority queue
                nextnonemptyq[queuenumber] = nextnonemptyq[i];
                nextnonemptyq[i] = queuenumber;
            }

            // Put the bad triangle at the beginning of the (empty) queue
            queueFront[queuenumber] = badTriangle;
        } else {
            // Add the bad triangle to the end of an already nonempty queue
            queueTail[queuenumber].next = badTriangle;
        }

        // Maintain a pointer to the last triangle of the queue
        queueTail[queuenumber] = badTriangle;
        // Newly enqueued bad triangle has no successor in the queue
        badTriangle.next = null;
    }

    /**
     * Add a bad triangle to the end of the queue
     */
    public void enqueue(Otri enqtri, double minedge, Vertex apex, Vertex org, Vertex dest) {
        // Allocate space for the bad triangle
        BadTriangle newbad = new BadTriangle();
        newbad.poortri = enqtri;
        newbad.key = minedge;
        newbad.apex = apex;
        newbad.org = org;
        newbad.dest = dest;

        enqueue(newbad);
    }

    /**
     * Remove a triangle from the front of the queue
     */
    public BadTriangle dequeue() {
        // if no queues are nonempty, return null
        if (firstnonemptyq < 0)
            return null;

        this.count--;

        // Find the first triangle of the highest priority queue
        BadTriangle result = queueFront[firstnonemptyq];
        // Remove the triangle from the queue
        queueFront[firstnonemptyq] = result.next;
        // If this queue is not empty, note the highest-priority
        // nonempty queue
        if (result == queueTail[firstnonemptyq])
            firstnonemptyq = nextnonemptyq[firstnonemptyq];

        return result;
    }
}
