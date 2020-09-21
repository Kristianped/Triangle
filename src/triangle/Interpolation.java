package triangle;

public class Interpolation {

    /**
     * Linear interpolation of vertex attributes.
     * @param vertex The interpolation vertex
     * @param triangle The triangle containg the vertex
     * @param n The number of vertex attributes
     */
    public static void interpolateAttributes(Vertex vertex, ITriangle triangle, int n) {
        Vertex org = triangle.getVertex(0);
        Vertex dest = triangle.getVertex(1);
        Vertex apex = triangle.getVertex(2);

        double xdo, ydo, xao, yao;
        double denominator;
        double dx, dy;
        double xi, eta;

        // Compute the circumcenter of the triangle.
        xdo = dest.x - org.x;
        ydo = dest.y - org.y;
        xao = apex.x - org.x;
        yao = apex.y - org.y;

        denominator = 0.5 / (xdo * yao - xao * ydo);

        //dx = (yao * dodist - ydo * aodist) * denominator;
        //dy = (xdo * aodist - xao * dodist) * denominator;

        dx = vertex.x - org.x;
        dy = vertex.y - org.y;

        // To interpolate vertex attributes for the new vertex, define a
        // coordinate system with a xi-axis directed from the triangle's
        // origin to its destination, and an eta-axis, directed from its
        // origin to its apex.
        xi = (yao * dx - xao * dy) * (2.0 * denominator);
        eta = (xdo * dy - ydo * dx) * (2.0 * denominator);

        for (int i = 0; i < n; i++) {
            // Interpolate the vertex attributes.
            vertex.attributes[i] = org.attributes[i]
                    + xi * (dest.attributes[i] - org.attributes[i])
                    + eta * (apex.attributes[i] - org.attributes[i]);
        }
    }
}
