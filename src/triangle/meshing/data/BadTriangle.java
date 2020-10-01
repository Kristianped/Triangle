package triangle.meshing.data;

import triangle.Otri;
import triangle.geometry.Vertex;

/**
 * A queue used to store bad triangles<br><br>
 * Note: The key is the square of the cosine of the smallest angle of the triangle.
 * Each triangle's vertices are stored so that one can check whether a triangle
 * is still the same
 */
public class BadTriangle {

    Otri poortri;            // A skinny or too-large triangle
    double key;              // cos^2 of smallest (atypical) angle
    Vertex org, dest, apex;  // Its three vertices
    BadTriangle next;        // Pointer to next bad triangle

    public BadTriangle() {
        poortri = new Otri();
    }

    @Override
    public String toString() {
        return String.format("B-TID %d", poortri.tri.hashCode());
    }

    public Otri getPoortri() {
        return poortri;
    }

    public Vertex getOrg() {
        return org;
    }

    public void setOrg(Vertex org) {
        this.org = org;
    }

    public Vertex getDest() {
        return dest;
    }

    public void setDest(Vertex dest) {
        this.dest = dest;
    }

    public Vertex getApex() {
        return apex;
    }

    public void setApex(Vertex apex) {
        this.apex = apex;
    }
}
