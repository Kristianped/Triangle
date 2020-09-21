package triangle;

public class ConstraintOptions {

    private boolean conformingDelaunay;
    private boolean convex;
    private int segmentSplitting;

    public boolean isConformingDelaunay() {
        return conformingDelaunay;
    }

    public void setConformingDelaunay(boolean conformingDelaunay) {
        this.conformingDelaunay = conformingDelaunay;
    }

    public boolean isConvex() {
        return convex;
    }

    public void setConvex(boolean convex) {
        this.convex = convex;
    }

    public int getSegmentSplitting() {
        return segmentSplitting;
    }

    public void setSegmentSplitting(int segmentSlpitting) {
        this.segmentSplitting = segmentSlpitting;
    }
}
