package triangle;

public class ConstraintOptions {

    protected boolean conformingDelaunay;
    protected boolean convex;
    protected int segmentSplitting;

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
