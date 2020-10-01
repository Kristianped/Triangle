package triangle.tools;

public class MutableDouble {

    private double value;

    public MutableDouble() {
        this(0);
    }

    public MutableDouble(double value) {
        this.value = value;
    }

    public void add(double value) {
        this.value += value;
    }

    public double getValue() {
        return this.value;
    }

    public void setValue(double value) {
        this.value = value;
    }
}
