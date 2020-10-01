package triangle.tools;

public class MutableBoolean {

    private boolean value;

    public MutableBoolean() {
        this(false);
    }

    public MutableBoolean(boolean value) {
        this.value = value;
    }

    public void setValue(boolean value) {
        this.value = value;
    }

    public boolean getValue() {
        return value;
    }
}
