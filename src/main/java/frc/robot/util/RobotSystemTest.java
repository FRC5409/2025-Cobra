package frc.robot.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;
import java.util.logging.Level;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * <h3>Framework to create robot system tests.</h3>
 * 
 * Such tests can be run to ensure proper funcitonality
 * of the component it is designed for.
 * 
 * @apiNote Do not instantiate nor extend this class.
 * @author logandhillon.com
 * @version v0.9.15
 */
public final class RobotSystemTest {
    private static final ShuffleboardTab    TAB     = Shuffleboard.getTab("System Tests");
    private static final List<String>       TESTS   = new ArrayList<>();
    private static final ShuffleboardLayout WIDGETS = TAB
            .getLayout("System Tests", BuiltInLayouts.kList)
            .withSize(3, 3)
            .withProperties(Map.of("Label position", "TOP"));

    private RobotSystemTest() {}

    /**
     * Test result for registered tests in {@link DebugCommand}.
     * @see {@link DebugCommand#register}
     */
    public static class TestResult {
        protected final boolean ok;
        protected final String message;

        private TestResult(boolean ok, String message) {
            this.ok = ok;
            this.message = message;
        }

        /**
         * @return Create a new test result that PASSED with no message
         */
        public static TestResult success() {
            return new TestResult(true, null);
        }

        /**
         * @param message any information to note about the test
         * @return Create a new test result that PASSED
         */
        public static TestResult success(String message) {
            return new TestResult(true, message);
        }

        /**
         * @param message reason the test failed
         * @return Create a new test result that FAILED
         */
        public static TestResult fail(String message) {
            return new TestResult(false, message);
        }
    }

    private static final void log(Level level, String format, Object... args) {
        System.out.printf("[RobotSystemTest] (" + level.getName() + ") " + format + "%n", args);
    }

    private static class SystemTestWidget {
        private final ShuffleboardLayout layout;

        public SystemTestWidget(ShuffleboardLayout parent, String id, Supplier<TestResult> runnable) {
            layout = parent.getLayout(id, BuiltInLayouts.kGrid)
                .withSize(3, 1).withProperties(Map.of(
                    "Number of columns", 3,
                    "Label position", "HIDDEN"));

            GenericEntry isOk = layout.add(id + " OK?", true)
                .withSize(1, 1)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(1, 0)
                .getEntry();

            GenericEntry msg = layout.add(id + " Output", "")
                .withSize(1, 1)
                .withPosition(2, 0)
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

            layout.add(
                "Execute " + id,
                Commands.runOnce(() -> {
                    log(Level.INFO, "Running test '%s'", id);
                    TestResult result = runnable.get();
                    isOk.setBoolean(result.ok);
                    msg.setString(String.format("%s: %s", result.ok ? "PASS" : "FAIL",
                                  Objects.requireNonNullElse(result.message, "no message")));
                })
                .withName("Execute")
                .ignoringDisable(false))
            .withSize(1, 1)
            .withPosition(0, 0)
            .withWidget("Command");
        }
    }

    /**
     * Creates a new robot test that can be run via the debug tab on shuffleboard.
     * @param name      Unique test name
     * @param runnable  Test function. Returns: {@link TestResult}
     */
    public static void register(String name, Supplier<TestResult> runnable) {
        if (name.isBlank())
            throw new IllegalArgumentException("Cannot register an unnamed test.");
        String stack = Thread.currentThread().getStackTrace()[2].getClassName();
        name = stack.substring(stack.lastIndexOf(".") + 1) + " // " + name;
        log(Level.INFO, "Registering new test '%s'", name);
        if (TESTS.contains(name))
            throw new IllegalArgumentException(String.format(
                "RobotSystemTest with name %s already exists! (#%s)", name, TESTS.indexOf(name)));
        new SystemTestWidget(WIDGETS, name, runnable);
        TESTS.add(name);
    }
}
