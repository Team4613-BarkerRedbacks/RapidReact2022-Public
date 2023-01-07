package redbacks.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class TestAssignmentOnCrash {
    @Test
    void testVariableValueOnCrash() {
        int i = 1;

        try {
            i = crash();
        }
        catch(Exception e) {
            assertEquals(1, i);
        }
    }

    int crash() throws Exception {
        throw new Exception();
    }
}
