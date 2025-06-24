from pycontract import *
import test.utest

"""
Integration test for OrStates within a Monitor.
"""

# Events
@data
class A:
    x: int


@data
class B:
    x: int


@data
class C:
    x: int


@data
class D:
    x: int


@data
class E:
    x: int


@data
class F:
    x: int


class OrMonitor(Monitor):
    """
    A monitor to test OrState logic.
    After A(x), it expects one of two sequences:
    1. B(x) then C(x), but not D(x) in between.
    2. D(x) then E(x), but not F(x) in between.
    """

    def transition(self, event):
        match event:
            case A(x):
                return OrState(
                    OrMonitor.Expect_B_NotD_C(x),
                    OrMonitor.Expect_D_NotF_E(x)
                )

    @data
    class Expect_B_NotD_C(HotState):
        x: int

        def transition(self, event):
            match event:
                case B(self.x):
                    return OrMonitor.Expect_NotD_C(self.x)

    @data
    class Expect_NotD_C(HotState):
        x: int

        def transition(self, event):
            match event:
                case D(self.x):
                    return error("D is not allowed in this branch")
                case C(self.x):
                    return ok

    @data
    class Expect_D_NotF_E(HotState):
        x: int

        def transition(self, event):
            match event:
                case D(self.x):
                    return OrMonitor.Expect_NotF_E(self.x)

    @data
    class Expect_NotF_E(HotState):
        x: int

        def transition(self, event):
            match event:
                case F(self.x):
                    return error("F is not allowed in this branch")
                case E(self.x):
                    return ok


class TestOrStateIntegration(test.utest.Test):
    def test1(self):
        """
        Tests that the monitor correctly reports hot state errors at the end.
        """
        m = OrMonitor()
        trace = [A(1), D(1), E(2)]
        m.verify(trace)

        self.assertTrue(m.errors_found())
        messages = m.get_all_message_texts()
        self.assertEqual(len(messages), 2)
        message_set = set(messages)
        self.assertIn("*** error at end in OrMonitor:\n    terminates in hot state Expect_B_NotD_C(1)", message_set)
        self.assertIn("*** error at end in OrMonitor:\n    terminates in hot state Expect_NotF_E(1)", message_set)

    def test2(self):
        """
        Tests that the monitor correctly reports an error during a transition.
        Note that altough the F event causes an error transition it is not recorded as such.
        Whether this should be like this or there should be some otherway of reporting the error transition is
        to be decided.
        """
        m = OrMonitor()
        trace = [A(1), D(1), F(1)]
        m.verify(trace)
        self.assertTrue(m.errors_found())
        messages = m.get_all_message_texts()
        self.assertEqual(len(messages), 1)
        self.assertIn("terminates in hot state", messages[0])
        self.assertIn("Expect_B_NotD_C", messages[0])

    def test3(self):
        """
        Tests a successful trace that satisfies the first branch of the OrState.
        """
        m = OrMonitor()
        trace = [A(1), B(1), C(1)]
        m.verify(trace)
        self.assertFalse(m.errors_found())

    def test4(self):
        """
        Tests a successful trace that satisfies the second branch of the OrState.
        """
        m = OrMonitor()
        trace = [A(1), D(1), E(1)]
        m.verify(trace)
        self.assertFalse(m.errors_found())

    def test5(self):
        """
        Tests a trace where both branches of the OrState progress initially,
        and one is later satisfied.
        """
        m = OrMonitor()
        # After A(1), state is OrState(Expect_B_NotD_C(1), Expect_D_NotF_E(1))
        # After B(1), state is OrState(Expect_NotD_C(1), Expect_D_NotF_E(1))
        # After D(1), first branch errors and is dropped. State becomes Expect_NotF_E(1).
        # After E(1), state becomes ok.
        trace = [A(1), B(1), D(1), E(1)]
        m.verify(trace)
        self.assertFalse(m.errors_found())

if __name__ == '__main__':
    set_debug(True) 
    m = OrMonitor()
    trace = [A(1), D(1), E(2)]
    m.verify(trace)
