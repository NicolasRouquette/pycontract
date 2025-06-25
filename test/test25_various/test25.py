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

# set_debug(True)

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


class TestVarious(test.utest.Test):
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
        self.assertIn("[HOT STATE] *** error at end in OrMonitor:\n    terminates in hot state Expect_B_NotD_C(1)", message_set)
        self.assertIn("[HOT STATE] *** error at end in OrMonitor:\n    terminates in hot state Expect_NotF_E(1)", message_set)


# Events for NotState test
@data
class G:
    pass

@data
class H:
    pass

@data
class I:
    pass


class NotMonitor(Monitor):
    """
    A monitor to test NotState logic.
    """


    @initial
    @data
    class Start(State):
        def transition(self, event):
            match event:
                case G():
                    return NotState(NotMonitor.Inner()) 

    @data
    class Inner(HotState):
        def transition(self, event):
            match event:
                case H():
                    return ok
                case I():
                    return error("Inner state failed as expected")

    @data
    class AndBranch(State):
        def transition(self, event):
            match event:
                case G():
                    return [NotMonitor.Branch1(), NotMonitor.Branch2()]

    @data
    class Branch1(State):
        def transition(self, event):
            match event:
                case H():
                    return ok
                case I():
                    return error("Branch1 failed")

    @data
    class Branch2(State):
        def transition(self, event):
            match event:
                case H():
                    return ok


class TestNotState(test.utest.Test):
    def test1(self):
        """
        Tests that NotState returns ok when its inner state returns an error.
        """
        m = NotMonitor()
        trace = [G(), I()]
        m.verify(trace)
        self.assertFalse(m.errors_found())

    def test2(self):
        """
        Tests that NotState returns an error when its inner state returns ok.
        """
        m = NotMonitor()
        trace = [G(), H()]
        m.verify(trace)
        self.assertTrue(m.errors_found())
        self.assertIn("NotState: inner state succeeded where it should have failed", m.get_all_message_texts()[0])

    def test3(self):
        """
        Tests that NotState correctly handles an implicit AndState.
        """
        # Scenario 1: One branch fails, so NotState should succeed.
        m1 = NotMonitor()
        m1.states = {NotState(NotMonitor.AndBranch())}
        trace1 = [G(), I()]
        m1.verify(trace1)
        self.assertFalse(m1.errors_found(), "Scenario 1 failed: NotState should succeed when one branch fails.")

        # Scenario 2: Both branches succeed, so NotState should fail.
        m2 = NotMonitor()
        m2.states = {NotState(NotMonitor.AndBranch())}
        trace2 = [G(), H()]
        m2.verify(trace2)
        self.assertTrue(m2.errors_found(), "Scenario 2 failed: NotState should fail when all branches succeed.")
        self.assertIn("NotState: inner state succeeded where it should have failed", m2.get_all_message_texts()[0])




