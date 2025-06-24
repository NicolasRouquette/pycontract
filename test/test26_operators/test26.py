from pycontract import *
import test.utest

# Events
@data
class A:
    pass

@data
class B:
    pass

@data
class C:
    pass

@data
class D:
    pass

class OperatorMonitor(Monitor):
    @initial
    @data
    class Start(State):
        def transition(self, event):
            match event:
                case A():
                    return OperatorMonitor.Branch1() | OperatorMonitor.Branch2()
                case B():
                    return OperatorMonitor.Branch1() & OperatorMonitor.Branch2()

    @data
    class Branch1(HotState):
        def transition(self, event):
            match event:
                case C():
                    return ok

    @data
    class Branch2(HotState):
        def transition(self, event):
            match event:
                case D():
                    return ok

class TestOperators(test.utest.Test):
    def test_or_operator(self):
        """
        Tests that the | operator behaves like an OrState.
        """
        m = OperatorMonitor()
        trace = [A(), C()]
        m.verify(trace)
        self.assertFalse(m.errors_found())

    def test_and_operator_success(self):
        """
        Tests that the & operator behaves like an AndState (success case).
        """
        m = OperatorMonitor()
        trace = [B(), C(), D()]
        m.verify(trace)
        self.assertFalse(m.errors_found())

    def test_and_operator_failure(self):
        """
        Tests that the & operator behaves like an AndState (failure case).
        """
        m = OperatorMonitor()
        trace = [B(), C()]
        m.verify(trace)
        self.assertTrue(m.errors_found())
        self.assertIn("terminates in hot state", m.get_all_message_texts()[0])
