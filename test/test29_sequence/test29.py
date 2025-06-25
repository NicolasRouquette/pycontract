from pycontract import *
import test.utest

# Events
@data
class A(Event):
    pass

@data
class B(Event):
    pass

@data
class C(Event):
    pass

class SequenceMonitor(Monitor):
    @initial
    class Start(State):
        def transition(self, event):
            match event:
                case A():
                    return SequenceMonitor.StateB() >> SequenceMonitor.StateC()

    class StateB(HotState):
        def transition(self, event):
            match event:
                case B():
                    return ok

    class StateC(HotState):
        def transition(self, event):
            match event:
                case C():
                    return ok


class TestSequence(test.utest.Test):
    def test_success(self):
        """Tests a successful sequence of events."""
        m = SequenceMonitor()
        trace = [A(), B(), C()]
        m.verify(trace)
        self.assertFalse(m.errors_found())

    def test_fail_early(self):
        """Tests a trace that fails to complete the sequence."""
        m = SequenceMonitor()
        trace = [A(), C()]
        m.verify(trace)
        self.assertTrue(m.errors_found())
        messages = m.get_all_message_texts()
        self.assertEqual(len(messages), 2)
        message_text = "".join(messages)
        self.assertIn("terminates in hot state StateB()", message_text)
        self.assertIn("terminates in hot state StateC()", message_text)

    def test_fail_incomplete(self):
        """Tests a trace that starts but does not complete the sequence."""
        m = SequenceMonitor()
        trace = [A(), B()]
        m.verify(trace)
        self.assertTrue(m.errors_found())
        messages = m.get_all_message_texts()
        self.assertEqual(len(messages), 1)
        self.assertIn("terminates in hot state StateC()", messages[0])


# Events for AndState sequence test
@data
class D(Event):
    pass


@data
class E(Event):
    pass


class AndSequenceMonitor(Monitor):
    @initial
    class Start(State):
        def transition(self, event):
            match event:
                case A():
                    # This sequence will test the AndState logic
                    return AndSequenceMonitor.Splitter() >> AndSequenceMonitor.Final()

    class Splitter(HotState):
        def transition(self, event):
            match event:
                case B():
                    # This returns two states, which will be wrapped in an AndState
                    return [AndSequenceMonitor.BranchD(), AndSequenceMonitor.BranchE()]

    class BranchD(HotState):
        def transition(self, event):
            match event:
                case D():
                    return ok

    class BranchE(HotState):
        def transition(self, event):
            match event:
                case E():
                    return ok

    class Final(HotState):
        def transition(self, event):
            match event:
                case C():
                    return ok


class TestAndStateInSequence(test.utest.Test):
    def test_and_sequence_success(self):
        """
        Tests that a sequence with an AndState proceeds only after all branches are satisfied.
        """
        m = AndSequenceMonitor()
        trace = [A(), B(), D(), E(), C()]
        m.verify(trace)
        self.assertFalse(m.errors_found(), f"Expected no errors, but got: {m.get_all_message_texts()}")

    def test_and_sequence_incomplete_one_branch(self):
        """
        Tests that the sequence remains hot if only one branch of the AndState is satisfied.
        """
        m = AndSequenceMonitor()
        trace = [A(), B(), D()]
        m.verify(trace)
        self.assertTrue(m.errors_found())
        messages = m.get_all_message_texts()
        # We expect BranchE and Final to be hot.
        self.assertEqual(len(messages), 2)
        message_text = "".join(messages)
        self.assertIn("terminates in hot state BranchE()", message_text)
        self.assertIn("terminates in hot state Final()", message_text)

    def test_and_sequence_incomplete_both_branches(self):
        """
        Tests that the sequence remains hot if neither branch of the AndState is satisfied.
        """
        m = AndSequenceMonitor()
        trace = [A(), B()]
        m.verify(trace)
        self.assertTrue(m.errors_found())
        messages = m.get_all_message_texts()
        # We expect BranchD, BranchE, and Final to be hot.
        self.assertEqual(len(messages), 3)
        message_text = "".join(messages)
        self.assertIn("terminates in hot state BranchD()", message_text)
        self.assertIn("terminates in hot state BranchE()", message_text)
        self.assertIn("terminates in hot state Final()", message_text)
