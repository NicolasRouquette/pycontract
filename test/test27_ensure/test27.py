from pycontract import *
import test.utest

# Events
@data
class Check(Event):
    x: int

class EnsureMonitor(Monitor):
    @initial
    class Start(State):
        def transition(self, event):
            match event:
                case Check(x):
                    return self.ensure(x > 10, f"x is not > 10: {x}")

class TestEnsure(test.utest.Test):
    def test_ensure_success(self):
        """Tests that ensure() succeeds when the condition is true."""
        m = EnsureMonitor()
        trace = [Check(20)]
        m.verify(trace)
        self.assertFalse(m.errors_found(), f"Expected no errors, but got: {m.get_all_message_texts()}")

    def test_ensure_fail(self):
        """Tests that ensure() fails when the condition is false."""
        m = EnsureMonitor()
        trace = [Check(5)]
        m.verify(trace)
        self.assertTrue(m.errors_found())
        messages = m.get_all_message_texts()
        self.assertEqual(len(messages), 1)
        self.assertIn("x is not > 10: 5", messages[0])
