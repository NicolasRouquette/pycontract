from pycontract import *
import test.utest

# Events
@data
class Success(Event):
    pass

class OkMessageMonitor(Monitor):
    @initial
    class Start(State):
        def transition(self, event):
            match event:
                case Success():
                    return self.ok_message("Operation was successful")

class TestOkMessage(test.utest.Test):
    def test_ok_message_reporting(self):
        """Tests that ok_message() correctly reports a success message."""
        m = OkMessageMonitor()
        trace = [Success()]
        m.verify(trace)
        self.assertFalse(m.errors_found())
        messages = m.get_all_message_texts()
        self.assertEqual(len(messages), 1)
        # The message includes ANSI color codes, so we check for the core text.
        self.assertIn("--- ok: Operation was successful", messages[0])
