from pycontract import *
import test.utest

# Events
@data
class CommandEvent:
    op: str
    cmd: str
    nr: str

class Commands(Monitor):
    @initial
    @data
    class Start(State):
        def transition(self, event):
            match event:
                case CommandEvent('DISPATCH', 'TURN', nr):
                    return Commands.DoCompleteTurn(nr)

    @data
    class DoCompleteTurn(HotState):
        nr: str

        @exhaustive
        def transition(self, event):
            match event:
                case CommandEvent('EVR1', 'TURN', self.nr):
                    return done()
                case CommandEvent('EVR2', 'TURN', self.nr):
                    return done()
                case CommandEvent('EVR3', 'TURN', self.nr):
                    return done()
                case CommandEvent('COMPLETE', 'TURN', self.nr):
                    return error('TURN Completes too early')

class TestExhaustive(test.utest.Test):
    def test_exhaustive_success(self):
        """
        Tests that a state with an @exhaustive transition succeeds when all obligations are met.
        """
        m = Commands()
        trace = [
            CommandEvent('DISPATCH', 'TURN', '1'),
            CommandEvent('EVR1', 'TURN', '1'),
            CommandEvent('EVR3', 'TURN', '1'),
            CommandEvent('EVR2', 'TURN', '1'),
        ]
        m.verify(trace)
        self.assertFalse(m.errors_found())

    def test_exhaustive_failure(self):
        """
        Tests that a state with an @exhaustive transition fails if not all obligations are met.
        """
        m = Commands()
        trace = [
            CommandEvent('DISPATCH', 'TURN', '1'),
            CommandEvent('EVR1', 'TURN', '1'),
            CommandEvent('EVR2', 'TURN', '1'),
        ]
        m.verify(trace)
        self.assertTrue(m.errors_found())
        self.assertIn("Cases not matched that lead to calls of done()", m.get_all_message_texts()[0])
