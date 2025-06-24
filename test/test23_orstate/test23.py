import unittest
from pycontract import *

# Events for testing
@data
class E1:
    pass

@data
class E2:
    pass

@data
class E3:
    pass

# A simple state that transitions to ok on E1
class S1(State):
    def transition(self, event):
        if isinstance(event, E1):
            return ok

# A simple state that transitions to S4 on E2
class S2(State):
    def transition(self, event):
        if isinstance(event, E2):
            return S4()

# A simple state that transitions to an error on E3
class S3(State):
    def transition(self, event):
        if isinstance(event, E3):
            return error("S3 failed")

# A terminal state
class S4(State):
    pass

class TestOrAndState(unittest.TestCase):
    def test1(self):
        """Test OrState where the first branch succeeds and becomes ok."""
        m = Monitor()
        m.states = {OrState(S1(), S2())}
        m.eval(E1())
        m.end()
        self.assertFalse(m.errors_found())

    def test2(self):
        """Test OrState where the second branch succeeds and moves to a new state."""
        m = Monitor()
        m.states = {OrState(S1(), S2())}
        m.eval(E2())
        m.end()
        self.assertFalse(m.errors_found())

    def test3(self):
        """Test OrState ignores a branch with an error and tries the next."""
        m = Monitor()
        m.states = {OrState(S3(), S1())}
        m.eval(E3())
        # S3 errors, so it's skipped. S1 remains. The result just S1.
        self.assertEqual(len(m.states), 1)
        self.assertIsInstance(list(m.states)[0], S1)
        self.assertFalse(m.errors_found())

    def test4(self):
        """Test OrState fails when no branch can handle the event."""
        m = Monitor()
        m.states = {OrState(S1(), S2())}
        m.eval(E3()) # Neither S1 nor S2 handle E3
        m.end()
        self.assertEqual(len(m.states), 1)
        self.assertIsInstance(list(m.states)[0], OrState)
        self.assertFalse(m.errors_found())

    def test5(self):
        """Test AndState where branches transition to new states."""
        m = Monitor()
        m.states = {AndState(S1(), S2())}
        m.eval(E1()) # S1 -> ok, S2 remains.
        self.assertEqual(len(m.states), 1)
        self.assertIsInstance(list(m.states)[0], S2)
        m.eval(E2()) # S2 -> S4
        self.assertEqual(len(m.states), 1)
        self.assertIsInstance(list(m.states)[0], S4)
        m.end()
        self.assertFalse(m.errors_found())

    def test6(self):
        """Test AndState where all branches resolve to ok."""
        class S2Ok(State):
            def transition(self, event):
                if isinstance(event, E2):
                    return ok
        m = Monitor()
        m.states = {AndState(S1(), S2Ok())}
        m.eval(E1())
        m.eval(E2())
        m.end()
        self.assertFalse(m.errors_found())

    def test7(self):
        """Test AndState fails immediately if any branch has an error."""
        m = Monitor()
        m.states = {AndState(S1(), S3())}
        m.eval(E3())
        m.end()
        self.assertTrue(m.errors_found())

    def test8(self):
        """Test that end() finds a HotState nested in an AndState."""
        class MyHotState(HotState):
            pass
        m = Monitor()
        m.states = {AndState(S1(), MyHotState())}
        m.end()
        self.assertTrue(m.errors_found())

if __name__ == '__main__':
    unittest.main()
