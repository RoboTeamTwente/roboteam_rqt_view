import threading

class StoppableThread(threading.Thread):
    """
    # Source: http://stackoverflow.com/questions/323972/is-there-any-way-to-kill-a-thread-in-python
    # Philippe F's answer.

    Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition.
    """

    def __init__(self, group=None, target=None, name=None, args=(), kwargs={}):
        super(StoppableThread, self).__init__(group, target, name, args, kwargs)
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()
